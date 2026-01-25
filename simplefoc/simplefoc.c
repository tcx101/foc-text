/**
 * @file simplefoc.c
 * @brief 标准FOC(Field Oriented Control)算法实现
 * @version 2.0
 * @date 2024
 * 严格按照FOC理论实现：
 * 1. Clarke变换(abc->αβ)
 * 2. Park变换(αβ->dq) 
 * 3. PID电流控制(d轴id=0, q轴iq=转矩)
 * 4. 反Park变换(dq->αβ)
 * 5. SVPWM(αβ->abc)
 */
#include "simplefoc.h"
#include <math.h>
#include "arm_math.h"
#include <string.h>
#include <stdio.h>
#include "tim.h"
#include "as5600.h"
#include "adc_measure.h"

extern AS5600_t as5600_l; 
extern AS5600_t as5600_r; 


// ===== 内部函数声明 =====
static void FOC_CurrentControl(FOC_Motor_t *motor, float target_iq);

// ===== 板级默认HAL静态实现（从main.c迁移） =====
// 依赖外部句柄与变量	extern AS5600_t as5600_l;

/**
 * @brief 【HAL-电机1】获取母线电压
 * @return 母线电压值(V)，默认返回12V
 * @note 用于电机1的母线电压读取，可根据实际硬件修改
 */
static float default_board_get_bus_voltage(void)
{
    return 12.0f;
}

/**
 * @brief 【HAL-电机1】读取编码器角度
 * @return 机械角度(弧度)
 * @note 从AS5600磁编码器读取电机1的转子位置
 */
static float default_board_read_angle(void)
{
    return AS5600_GetAngleRad(&as5600_l);
}

/**
 * @brief 【HAL-电机1】读取三相电流
 * @param ia 输出A相电流(A)
 * @param ib 输出B相电流(A)
 * @param ic 输出C相电流(A)，通过基尔霍夫定律计算: ic = -(ia + ib)
 * @note 从ADC读取电机1的相电流，只需采样两相，第三相通过计算得出
 */
static void default_board_read_currents(float *ia, float *ib, float *ic)
{
    if (!ia || !ib || !ic) return;
    *ia = ADC_Get_Phase_Current_A();
    *ib = ADC_Get_Phase_Current_B();
    *ic = -(*ia + *ib);  // 基尔霍夫电流定律
}

/**
 * @brief 【HAL-电机1】设置三相电压输出
 * @param va A相电压(V)
 * @param vb B相电压(V)
 * @param vc C相电压(V)
 * @note 将电压值转换为PWM占空比，通过TIM2输出到电机1的三相桥臂
 *       占空比计算: duty = 0.5 + (v / vdc)，范围限制在[0, 1]
 */
static void default_board_set_voltages(float va, float vb, float vc)
{
    float vdc = default_board_get_bus_voltage();
    if (vdc < 6.0f) vdc = 12.0f;

    // 电压转占空比：中点对齐，0V对应50%占空比
    float duty_a = 0.5f + (va / vdc);
    float duty_b = 0.5f + (vb / vdc);
    float duty_c = 0.5f + (vc / vdc);

    // 占空比限幅
    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f;
    if (duty_c > 1.0f) duty_c = 1.0f;

    // 设置PWM比较值
    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(duty_a * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(duty_b * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(duty_c * pwm_period));
}

/**
 * @brief 【HAL-电机1】获取电机角速度
 * @return 角速度(rad/s)
 * @note 从AS5600编码器读取电机1的转速
 */
static float default_board_get_velocity(void)
{
    return AS5600_GetVelRad(&as5600_l);
}

// ==== Motor2 HAL (Right motor: AS5600_r + TIM4 PD12-14) ====

/**
 * @brief 【HAL-电机2】获取母线电压
 * @return 母线电压值(V)，默认返回12V
 * @note 用于电机2（右电机）的母线电压读取
 */
static float motor2_get_bus_voltage(void)
{
    return 12.0f;
}

/**
 * @brief 【HAL-电机2】读取编码器角度
 * @return 机械角度(弧度)
 * @note 从AS5600磁编码器读取电机2的转子位置
 */
static float motor2_read_angle(void)
{
    return AS5600_GetAngleRad(&as5600_r);
}

/**
 * @brief 【HAL-电机2】读取三相电流
 * @param ia 输出A相电流(A)
 * @param ib 输出B相电流(A)
 * @param ic 输出C相电流(A)
 * @note 从ADC读取电机2的相电流，注意：这里交换了ia和ib，用于相序校正
 */
static void motor2_read_currents(float *ia, float *ib, float *ic)
{
    if (!ia || !ib || !ic) return;
    float ia_raw = ADC_Get_Phase_Current_A_Motor2();
    float ib_raw = ADC_Get_Phase_Current_B_Motor2();
    *ia = ib_raw;  // 相序交换
    *ib = ia_raw;  // 相序交换
    *ic = -(*ia + *ib);
}

/**
 * @brief 【HAL-电机2】设置三相电压输出
 * @param va A相电压(V)
 * @param vb B相电压(V)
 * @param vc C相电压(V)
 * @note 将电压值转换为PWM占空比，通过TIM4输出到电机2的三相桥臂
 */
static void motor2_set_voltages(float va, float vb, float vc)
{
    float vdc = motor2_get_bus_voltage();
    if (vdc < 6.0f) vdc = 12.0f;

    float duty_a = 0.5f + (va / vdc);
    float duty_b = 0.5f + (vb / vdc);
    float duty_c = 0.5f + (vc / vdc);

    if (duty_a < 0.0f) duty_a = 0.0f;
    if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f;
    if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f;
    if (duty_c > 1.0f) duty_c = 1.0f;

    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)(duty_a * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)(duty_b * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)(duty_c * pwm_period));
}

/**
 * @brief 【HAL-电机2】获取电机角速度
 * @return 角速度(rad/s)
 * @note 从AS5600编码器读取电机2的转速
 */
static float motor2_get_velocity(void)
{
    return AS5600_GetVelRad(&as5600_r);
}

// ===== 数学工具函数 =====

/**
 * @brief 【数学工具】角度标准化到[-π, π]区间
 * @param angle 输入角度(弧度)
 * @return 标准化后的角度(弧度)，范围[-π, π]
 * @note 用于角度计算时避免累积误差，确保角度在合理范围内
 */
float FOC_NormalizeAngle(float angle)
{
    while (angle > FOC_PI) angle -= FOC_2PI;
    while (angle < -FOC_PI) angle += FOC_2PI;
    return angle;
}

/**
 * @brief 【FOC变换】Clarke变换 - 三相坐标系(abc) -> 两相静止坐标系(αβ)
 * @param ia A相电流(A)
 * @param ib B相电流(A)
 * @param ic C相电流(A)，实际未使用，因为可由ia和ib计算得出
 * @param alpha 输出α轴电流(A)
 * @param beta 输出β轴电流(A)
 * @note FOC第一步：将三相电流转换为两相静止坐标系，简化计算
 *       使用ARM CMSIS-DSP库加速计算
 */
void FOC_Clarke(float ia, float ib, float ic, float *alpha, float *beta)
{
    (void)ic; // CMSIS-DSP Clarke 仅需 Ia/Ib；ic 由 Ia/Ib 决定
    arm_clarke_f32(ia, ib, alpha, beta);
}

/**
 * @brief 【FOC变换】Park变换 - 静止坐标系(αβ) -> 旋转坐标系(dq)
 * @param alpha α轴分量
 * @param beta β轴分量
 * @param cos_theta 电角度的余弦值
 * @param sin_theta 电角度的正弦值
 * @param d 输出d轴分量（磁场方向）
 * @param q 输出q轴分量（转矩方向）
 * @note FOC第二步：将静止坐标系转换为随转子旋转的dq坐标系
 *       在dq坐标系中，电流变为直流量，便于PID控制
 */
void FOC_Park(float alpha, float beta, float cos_theta, float sin_theta, float *d, float *q)
{
    // CMSIS-DSP: arm_park_f32(Ialpha, Ibeta, pId, pIq, sinVal, cosVal)
    arm_park_f32(alpha, beta, d, q, sin_theta, cos_theta);
}

/**
 * @brief 【FOC变换】反Park变换 - 旋转坐标系(dq) -> 静止坐标系(αβ)
 * @param d d轴分量（磁场方向）
 * @param q q轴分量（转矩方向）
 * @param cos_theta 电角度的余弦值
 * @param sin_theta 电角度的正弦值
 * @param alpha 输出α轴分量
 * @param beta 输出β轴分量
 * @note FOC第四步：将PID控制后的dq电压转换回静止坐标系
 *       为SVPWM做准备
 */
void FOC_InvPark(float d, float q, float cos_theta, float sin_theta, float *alpha, float *beta)
{
    // CMSIS-DSP: arm_inv_park_f32(Id, Iq, pIalpha, pIbeta, sinVal, cosVal)
    arm_inv_park_f32(d, q, alpha, beta, sin_theta, cos_theta);
}

/**
 * @brief 【FOC变换】空间矢量脉宽调制(SVPWM) - αβ坐标系 -> 三相电压
 * @param alpha α轴电压(V)
 * @param beta β轴电压(V)
 * @param vdc 母线电压(V)
 * @param va 输出A相电压(V)
 * @param vb 输出B相电压(V)
 * @param vc 输出C相电压(V)
 * @note FOC第五步：将αβ电压转换为三相PWM输出
 *       采用中性点偏移法，提高直流母线利用率
 *       限制电压在线性调制区内(√3/3 ≈ 0.5773倍母线电压)
 */
void FOC_SVPWM(float alpha, float beta, float vdc, float *va, float *vb, float *vc)
{
    // 计算三相参考电压
    float va_ref = alpha;
    float vb_ref = -0.5f * alpha + FOC_SQRT3_2 * beta;
    float vc_ref = -0.5f * alpha - FOC_SQRT3_2 * beta;
    
    // 找到最大值和最小值，计算中性点偏移
    float v_max = fmaxf(fmaxf(va_ref, vb_ref), vc_ref);
    float v_min = fminf(fminf(va_ref, vb_ref), vc_ref);
    float v_offset = -0.5f * (v_max + v_min);
    
    // 应用中性点偏移并转换为实际电压
    *va = va_ref + v_offset;
    *vb = vb_ref + v_offset;
    *vc = vc_ref + v_offset;
    
    // 限制电压幅值在线性调制区内
    float v_limit = vdc * 0.5773f; // √3/3 ≈ 0.5773
    float v_magnitude = fmaxf(fabsf(*va), fmaxf(fabsf(*vb), fabsf(*vc)));
    if (v_magnitude > v_limit) {
        float scale = v_limit / v_magnitude;
        *va *= scale;
        *vb *= scale;
        *vc *= scale;
    }
}

// ===== PID控制器 =====

/**
 * @brief 【PID控制】初始化PID控制器
 * @param pid PID控制器指针
 * @param mode PID模式 (DELTA_PID增量式/POSITION_PID位置式)
 * @param p 比例系数Kp
 * @param i 积分系数Ki
 * @param d 微分系数Kd
 * @param output_limit 输出限幅值
 * @note 增量式PID：输出增量，适合执行器有积分特性的场合
 *       位置式PID：输出绝对值，适合一般控制场合
 */
void FOC_PID_Init(FOC_PID_t *pid, uint32_t mode, float p, float i, float d, float output_limit)
{
    memset(pid, 0, sizeof(FOC_PID_t));
    
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->output_limit = output_limit;
    // 积分限幅默认设置为输出限幅的100%（允许积分项充分累积）
    pid->integral_limit = output_limit * 1.0f;
    // 积分分离阈值默认设置为很大的值（实际禁用积分分离，让积分项始终工作）
    pid->integral_separation_threshold = 1000.0f;
}

/**
 * @brief 【PID控制】PID控制器更新计算（带积分抗饱和）
 * @param pid PID控制器指针
 * @param target 目标值
 * @param now 当前值
 * @return PID输出值（已限幅）
 * @note 增量式PID公式：Δu = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2*e[k-1]+e[k-2])
 *       位置式PID公式：u = Kp*e[k] + Ki*Σe[k] + Kd*(e[k]-e[k-1])
 *       新增功能：
 *       1. 积分分离：误差过大时不累积积分
 *       2. 积分限幅：防止积分项无限累积
 *       3. 反向积分：输出饱和时停止积分累积
 */
float FOC_PID_Update(FOC_PID_t *pid, float target, float now)
{
    // 更新目标值和当前值
    pid->target = target;
    pid->now = now;
    
    // 计算当前偏差
    pid->error[0] = pid->target - pid->now;

    // 计算输出
    if (pid->pid_mode == DELTA_PID) // 增量式
    {
        // 计算增量
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
        
        // 累加输出
        pid->out += pid->pout + pid->iout + pid->dout;
        
        // 【输出限幅】：防止输出无限累积
        if (pid->out > pid->output_limit) {
            pid->out = pid->output_limit;
        }
        if (pid->out < -pid->output_limit) {
            pid->out = -pid->output_limit;
        }
    }
    else if (pid->pid_mode == POSITION_PID) // 位置式
    {
        // 比例项
        pid->pout = pid->p * pid->error[0];
        
        // 【积分分离】：只有误差在阈值内才累积积分
        float error_abs = fabsf(pid->error[0]);
        if (error_abs < pid->integral_separation_threshold) {
            pid->iout += pid->i * pid->error[0];
            
            // 【积分限幅】：防止积分项无限累积
            if (pid->iout > pid->integral_limit) {
                pid->iout = pid->integral_limit;
            }
            if (pid->iout < -pid->integral_limit) {
                pid->iout = -pid->integral_limit;
            }
        }
        
        // 微分项
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        
        // 计算总输出
        pid->out = pid->pout + pid->iout + pid->dout;
        
        // 【反向积分（抗饱和）】：如果输出饱和，回退积分项
        if (pid->out > pid->output_limit) {
            // 输出超过上限，回退积分项
            float excess = pid->out - pid->output_limit;
            pid->iout -= excess;  // 减去超出部分
            pid->out = pid->output_limit;
        } else if (pid->out < -pid->output_limit) {
            // 输出超过下限，回退积分项
            float excess = pid->out - (-pid->output_limit);
            pid->iout -= excess;  // 减去超出部分
            pid->out = -pid->output_limit;
        }
    }

    // 记录前两次偏差
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    
    // 返回PID输出
    return pid->out;
}



// ===== FOC核心函数 =====

/**
 * @brief 【FOC初始化】初始化FOC电机控制器
 * @param motor 电机控制结构体指针
 * @param pole_pairs 电机极对数（磁极对数，例如14极电机极对数为7）
 * @return true-初始化成功，false-初始化失败
 * @note 设置电机基本参数、PID控制器、物理参数等
 *       必须在使用FOC控制前调用此函数
 */
bool FOC_Init(FOC_Motor_t *motor, uint8_t pole_pairs)
{
        
    // 基本参数
    motor->pole_pairs = pole_pairs;
    motor->initialized = true;
    motor->sensor_direction = 1;
    
    // 控制限制
    motor->voltage_limit = 12.0f;   // 电压限制(V)
    motor->current_limit = 2.0f;    // 电流限制(A)
    
    // 控制模式
    motor->mode = FOC_MODE_TORQUE;
    motor->target = 0.0f;
    
    // 初始化PID控制器（针对20kHz控制频率优化，减少电流尖峰）
    FOC_PID_Init(&motor->pid_id, POSITION_PID, 1.6f, 1.2f, 0.0f, motor->voltage_limit);   // d轴电流环
    FOC_PID_Init(&motor->pid_iq, POSITION_PID, 1.6f, 1.2f, 0.0f, motor->voltage_limit);   // q轴电流环
    return true;
}

/**
 * @brief 【FOC配置】设置硬件抽象层接口
 * @param motor 电机控制结构体指针
 * @param hal 硬件抽象层接口结构体指针
 * @note 绑定硬件相关的读写函数（角度、电流、电压、母线电压等）
 *       支持多电机控制，每个电机可以有不同的HAL实现
 */
void FOC_SetHAL(FOC_Motor_t *motor, FOC_HAL_t *hal)
{

    motor->hal = *hal;
}

/**
 * @brief 【内部函数】电流环控制（FOC核心）
 * @param motor 电机控制结构体指针
 * @param target_iq 目标q轴电流(A)，决定输出转矩
 * @note FOC第三步：在dq坐标系下进行电流PID控制
 *       - d轴电流控制：id=0（最大转矩电流比MTPA控制）
 *       - q轴电流控制：iq=目标转矩电流
 */
static void FOC_CurrentControl(FOC_Motor_t *motor, float target_iq)
{
    // d轴电流控制(id=0，实现最大转矩电流比控制)
    motor->vd = FOC_PID_Update(&motor->pid_id, 0.0f, motor->id);
    // q轴电流控制（q轴电流产生转矩）
    motor->vq = FOC_PID_Update(&motor->pid_iq, target_iq, motor->iq);
}



/**
 * @brief 电角度零点校准
 * @note 采用d轴对齐法，使用电压斜坡和多点采样提高精度和鲁棒性
 */
void FOC_CalibrateZeroOffset(FOC_Motor_t *motor)
{    
    // 获取母线电压
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    float v_align = vdc * 0.2f; // 使用20%母线电压对齐
    
    float alpha, beta, va, vb, vc;
    
    // 阶段1: 使用电压斜坡平滑对齐到电角度0，避免冲击
    for (int step = 0; step <= 20; step++) {
        float v_ramp = v_align * step / 20.0f;
        FOC_InvPark(v_ramp, 0.0f, 1.0f, 0.0f, &alpha, &beta);
        FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
        motor->hal.set_voltages(va, vb, vc);
        HAL_Delay(15); // 总共300ms斜坡上升
    }
    
    // 阶段2: 保持对齐电压，等待转子完全稳定，持续更新编码器
    for (int i = 0; i < 300; i++) {
        HAL_Delay(2);
        if (motor->hal.read_angle) {
            motor->hal.read_angle(); // 每次循环都更新编码器，避免角度跳变
        }
    }
    
    // 阶段3: 多次采样求平均，减少测量噪声影响
    float angle_sum = 0.0f;
    const int samples = 50;
    for (int i = 0; i < samples; i++) {
        angle_sum += motor->hal.read_angle();
        HAL_Delay(2);
    }
    float aligned_angle = angle_sum / samples;
    
    // 阶段4: 计算电角度零偏移（考虑传感器方向）
    // 公式推导：电角度 = (机械角度 × 极对数 + 零偏移) % 2π
    // 对齐时电角度=0，所以：零偏移 = -机械角度 × 极对数
    motor->elec_zero_offset = -(motor->sensor_direction * aligned_angle) * motor->pole_pairs;
    
    // 阶段5: 归一化到[0, 2π)，使用fmodf效率更高
    motor->elec_zero_offset = fmodf(motor->elec_zero_offset, FOC_2PI);
    if (motor->elec_zero_offset < 0.0f) {
        motor->elec_zero_offset += FOC_2PI;
    }

    // 阶段6: 电压斜坡下降，平滑释放转子
    for (int step = 20; step >= 0; step--) {
        float v_ramp = v_align * step / 20.0f;
        FOC_InvPark(v_ramp, 0.0f, 1.0f, 0.0f, &alpha, &beta);
        FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
        motor->hal.set_voltages(va, vb, vc);
        HAL_Delay(8); // 总共168ms斜坡下降
    }
    
    // 完全停止输出
    motor->hal.set_voltages(0.0f, 0.0f, 0.0f);
    HAL_Delay(200);
}

// ===== 配置函数 =====

/**
 * @brief 【配置】设置FOC控制模式
 * @param motor 电机控制结构体指针
 * @param mode 控制模式（目前只支持FOC_MODE_TORQUE转矩控制）
 * @note 转矩控制模式：直接控制q轴电流，实现转矩控制
 */
void FOC_SetMode(FOC_Motor_t *motor, FOC_Mode_t mode)
{
    
    motor->mode = mode;
}

/**
 * @brief 【配置】设置控制目标值
 * @param motor 电机控制结构体指针
 * @param target 目标值（转矩模式下为目标电流，单位A）
 * @note 在转矩控制模式下，target即为目标q轴电流
 */
void FOC_SetTarget(FOC_Motor_t *motor, float target)
{
  
    motor->target = target;
}

/**
 * @brief 【配置】设置电压限制
 * @param motor 电机控制结构体指针
 * @param limit 电压限制值(V)
 * @note 限制电流环PID输出的最大电压，防止过压
 *       同时更新d轴和q轴电流环PID的输出限幅
 */
void FOC_SetVoltageLimit(FOC_Motor_t *motor, float limit)
{
    motor->voltage_limit = limit;
    motor->pid_id.output_limit = limit;
    motor->pid_iq.output_limit = limit;
}

/**
 * @brief 【配置】设置电流限制
 * @param motor 电机控制结构体指针
 * @param limit 电流限制值(A)
 * @note 限制目标电流的最大值，防止过流损坏电机或驱动器
 */
void FOC_SetCurrentLimit(FOC_Motor_t *motor, float limit)
{
    motor->current_limit = limit;
}

// ===== 状态获取函数 =====

/**
 * @brief 【状态读取】获取d轴电流
 * @param motor 电机控制结构体指针
 * @return d轴电流(A)，磁场方向电流
 * @note d轴电流在FOC控制中通常控制为0（MTPA控制）
 */
float FOC_GetCurrent_D(FOC_Motor_t *motor)
{
    return motor ? motor->id : 0.0f;
}

/**
 * @brief 【状态读取】获取q轴电流
 * @param motor 电机控制结构体指针
 * @return q轴电流(A)，转矩方向电流
 * @note q轴电流直接决定电机输出转矩，Torque = Kt * Iq
 */
float FOC_GetCurrent_Q(FOC_Motor_t *motor)
{
    return motor ? motor->iq : 0.0f;
} 

/**
 * @brief 【状态读取】获取电机角速度
 * @param motor 电机控制结构体指针
 * @return 角速度(rad/s)
 * @note 从编码器读取的机械角速度
 */
float FOC_GetVelocity(FOC_Motor_t *motor)
{
    return motor ? motor->velocity : 0.0f;
}

/**
 * @brief 【状态读取】获取电机机械角度
 * @param motor 电机控制结构体指针
 * @return 机械角度(rad)
 * @note 从编码器读取的转子位置
 */
float FOC_GetAngle(FOC_Motor_t *motor)
{
    return motor ? motor->angle_mech : 0.0f;
}

// ===== 默认HAL绑定 =====

/**
 * @brief 【HAL绑定】绑定电机1的默认硬件接口
 * @param motor 电机控制结构体指针
 * @note 将电机1的硬件接口（AS5600_l + TIM2 + ADC）绑定到motor->hal
 *       适用于左侧电机或主电机
 */
void FOC_AttachDefaultHAL(FOC_Motor_t *motor)
{
    FOC_HAL_t hal = {
        .read_angle = default_board_read_angle,
        .read_currents = default_board_read_currents,
        .set_voltages = default_board_set_voltages,
        .get_bus_voltage = default_board_get_bus_voltage,
        .get_velocity = default_board_get_velocity,
    };
    FOC_SetHAL(motor, &hal);
}

/**
 * @brief 【HAL绑定】绑定电机2的硬件接口
 * @param motor 电机控制结构体指针
 * @note 将电机2的硬件接口（AS5600_r + TIM4 + ADC）绑定到motor->hal
 *       适用于右侧电机或副电机
 */
void FOC_AttachMotor2HAL(FOC_Motor_t *motor)
{

    FOC_HAL_t hal = {
        .read_angle = motor2_read_angle,
        .read_currents = motor2_read_currents,
        .set_voltages = motor2_set_voltages,
        .get_bus_voltage = motor2_get_bus_voltage,
        .get_velocity = motor2_get_velocity,
    };
    FOC_SetHAL(motor, &hal);
}

/**
 * @brief 【配置】设置传感器方向
 * @param motor 电机控制结构体指针
 * @param direction 传感器方向（>=0为正向，<0为反向）
 * @note 用于校正编码器安装方向，确保角度和速度符号正确
 */
void FOC_SetSensorDirection(FOC_Motor_t *motor, int8_t direction)
{

    motor->sensor_direction = (direction >= 0) ? 1 : -1;
}

/**
 * @brief 【校准】自动校准传感器方向
 * @param motor 电机控制结构体指针
 * @note 通过施加正向电角度，观察机械角度变化方向来判断传感器方向
 *       步骤：1.对齐到电角度0  2.转动到电角度20°  3.比较角度变化
 *       如果角度增加，传感器方向为正；如果角度减少，传感器方向为负
 */
void FOC_CalibrateDirection(FOC_Motor_t *motor)
{
   

    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    float v_align = vdc * 0.2f;

    // 先在电角度0对齐
    float alpha, beta, va, vb, vc;
    FOC_InvPark(v_align, 0.0f, 1.0f, 0.0f, &alpha, &beta);
    FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
    motor->hal.set_voltages(va, vb, vc);
    HAL_Delay(200);

    // 读取对齐角度
    float angle0 = motor->hal.read_angle();

    // 沿着+电角方向小幅转动（约20度）
    float theta = 0.35f; // ~20deg
    float cos1 = arm_cos_f32(theta);
    float sin1 = arm_sin_f32(theta);
    FOC_InvPark(v_align, 0.0f, cos1, sin1, &alpha, &beta);
    FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
    motor->hal.set_voltages(va, vb, vc);
    HAL_Delay(200);

    float angle1 = motor->hal.read_angle();

    // 停止输出
    motor->hal.set_voltages(0.0f, 0.0f, 0.0f);

    // 比较机械角度变化方向
    float d = FOC_NormalizeAngle(angle1 - angle0);
    motor->sensor_direction = (d >= 0.0f) ? 1 : -1;
}

/**
 * @brief 【FOC主循环】电流环更新函数 - FOC控制的核心执行函数
 * @param motor 电机控制结构体指针
 * @note 这是FOC控制的主要执行函数，需要在高频定时器中断中调用（建议20kHz）
 * 
 * 执行流程：
 * 1. 读取传感器数据（角度、三相电流）
 * 2. 计算电角度（机械角度 × 极对数 + 零偏移）
 * 3. Clarke变换：abc三相电流 -> αβ两相静止坐标系
 * 4. Park变换：αβ静止坐标系 -> dq旋转坐标系
 * 5. 电流环PID控制：计算dq轴控制电压
 * 6. 反Park变换：dq旋转坐标系 -> αβ静止坐标系
 * 7. SVPWM：αβ电压 -> abc三相PWM输出
 * 8. 过流保护：检测电流幅值，超限则停机
 */
void FOC_UpdateCurrentLoop(FOC_Motor_t *motor)
{
    // ========== 步骤1：读取传感器数据 ==========
    // 读取机械角度
    if (motor->hal.read_angle) {
        motor->angle_mech = motor->sensor_direction * motor->hal.read_angle();
    }
    // 读取三相电流
    if (motor->hal.read_currents) {
        motor->hal.read_currents(&motor->ia, &motor->ib, &motor->ic);
    }
    // ========== 步骤2：计算电角度 ==========
    // 电角度 = 机械角度 × 极对数 + 零偏移
    motor->angle_elec = fmodf(motor->angle_mech * motor->pole_pairs + motor->elec_zero_offset, FOC_2PI);
    if (motor->angle_elec < 0.0f) motor->angle_elec += FOC_2PI;

    // ========== 步骤3：Clarke变换 abc -> αβ ==========
    float i_alpha, i_beta;
    FOC_Clarke(motor->ia, motor->ib, motor->ic, &i_alpha, &i_beta);

    // ========== 步骤4：Park变换 αβ -> dq ==========
    float cos_theta = arm_cos_f32(motor->angle_elec);
    float sin_theta = arm_sin_f32(motor->angle_elec);
    FOC_Park(i_alpha, i_beta, cos_theta, sin_theta, &motor->id, &motor->iq);

    // ========== 步骤5：电流环控制 ==========
    // 目标电流限幅
    float target_iq = motor->target;
    if (target_iq > motor->current_limit) target_iq = motor->current_limit;
    if (target_iq < -motor->current_limit) target_iq = -motor->current_limit;

    // PID控制计算dq轴电压
    FOC_CurrentControl(motor, target_iq);

    // ========== 步骤6：反Park变换 dq -> αβ ==========
    float v_alpha, v_beta;
    FOC_InvPark(motor->vd, motor->vq, cos_theta, sin_theta, &v_alpha, &v_beta);

    // ========== 步骤7：SVPWM αβ -> abc ==========
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    FOC_SVPWM(v_alpha, v_beta, vdc, &motor->va, &motor->vb, &motor->vc);

    // 输出三相电压到PWM
    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(motor->va, motor->vb, motor->vc);
    }

    // ========== 步骤8：过流保护 ==========
    // // 计算电流幅值 |I| = sqrt(Id² + Iq²)
    // float current_magnitude_sq = motor->id * motor->id + motor->iq * motor->iq;
    // float current_magnitude = 0.0f;
    // arm_sqrt_f32(current_magnitude_sq, &current_magnitude);
    
    // // 如果电流超过限制的120%，立即停机
    // if (current_magnitude > motor->current_limit * 1.2f) {
    //     if (motor->hal.set_voltages) {
    //         motor->hal.set_voltages(0.0f, 0.0f, 0.0f);
    //     }
    // }
}


