/**
 * @file simplefoc.c
 * @brief SimpleFOC库实现 - 完全重写版本
 * @date 2026-01-27
 * @version 3.0
 *
 * 参考SimpleFOC开源库的标准实现
 * 专注于电流环闭环控制，确保id/iq正确追踪
 */

#include "simplefoc.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "as5600.h"
#include "tim.h"
#include <string.h>

// 外部硬件接口（在main.c中定义）
extern AS5600_t as5600_l, as5600_r;

// ============================================================================
// 数学工具函数
// ============================================================================

/**
 * @brief 角度归一化到[0, 2π)
 */
float FOC_NormalizeAngle(float angle)
{
    angle = fmodf(angle, FOC_2PI);
    if (angle < 0.0f) {
        angle += FOC_2PI;
    }
    return angle;
}

/**
 * @brief 限幅函数
 */
static inline float constrain(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ============================================================================
// Clarke变换：abc三相 -> αβ两相静止坐标系
// ============================================================================

/**
 * @brief Clarke变换
 * @note 使用CMSIS-DSP库函数
 *
 * 公式：
 *   iα = ia
 *   iβ = (ia + 2*ib) / √3
 */
void FOC_Clarke(float ia, float ib, float ic, float *alpha, float *beta)
{
    arm_clarke_f32(ia, ib, alpha, beta);
}

// ============================================================================
// Park变换：αβ静止坐标系 -> dq旋转坐标系
// ============================================================================

/**
 * @brief Park变换（正变换）
 * @note 使用CMSIS-DSP库函数
 *
 * 公式：
 *   id = iα * cos(θ) + iβ * sin(θ)
 *   iq = -iα * sin(θ) + iβ * cos(θ)
 *
 * 物理意义：
 *   - d轴：磁场方向（flux axis）
 *   - q轴：转矩方向（torque axis），领先d轴90°
 *   - θ：转子电角度
 */
void FOC_Park(float alpha, float beta, float cos_theta, float sin_theta, float *d, float *q)
{
    // CMSIS-DSP参数顺序：(Ialpha, Ibeta, pId, pIq, sinVal, cosVal)
    arm_park_f32(alpha, beta, d, q, sin_theta, cos_theta);
}

/**
 * @brief 反Park变换（逆变换）
 * @note 使用CMSIS-DSP库函数
 *
 * 公式：
 *   vα = vd * cos(θ) - vq * sin(θ)
 *   vβ = vd * sin(θ) + vq * cos(θ)
 */
void FOC_InvPark(float d, float q, float cos_theta, float sin_theta, float *alpha, float *beta)
{
    // CMSIS-DSP参数顺序：(Id, Iq, pIalpha, pIbeta, sinVal, cosVal)
    arm_inv_park_f32(d, q, alpha, beta, sin_theta, cos_theta);
}

// ============================================================================
// SVPWM：空间矢量脉宽调制
// ============================================================================

/**
 * @brief SVPWM调制
 * @param alpha α轴电压
 * @param beta β轴电压
 * @param vdc 母线电压
 * @param va, vb, vc 输出三相电压
 *
 * @note 使用标准SVPWM算法
 */
void FOC_SVPWM(float alpha, float beta, float vdc, float *va, float *vb, float *vc)
{
    // 计算三相电压（反Clarke变换）
    *va = alpha;
    *vb = -0.5f * alpha + FOC_SQRT3_2 * beta;
    *vc = -0.5f * alpha - FOC_SQRT3_2 * beta;

    // 找出最大值和最小值
    float max_val = *va;
    if (*vb > max_val) max_val = *vb;
    if (*vc > max_val) max_val = *vc;

    float min_val = *va;
    if (*vb < min_val) min_val = *vb;
    if (*vc < min_val) min_val = *vc;

    // 中点偏移（SVPWM的关键）
    float offset = -(max_val + min_val) / 2.0f;

    // 应用偏移并归一化到[0, vdc]
    *va = (*va + offset) / vdc * 0.5f + 0.5f;
    *vb = (*vb + offset) / vdc * 0.5f + 0.5f;
    *vc = (*vc + offset) / vdc * 0.5f + 0.5f;

    // 限幅到[0, 1]
    *va = constrain(*va, 0.0f, 1.0f);
    *vb = constrain(*vb, 0.0f, 1.0f);
    *vc = constrain(*vc, 0.0f, 1.0f);
}

// ============================================================================
// PID控制器
// ============================================================================

/**
 * @brief PID控制器初始化
 */
void FOC_PID_Init(FOC_PID_t *pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;
}

/**
 * @brief PID控制器更新
 * @param pid PID控制器
 * @param error 误差
 * @param dt 采样周期（秒）
 * @return 控制输出
 */
float FOC_PID_Update(FOC_PID_t *pid, float error, float dt)
{
    // 比例项
    float p_term = pid->kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * dt;
    // 积分限幅（防止积分饱和）
    float integral_limit = pid->output_limit / (pid->ki + 1e-6f);
    pid->integral = constrain(pid->integral, -integral_limit, integral_limit);
    float i_term = pid->ki * pid->integral;

    // 微分项
    float d_term = pid->kd * (error - pid->error_prev) / dt;
    pid->error_prev = error;

    // 总输出
    float output = p_term + i_term + d_term;

    // 输出限幅
    output = constrain(output, -pid->output_limit, pid->output_limit);

    return output;
}

/**
 * @brief PID控制器复位
 */
void FOC_PID_Reset(FOC_PID_t *pid)
{
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;
}

// ============================================================================
// FOC电机初始化
// ============================================================================

/**
 * @brief FOC电机初始化
 */
bool FOC_Init(FOC_Motor_t *motor, uint8_t pole_pairs)
{
    if (!motor) return false;

    // 清零结构体
    memset(motor, 0, sizeof(FOC_Motor_t));

    // 设置基本参数
    motor->pole_pairs = pole_pairs;
    motor->voltage_limit = 12.0f;
    motor->current_limit = 2.0f;
    motor->mode = FOC_MODE_TORQUE;
    motor->sensor_direction = 1;

    // 电机参数（默认值，可后续修改）
    motor->rs = 1.0f;           // 相电阻 1Ω
    motor->ld = 0.001f;         // d轴电感 1mH
    motor->lq = 0.001f;         // q轴电感 1mH
    motor->use_decoupling = false;  // 默认不使用解耦

    // 初始化PID控制器
    // d轴电流环：目标id=0，需要快速响应
    FOC_PID_Init(&motor->pid_id, 5.0f, 50.0f, 0.0f, motor->voltage_limit);

    // q轴电流环：控制转矩
    FOC_PID_Init(&motor->pid_iq, 5.0f, 50.0f, 0.0f, motor->voltage_limit);

    motor->initialized = true;

    return true;
}

/**
 * @brief 设置硬件抽象层接口
 */
void FOC_SetHAL(FOC_Motor_t *motor, FOC_HAL_t *hal)
{
    if (motor && hal) {
        motor->hal = *hal;
    }
}

// ============================================================================
// 配置函数
// ============================================================================

void FOC_SetMode(FOC_Motor_t *motor, FOC_Mode_t mode)
{
    if (motor) motor->mode = mode;
}

void FOC_SetTarget(FOC_Motor_t *motor, float target)
{
    if (motor) motor->target = target;
}

void FOC_SetVoltageLimit(FOC_Motor_t *motor, float limit)
{
    if (motor) {
        motor->voltage_limit = limit;
        motor->pid_id.output_limit = limit;
        motor->pid_iq.output_limit = limit;
    }
}

void FOC_SetCurrentLimit(FOC_Motor_t *motor, float limit)
{
    if (motor) motor->current_limit = limit;
}

void FOC_SetSensorDirection(FOC_Motor_t *motor, int8_t direction)
{
    if (motor) motor->sensor_direction = (direction >= 0) ? 1 : -1;
}

// ============================================================================
// 状态获取函数
// ============================================================================

float FOC_GetCurrent_D(FOC_Motor_t *motor)
{
    return motor ? motor->id : 0.0f;
}

float FOC_GetCurrent_Q(FOC_Motor_t *motor)
{
    return motor ? motor->iq : 0.0f;
}

float FOC_GetVelocity(FOC_Motor_t *motor)
{
    return motor ? motor->velocity : 0.0f;
}

float FOC_GetAngle(FOC_Motor_t *motor)
{
    return motor ? motor->angle_mech : 0.0f;
}

// ============================================================================
// 校准函数
// ============================================================================

/**
 * @brief 传感器方向校准
 * @note 通过开环旋转电机，检测传感器角度变化方向
 */
void FOC_CalibrateDirection(FOC_Motor_t *motor)
{
    if (!motor || !motor->hal.read_angle || !motor->hal.set_voltages) {
        return;
    }

    // 读取初始角度
    float angle_start = motor->hal.read_angle();

    // 开环旋转一段时间
    float v_test = 3.0f;  // 测试电压
    float angle_elec = 0.0f;
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;

    uint32_t tick_start = HAL_GetTick();
    while ((HAL_GetTick() - tick_start) < 1000) {  // 旋转1秒
        angle_elec += 0.001f * FOC_2PI;  // 每ms增加一点角度
        if (angle_elec >= FOC_2PI) angle_elec -= FOC_2PI;

        float cos_e = arm_cos_f32(angle_elec);
        float sin_e = arm_sin_f32(angle_elec);

        float alpha, beta;
        FOC_InvPark(0.0f, v_test, cos_e, sin_e, &alpha, &beta);

        float va, vb, vc;
        FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
        motor->hal.set_voltages(va, vb, vc);

        HAL_Delay(1);
    }

    // 读取结束角度
    float angle_end = motor->hal.read_angle();

    // 停止电机
    motor->hal.set_voltages(0.0f, 0.0f, 0.0f);

    // 判断方向
    float angle_diff = angle_end - angle_start;
    if (angle_diff > FOC_PI) angle_diff -= FOC_2PI;
    if (angle_diff < -FOC_PI) angle_diff += FOC_2PI;

    motor->sensor_direction = (angle_diff > 0) ? 1 : -1;
}

/**
 * @brief 电角度零偏移校准
 * @note 施加直流电压到d轴，使转子对齐，记录此时的机械角度
 *
 * 关键点：
 * 1. 施加电压到d轴（vd=v_align, vq=0）
 * 2. 转子会对齐到d轴方向
 * 3. 记录此时的机械角度
 * 4. 计算零偏移，使得运行时电角度=0对应d轴对齐位置
 */
void FOC_CalibrateZeroOffset(FOC_Motor_t *motor)
{
    if (!motor || !motor->hal.read_angle || !motor->hal.set_voltages) {
        return;
    }

    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    float v_align = 3.0f;  // 对齐电压

    // 阶段1：电压斜坡上升，平滑启动
    for (int step = 0; step <= 100; step++) {
        float v_ramp = v_align * step / 100.0f;

        // 施加电压到d轴（vd=v_ramp, vq=0）
        // 此时电角度=0，所以cos=1, sin=0
        float alpha, beta;
        FOC_InvPark(v_ramp, 0.0f, 1.0f, 0.0f, &alpha, &beta);

        float va, vb, vc;
        FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
        motor->hal.set_voltages(va, vb, vc);

        HAL_Delay(5);  // 每步5ms
    }

    // 阶段2：保持电压，等待转子稳定
    float alpha, beta;
    FOC_InvPark(v_align, 0.0f, 1.0f, 0.0f, &alpha, &beta);
    float va, vb, vc;
    FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
    motor->hal.set_voltages(va, vb, vc);
    HAL_Delay(1000);  // 保持1秒

    // 阶段3：采样对齐角度（多次采样取平均）
    float angle_sum = 0.0f;
    int samples = 100;
    for (int i = 0; i < samples; i++) {
        angle_sum += motor->hal.read_angle();
        HAL_Delay(2);
    }
    float aligned_angle = angle_sum / samples;

    // 阶段4：计算电角度零偏移
    // 关键公式：
    //   运行时：angle_elec = (angle_mech * sensor_direction) * pole_pairs + zero_offset
    //   对齐时：angle_elec = 0（d轴对齐）
    //   所以：zero_offset = -(aligned_angle * sensor_direction) * pole_pairs
    motor->elec_zero_offset = -(aligned_angle * motor->sensor_direction) * motor->pole_pairs;

    // 归一化到[0, 2π)
    motor->elec_zero_offset = FOC_NormalizeAngle(motor->elec_zero_offset);

    // 同步到AS5600结构体（用于预计算电角度）
    as5600_l.zero_elec_offset = motor->elec_zero_offset;
    as5600_l.sensor_direction = motor->sensor_direction;

    // 阶段5：电压斜坡下降，平滑释放
    for (int step = 100; step >= 0; step--) {
        float v_ramp = v_align * step / 100.0f;
        FOC_InvPark(v_ramp, 0.0f, 1.0f, 0.0f, &alpha, &beta);
        FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
        motor->hal.set_voltages(va, vb, vc);
        HAL_Delay(5);
    }

    // 停止电机
    motor->hal.set_voltages(0.0f, 0.0f, 0.0f);
}

// ============================================================================
// FOC电流环控制（核心函数）
// ============================================================================

/**
 * @brief FOC电流环更新
 * @param motor 电机控制结构体
 *
 * @note 这是FOC控制的核心函数，需要在高频定时器中断中调用（20kHz）
 *
 * 执行流程：
 * 1. 读取传感器数据（电角度、三相电流）
 * 2. Clarke变换：abc -> αβ
 * 3. Park变换：αβ -> dq
 * 4. 电流环PID控制
 * 5. 反Park变换：dq -> αβ
 * 6. SVPWM：αβ -> abc
 */
void FOC_UpdateCurrentLoop(FOC_Motor_t *motor)
{
    // ========== 步骤1：读取传感器数据 ==========
    // 读取电角度（优先使用预计算值）
    if (motor->hal.read_elec_angle) {
        // 使用预计算的电角度（在编码器中断中已计算好）
        motor->angle_elec = motor->hal.read_elec_angle();
    } else {
        // 实时计算电角度
        if (motor->hal.read_angle) {
            motor->angle_mech = motor->hal.read_angle();
        }
        motor->angle_elec = (motor->angle_mech * motor->sensor_direction) * motor->pole_pairs + motor->elec_zero_offset;
        motor->angle_elec = FOC_NormalizeAngle(motor->angle_elec);
    }

    // 读取三相电流
    if (motor->hal.read_currents) {
        motor->hal.read_currents(&motor->ia, &motor->ib, &motor->ic);
    }

    // ========== 步骤2：Clarke变换 abc -> αβ ==========
    float i_alpha, i_beta;
    FOC_Clarke(motor->ia, motor->ib, motor->ic, &i_alpha, &i_beta);

    // ========== 步骤3：Park变换 αβ -> dq ==========
    float cos_theta = arm_cos_f32(motor->angle_elec);
    float sin_theta = arm_sin_f32(motor->angle_elec);
    FOC_Park(i_alpha, i_beta, cos_theta, sin_theta, &motor->id, &motor->iq);

    // ========== 步骤4：电流环PID控制 ==========

    // 目标电流
    float target_id = 0.0f;  // d轴电流目标始终为0（最大转矩控制）
    float target_iq = motor->target;  // q轴电流目标

    // 限幅
    target_iq = constrain(target_iq, -motor->current_limit, motor->current_limit);

    // 计算误差
    float error_id = target_id - motor->id;
    float error_iq = target_iq - motor->iq;

    // PID控制（采样周期 = 1/20kHz = 50us = 0.00005s）
    float dt = 0.00005f;
    motor->vd = FOC_PID_Update(&motor->pid_id, error_id, dt);
    motor->vq = FOC_PID_Update(&motor->pid_iq, error_iq, dt);
    // 电压限幅（圆形限幅）
    float v_magnitude = sqrtf(motor->vd * motor->vd + motor->vq * motor->vq);
    if (v_magnitude > motor->voltage_limit) {
        float scale = motor->voltage_limit / v_magnitude;
        motor->vd *= scale;
        motor->vq *= scale;
    }
    // ========== 步骤5：反Park变换 dq -> αβ ==========
    float v_alpha, v_beta;
    FOC_InvPark(motor->vd, motor->vq, cos_theta, sin_theta, &v_alpha, &v_beta);

    // ========== 步骤6：SVPWM αβ -> abc ==========
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    FOC_SVPWM(v_alpha, v_beta, vdc, &motor->va, &motor->vb, &motor->vc);

    // ========== 步骤7：输出PWM ==========
    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(motor->va, motor->vb, motor->vc);
    }
}

// ============================================================================
// 硬件接口绑定（板级HAL）
// ============================================================================

// 这些函数在hardware目录中实现
extern float ADC_Get_Motor1_Current_A(void);
extern float ADC_Get_Motor1_Current_B(void);
extern float ADC_Get_Motor1_Current_C(void);
extern float ADC_Get_Motor2_Current_A(void);
extern float ADC_Get_Motor2_Current_B(void);
extern float ADC_Get_Motor2_Current_C(void);
extern void TIM_SetDutyCycle_Motor1(float da, float db, float dc);
extern void TIM_SetDutyCycle_Motor2(float da, float db, float dc);

// 电机1的HAL接口实现
static float motor1_read_angle(void)
{
    return AS5600_GetAngleRad(&as5600_l);
}

static void motor1_read_currents(float *ia, float *ib, float *ic)
{
    *ia = ADC_Get_Motor1_Current_A();
    *ib = ADC_Get_Motor1_Current_B();
    *ic = ADC_Get_Motor1_Current_C();
}

static void motor1_set_voltages(float va, float vb, float vc)
{
    TIM_SetDutyCycle_Motor1(va, vb, vc);
}

static float motor1_get_bus_voltage(void)
{
    return 12.0f;  // 假设12V母线电压
}

static float motor1_get_velocity(void)
{
    return AS5600_GetVelRad(&as5600_l);
}

static float motor1_read_elec_angle(void)
{
    return AS5600_GetPrecomputedElecAngle(&as5600_l);
}

/**
 * @brief 绑定电机1的默认HAL接口
 */
void FOC_AttachDefaultHAL(FOC_Motor_t *motor)
{
    if (!motor) return;

    motor->hal.read_angle = motor1_read_angle;
    motor->hal.read_currents = motor1_read_currents;
    motor->hal.set_voltages = motor1_set_voltages;
    motor->hal.get_bus_voltage = motor1_get_bus_voltage;
    motor->hal.get_velocity = motor1_get_velocity;
    motor->hal.read_elec_angle = motor1_read_elec_angle;
}

// 电机2的HAL接口实现
static float motor2_read_angle(void)
{
    return AS5600_GetAngleRad(&as5600_r);
}

static void motor2_read_currents(float *ia, float *ib, float *ic)
{
    *ia = ADC_Get_Motor2_Current_A();
    *ib = ADC_Get_Motor2_Current_B();
    *ic = ADC_Get_Motor2_Current_C();
}

static void motor2_set_voltages(float va, float vb, float vc)
{
    TIM_SetDutyCycle_Motor2(va, vb, vc);
}

static float motor2_get_bus_voltage(void)
{
    return 12.0f;
}

static float motor2_get_velocity(void)
{
    return AS5600_GetVelRad(&as5600_r);
}

static float motor2_read_elec_angle(void)
{
    return AS5600_GetPrecomputedElecAngle(&as5600_r);
}

/**
 * @brief 绑定电机2的默认HAL接口
 */
void FOC_AttachMotor2HAL(FOC_Motor_t *motor)
{
    if (!motor) return;

    motor->hal.read_angle = motor2_read_angle;
    motor->hal.read_currents = motor2_read_currents;
    motor->hal.set_voltages = motor2_set_voltages;
    motor->hal.get_bus_voltage = motor2_get_bus_voltage;
    motor->hal.get_velocity = motor2_get_velocity;
    motor->hal.read_elec_angle = motor2_read_elec_angle;
}

// ============================================================================
// 开环测试函数
// ============================================================================

/**
 * @brief 开环测试
 * @param motor 电机结构体
 * @param voltage 测试电压
 * @param speed 旋转速度（rad/s）
 */
void FOC_OpenLoopTest(FOC_Motor_t *motor, float voltage, float speed)
{
    if (!motor) return;

    motor->open_loop_voltage = voltage;
    motor->open_loop_angle += speed * 0.00005f;  // dt = 50us
    motor->open_loop_angle = FOC_NormalizeAngle(motor->open_loop_angle);

    float cos_e = arm_cos_f32(motor->open_loop_angle);
    float sin_e = arm_sin_f32(motor->open_loop_angle);

    float alpha, beta;
    FOC_InvPark(0.0f, voltage, cos_e, sin_e, &alpha, &beta);

    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    FOC_SVPWM(alpha, beta, vdc, &motor->va, &motor->vb, &motor->vc);

    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(motor->va, motor->vb, motor->vc);
    }
}

/**
 * @brief 停止开环测试
 */
void FOC_StopOpenLoopTest(FOC_Motor_t *motor)
{
    if (!motor) return;

    motor->open_loop_voltage = 0.0f;
    motor->open_loop_angle = 0.0f;

    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(0.0f, 0.0f, 0.0f);
    }
}
