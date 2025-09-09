/**
 * @file simplefoc.c
 * @brief 标准FOC(Field Oriented Control)算法实现
 * @version 2.0
 * @date 2024
 * 
 * 严格按照FOC理论实现：
 * 1. Clarke变换(abc->αβ)
 * 2. Park变换(αβ->dq) 
 * 3. PID电流控制(d轴id=0, q轴iq=转矩)
 * 4. 反Park变换(dq->αβ)
 * 5. SVPWM(αβ->abc)
 */

#include "simplefoc.h"
#include <string.h>
#include <stdio.h>
#include "tim.h"
#include "adc_measure.h"
#include "as5600.h"

extern AS5600_t as5600_l; 
extern AS5600_t as5600_r; 


// ===== 内部函数声明 =====
static void FOC_ComputeVelocity(FOC_Motor_t *motor, float dt);
static void FOC_CurrentControl(FOC_Motor_t *motor, float dt);
static void FOC_OuterControl(FOC_Motor_t *motor, float dt);

// ===== 板级默认HAL静态实现（从main.c迁移） =====
// 依赖外部句柄与变量	extern AS5600_t as5600_l;

static float default_board_get_bus_voltage(void)
{
    return 12.0f;
}

static float default_board_read_angle(void)
{
    return AS5600_GetAngleRad(&as5600_l);
}

static void default_board_read_currents(float *ia, float *ib, float *ic)
{
    if (!ia || !ib || !ic) return;
    *ia = ADC_Get_Phase_Current_A();
    *ib = ADC_Get_Phase_Current_B();
    *ic = -(*ia + *ib);
}

static void default_board_set_voltages(float va, float vb, float vc)
{
    float vdc = default_board_get_bus_voltage();
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

    uint32_t pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(duty_a * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(duty_b * pwm_period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(duty_c * pwm_period));
}

// ===== 数学工具函数 =====

/**
 * @brief 角度标准化到[-π, π]
 */
float FOC_NormalizeAngle(float angle)
{
    while (angle > FOC_PI) angle -= FOC_2PI;
    while (angle < -FOC_PI) angle += FOC_2PI;
    return angle;
}

/**
 * @brief Clarke变换: 三相坐标系(abc) -> 两相静止坐标系(αβ)
 * @param ia, ib, ic 三相电流
 * @param alpha, beta 输出αβ轴电流
 */
void FOC_Clarke(float ia, float ib, float ic, float *alpha, float *beta)
{    
    *alpha = ia;
    *beta = FOC_INV_SQRT3 * (ib - ic);
}

/**
 * @brief Park变换: 静止坐标系(αβ) -> 旋转坐标系(dq)
 * @param alpha, beta αβ轴分量
 * @param cos_theta, sin_theta 旋转角度的余弦和正弦
 * @param d, q 输出dq轴分量
 */
void FOC_Park(float alpha, float beta, float cos_theta, float sin_theta, float *d, float *q)
{
    // 标准Park变换矩阵
    // [d] = [ cos(θ)  sin(θ)] [α]
    // [q]   [-sin(θ)  cos(θ)] [β]
    
    *d =  alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

/**
 * @brief 反Park变换: 旋转坐标系(dq) -> 静止坐标系(αβ)
 * @param d, q dq轴分量
 * @param cos_theta, sin_theta 旋转角度的余弦和正弦
 * @param alpha, beta 输出αβ轴分量
 */
void FOC_InvPark(float d, float q, float cos_theta, float sin_theta, float *alpha, float *beta)
{
   
    *alpha = d * cos_theta - q * sin_theta;
    *beta  = d * sin_theta + q * cos_theta;
}

/**
 * @brief 空间矢量脉宽调制(SVPWM)
 * @param alpha, beta αβ轴电压
 * @param vdc 母线电压
 * @param va, vb, vc 输出三相电压
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
 * @brief 初始化PID控制器
 */
void FOC_PID_Init(FOC_PID_t *pid, float kp, float ki, float kd, float output_limit)
{
    if (!pid) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;
}

/**
 * @brief PID控制器更新
 */
float FOC_PID_Update(FOC_PID_t *pid, float error, float dt)
{
    if (!pid || dt <= 0.0f) return 0.0f;
    
    // 比例项
    float proportional = pid->kp * error;
    
    // 积分项 (带积分限幅)
    pid->integral += error * dt;
    float integral_limit = pid->output_limit / (pid->ki + 1e-6f);
    if (pid->integral > integral_limit) pid->integral = integral_limit;
    if (pid->integral < -integral_limit) pid->integral = -integral_limit;
    float integral = pid->ki * pid->integral;
    
    // 微分项
    float derivative = pid->kd * (error - pid->error_prev) / dt;
    pid->error_prev = error;
    
    // 总输出
    float output = proportional + integral + derivative;
    
    // 输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}

/**
 * @brief 重置PID控制器
 */
void FOC_PID_Reset(FOC_PID_t *pid)
{
    if (!pid) return;
    
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;
}

// ===== FOC核心函数 =====

/**
 * @brief 初始化FOC电机控制器
 */
bool FOC_Init(FOC_Motor_t *motor, uint8_t pole_pairs)
{
    if (!motor || pole_pairs == 0) return false;
    
    // 清零结构体
    memset(motor, 0, sizeof(FOC_Motor_t));
    
    // 基本参数
    motor->pole_pairs = pole_pairs;
    motor->initialized = true;
    motor->sensor_direction = 1;
    
    // 电机物理参数默认值
    motor->rs = 2.3f;           // 相电阻(Ω)
    motor->ld = 0.00086f;       // d轴电感(H)
    motor->lq = 0.00086f;       // q轴电感(H)
    motor->use_decoupling = false; // 默认不使用解耦
    
    // 控制限制
    motor->voltage_limit = 12.0f;   // 电压限制(V)
    motor->current_limit = 2.0f;    // 电流限制(A)
    
    // 控制模式
    motor->mode = FOC_MODE_TORQUE;
    motor->target = 0.0f;
    
    // 初始化PID控制器 - 保守参数确保稳定
    FOC_PID_Init(&motor->pid_id, 1.6f, 20.0f, 0.0f, motor->voltage_limit);
    FOC_PID_Init(&motor->pid_iq, 1.6f, 20.0f, 0.0f, motor->voltage_limit);
    FOC_PID_Init(&motor->pid_vel, 0.2f, 2.0f, 0.0f, motor->current_limit);
    FOC_PID_Init(&motor->pid_pos, 20.0f, 0.0f, 0.1f, 50.0f);
    
    return true;
}

/**
 * @brief 设置硬件抽象层接口
 */
void FOC_SetHAL(FOC_Motor_t *motor, FOC_HAL_t *hal)
{
    if (!motor || !hal) return;
    motor->hal = *hal;
}

/**
 * @brief 速度计算
 */
static void FOC_ComputeVelocity(FOC_Motor_t *motor, float dt)
{
    (void)dt;
    // 使用编码器内部的速度估计，避免由于传感器更新率(2kHz)低于FOC控制率(20kHz)导致的微分噪声
    motor->velocity = AS5600_GetVelRad(&as5600_l);
    motor->angle_prev = motor->angle_mech;
}

/**
 * @brief 外环控制(位置环、速度环)
 */
static void FOC_OuterControl(FOC_Motor_t *motor, float dt)
{
    float target_iq = 0.0f;
    
    switch (motor->mode) {
        case FOC_MODE_TORQUE:
            // 直接转矩控制
            target_iq = motor->target;
            break;
            
        case FOC_MODE_VELOCITY: {
            // 速度环控制
            float vel_error = motor->target - motor->velocity;
            target_iq = FOC_PID_Update(&motor->pid_vel, vel_error, dt);
            break;
        }
        
        case FOC_MODE_POSITION: {
            // 位置环控制
            float pos_error = FOC_NormalizeAngle(motor->target - motor->angle_mech);
            float target_vel = FOC_PID_Update(&motor->pid_pos, pos_error, dt);
            
            // 速度环
            float vel_error = target_vel - motor->velocity;
            target_iq = FOC_PID_Update(&motor->pid_vel, vel_error, dt);
            break;
        }
    }
    
    // 电流限制
    if (target_iq > motor->current_limit) target_iq = motor->current_limit;
    if (target_iq < -motor->current_limit) target_iq = -motor->current_limit;
    
    // 电流环控制
    FOC_CurrentControl(motor, dt);
}

/**
 * @brief 电流环控制
 */
static void FOC_CurrentControl(FOC_Motor_t *motor, float dt)
{
    // d轴电流控制(id=0，实现最大转矩电流比控制)
    float id_error = 0.0f - motor->id;
    motor->vd = FOC_PID_Update(&motor->pid_id, id_error, dt);
    
    // q轴电流控制
    float iq_target = 0.0f;
    switch (motor->mode) {
        case FOC_MODE_TORQUE:
            iq_target = motor->target;
            break;
        case FOC_MODE_VELOCITY:
        case FOC_MODE_POSITION:
            // 在外环里已把目标写入pid_vel，直接使用pid_vel的上一次输出作为目标
            iq_target = motor->pid_vel.error_prev * motor->pid_vel.kp + motor->pid_vel.integral * motor->pid_vel.ki + 0.0f; // 近似，不增加额外存储
            break;
    }
    float iq_error = iq_target - motor->iq;
    motor->vq = FOC_PID_Update(&motor->pid_iq, iq_error, dt);
    
    // 解耦补偿(如果启用)
    if (motor->use_decoupling) {
        float omega_e = motor->velocity * motor->pole_pairs; // 电角速度

        // 前馈解耦
        motor->vd += -omega_e * motor->lq * motor->iq;
        motor->vq += omega_e * motor->ld * motor->id + motor->rs * motor->iq;
    }
}

/**
 * @brief FOC主更新函数
 */
void FOC_Update(FOC_Motor_t *motor)
{
    if (!motor || !motor->initialized) return;
    
    // 固定控制周期（由TIM5以20kHz调度）
    const float dt = 0.00005f;
    
    // === 1. 传感器数据采集 ===
    if (motor->hal.read_angle) {
        motor->angle_mech = motor->sensor_direction * motor->hal.read_angle();
    }
    
    if (motor->hal.read_currents) {
        motor->hal.read_currents(&motor->ia, &motor->ib, &motor->ic);
    }
    
    // === 2. 速度计算 ===
    FOC_ComputeVelocity(motor, dt);

    // === 3. 电角度计算 ===
    motor->angle_elec = fmodf(motor->angle_mech * motor->pole_pairs + motor->elec_zero_offset, FOC_2PI);
    if (motor->angle_elec < 0.0f) motor->angle_elec += FOC_2PI;
    
    // === 4. Clarke变换 ===
    float i_alpha, i_beta;
    FOC_Clarke(motor->ia, motor->ib, motor->ic, &i_alpha, &i_beta);
    
    // === 5. Park变换 ===
    float cos_theta = cosf(motor->angle_elec);
    float sin_theta = sinf(motor->angle_elec);
    FOC_Park(i_alpha, i_beta, cos_theta, sin_theta, &motor->id, &motor->iq);
    
    // === 6. 控制环计算 ===
    FOC_OuterControl(motor, dt);
    
    // === 7. 反Park变换 ===
    float v_alpha, v_beta;
    FOC_InvPark(motor->vd, motor->vq, cos_theta, sin_theta, &v_alpha, &v_beta);
    
    // === 8. SVPWM ===
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    FOC_SVPWM(v_alpha, v_beta, vdc, &motor->va, &motor->vb, &motor->vc);
    
    // === 9. 输出到硬件 ===
    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(motor->va, motor->vb, motor->vc);
    }
    
    // === 10. 安全保护 ===
    float current_magnitude = sqrtf(motor->id * motor->id + motor->iq * motor->iq);
    if (current_magnitude > motor->current_limit * 1.2f) {
        // 过流保护：停止输出并重置积分项
        if (motor->hal.set_voltages) {
            motor->hal.set_voltages(0.0f, 0.0f, 0.0f);
        }
        FOC_PID_Reset(&motor->pid_id);
        FOC_PID_Reset(&motor->pid_iq);
        FOC_PID_Reset(&motor->pid_vel);
    }
}

/**
 * @brief 电角度零点校准
 */
void FOC_CalibrateZeroOffset(FOC_Motor_t *motor)
{
    if (!motor || !motor->hal.set_voltages || !motor->hal.read_angle) return;
    
    // 获取母线电压
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    float v_align = vdc * 0.2f; // 使用20%母线电压对齐
    
    // 使用InvPark+SVPWM在电角度0进行对齐
    float alpha, beta, va, vb, vc;
    FOC_InvPark(v_align, 0.0f, 1.0f, 0.0f, &alpha, &beta);
    FOC_SVPWM(alpha, beta, vdc, &va, &vb, &vc);
        motor->hal.set_voltages(va, vb, vc);
    
    // 等待转子稳定
    for (int i = 0; i < 300; i++) {
        HAL_Delay(2);
        if (motor->hal.read_angle) {
            motor->hal.read_angle(); // 让编码器稳定
    }
}

    // 读取对齐后的机械角度
    float aligned_angle = motor->hal.read_angle();
    
    // 计算电角度零偏移（考虑传感器方向）
    motor->elec_zero_offset = -(motor->sensor_direction * aligned_angle) * motor->pole_pairs;
    
    // 标准化到[0, 2π)
    while (motor->elec_zero_offset < 0.0f) motor->elec_zero_offset += FOC_2PI;
    while (motor->elec_zero_offset >= FOC_2PI) motor->elec_zero_offset -= FOC_2PI;

    // 停止输出
    motor->hal.set_voltages(0.0f, 0.0f, 0.0f);
    HAL_Delay(200);
}

// ===== 配置函数 =====

void FOC_SetMode(FOC_Motor_t *motor, FOC_Mode_t mode)
{
    if (!motor) return;
    motor->mode = mode;
}

void FOC_SetTarget(FOC_Motor_t *motor, float target)
{
    if (!motor) return;
    motor->target = target;
}

void FOC_SetVoltageLimit(FOC_Motor_t *motor, float limit)
{
    if (!motor || limit <= 0.0f) return;
    motor->voltage_limit = limit;
    motor->pid_id.output_limit = limit;
    motor->pid_iq.output_limit = limit;
}

void FOC_SetCurrentLimit(FOC_Motor_t *motor, float limit)
{
    if (!motor || limit <= 0.0f) return;
    motor->current_limit = limit;
    motor->pid_vel.output_limit = limit;
}

void FOC_ConfigMotorRL(FOC_Motor_t *motor, float rs, float ld, float lq, bool use_decoupling)
{
    if (!motor) return;
    motor->rs = rs;
    motor->ld = ld;
    motor->lq = lq;
    motor->use_decoupling = use_decoupling;
}

// ===== 状态获取函数 =====

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

// ===== 默认HAL绑定 =====
void FOC_AttachDefaultHAL(FOC_Motor_t *motor)
{
    if (!motor) return;
    FOC_HAL_t hal = {
        .read_angle = default_board_read_angle,
        .read_currents = default_board_read_currents,
        .set_voltages = default_board_set_voltages,
        .get_bus_voltage = default_board_get_bus_voltage,
    };
    FOC_SetHAL(motor, &hal);
}

void FOC_SetSensorDirection(FOC_Motor_t *motor, int8_t direction)
{
    if (!motor) return;
    motor->sensor_direction = (direction >= 0) ? 1 : -1;
}

void FOC_CalibrateDirection(FOC_Motor_t *motor)
{
    if (!motor || !motor->hal.set_voltages || !motor->hal.read_angle) return;

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

    // 沿着+电角方向小幅转动
    float theta = 0.35f; // ~20deg
    float cos1 = cosf(theta);
    float sin1 = sinf(theta);
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
