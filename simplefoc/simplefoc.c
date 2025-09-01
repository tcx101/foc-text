/*
 * SimpleFOC - 重新设计的FOC控制库实现
 * 正确的数学变换，清晰的控制逻辑
 */
#include "simplefoc.h"
#include <math.h>
#include <string.h>
#include "tim.h"

// 内部辅助函数
static float normalize_angle(float angle);
static void clarke_transform(float a, float b, float c, float *alpha, float *beta);
static void park_transform(float alpha, float beta, float theta, float *d, float *q);
static void inv_park_transform(float d, float q, float theta, float *alpha, float *beta);
static void space_vector_pwm(float alpha, float beta, float vdc, float *va, float *vb, float *vc);
static float pid_update(FOC_PID_t *pid, float error, float dt);

// 角度标准化到[-π, π]
static float normalize_angle(float angle) {
    while (angle > FOC_PI) angle -= FOC_2PI;
    while (angle < -FOC_PI) angle += FOC_2PI;
    return angle;
}

// Clarke变换 (3相 -> 2相αβ)
static void clarke_transform(float a, float b, float c, float *alpha, float *beta) {
    (void)c;
    *alpha = a;
    *beta  = (a + 2.0f * b) * (1.0f / FOC_SQRT3);
}

// Park变换 (αβ -> dq)
static void park_transform(float alpha, float beta, float theta, float *d, float *q) {
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

// 反Park变换 (dq -> αβ)
static void inv_park_transform(float d, float q, float theta, float *alpha, float *beta) {
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

// 空间矢量PWM
static void space_vector_pwm(float alpha, float beta, float vdc, float *va, float *vb, float *vc) {
    // 在线性区限幅：|V_alpha_beta| <= Vdc / sqrt(3)
    float vref = sqrtf(alpha * alpha + beta * beta);
    float vmax_alpha_beta = vdc * FOC_INV_SQRT3;
    if (vref > vmax_alpha_beta && vref > 1e-6f) {
        float scale = vmax_alpha_beta / vref;
        alpha *= scale;
        beta  *= scale;
    }

    // 计算三相参考电压
    float v_a = alpha;
    float v_b = -0.5f * alpha + FOC_SQRT3_2 * beta;
    float v_c = -0.5f * alpha - FOC_SQRT3_2 * beta;
    
    // 找到最大值和最小值
    float v_max = v_a;
    if (v_b > v_max) v_max = v_b;
    if (v_c > v_max) v_max = v_c;
    
    float v_min = v_a;
    if (v_b < v_min) v_min = v_b;
    if (v_c < v_min) v_min = v_c;
    
    // 计算零序分量使电压居中
    float v_offset = 0.5f * vdc - 0.5f * (v_max + v_min);
    
    *va = v_a + v_offset;
    *vb = v_b + v_offset;
    *vc = v_c + v_offset;
    
    // 限制电压范围
    if (*va > vdc) *va = vdc;
    if (*va < 0) *va = 0;
    if (*vb > vdc) *vb = vdc;
    if (*vb < 0) *vb = 0;
    if (*vc > vdc) *vc = vdc;
    if (*vc < 0) *vc = 0;
}

// PID控制器更新
static float pid_update(FOC_PID_t *pid, float error, float dt) {
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // 微分项
    float d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    
    // 输出
    float output = p_term + i_term + d_term;
    
    // 输出限制和积分抗饱和 (修正反向回算符号)
    if (output > pid->output_limit) {
        output = pid->output_limit;
        if (pid->ki > 0) pid->integral += (output - (p_term + i_term + d_term)) / pid->ki;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
        if (pid->ki > 0) pid->integral += (output - (p_term + i_term + d_term)) / pid->ki;
    }
    
    return output;
}

// 初始化FOC电机
bool FOC_Init(FOC_Motor_t *motor, uint8_t pole_pairs) {
    if (!motor || pole_pairs == 0) return false;
    
    memset(motor, 0, sizeof(FOC_Motor_t));
    
    motor->pole_pairs = pole_pairs;
    motor->voltage_limit = 12.0f;  // 默认12V
    motor->current_limit = 2.0f;   // 默认2A
    motor->mode = FOC_MODE_TORQUE;
    
    // 默认电机等效参数
    motor->rs = 0.0f;
    motor->ld = 0.0f;
    motor->lq = 0.0f;
    motor->use_decoupling = false;
    
    // 默认PID参数
    motor->pid_vel.kp = 0.5f;
    motor->pid_vel.ki = 10.0f;
    motor->pid_vel.kd = 0.0f;
    motor->pid_vel.output_limit = motor->current_limit;
    
    motor->pid_pos.kp = 20.0f;
    motor->pid_pos.ki = 0.0f;
    motor->pid_pos.kd = 0.1f;
    motor->pid_pos.output_limit = 50.0f; // rad/s
    
    // 内部电流控制器 (Id, Iq) - 优化参数
    motor->pid_id.kp = 1.5f;
    motor->pid_id.ki = 40.0f;
    motor->pid_id.kd = 0.0f;
    motor->pid_id.output_limit = motor->voltage_limit;
    
    motor->pid_iq.kp = 1.5f;
    motor->pid_iq.ki = 40.0f;
    motor->pid_iq.kd = 0.0f;
    motor->pid_iq.output_limit = motor->voltage_limit;
    
    motor->initialized = true;
    return true;
}

// 设置硬件接口
void FOC_SetHAL(FOC_Motor_t *motor, FOC_HAL_t *hal) {
    if (!motor || !hal) return;
    motor->hal = *hal;
    
    // 初始化angle_prev为当前角度，避免第一次速度计算错误
    if (motor->hal.read_angle) {
        motor->angle_prev = motor->hal.read_angle();
    }
}

// 设置控制模式
void FOC_SetMode(FOC_Motor_t *motor, FOC_Mode_t mode) {
    if (!motor) return;
    motor->mode = mode;
    
    // 清零PID积分器
    motor->pid_vel.integral = 0;
    motor->pid_vel.prev_error = 0;
    motor->pid_pos.integral = 0;
    motor->pid_pos.prev_error = 0;
    motor->pid_id.integral = 0;
    motor->pid_id.prev_error = 0;
    motor->pid_iq.integral = 0;
    motor->pid_iq.prev_error = 0;
}

// 设置目标值
void FOC_SetTarget(FOC_Motor_t *motor, float target) {
    if (!motor) return;
    motor->target = target;
}

// 设置电流限制
void FOC_SetCurrentLimit(FOC_Motor_t *motor, float limit) {
    if (!motor || limit <= 0) return;
    motor->current_limit = limit;
    motor->pid_vel.output_limit = limit;
}

// 设置电压限制
void FOC_SetVoltageLimit(FOC_Motor_t *motor, float limit) {
    if (!motor || limit <= 0) return;
    motor->voltage_limit = limit;
    motor->pid_id.output_limit = limit;
    motor->pid_iq.output_limit = limit;
}

// 配置速度PID
void FOC_ConfigPID_Velocity(FOC_Motor_t *motor, float kp, float ki, float kd, float limit) {
    if (!motor) return;
    motor->pid_vel.kp = kp;
    motor->pid_vel.ki = ki;
    motor->pid_vel.kd = kd;
    motor->pid_vel.output_limit = limit;
}

// 配置位置PID
void FOC_ConfigPID_Position(FOC_Motor_t *motor, float kp, float ki, float kd, float limit) {
    if (!motor) return;
    motor->pid_pos.kp = kp;
    motor->pid_pos.ki = ki;
    motor->pid_pos.kd = kd;
    motor->pid_pos.output_limit = limit;
}

// 配置电机等效参数（Rs/Ld/Lq）并选择是否开启去耦
void FOC_ConfigMotorRL(FOC_Motor_t *motor, float rs, float ld, float lq, bool enable_decoupling) {
    if (!motor) return;
    motor->rs = rs;
    motor->ld = ld;
    motor->lq = lq;
    motor->use_decoupling = enable_decoupling;
}

// 主控制更新函数
void FOC_Update(FOC_Motor_t *motor) {
    if (!motor || !motor->initialized) return;
    
    static float s_control_dt = 0.0f;
    if (s_control_dt <= 0.0f) {
        s_control_dt = 1.0f / 20000.0f;
    }
    const float dt = s_control_dt; // 实际控制周期
    // 动态计算线性工作区电压上限（dq向量幅值最大值）
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    float v_limit = vdc * FOC_INV_SQRT3;
    // 将电流环PID的输出上限与SVPWM线性范围一致，避免“隐藏饱和”
    motor->pid_id.output_limit = v_limit;
    motor->pid_iq.output_limit = v_limit;
    
    // 1. 读取传感器数据
    if (motor->hal.read_angle) {
        motor->angle_mech = motor->hal.read_angle();
    }
    
    if (motor->hal.read_currents) {
        motor->hal.read_currents(&motor->ia, &motor->ib, &motor->ic);
    }
    
    // 2. 计算速度 (修复角度跳变问题)
    float angle_diff = motor->angle_mech - motor->angle_prev;
    // 处理角度跳变：如果差值过大，说明发生了跳变
    if (angle_diff > FOC_PI) {
        angle_diff -= FOC_2PI;
    } else if (angle_diff < -FOC_PI) {
        angle_diff += FOC_2PI;
    }
    // 使用一阶低通滤波的速度估计，避免I2C角度更新稀疏导致的尖峰
    static float s_velocity_filtered = 0.0f;
    const float vel_alpha = 0.2f; // 0..1，越大响应越快
    float raw_velocity = angle_diff / dt;
    s_velocity_filtered += vel_alpha * (raw_velocity - s_velocity_filtered);
    motor->velocity = s_velocity_filtered;
    motor->angle_prev = motor->angle_mech;
    
    // 3. 计算电角度
    motor->angle_elec = normalize_angle(motor->angle_mech * motor->pole_pairs - motor->elec_zero_offset);
    
    // 4. Clarke和Park变换
    float alpha, beta;
    clarke_transform(motor->ia, motor->ib, motor->ic, &alpha, &beta);
    park_transform(alpha, beta, motor->angle_elec, &motor->id, &motor->iq);
    
    // 5. 控制环计算
    float target_iq = 0;
    
    switch (motor->mode) {
        case FOC_MODE_TORQUE:
            target_iq = motor->target;
            break;
            
        case FOC_MODE_VELOCITY: {
            float vel_error = motor->target - motor->velocity;
            target_iq = pid_update(&motor->pid_vel, vel_error, dt);
            break;
        }
        
        case FOC_MODE_POSITION: {
            float pos_error = normalize_angle(motor->target - motor->angle_mech);
            float target_vel = pid_update(&motor->pid_pos, pos_error, dt);
            float vel_error = target_vel - motor->velocity;
            target_iq = pid_update(&motor->pid_vel, vel_error, dt);
            break;
        }
    }
    
    // 6. 电流限制
    if (target_iq > motor->current_limit) target_iq = motor->current_limit;
    if (target_iq < -motor->current_limit) target_iq = -motor->current_limit;
    
    // 7. 电流环PI控制
    float id_error = 0.0f - motor->id;  // Id目标为0
    float iq_error = target_iq - motor->iq;
    
    motor->vd = pid_update(&motor->pid_id, id_error, dt);
    motor->vq = pid_update(&motor->pid_iq, iq_error, dt);
    
    // 7.5 Rs与交轴去耦/前馈补偿（可选）
    if (motor->use_decoupling) {
        float omega_e = motor->velocity * motor->pole_pairs; // 电角速度(rad/s)
        // PMSM标准模型：
        // v_d = R_s i_d - ω_e L_q i_q
        // v_q = R_s i_q + ω_e L_d i_d (+ ω_e ψ_f)
        motor->vd += motor->rs * motor->id - omega_e * motor->lq * motor->iq;
        motor->vq += motor->rs * motor->iq + omega_e * motor->ld * motor->id;
    }
    
    // 8. 电压限制（按dq向量幅值限幅到SVPWM线性范围）
    float v_mag = sqrtf(motor->vd * motor->vd + motor->vq * motor->vq);
    if (v_mag > v_limit) {
        float scale = v_limit / v_mag;
        motor->vd *= scale;
        motor->vq *= scale;
    }
    
    // 9. 反Park和SVPWM
    inv_park_transform(motor->vd, motor->vq, motor->angle_elec, &alpha, &beta);
    
    space_vector_pwm(alpha, beta, vdc, &motor->va, &motor->vb, &motor->vc);
    
    // 10. 输出PWM
    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(motor->va, motor->vb, motor->vc);
    }
}

// 校准电角度零点
void FOC_CalibrateZeroOffset(FOC_Motor_t *motor) {
    if (!motor) return;
    
    // 采用SVPWM在θ=0方向注入纯D轴电压，等待转子锁定后读角度
    if (motor->hal.set_voltages && motor->hal.read_angle) {
        // 选择注入幅值：母线电压的20%左右，避免过流
        float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
        float vd_inject = 0.2f * vdc;
        
        // 在θ=0上电保持，等待机械对准
        FOC_OpenLoop_SVPWM(motor, vd_inject, 0.0f, 0.0f);
        for (volatile int i = 0; i < 500000; i++); // 简单等待一段时间
        
        // 多次采样取平均，降低抖动
        float sum = 0.0f;
        const int samples = 32;
        for (int i = 0; i < samples; i++) {
            sum += motor->hal.read_angle();
            for (volatile int d = 0; d < 20000; d++);
        }
        float calib_angle = sum / (float)samples;
        
        // 目标是使θ_e = p*θ_m - offset 在当前对准时为0 → offset = p*θ_m
        motor->elec_zero_offset = calib_angle * motor->pole_pairs;
        
        // 关闭输出
        FOC_OpenLoop_SVPWM(motor, 0.0f, 0.0f, 0.0f);
        for (volatile int i = 0; i < 50000; i++);
    }
}

// 开环测试/调试辅助：SVPWM 映射并输出
void FOC_OpenLoop_SVPWM(FOC_Motor_t *motor, float vd, float vq, float theta) {
    if (!motor || !motor->initialized) return;
    float alpha, beta;
    inv_park_transform(vd, vq, theta, &alpha, &beta);
    float vdc = motor->hal.get_bus_voltage ? motor->hal.get_bus_voltage() : 12.0f;
    float va, vb, vc;
    space_vector_pwm(alpha, beta, vdc, &va, &vb, &vc);
    if (motor->hal.set_voltages) {
        motor->hal.set_voltages(va, vb, vc);
    }
}

// 获取状态函数
float FOC_GetAngle(FOC_Motor_t *motor) {
    return motor ? motor->angle_mech : 0.0f;
}

float FOC_GetVelocity(FOC_Motor_t *motor) {
    return motor ? motor->velocity : 0.0f;
}

float FOC_GetCurrent_D(FOC_Motor_t *motor) {
    return motor ? motor->id : 0.0f;
}

float FOC_GetCurrent_Q(FOC_Motor_t *motor) {
    return motor ? motor->iq : 0.0f;
} 
