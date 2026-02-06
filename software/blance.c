#include "blance.h"

vertical vpid;      // 直立环对象
speed_pid spid;     // 速度环对象
IMU imu;            // 陀螺仪对象
float target_angle = 0.0f; // 目标角度（由速度环输出）

/**
 * @brief 直立环初始化
 * @param pid 直立环PD控制器
 * @param kp 比例系数
 * @param kd 微分系数
 * @param target 初始目标角度
 */
void balance_init(vertical *pid, float kp, float kd, float target)
{
    pid->kp = kp;
    pid->kd = kd;
    pid->target = target;
}

/**
 * @brief 速度环初始化
 * @param pid 速度环PI控制器
 * @param kp 比例系数
 * @param ki 积分系数
 * @param target 目标速度
 */
void speed_init(speed_pid *pid, float kp, float ki, float target)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->integral = 0.0f;
    pid->integral_limit = 30.0f; // 积分限幅（角度限制±30度）
    pid->target = target;
}

/**
 * @brief 速度环控制（外环）
 * @param pid 速度环控制器
 * @param current_speed 当前速度（rad/s）
 * @return 目标角度（度）
 * @note 速度环输出目标角度给直立环
 */
float speed_control(speed_pid *pid, float current_speed)
{
    // 计算速度误差
    float error = pid->target - current_speed;
    
    // 积分累积
    pid->integral += error;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // PI控制输出目标角度
    float output_angle = pid->kp * error + pid->ki * pid->integral;
    
    // 角度限幅（±30度）
    if (output_angle > 30.0f) {
        output_angle = 30.0f;
    } else if (output_angle < -30.0f) {
        output_angle = -30.0f;
    }
    
    return output_angle;
}

/**
 * @brief 直立环控制（中环）
 * @param pid 直立环PD控制器
 * @param angle 当前角度（度）
 * @param gyro 当前角速度（度/s）
 * @note 直立环输出目标电流给电流环
 */
void balance_vertical(vertical *pid, float angle, float gyro)
{
    // PD控制计算目标电流
    float err_iq = pid->kp * (angle - pid->target) + pid->kd * gyro;
    
    // 电流限幅
    if (err_iq > motor1.current_limit) {
        err_iq = motor1.current_limit;
    } else if (err_iq < -motor1.current_limit) {
        err_iq = -motor1.current_limit;
    }
    // 设置目标电流（左右电机反向）
    FOC_SetTarget(&motor1, err_iq);
    FOC_SetTarget(&motor2, -err_iq);
}

/**
 * @brief 控制回路定时器中断
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim9)
    {
        FOC_UpdateCurrentLoop(&motor1);
        FOC_UpdateCurrentLoop(&motor2);
    }
    else if (htim == &htim3)
    {
        //1khz编码器加直立环
        AS5600_StartRead(&as5600_l); // 触发编码器读取
        AS5600_StartRead(&as5600_r); 
    }
    else if (htim == &htim5)
    {
        // 速度环（外环）+ 陀螺仪读取 
        imu.roll = JY60_GetRoll();
        imu.gx = JY60_GetGyroX();
    }
}

/**
 * @brief 陀螺仪参数显示
 */
void windowMenu(IMU *imu)
{
    // TODO: 实现显示功能
}
