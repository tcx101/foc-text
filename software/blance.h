#ifndef BLANCE_H
#define BLANCE_H
//包含头文件
#include "Allfile.h"

// 直立环PD控制器
typedef struct
{
    float kp;           // 比例增益
    float kd;           // 微分增益
    float target;       // 目标角度
} vertical;

// 速度环PI控制器
typedef struct 
{
    float kp;           // 比例增益
    float ki;           // 积分增益
    float integral;     // 积分累积
    float integral_limit; // 积分限幅
    float target;       // 目标速度
} speed_pid;

// 陀螺仪数据
typedef struct 
{
    float ax, ay, az;
    float gx, gy, gz;
    float roll, yaw, pitch;
    float temp;
} IMU;

extern IMU imu;                 // 陀螺仪对象
extern vertical vpid;           // 直立环对象
extern speed_pid spid;          // 速度环对象
extern float target_angle;      // 目标角度（由速度环输出）
extern I2C_HandleTypeDef hi2c3;
extern FOC_Motor_t motor1;
extern FOC_Motor_t motor2;
extern AS5600_t as5600_l;
extern AS5600_t as5600_r;
extern float vel_left, vel_right ; // 左右轮速度
extern float vel ; // 平均速度
// 直立环初始化
void balance_init(vertical *pid, float kp, float kd, float target);
// 速度环初始化
void speed_init(speed_pid *pid, float kp, float ki, float target);
// 直立环控制（输出目标电流）
void balance_vertical(vertical *pid, float angle, float gyro);
// 速度环控制（输出目标角度）
float speed_control(speed_pid *pid, float current_speed);
// 控制回路
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
// 陀螺仪参数显示
void windowMenu(IMU *imu);

#endif
