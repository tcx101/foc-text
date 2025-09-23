#ifndef BLANCE_H
#define BLANCE_H
//包含头文件
#include "Allfile.h"
typedef struct 
{
    float kp;           // 比例增益
    float kd;           // 微分增益
    float target;      // 目标值
}vertical;

typedef struct 
{
    float ax,ay,az;
    float gx,gy,gz;
    float roll,yaw,pitch;
    float temp;
}IMU;

extern IMU imu;//陀螺仪对象

//直立环参数设定函数
void blance_init(vertical*pid, float kp, float kd);
//直立环函数
double blance_vertical(vertical*pid, float Angle, float gx);
//控制回路及传感器数据获取
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//陀螺仪参数显示
void windowMenu (IMU*imu);

#endif
