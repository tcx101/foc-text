#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "usart.h"
typedef struct 
{
    float ax,ay,az;
    float gx,gy,gz;
    float roll,yaw,pitch;
    float temp;
}IMU;

extern IMU imu;

/**
 * @brief 打印日志信息（使用printf实现）
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void print_log(const char *format, ...);

#endif
