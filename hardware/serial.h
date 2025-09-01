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








#endif
