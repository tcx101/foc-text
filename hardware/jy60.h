#ifndef __jy60_H
#define __jy60_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    float temperature;
} JY60_Data_t;

void JY60_Init(void);
bool JY60_GetData(JY60_Data_t *out);

bool JY60_GetAccel(float *ax, float *ay, float *az);
bool JY60_GetGyro(float *gx, float *gy, float *gz);
bool JY60_GetAngle(float *roll, float *pitch, float *yaw);

float JY60_GetAccelX(void);
float JY60_GetAccelY(void);
float JY60_GetAccelZ(void);
float JY60_GetGyroX(void);
float JY60_GetGyroY(void);
float JY60_GetGyroZ(void);
float JY60_GetRoll(void);
float JY60_GetPitch(void);
float JY60_GetYaw(void);
float JY60_GetTemperature(void);

void JY60_Unlock(void);
void JY60_Save(void);
void JY60_SetAccelRangeG(float range_g);

#endif
