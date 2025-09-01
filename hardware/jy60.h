#ifndef __jy60_H
#define __jy60_H
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "stm32f4xx_hal.h"  /* 如使用其他系列，请替换为对应 HAL 头文件 */
#include <stdbool.h>
#include <stdint.h>

/* -------------------------------- 数据结构 -------------------------------- */
typedef struct
{
    float ax, ay, az;   /* m/s^2 */
    float gx, gy, gz;   /* deg/s */
    float roll, pitch, yaw; /* deg */
    float temperature;  /* °C */
} JY60DMA_Data_t;

/* -------------------------------- API -------------------------------- */
/* 启动 huart3 DMA + 空闲中断接收并初始化解析器 */
void JY60DMA_Init(void);

/* 获取最新一次完整数据 */
bool JY60DMA_GetData(JY60DMA_Data_t *out);

/* 便捷读取接口 */
bool JY60DMA_GetAccel(float *ax, float *ay, float *az);
bool JY60DMA_GetGyro(float *gx, float *gy, float *gz);
bool JY60DMA_GetAngle(float *roll, float *pitch, float *yaw);
float JY60DMA_GetAccelX(void);
float JY60DMA_GetAccelY(void);
float JY60DMA_GetAccelZ(void);
float JY60DMA_GetGyroX(void);
float JY60DMA_GetGyroY(void);
float JY60DMA_GetGyroZ(void);
float JY60DMA_GetRoll(void);
float JY60DMA_GetPitch(void);
float JY60DMA_GetYaw(void);
float JY60DMA_GetTemperature(void);

#endif

