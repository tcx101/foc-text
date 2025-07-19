#include "as5600.h"
#include <math.h>
#include <stdint.h>

#ifndef M_2PI_F
#define M_2PI_F 6.28318530717958647692f
#endif

void AS5600_Init(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs)
{
    h->hi2c = hi2c;
    h->pole_pairs = pole_pairs;
    h->raw_angle = 0;
    h->elec_angle = 0.0f;
    h->read_pending = 0;
}

HAL_StatusTypeDef AS5600_RequestAngle_IT(AS5600_Handle_t *h)
{
    // 如果上一次的传输还没完成，则不发起新的请求
    if (h->read_pending) {
        return HAL_BUSY;
    }

    h->read_pending = 1;
    
    // 发起非阻塞中断模式读取请求
    return HAL_I2C_Mem_Read_IT(h->hi2c, AS5600_ADDRESS, AS5600_REG_RAW_ANGLE_H, I2C_MEMADD_SIZE_8BIT, h->rx_buf, 2);
}

void AS5600_RxCpltCallback(AS5600_Handle_t *h)
{
    uint16_t raw = ((uint16_t)h->rx_buf[0] << 8) | h->rx_buf[1];
    h->raw_angle = raw & 0x0FFF;

    h->elec_angle = ((float)h->raw_angle / 4096.0f) * h->pole_pairs * M_2PI_F;

    h->read_pending = 0;
}

void AS5600_ErrorCallback(AS5600_Handle_t *h)
{
    h->read_pending = 0;
    }
    
// 注释或删除所有旧的阻塞式函数
/*
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c) { ... }
...
*/ 
