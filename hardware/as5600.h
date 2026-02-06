#ifndef AS5600_H
#define AS5600_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define TWO_PI (2.0f * 3.14159265358979323846f)
#define M_PI 3.14159265358979323846f

#define AS5600_I2C_ADDR (0x36 << 1)
#define AS5600_REG_ANGLE_H 0x0E

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t raw_angle;
    uint8_t dma_buf[2];
    float mech_angle;
    float velocity_rads;
    float velocity_rpm;
    float last_angle;
    uint32_t last_time_us;
    bool busy;
    uint8_t pole_pairs;
    float zero_elec_offset;

    // ⭐ 新增：预计算的电角度（在编码器中断中更新）
    float elec_angle;           // 电角度（rad），在编码器中断中计算
    int8_t sensor_direction;    // 传感器方向（+1或-1）
} AS5600_t;

void AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs);
void AS5600_StartRead(AS5600_t *enc);

uint16_t AS5600_GetRawAngle(AS5600_t *enc);
float AS5600_GetAngleRad(AS5600_t *enc);
float AS5600_GetVelRad(AS5600_t *enc);
float AS5600_GetVelRPM(AS5600_t *enc);
float AS5600_GetElecRad(AS5600_t *enc);

// ⭐ 新增：获取预计算的电角度（在编码器中断中已计算好）
float AS5600_GetPrecomputedElecAngle(AS5600_t *enc);

#endif
