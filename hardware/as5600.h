#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define TWO_PI (2.0f * 3.14159265358979323846f)

/* I2C 地址与寄存器 */
#define AS5600_I2C_ADDR (0x36 << 1) /* 7-bit address is 0x36 */
#define AS5600_REG_ANGLE_H 0x0E
#define AS5600_TIMEOUT_MS 50        /* I2C超时时间 (ms) - 增加到50ms以应对高负载 */
#define AS5600_MAX_RETRY 5          /* 最大重试次数 - 增加到5次 */

/* 编码器实例结构体 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;

    /* 运动学参数 */
    uint16_t raw_angle;
    float mech_angle;
    float velocity_rads;
    float velocity_rpm;

    /* 内部状态 */
    float last_mech_angle;
    uint32_t last_update_ms;
    uint8_t i2c_buf[2];
    bool busy;
    uint32_t io_start_ms;
    uint8_t retry_count;    /* 当前重试计数 */
    bool need_recovery;     /* 是否需要总线恢复 */

    /* 非阻塞恢复状态机 */
    uint8_t recovery_state; /* 0=空闲, 1=等待abort, 2=等待deinit, 3=等待init */
    uint32_t recovery_start_ms; /* 恢复开始时间戳 */

    /* FOC 相关 */
    uint8_t pole_pairs;
    float zero_elec_offset;
} AS5600_t;

/* 公开 API */
bool AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs);
void AS5600_Process(AS5600_t *enc);
void AS5600_Reset(AS5600_t *enc);

uint16_t AS5600_GetRawAngle(AS5600_t *enc);
float AS5600_GetAngleRad(AS5600_t *enc);
float AS5600_GetVelRad(AS5600_t *enc);
float AS5600_GetVelRPM(AS5600_t *enc);
float AS5600_GetElecRad(AS5600_t *enc);
bool AS5600_IsBusy(AS5600_t *enc);

/* 兼容旧版别名 */
#define AS5600_GetAngle(enc) AS5600_GetAngleRad(enc)

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */
