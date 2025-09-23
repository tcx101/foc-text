
#ifndef AS5600_H
#define AS5600_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define TWO_PI (2.0f * 3.14159265358979323846f)

/* 寄存器地址 */
#define AS5600_I2C_ADDR            (0x36 << 1)  /* 0x36 7bit, HAL 要求左移一位 */
#define AS5600_REG_ANGLE_H         0x0E
#define AS5600_REG_ANGLE_L         0x0F

/* 编码器数据结构 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_buf[2];
    uint16_t raw_angle;
    uint16_t prev_raw_angle;
    float mech_angle;      // 机械角度（弧度，0~2π）
    float velocity_rads;   // 速度（rad/s）
    float velocity_rpm;    // 速度（rpm）
    uint32_t last_update_ms;
    uint32_t last_read_start_ms;  // 记录最后一次开始读取的时间
    bool busy;
    uint8_t error_cnt;
    uint8_t consecutive_errors;   // 连续错误计数
    uint8_t pole_pairs;
    float zero_elec_offset;
    float last_mech_angle;
} AS5600_t;

/* ---- 对外 API ---- */
bool AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs);
void AS5600_Process(AS5600_t *enc);
void AS5600_Reset(AS5600_t *enc);
uint8_t AS5600_GetErrorCount(AS5600_t *enc);
float AS5600_GetAngleRad(AS5600_t *enc);
float AS5600_GetVelRad(AS5600_t *enc);

/* 数据读取函数 */
uint16_t AS5600_GetRawAngle (AS5600_t *enc);

/* 获取电角度 (rad) */
float    AS5600_GetElecRad  (AS5600_t *enc);

float    AS5600_GetVelRPM   (AS5600_t *enc);

/* -------- 兼容旧版函数名（保持向后兼容） -------- */
#define AS5600_GetAngle(enc)         AS5600_GetAngleRad(enc)
#define AS5600_GetVelocityRPM(enc)   AS5600_GetVelRPM(enc)

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */ 
