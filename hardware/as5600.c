/*
 * as5600.c  --  全新中断驱动实现
 */
#include "as5600.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define AS5600_I2C_ADDR (0x36 << 1)
#define AS5600_REG_ANGLE_H 0x0E

/* --- 本地变量 --- */
static AS5600_t *s_instances[2] = {0};   /* 最多支持 I2C1 / I2C3 两只编码器 */

/* 前置声明 */
static void     AS5600_StartRead_IT(AS5600_t *enc);
static AS5600_t *AS5600_FromHi2c(I2C_HandleTypeDef *hi2c);

/* ---------------- API 实现 ---------------- */
bool AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs)
{
    if (!enc || !hi2c) return false;
    
    // 注册实例到静态数组
    if (hi2c->Instance == I2C1) s_instances[0] = enc;
    else if (hi2c->Instance == I2C3) s_instances[1] = enc;
    
    enc->hi2c = hi2c;
    enc->pole_pairs = pole_pairs;
    enc->zero_elec_offset = 0.0f;
    enc->raw_angle = 0;
    enc->prev_raw_angle = 0;
    enc->mech_angle = 0.0f;
    enc->velocity_rads = 0.0f;
    enc->velocity_rpm = 0.0f;
    enc->last_update_ms = HAL_GetTick();
    enc->busy = false;
    enc->error_cnt = 0;
    AS5600_StartRead_IT(enc);
    return true;
}

void AS5600_Process(AS5600_t *enc)
{
    if (!enc) return;
    if (!enc->busy && HAL_I2C_GetState(enc->hi2c) == HAL_I2C_STATE_READY) {
        AS5600_StartRead_IT(enc);
    }
}

void AS5600_Reset(AS5600_t *enc)
{
    if (!enc) return;
    enc->busy = false;
    enc->error_cnt = 0;
    AS5600_StartRead_IT(enc);
}

/* ---- 获取函数 ---- */
uint16_t AS5600_GetRawAngle(AS5600_t *enc) { return enc ? enc->raw_angle : 0; }
float    AS5600_GetAngleRad(AS5600_t *enc) { return enc ? enc->mech_angle : 0.0f; } // 去掉反转角度方向
/* 获取电角度 (rad) */
float AS5600_GetElecRad(AS5600_t *enc)
{
    float elec_angle = fmodf(enc->mech_angle * enc->pole_pairs + enc->zero_elec_offset, TWO_PI);
    
    /* 规范化到 [0, 2π) */
    if (elec_angle < 0) elec_angle += TWO_PI;
    
    return elec_angle;
}

float    AS5600_GetVelRad  (AS5600_t *enc) { return enc ? enc->velocity_rads : 0.0f; } // 顺时针为正
float    AS5600_GetVelRPM  (AS5600_t *enc) { return enc ? enc->velocity_rpm  : 0.0f; }

/* ---------------- 内部实现 ---------------- */
static void AS5600_StartRead_IT(AS5600_t *enc)
{
    if (!enc || enc->busy) return;
    enc->busy = true;
    uint8_t reg = AS5600_REG_ANGLE_H;
    HAL_I2C_Mem_Read_IT(enc->hi2c, AS5600_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, enc->i2c_buf, 2);
}

/* ------ HAL 回调 ------ */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = AS5600_FromHi2c(hi2c);
    if (!enc || enc->hi2c != hi2c) return;
    enc->busy = false;

    // 正确拼接高低字节
    uint16_t value = ((uint16_t)enc->i2c_buf[0] << 8) | enc->i2c_buf[1];
    value &= 0x0FFF; // 12位有效

    // 机械角度
    enc->prev_raw_angle = enc->raw_angle;
    enc->raw_angle = value;
    enc->mech_angle = (float)enc->raw_angle * (TWO_PI / 4096.0f);
    
    // 计算速度
    uint32_t current_time = HAL_GetTick();
    if (enc->last_update_time > 0) {
        uint32_t dt_ms = current_time - enc->last_update_time;
        if (dt_ms > 0) {
            float dt_s = dt_ms / 1000.0f;
            float angle_diff = enc->mech_angle - enc->last_mech_angle;
            
            // 处理角度跳跃（0-2π边界）
            if (angle_diff > M_PI) angle_diff -= TWO_PI;
            else if (angle_diff < -M_PI) angle_diff += TWO_PI;
            
            enc->velocity_rads = angle_diff / dt_s;
            enc->velocity_rpm = enc->velocity_rads * 30.0f / M_PI;
        }
    }
    
    enc->last_mech_angle = enc->mech_angle;
    enc->last_update_time = current_time;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = AS5600_FromHi2c(hi2c);
    if (!enc) return;

    enc->busy = false;
    enc->error_cnt++;
    /* 不在中断里复位 I2C，只计数，交给 Process 处理 */
    }
    
/* ----------------- 辅助函数 ----------------- */
static AS5600_t *AS5600_FromHi2c(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) return s_instances[0];
    if (hi2c->Instance == I2C3) return s_instances[1];
    return NULL;
}
