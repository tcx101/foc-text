/*
 * as5600.c - AS5600磁编码器驱动 (DMA+中断)
 */
#include "as5600.h"
#include <math.h>

#define MAX_INSTANCES 4
static AS5600_t *g_instances[MAX_INSTANCES] = {0};

static void calculate_velocity(AS5600_t *enc);
static AS5600_t *find_instance(I2C_HandleTypeDef *hi2c);

void AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs)
{
    if (!enc || !hi2c) return;
    
    enc->hi2c = hi2c;
    enc->pole_pairs = pole_pairs;
    enc->raw_angle = 0;
    enc->mech_angle = 0.0f;
    enc->velocity_rads = 0.0f;
    enc->velocity_rpm = 0.0f;
    enc->last_angle = 0.0f;
    enc->last_time_us = 0;
    enc->busy = false;
    enc->zero_elec_offset = 0.0f;
    
    for (int i = 0; i < MAX_INSTANCES; i++) {
        if (g_instances[i] == NULL) {
            g_instances[i] = enc;
            break;
        }
    }
}

/**
 * @brief 启动DMA读取
 */
void AS5600_StartRead(AS5600_t *enc)
{
    if (!enc) return;
    
    // 如果上次传输超时（busy标志超过10ms），强制复位
    static uint32_t last_start_time = 0;
    uint32_t now = HAL_GetTick();
    if (enc->busy && (now - last_start_time > 10)) {
        enc->busy = false;  // 超时复位
        HAL_I2C_Master_Abort_IT(enc->hi2c, AS5600_I2C_ADDR);  // 中止传输
    }
    
    if (enc->busy) return;  // 仍在传输中
    
    last_start_time = now;
    if (HAL_I2C_Mem_Read_DMA(enc->hi2c, AS5600_I2C_ADDR, 
                             AS5600_REG_ANGLE_H, I2C_MEMADD_SIZE_8BIT,
                             enc->dma_buf, 2) == HAL_OK) {
        enc->busy = true;
    } else {
        // DMA启动失败，复位busy标志
        enc->busy = false;
    }
}

/**
 * @brief 计算速度
 */
static void calculate_velocity(AS5600_t *enc)
{
    uint32_t now_us = HAL_GetTick() * 1000;
    
    if (enc->last_time_us == 0) {
        enc->last_time_us = now_us;
        enc->last_angle = enc->mech_angle;
        return;
    }
    
    float dt = (now_us - enc->last_time_us) * 1e-6f;
    
    if (dt > 0.001f && dt < 0.1f) {
        float angle_diff = enc->mech_angle - enc->last_angle;
        
        if (angle_diff > M_PI) {
            angle_diff -= TWO_PI;
        } else if (angle_diff < -M_PI) {
            angle_diff += TWO_PI;
        }
        
        float vel = angle_diff / dt;
        enc->velocity_rads = 0.3f * vel + 0.7f * enc->velocity_rads;
        enc->velocity_rpm = enc->velocity_rads * 30.0f / M_PI;
    }
    
    enc->last_angle = enc->mech_angle;
    enc->last_time_us = now_us;
}

uint16_t AS5600_GetRawAngle(AS5600_t *enc)
{
    return enc ? enc->raw_angle : 0;
}

float AS5600_GetAngleRad(AS5600_t *enc)
{
    return enc ? enc->mech_angle : 0.0f;
}

float AS5600_GetVelRad(AS5600_t *enc)
{
    return enc ? enc->velocity_rads : 0.0f;
}

float AS5600_GetVelRPM(AS5600_t *enc)
{
    return enc ? enc->velocity_rpm : 0.0f;
}

float AS5600_GetElecRad(AS5600_t *enc)
{
    if (!enc) return 0.0f;
    
    float elec_angle = fmodf(enc->mech_angle * enc->pole_pairs - enc->zero_elec_offset, TWO_PI);
    return elec_angle < 0 ? elec_angle + TWO_PI : elec_angle;
}

static AS5600_t *find_instance(I2C_HandleTypeDef *hi2c)
{
    for (int i = 0; i < MAX_INSTANCES; i++) {
        if (g_instances[i] && g_instances[i]->hi2c == hi2c) {
            return g_instances[i];
        }
    }
    return NULL;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = find_instance(hi2c);
    if (!enc) return;
    
    enc->busy = false;
    enc->raw_angle = ((uint16_t)enc->dma_buf[0] << 8 | enc->dma_buf[1]) & 0x0FFF;
    enc->mech_angle = (float)enc->raw_angle * (TWO_PI / 4096.0f);
    calculate_velocity(enc);
}

/**
 * @brief I2C错误回调
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = find_instance(hi2c);
    if (!enc) return;
    
    // 发生错误，复位busy标志，允许下次重试
    enc->busy = false;
    
    // 清除I2C错误标志
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
}
