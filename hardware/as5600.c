/*
 * as5600.c  --  AS5600 简化驱动（I2C 轮询，超级稳定）
 * 仅面向 FOC：提供机械角度(rad)、速度(rad/s & rpm)、电角度(rad)
 */
#include "as5600.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 内部函数声明 */
#define MAX_AS5600_INSTANCES 4
static AS5600_t *s_instances[MAX_AS5600_INSTANCES] = {0};

static void start_it_read(AS5600_t *enc);
static AS5600_t *find_instance(I2C_HandleTypeDef *hi2c);
static void update_kinematics(AS5600_t *enc, uint16_t raw_angle);

/* ---------- 公开 API 实现 ---------- */

bool AS5600_Init(AS5600_t *enc, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs)
{
    if (!enc || !hi2c) return false;

    *enc = (AS5600_t){
        .hi2c = hi2c,
        .pole_pairs = pole_pairs,
        .last_update_ms = HAL_GetTick(),
        .busy = false,
        // .error_cnt = 0,  // 移除错误计数初始化
    };

    for (int i = 0; i < MAX_AS5600_INSTANCES; ++i) {
        if (s_instances[i] == NULL) {
            s_instances[i] = enc;
            break;
        }
    }

    return true;
}

void AS5600_Process(AS5600_t *enc)
{
    if (!enc) return;

    uint32_t now = HAL_GetTick();
    // 简单超时重置，不累计错误
    if (enc->busy && (now - enc->io_start_ms > 20)) {
        enc->busy = false;
        // 移除 enc->error_cnt++;
    }

    if (!enc->busy && HAL_I2C_GetState(enc->hi2c) == HAL_I2C_STATE_READY) {
        start_it_read(enc);
    }
}

void AS5600_Reset(AS5600_t *enc)
{
    if (!enc) return;
    enc->busy = false;
    // enc->error_cnt = 0;  // 移除错误计数重置
    enc->velocity_rads = 0.0f;
    enc->velocity_rpm = 0.0f;
    enc->last_mech_angle = enc->mech_angle;
}

/* ---------- Getters ---------- */
uint16_t AS5600_GetRawAngle(AS5600_t *enc) { return enc ? enc->raw_angle : 0; }
float AS5600_GetAngleRad(AS5600_t *enc) { return enc ? enc->mech_angle : 0.0f; }
float AS5600_GetVelRad(AS5600_t *enc) { return enc ? enc->velocity_rads : 0.0f; }
float AS5600_GetVelRPM(AS5600_t *enc) { return enc ? enc->velocity_rpm : 0.0f; }
// uint8_t AS5600_GetErrorCount(AS5600_t *enc) { return enc ? enc->error_cnt : 0; }  // 移除错误计数API

float AS5600_GetElecRad(AS5600_t *enc)
{
    if (!enc) return 0.0f;
    float elec_angle = fmodf(enc->mech_angle * enc->pole_pairs - enc->zero_elec_offset, TWO_PI);
    return elec_angle < 0 ? elec_angle + TWO_PI : elec_angle;
}


/* ---------- 内部函数实现 ---------- */

static void start_it_read(AS5600_t *enc)
{
    if (!enc || enc->busy) return;

    enc->busy = true;
    enc->io_start_ms = HAL_GetTick();
    uint8_t reg = AS5600_REG_ANGLE_H;
    HAL_I2C_Mem_Read_IT(enc->hi2c, AS5600_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, enc->i2c_buf, 2);
}

static void update_kinematics(AS5600_t *enc, uint16_t raw_angle)
{
    enc->raw_angle = raw_angle;
    float new_mech_angle = (float)raw_angle * (TWO_PI / 4096.0f);

    uint32_t now_ms = HAL_GetTick();
    float dt = (now_ms - enc->last_update_ms) * 0.001f;

    if (dt > 1e-4f && dt < 0.1f) {
        float angle_diff = new_mech_angle - enc->last_mech_angle;
        if (fabsf(angle_diff) > M_PI) {
            angle_diff = (angle_diff > 0) ? angle_diff - TWO_PI : angle_diff + TWO_PI;
        }
        float vel_raw = angle_diff / dt;
        enc->velocity_rads = 0.12f * vel_raw + 0.88f * enc->velocity_rads; /* LPF */
        enc->velocity_rpm = enc->velocity_rads * 30.0f / M_PI;
    }

    enc->mech_angle = new_mech_angle;
    enc->last_mech_angle = new_mech_angle;
    enc->last_update_ms = now_ms;
}

/* ---------- HAL I2C 回调 ---------- */
static AS5600_t *find_instance(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *candidate = NULL;
    for (int i = 0; i < MAX_AS5600_INSTANCES; ++i) {
        if (s_instances[i] && s_instances[i]->hi2c == hi2c) {
            if (s_instances[i]->busy) {
                return s_instances[i];
            }
            candidate = s_instances[i];
        }
    }
    return candidate;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = find_instance(hi2c);
    if (!enc) return;
    enc->busy = false;
    uint16_t raw = ((uint16_t)enc->i2c_buf[0] << 8 | enc->i2c_buf[1]) & 0x0FFF;
    update_kinematics(enc, raw);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = find_instance(hi2c);
    if (!enc) return;
    enc->busy = false;
    // 移除错误累计，只重置busy状态
    // enc->error_cnt++;
}


