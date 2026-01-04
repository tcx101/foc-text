/*
 * as5600.c  --  AS5600 优化驱动(I2C中断,增强错误恢复)
 * 面向FOC高实时性应用：机械角度(rad)、速度(rad/s & rpm)、电角度(rad)
 * 优化点：智能重试机制、I2C总线恢复、更快的错误检测
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
static void recover_i2c_bus(AS5600_t *enc);
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
        .retry_count = 0,
        .need_recovery = false,
        .recovery_state = 0,
        .recovery_start_ms = 0,
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

    // 检测超时并触发恢复
    if (enc->busy && (now - enc->io_start_ms > AS5600_TIMEOUT_MS)) {
        enc->busy = false;
        enc->retry_count++;

        // 超过最大重试次数,标记需要总线恢复
        if (enc->retry_count >= AS5600_MAX_RETRY) {
            enc->need_recovery = true;
            enc->retry_count = 0;
        }
    }

    // 执行I2C总线恢复(如果需要) - 非阻塞状态机,需要多次调用
    if (enc->need_recovery || enc->recovery_state != 0) {
        recover_i2c_bus(enc);
        return;  // 恢复期间不进行读取
    }

    // 确保本实例不忙 且 I2C总线完全空闲
    HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(enc->hi2c);
    if (!enc->busy && i2c_state == HAL_I2C_STATE_READY) {
        start_it_read(enc);
    }
}

void AS5600_Reset(AS5600_t *enc)
{
    if (!enc) return;

    // 重置所有状态
    enc->busy = false;
    enc->retry_count = 0;
    enc->need_recovery = false;
    enc->recovery_state = 0;
    enc->recovery_start_ms = 0;
    enc->velocity_rads = 0.0f;
    enc->velocity_rpm = 0.0f;
    enc->last_mech_angle = enc->mech_angle;

    // 强制触发I2C总线恢复(非阻塞)
    enc->need_recovery = true;
    enc->recovery_state = 0;
}

/* ---------- Getters ---------- */
uint16_t AS5600_GetRawAngle(AS5600_t *enc) { return enc ? enc->raw_angle : 0; }
float AS5600_GetAngleRad(AS5600_t *enc) { return enc ? enc->mech_angle : 0.0f; }
float AS5600_GetVelRad(AS5600_t *enc) { return enc ? enc->velocity_rads : 0.0f; }
float AS5600_GetVelRPM(AS5600_t *enc) { return enc ? enc->velocity_rpm : 0.0f; }
bool AS5600_IsBusy(AS5600_t *enc) { return enc ? enc->busy : false; }

float AS5600_GetElecRad(AS5600_t *enc)
{
    if (!enc) return 0.0f;
    float elec_angle = fmodf(enc->mech_angle * enc->pole_pairs - enc->zero_elec_offset, TWO_PI);
    return elec_angle < 0 ? elec_angle + TWO_PI : elec_angle;
}


/* ---------- 内部函数实现 ---------- */

/**
 * @brief I2C总线恢复函数 (非阻塞状态机版本)
 * @note 当连续超时后,执行总线恢复操作,完全非阻塞,适合在中断中调用
 */
static void recover_i2c_bus(AS5600_t *enc)
{
    if (!enc || !enc->hi2c) return;

    uint32_t now = HAL_GetTick();
    HAL_I2C_StateTypeDef state;

    switch (enc->recovery_state) {
        case 0:  // 初始状态:启动abort
            state = HAL_I2C_GetState(enc->hi2c);
            if (state != HAL_I2C_STATE_READY) {
                HAL_I2C_Master_Abort_IT(enc->hi2c, AS5600_I2C_ADDR);
                enc->recovery_state = 1;
                enc->recovery_start_ms = now;
            } else {
                // 已经就绪,直接完成恢复
                enc->recovery_state = 0;
                enc->need_recovery = false;
            }
            break;

        case 1:  // 等待abort完成 (等待1ms)
            if (now - enc->recovery_start_ms >= 1) {
                state = HAL_I2C_GetState(enc->hi2c);
                if (state != HAL_I2C_STATE_READY) {
                    // 仍然不正常,执行DeInit
                    HAL_I2C_DeInit(enc->hi2c);
                    enc->recovery_state = 2;
                    enc->recovery_start_ms = now;
                } else {
                    // abort成功,恢复完成
                    enc->recovery_state = 0;
                    enc->need_recovery = false;
                }
            }
            break;

        case 2:  // 等待deinit稳定 (等待2ms)
            if (now - enc->recovery_start_ms >= 2) {
                // 重新初始化I2C
                HAL_I2C_Init(enc->hi2c);
                enc->recovery_state = 3;
                enc->recovery_start_ms = now;
            }
            break;

        case 3:  // 等待init完成 (等待1ms)
            if (now - enc->recovery_start_ms >= 1) {
                // 恢复完成
                enc->recovery_state = 0;
                enc->need_recovery = false;
            }
            break;

        default:
            // 异常状态,重置
            enc->recovery_state = 0;
            enc->need_recovery = false;
            break;
    }
}

static void start_it_read(AS5600_t *enc)
{
    if (!enc || enc->busy) return;

    enc->busy = true;
    enc->io_start_ms = HAL_GetTick();

    // 启动中断读取
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(enc->hi2c, AS5600_I2C_ADDR,
                                                     AS5600_REG_ANGLE_H,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     enc->i2c_buf, 2);

    // 如果启动失败,清除busy标志并增加重试计数
    if (status != HAL_OK) {
        enc->busy = false;
        enc->retry_count++;
        if (enc->retry_count >= AS5600_MAX_RETRY) {
            enc->need_recovery = true;
            enc->retry_count = 0;
        }
    }
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
    for (int i = 0; i < MAX_AS5600_INSTANCES; ++i) {
        if (s_instances[i] && s_instances[i]->hi2c == hi2c) {
            return s_instances[i];
        }
    }
    return NULL;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = find_instance(hi2c);
    if (!enc || !enc->busy) return;

    enc->busy = false;
    enc->retry_count = 0;  // 成功读取,清零重试计数

    uint16_t raw = ((uint16_t)enc->i2c_buf[0] << 8 | enc->i2c_buf[1]) & 0x0FFF;
    update_kinematics(enc, raw);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    AS5600_t *enc = find_instance(hi2c);
    if (!enc || !enc->busy) return;

    enc->busy = false;
    enc->retry_count++;

    // 连续错误触发总线恢复
    if (enc->retry_count >= AS5600_MAX_RETRY) {
        enc->need_recovery = true;
        enc->retry_count = 0;
    }
}


