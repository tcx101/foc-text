#ifndef FOC_CORE_H
#define FOC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "alg/clarke_park.h"
#include "alg/svpwm.h"
#include "alg/pi.h"
#include "hal/foc_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------- 控制模式 -------------------- */
typedef enum {
    FOC_MODE_DISABLED = 0,
    FOC_MODE_CURRENT,   /* Id / Iq 电流环 */
    FOC_MODE_VELOCITY,  /* 速度环 (外环) */
    FOC_MODE_POSITION   /* 位置环 (最外环) */
} FOC_Mode_t;

/* ------------------- 电机参数（由用户生成） -------------------- */
typedef struct {
    uint8_t pole_pairs;      /* 极对数 */
    float   v_bus_nominal;   /* 额定母线电压 */
    float   current_limit;   /* 保护用最大电流(A) */
} MotorParam_t;

/* ------------------- FOC 运行时句柄 -------------------- */
typedef struct {
    /* ---- 配置 ---- */
    uint8_t id;                  /* 0 / 1 */
    const MotorParam_t *motor;   /* 指向只读电机参数 */

    /* ---- 目标 ---- */
    float id_ref;
    float iq_ref;
    float vel_ref_rpm;
    float pos_ref_rad;

    /* ---- 反馈 ---- */
    Phase_Currents_t   phase_cur;
    Clarke_Currents_t  clarke_cur;
    Park_Currents_t    park_cur;
    float              elec_angle;      /* rad */
    float              mech_angle_rad;  /* rad */
    float              mech_vel_rpm;    /* rpm - 滤波值 */

    /* ---- PI 控制器 ---- */
    PI_Handle_t id_pi;
    PI_Handle_t iq_pi;
    PI_Handle_t vel_pi;
    PI_Handle_t pos_pi;

    /* ---- 中间量 ---- */
    DQ_Voltage_t        dq_voltage;
    Alpha_Beta_Voltage_t ab_voltage;
    PWM_Duty_t          duty;

    /* ---- 状态 ---- */
    FOC_Mode_t mode;
    bool       enabled;
    float      zero_electrical_offset; /* 新增: 电角度零点偏移 */
} FOC_Handle_t;

/* ------------------- API -------------------- */
void FOC_HandleInit(FOC_Handle_t *h, uint8_t id, const MotorParam_t *motor);
void FOC_SetMode(FOC_Handle_t *h, FOC_Mode_t mode);
void FOC_SetCurrentTarget(FOC_Handle_t *h, float id_ref, float iq_ref);
void FOC_SetVelocityTarget(FOC_Handle_t *h, float rpm);
void FOC_SetPositionTarget(FOC_Handle_t *h, float rad);
void FOC_SetVoltage(FOC_Handle_t *h, float u_q, float u_d, float angle); /* 新增: 直接电压控制 */
void FOC_Calibrate(FOC_Handle_t *h); /* 新增: 校准函数声明 */

/* 主循环（在固定采样周期中断里调用） */
void FOC_RunCurrentLoop(FOC_Handle_t *h, float dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CORE_H */ 
