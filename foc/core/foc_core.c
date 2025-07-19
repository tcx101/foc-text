#include "foc_core.h"
#include <math.h>

void FOC_HandleInit(FOC_Handle_t *h, uint8_t id, const MotorParam_t *m)
{
    h->id = id;
    h->motor = m;
    h->id_ref = 0.0f;
    h->iq_ref = 0.0f;
    h->vel_ref_rpm = 0.0f;
    h->pos_ref_rad = 0.0f;
    h->enabled = false;
    h->mode = FOC_MODE_DISABLED;
    h->zero_electrical_offset = 0.0f; /* 初始化偏移为0 */

    /* PI 默认限幅 ±50% 母线电压 */
    PI_Init(&h->id_pi, 0.3f, 1.0f, -m->v_bus_nominal * 0.5f, m->v_bus_nominal * 0.5f);
    PI_Init(&h->iq_pi, 0.3f, 1.0f, -m->v_bus_nominal * 0.5f, m->v_bus_nominal * 0.5f);
    PI_Init(&h->vel_pi, 0.05f, 0.01f, -m->current_limit, m->current_limit);
    PI_Init(&h->pos_pi, 5.0f, 0.1f, -500.0f, 500.0f); // 位置环输出速度参考 (RPM)
}

void FOC_SetMode(FOC_Handle_t *h, FOC_Mode_t mode)
{
    h->mode = mode;
    h->enabled = (mode != FOC_MODE_DISABLED);
}

void FOC_SetCurrentTarget(FOC_Handle_t *h, float id_ref, float iq_ref)
{
    h->id_ref = id_ref;
    h->iq_ref = iq_ref;
}

void FOC_SetVelocityTarget(FOC_Handle_t *h, float rpm)
{
    h->vel_ref_rpm = rpm;
}

void FOC_SetPositionTarget(FOC_Handle_t *h, float rad)
{
    h->pos_ref_rad = rad;
}

/* ---- 新增: 直接设置电压矢量 ---- */
void FOC_SetVoltage(FOC_Handle_t *h, float u_q, float u_d, float angle_rad)
{
    h->dq_voltage.q = u_q;
    h->dq_voltage.d = u_d;

    Inverse_Park_Transform(&h->dq_voltage, &h->ab_voltage, angle_rad);
    SVPWM_Calculate(&h->ab_voltage, &h->duty, h->motor->v_bus_nominal);
    hal_pwm_apply(&h->duty, 1000, h->id);
}

/* ---- 修改: 电角度校准 (增加预驱动) ---- */
void FOC_Calibrate(FOC_Handle_t *h)
{
    h->enabled = true;
    const float cal_voltage = 3.0f; // 进一步增校准电压
    const float M_PI_F = 3.1415926f;

    /* 1. 新增: 预驱动环节，来回摆动以克服静摩擦 */
    for (int i = 0; i < 3; i++) {
        FOC_SetVoltage(h, 0, cal_voltage, M_PI_F / 3.0f); // 60度
        hal_delay_ms(100);
        FOC_SetVoltage(h, 0, cal_voltage, -M_PI_F / 3.0f); // -60度
        hal_delay_ms(100);
    }

    /* 2. 最终对齐到电气零度 */
    FOC_SetVoltage(h, 0, cal_voltage, 0); 
    hal_delay_ms(1000); // 延长稳定时间

    /* 3. 读取此时的传感器角度作为偏移 */
    h->zero_electrical_offset = hal_get_electrical_angle_raw(h->id);

    /* 4. 校准后禁用电机 */
    FOC_SetVoltage(h, 0, 0, 0);
    h->enabled = false;
}


/* ---- 主循环 (电流环频率调用) ---- */
void FOC_RunCurrentLoop(FOC_Handle_t *h, float dt)
{
    if (!h->enabled) return;

    /* 1. 采样三相电流 */
    h->phase_cur.a = hal_get_phase_current_a(h->id);
    h->phase_cur.b = hal_get_phase_current_b(h->id);
    h->phase_cur.c = -h->phase_cur.a - h->phase_cur.b; // 两电阻采样系统

    /* 2. Clarke */
    Clarke_Transform(&h->phase_cur, &h->clarke_cur);

    /* 3. 获取电角度 */
    h->elec_angle = hal_get_electrical_angle(h->id);

    /* 4. Park */
    Park_Transform(&h->clarke_cur, &h->park_cur, h->elec_angle);

    /* 5. 外环（若有） */
    if (h->mode == FOC_MODE_VELOCITY)
    {
        float iq_ref = PI_Calc(&h->vel_pi, h->vel_ref_rpm, h->mech_vel_rpm);
        h->id_ref = 0.0f;
        h->iq_ref = iq_ref;
    }
    else if (h->mode == FOC_MODE_POSITION)
    {
        /* 位置 -> 速度 */
        float vel_ref = PI_Calc(&h->pos_pi, h->pos_ref_rad, h->mech_angle_rad);
        /* 速度 -> 电流 */
        float iq_ref = PI_Calc(&h->vel_pi, vel_ref, h->mech_vel_rpm);
        h->id_ref = 0.0f;
        h->iq_ref = iq_ref;
    }

    /* 6. 电流 PI */
    h->dq_voltage.d = PI_Calc(&h->id_pi, h->id_ref, h->park_cur.d);
    h->dq_voltage.q = PI_Calc(&h->iq_pi, h->iq_ref, h->park_cur.q);

    /* 7. 逆 Park -> αβ */
    Inverse_Park_Transform(&h->dq_voltage, &h->ab_voltage, h->elec_angle);

    /* 8. SVPWM */
    SVPWM_Calculate(&h->ab_voltage, &h->duty, h->motor->v_bus_nominal);

    /* 9. 输出 PWM */
    hal_pwm_apply(&h->duty, 1000, h->id); // timer_period占位值 1000
} 
