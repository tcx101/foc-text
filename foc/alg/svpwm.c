#include "svpwm.h"
#include <math.h>

/* 最大调制比留 0.95 余量 */
#define MAX_MODULATION_INDEX 0.95f

void SVPWM_Calculate(const Alpha_Beta_Voltage_t *v, PWM_Duty_t *duty, float v_bus)
{
    /* 步骤: 归一化电压 -> 计算三相参考 -> 计算中点 -> 得占空比 */
    float v_alpha = v->alpha / v_bus;
    float v_beta  = v->beta  / v_bus;

    /* 限幅 */
    float mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (mag > MAX_MODULATION_INDEX && mag > 1e-3f)
    {
        float scale = MAX_MODULATION_INDEX / mag;
        v_alpha *= scale;
        v_beta  *= scale;
    }

    /* 计算三相电压 */
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + 0.866025404f * v_beta;   /* sqrt(3)/2=0.8660 */
    float v_c = -0.5f * v_alpha - 0.866025404f * v_beta;

    /* 找最大/最小以计算零序 */
    float v_max = fmaxf(fmaxf(v_a, v_b), v_c);
    float v_min = fminf(fminf(v_a, v_b), v_c);
    float v_offset = -0.5f * (v_max + v_min);

    duty->ta = 0.5f + v_a + v_offset;
    duty->tb = 0.5f + v_b + v_offset;
    duty->tc = 0.5f + v_c + v_offset;

    /* 限幅到 [0,1] */
#define CLAMP01(x) if ((x) < 0.0f) (x) = 0.0f; else if ((x) > 1.0f) (x) = 1.0f;
    CLAMP01(duty->ta);
    CLAMP01(duty->tb);
    CLAMP01(duty->tc);
#undef CLAMP01
} 
