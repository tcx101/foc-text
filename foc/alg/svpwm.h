#ifndef FOC_ALG_SVPWM_H
#define FOC_ALG_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "clarke_park.h"

/* PWM 占空比结构体 (0~1) */
typedef struct {
    float ta;
    float tb;
    float tc;
} PWM_Duty_t;

void SVPWM_Calculate(const Alpha_Beta_Voltage_t *v_ab,
                     PWM_Duty_t *duty,
                     float v_bus);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ALG_SVPWM_H */ 
