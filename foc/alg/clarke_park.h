#ifndef FOC_ALG_CLARKE_PARK_H
#define FOC_ALG_CLARKE_PARK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ----------- 数据类型 --------------------------------------------------- */
typedef struct {
    float a;
    float b;
    float c; /* 在两电流采样系统中可忽略 */
} Phase_Currents_t;

typedef struct {
    float alpha;
    float beta;
} Clarke_Currents_t;

typedef struct {
    float d;
    float q;
} Park_Currents_t;

/* 电压向量 */
typedef struct {
    float d;
    float q;
} DQ_Voltage_t;

typedef struct {
    float alpha;
    float beta;
} Alpha_Beta_Voltage_t;

/* ----------- 函数原型 --------------------------------------------------- */
void Clarke_Transform(const Phase_Currents_t *phase_currents,
                      Clarke_Currents_t *clarke_currents);

void Park_Transform(const Clarke_Currents_t *clarke_currents,
                    Park_Currents_t *park_currents,
                    float theta_rad);

void Inverse_Park_Transform(const DQ_Voltage_t *dq_voltage,
                            Alpha_Beta_Voltage_t *alpha_beta_voltage,
                            float theta_rad);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ALG_CLARKE_PARK_H */ 
