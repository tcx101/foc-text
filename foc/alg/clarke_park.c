#include "clarke_park.h"
#include <math.h>

/* 1/sqrt(3) */
#define INV_SQRT3 0.57735026919f

void Clarke_Transform(const Phase_Currents_t *p, Clarke_Currents_t *c)
{
    /* 功率不变 Clarke:
       alpha = Ia
       beta  = (Ia + 2*Ib)/sqrt(3)
     */
    c->alpha = p->a;
    c->beta  = (p->a + 2.0f * p->b) * INV_SQRT3;
}

void Park_Transform(const Clarke_Currents_t *c, Park_Currents_t *p, float theta)
{
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    p->d =  c->alpha * cos_t + c->beta * sin_t;
    p->q = -c->alpha * sin_t + c->beta * cos_t;
}

void Inverse_Park_Transform(const DQ_Voltage_t *dq, Alpha_Beta_Voltage_t *ab, float theta)
{
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    ab->alpha = dq->d * cos_t - dq->q * sin_t;
    ab->beta  = dq->d * sin_t + dq->q * cos_t;
} 
