#ifndef FOC_ALG_PI_H
#define FOC_ALG_PI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;

    float proportional;
    float integral;
    float out_min;
    float out_max;
} PI_Handle_t;

void PI_Init(PI_Handle_t *pi, float kp, float ki, float out_min, float out_max);
void PI_Reset(PI_Handle_t *pi);
float PI_Calc(PI_Handle_t *pi, float ref, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ALG_PI_H */ 
