#include "pi.h"

void PI_Init(PI_Handle_t *pi, float kp, float ki, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

void PI_Reset(PI_Handle_t *pi)
{
    pi->integral = 0.0f;
}

float PI_Calc(PI_Handle_t *h, float ref, float fdb)
{
    float err = ref - fdb;
    h->proportional = err * h->kp;

    /*
     * 抗积分饱和 (Anti-Windup)
     * 1. 先计算无约束的输出
     * 2. 判断输出是否饱和
     * 3. 仅在不饱和时累加积分
    */
    float out_noclamp = h->proportional + h->integral;

    if (out_noclamp < h->out_max && out_noclamp > h->out_min)
    {
        h->integral += err * h->ki;
    }

    /* 4. 对最终输出进行限幅 */
    if (out_noclamp > h->out_max)
    {
        return h->out_max;
    }
    else if (out_noclamp < h->out_min)
    {
        return h->out_min;
    }
    else
    {
        return out_noclamp;
    }
} 
