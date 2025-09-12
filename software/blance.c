#include "blance.h"
vertical vpid;
//直立环参数设定
void blance_init(vertical*pid, float kp, float kd){
    pid->kp = kp;
    pid->kd = kd;
    pid->target = 0.0f;
}
//直立环
double blance_vertical(vertical*pid, float Angle, float gx){
    float err = pid->kp*(Angle - pid->target) +  pid->kd*gx;
    return err;
}
