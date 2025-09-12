#ifndef BLANCE_H
#define BLANCE_H
//包含头文件
#include "Allfile.h"
typedef struct 
{
    float kp;           // 比例增益
    float kd;           // 微分增益
    float target;      // 目标值
}vertical;

//直立环参数设定函数
void blance_init(vertical*pid, float kp, float kd);
//直立环函数
double blance_vertical(vertical*pid, float Angle, float gx);


#endif
