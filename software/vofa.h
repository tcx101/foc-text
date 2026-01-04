#ifndef VOFA__H
#define VOFA__H
//包含头文件
#include "Allfile.h"
extern FOC_Motor_t motor1;
extern FOC_Motor_t motor2;
extern AS5600_t as5600_l; 
extern AS5600_t as5600_r; 
//函数 
void vofa_currentLoop(void);
void vofa_velocityLoop(void);
void vofa_currentLoop_m2(void);
void vofa_openLoop_m2(void);
void vofa_currentLoop_Allmotor(void);
void as5600_show(void);
#endif
