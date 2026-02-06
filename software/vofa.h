#ifndef VOFA__H
#define VOFA__H
//包含头文件
#include "Allfile.h"
extern FOC_Motor_t motor1;
extern FOC_Motor_t motor2;
extern AS5600_t as5600_l; 
extern AS5600_t as5600_r; 
//函数 
void vofa_as5600_show(void);
void vofa_currentLoop(void);
void vofa_openloop_test(void);  // 开环测试函数
void vofa_debug_sensors(void);  // 调试函数 - 打印原始传感器数据
void vofa_debug_adc_irq(void);  // 调试函数 - 检查ADC中断计数
void FOC_OpenLoopTest(FOC_Motor_t *motor, float voltage, float speed);
#endif
