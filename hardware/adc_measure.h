/**
  ******************************************************************************
  * @file    adc_measure.h
  * @brief   电压电流采集 - 极简版（仅一级低通滤波）
  ******************************************************************************
  */
#ifndef ADC_MEASURE_H
#define ADC_MEASURE_H

#include "main.h"
#include "adc.h"
#include "tim.h"

/* 硬件参数配置 ---------------------------------------------------------------*/
#define ADC_VREF                (3.3f)      // ADC参考电压 (V)
#define ADC_RESOLUTION          (4095.0f)   // ADC分辨率 (12-bit)
#define SHUNT_RESISTANCE        (0.01f)     // 采样电阻值 (Ohm)
#define CURRENT_SENSOR_GAIN     (50.0f)     // 电流采样放大器增益 (V/V)

/* 函数接口 -------------------------------------------------------------------*/

// 初始化和校准
void ADC_Measure_Init(void);              // 初始化PWM和定时器（不启动ADC中断）
void ADC_Calibrate_Current_Sensors(void); // 校准电流传感器零点
void ADC_Start_Interrupt(void);           // 启动ADC中断

// 电机1电流获取（新接口）
float ADC_Get_Motor1_Current_A(void);
float ADC_Get_Motor1_Current_B(void);
float ADC_Get_Motor1_Current_C(void);

// 电机2电流获取（新接口）
float ADC_Get_Motor2_Current_A(void);
float ADC_Get_Motor2_Current_B(void);
float ADC_Get_Motor2_Current_C(void);

// 诊断函数：验证ABC相匹配
void ADC_Diagnose_Phase_Matching(void);

#endif /* ADC_MEASURE_H */
 






















































