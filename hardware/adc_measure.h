/**
  ******************************************************************************
  * @file    adc_measure.h
  * @brief   电压电流采集相关头文件
  ******************************************************************************
  */

#ifndef ADC_MEASURE_H
#define ADC_MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 -----------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include <stdint.h>  // 添加标准整数类型

/* 宏定义 ---------------------------------------------------------------------*/
// ADC相关物理参数 - !!请根据您的硬件原理图核对这些值!!
#define ADC_VREF                (3.3f)      // ADC参考电压 (V)
#define ADC_RESOLUTION          (4095.0f)   // ADC分辨率 (12-bit)
#define SHUNT_RESISTANCE        (0.01f)     // 采样电阻值 (Ohm)
#define CURRENT_SENSOR_GAIN     (50.0f)     // 电流采样放大器增益 (V/V)

#ifndef CURRENT_MA_WINDOW
#define CURRENT_MA_WINDOW       16           // 电流滑动平均窗口大小（样本数）
#endif

/* 函数原型 */
void ADC_Measure_Init(void);
void ADC_Calibrate_Current_Sensors(void);
float ADC_Get_Phase_Current_A(void);
float ADC_Get_Phase_Current_B(void);
float ADC_Get_Phase_Current_C(void);
void ADC_DMA_ConvCpltCallback(ADC_HandleTypeDef* hadc);
uint8_t ADC_Consume_Motor1_Ready(void);

/* 电机2测量与接口 */
uint8_t ADC_Consume_Motor2_Ready(void);
float ADC_Get_Phase_Current_A_Motor2(void);
float ADC_Get_Phase_Current_B_Motor2(void);
float ADC_Get_Phase_Current_C_Motor2(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_MEASURE_H */ 






















































