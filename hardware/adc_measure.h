/**
  ******************************************************************************
  * @file    adc_measure.h
  * @brief   电压电流采集相关头文件
  ******************************************************************************
  */

#ifndef __ADC_MEASURE_H
#define __ADC_MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 -----------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include <stdint.h>  // 添加标准整数类型

/* 宏定义 ---------------------------------------------------------------------*/
#define ADC_VREF                  3.3f      // ADC参考电压
#define ADC_RESOLUTION            4096.0f   // 12位ADC分辨率
#define CURRENT_SENSOR_GAIN       50.0f     // INA240A2 增益50 V/V
#define CURRENT_SENSOR_REF        1.65f     // INA240零电流输出电压 (VDD/2)
#define SHUNT_RESISTANCE          0.01f     // 分流电阻0.01Ω
#define VOLTAGE_DIVIDER_RATIO     3.63f     // 电压分压比（校准后：根据12V实际电源与18.55V测量值校正）
#define CALIBRATION_SAMPLES       100       // 零点校准时的采样数量

/* 变量定义 -------------------------------------------------------------------*/
// 电机1电流相关变量
extern uint16_t motor1_adc_buffer[2];       // 电机1的ADC DMA缓冲区
extern float motor1_current_a;              // 电机1的A相电流
extern float motor1_current_b;              // 电机1的B相电流
extern float motor1_current_c;              // 电机1的C相电流（计算得出）
extern float motor1_current_a_offset;       // 电机1的A相电流零点偏移
extern float motor1_current_b_offset;       // 电机1的B相电流零点偏移

// 电机2电流相关变量
extern uint16_t motor2_adc_buffer[2];       // 电机2的ADC DMA缓冲区
extern float motor2_current_a;              // 电机2的A相电流
extern float motor2_current_b;              // 电机2的B相电流
extern float motor2_current_c;              // 电机2的C相电流（计算得出）
extern float motor2_current_a_offset;       // 电机2的A相电流零点偏移
extern float motor2_current_b_offset;       // 电机2的B相电流零点偏移

// 电源电压
extern float motor_supply_voltage;          // 电机供电电压

/* 函数声明 -------------------------------------------------------------------*/
void ADC_Measure_Init(void);                // 初始化ADC测量模块

// 电机1电流相关函数
float Motor1_Get_Phase_Current_A(void);     // 获取电机1的A相电流
float Motor1_Get_Phase_Current_B(void);     // 获取电机1的B相电流
float Motor1_Get_Phase_Current_C(void);     // 获取电机1的C相电流（通过A、B相计算）

// 电机2电流相关函数
float Motor2_Get_Phase_Current_A(void);     // 获取电机2的A相电流
float Motor2_Get_Phase_Current_B(void);     // 获取电机2的B相电流
float Motor2_Get_Phase_Current_C(void);     // 获取电机2的C相电流（通过A、B相计算）

// 通用函数
float ADC_Get_Supply_Voltage(void);         // 获取供电电压
void ADC_Update_All(void);                  // 更新所有ADC测量值 (兼容函数)
void ADC_Calibrate_Current_Sensors(void);   // 校准电流传感器零点

// 兼容函数 (用于FOC库)
float ADC_Get_Phase_Current_A(void);        // 兼容函数，获取A相电流 (默认使用电机1)
float ADC_Get_Phase_Current_B(void);        // 兼容函数，获取B相电流 (默认使用电机1)

#ifdef __cplusplus
}
#endif

#endif /* __ADC_MEASURE_H */ 
