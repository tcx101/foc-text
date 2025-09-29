/**
 ******************************************************************************
 * @file    adc_measure.c
 * @brief   电压电流采集相关函数实现 - 简化版低通滤波
 ******************************************************************************
 */

#include "adc_measure.h"
#include "arm_math.h"

static float adc_to_current(int32_t calibrated_adc);

// ADC2 DMA缓冲区：[0]=IN5, [1]=IN8（电机1）
volatile uint16_t adc2_dma_buffer[2] = {0};
// ADC3 DMA缓冲区：[0]=IN10, [1]=IN13（电机2）
volatile uint16_t adc3_dma_buffer[2] = {0};

// 锁存的原始ADC值（保证A/B成对一致）
static volatile uint16_t motor1_adc_latched_a = 0;
static volatile uint16_t motor1_adc_latched_b = 0;
static volatile uint16_t motor2_adc_latched_a = 0;
static volatile uint16_t motor2_adc_latched_b = 0;

// 校准后的ADC零点偏移值（电机1）
static uint16_t motor1_adc_offset_a = 0; // ADC2 IN5 偏移值
static uint16_t motor1_adc_offset_b = 0; // ADC2 IN8 偏移值
// 校准后的ADC零点偏移值（电机2）
static uint16_t motor2_adc_offset_a = 0; // ADC3 IN10 偏移值
static uint16_t motor2_adc_offset_b = 0; // ADC3 IN13 偏移值

// 简化版：仅使用一级低通滤波
static volatile float motor1_current_a_filtered = 0.0f;
static volatile float motor1_current_b_filtered = 0.0f;
static volatile float motor1_current_a_filtered_2 = 0.0f;
static volatile float motor1_current_b_filtered_2 = 0.0f;
static volatile uint8_t motor1_current_filter_inited = 0;

static volatile float motor2_current_a_filtered = 0.0f;
static volatile float motor2_current_b_filtered = 0.0f;
static volatile float motor2_current_a_filtered_2 = 0.0f;
static volatile float motor2_current_b_filtered_2 = 0.0f;
static volatile uint8_t motor2_current_filter_inited = 0;

// 低通滤波参数 - 针对5kHz控制频率优化
#ifndef CURRENT_FILTER_ALPHA
#define CURRENT_FILTER_ALPHA (0.3f) // 截止频率约800Hz，平衡响应速度与噪声抑制
#endif

#ifndef CURRENT_FILTER_ALPHA_STAGE2
#define CURRENT_FILTER_ALPHA_STAGE2 (CURRENT_FILTER_ALPHA * 0.5f) // 二级滤波，轻微增加平滑
#endif

// DMA转换完成标志
static volatile uint8_t adc2_conv_complete = 0;
static volatile uint8_t adc3_conv_complete = 0;

void ADC_Measure_Init(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);//触发adc采样
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim2) / 2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim4) / 2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  // 停止可能存在的ADC传输
  HAL_ADC_Stop_DMA(&hadc2);
  HAL_ADC_Stop_DMA(&hadc3);

  // 清零ADC缓冲区
  adc2_dma_buffer[0] = 0;
  adc2_dma_buffer[1] = 0;
  adc3_dma_buffer[0] = 0;
  adc3_dma_buffer[1] = 0;

  // 重置滤波状态
  motor1_current_filter_inited = 0;
  motor2_current_filter_inited = 0;
  motor1_current_a_filtered = motor1_current_b_filtered = 0.0f;
  motor2_current_a_filtered = motor2_current_b_filtered = 0.0f;
  motor1_current_a_filtered_2 = motor1_current_b_filtered_2 = 0.0f;
  motor2_current_a_filtered_2 = motor2_current_b_filtered_2 = 0.0f;

  // 启动DMA传输
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma_buffer, 2);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_dma_buffer, 2);
}

void ADC_Calibrate_Current_Sensors(void)
{
  uint32_t sum_m1a = 0, sum_m1b = 0;
  uint32_t sum_m2a = 0, sum_m2b = 0;
  const int calibration_samples = 500;

  // 在校准前，确保所有PWM通道输出为0（完全关闭）避免电流流动
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

  HAL_Delay(50); // 等待ADC DMA稳定

  // 采集并累加ADC原始读数
  for (int i = 0; i < calibration_samples; i++)
  {
    sum_m1a += adc2_dma_buffer[0]; // ADC2 IN5
    sum_m1b += adc2_dma_buffer[1]; // ADC2 IN8
    sum_m2a += adc3_dma_buffer[0]; // ADC3 IN10
    sum_m2b += adc3_dma_buffer[1]; // ADC3 IN13
    HAL_Delay(1);
  }

  // 计算平均值作为ADC零点偏移
  motor1_adc_offset_a = sum_m1a / calibration_samples;
  motor1_adc_offset_b = sum_m1b / calibration_samples;
  motor2_adc_offset_a = sum_m2a / calibration_samples;
  motor2_adc_offset_b = sum_m2b / calibration_samples;
}

void ADC_DMA_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == hadc2.Instance)
  {
    // 锁存一对一致的A/B采样
    motor1_adc_latched_a = adc2_dma_buffer[0];
    motor1_adc_latched_b = adc2_dma_buffer[1];

    // 采样侧：计算并更新滤波电流
    int32_t cal_a = (int32_t)motor1_adc_latched_a - (int32_t)motor1_adc_offset_a;
    int32_t cal_b = (int32_t)motor1_adc_latched_b - (int32_t)motor1_adc_offset_b;
    float ia_raw = adc_to_current(cal_a);
    float ib_raw = adc_to_current(cal_b);

    // 简化版低通滤波
    if (!motor1_current_filter_inited)
    {
      motor1_current_a_filtered = ia_raw;
      motor1_current_b_filtered = ib_raw;
      motor1_current_a_filtered_2 = ia_raw;
      motor1_current_b_filtered_2 = ib_raw;
      motor1_current_filter_inited = 1;
    }
    else
    {
      motor1_current_a_filtered += CURRENT_FILTER_ALPHA * (ia_raw - motor1_current_a_filtered);
      motor1_current_b_filtered += CURRENT_FILTER_ALPHA * (ib_raw - motor1_current_b_filtered);
      motor1_current_a_filtered_2 += CURRENT_FILTER_ALPHA_STAGE2 * (motor1_current_a_filtered - motor1_current_a_filtered_2);
      motor1_current_b_filtered_2 += CURRENT_FILTER_ALPHA_STAGE2 * (motor1_current_b_filtered - motor1_current_b_filtered_2);
    }

    adc2_conv_complete = 1;
  }
  else if (hadc->Instance == hadc3.Instance)
  {
    // 锁存一对一致的A/B采样
    motor2_adc_latched_a = adc3_dma_buffer[0];
    motor2_adc_latched_b = adc3_dma_buffer[1];

    int32_t cal_a = (int32_t)motor2_adc_latched_a - (int32_t)motor2_adc_offset_a;
    int32_t cal_b = (int32_t)motor2_adc_latched_b - (int32_t)motor2_adc_offset_b;
    float ia_raw = adc_to_current(cal_a);
    float ib_raw = adc_to_current(cal_b);

    // 简化版低通滤波
    if (!motor2_current_filter_inited)
    {
      motor2_current_a_filtered = ia_raw;
      motor2_current_b_filtered = ib_raw;
      motor2_current_a_filtered_2 = ia_raw;
      motor2_current_b_filtered_2 = ib_raw;
      motor2_current_filter_inited = 1;
    }
    else
    {
      motor2_current_a_filtered += CURRENT_FILTER_ALPHA * (ia_raw - motor2_current_a_filtered);
      motor2_current_b_filtered += CURRENT_FILTER_ALPHA * (ib_raw - motor2_current_b_filtered);
      motor2_current_a_filtered_2 += CURRENT_FILTER_ALPHA_STAGE2 * (motor2_current_a_filtered - motor2_current_a_filtered_2);
      motor2_current_b_filtered_2 += CURRENT_FILTER_ALPHA_STAGE2 * (motor2_current_b_filtered - motor2_current_b_filtered_2);
    }

    adc3_conv_complete = 1;
  }
}

uint8_t ADC_Consume_Motor1_Ready(void)
{
  uint8_t ready = adc2_conv_complete;
  adc2_conv_complete = 0;
  return ready;
}

uint8_t ADC_Consume_Motor2_Ready(void)
{
  uint8_t ready = adc3_conv_complete;
  adc3_conv_complete = 0;
  return ready;
}

float ADC_Get_Phase_Current_A(void)
{
  // 优先返回采样侧滤波值；若尚未初始化，则按旧路径计算一次
  if (motor1_current_filter_inited)
  {
    return motor1_current_a_filtered_2;
  }
  int32_t calibrated_adc = (int32_t)motor1_adc_latched_a - (int32_t)motor1_adc_offset_a;
  return adc_to_current(calibrated_adc);
}

float ADC_Get_Phase_Current_B(void)
{
  if (motor1_current_filter_inited)
  {
    return motor1_current_b_filtered_2;
  }
  int32_t calibrated_adc = (int32_t)motor1_adc_latched_b - (int32_t)motor1_adc_offset_b;
  return adc_to_current(calibrated_adc);
}

float ADC_Get_Phase_Current_C(void)
{
  // 根据基尔霍夫电流定律 (Ia + Ib + Ic = 0)
  return -(ADC_Get_Phase_Current_A() + ADC_Get_Phase_Current_B());
}

float ADC_Get_Phase_Current_A_Motor2(void)
{
  if (motor2_current_filter_inited)
  {
    return motor2_current_a_filtered_2;
  }
  int32_t calibrated_adc = (int32_t)motor2_adc_latched_a - (int32_t)motor2_adc_offset_a;
  return adc_to_current(calibrated_adc);
}

float ADC_Get_Phase_Current_B_Motor2(void)
{
  if (motor2_current_filter_inited)
  {
    return  motor2_current_b_filtered_2;
  }
  int32_t calibrated_adc = (int32_t)motor2_adc_latched_b - (int32_t)motor2_adc_offset_b;
  return  adc_to_current(calibrated_adc);
}

float ADC_Get_Phase_Current_C_Motor2(void)
{
  return -(ADC_Get_Phase_Current_A_Motor2() + ADC_Get_Phase_Current_B_Motor2());
}

static float adc_to_current(int32_t calibrated_adc)
{
  float voltage_calibrated = (float)calibrated_adc * (ADC_VREF / ADC_RESOLUTION);
  float current = -voltage_calibrated / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);  
  return current;
}
