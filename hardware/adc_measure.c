
#include "adc_measure.h"
#include <string.h>
#include <stdio.h>

// 零点偏移值（支持浮点数，更精确）
static float motor1_offset_a = 0.0f;
static float motor1_offset_b = 0.0f;
static float motor2_offset_a = 0.0f;
static float motor2_offset_b = 0.0f;

// 滤波后的电流值（三相都需要滤波）
static float motor1_current_a = 0.0f;
static float motor1_current_b = 0.0f;
static float motor1_current_c = 0.0f;  // 新增C相滤波值
static float motor2_current_a = 0.0f;
static float motor2_current_b = 0.0f;
static float motor2_current_c = 0.0f;  // 新增C相滤波值


#define FILTER_ALPHA 0.95f  

void ADC_Measure_Init(void)
{
  // 启动TIM2的PWM输出通道（CH1, CH2, CH3）
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  // 启动TIM4的PWM输出通道（CH1, CH2, CH3）
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  // 启动TIM2_CH4和TIM4_CH4用于TRGO触发ADC（PWM模式，但不输出到引脚）
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1050);  // 设置触发时刻（中心点）
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1050);  // 设置触发时刻（中心点）
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // 启动CH4用于产生TRGO
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);  // 启动CH4用于产生TRGO

  // 启动ADC注入模式（硬件触发 - 使用TRGO）
  HAL_ADCEx_InjectedStart_IT(&hadc2);  // ADC2注入模式，TIM2_TRGO触发
  HAL_ADCEx_InjectedStart_IT(&hadc3);  // ADC3注入模式，TIM4_TRGO触发

  // 启动定时器
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);
}

void ADC_Calibrate_Current_Sensors(void)
{
  float sum1a = 0.0f, sum1b = 0.0f, sum2a = 0.0f, sum2b = 0.0f;

  // 确保PWM为0（设置CH1, CH2, CH3输出通道）
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

  HAL_Delay(100);  // 增加稳定时间

  // 采样2000次求平均（提高精度）
  // 注入模式下，直接读取注入数据寄存器
  for (int i = 0; i < 2000; i++)
  {
    sum1a += (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    sum1b += (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    sum2a += (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
    sum2b += (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
    HAL_Delay(1);
  }

  motor1_offset_a = sum1a / 2000.0f;
  motor1_offset_b = sum1b / 2000.0f;
  motor2_offset_a = sum2a / 2000.0f;
  motor2_offset_b = sum2b / 2000.0f;
}

/**
 * @brief ADC注入转换完成回调（注入模式）
 * @note 注入模式下，硬件触发后自动转换，转换完成后触发此中断
 *       关键：先计算C相，再对三相同时滤波，避免C相延迟
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC2)
  {
    // 读取注入通道的ADC值并去零点（使用浮点数提高精度）
    // ⭐ 交换RANK_1和RANK_2的读取，修正A、B相反接问题
    float adc_a = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) - motor1_offset_a;
    float adc_b = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) - motor1_offset_b;

    // 转换为电流（A）
    float ia = adc_a * (ADC_VREF / ADC_RESOLUTION) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
    float ib = adc_b * (ADC_VREF / ADC_RESOLUTION) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);

    // ⭐ 关键修复：先用原始电流计算C相
    float ic = -(ia + ib);

    // 然后对三相同时滤波，保证相位一致
    motor1_current_a += FILTER_ALPHA * (ia - motor1_current_a);
    motor1_current_b += FILTER_ALPHA * (ib - motor1_current_b);
    motor1_current_c += FILTER_ALPHA * (ic - motor1_current_c);
  }
  else if (hadc->Instance == ADC3)
  {
    float adc_a = (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - motor2_offset_a;
    float adc_b = (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2) - motor2_offset_b;

    // 转换为电流（A）
    float ia = adc_a * (ADC_VREF / ADC_RESOLUTION) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
    float ib = adc_b * (ADC_VREF / ADC_RESOLUTION) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);

    // ⭐ 关键修复：先用原始电流计算C相
    float ic = -(ia + ib);
    // 然后对三相同时滤波
    motor2_current_a += FILTER_ALPHA * (ia - motor2_current_a);
    motor2_current_b += FILTER_ALPHA * (ib - motor2_current_b);
    motor2_current_c += FILTER_ALPHA * (ic - motor2_current_c);
  }
}

// 获取电机1电流（新接口）
// ⭐ 关键修复：统一符号处理，确保基尔霍夫定律成立
float ADC_Get_Motor1_Current_A(void)
{
  return -motor1_current_a;  // 负号根据硬件方向调整
}

float ADC_Get_Motor1_Current_B(void)
{
  return -motor1_current_b;  // 负号根据硬件方向调整
}

float ADC_Get_Motor1_Current_C(void)
{
  // ⭐ 修复：C相也取负号，与A、B相保持一致
  return -motor1_current_c;
}

// 获取电机2电流（新接口）
float ADC_Get_Motor2_Current_A(void)
{
  return -motor2_current_a;
}

float ADC_Get_Motor2_Current_B(void)
{
  return -motor2_current_b;
}

float ADC_Get_Motor2_Current_C(void)
{
  // ⭐ 修复：C相也取负号，与A、B相保持一致
  return -motor2_current_c;
}
