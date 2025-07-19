/**
  ******************************************************************************
  * @file    adc_measure.c
  * @brief   电压电流采集相关函数实现
  ******************************************************************************
  */

/* 包含 -----------------------------------------------------------------------*/
#include "adc_measure.h"
#include "serial.h"
#include <math.h>  // 添加math.h头文件解决fabsf未声明的警告

/* 私有宏 ---------------------------------------------------------------------*/
#define FILTER_SIZE 4  // 统一所有滤波器的窗口大小

/* 私有变量 -------------------------------------------------------------------*/
// 电机1电流相关变量
uint16_t motor1_adc_buffer[2] = {0};       // 电机1的ADC DMA缓冲区
float motor1_current_a = 0.0f;             // 电机1的A相电流
float motor1_current_b = 0.0f;             // 电机1的B相电流
float motor1_current_c = 0.0f;             // 电机1的C相电流（计算得出）
float motor1_current_a_offset = 0.0f;      // 电机1的A相电流零点偏移
float motor1_current_b_offset = 0.0f;      // 电机1的B相电流零点偏移

// 电机2电流相关变量
uint16_t motor2_adc_buffer[2] = {0};       // 电机2的ADC DMA缓冲区
float motor2_current_a = 0.0f;             // 电机2的A相电流
float motor2_current_b = 0.0f;             // 电机2的B相电流
float motor2_current_c = 0.0f;             // 电机2的C相电流（计算得出）
float motor2_current_a_offset = 0.0f;      // 电机2的A相电流零点偏移
float motor2_current_b_offset = 0.0f;      // 电机2的B相电流零点偏移

// 电源电压
float motor_supply_voltage = 12.0f;        // 电机供电电压

/**
  * @简述  初始化ADC测量模块
  * @参数  无
  * @返回值 无
  */
void ADC_Measure_Init(void)
{
    // 停止可能正在运行的ADC DMA传输
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    
    // 清零DMA缓冲区
    motor1_adc_buffer[0] = 0;
    motor1_adc_buffer[1] = 0;
    motor2_adc_buffer[0] = 0;
    motor2_adc_buffer[1] = 0;
    
    // 设置默认值
    motor1_current_a = 0.0f;
    motor1_current_b = 0.0f;
    motor1_current_c = 0.0f;
    motor2_current_a = 0.0f;
    motor2_current_b = 0.0f;
    motor2_current_c = 0.0f;
    motor_supply_voltage = 12.0f;
    
    // 初始化零点偏移校准值
    motor1_current_a_offset = 0.0f;
    motor1_current_b_offset = 0.0f;
    motor2_current_a_offset = 0.0f;
    motor2_current_b_offset = 0.0f;
    
    // 启动DMA传输
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)motor1_adc_buffer, 2);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)motor2_adc_buffer, 2);
    
    // 等待一段时间确保DMA传输开始
    HAL_Delay(50);
    
    // 执行电流传感器零点校准
    ADC_Calibrate_Current_Sensors();
}

/**
  * @简述  校准电流传感器零点
  * @参数  无
  * @返回值 无
  */
void ADC_Calibrate_Current_Sensors(void)
{
    float sum_m1a = 0.0f;
    float sum_m1b = 0.0f;
    float sum_m2a = 0.0f;
    float sum_m2b = 0.0f;
    
    // 为了稳定的校准，等待DMA多次采样
    HAL_Delay(50);
    
    // 采集多个样本并求平均值
    for(uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        // 计算电机1的A相电流（未校准）
        float voltage_m1a = (motor1_adc_buffer[0] / ADC_RESOLUTION) * ADC_VREF;
        float current_m1a = (voltage_m1a - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
        
        // 计算电机1的B相电流（未校准）
        float voltage_m1b = (motor1_adc_buffer[1] / ADC_RESOLUTION) * ADC_VREF;
        float current_m1b = (voltage_m1b - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
        
        // 计算电机2的A相电流（未校准）
        float voltage_m2a = (motor2_adc_buffer[0] / ADC_RESOLUTION) * ADC_VREF;
        float current_m2a = (voltage_m2a - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
        
        // 计算电机2的B相电流（未校准）
        float voltage_m2b = (motor2_adc_buffer[1] / ADC_RESOLUTION) * ADC_VREF;
        float current_m2b = (voltage_m2b - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
        
        // 累加采样值
        sum_m1a += current_m1a;
        sum_m1b += current_m1b;
        sum_m2a += current_m2a;
        sum_m2b += current_m2b;
        
        // 短暂延时确保DMA采样更新
        HAL_Delay(5);
    }
    
    // 计算平均偏移值
    motor1_current_a_offset = sum_m1a / CALIBRATION_SAMPLES;
    motor1_current_b_offset = sum_m1b / CALIBRATION_SAMPLES;
    motor2_current_a_offset = sum_m2a / CALIBRATION_SAMPLES;
    motor2_current_b_offset = sum_m2b / CALIBRATION_SAMPLES;
    
    // 对偏移值进行合理性检查
    if (fabsf(motor1_current_a_offset) > 1.0f) motor1_current_a_offset = 0.0f;
    if (fabsf(motor1_current_b_offset) > 1.0f) motor1_current_b_offset = 0.0f;
    if (fabsf(motor2_current_a_offset) > 1.0f) motor2_current_a_offset = 0.0f;
    if (fabsf(motor2_current_b_offset) > 1.0f) motor2_current_b_offset = 0.0f;
}

/**
  * @简述  获取电机1的A相电流
  * @参数  无
  * @返回值 电机1的A相电流值(A)
  */
float Motor1_Get_Phase_Current_A(void)
{
    // 计算电压值
    float voltage = (motor1_adc_buffer[0] / ADC_RESOLUTION) * ADC_VREF;
    
    // 单端模式计算电流值，参考电压为VDD/2（1.65V）
    float current_raw = (voltage - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
    
    // 应用零点校准
    float current = current_raw - motor1_current_a_offset;
    
    // 为 M1-A 相创建独立的滤波变量
    static float m1a_current_buffer[FILTER_SIZE] = {0};
    static uint8_t m1a_buffer_index = 0;
    static uint8_t m1a_buffer_filled = 0;
    static uint8_t m1a_first_call = 1;
    
    // 初始化
    if (m1a_first_call) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            m1a_current_buffer[i] = current;
        }
        m1a_first_call = 0;
        m1a_buffer_filled = 1;
    }
    
    // 移动窗口滤波
    m1a_current_buffer[m1a_buffer_index] = current;
    m1a_buffer_index = (m1a_buffer_index + 1) % FILTER_SIZE;
    
    if (m1a_buffer_filled) {
        // 计算平均值
        float sum = 0.0f;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += m1a_current_buffer[i];
        }
        current = sum / FILTER_SIZE;
        
        // 应用额外的一阶低通滤波以平滑输出
        static float m1a_filtered_current = 0.0f;
        const float alpha = 0.5f; // 平衡响应速度和平滑度
        m1a_filtered_current = alpha * current + (1.0f - alpha) * m1a_filtered_current;
        current = m1a_filtered_current;
    }
    
    // 电流限幅保护
    if (current > 5.0f) current = 5.0f;
    if (current < -5.0f) current = -5.0f;
    
    // 保持 A 相电流原始方向（已确认编码器/相序正确）
    // current 符号不再反转

    // 存储计算结果（不带符号配置）
    motor1_current_a = current;

    // 应用相方向配置并返回
    return current;
}

/**
  * @简述  获取电机1的B相电流
  * @参数  无
  * @返回值 电机1的B相电流值(A)
  */
float Motor1_Get_Phase_Current_B(void)
{
    // 计算电压值
    float voltage = (motor1_adc_buffer[1] / ADC_RESOLUTION) * ADC_VREF;
    
    // 单端模式计算电流值，参考电压为VDD/2（1.65V）
    float current_raw = (voltage - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
    
    // 应用零点校准
    float current = current_raw - motor1_current_b_offset;
    
    // 为 M1-B 相创建独立的滤波变量
    static float m1b_current_buffer[FILTER_SIZE] = {0};
    static uint8_t m1b_buffer_index = 0;
    static uint8_t m1b_buffer_filled = 0;
    static uint8_t m1b_first_call = 1;
    
    // 初始化
    if (m1b_first_call) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            m1b_current_buffer[i] = current;
        }
        m1b_first_call = 0;
        m1b_buffer_filled = 1;
    }
    
    // 移动窗口滤波
    m1b_current_buffer[m1b_buffer_index] = current;
    m1b_buffer_index = (m1b_buffer_index + 1) % FILTER_SIZE;
    
    if (m1b_buffer_filled) {
        // 计算平均值
        float sum = 0.0f;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += m1b_current_buffer[i];
        }
        current = sum / FILTER_SIZE;
        
        // 应用额外的一阶低通滤波以平滑输出
        static float m1b_filtered_current = 0.0f;
        const float alpha = 0.5f; // 平衡响应速度和平滑度
        m1b_filtered_current = alpha * current + (1.0f - alpha) * m1b_filtered_current;
        current = m1b_filtered_current;
    }
    
    // 电流限幅保护
    if (current > 5.0f) current = 5.0f;
    if (current < -5.0f) current = -5.0f;
    
    // 存储计算结果
    motor1_current_b = current;
    
    return current;
}

/**
  * @简述  获取电机1的C相电流（通过A、B相计算）
  * @参数  无
  * @返回值 电机1的C相电流值(A)
  */
float Motor1_Get_Phase_Current_C(void)
{
    // 更新A、B相电流
    Motor1_Get_Phase_Current_A();
    Motor1_Get_Phase_Current_B();
    
    // 根据基尔霍夫电流定律：Ia + Ib + Ic = 0，计算C相电流
    float current = -motor1_current_a - motor1_current_b;
    
    // 存储计算结果
    motor1_current_c = current;
    
    return current;
}

/**
  * @简述  获取电机2的A相电流
  * @参数  无
  * @返回值 电机2的A相电流值(A)
  */
float Motor2_Get_Phase_Current_A(void)
{
    // 计算电压值
    float voltage = (motor2_adc_buffer[0] / ADC_RESOLUTION) * ADC_VREF;
    
    // 单端模式计算电流值
    float current_raw = (voltage - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
    
    // 应用零点校准
    float current = current_raw - motor2_current_a_offset;

    // 为 M2-A 相创建独立的滤波变量
    static float m2a_current_buffer[FILTER_SIZE] = {0};
    static uint8_t m2a_buffer_index = 0;
    static uint8_t m2a_buffer_filled = 0;
    static uint8_t m2a_first_call = 1;

    // 初始化
    if (m2a_first_call) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            m2a_current_buffer[i] = current;
        }
        m2a_first_call = 0;
        m2a_buffer_filled = 1;
    }

    // 移动窗口滤波
    m2a_current_buffer[m2a_buffer_index] = current;
    m2a_buffer_index = (m2a_buffer_index + 1) % FILTER_SIZE;

    if (m2a_buffer_filled) {
        // 计算平均值
        float sum = 0.0f;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += m2a_current_buffer[i];
        }
        current = sum / FILTER_SIZE;

        // 应用额外的一阶低通滤波以平滑输出
        static float m2a_filtered_current = 0.0f;
        const float alpha = 0.5f; // 平衡响应速度和平滑度
        m2a_filtered_current = alpha * current + (1.0f - alpha) * m2a_filtered_current;
        current = m2a_filtered_current;
    }

    // 电流限幅保护
    if (current > 5.0f) current = 5.0f;
    if (current < -5.0f) current = -5.0f;
    
    // 存储计算结果
    motor2_current_a = current;
    
    return current;
}

/**
  * @简述  获取电机2的B相电流
  * @参数  无
  * @返回值 电机2的B相电流值(A)
  */
float Motor2_Get_Phase_Current_B(void)
{
    // 计算电压值
    float voltage = (motor2_adc_buffer[1] / ADC_RESOLUTION) * ADC_VREF;
    
    // 单端模式计算电流值
    float current_raw = (voltage - CURRENT_SENSOR_REF) / (CURRENT_SENSOR_GAIN * SHUNT_RESISTANCE);
    
    // 应用零点校准
    float current = current_raw - motor2_current_b_offset;

    // 为 M2-B 相创建独立的滤波变量
    static float m2b_current_buffer[FILTER_SIZE] = {0};
    static uint8_t m2b_buffer_index = 0;
    static uint8_t m2b_buffer_filled = 0;
    static uint8_t m2b_first_call = 1;

    // 初始化
    if (m2b_first_call) {
        for (int i = 0; i < FILTER_SIZE; i++) {
            m2b_current_buffer[i] = current;
        }
        m2b_first_call = 0;
        m2b_buffer_filled = 1;
    }

    // 移动窗口滤波
    m2b_current_buffer[m2b_buffer_index] = current;
    m2b_buffer_index = (m2b_buffer_index + 1) % FILTER_SIZE;

    if (m2b_buffer_filled) {
        // 计算平均值
        float sum = 0.0f;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += m2b_current_buffer[i];
        }
        current = sum / FILTER_SIZE;

        // 应用额外的一阶低通滤波以平滑输出
        static float m2b_filtered_current = 0.0f;
        const float alpha = 0.5f; // 平衡响应速度和平滑度
        m2b_filtered_current = alpha * current + (1.0f - alpha) * m2b_filtered_current;
        current = m2b_filtered_current;
    }
    
    // 电流限幅保护
    if (current > 5.0f) current = 5.0f;
    if (current < -5.0f) current = -5.0f;
    
    // 存储计算结果
    motor2_current_b = current;
    
    return current;
}

/**
  * @简述  获取电机2的C相电流（通过A、B相计算）
  * @参数  无
  * @返回值 电机2的C相电流值(A)
  */
float Motor2_Get_Phase_Current_C(void)
{
    // 更新A、B相电流
    Motor2_Get_Phase_Current_A();
    Motor2_Get_Phase_Current_B();
    
    // 根据基尔霍夫电流定律：Ia + Ib + Ic = 0，计算C相电流
    float current = -motor2_current_a - motor2_current_b;
    
    // 存储计算结果
    motor2_current_c = current;
    
    return current;
}

/**
  * @简述  获取供电电压
  * @参数  无
  * @返回值 供电电压值(V)
  */
float ADC_Get_Supply_Voltage(void)
{
    uint32_t adc_value;
    
    // 使用阻塞方式读取ADC1的值（PA4引脚）
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        adc_value = HAL_ADC_GetValue(&hadc1);
        
        // 转换为电压值，考虑分压比
        float voltage = (adc_value / ADC_RESOLUTION) * ADC_VREF * VOLTAGE_DIVIDER_RATIO;
        
        // 添加低通滤波，减少测量波动
        static float filtered_voltage = 0.0f;
        static uint8_t first_call = 1;
        
        if (first_call) {
            filtered_voltage = voltage;
            first_call = 0;
        } else {
            // 低通滤波系数，值越小滤波效果越强
            const float alpha = 0.1f;
            filtered_voltage = alpha * voltage + (1.0f - alpha) * filtered_voltage;
        }
        
        // 存储计算结果
        motor_supply_voltage = filtered_voltage;
        
        return filtered_voltage;
    }
    
    // 如果采集失败，则返回上次的值
    return motor_supply_voltage;
}

/**
  * @简述  更新所有ADC测量值
  * @参数  无
  * @返回值 无
  */
void ADC_Update_All(void)
{
    // 在DMA模式下，ADC值会自动更新到缓冲区
    // 此函数仅用于兼容旧代码，实际不需要执行任何操作
    // 如果需要获取最新值，直接调用相应的Get函数即可
}

/**
  * @简述  ADC转换完成回调函数（DMA传输完成时调用）
  * @参数  hadc: ADC句柄
  * @返回值 无
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // DMA传输完成后自动更新缓冲区，不需要额外处理
    // 如果需要在每次DMA传输完成后执行特定操作，可以在此添加代码
} 

/**
  * @简述  获取A相电流（兼容函数，默认使用电机1）
  * @参数  无
  * @返回值 A相电流值(A)
  */
float ADC_Get_Phase_Current_A(void)
{
    // 调用电机1的A相电流获取函数
    return Motor1_Get_Phase_Current_A();
}

/**
  * @简述  获取B相电流（兼容函数，默认使用电机1）
  * @参数  无
  * @返回值 B相电流值(A)
  */
float ADC_Get_Phase_Current_B(void)
{
    // 调用电机1的B相电流获取函数
    return Motor1_Get_Phase_Current_B();
} 
