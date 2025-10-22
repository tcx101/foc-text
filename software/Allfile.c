#include "Allfile.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h" // 添加这行，确保包含所有STM32相关定义
#include <stdio.h>
#include <math.h>
#include "lcd.h"
#include "lcd_init.h"
#include "as5600.h"
#include "adc_measure.h"
#include "key.h"
#include "serial.h"
#include "simplefoc.h"
#include "vofa.h"
#include "tim.h"

// 外部变量声明
extern AS5600_t as5600_l;
extern AS5600_t as5600_r;

// ============================================================================
// 电机1的HAL接口实现
// ============================================================================

// 电机1角度传感器HAL
static float Motor1_ReadAngle(void)
{
    return AS5600_GetElecRad(&as5600_l);
}

static FOC_AngleSensorHAL_t motor1_angle_hal = {
    .read_angle = Motor1_ReadAngle
};

// 电机1电流传感器HAL
static void Motor1_ReadCurrents(float *ia, float *ib, float *ic)
{
    *ia = ADC_Get_Phase_Current_A();
    *ib = ADC_Get_Phase_Current_B();
    *ic = ADC_Get_Phase_Current_C();
}

static FOC_CurrentSensorHAL_t motor1_current_hal = {
    .read_currents = Motor1_ReadCurrents
};

// 电机1 PWM驱动HAL
static void Motor1_SetVoltages(float va, float vb, float vc)
{
    // 将电压转换为占空比 (-1.0 ~ +1.0)
    // TIM2是电机1的PWM定时器
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    
    // 将电压映射到PWM占空比 (假设双极性驱动，0.5为中点)
    uint32_t ccr_a = (uint32_t)((va + 1.0f) * 0.5f * arr);
    uint32_t ccr_b = (uint32_t)((vb + 1.0f) * 0.5f * arr);
    uint32_t ccr_c = (uint32_t)((vc + 1.0f) * 0.5f * arr);
    
    // 限幅
    if (ccr_a > arr) ccr_a = arr;
    if (ccr_b > arr) ccr_b = arr;
    if (ccr_c > arr) ccr_c = arr;
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr_c);
}

static FOC_PwmDriverHAL_t motor1_pwm_hal = {
    .set_voltages = Motor1_SetVoltages
};

// 电机1母线电压传感器HAL
static float Motor1_ReadBusVoltage(void)
{
    return 12.0f; // 假设12V供电，实际应从ADC读取
}

static FOC_BusVoltageHAL_t motor1_vbus_hal = {
    .read_voltage = Motor1_ReadBusVoltage
};

// ============================================================================
// 电机2的HAL接口实现
// ============================================================================

// 电机2角度传感器HAL
static float Motor2_ReadAngle(void)
{
    return AS5600_GetElecRad(&as5600_r);
}

static FOC_AngleSensorHAL_t motor2_angle_hal = {
    .read_angle = Motor2_ReadAngle
};

// 电机2电流传感器HAL
static void Motor2_ReadCurrents(float *ia, float *ib, float *ic)
{
    *ia = ADC_Get_Phase_Current_A_Motor2();
    *ib = ADC_Get_Phase_Current_B_Motor2();
    *ic = ADC_Get_Phase_Current_C_Motor2();
}

static FOC_CurrentSensorHAL_t motor2_current_hal = {
    .read_currents = Motor2_ReadCurrents
};

// 电机2 PWM驱动HAL
static void Motor2_SetVoltages(float va, float vb, float vc)
{
    // 将电压转换为占空比 (-1.0 ~ +1.0)
    // TIM4是电机2的PWM定时器
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
    
    // 将电压映射到PWM占空比 (假设双极性驱动，0.5为中点)
    uint32_t ccr_a = (uint32_t)((va + 1.0f) * 0.5f * arr);
    uint32_t ccr_b = (uint32_t)((vb + 1.0f) * 0.5f * arr);
    uint32_t ccr_c = (uint32_t)((vc + 1.0f) * 0.5f * arr);
    
    // 限幅
    if (ccr_a > arr) ccr_a = arr;
    if (ccr_b > arr) ccr_b = arr;
    if (ccr_c > arr) ccr_c = arr;
    
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr_c);
}

static FOC_PwmDriverHAL_t motor2_pwm_hal = {
    .set_voltages = Motor2_SetVoltages
};

// 电机2母线电压传感器HAL
static float Motor2_ReadBusVoltage(void)
{
    return 12.0f; // 假设12V供电，实际应从ADC读取
}

static FOC_BusVoltageHAL_t motor2_vbus_hal = {
    .read_voltage = Motor2_ReadBusVoltage
};

// ============================================================================
// HAL获取函数
// ============================================================================

void FOC_GetMotor1HAL(
    FOC_AngleSensorHAL_t **angle_hal,
    FOC_CurrentSensorHAL_t **current_hal,
    FOC_PwmDriverHAL_t **pwm_hal,
    FOC_BusVoltageHAL_t **vbus_hal,
    FOC_VelocityEstimatorHAL_t **vel_hal)
{
    if (angle_hal) *angle_hal = &motor1_angle_hal;
    if (current_hal) *current_hal = &motor1_current_hal;
    if (pwm_hal) *pwm_hal = &motor1_pwm_hal;
    if (vbus_hal) *vbus_hal = &motor1_vbus_hal;
    if (vel_hal) *vel_hal = NULL; // 暂不使用速度估计器
}

void FOC_GetMotor2HAL(
    FOC_AngleSensorHAL_t **angle_hal,
    FOC_CurrentSensorHAL_t **current_hal,
    FOC_PwmDriverHAL_t **pwm_hal,
    FOC_BusVoltageHAL_t **vbus_hal,
    FOC_VelocityEstimatorHAL_t **vel_hal)
{
    if (angle_hal) *angle_hal = &motor2_angle_hal;
    if (current_hal) *current_hal = &motor2_current_hal;
    if (pwm_hal) *pwm_hal = &motor2_pwm_hal;
    if (vbus_hal) *vbus_hal = &motor2_vbus_hal;
    if (vel_hal) *vel_hal = NULL; // 暂不使用速度估计器
}

// ============================================================================
// 电机初始化配置封装函数
// ============================================================================

/**
 * @brief 初始化并配置电机1
 * @param motor 电机结构体指针
 */
void Motor1_Init_And_Config(FOC_Motor_t *motor)
{
    // 构造电机1配置
    FOC_MotorConfig_t config = {0};
    config.pole_pairs = 7;            // 7对极
    config.voltage_limit = 12.0f;     // 12V供电
    config.current_limit = 2.0f;      // 电流限制
    config.use_decoupling = true;
    config.sensor_direction = 1;
    
    // 初始化电机
    FOC_Init(motor, &config);
    
    // 绑定HAL接口
    FOC_AngleSensorHAL_t *angle = NULL;
    FOC_CurrentSensorHAL_t *current = NULL;
    FOC_PwmDriverHAL_t *pwm = NULL;
    FOC_BusVoltageHAL_t *vbus = NULL;
    FOC_VelocityEstimatorHAL_t *vel = NULL;
    
    FOC_GetMotor1HAL(&angle, &current, &pwm, &vbus, &vel);
    FOC_AttachAngleSensor(motor, angle);
    FOC_AttachCurrentSensor(motor, current);
    FOC_AttachPwmDriver(motor, pwm);
    FOC_AttachBusVoltageSensor(motor, vbus);
    if (vel) { FOC_AttachVelocityEstimator(motor, vel); }
    
    // 校准与模式设置
    FOC_CalibrateDirection(motor);       // 方向校准
    FOC_CalibrateZeroOffset(motor);      // 零点校准
    FOC_SetMode(motor, FOC_MODE_TORQUE); // 转矩模式
    FOC_SetTarget(motor, 0.0f);          // 目标电流
}

/**
 * @brief 初始化并配置电机2
 * @param motor 电机结构体指针
 */
void Motor2_Init_And_Config(FOC_Motor_t *motor)
{
    // 构造电机2配置
    FOC_MotorConfig_t config = {0};
    config.pole_pairs = 7;            // 7对极
    config.voltage_limit = 12.0f;     // 12V供电
    config.current_limit = 2.0f;      // 电流限制
    config.use_decoupling = true;
    config.sensor_direction = 1;
    
    // 初始化电机
    FOC_Init(motor, &config);
    
    // 绑定HAL接口
    FOC_AngleSensorHAL_t *angle = NULL;
    FOC_CurrentSensorHAL_t *current = NULL;
    FOC_PwmDriverHAL_t *pwm = NULL;
    FOC_BusVoltageHAL_t *vbus = NULL;
    FOC_VelocityEstimatorHAL_t *vel = NULL;
    
    FOC_GetMotor2HAL(&angle, &current, &pwm, &vbus, &vel);
    FOC_AttachAngleSensor(motor, angle);
    FOC_AttachCurrentSensor(motor, current);
    FOC_AttachPwmDriver(motor, pwm);
    FOC_AttachBusVoltageSensor(motor, vbus);
    if (vel) { FOC_AttachVelocityEstimator(motor, vel); }
    
    // 校准与模式设置
    FOC_CalibrateDirection(motor);       // 方向校准
    FOC_CalibrateZeroOffset(motor);      // 零点校准
    FOC_SetMode(motor, FOC_MODE_TORQUE); // 转矩模式
    FOC_SetTarget(motor, 0.0f);          // 目标电流
}