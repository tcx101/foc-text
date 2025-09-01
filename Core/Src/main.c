/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 新FOC库实�?????????? */
FOC_Motor_t motor1;
extern AS5600_t as5600_l; /* 使用左轮编码器做角度来源 */

/* 新增：当前PWM占空比缓存（0..1�???? */
static volatile float g_pwm_duty_a = 0.0f;
static volatile float g_pwm_duty_b = 0.0f;
static volatile float g_pwm_duty_c = 0.0f;

/* 母线电压缓存（仅在主循环更新，ISR只读�???? */
static volatile float g_vbus_cached = 12.0f;
static volatile uint32_t g_vbus_last_ms = 0;

/* 硬件接口函数 */
static float board_get_bus_voltage(void);
static float board_read_angle(void) {
    return AS5600_GetAngleRad(&as5600_l);
}

static void board_read_currents(float *ia, float *ib, float *ic) {
    if (!ia || !ib || !ic) return;
    *ia = ADC_Get_Phase_Current_A();
    *ib = ADC_Get_Phase_Current_B();
    *ic = -(*ia + *ib);
}

static void board_set_voltages(float va, float vb, float vc) {
    // 转换电压为占空比 (使用缓存的�?�线电压，避免在中断中阻塞ADC)
    float vdc = board_get_bus_voltage();
    if (vdc < 6.0f) vdc = 12.0f;
    
    float duty_a = va / vdc;
    float duty_b = vb / vdc;
    float duty_c = vc / vdc;
    
    // 限制占空比范�????
    if (duty_a < 0.0f) duty_a = 0.0f; if (duty_a > 1.0f) duty_a = 1.0f;
    if (duty_b < 0.0f) duty_b = 0.0f; if (duty_b > 1.0f) duty_b = 1.0f;
    if (duty_c < 0.0f) duty_c = 0.0f; if (duty_c > 1.0f) duty_c = 1.0f;

    // 记录占空比（0..1�????
    g_pwm_duty_a = duty_a;
    g_pwm_duty_b = duty_b;
    g_pwm_duty_c = duty_c;
    
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(duty_a * period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(duty_b * period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)(duty_c * period));
}
static float board_get_bus_voltage(void) {
    return 12.0f;
}
/* 组装新的HAL接口 */
FOC_HAL_t motor_hal = {
    .read_angle = board_read_angle,
    .read_currents = board_read_currents,
    .set_voltages = board_set_voltages,
    .get_bus_voltage = board_get_bus_voltage
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
AS5600_t as5600_l; // 编码器左�??????????
AS5600_t as5600_r; // 编码器右�??????????
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
  //触发adc采样
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
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
  HAL_Delay(10);
  /* 初始化ADC */
  ADC_Measure_Init();
  //初始化编码器
  AS5600_Init(&as5600_l, &hi2c1, 7);
  AS5600_Init(&as5600_r, &hi2c3, 7);
  /* 校准电流传感器和电机零点 */
  ADC_Calibrate_Current_Sensors();
  FOC_Init(&motor1, 7);  // 7对极
  FOC_SetHAL(&motor1, &motor_hal);
  FOC_CalibrateZeroOffset(&motor1);
  FOC_SetVoltageLimit(&motor1, 12.0f);  
  FOC_SetCurrentLimit(&motor1, 2.0f);  
  FOC_SetMode(&motor1, FOC_MODE_TORQUE);
  FOC_SetTarget(&motor1, 0.0f);
  HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    AS5600_Process(&as5600_l);
    AS5600_Process(&as5600_r);

    /* Vbus fixed at 12.0V; measurement disabled */
    key_scan();
    FOC_SetTarget(&motor1, iq_target);

    float ia = ADC_Get_Phase_Current_A();
    float ib = ADC_Get_Phase_Current_B();
    float ic = -(ia + ib);
    float id = FOC_GetCurrent_D(&motor1);
    float iq = FOC_GetCurrent_Q(&motor1);
  
      printf("Vbus:%.2f,%.3f, %.3f, %.3f, %.1f, %.1f, %.1f, %.3f, %.3f, %.3f\n",
             g_vbus_cached,
             iq_target,
             id, iq,
             g_pwm_duty_a * 100.0f, g_pwm_duty_b * 100.0f, g_pwm_duty_c * 100.0f,
             ia, ib, ic);
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  ADC转换完成回调函数
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 调用自定义的ADC DMA回调处理函数
    ADC_DMA_ConvCpltCallback(hadc);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	if (htim == &htim5) {
		if (ADC_Consume_Motor1_Ready()) {
			FOC_Update(&motor1);
		}
	}
	/* USER CODE END Callback 0 */


	/* USER CODE BEGIN Callback 1 */
	/* USER CODE END Callback 1 */
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
