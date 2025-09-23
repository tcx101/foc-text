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
#include "Allfile.h"
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
FOC_Motor_t motor1;
FOC_Motor_t motor2;
AS5600_t as5600_l;
AS5600_t as5600_r;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
  JY60DMA_Init();
  ADC_Measure_Init(); /* 初始化ADC */
  LCD_ShowString(10, 10, "ADC INIT", GREEN, BLACK, 32, 0);
  AS5600_Init(&as5600_l, &hi2c1, 7); // 初始化左轮编码器
  AS5600_Init(&as5600_r, &hi2c3, 7); // 初始化右轮编码器
  HAL_TIM_Base_Start_IT(&htim3);     // 启动编码器调度定时器，确保校准时角度连续更新
  LCD_ShowString(10, 40, "AS5600 INIT", GREEN, BLACK, 32, 0);
  ADC_Calibrate_Current_Sensors(); /* 校准电流传感器和电机零点 */
  LCD_ShowString(10, 70, "CURRENT SENSOR CALIBRATE", GREEN, BLACK, 32, 0);
  //FOC_Init(&motor1, 7); // 7对极
  FOC_Init(&motor2, 7);         // 7对极
  //FOC_AttachDefaultHAL(&motor1);//绑定各外设
  FOC_AttachMotor2HAL(&motor2); // 绑定各外设
  // FOC_SetVoltageLimit(&motor1, 12.0f);// 12V供电
  // FOC_ConfigMotorRL(&motor1, 2.3f, 0.00086f, 0.00086f, true);//电机参数
  // FOC_CalibrateDirection(&motor1);//方向校准
  // FOC_CalibrateZeroOffset(&motor1);//零点校准
  // FOC_SetCurrentLimit(&motor1, 2.0f);//设定电流限制
  // FOC_SetMode(&motor1, FOC_MODE_VELOCITY);//速度模式
  // FOC_SetTarget(&motor1, 0.0f); // 目标速度
  FOC_SetVoltageLimit(&motor2, 12.0f); // 12V供电
  FOC_ConfigMotorRL(&motor2, 2.3f, 0.00086f, 0.00086f, false);
  FOC_CalibrateDirection(&motor2);       // 方向校准
  FOC_CalibrateZeroOffset(&motor2);      // 零点校准
  FOC_SetCurrentLimit(&motor2, 2.0f);    // 设定电流限制
  FOC_SetMode(&motor2, FOC_MODE_TORQUE); // 转矩模式
  FOC_SetTarget(&motor2, 0.0f);          // 目标电流
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
  HAL_TIM_Base_Start_IT(&htim5); // 开启foc控制
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    key_openLoop();
    vofa_openLoop_m2();
    windowMenu(&imu);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // 调用自定义的ADC DMA回调处理函数
  ADC_DMA_ConvCpltCallback(hadc);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 */

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

#ifdef USE_FULL_ASSERT
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
