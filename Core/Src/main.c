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
#include "lcd.h"
#include "lcd_init.h"
#include "adc_measure.h"
#include <stdio.h>
#include "as5600.h"
#include "jy60.h"
#include "key.h"
#include <math.h>
/* -------- 新增 FOC 相关头文�?? -------- */
#include "foc_core.h"
#include "foc_hal_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 删除了旧�?? extern uint16_t AS5600_ReadRawAngle
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3; /* 新增 M1 �?? I2C 句柄 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---------- 新增: FOC 全局句柄与参�?? ---------- */
MotorParam_t motor_param = {
    .pole_pairs = 7,
    .v_bus_nominal = 12.0f,
    .current_limit = 2.0f};
FOC_Handle_t foc_motor0;
FOC_Handle_t foc_motor1;

/* 新增: AS5600 句柄实例 */
AS5600_Handle_t as5600_m0;
AS5600_Handle_t as5600_m1;

/* 新增: 引用ADC采样缓冲区 */
extern uint16_t motor1_adc_buffer[2];
extern uint16_t motor2_adc_buffer[2];

/* 删除了旧的全�??角度变量 */
// volatile float g_motor0_elec_angle = 0.0f;
// volatile float g_motor1_elec_angle = 0.0f;

/* 新增: 声明来自 key.c 的外部变�?? */
extern float iq_target;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化LCD与ADC测量模块 */
  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
  // 初始化陀螺仪
  JY60DMA_Init();
  /* 清屏，准备显示ADC调试信息 */
  /* 初始化ADC测量模块 */
  ADC_Measure_Init();
  ADC_Calibrate_Current_Sensors();

  /* -------- 新增: FOC �? AS5600 初始�? -------- */
  AS5600_Init(&as5600_m0, &hi2c1, motor_param.pole_pairs);
  AS5600_Init(&as5600_m1, &hi2c3, motor_param.pole_pairs);

  FOC_HandleInit(&foc_motor0, 0, &motor_param);
  FOC_HandleInit(&foc_motor1, 1, &motor_param);
   FOC_Calibrate(&foc_motor0); 
   FOC_Calibrate(&foc_motor1); 
  FOC_SetMode(&foc_motor0, FOC_MODE_CURRENT);
  FOC_SetMode(&foc_motor1, FOC_MODE_CURRENT);
  FOC_SetCurrentTarget(&foc_motor0, 0.0f, iq_target);
  FOC_SetCurrentTarget(&foc_motor1, 0.0f, iq_target);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  /* 启动 TIM3 1ms 中断 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    key_scan();
    /* 将按键调节后的新目标值，实时设置给FOC控制�?? */
    FOC_SetCurrentTarget(&foc_motor0, 0.0f, iq_target);
    FOC_SetCurrentTarget(&foc_motor1, 0.0f, iq_target);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1. 删除了主循环中的角度读取代码 */
    
    /* 2. 低频刷新LCD */
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick > 100) 
    {
      last_tick = HAL_GetTick();
      /* --- �?终显示界�? --- */
      /* 目标�? (白色) */
      LCD_ShowString(0, 0, (const uint8_t *)"Iq Ref:", WHITE, BLACK, 16, 0);
      LCD_ShowFloatNum1(88, 0, iq_target, 4, WHITE, BLACK, 16);

      /* M0 实际�? (绿色) */
      LCD_ShowString(0, 16, (const uint8_t *)"M0 IqAct:", GREEN, BLACK, 16, 0);
      LCD_ShowFloatNum1(88, 16, foc_motor0.park_cur.q, 4, GREEN, BLACK, 16);

      /* M1 实际�? (黄色) */
      LCD_ShowString(0, 32, (const uint8_t *)"M1 IqAct:", YELLOW, BLACK, 16, 0);
      LCD_ShowFloatNum1(88, 32, foc_motor1.park_cur.q, 4, YELLOW, BLACK, 16);
      
      /* 两个电机的q轴电压比�? */
      LCD_ShowString(0, 48, (const uint8_t *)"Vq:", WHITE, BLACK, 16, 0);
      LCD_ShowFloatNum1(88, 48, foc_motor0.dq_voltage.q, 4, GREEN, BLACK, 16);
      LCD_ShowFloatNum1(128, 48, foc_motor1.dq_voltage.q, 4, YELLOW, BLACK, 16);
      
      /* 电机电阻估计 (V/I) */
      float m0_resistance = 0.0f;
      if (fabsf(foc_motor0.park_cur.q) > 0.05f) { // 避免除以接近零的�?
        m0_resistance = fabsf(foc_motor0.dq_voltage.q) / fabsf(foc_motor0.park_cur.q);
      }
      
      float m1_resistance = 0.0f;
      if (fabsf(foc_motor1.park_cur.q) > 0.05f) { // 避免除以接近零的�?
        m1_resistance = fabsf(foc_motor1.dq_voltage.q) / fabsf(foc_motor1.park_cur.q);
      }
      
      LCD_ShowString(0, 64, (const uint8_t *)"R(V/I):", WHITE, BLACK, 16, 0);
      LCD_ShowFloatNum1(88, 64, m0_resistance, 4, GREEN, BLACK, 16);
      LCD_ShowFloatNum1(128, 64, m1_resistance, 4, YELLOW, BLACK, 16);
      
      /* 定时器配置比�? */
      LCD_ShowString(0, 80, (const uint8_t *)"T Period:", WHITE, BLACK, 16, 0);
      LCD_ShowIntNum(88, 80, htim2.Init.Period, 4, GREEN, BLACK, 16);
      LCD_ShowIntNum(128, 80, htim4.Init.Period, 4, YELLOW, BLACK, 16);
      
      /* ADC采样原始值显示 - A相 */
      LCD_ShowString(0, 96, (const uint8_t *)"ADC A:", WHITE, BLACK, 16, 0);
      LCD_ShowIntNum(88, 96, motor1_adc_buffer[0], 4, GREEN, BLACK, 16);
      LCD_ShowIntNum(128, 96, motor2_adc_buffer[0], 4, YELLOW, BLACK, 16);
      
      /* ADC采样原始值显示 - B相 */
      LCD_ShowString(0, 112, (const uint8_t *)"ADC B:", WHITE, BLACK, 16, 0);
      LCD_ShowIntNum(88, 112, motor1_adc_buffer[1], 4, GREEN, BLACK, 16);
      LCD_ShowIntNum(128, 112, motor2_adc_buffer[1], 4, YELLOW, BLACK, 16);
      
      /* ADC电压值显示 - A相 */
      float v_m1a = (motor1_adc_buffer[0] / 4096.0f) * 3.3f;
      float v_m2a = (motor2_adc_buffer[0] / 4096.0f) * 3.3f;
      LCD_ShowString(0, 128, (const uint8_t *)"VA:", WHITE, BLACK, 16, 0);
      LCD_ShowFloatNum1(88, 128, v_m1a, 4, GREEN, BLACK, 16);
      LCD_ShowFloatNum1(128, 128, v_m2a, 4, YELLOW, BLACK, 16);
    }
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
 * @brief 定时器中断回调函�??
 * @param htim 定时器句�??
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 判断是否�?? TIM3 中断 (4ms 周期) */
  if (htim->Instance == TIM3)
  {
    /* 中断中恢复FOC执行 */
 AS5600_RequestAngle_IT(&as5600_m0);
 AS5600_RequestAngle_IT(&as5600_m1);
 // 2. 使用上一次完成的角度，执行FOC计算 (dt = 0.004f for 4ms interrupt)
 FOC_RunCurrentLoop(&foc_motor0, 0.004f);
 FOC_RunCurrentLoop(&foc_motor1, 0.004f);
  }
}

// I2C 传输完成回调
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    AS5600_RxCpltCallback(&as5600_m0);
  }
  else if (hi2c->Instance == I2C3) {
    AS5600_RxCpltCallback(&as5600_m1);
  }
}

// I2C 错误回调
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    AS5600_ErrorCallback(&as5600_m0);
  }
  else if (hi2c->Instance == I2C3) {
    AS5600_ErrorCallback(&as5600_m1);
  }
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
