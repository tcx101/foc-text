#ifndef FOC_HAL_STM32_H
#define FOC_HAL_STM32_H

#include "hal/foc_hal.h"
#include "tim.h"
#include "adc_measure.h"  /* MotorX_Get_Phase_Current_x() 原型 */
#include "as5600.h"       /* AS5600_GetAngleRad() 原型 (若使用) */

#ifdef __cplusplus
extern "C" {
#endif

/* 将 hal_* 接口映射到 STM32 实现，无额外 API */

#ifdef __cplusplus
}
#endif

#endif /* FOC_HAL_STM32_H */ 
