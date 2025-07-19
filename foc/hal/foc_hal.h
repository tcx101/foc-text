#ifndef FOC_HAL_H
#define FOC_HAL_H

/* 该文件由使用者在适配 MCU 时实现。算法层仅依赖以下函数原型，不包含任何芯片头文件。*/

#include <stdint.h>
#include "alg/svpwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------ PWM 输出 ------ */
void hal_pwm_apply(const PWM_Duty_t *duty, uint16_t timer_period, uint8_t motor_id);

/* ------ ADC 原始读取 ------ */
float hal_get_phase_current_a(uint8_t motor_id);
float hal_get_phase_current_b(uint8_t motor_id);

/* ------ 传感器角度 ------ */
float hal_get_electrical_angle(uint8_t motor_id);
float hal_get_electrical_angle_raw(uint8_t motor_id); /* 新增: 用于校准的原始角度读取 */

/* ------ 延时与日志 ------ */
void hal_delay_ms(uint32_t ms);
void hal_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* FOC_HAL_H */ 
