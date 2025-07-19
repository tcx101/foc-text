#include "foc_hal_stm32.h"
#include <stdarg.h>
#include <stdio.h>

/* FOC 核心句柄，用于访问零点偏移 */
#include "core/foc_core.h"
extern FOC_Handle_t foc_motor0;
extern FOC_Handle_t foc_motor1;

/* 新增: 外部声明 AS5600 句柄 */
#include "as5600.h"
extern AS5600_Handle_t as5600_m0;
extern AS5600_Handle_t as5600_m1;

/* ---------------- PWM 定时器映射 ---------------- */
static TIM_HandleTypeDef* get_pwm_timer(uint8_t motor_id)
{
    return (motor_id == 0) ? &htim2 : &htim4; /* Motor0->TIM2, Motor1->TIM4 */
}

static const uint32_t pwm_channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

void hal_pwm_apply(const PWM_Duty_t *d, uint16_t period, uint8_t motor_id)
{
    TIM_HandleTypeDef *htim = get_pwm_timer(motor_id);
    if (!htim) return;

    uint32_t ta = (uint32_t)(d->ta * period);
    uint32_t tb = (uint32_t)(d->tb * period);
    uint32_t tc = (uint32_t)(d->tc * period);

    /* 统一配置: 两个电机都使用已知的正确配置 (交换A,C相) */
    __HAL_TIM_SET_COMPARE(htim, pwm_channels[0], tc);
    __HAL_TIM_SET_COMPARE(htim, pwm_channels[1], tb);
    __HAL_TIM_SET_COMPARE(htim, pwm_channels[2], ta);
}

/* ---------------- 相电流 ---------------- */
float hal_get_phase_current_a(uint8_t motor_id)
{
    // 恢复原来的电流方向
    if (motor_id == 0) {
        return Motor1_Get_Phase_Current_A();
    } else {
        return Motor2_Get_Phase_Current_A(); // 去掉负号，恢复原来的方向
    }
}

float hal_get_phase_current_b(uint8_t motor_id)
{
    // 恢复原来的电流方向
    if (motor_id == 0) {
        return Motor1_Get_Phase_Current_B();
    } else {
        return Motor2_Get_Phase_Current_B(); // 去掉负号，恢复原来的方向
    }
}

/* ---------------- 电角度 (修改后) ---------------- */

/* 校准时使用：直接读取传感器的原始电角度 */
float hal_get_electrical_angle_raw(uint8_t motor_id)
{
    // 直接返回句柄中最新的电角度值
    return (motor_id == 0) ? as5600_m0.elec_angle : as5600_m1.elec_angle;
}

/* 运行时使用：返回减去零点偏移后的正确角度 */
float hal_get_electrical_angle(uint8_t motor_id)
{
    if (motor_id == 0)
    {
        return as5600_m0.elec_angle - foc_motor0.zero_electrical_offset;
    }
    else
    {
        return as5600_m1.elec_angle - foc_motor1.zero_electrical_offset;
    }
}

/* ---------------- 系统辅助 ---------------- */
void hal_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void hal_log(const char *fmt, ...)
{
#ifdef DEBUG
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
#else
    (void)fmt;
#endif
} 
