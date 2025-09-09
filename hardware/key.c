#include "key.h"
Key_t Key;//按键对象
extern FOC_Motor_t motor1;//电机对象
float iq_target = 0.00f;// 目标q轴电流
float vel_target = 0.00f; // 目标速度
// 声明外部变量
//按键扫描
void key_scan(void){
    static uint32_t last_time = 0;
    uint32_t time = HAL_GetTick();
    Key.key_state[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); // 读取按键1状态
    Key.key_state[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); // 读取按键2状态
    // 按键1按下 - 增加目标电流
    if (Key.key_state[0] == GPIO_PIN_RESET && Key.key_last[0] == GPIO_PIN_SET && (time-last_time) > 150) 
    {
        // 小功率电机使用更精细的步进：0.02A (原来0.05A)
        iq_target += 0.02f;  
        if (iq_target > 1.8f) iq_target = 1.8f; // 限制最大电流为 1.8A (电机最大电流的90%)
         FOC_SetTarget(&motor1,iq_target);
        last_time = time;   
    } 
    // 按键2按下 - 减少目标电流
    else if (Key.key_state[1] == GPIO_PIN_RESET && Key.key_last[1] == GPIO_PIN_SET && (time-last_time) > 150) 
    {
        // 小功率电机使用更精细的步进：0.02A (原来0.05A)
        iq_target -= 0.02f;  
        if (iq_target < -1.8f) iq_target = -1.8f; // 限制最小电流为 -1.8A
        FOC_SetTarget(&motor1,iq_target);
        last_time = time;   
    } 
    Key.key_last[0] = Key.key_state[0]; // 更新按键1状态
    Key.key_last[1] = Key.key_state[1]; // 更新按键2状态
}


