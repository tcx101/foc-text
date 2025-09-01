#include "key.h"
Key_t Key;//按键对象
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
        iq_target += 0.05f;
        if (iq_target > 2.0f) iq_target = 2.0f; // 限制最大电流为 2.0A
     
        last_time = time;   
    } 
    // 按键2按下 - 减少目标电流
    else if (Key.key_state[1] == GPIO_PIN_RESET && Key.key_last[1] == GPIO_PIN_SET && (time-last_time) > 150) 
    {
        iq_target -= 0.05f;
        if (iq_target < -2.0f) iq_target = -2.0f; // 限制最小电流为 -2.0A
   
        last_time = time;   
    } 
    Key.key_last[0] = Key.key_state[0]; // 更新按键1状态
    Key.key_last[1] = Key.key_state[1]; // 更新按键2状态
}


