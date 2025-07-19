#include "key.h"
Key_t Key;
uint16_t text;
float iq_target = 0.00f;
//按键扫描
void key_scan(void){
    static uint32_t last_time =0;
    uint32_t time=HAL_GetTick();
    Key.key_state[0]= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); // 读取按键状态
    Key.key_state[1]= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); // 读取按键状态
    if (Key.key_state[0] == GPIO_PIN_RESET && Key.key_last[0] == GPIO_PIN_SET&&(time-last_time)>150) 
    {
         iq_target += 0.05f;
        last_time=time;   
    } 
    else if (Key.key_state[0] == GPIO_PIN_SET && Key.key_last[0] == GPIO_PIN_RESET&&(time-last_time)>150) 
    {
        last_time=time;
    }
 Key.key_last[0]=Key.key_state[0]; // 更新按键状态
 Key.key_last[1]=Key.key_state[1]; // 更新按键状态
    
}


