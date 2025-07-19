#ifndef __KEY_H
#define __KEY_H
#include "stm32f4xx_hal.h"
typedef struct {
    uint8_t key_state[2];  // 按键状态
    uint8_t key_last[2]; // 按键是否被按下
} Key_t;
extern Key_t Key;
extern float iq_target ;
void key_scan(void);
#endif

