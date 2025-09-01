#ifndef __KEY_H
#define __KEY_H
#include "stm32f4xx_hal.h"
#include "simplefoc.h"


typedef struct {
    uint8_t key_state[2];  // 按键状态
    uint8_t key_last[2]; // 按键是否被按下
} Key_t;

extern Key_t Key;
extern float iq_target ;
extern float vel_target; // 将vel_target声明为外部变量


void key_scan(void);


#endif

