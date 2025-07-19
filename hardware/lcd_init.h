#ifndef __LCD_INIT_H
#define __LCD_INIT_H
#include "stm32f4xx_hal.h"


#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1

// 显示分辨率 240x280（可视区域）
#define LCD_W 240
#define LCD_H 280
#define LCD_size 16

#else
#define LCD_W 240  // 横屏下宽高互换，但列偏移保持不变
#define LCD_H 280
#endif

// 行列偏移（部分屏幕控制器留黑边，需要加偏移量）
#define COL_OFFSET 0   // 列偏移
#define ROW_OFFSET 20  // 行偏移

//-----------------LCD端口定义---------------- 


#define LCD_MOSI_Clr() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET)


#define LCD_SCLK_Clr() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET)


#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET)


#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET)


#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET)
 		     

#define LCD_CS_Clr()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET)

// 延时函数声明
void delay_ms(uint32_t ms);

void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




