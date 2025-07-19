#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 寄存器地址 */
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_RAW_ANGLE_L  0x0D
#define AS5600_REG_ANGLE_H      0x0E
#define AS5600_REG_ANGLE_L      0x0F
#define AS5600_ADDRESS          (0x36 << 1)

/* AS5600 实例句柄，用于管理每个传感器的状态 */
typedef struct {
    I2C_HandleTypeDef* hi2c;         // 指向I2C句柄
    uint8_t            pole_pairs;   // 电机极对数
    
    // 内部状态
    uint8_t            rx_buf[2];    // 中断接收缓冲区
    volatile uint16_t  raw_angle;    // 最新的原始角度值 (12-bit)
    volatile float     elec_angle;   // 最新的电角度 (弧度)
    volatile uint8_t   read_pending; // 读取请求挂起标志
} AS5600_Handle_t;


/* ------ 新的非阻塞函数声明 ------ */

void AS5600_Init(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c, uint8_t pole_pairs);

/**
 * @brief  【非阻塞】发起一次角度读取请求 (中断模式)
 * @param  h AS5600句柄指针
 * @return HAL_StatusTypeDef HAL_OK表示成功发起请求
 */
HAL_StatusTypeDef AS5600_RequestAngle_IT(AS5600_Handle_t *h);

/**
 * @brief  【中断回调中调用】处理I2C接收完成事件
 * @param  h AS5600句柄指针
 * @note   此函数应在 HAL_I2C_MasterRxCpltCallback 中被调用
 */
void AS5600_RxCpltCallback(AS5600_Handle_t *h);

/**
 * @brief  【中断回调中调用】处理I2C错误事件
 * @param  h AS5600句柄指针
 * @note   此函数应在 HAL_I2C_ErrorCallback 中被调用
 */
void AS5600_ErrorCallback(AS5600_Handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */ 
