#include "jy60.h"
#include "serial.h"
#include <string.h>

/* ------------------ 外设及 DMA 缓存区 ------------------ */
extern UART_HandleTypeDef huart4;          /* 由 CubeMX 生成 */
#define JY60DMA_RX_BUF_LEN   256
static uint8_t s_rx_buf[JY60DMA_RX_BUF_LEN];

/* ------------------ 内部状态及数据 ------------------ */
static JY60DMA_Data_t s_data;

/* 解析状态机缓存 */
static uint8_t s_frame_buf[11];
static uint8_t s_frame_idx = 0;

/* 将 16bit 原始值转换为浮点, full_scale 对应满量程 */
static inline float int16_to_float(int16_t raw, float full_scale)
{
    return ((float)raw) / 32768.0f * full_scale;
}

/* 解析一个字节 */
static void JY60DMA_ParseByte(uint8_t byte)
{
    if (s_frame_idx == 0 && byte != 0x55)
        return; /* 未对齐, 等待帧头 */

    s_frame_buf[s_frame_idx++] = byte;

    if (s_frame_idx < 11)
        return; /* 未收满一帧 */

    /* 校验和 */
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++)
        sum += s_frame_buf[i];
    if (sum != s_frame_buf[10])
    {
        s_frame_idx = 0; /* 丢帧 */
        return;
    }

    /* 功能字解析 */
    switch (s_frame_buf[1])
    {
    case 0x51: /* 加速度 + 温度 */
    {
        int16_t ax = (int16_t)(s_frame_buf[3] << 8 | s_frame_buf[2]);
        int16_t ay = (int16_t)(s_frame_buf[5] << 8 | s_frame_buf[4]);
        int16_t az = (int16_t)(s_frame_buf[7] << 8 | s_frame_buf[6]);
        int16_t t  = (int16_t)(s_frame_buf[9] << 8 | s_frame_buf[8]);
        s_data.ax = int16_to_float(ax, 16.0f) * 9.80665f;
        s_data.ay = int16_to_float(ay, 16.0f) * 9.80665f;
        s_data.az = int16_to_float(az, 16.0f) * 9.80665f;
        s_data.temperature = ((float)t) / 100.0f;
        break;
    }
    case 0x52: /* 角速度 */
    {
        int16_t gx = (int16_t)(s_frame_buf[3] << 8 | s_frame_buf[2]);
        int16_t gy = (int16_t)(s_frame_buf[5] << 8 | s_frame_buf[4]);
        int16_t gz = (int16_t)(s_frame_buf[7] << 8 | s_frame_buf[6]);
        s_data.gx = int16_to_float(gx, 2000.0f);
        s_data.gy = int16_to_float(gy, 2000.0f);
        s_data.gz = int16_to_float(gz, 2000.0f);
        break;
    }
    case 0x53: /* 角度 */
    {
        int16_t roll  = (int16_t)(s_frame_buf[3] << 8 | s_frame_buf[2]);
        int16_t pitch = (int16_t)(s_frame_buf[5] << 8 | s_frame_buf[4]);
        int16_t yaw   = (int16_t)(s_frame_buf[7] << 8 | s_frame_buf[6]);
        s_data.roll  = int16_to_float(roll, 180.0f);
        s_data.pitch = int16_to_float(pitch, 180.0f);
        s_data.yaw   = int16_to_float(yaw, 180.0f);
        break;
    }
    default:
        break;
    }

    s_frame_idx = 0; /* 等待下一帧 */
}

/* 批量解析 DMA 接收数据 */
static void JY60DMA_ParseBuffer(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        JY60DMA_ParseByte(buf[i]);
}

/* ------------------ 公共 API ------------------ */
void JY60DMA_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));

    /* 启动 DMA + IDLE 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_rx_buf, JY60DMA_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT); /* 关闭半满中断 */
}

bool JY60DMA_GetData(JY60DMA_Data_t *out)
{
    if (!out) return false;
    memcpy(out, &s_data, sizeof(JY60DMA_Data_t));
    return true;
}

bool JY60DMA_GetAccel(float *ax, float *ay, float *az)
{
    if (!ax || !ay || !az) return false;
    *ax = s_data.ax; *ay = s_data.ay; *az = s_data.az;
    return true;
}

bool JY60DMA_GetGyro(float *gx, float *gy, float *gz)
{
    if (!gx || !gy || !gz) return false;
    *gx = s_data.gx; *gy = s_data.gy; *gz = s_data.gz;
    return true;
}

bool JY60DMA_GetAngle(float *roll, float *pitch, float *yaw)
{
    if (!roll || !pitch || !yaw) return false;
    *roll = s_data.roll; *pitch = s_data.pitch; *yaw = s_data.yaw;
    return true;
}

float JY60DMA_GetAccelX(void) { return s_data.ax; }
float JY60DMA_GetAccelY(void) { return s_data.ay; }
float JY60DMA_GetAccelZ(void) { return s_data.az; }

float JY60DMA_GetGyroX(void) { return s_data.gx; }
float JY60DMA_GetGyroY(void) { return s_data.gy; }
float JY60DMA_GetGyroZ(void) { return s_data.gz; }

float JY60DMA_GetRoll(void)  { return s_data.roll; }
float JY60DMA_GetPitch(void) { return s_data.pitch; }
float JY60DMA_GetYaw(void)   { return s_data.yaw; }

float JY60DMA_GetTemperature(void) { return s_data.temperature; }

/* HAL 回调：DMA + 空闲中断接收完成 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart4)
    {
        JY60DMA_ParseBuffer(s_rx_buf, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_rx_buf, JY60DMA_RX_BUF_LEN);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }
   
            }
