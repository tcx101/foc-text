#include "jy60.h"
#include "serial.h"
#include "stm32f4xx_hal_dma.h"
#include <string.h>

/* ------------------ 可配置常量（按手册） ------------------ */
#ifndef JY60_ACCEL_RANGE_G
#define JY60_ACCEL_RANGE_G     16.0f         /* 默认 ±16g */
#endif
#ifndef JY60_GRAVITY
#define JY60_GRAVITY           9.80665f      /* m/s^2 */
#endif

/* ------------------ 外设及 DMA 缓存区 ------------------ */
extern UART_HandleTypeDef huart4;          /* 由 CubeMX 生成 */
#define JY60DMA_RX_BUF_LEN   256
static uint8_t s_rx_buf[JY60DMA_RX_BUF_LEN];

/* ------------------ 内部状态及数据 ------------------ */
static JY60DMA_Data_t s_data;

/* 运行期可调缩放：raw -> m/s^2 */
static float s_accel_scale = (JY60_ACCEL_RANGE_G * JY60_GRAVITY) / 32768.0f;

/* 解析状态机缓存 */
static uint8_t s_frame_buf[11];
static uint8_t s_frame_idx = 0;

/* WT61协议数据转换参数 - 根据官方手册 */
/* 
 * 根据WT61手册：
 * 加速度：X=((AxH<<8)|AxL)/32768*16g (g为重力加速度，可取9.8m/s²)
 * 角速度：X=((WxH<<8)|WxL)/32768*2000°/s  
 * 角度：X=((RollH<<8)|RollL)/32768*180°
 */
#define WT61_GYRO_SCALE     (2000.0f / 32768.0f)         /* ±2000°/s量程 */
#define WT61_ANGLE_SCALE    (180.0f / 32768.0f)          /* ±180°量程 */

/* 通用 0xFF 0xAA 写命令（低字节在前） */
static HAL_StatusTypeDef JY60_WriteCommand(uint8_t addr, uint16_t data)
{
    uint8_t frame[5];
    frame[0] = 0xFF;
    frame[1] = 0xAA;
    frame[2] = addr;
    frame[3] = (uint8_t)(data & 0xFF);       /* DATAL */
    frame[4] = (uint8_t)((data >> 8) & 0xFF);/* DATAH */
    return HAL_UART_Transmit(&huart4, frame, sizeof(frame), 100);
}

void JY60_Unlock(void)
{
    /* 解锁：FF AA 69 88 B5 */
    (void)JY60_WriteCommand(0x69, 0xB588);
}

void JY60_Save(void)
{
    /* 保存：FF AA 00 00 00 */
    (void)JY60_WriteCommand(0x00, 0x0000);
}

/* 允许外部在运行时调整量程或单位（g->m/s^2） */
void JY60_SetAccelRangeG(float range_g)
{
    if (range_g <= 0.0f) return;
    s_accel_scale = (range_g * JY60_GRAVITY) / 32768.0f;
}

/* 解析一个字节 */
static void JY60DMA_ParseByte(uint8_t byte)
{
    if (s_frame_idx == 0 && byte != 0x55)
        return; /* 未对齐, 等待帧头 */

    s_frame_buf[s_frame_idx++] = byte;

    if (s_frame_idx < 11)
        return; /* 未收满一帧 */

    /* 校验和验证 */
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++)
        sum += s_frame_buf[i];
    if (sum != s_frame_buf[10])
    {
        s_frame_idx = 0; /* 校验失败，丢弃此帧 */
        return;
    }

    /* 功能字解析 */
    switch (s_frame_buf[1])
    {
    case 0x51: /* 加速度输出 */
    {
        /* 按照手册格式：AxL AxH AyL AyH AzL AzH TL TH */
        uint8_t AxL = s_frame_buf[2];
        uint8_t AxH = s_frame_buf[3];
        uint8_t AyL = s_frame_buf[4]; 
        uint8_t AyH = s_frame_buf[5];
        uint8_t AzL = s_frame_buf[6];
        uint8_t AzH = s_frame_buf[7];
        uint8_t TL = s_frame_buf[8];
        uint8_t TH = s_frame_buf[9];
        
        /* 组合成16位有符号数 */
        int16_t ax_raw = (int16_t)((AxH << 8) | AxL);
        int16_t ay_raw = (int16_t)((AyH << 8) | AyL);
        int16_t az_raw = (int16_t)((AzH << 8) | AzL);
        int16_t t_raw = (int16_t)((TH << 8) | TL);
        
        /* raw -> m/s^2 */
        s_data.ax = ((float)ax_raw) * s_accel_scale;
        s_data.ay = ((float)ay_raw) * s_accel_scale;
        s_data.az = ((float)az_raw) * s_accel_scale;
        
        /* 温度计算：温度=((TH<<8)|TL) / 100 ℃ */
        s_data.temperature = ((float)t_raw) / 100.0f;
        break;
    }
    case 0x52: /* 角速度输出 */
    {
        /* 按照手册格式：WxL WxH WyL WyH WzL WzH VolL VolH */
        uint8_t WxL = s_frame_buf[2];
        uint8_t WxH = s_frame_buf[3];
        uint8_t WyL = s_frame_buf[4];
        uint8_t WyH = s_frame_buf[5]; 
        uint8_t WzL = s_frame_buf[6];
        uint8_t WzH = s_frame_buf[7];
        
        /* 组合成16位有符号数 */
        int16_t wx_raw = (int16_t)((WxH << 8) | WxL);
        int16_t wy_raw = (int16_t)((WyH << 8) | WyL);
        int16_t wz_raw = (int16_t)((WzH << 8) | WzL);
        
        /* 按照手册公式转换：X=((WxH<<8)|WxL)/32768*2000°/s */
        s_data.gx = ((float)wx_raw) * WT61_GYRO_SCALE;
        s_data.gy = ((float)wy_raw) * WT61_GYRO_SCALE;
        s_data.gz = ((float)wz_raw) * WT61_GYRO_SCALE;
        break;
    }
    case 0x53: /* 角度输出 */
    {
        /* 按照手册格式：RollL RollH PitchL PitchH YawL YawH VL VH */
        uint8_t RollL = s_frame_buf[2];
        uint8_t RollH = s_frame_buf[3];
        uint8_t PitchL = s_frame_buf[4];
        uint8_t PitchH = s_frame_buf[5];
        uint8_t YawL = s_frame_buf[6];
        uint8_t YawH = s_frame_buf[7];
        
        /* 组合成16位有符号数 */
        int16_t roll_raw = (int16_t)((RollH << 8) | RollL);
        int16_t pitch_raw = (int16_t)((PitchH << 8) | PitchL);
        int16_t yaw_raw = (int16_t)((YawH << 8) | YawL);
        
        /* 按照手册公式转换：滚转角X=((RollH<<8)|RollL)/32768*180° */
        s_data.roll = ((float)roll_raw) * WT61_ANGLE_SCALE;
        s_data.pitch = ((float)pitch_raw) * WT61_ANGLE_SCALE;
        s_data.yaw = ((float)yaw_raw) * WT61_ANGLE_SCALE;
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
    /* 关闭半满中断 - 避免不必要的中断 */
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
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
