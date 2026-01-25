#include "jy60.h"
#include <string.h>

#define JY60_ACCEL_RANGE_G 16.0f
#define JY60_GRAVITY 9.80665f
#define WT61_GYRO_SCALE (2000.0f / 32768.0f)
#define WT61_ANGLE_SCALE (180.0f / 32768.0f)

extern UART_HandleTypeDef huart4;

static JY60_Data_t s_data;
static float s_accel_scale = (JY60_ACCEL_RANGE_G * JY60_GRAVITY) / 32768.0f;

static uint8_t s_frame_buf[11];
static uint8_t s_frame_idx = 0;
static uint8_t s_rx_byte;

static HAL_StatusTypeDef JY60_WriteCommand(uint8_t addr, uint16_t data)
{
    uint8_t frame[5];
    frame[0] = 0xFF;
    frame[1] = 0xAA;
    frame[2] = addr;
    frame[3] = (uint8_t)(data & 0xFF);
    frame[4] = (uint8_t)((data >> 8) & 0xFF);
    return HAL_UART_Transmit(&huart4, frame, sizeof(frame), 100);
}

void JY60_Unlock(void)
{
    JY60_WriteCommand(0x69, 0xB588);
}

void JY60_Save(void)
{
    JY60_WriteCommand(0x00, 0x0000);
}

void JY60_SetAccelRangeG(float range_g)
{
    if (range_g <= 0.0f) return;
    s_accel_scale = (range_g * JY60_GRAVITY) / 32768.0f;
}

static void JY60_ParseByte(uint8_t byte)
{
    if (s_frame_idx == 0 && byte != 0x55)
        return;

    s_frame_buf[s_frame_idx++] = byte;

    if (s_frame_idx < 11)
        return;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++)
        sum += s_frame_buf[i];
    
    if (sum != s_frame_buf[10])
    {
        s_frame_idx = 0;
        return;
    }

    switch (s_frame_buf[1])
    {
    case 0x51:
    {
        int16_t ax_raw = (int16_t)((s_frame_buf[3] << 8) | s_frame_buf[2]);
        int16_t ay_raw = (int16_t)((s_frame_buf[5] << 8) | s_frame_buf[4]);
        int16_t az_raw = (int16_t)((s_frame_buf[7] << 8) | s_frame_buf[6]);
        int16_t t_raw = (int16_t)((s_frame_buf[9] << 8) | s_frame_buf[8]);
        
        s_data.ax = ((float)ax_raw) * s_accel_scale;
        s_data.ay = ((float)ay_raw) * s_accel_scale;
        s_data.az = ((float)az_raw) * s_accel_scale;
        s_data.temperature = ((float)t_raw) / 100.0f;
        break;
    }
    case 0x52:
    {
        int16_t wx_raw = (int16_t)((s_frame_buf[3] << 8) | s_frame_buf[2]);
        int16_t wy_raw = (int16_t)((s_frame_buf[5] << 8) | s_frame_buf[4]);
        int16_t wz_raw = (int16_t)((s_frame_buf[7] << 8) | s_frame_buf[6]);
        
        s_data.gx = ((float)wx_raw) * WT61_GYRO_SCALE;
        s_data.gy = ((float)wy_raw) * WT61_GYRO_SCALE;
        s_data.gz = ((float)wz_raw) * WT61_GYRO_SCALE;
        break;
    }
    case 0x53:
    {
        int16_t roll_raw = (int16_t)((s_frame_buf[3] << 8) | s_frame_buf[2]);
        int16_t pitch_raw = (int16_t)((s_frame_buf[5] << 8) | s_frame_buf[4]);
        int16_t yaw_raw = (int16_t)((s_frame_buf[7] << 8) | s_frame_buf[6]);
        
        s_data.roll = ((float)roll_raw) * WT61_ANGLE_SCALE;
        s_data.pitch = ((float)pitch_raw) * WT61_ANGLE_SCALE;
        s_data.yaw = ((float)yaw_raw) * WT61_ANGLE_SCALE;
        break;
    }
    default:
        break;
    }

    s_frame_idx = 0;
}

void JY60_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
}

bool JY60_GetData(JY60_Data_t *out)
{
    if (!out) return false;
    memcpy(out, &s_data, sizeof(JY60_Data_t));
    return true;
}

bool JY60_GetAccel(float *ax, float *ay, float *az)
{
    if (!ax || !ay || !az) return false;
    *ax = s_data.ax; *ay = s_data.ay; *az = s_data.az;
    return true;
}

bool JY60_GetGyro(float *gx, float *gy, float *gz)
{
    if (!gx || !gy || !gz) return false;
    *gx = s_data.gx; *gy = s_data.gy; *gz = s_data.gz;
    return true;
}

bool JY60_GetAngle(float *roll, float *pitch, float *yaw)
{
    if (!roll || !pitch || !yaw) return false;
    *roll = s_data.roll; *pitch = s_data.pitch; *yaw = s_data.yaw;
    return true;
}

float JY60_GetAccelX(void) { return s_data.ax; }
float JY60_GetAccelY(void) { return s_data.ay; }
float JY60_GetAccelZ(void) { return s_data.az; }

float JY60_GetGyroX(void) { return s_data.gx; }
float JY60_GetGyroY(void) { return s_data.gy; }
float JY60_GetGyroZ(void) { return s_data.gz; }

float JY60_GetRoll(void)  { return s_data.roll; }
float JY60_GetPitch(void) { return s_data.pitch; }
float JY60_GetYaw(void)   { return s_data.yaw; }

float JY60_GetTemperature(void) { return s_data.temperature; }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        JY60_ParseByte(s_rx_byte);
        HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1);
    }
}
