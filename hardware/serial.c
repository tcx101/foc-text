#include "serial.h"
#include <stdarg.h>

IMU imu;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (unsigned char *)&ch, 1, 50);
  return ch;
}

/**
 * @brief 打印日志信息（使用printf实现）
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void print_log(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}




