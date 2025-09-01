#include "serial.h"
#include <stdarg.h>
#include <stdio.h>

IMU imu;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (unsigned char *)&ch, 1, 50);
  return ch;
}














