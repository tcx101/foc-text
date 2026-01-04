#include "blance.h"
vertical vpid;//直立环环对象
IMU imu;//陀螺仪对象

// 双AS5600错开调度计数器
static uint8_t as5600_schedule_counter = 0;

//直立环参数设定
void blance_init(vertical*pid, float kp, float kd){
    pid->kp = kp;
    pid->kd = kd;
    pid->target = 0.0f;
}
//直立环
double blance_vertical(vertical*pid, float Angle, float gx){
    float err_iq = pid->kp*(Angle - pid->target) +  pid->kd*gx;
    return err_iq;
}
//控制回路
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim == &htim9)
  {//10khz电流环控制
    FOC_UpdateCurrentLoop(&motor1, 0.0002f);
    FOC_UpdateCurrentLoop(&motor2, 0.0002f);
  }
  else if (htim == &htim5)
   {//1khz速度环位置环控制
    FOC_UpdateOuterLoop(&motor1, 0.001f);
    FOC_UpdateOuterLoop(&motor2, 0.001f);
  }
  else if (htim == &htim3)
  {
    static uint8_t tim3_divider = 0;
    tim3_divider++;
    if (tim3_divider >= 2) {  // 250Hz / 2 = 125Hz
      tim3_divider = 0;
      
      if (as5600_schedule_counter == 0) {
        AS5600_Process(&as5600_l);  // I2C1
      } else {
        AS5600_Process(&as5600_r);  // I2C3
      }
      as5600_schedule_counter = !as5600_schedule_counter;
    }
  }

}
//陀螺仪参数显示
void windowMenu (IMU*imu){
    char text[50];
    sprintf(text,"R_Angle:%.2f",AS5600_GetAngle(&as5600_r));
    LCD_ShowString(20,60,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf(text,"RD_Angle:%.2f",AS5600_GetElecRad(&as5600_r));
    LCD_ShowString(20,80,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf(text,"L_Angle:%.2f",AS5600_GetAngle(&as5600_l));
    LCD_ShowString(20,100,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf(text,"LD_Angle:%.2f",AS5600_GetElecRad(&as5600_l));
    LCD_ShowString(20,120,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf(text,"R_Vel:%.2f",imu->ay);
    LCD_ShowString(20,140,(uint8_t*)text,BLUE,BLACK,16,0);
}
