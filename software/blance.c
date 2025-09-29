#include "blance.h"
vertical vpid;//直立环环对象
IMU imu;//陀螺仪对象
//直立环参数设定
void blance_init(vertical*pid, float kp, float kd){
    pid->kp = kp;
    pid->kd = kd;
    pid->target = 0.0f;
}
//直立环
double blance_vertical(vertical*pid, float Angle, float gx){
    float err = pid->kp*(Angle - pid->target) +  pid->kd*gx;
    return err;
}
//控制回路
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim == &htim9)
  {
    FOC_UpdateCurrentLoop(&motor1, 0.0002f);
    FOC_UpdateCurrentLoop(&motor2, 0.0002f);
  }
  else if (htim == &htim5)
  {
    FOC_UpdateOuterLoop(&motor1, 0.001f);
    FOC_UpdateOuterLoop(&motor2, 0.001f);
  }
  else if (htim == &htim3)
  {
    AS5600_Process(&as5600_l);
    AS5600_Process(&as5600_r);
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
}
