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
  if (htim == &htim5)

  {
   // FOC_Update(&motor1);
    FOC_Update(&motor2);
  }
  else if (htim == &htim3)
  {
    AS5600_Process(&as5600_l);
    AS5600_Process(&as5600_r);
    imu.roll = JY60DMA_GetRoll();
    imu.gx = JY60DMA_GetGyroX();
  }
}
//陀螺仪参数显示
void windowMenu (IMU*imu){
    char text[50];
    sprintf (text,"roll:%.2f",imu->roll);
    LCD_ShowString(20,20,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf (text,"gx:%.2f",imu->gx);
    LCD_ShowString(20,40,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf(text,"Angle:%.2f",AS5600_GetAngle(&as5600_r));
    LCD_ShowString(20,60,(uint8_t*)text,BLUE,BLACK,16,0);
    sprintf(text,"D_Angle:%.2f",AS5600_GetElecRad(&as5600_r));
    LCD_ShowString(20,80,(uint8_t*)text,BLUE,BLACK,16,0);
}
