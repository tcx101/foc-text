#include "vofa.h"
//vofa+电流环调参
void vofa_currentLoop(void){
printf("iq_tgt:%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.2f\n", motor1.target, FOC_GetCurrent_D(&motor1), FOC_GetCurrent_Q(&motor1), AS5600_GetAngleRad(&as5600_l), AS5600_GetVelRPM(&as5600_l), AS5600_GetAngleRad(&as5600_r), AS5600_GetVelRPM(&as5600_r));
}
//vofa+速度环调参
