#include "vofa.h"
#include "arm_math.h"

void vofa_as5600_show(void){
    float angle_l = AS5600_GetAngleRad(&as5600_l);
    float angle_r = AS5600_GetAngleRad(&as5600_r);
    float vel_l = -AS5600_GetVelRPM(&as5600_l);
    float vel_r = AS5600_GetVelRPM(&as5600_r);
    float average_vel = (vel_l + vel_r) / 2.0f;
    printf("Left Angle: %.2f,%.2f,%.2f,%.2f,%.3f\n", angle_l, angle_r, vel_l, vel_r, average_vel);
}

void vofa_currentLoop(void){
     float angle_l = AS5600_GetAngleRad(&as5600_l);
    float angle_r = AS5600_GetAngleRad(&as5600_r);
    printf("current:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",iq_target, motor1.iq, motor2.iq,angle_l,angle_r,motor1.ia,motor1.ib,motor1.ic);
}
