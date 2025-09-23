#include "vofa.h"
//vofa+电流环调参
void vofa_currentLoop(void){
printf("iq_tgt:%.3f,%.3f,%.3f,%.3f,%.2f\n", motor1.target, FOC_GetCurrent_D(&motor1), FOC_GetCurrent_Q(&motor1),  AS5600_GetAngleRad(&as5600_r), AS5600_GetVelRPM(&as5600_r));
}
//vofa+速度环调参
void vofa_velocityLoop(void){
printf("vel_tgt:%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f\n", motor1.target, FOC_GetVelocity(&motor1), FOC_GetCurrent_D(&motor1), FOC_GetCurrent_Q(&motor1), AS5600_GetAngleRad(&as5600_r), AS5600_GetVelRPM(&as5600_r), FOC_GetVelocity(&motor2), FOC_GetCurrent_Q(&motor2));
}
void vofa_currentLoop_m2(void){
printf("m2_cur:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%3f,%.3f\n", motor2.ia, motor2.ib, motor2.ic, motor2.target, motor2.id, motor2.iq,AS5600_GetAngle(&as5600_r),AS5600_GetAngleRad(&as5600_r));
}

// vofa+开环控制调试
void vofa_openLoop_m2(void){
    // 输出：电角度，机械角度，三相电流，i_alpha，i_beta，id/iq(传感器角)，id_ol/iq_ol(开环角)
    float i_alpha, i_beta;
    FOC_Clarke(motor2.ia, motor2.ib, motor2.ic, &i_alpha, &i_beta);

    // 使用开环角度计算dq，便于判断相序与极性
    float cos_open = cosf(motor2.open_loop_angle);
    float sin_open = sinf(motor2.open_loop_angle);
    float id_ol = 0.0f, iq_ol = 0.0f;
    FOC_Park(i_alpha, i_beta, cos_open, sin_open, &id_ol, &iq_ol);

    printf("m2_open:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", 
           motor2.angle_elec,    // 电角度(rad)
           motor2.angle_mech,    // 机械角度(rad)
           motor2.ia,            // A相电流(A)
           motor2.ib,            // B相电流(A)
           motor2.ic,            // C相电流(A)
           i_alpha,              // α轴电流(A)
           i_beta,               // β轴电流(A)
           motor2.id,            // d轴电流(A) - 传感器角
           motor2.iq,            // q轴电流(A) - 传感器角
           id_ol,                // d轴电流(A) - 开环角
           iq_ol                 // q轴电流(A) - 开环角
    );
}

