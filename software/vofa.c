#include "vofa.h"
#include "arm_math.h"

void vofa_as5600_show(void){
    printf("Left Angle: %.2f,%.2f,%.3f\n", motor1.angle_elec, motor1.angle_mech, AS5600_GetVelRPM(&as5600_l));
}

void vofa_currentLoop(void){
    // 打印：目标iq, 实际iq, 实际id, 机械角度, 电角度, 零偏移, ia, ib, ic
    float angle1 =AS5600_GetAngleRad(&as5600_l);
    float angle2 =AS5600_GetAngleRad(&as5600_r);
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        motor1.target,          // 目标q轴电流
        motor1.iq,              // 实际q轴电流
        motor1.id,              // 实际d轴电流
        motor2.iq,              // 目标q轴电流
        motor2.id,              // 实际q轴电流
        angle1,
        angle2
    );
}

/**
 * @brief 开环测试 - 打印三相电流和dq轴电流
 * @note 用于验证ADC采样和Clarke/Park变换是否正确
 */
void vofa_openloop_test(void){
    // 打印格式：ia, ib, ic, id, iq, 电角度, 机械角度, Vq
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        motor1.ia,           // A相电流
        motor1.ib,           // B相电流
        motor1.ic,           // C相电流
        motor2.ia,           // q轴电流
        motor2.ib,           // A相电流
        motor2.ic,           // B相电流
        motor2.angle_elec,   // 电角度
        motor2.angle_mech   // 机械角度
    );
}



