/*
 * SimpleFOC - 重新设计的FOC控制库
 * 专注于核心功能，简洁易用
 */
#ifndef SIMPLEFOC_H
#define SIMPLEFOC_H

#include <stdint.h>
#include <stdbool.h>

// 数学常量
#define FOC_PI          3.14159265359f
#define FOC_2PI         6.28318530718f
#define FOC_SQRT3       1.73205080757f
#define FOC_SQRT3_2     0.86602540379f
#define FOC_INV_SQRT3   0.57735026919f

// 控制模式
typedef enum {
    FOC_MODE_TORQUE,        // 直接扭矩控制(Iq)
    FOC_MODE_VELOCITY,      // 速度控制
    FOC_MODE_POSITION       // 位置控制
} FOC_Mode_t;

// PID控制器
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_limit;
} FOC_PID_t;

// 硬件接口
typedef struct {
    // 必需的硬件接口
    float (*read_angle)(void);              // 读取机械角度(rad)
    void  (*read_currents)(float *ia, float *ib, float *ic);  // 读取三相电流
    void  (*set_voltages)(float va, float vb, float vc);      // 设置三相电压
    float (*get_bus_voltage)(void);         // 读取母线电压
} FOC_HAL_t;

// FOC实例
typedef struct {
    // 硬件接口
    FOC_HAL_t hal;
    
    // 电机参数
    uint8_t pole_pairs;
    float voltage_limit;        // 电压限制(V)
    float current_limit;        // 电流限制(A)
    // 电机等效参数（相）
    float rs;                   // 相电阻(Ω)
    float ld;                   // d轴电感(H)
    float lq;                   // q轴电感(H)
    bool use_decoupling;        // 是否启用Rs/交轴去耦前馈
    
    // 控制参数
    FOC_Mode_t mode;
    float target;               // 目标值(扭矩/速度/位置)
    FOC_PID_t pid_vel;         // 速度环PID
    FOC_PID_t pid_pos;         // 位置环PID
    FOC_PID_t pid_id;          // Id电流环PID
    FOC_PID_t pid_iq;          // Iq电流环PID
    
    // 状态变量
    float angle_mech;           // 机械角度(rad)
    float angle_elec;           // 电角度(rad)
    float velocity;             // 速度(rad/s)
    float ia, ib, ic;          // 三相电流(A)
    float id, iq;              // dq电流(A)
    float vd, vq;              // dq电压(V)
    float va, vb, vc;          // 三相电压(V)
    
    // 内部变量
    float elec_zero_offset;     // 电角度零点偏移
    float angle_prev;           // 上次角度(用于速度计算)
    bool initialized;
} FOC_Motor_t;

// API函数
bool FOC_Init(FOC_Motor_t *motor, uint8_t pole_pairs);
void FOC_SetHAL(FOC_Motor_t *motor, FOC_HAL_t *hal);
void FOC_SetMode(FOC_Motor_t *motor, FOC_Mode_t mode);
void FOC_SetTarget(FOC_Motor_t *motor, float target);
void FOC_SetCurrentLimit(FOC_Motor_t *motor, float limit);
void FOC_SetVoltageLimit(FOC_Motor_t *motor, float limit);
void FOC_ConfigPID_Velocity(FOC_Motor_t *motor, float kp, float ki, float kd, float limit);
void FOC_ConfigPID_Position(FOC_Motor_t *motor, float kp, float ki, float kd, float limit);
// 电机等效参数配置（单位：Rs[Ω], Ld/Lq[H]）
void FOC_ConfigMotorRL(FOC_Motor_t *motor, float rs, float ld, float lq, bool enable_decoupling);

// 主要控制函数
void FOC_Update(FOC_Motor_t *motor);
void FOC_CalibrateZeroOffset(FOC_Motor_t *motor);

// 开环测试/调试辅助：使用库内SVPWM将(d,q,theta)映射到三相并输出
void FOC_OpenLoop_SVPWM(FOC_Motor_t *motor, float vd, float vq, float theta);

// 获取状态函数
float FOC_GetAngle(FOC_Motor_t *motor);
float FOC_GetVelocity(FOC_Motor_t *motor);
float FOC_GetCurrent_D(FOC_Motor_t *motor);
float FOC_GetCurrent_Q(FOC_Motor_t *motor);

#endif // SIMPLEFOC_H
