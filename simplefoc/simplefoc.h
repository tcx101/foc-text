/*
 * SimpleFOC - 重新设计的FOC控制库
 * 专注于核心功能，简洁易用
 */
#ifndef __SIMPLEFOC_H__
#define __SIMPLEFOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// 数学常数
#define FOC_PI          3.14159265358979323846f
#define FOC_2PI         (2.0f * FOC_PI)
#define FOC_PI_2        (FOC_PI / 2.0f)
#define FOC_SQRT3       1.73205080756887729353f
#define FOC_SQRT3_2     (FOC_SQRT3 / 2.0f)
#define FOC_INV_SQRT3   0.57735026918962576451f

// PID控制器结构体
typedef struct {
    float kp;           // 比例增益
    float ki;           // 积分增益  
    float kd;           // 微分增益
    float error_prev;   // 上次误差
    float integral;     // 积分累积
    float output_limit; // 输出限制
} FOC_PID_t;

// 硬件抽象层接口
typedef struct {
    float (*read_angle)(void);                          // 读取角度(弧度)
    void (*read_currents)(float *ia, float *ib, float *ic); // 读取三相电流
    void (*set_voltages)(float va, float vb, float vc); // 设置三相电压
    float (*get_bus_voltage)(void);                     // 获取母线电压
} FOC_HAL_t;

// FOC控制模式
typedef enum {
    FOC_MODE_TORQUE = 0,    // 转矩控制模式
    FOC_MODE_VELOCITY,      // 速度控制模式  
    FOC_MODE_POSITION       // 位置控制模式
} FOC_Mode_t;

// FOC电机控制结构体
typedef struct {
    // 电机物理参数
    uint8_t pole_pairs;     // 极对数
    float rs;               // 相电阻(Ω)
    float ld;               // d轴电感(H)
    float lq;               // q轴电感(H)
    
    // 控制参数
    float voltage_limit;    // 电压限制(V)
    float current_limit;    // 电流限制(A)
    FOC_Mode_t mode;        // 控制模式
    float target;           // 目标值
    
    // 传感器数据
    float angle_mech;       // 机械角度(rad)
    float angle_elec;       // 电角度(rad)
    float velocity;         // 角速度(rad/s)
    float elec_zero_offset; // 电角度零偏移
    
    // 三相电流
    float ia, ib, ic;       // 三相电流(A)
    
    // dq坐标系电流
    float id, iq;           // dq轴电流(A)
    
    // dq坐标系电压
    float vd, vq;           // dq轴电压(V)
    
    // 三相电压/占空比
    float va, vb, vc;       // 三相电压(V)
    
    // PID控制器
    FOC_PID_t pid_id;       // d轴电流环PID
    FOC_PID_t pid_iq;       // q轴电流环PID
    FOC_PID_t pid_vel;      // 速度环PID
    FOC_PID_t pid_pos;      // 位置环PID
    
    // 内部状态
    float angle_prev;       // 上次角度
    uint32_t last_update_us; // 上次更新时间(微秒)
    bool initialized;       // 初始化标志
    bool use_decoupling;    // 是否使用解耦
    int8_t sensor_direction; // 传感器方向(+1/-1)
    
    // 硬件接口
    FOC_HAL_t hal;
    
} FOC_Motor_t;

// === 核心FOC函数 ===
bool FOC_Init(FOC_Motor_t *motor, uint8_t pole_pairs);
void FOC_SetHAL(FOC_Motor_t *motor, FOC_HAL_t *hal);
void FOC_Update(FOC_Motor_t *motor);
// 默认板级HAL绑定（把板载ADC/AS5600/TIM绑定到motor->hal）
void FOC_AttachDefaultHAL(FOC_Motor_t *motor);

// === 配置函数 ===
void FOC_SetMode(FOC_Motor_t *motor, FOC_Mode_t mode);
void FOC_SetTarget(FOC_Motor_t *motor, float target);
void FOC_SetVoltageLimit(FOC_Motor_t *motor, float limit);
void FOC_SetCurrentLimit(FOC_Motor_t *motor, float limit);
void FOC_ConfigMotorRL(FOC_Motor_t *motor, float rs, float ld, float lq, bool use_decoupling);
void FOC_SetSensorDirection(FOC_Motor_t *motor, int8_t direction);

// === 校准函数 ===
void FOC_CalibrateZeroOffset(FOC_Motor_t *motor);
void FOC_CalibrateDirection(FOC_Motor_t *motor);

// === 状态获取函数 ===
float FOC_GetCurrent_D(FOC_Motor_t *motor);
float FOC_GetCurrent_Q(FOC_Motor_t *motor);
float FOC_GetVelocity(FOC_Motor_t *motor);
float FOC_GetAngle(FOC_Motor_t *motor);

// === PID控制函数 ===
void FOC_PID_Init(FOC_PID_t *pid, float kp, float ki, float kd, float output_limit);
float FOC_PID_Update(FOC_PID_t *pid, float error, float dt);
void FOC_PID_Reset(FOC_PID_t *pid);

// === 数学工具函数 ===
float FOC_NormalizeAngle(float angle);
void FOC_Clarke(float ia, float ib, float ic, float *alpha, float *beta);
void FOC_Park(float alpha, float beta, float cos_theta, float sin_theta, float *d, float *q);
void FOC_InvPark(float d, float q, float cos_theta, float sin_theta, float *alpha, float *beta);
void FOC_SVPWM(float alpha, float beta, float vdc, float *va, float *vb, float *vc);

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLEFOC_H__ */
