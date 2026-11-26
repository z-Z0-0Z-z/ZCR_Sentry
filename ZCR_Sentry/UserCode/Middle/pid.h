#ifndef __PID_H__
#define __PID_H__

#include "main.h"

//定值参数
typedef struct {
    float kp;
    float kd;
    float out_max;        /* 输出限幅（目标速度限幅，度/秒） */
} PID_Pos_t;

typedef struct {
    float kp;
    float ki;
    float kd;             /* 微分项 Kd*err_diff/dt，带一阶低通滤波 */
    float imax;           /* 积分限幅 */
    float out_max;        /* 输出限幅 (m/s) */
} PID_Yaw_t;

typedef struct {
    float kp;
    float ki;
    float kd;             /* 微分项 Kd*err_diff/dt */
    float out_max;        /* 输出限幅 PWM 占空比 (0~10000) */
    float inc_max;        /* 增量限幅，P/I/D 各项每帧 PWM 变化限制，建议 300~800 */
} PID_Speed_t;

//历史记录
typedef struct {
    float last_error;       /* 上一次误差 */
    float last_deriv;       /* 上一次微分值（滤波后） */
    float out;              /* 当前输出 */
    float last_out;         /* 上一次输出 */
} PID_Pos_State_t;

typedef struct {
    float integrator;       /* 积分累加器 */
    float last_error;       /* 上一次误差 */
    float last_deriv;       /* 上一次微分值（一阶低通滤波后） */
    float out;              /* 当前输出 */
} PID_Yaw_State_t;

typedef struct {
    float prev_error;       /* 前两次误差 e[k-2] */
    float last_error;       /* 上一次误差 e[k-1] */
    float last_out;         /* 上一次输出 u[k-1]（增量式累加基础） */
    float out;              /* 当前输出 */
} PID_Speed_State_t;


void PID_Pos_Init(PID_Pos_State_t *s);
void PID_Yaw_Init(PID_Yaw_State_t *s);
void PID_Speed_Init(PID_Speed_State_t *s);

/* 位置式 PD 计算
 * @param param   PID 参数（kp, kd, out_max）
 * @param s       状态
 * @param error   偏差（循线取值/位置偏差/角度偏差等）
 * @param dt_s    时间步长（秒）
 * @return        目标角速度 (度/秒)
 */
float PID_Pos_Calc(const PID_Pos_t *param, PID_Pos_State_t *s, float error, float dt_s);

/* 偏航角 PI 计算
 * @param param        PID 参数（kp, ki, imax, out_max）
 * @param s            状态
 * @param target_omega 目标角速度 (度/秒)
 * @param gyro_z       实际角速度 (度/秒)
 * @param dt_s         时间步长（秒）
 * @return             目标转向量 (m/s)
 */
float PID_Yaw_Calc(const PID_Yaw_t *param, PID_Yaw_State_t *s, float target_omega, float gyro_z, float dt_s);

/* 速度环 PID 计算（增量式 + back-calculation 抗积分饱和）
 * @param param        PID 参数（kp, ki, kd, out_max, inc_max）
 * @param s            状态
 * @param target_speed 目标速度 (mm/s)
 * @param actual_speed 实际速度 (mm/s)
 * @param dt_s         时间步长（秒）
 * @return             控制电流值范围（+/- 16384），也可以是PWM占空比
 */
float PID_Speed_Calc(const PID_Speed_t *param, PID_Speed_State_t *s, float target_speed, float actual_speed, float dt_s);

#endif