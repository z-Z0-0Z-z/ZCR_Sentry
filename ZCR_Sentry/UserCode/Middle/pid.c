/*******************************************************************************
 * @file    pid.c
 * @brief   通用PID控制器实现（纯算法库，无全局状态、完全解耦）
 *
 * @note    本文件为完全解耦的纯算法模块：
 *          不定义任何全局调参参数，所有参数由调用方通过形参传入；
 *          哨兵工程的 PID 调参参数（g_param）统一放在 motor 模块中。
 *
 *          包含三种 PID 实现：
 *          位置式 PD / 偏航角 PI / 速度环增量式 PID
 *
 *          通用保护机制：
 *          - 输出限幅 out_max
 *          - 积分限幅 imax，用于偏航角 PI 控制器
 *          - 微分一阶低通滤波器，防止高频噪声放大，D_FILTER_ALPHA 可调
 *          - 增量式抗积分饱和 back-calculation，输出限幅时修正增量，防止积分深度饱和
 *
 *          调用顺序：速度环 PID -> 偏航角 PI -> 位置式 PD
 *******************************************************************************/

#include "pid.h"

/* 微分一阶低通滤波系数，范围 0~1，越小滤波越强
 * 配合 5ms 控制周期，alpha=0.5 约对应 64Hz 截止频率
 */
#define D_FILTER_ALPHA      (0.5f)

/* 增量式 dt 下限，防止上层传 0 或异常值时出错 */
#define SPEED_PID_DT_FLOOR  (0.001f)

/* 辅助函数：限幅 */
static float pid_constrain(float amt, float low, float high)
{
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

/* ============================================================
 *  位置式 PD
 * ============================================================ */
void PID_Pos_Init(PID_Pos_State_t *s)
{
    s->last_error  = 0.0f;
    s->last_deriv  = 0.0f;
    s->out         = 0.0f;
}

float PID_Pos_Calc(const PID_Pos_t *param, PID_Pos_State_t *s, float error, float dt_s)
{
    float p_out, d_out;
    float deriv;

    /* P 项 */
    p_out = param->kp * error;

    /* D 项（一阶低通滤波） */
    if (dt_s > 1e-6f) {
        deriv = (error - s->last_error) / dt_s;
    } else {
        deriv = 0.0f;
    }
    s->last_deriv = s->last_deriv * (1.0f - D_FILTER_ALPHA) + deriv * D_FILTER_ALPHA;
    d_out = param->kd * s->last_deriv;

    s->last_error = error;

    /* 输出限幅 */
    /* 低通滤波加权：0.5 * 上一帧输出 + 0.5 * 本次计算值 */
    s->out = (0.5f) * s->last_out + (0.5f) * pid_constrain(p_out + d_out, -param->out_max, param->out_max);
    s->last_out = pid_constrain(p_out + d_out, -param->out_max, param->out_max);

    return s->out;
}

/* ============================================================
 *  偏航角 PI
 * ============================================================ */
void PID_Yaw_Init(PID_Yaw_State_t *s)
{
    s->integrator  = 0.0f;
    s->last_error  = 0.0f;
    s->last_deriv  = 0.0f;   /* D 项滤波状态 */
    s->out         = 0.0f;
}

float PID_Yaw_Calc(const PID_Yaw_t *param, PID_Yaw_State_t *s, float target_omega, float gyro_z, float dt_s)
{
    float error;
    float p_out, i_out, d_out;
    float deriv;

    error = target_omega - gyro_z;

    /* P 项 */
    p_out = param->kp * error;

    /* I 项（带积分限幅） */
    s->integrator += error * dt_s;
    s->integrator = pid_constrain(s->integrator, -param->imax, param->imax);
    i_out = param->ki * s->integrator;

    /* D 项（一阶低通滤波）
     * @note  微分项对噪声敏感，建议配合 D_FILTER_ALPHA 低通滤波使用 Kd
     *        dt<=0 时跳过微分，正常情况不会触发
     */
    if (dt_s > 1e-6f) {
        deriv = (error - s->last_error) / dt_s;
    } else {
        deriv = 0.0f;
    }
    s->last_deriv = s->last_deriv * (1.0f - D_FILTER_ALPHA) + deriv * D_FILTER_ALPHA;
    d_out = param->kd * s->last_deriv;

    s->last_error = error;

    /* 输出限幅 */
    s->out = pid_constrain(p_out + i_out + d_out, -param->out_max, param->out_max);
    return s->out;
}

/* ============================================================
 *  速度环增量式 PID + back-calculation 抗积分饱和
 *
 *  增量式公式：
 *    du = Kp*(e[k] - e[k-1]) + Ki*e[k]*dt + Kd*(e[k] - 2*e[k-1] + e[k-2])/dt
 *    u[k] = u[k-1] + du
 *
 *  back-calculation：
 *    当 u[k] 超限（>out_max 或 <-out_max）时，限制 du 增量
 *    效果：防止积分深度饱和，使输出快速回到限幅内，避免超调
 * ============================================================ */
void PID_Speed_Init(PID_Speed_State_t *s)
{
    s->prev_error = 0.0f;
    s->last_error = 0.0f;
    s->last_out   = 0.0f;
    s->out        = 0.0f;
}

float PID_Speed_Calc(const PID_Speed_t *param, PID_Speed_State_t *s, float target_speed, float actual_speed, float dt_s)
{
    float error;
    float p_inc, i_inc, d_inc;
    float candidate;
    float dt;

    error = target_speed - actual_speed;

    /* dt 下限，防止上层传入 0 或异常值 */
    dt = (dt_s > SPEED_PID_DT_FLOOR) ? dt_s : SPEED_PID_DT_FLOOR;

    p_inc = param->kp * (error - s->last_error);
    //p_inc = pid_constrain(p_inc, -param->inc_max, param->inc_max);
    i_inc = param->ki * error * dt;
    //i_inc = pid_constrain(i_inc, -param->inc_max, param->inc_max);
    d_inc = (param->kd / dt) * (error - 2.0f * s->last_error + s->prev_error);

    /* 候选输出 = 上一帧输出 + 每帧增量变化 */
    candidate = s->last_out + pid_constrain(p_inc, -param->inc_max, param->inc_max) + pid_constrain(i_inc, -param->inc_max, param->inc_max) + pid_constrain(d_inc, -param->inc_max, param->inc_max);

    /* 输出限幅 + back-calculation
     * last_out 记录限幅前值，s->out 记录限幅后值，限幅时丢弃超出部分
     * 效果：防止积分饱和，使输出快速回到限幅内
     */
    s->out = pid_constrain(candidate, -param->out_max, param->out_max);

    /* 更新历史误差和输出 */
    s->prev_error = s->last_error;
    s->last_error = error;
    s->last_out   = s->out;

    return s->out;
}