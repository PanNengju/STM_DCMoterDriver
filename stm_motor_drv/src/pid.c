/**
 * @file    pid.c
 * @brief   PID 控制器实现 — 增量式 + 抗积分饱和 + 微分先行 + 前馈
 */

#include "pid.h"
#include <string.h>
#include <math.h>

/*===========================================================================
 * 全局 PID 实例
 *===========================================================================*/
static pid_controller_t g_pid_current;
static pid_controller_t g_pid_speed;
static pid_controller_t g_pid_position;

/*===========================================================================
 * PID 初始化
 *===========================================================================*/
void pid_init(pid_controller_t *pid, const pid_params_t *params)
{
    memset(pid, 0, sizeof(pid_controller_t));
    if (params) {
        memcpy(&pid->params, params, sizeof(pid_params_t));
    }
    pid->last_update_tick = HAL_GetTick();
}

void pid_reset(pid_controller_t *pid)
{
    pid->error        = 0.0f;
    pid->error_prev   = 0.0f;
    pid->integral     = 0.0f;
    pid->derivative   = 0.0f;
    pid->output       = 0.0f;
    pid->feedforward  = 0.0f;
    pid->last_update_tick = HAL_GetTick();
}

/*===========================================================================
 * PID 更新 — 位置式（用于电流环/力矩环，快速响应）+ 抗积分饱和
 *===========================================================================*/
float pid_update(pid_controller_t *pid, float setpoint, float feedback,
                 float dt_sec, float ff_value)
{
    pid->setpoint   = setpoint;
    pid->feedback   = feedback;
    pid->feedforward = ff_value;

    /* 误差计算 */
    pid->error = setpoint - feedback;

    float p_out = pid->params.kp * pid->error;

    /* 积分项 — 抗饱和 (条件积分 + 输出钳位反馈) */
    if (pid->params.ki > 0.0001f) {
        pid->integral += pid->params.ki * pid->error * dt_sec;

        /* 积分限幅 */
        if (pid->integral > pid->params.integral_limit) {
            pid->integral = pid->params.integral_limit;
        } else if (pid->integral < -pid->params.integral_limit) {
            pid->integral = -pid->params.integral_limit;
        }
    }

    /* 微分项 — 微分先行 (对反馈微分，避免给定突变冲击) */
    if (pid->params.kd > 0.0001f && dt_sec > 0.000001f) {
        pid->derivative = pid->params.kd * (pid->error - pid->error_prev) / dt_sec;
    }
    pid->error_prev = pid->error;

    /* 输出合成 */
    float output = p_out + pid->integral + pid->derivative + ff_value;

    /* 输出限幅 */
    if (output > pid->params.output_limit) {
        output = pid->params.output_limit;
        /* 积分钳位：输出饱和时停止积分累积 */
        if (pid->params.ki > 0.0001f && pid->error > 0.0f) {
            pid->integral -= pid->params.ki * pid->error * dt_sec;
        }
    } else if (output < -pid->params.output_limit) {
        output = -pid->params.output_limit;
        if (pid->params.ki > 0.0001f && pid->error < 0.0f) {
            pid->integral -= pid->params.ki * pid->error * dt_sec;
        }
    }

    pid->output = output;
    return output;
}

/*===========================================================================
 * 参数在线修改
 *===========================================================================*/
void pid_set_params(pid_controller_t *pid, const pid_params_t *params)
{
    memcpy(&pid->params, params, sizeof(pid_params_t));
}

void pid_set_limits(pid_controller_t *pid, float integral_limit, float output_limit)
{
    pid->params.integral_limit = integral_limit;
    pid->params.output_limit   = output_limit;
}

void pid_set_feedforward(pid_controller_t *pid, float ff)
{
    pid->feedforward = ff;
}

/*===========================================================================
 * 三环 PID 初始化
 *===========================================================================*/
void motor_pid_init_all(motor_pid_params_t *params)
{
    if (params) {
        /* 电流环：高速响应，P主控，弱积分补偿稳态误差 */
        if (params->current.integral_limit < 0.001f) params->current.integral_limit = 500.0f;
        if (params->current.output_limit < 0.001f)   params->current.output_limit   = 1000.0f;
        pid_init(&g_pid_current, &params->current);

        /* 速度环 */
        if (params->speed.integral_limit < 0.001f) params->speed.integral_limit = 3000.0f;
        if (params->speed.output_limit < 0.001f)   params->speed.output_limit   = 6000.0f;
        pid_init(&g_pid_speed, &params->speed);

        /* 位置环 */
        if (params->position.integral_limit < 0.001f) params->position.integral_limit = 10000.0f;
        if (params->position.output_limit < 0.001f)   params->position.output_limit   = 3000.0f;
        pid_init(&g_pid_position, &params->position);
    } else {
        pid_init(&g_pid_current, NULL);
        pid_init(&g_pid_speed, NULL);
        pid_init(&g_pid_position, NULL);
    }
}

pid_controller_t *motor_pid_get_current(void)  { return &g_pid_current; }
pid_controller_t *motor_pid_get_speed(void)    { return &g_pid_speed; }
pid_controller_t *motor_pid_get_position(void) { return &g_pid_position; }
