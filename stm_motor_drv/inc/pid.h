/**
 * @file    pid.h
 * @brief   直流电机驱动 — PID 控制器（抗积分饱和 + 前馈）
 */

#ifndef __PID_H
#define __PID_H

#include "motor_config.h"

/* PID 控制器实例 */
typedef struct {
    pid_params_t params;
    float        setpoint;
    float        feedback;
    float        error;
    float        error_prev;
    float        integral;
    float        derivative;
    float        output;
    float        feedforward;
    uint32_t     last_update_tick;
} pid_controller_t;

/* PID 初始化 */
void pid_init(pid_controller_t *pid, const pid_params_t *params);
void pid_reset(pid_controller_t *pid);

/**
 * @brief   PID 计算（增量式 + 抗积分饱和 + 微分先行）
 * @param   pid          PID 实例
 * @param   setpoint     设定值
 * @param   feedback     反馈值
 * @param   dt_sec       采样周期 (秒)
 * @param   ff_value     前馈值 (0=无前馈)
 * @return  控制输出
 */
float pid_update(pid_controller_t *pid, float setpoint, float feedback,
                 float dt_sec, float ff_value);

/* PID 参数在线修改 */
void pid_set_params(pid_controller_t *pid, const pid_params_t *params);
void pid_set_limits(pid_controller_t *pid, float integral_limit, float output_limit);
void pid_set_feedforward(pid_controller_t *pid, float ff);

/* 三环 PID 初始化 */
void motor_pid_init_all(motor_pid_params_t *params);

/* 获取全局 PID 实例 */
pid_controller_t *motor_pid_get_current(void);
pid_controller_t *motor_pid_get_speed(void);
pid_controller_t *motor_pid_get_position(void);

#endif /* __PID_H */
