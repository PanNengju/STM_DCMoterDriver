/**
 * @file    pwm.h
 * @brief   直流电机驱动 — PWM 输出模块（H桥 / 死区互补输出）
 */

#ifndef __PWM_H
#define __PWM_H

#include "motor_config.h"

/* 停止方式 */
typedef enum {
    STOP_MODE_COAST  = 0,           /* 惯性停止 (全关) */
    STOP_MODE_BRAKE  = 1,           /* 主动制动 (下桥导通) */
} stop_mode_t;

/* PWM 初始化 */
void pwm_init(void);

/**
 * @brief   设置 PWM 占空比
 * @param   duty 占空比 -1000 ~ +1000
 *          + = 正转, - = 反转, 0 = 停止
 *          幅值 1000 = 100%
 */
void pwm_set_duty(int16_t duty);

/* 立即关闭 PWM 输出 */
void pwm_emergency_stop(stop_mode_t mode);

/* 正常停止 */
void pwm_stop(stop_mode_t mode);

/* 获取当前占空比 */
int16_t pwm_get_duty(void);

/* PWM 使能/失能 */
void pwm_enable(uint8_t enable);

#endif /* __PWM_H */
