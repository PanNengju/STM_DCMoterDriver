/**
 * @file    motor_drv.h
 * @brief   直流电机驱动 — 纯驱动模块 API
 *          不含 MCU 配置、外设初始化、RTOS 调度
 */

#ifndef __MOTOR_DRV_H
#define __MOTOR_DRV_H

#include "motor_config.h"
#include "pid.h"
#include "encoder.h"
#include "pwm.h"
#include "adc.h"
#include "protection.h"
#include "state_machine.h"
#include "canopen/canopen.h"
#include "uart_cli.h"
#include "flash.h"

/*===========================================================================
 * 全局: 电流环定时器句柄 (main.c 初始化, motor_drv.c 的 ISR 使用)
 *===========================================================================*/
extern TIM_HandleTypeDef g_htim_current;

/*===========================================================================
 * 初始化 (不含外设初始化 — 外设在 main.c 中完成后再调用)
 *===========================================================================*/
void motor_drv_init(void);

/*===========================================================================
 * 控制循环任务入口 (由 main.c 通过 xTaskCreate 创建)
 *===========================================================================*/
void motor_task_speed_loop(void *pvParameters);
void motor_task_position_loop(void *pvParameters);
void motor_task_monitor(void *pvParameters);

/*===========================================================================
 * 电流环定时器 ISR (由 startup 向量表跳转)
 *===========================================================================*/
void CURRENT_LOOP_IRQHandler(void);

/*===========================================================================
 * 控制 API
 *===========================================================================*/
void motor_set_mode(control_mode_t mode);
void motor_set_speed(int32_t rpm_x100);
void motor_set_position(int32_t pulses, uint8_t relative);
void motor_set_torque(int32_t nm_x100);
void motor_stop(stop_mode_t mode);
void motor_emergency_stop(void);
uint8_t motor_fault_reset(void);
void motor_zero_sensor(uint8_t manual_mode);
void motor_zero_auto(void);

/*===========================================================================
 * 状态查询 API
 *===========================================================================*/
system_state_t motor_get_state(void);
control_mode_t motor_get_mode(void);
const system_feedback_t *motor_get_feedback(void);
float motor_get_speed_rpm(void);
int32_t motor_get_position(void);
float motor_get_current(void);
float motor_get_torque(void);

/*===========================================================================
 * 配置 API
 *===========================================================================*/
void motor_set_params(const motor_params_t *params);
void motor_set_pid_params(const motor_pid_params_t *params);
void motor_set_protection_thresholds(const protection_thresholds_t *thresholds);
uint8_t motor_params_save(void);
void motor_restore_factory(void);

#endif /* __MOTOR_DRV_H */
