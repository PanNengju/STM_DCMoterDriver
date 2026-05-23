/**
 * @file    motor_drv.c
 * @brief   直流电机驱动 — 纯驱动模块 (控制算法 / 状态管理 / API)
 *
 * 三环控制:
 *   电流环 20kHz — 硬件定时器中断, ISR 在此文件
 *   速度环 1kHz  — FreeRTOS 任务, 入口 motor_task_speed_loop
 *   位置环 200Hz — FreeRTOS 任务, 入口 motor_task_position_loop
 *
 * 不含: 系统时钟 / 外设初始化 / RTOS 任务创建 — 这些在 main.c
 */

#include "motor_drv.h"
#include <string.h>

/*===========================================================================
 * 全局
 *===========================================================================*/
motor_system_t g_motor;
TIM_HandleTypeDef g_htim_current;

/*===========================================================================
 * 内部辅助
 *===========================================================================*/
static void current_loop_run(void);
static void speed_ramp_calc(int32_t target, int32_t *current, uint32_t accel, float dt);

/*===========================================================================
 * 初始化 — 仅初始化驱动内部状态
 * 前置条件: main.c 已完成 HAL_Init / 时钟 / 外设 / Flash 加载
 *===========================================================================*/
void motor_drv_init(void)
{
    memset(&g_motor, 0, sizeof(motor_system_t));

    flash_init();
    flash_params_load();

    motor_pid_init_all(&g_motor.pid_params);

    g_motor.state           = SYS_STATE_INIT;
    g_motor.ctrl_mode       = CTRL_MODE_NONE;
    g_motor.sw_position_min = -1000000;
    g_motor.sw_position_max =  1000000;
    g_motor.last_comm_tick  = HAL_GetTick();
    g_motor.mutex           = xSemaphoreCreateMutex();

    state_machine_init();
    state_machine_transition(SYS_STATE_DISABLE);

    LOG_INFO("MOTOR", "Driver initialized");
}

/*===========================================================================
 * 电流环定时器 ISR
 *===========================================================================*/
void CURRENT_LOOP_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_htim_current, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&g_htim_current, TIM_FLAG_UPDATE);
        current_loop_run();
    }
}

/*===========================================================================
 * 电流环 (20kHz) — 最内环, 速度前馈
 *===========================================================================*/
static void current_loop_run(void)
{
    if (g_motor.state != SYS_STATE_RUNNING) return;

    pid_controller_t *pid_cur = motor_pid_get_current();
    float cur_a = adc_get_current_a();
    float volt  = adc_get_voltage_v();
    float dt    = 1.0f / (float)CURRENT_LOOP_FREQ_HZ;

    float ff = 0.0f;
    if (g_motor.ctrl_mode == CTRL_MODE_SPEED && volt > 1.0f) {
        float target_rpm = (float)g_motor.speed_demand_rpm_x100 / 100.0f;
        ff = target_rpm * g_motor.motor_params.bemf_constant / 1000.0f / volt * 1000.0f;
    }

    float out = pid_update(pid_cur, pid_cur->setpoint, cur_a, dt, ff);
    out = CLAMP(out, -1000.0f, 1000.0f);
    pwm_set_duty((int16_t)out);
}

/*===========================================================================
 * 速度环 (1kHz FreeRTOS 任务)
 *===========================================================================*/
void motor_task_speed_loop(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));

        if (g_motor.state != SYS_STATE_RUNNING) continue;
        if (g_motor.ctrl_mode != CTRL_MODE_SPEED
            && g_motor.ctrl_mode != CTRL_MODE_POSITION) continue;

        pid_controller_t *pid_spd = motor_pid_get_speed();
        int32_t measured = encoder_get_speed_rpm_x100();
        float dt = 1.0f / (float)SPEED_LOOP_FREQ_HZ;

        float target;
        if (g_motor.ctrl_mode == CTRL_MODE_POSITION) {
            target = pid_spd->setpoint;
        } else {
            speed_ramp_calc(g_motor.target_speed_rpm_x100,
                           &g_motor.speed_demand_rpm_x100,
                           g_motor.profile_accel ? g_motor.profile_accel : 10000,
                           dt);
            target = (float)g_motor.speed_demand_rpm_x100;
        }

        float out = pid_update(pid_spd, target, (float)measured, dt, 0.0f);
        motor_pid_get_current()->setpoint = out / 1000.0f;

        g_motor.feedback.speed_rpm_x100 = measured;
    }
}

/*===========================================================================
 * 位置环 (200Hz FreeRTOS 任务)
 *===========================================================================*/
void motor_task_position_loop(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));

        if (g_motor.state != SYS_STATE_RUNNING) continue;
        if (g_motor.ctrl_mode != CTRL_MODE_POSITION) continue;

        pid_controller_t *pid_pos = motor_pid_get_position();
        int32_t pos = encoder_get_position();
        float dt = 1.0f / (float)POSITION_LOOP_FREQ_HZ;

        int32_t target = CLAMP(g_motor.target_position_pulses,
                               g_motor.sw_position_min,
                               g_motor.sw_position_max);

        float out = pid_update(pid_pos, (float)target, (float)pos, dt, 0.0f);
        motor_pid_get_speed()->setpoint = out;

        g_motor.feedback.position_pulses = pos;

        if (g_motor.pos_sequence_len > 0 && ABS(pos - target) < 5) {
            if (++g_motor.pos_sequence_idx < g_motor.pos_sequence_len) {
                g_motor.target_position_pulses =
                    g_motor.pos_sequence[g_motor.pos_sequence_idx];
            } else {
                g_motor.pos_sequence_len = 0;
            }
        }
    }
}

/*===========================================================================
 * 监控任务 (100Hz) — 保护检查 / 状态机 / 喂狗
 *===========================================================================*/
void motor_task_monitor(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        protection_check_all();
        state_machine_process();
        pds_state_machine_process();

#if STM32_PLATFORM == STM32_PLATFORM_F1
        HAL_IWDG_Refresh(&g_motor.hiwdg);
#else
        HAL_WWDG_Refresh(&g_motor.hwwdg);
#endif
    }
}

/*===========================================================================
 * 梯形速度曲线
 *===========================================================================*/
static void speed_ramp_calc(int32_t target, int32_t *current,
                            uint32_t accel, float dt)
{
    if (accel == 0 || target == *current) return;
    int32_t step = (int32_t)((float)accel * dt);
    if (step < 1) step = 1;
    if (target > *current) {
        *current += step;
        if (*current > target) *current = target;
    } else {
        *current -= step;
        if (*current < target) *current = target;
    }
}

/*===========================================================================
 * 控制 API
 *===========================================================================*/
void motor_set_mode(control_mode_t mode)
{
    xSemaphoreTake(g_motor.mutex, portMAX_DELAY);
    g_motor.ctrl_mode = mode;
    pid_reset(motor_pid_get_speed());
    pid_reset(motor_pid_get_position());
    xSemaphoreGive(g_motor.mutex);
}

void motor_set_speed(int32_t rpm_x100)
{
    xSemaphoreTake(g_motor.mutex, portMAX_DELAY);
    g_motor.target_speed_rpm_x100 = CLAMP(rpm_x100, -300000, 300000);
    g_motor.ctrl_mode = CTRL_MODE_SPEED;
    if (g_motor.state == SYS_STATE_READY)
        state_machine_transition(SYS_STATE_RUNNING);
    xSemaphoreGive(g_motor.mutex);
}

void motor_set_position(int32_t pulses, uint8_t relative)
{
    xSemaphoreTake(g_motor.mutex, portMAX_DELAY);
    g_motor.ctrl_mode = CTRL_MODE_POSITION;
    g_motor.target_position_pulses = relative
        ? encoder_get_position() + pulses : pulses;
    g_motor.target_position_pulses = CLAMP(g_motor.target_position_pulses,
                                           g_motor.sw_position_min,
                                           g_motor.sw_position_max);
    if (g_motor.state == SYS_STATE_READY)
        state_machine_transition(SYS_STATE_RUNNING);
    xSemaphoreGive(g_motor.mutex);
}

void motor_set_torque(int32_t nm_x100)
{
    xSemaphoreTake(g_motor.mutex, portMAX_DELAY);
    g_motor.target_torque_nm_x100 = nm_x100;
    g_motor.ctrl_mode = CTRL_MODE_TORQUE;
    float target_a = (float)nm_x100 / 100.0f / g_motor.motor_params.torque_constant;
    motor_pid_get_current()->setpoint = target_a;
    if (g_motor.state == SYS_STATE_READY)
        state_machine_transition(SYS_STATE_RUNNING);
    xSemaphoreGive(g_motor.mutex);
}

void motor_stop(stop_mode_t mode)
{
    xSemaphoreTake(g_motor.mutex, portMAX_DELAY);
    state_machine_transition(SYS_STATE_READY);
    pwm_stop(mode);
    pid_reset(motor_pid_get_current());
    pid_reset(motor_pid_get_speed());
    pid_reset(motor_pid_get_position());
    xSemaphoreGive(g_motor.mutex);
}

void motor_emergency_stop(void)     { protection_estop_trigger(); }
uint8_t motor_fault_reset(void)     { return protection_fault_reset(); }

void motor_zero_sensor(uint8_t manual)
{
    if (manual) {
        int32_t raw = encoder_get_raw_count();
        encoder_set_zero_offset(raw);
        flash_save_zero_offset(raw);
        LOG_INFO("MOTOR", "Manual zero: offset=%ld", (long)raw);
    } else {
        motor_zero_auto();
    }
}

void motor_zero_auto(void)
{
    g_motor.ctrl_mode = CTRL_MODE_HOMING;
    int32_t spd = (int32_t)g_motor.homing_speed_switch;
    if (g_motor.homing_method == 33) motor_set_speed(spd * 100);
    else if (g_motor.homing_method == 17) motor_set_speed(-spd * 100);
    LOG_INFO("MOTOR", "Auto homing: speed=%ld", (long)spd);
}

/*===========================================================================
 * 状态查询 API
 *===========================================================================*/
system_state_t motor_get_state(void)               { return g_motor.state; }
control_mode_t motor_get_mode(void)                { return g_motor.ctrl_mode; }
const system_feedback_t *motor_get_feedback(void)  { return &g_motor.feedback; }
float motor_get_speed_rpm(void)                    { return (float)g_motor.feedback.speed_rpm_x100 / 100.0f; }
int32_t motor_get_position(void)                   { return encoder_get_position(); }
float motor_get_current(void)                      { return adc_get_current_a(); }
float motor_get_torque(void)                       { return adc_get_current_a() * g_motor.motor_params.torque_constant; }

/*===========================================================================
 * 配置 API
 *===========================================================================*/
void motor_set_params(const motor_params_t *p)              { memcpy(&g_motor.motor_params, p, sizeof(motor_params_t)); }
void motor_set_pid_params(const motor_pid_params_t *p)     { memcpy(&g_motor.pid_params, p, sizeof(motor_pid_params_t)); motor_pid_init_all(&g_motor.pid_params); }
void motor_set_protection_thresholds(const protection_thresholds_t *t) { memcpy(&g_motor.prot_thresholds, t, sizeof(protection_thresholds_t)); }
uint8_t motor_params_save(void)                             { return flash_params_save(); }
void motor_restore_factory(void)                            { flash_restore_factory(); motor_pid_init_all(&g_motor.pid_params); }
