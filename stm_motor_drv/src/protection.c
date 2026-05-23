/**
 * @file    protection.c
 * @brief   保护报警模块 — 8种故障检测 / 分级处理 / EMCY上报 / LED指示
 *          F1/F4 兼容
 */

#include "protection.h"
#include "pwm.h"
#include "state_machine.h"
#include "encoder.h"
#include "adc.h"
#include "uart_cli.h"
#include "canopen/co_emcy.h"
#include <string.h>

static alarm_record_t g_alarm_history[ALARM_HISTORY_MAX];
static uint8_t g_alarm_history_idx;
static uint8_t g_alarm_history_count;
static uint32_t g_last_emcy_time_ms;

void protection_init(void)
{
    memset(g_alarm_history, 0, sizeof(g_alarm_history));
    g_alarm_history_idx   = 0;
    g_alarm_history_count = 0;
    g_last_emcy_time_ms   = 0;

    /* 急停按钮 GPIO — 外部中断 */
    ESTOP_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin   = ESTOP_GPIO_PIN;
    gpio_init.Mode  = GPIO_MODE_IT_FALLING;
    gpio_init.Pull  = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ESTOP_GPIO_PORT, &gpio_init);
    HAL_NVIC_SetPriority(ESTOP_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ESTOP_EXTI_IRQn);

    /* 限位开关 */
    gpio_init.Pin  = LIMIT_MIN_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LIMIT_MIN_GPIO_PORT, &gpio_init);
    gpio_init.Pin  = LIMIT_MAX_GPIO_PIN;
    HAL_GPIO_Init(LIMIT_MAX_GPIO_PORT, &gpio_init);

    /* LED */
    LED_GPIO_CLK_ENABLE();
    gpio_init.Pin   = LED_GREEN_GPIO_PIN | LED_YELLOW_GPIO_PIN | LED_RED_GPIO_PIN;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GREEN_GPIO_PORT, &gpio_init);

    HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_PORT, LED_RED_GPIO_PIN, GPIO_PIN_RESET);
}

alarm_level_t protection_check_all(void)
{
    float cur_a  = adc_get_current_a();
    float volt   = adc_get_voltage_v();
    float t_drv  = adc_get_temp_driver_c();
    float t_mot  = adc_get_temp_motor_c();
    int32_t spd  = encoder_get_speed_rpm_x100();
    int32_t pos  = encoder_get_position();
    float tor    = cur_a * g_motor.motor_params.torque_constant;

    g_motor.feedback.speed_rpm_x100  = spd;
    g_motor.feedback.position_pulses = pos;
    g_motor.feedback.current_ma      = (int32_t)(cur_a * 1000.0f);
    g_motor.feedback.torque_nm_x100  = (int16_t)(tor * 100.0f);
    g_motor.feedback.bus_voltage_mv  = (uint16_t)(volt * 1000.0f);
    g_motor.feedback.temp_driver_c   = (uint8_t)(t_drv + 40.0f);
    g_motor.feedback.temp_motor_c    = (uint8_t)(t_mot + 40.0f);

    alarm_level_t lvl = ALARM_LVL_NORMAL;
    protection_thresholds_t *th = &g_motor.prot_thresholds;

    if (cur_a > th->overcurrent_threshold)                       lvl = ALARM_LVL_CRITICAL;
    if (volt > th->overvoltage_threshold && lvl < ALARM_LVL_CRITICAL) lvl = ALARM_LVL_CRITICAL;
    if (ABS(spd) < 100 && cur_a > th->stall_current)             lvl = ALARM_LVL_CRITICAL;
    if (encoder_check_fault())                                   lvl = ALARM_LVL_CRITICAL;
    if (t_drv > th->overtemp_driver && lvl < ALARM_LVL_CRITICAL)  lvl = ALARM_LVL_CRITICAL;
    if (t_mot > th->overtemp_motor && lvl < ALARM_LVL_CRITICAL)   lvl = ALARM_LVL_CRITICAL;
    if (tor > th->overload_torque && lvl < ALARM_LVL_RECOVERABLE)  lvl = ALARM_LVL_RECOVERABLE;
    if (volt < th->undervoltage_threshold && volt > 0.1f
        && lvl < ALARM_LVL_RECOVERABLE)                          lvl = ALARM_LVL_RECOVERABLE;
    if ((HAL_GetTick() - g_motor.last_comm_tick) > th->comm_timeout_ms
        && lvl < ALARM_LVL_RECOVERABLE)                          lvl = ALARM_LVL_RECOVERABLE;

    if (lvl >= ALARM_LVL_CRITICAL) {
        error_code_t err = ERR_OVERCURRENT;
        if (volt > th->overvoltage_threshold) err = ERR_OVERVOLTAGE;
        else if (t_drv > th->overtemp_driver) err = ERR_OVERTEMP_DRIVER;
        else if (t_mot > th->overtemp_motor)  err = ERR_OVERTEMP_MOTOR;
        protection_handle_fault(err, lvl);
    } else if (lvl == ALARM_LVL_RECOVERABLE) {
        error_code_t err = ERR_OVERLOAD;
        if (volt < th->undervoltage_threshold && volt > 0.1f) err = ERR_UNDERVOLTAGE;
        protection_handle_fault(err, lvl);
    }

    protection_update_led(lvl);
    g_motor.alarm_level = lvl;

    /* 更新状态字 */
    g_motor.feedback.status_byte =
        (g_motor.is_enabled  ? 0x01 : 0x00) |
        (g_motor.is_running   ? 0x02 : 0x00) |
        (g_motor.error_count > 0 ? 0x04 : 0x00) |
        (g_motor.is_estopped ? 0x08 : 0x00);

    return lvl;
}

/* --- 单项检查 --- */
uint8_t protection_check_overcurrent(float a)    { return a > g_motor.prot_thresholds.overcurrent_threshold; }
uint8_t protection_check_overvoltage(float v)     { return v > g_motor.prot_thresholds.overvoltage_threshold; }
uint8_t protection_check_undervoltage(float v)    { return (v < g_motor.prot_thresholds.undervoltage_threshold && v > 0.1f); }
uint8_t protection_check_overtemp_driver(float t) { return t > g_motor.prot_thresholds.overtemp_driver; }
uint8_t protection_check_overtemp_motor(float t)  { return t > g_motor.prot_thresholds.overtemp_motor; }
uint8_t protection_check_overload(float tor, float dur_s) {
    return (tor > g_motor.prot_thresholds.overload_torque && dur_s > g_motor.prot_thresholds.overload_time);
}
uint8_t protection_check_stall(int32_t spd, float cur, float dur_s) {
    return (ABS(spd) < 100 && cur > g_motor.prot_thresholds.stall_current && dur_s > g_motor.prot_thresholds.stall_time);
}
uint8_t protection_check_encoder_fault(void)      { return encoder_check_fault(); }
uint8_t protection_check_comm_timeout(void) {
    return (HAL_GetTick() - g_motor.last_comm_tick) > g_motor.prot_thresholds.comm_timeout_ms;
}

void protection_handle_fault(error_code_t err, alarm_level_t level)
{
    /* 记录历史 */
    alarm_record_t *rec = &g_alarm_history[g_alarm_history_idx];
    rec->timestamp_ms    = HAL_GetTick();
    rec->error_code      = (uint8_t)err;
    rec->alarm_level     = (uint8_t)level;
    rec->speed_snapshot  = (float)g_motor.feedback.speed_rpm_x100 / 100.0f;
    rec->current_snapshot = (float)g_motor.feedback.current_ma / 1000.0f;
    rec->temp_snapshot   = (float)g_motor.feedback.temp_driver_c - 40.0f;
    rec->voltage_snapshot = g_motor.feedback.bus_voltage_mv;
    g_alarm_history_idx = (g_alarm_history_idx + 1) % ALARM_HISTORY_MAX;
    if (g_alarm_history_count < ALARM_HISTORY_MAX) g_alarm_history_count++;

    if (g_motor.error_count < 8) g_motor.active_errors[g_motor.error_count++] = err;
    g_motor.feedback.error_code = (uint8_t)err;

    if (level >= ALARM_LVL_CRITICAL) {
        state_machine_transition(SYS_STATE_FAULT);
        pwm_emergency_stop(STOP_MODE_COAST);
        LOG_ERROR("PROT", "CRITICAL err=%d", err);
    } else if (level == ALARM_LVL_RECOVERABLE) {
        state_machine_transition(SYS_STATE_QUICK_STOP);
        pwm_set_duty(0);
        LOG_WARN("PROT", "Recoverable err=%d", err);
    } else {
        int16_t d = pwm_get_duty();
        pwm_set_duty(d / 2);
        LOG_WARN("PROT", "WARN err=%d", err);
    }

    /* EMCY */
    if (level >= ALARM_LVL_RECOVERABLE) {
        uint8_t mfg[5] = {0};
        uint16_t co_err = protection_err_to_co_emcy(err);
        co_emcy_send_alarm(co_err, mfg, 5);
        g_last_emcy_time_ms = HAL_GetTick();
    }
}

uint8_t protection_fault_reset(void)
{
    protection_thresholds_t *th = &g_motor.prot_thresholds;
    if (adc_get_current_a()      > th->overcurrent_threshold)  return 0;
    if (adc_get_voltage_v()      > th->overvoltage_threshold)  return 0;
    if (adc_get_temp_driver_c()  > th->overtemp_driver)       return 0;
    if (adc_get_temp_motor_c()   > th->overtemp_motor)        return 0;

    g_motor.error_count = 0;
    memset(g_motor.active_errors, 0, sizeof(g_motor.active_errors));
    g_motor.feedback.error_code = 0;
    g_motor.feedback.status_byte &= ~0x04;
    g_motor.need_fault_reset = 0;

    co_emcy_send_reset();

    if (g_motor.state == SYS_STATE_FAULT || g_motor.state == SYS_STATE_ESTOP)
        state_machine_transition(SYS_STATE_READY);

    LOG_INFO("PROT", "Fault reset OK");
    return 1;
}

void protection_estop_trigger(void)
{
    g_motor.is_estopped = 1;
    pwm_emergency_stop(STOP_MODE_BRAKE);
    state_machine_transition(SYS_STATE_ESTOP);
    LOG_ERROR("PROT", "ESTOP!");
    protection_update_led(ALARM_LVL_CRITICAL);
}

uint8_t protection_is_estopped(void) { return g_motor.is_estopped; }

uint8_t protection_get_alarm_history(alarm_record_t *records, uint8_t max_count)
{
    uint8_t cnt = MIN(g_alarm_history_count, max_count);
    memcpy(records, g_alarm_history, cnt * sizeof(alarm_record_t));
    return cnt;
}

void protection_reset_comm_timeout(void) { g_motor.last_comm_tick = HAL_GetTick(); }

void protection_update_led(alarm_level_t level)
{
    static uint32_t toggle_tick; uint32_t now = HAL_GetTick();
    switch (level) {
    case ALARM_LVL_NORMAL:
        HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT,  LED_GREEN_GPIO_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_GPIO_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_RED_GPIO_PORT,    LED_RED_GPIO_PIN,    GPIO_PIN_RESET);
        break;
    case ALARM_LVL_WARNING:
        HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT,  LED_GREEN_GPIO_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_RED_GPIO_PORT,    LED_RED_GPIO_PIN,    GPIO_PIN_RESET);
        if (now - toggle_tick > 250) { HAL_GPIO_TogglePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_GPIO_PIN); toggle_tick = now; }
        break;
    case ALARM_LVL_RECOVERABLE:
        HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT,  LED_GREEN_GPIO_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_GPIO_PIN, GPIO_PIN_RESET);
        if (now - toggle_tick > 500) { HAL_GPIO_TogglePin(LED_RED_GPIO_PORT, LED_RED_GPIO_PIN); toggle_tick = now; }
        break;
    case ALARM_LVL_CRITICAL:
        HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT,  LED_GREEN_GPIO_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_PORT, LED_YELLOW_GPIO_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_RED_GPIO_PORT,    LED_RED_GPIO_PIN,    GPIO_PIN_SET);
        break;
    }
}

uint16_t protection_err_to_co_emcy(error_code_t err)
{
    switch (err) {
    case ERR_OVERCURRENT:       return CO_EMCY_OVERCURRENT;
    case ERR_OVERVOLTAGE:       return CO_EMCY_OVERVOLTAGE;
    case ERR_UNDERVOLTAGE:      return CO_EMCY_UNDERVOLTAGE;
    case ERR_OVERTEMP_DRIVER:
    case ERR_OVERTEMP_MOTOR:    return CO_EMCY_DRIVER_OVERTEMP;
    case ERR_OVERLOAD:          return CO_EMCY_OVERLOAD;
    case ERR_STALL:             return CO_EMCY_STALL;
    case ERR_ENCODER_FAULT:     return CO_EMCY_ENCODER_FAULT;
    case ERR_COMM_TIMEOUT:      return CO_EMCY_COMM_TIMEOUT;
    default:                    return 0xFF00;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ESTOP_GPIO_PIN) protection_estop_trigger();
}
