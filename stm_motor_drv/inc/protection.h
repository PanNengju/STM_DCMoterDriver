/**
 * @file    protection.h
 * @brief   直流电机驱动 — 多级保护与报警模块
 */

#ifndef __PROTECTION_H
#define __PROTECTION_H

#include "motor_config.h"

/* 报警记录（最多 10 条） */
#define ALARM_HISTORY_MAX   10U

typedef struct {
    uint32_t  timestamp_ms;
    uint8_t   error_code;
    uint8_t   alarm_level;
    float     speed_snapshot;
    float     current_snapshot;
    float     temp_snapshot;
    uint16_t  voltage_snapshot;
} alarm_record_t;

/* 保护模块初始化 */
void protection_init(void);

/**
 * @brief   执行所有保护检查（在监控任务中周期性调用）
 * @return  当前报警等级
 */
alarm_level_t protection_check_all(void);

/* 单项保护检查 */
uint8_t protection_check_overcurrent(float current_a);
uint8_t protection_check_overload(float torque_nm, float duration_s);
uint8_t protection_check_stall(int32_t speed_rpm_x100, float current_a, float duration_s);
uint8_t protection_check_overtemp_driver(float temp_c);
uint8_t protection_check_overtemp_motor(float temp_c);
uint8_t protection_check_overvoltage(float voltage_v);
uint8_t protection_check_undervoltage(float voltage_v);
uint8_t protection_check_encoder_fault(void);
uint8_t protection_check_comm_timeout(void);

/* 故障处理 */
void protection_handle_fault(error_code_t err, alarm_level_t level);

/* 故障复位 */
uint8_t protection_fault_reset(void);

/* 急停处理 */
void protection_estop_trigger(void);
uint8_t protection_is_estopped(void);

/* 获取报警历史 */
uint8_t protection_get_alarm_history(alarm_record_t *records, uint8_t max_count);

/* 重置通信超时计时器 */
void protection_reset_comm_timeout(void);

/* LED 指示更新 */
void protection_update_led(alarm_level_t level);

/* 错误码转 CANopen EMCY */
uint16_t protection_err_to_co_emcy(error_code_t err);

#endif /* __PROTECTION_H */
