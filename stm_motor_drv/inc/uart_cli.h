/**
 * @file    uart_cli.h
 * @brief   直流电机驱动 — UART 调试 CLI
 */

#ifndef __UART_CLI_H
#define __UART_CLI_H

#include "motor_config.h"

/* 日志等级 */
typedef enum {
    LOG_LEVEL_OFF   = 0,
    LOG_LEVEL_ERROR = 1,
    LOG_LEVEL_WARN  = 2,
    LOG_LEVEL_INFO  = 3,
    LOG_LEVEL_DEBUG = 4,
} log_level_t;

/* CLI 初始化 */
void uart_cli_init(void);

/* CLI 任务 */
void uart_cli_task(void *pvParameters);

/* 调试日志输出 */
void log_print(log_level_t level, const char *module, const char *fmt, ...);

/* Trace 模式实时数据流输出 */
void uart_cli_trace_output(void);

/* 获取当前日志等级 */
log_level_t uart_cli_get_log_level(void);

#define LOG_ERROR(mod, fmt, ...)  log_print(LOG_LEVEL_ERROR, mod, fmt, ##__VA_ARGS__)
#define LOG_WARN(mod, fmt, ...)   log_print(LOG_LEVEL_WARN,  mod, fmt, ##__VA_ARGS__)
#define LOG_INFO(mod, fmt, ...)   log_print(LOG_LEVEL_INFO,  mod, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(mod, fmt, ...)  log_print(LOG_LEVEL_DEBUG, mod, fmt, ##__VA_ARGS__)

#endif /* __UART_CLI_H */
