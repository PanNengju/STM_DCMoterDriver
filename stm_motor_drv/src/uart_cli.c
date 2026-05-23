/**
 * @file    uart_cli.c
 * @brief   UART 调试 CLI — USART2 中断驱动命令行
 *          F1/F4 兼容
 */

#include "uart_cli.h"
#include "motor_drv.h"
#include "encoder.h"
#include "adc.h"
#include "pwm.h"
#include "protection.h"
#include "flash.h"
#include "canopen/canopen.h"
#include "canopen/co_objdict.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

static UART_HandleTypeDef huart_debug;
static log_level_t g_log_level = LOG_LEVEL_INFO;
static uint8_t g_trace_enabled = 0;
static uint32_t g_runtime_start_ms;

#define CLI_BUF_SIZE  128
#define CLI_LINE_BUF  256

static uint8_t g_cli_buf[CLI_BUF_SIZE];
static volatile uint8_t g_cli_idx;
static volatile uint8_t g_cli_line_ready;

static void cli_print(const char *str)
{
    HAL_UART_Transmit(&huart_debug, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

static void cli_printf(const char *fmt, ...)
{
    char buf[CLI_LINE_BUF];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, CLI_LINE_BUF, fmt, args);
    va_end(args);
    cli_print(buf);
}

void uart_cli_init(void)
{
    UART_DEBUG_CLK_ENABLE();
    UART_DEBUG_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin       = UART_DEBUG_GPIO_PIN_TX;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UART_DEBUG_GPIO_PORT, &gpio_init);

    gpio_init.Pin       = UART_DEBUG_GPIO_PIN_RX;
    gpio_init.Mode      = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(UART_DEBUG_GPIO_PORT, &gpio_init);

#if STM32_PLATFORM == STM32_PLATFORM_F1
    /* F1: USART2 不需要显式 AF 编号 */
#else
    /* F4: 需要 AF 编号 */
    gpio_init.Alternate = GPIO_AF7_USART2;
#endif

    huart_debug.Instance          = UART_DEBUG_INSTANCE;
    huart_debug.Init.BaudRate     = UART_DEBUG_BAUDRATE;
    huart_debug.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_debug.Init.StopBits     = UART_STOPBITS_1;
    huart_debug.Init.Parity       = UART_PARITY_NONE;
    huart_debug.Init.Mode         = UART_MODE_TX_RX;
    huart_debug.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart_debug);

    g_cli_idx        = 0;
    g_cli_line_ready  = 0;
    g_trace_enabled   = 0;
    g_runtime_start_ms = HAL_GetTick();

    /* 使能 UART RX 中断 */
    __HAL_UART_ENABLE_IT(&huart_debug, UART_IT_RXNE);

    cli_print("\r\n============================================\r\n");
    cli_print("  DC Motor Driver CLI v1.1 (STM32F1/F4)\r\n");
    cli_print("  Type 'help' for command list\r\n");
    cli_print("============================================\r\n\r\n> ");
}

void log_print(log_level_t level, const char *module, const char *fmt, ...)
{
    if (level > g_log_level || level == LOG_LEVEL_OFF) return;

    static const char *lvl_str[] = {"", "ERR", "WRN", "INF", "DBG"};
    uint32_t ts = HAL_GetTick() - g_runtime_start_ms;

    char buf[CLI_LINE_BUF];
    va_list args;
    va_start(args, fmt);
    int off = snprintf(buf, CLI_LINE_BUF, "[%06lu] [%s] %s: ", ts, lvl_str[level], module);
    vsnprintf(buf + off, CLI_LINE_BUF - off, fmt, args);
    va_end(args);
    strcat(buf, "\r\n");
    cli_print(buf);
}

static void cli_execute(const char *line)
{
    char cmd[32] = {0}, args[4][32] = {{0}};
    int argc = 0;
    const char *p = line;
    while (*p == ' ') p++;
    int ci = 0;
    while (*p && *p != ' ' && ci < 31) cmd[ci++] = *p++;
    cmd[ci] = 0;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p || argc >= 4) break;
        int ai = 0;
        while (*p && *p != ' ' && ai < 31) args[argc][ai++] = *p++;
        args[argc++][ai] = 0;
    }

    /* ===== 命令分发 ===== */
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cli_print(
            "Commands:\r\n"
            "  help                     Show this help\r\n"
            "  status                   Print system status\r\n"
            "  info                     Print firmware info\r\n"
            "  speed <rpm>              Set speed target (RPM)\r\n"
            "  pos <pulses> [abs|rel]   Set position target\r\n"
            "  torque <ma>              Set torque target (mA)\r\n"
            "  stop [coast|brake]       Normal stop\r\n"
            "  estop                    Emergency stop\r\n"
            "  reset                    Fault reset\r\n"
            "  zero [manual|auto]       Sensor zeroing\r\n"
            "  canopen                  CANopen status\r\n"
            "  param get <id>           Read parameter\r\n"
            "  param set <id> <val>     Write parameter\r\n"
            "  param save               Save to Flash\r\n"
            "  param list               List params\r\n"
            "  log <0-4>                Set log level\r\n"
            "  trace on|off             Real-time data stream\r\n"
            "  reboot                   Software reset\r\n"
        );
    } else if (strcmp(cmd, "status") == 0) {
        cli_printf("State:%d Mode:%d Speed:%ld Pos:%ld Cur:%.3fA Tor:%.2fNm "
                   "V:%.1fV Tdrv:%.1fC Tmot:%.1fC Alm:%d Err:%d\r\n",
                   g_motor.state, g_motor.ctrl_mode,
                   (long)g_motor.feedback.speed_rpm_x100,
                   (long)g_motor.feedback.position_pulses,
                   g_motor.feedback.current_ma / 1000.0f,
                   g_motor.feedback.torque_nm_x100 / 100.0f,
                   g_motor.feedback.bus_voltage_mv / 1000.0f,
                   g_motor.feedback.temp_driver_c - 40.0f,
                   g_motor.feedback.temp_motor_c - 40.0f,
                   g_motor.alarm_level, g_motor.error_count);
    } else if (strcmp(cmd, "info") == 0) {
        cli_printf("FW: v1.1  MCU: STM32F%d  RT:%lums  CAN:Node%d NMT:%d\r\n",
#if STM32_PLATFORM == STM32_PLATFORM_F1
                   1,
#else
                   4,
#endif
                   HAL_GetTick() - g_runtime_start_ms,
                   canopen_get_comm_params()->node_id,
                   canopen_get_nmt_state());
    } else if (strcmp(cmd, "speed") == 0 && argc >= 1) {
        int32_t rpm = atoi(args[0]);
        motor_set_mode(CTRL_MODE_SPEED);
        motor_set_speed(rpm * 100);
        cli_printf("Speed: %d RPM\r\n", (int)rpm);
    } else if (strcmp(cmd, "pos") == 0 && argc >= 1) {
        int32_t pls = atoi(args[0]);
        uint8_t rel = (argc >= 2 && strcmp(args[1], "rel") == 0) ? 1 : 0;
        motor_set_mode(CTRL_MODE_POSITION);
        motor_set_position(pls, rel);
        cli_printf("Pos: %ld %s\r\n", (long)pls, rel ? "rel" : "abs");
    } else if (strcmp(cmd, "torque") == 0 && argc >= 1) {
        int32_t ma = atoi(args[0]);
        motor_set_mode(CTRL_MODE_TORQUE);
        motor_set_torque(ma);
        cli_printf("Torque: %ld mA\r\n", (long)ma);
    } else if (strcmp(cmd, "stop") == 0) {
        stop_mode_t m = STOP_MODE_COAST;
        if (argc >= 1 && strcmp(args[0], "brake") == 0) m = STOP_MODE_BRAKE;
        motor_stop(m);
        cli_print("Stopped\r\n");
    } else if (strcmp(cmd, "estop") == 0) {
        motor_emergency_stop();
        cli_print("ESTOP!\r\n");
    } else if (strcmp(cmd, "reset") == 0) {
        cli_print(motor_fault_reset() ? "Reset OK\r\n" : "Reset FAIL\r\n");
    } else if (strcmp(cmd, "zero") == 0) {
        motor_zero_sensor((argc >= 1 && strcmp(args[0], "auto") == 0) ? 0 : 1);
        cli_print("Zero done\r\n");
    } else if (strcmp(cmd, "canopen") == 0) {
        co_comm_params_t *cp = canopen_get_comm_params();
        cli_printf("NMT:%d Node:%d Baud:%lu HB:%d SYNC:%d PDO:%d\r\n",
                   canopen_get_nmt_state(), cp->node_id, cp->baudrate,
                   cp->heartbeat_period_ms, cp->sync_period_ms,
                   cp->pdo_event_timer_ms);
    } else if (strcmp(cmd, "param") == 0 && argc >= 2) {
        if (strcmp(args[0], "get") == 0) {
            uint16_t idx = (uint16_t)strtoul(args[1], NULL, 0);
            uint8_t data[4] = {0}; uint32_t sz = 4;
            if (co_objdict_read(idx, 0, data, &sz)) {
                uint32_t v = 0; memcpy(&v, data, sz > 4 ? 4 : sz);
                cli_printf("0x%04X = %lu\r\n", idx, v);
            } else cli_printf("0x%04X not found\r\n", idx);
        } else if (strcmp(args[0], "set") == 0 && argc >= 3) {
            uint16_t idx = (uint16_t)strtoul(args[1], NULL, 0);
            uint32_t v = (uint32_t)strtoul(args[2], NULL, 0);
            cli_printf(co_objdict_write(idx, 0, (uint8_t *)&v, 4) ? "OK\r\n" : "FAIL\r\n");
        } else if (strcmp(args[0], "save") == 0) {
            cli_print(flash_params_save() ? "Saved\r\n" : "Save FAIL\r\n");
        } else if (strcmp(args[0], "list") == 0) {
            cli_printf("Total %d params. Use 'param get <id>'\r\n", co_objdict_get_count());
        }
    } else if (strcmp(cmd, "log") == 0 && argc >= 1) {
        int lv = atoi(args[0]);
        if (lv >= 0 && lv <= 4) { g_log_level = (log_level_t)lv; cli_printf("Log=%d\r\n", lv); }
    } else if (strcmp(cmd, "trace") == 0 && argc >= 1) {
        g_trace_enabled = (strcmp(args[0], "on") == 0);
        cli_print(g_trace_enabled ? "Trace ON\r\n" : "Trace OFF\r\n");
    } else if (strcmp(cmd, "reboot") == 0) {
        cli_print("Reboot...\r\n"); HAL_Delay(100); NVIC_SystemReset();
    } else if (strlen(cmd) > 0) {
        cli_printf("Unknown: '%s'. Type 'help'\r\n", cmd);
    }
}

void uart_cli_task(void *pvParameters)
{
    for (;;) {
        if (g_cli_line_ready) {
            g_cli_buf[g_cli_idx] = 0;
            g_cli_line_ready = 0;
            g_cli_idx = 0;
            cli_execute((const char *)g_cli_buf);
            cli_print("> ");
        }
        if (g_trace_enabled) uart_cli_trace_output();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* UART 中断处理: 逐字节接收，遇到 \r 置行完成标志 */
void UART_DEBUG_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart_debug, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart_debug.Instance->DR & 0xFF);
        __HAL_UART_CLEAR_FLAG(&huart_debug, UART_FLAG_RXNE);

        if (ch == '\r' || ch == '\n') {
            if (g_cli_idx > 0) {
                g_cli_line_ready = 1;
                /* 回显换行 */
                uint8_t crlf[] = "\r\n";
                HAL_UART_Transmit(&huart_debug, crlf, 2, 10);
            }
        } else if (ch == 0x08 || ch == 0x7F) {
            if (g_cli_idx > 0) g_cli_idx--;
            /* 回显退格 */
            uint8_t bs[] = "\b \b";
            HAL_UART_Transmit(&huart_debug, bs, 3, 10);
        } else if (g_cli_idx < CLI_BUF_SIZE - 1) {
            g_cli_buf[g_cli_idx++] = ch;
            /* 回显 */
            HAL_UART_Transmit(&huart_debug, &ch, 1, 10);
        }
    }
}

void uart_cli_trace_output(void)
{
    static uint32_t last;
    uint32_t now = HAL_GetTick();
    if (now - last < 10) return;
    last = now;
    cli_printf("t,%ld,%ld,%ld,%d,%d\r\n",
               (long)(now - g_runtime_start_ms),
               (long)g_motor.feedback.speed_rpm_x100 / 100,
               (long)g_motor.feedback.position_pulses,
               (int)(g_motor.feedback.current_ma / 100),
               (int)g_motor.feedback.torque_nm_x100);
}

log_level_t uart_cli_get_log_level(void) { return g_log_level; }
