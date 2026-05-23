/**
 * @file    co_emcy.c
 * @brief   CANopen EMCY (紧急报文) 实现
 *          故障主动上报 / 复位报文 / 历史记录
 */

#include "co_emcy.h"
#include "uart_cli.h"
#include <string.h>

/*===========================================================================
 * EMCY 历史记录条目
 *===========================================================================*/
typedef struct {
    uint32_t timestamp_ms;
    uint16_t error_code;
    uint8_t  error_register;
    uint8_t  mfg_data[5];
    uint8_t  is_active;
} emcy_record_t;

#define EMCY_HISTORY_MAX    10U

static emcy_record_t g_emcy_history[EMCY_HISTORY_MAX];
static uint8_t g_emcy_history_idx;
static uint8_t g_emcy_history_count;
static uint32_t g_last_emcy_time;

/*===========================================================================
 * EMCY 初始化
 *===========================================================================*/
void co_emcy_init(void)
{
    memset(g_emcy_history, 0, sizeof(g_emcy_history));
    g_emcy_history_idx   = 0;
    g_emcy_history_count = 0;
    g_last_emcy_time     = 0;
}

/*===========================================================================
 * EMCY 告警发送
 *
 * COB-ID: 0x080 + NodeID
 * Data[8]:
 *   [0-1]: 标准错误码 (u16 LE)
 *   [2]:   错误寄存器
 *   [3-7]: 制造商特定域 (速度/电流/温度快照等)
 *===========================================================================*/
void co_emcy_send_alarm(uint16_t error_code, const uint8_t *mfg_data, uint8_t mfg_len)
{
    /* EMCY 抑制时间检查 */
    if (!co_emcy_inhibit_check()) return;

    co_comm_params_t *params = canopen_get_comm_params();
    uint8_t data[8] = {0};

    data[0] = error_code & 0xFF;
    data[1] = (error_code >> 8) & 0xFF;
    data[2] = 0x01; /* Generic Error */

    if (mfg_data && mfg_len > 0) {
        uint8_t copy_len = MIN(mfg_len, 5);
        memcpy(&data[3], mfg_data, copy_len);
    }

    uint32_t cob_id = 0x080 + params->node_id;
    canopen_tx_frame(cob_id, data, 8);

    /* 存储历史 */
    emcy_record_t *rec = &g_emcy_history[g_emcy_history_idx];
    rec->timestamp_ms = HAL_GetTick();
    rec->error_code   = error_code;
    rec->error_register = 0x01;
    memcpy(rec->mfg_data, &data[3], 5);
    rec->is_active    = 1;

    g_emcy_history_idx = (g_emcy_history_idx + 1) % EMCY_HISTORY_MAX;
    if (g_emcy_history_count < EMCY_HISTORY_MAX) g_emcy_history_count++;

    g_last_emcy_time = HAL_GetTick();

    LOG_ERROR("EMCY", "Alarm sent: code=0x%04X", error_code);
}

/*===========================================================================
 * EMCY 复位发送
 *===========================================================================*/
void co_emcy_send_reset(void)
{
    co_comm_params_t *params = canopen_get_comm_params();
    uint8_t data[8] = {0};

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;

    uint32_t cob_id = 0x080 + params->node_id;
    canopen_tx_frame(cob_id, data, 8);

    /* 标记所有活跃记录为非活跃 */
    for (uint8_t i = 0; i < EMCY_HISTORY_MAX; i++) {
        g_emcy_history[i].is_active = 0;
    }

    LOG_INFO("EMCY", "Reset sent");
}

/*===========================================================================
 * EMCY 抑制时间检查
 *===========================================================================*/
uint8_t co_emcy_inhibit_check(void)
{
    co_comm_params_t *params = canopen_get_comm_params();
    uint32_t now = HAL_GetTick();

    if (now - g_last_emcy_time < params->emcy_inhibit_time_ms) {
        return 0;
    }
    return 1;
}

/*===========================================================================
 * EMCY 历史记录读取 (通过 SDO Index 0x2050)
 *===========================================================================*/
uint8_t co_emcy_get_history(uint8_t idx, uint16_t *error_code,
                            uint8_t *mfg_data, uint8_t *mfg_len)
{
    if (idx >= g_emcy_history_count) return 0;

    uint8_t actual_idx = (g_emcy_history_idx - 1 - idx + EMCY_HISTORY_MAX)
                          % EMCY_HISTORY_MAX;

    *error_code = g_emcy_history[actual_idx].error_code;
    memcpy(mfg_data, g_emcy_history[actual_idx].mfg_data, 5);
    *mfg_len = 5;

    return 1;
}
