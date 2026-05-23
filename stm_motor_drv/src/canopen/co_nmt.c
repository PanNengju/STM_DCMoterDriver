/**
 * @file    co_nmt.c
 * @brief   CANopen NMT 实现
 *          状态机管理 / Heartbeat 发送 / 节点守护
 */

#include "co_nmt.h"
#include "flash.h"
#include "uart_cli.h"

/*===========================================================================
 * 全局数据
 *===========================================================================*/
static co_nmt_state_t g_nmt_state = CO_NMT_INITIALIZING;

/*===========================================================================
 * NMT 初始化
 *===========================================================================*/
void co_nmt_init(void)
{
    g_nmt_state = CO_NMT_INITIALIZING;
}

/*===========================================================================
 * NMT 命令处理
 *
 * 格式: COB-ID 0x000, Data[2] = {命令码, 节点ID}
 * 节点ID=0 表示广播（所有节点）
 *===========================================================================*/
void co_nmt_process(uint8_t cmd, uint8_t node_id)
{
    co_comm_params_t *params = canopen_get_comm_params();

    /* 仅响应对本节点或广播 */
    if (node_id != 0 && node_id != params->node_id) {
        return;
    }

    switch (cmd) {
    case NMT_CMD_START_REMOTE_NODE:
        LOG_INFO("CANOPEN", "NMT: Start Remote Node");
        co_nmt_set_state(CO_NMT_OPERATIONAL);
        break;

    case NMT_CMD_STOP_REMOTE_NODE:
        LOG_INFO("CANOPEN", "NMT: Stop Remote Node");
        co_nmt_set_state(CO_NMT_STOPPED);
        break;

    case NMT_CMD_ENTER_PRE_OP:
        LOG_INFO("CANOPEN", "NMT: Enter Pre-Operational");
        co_nmt_set_state(CO_NMT_PRE_OPERATIONAL);
        break;

    case NMT_CMD_RESET_NODE:
        LOG_INFO("CANOPEN", "NMT: Reset Node");
        co_nmt_set_state(CO_NMT_INITIALIZING);
        /* 重新加载参数 */
        flash_params_load();
        co_nmt_set_state(CO_NMT_PRE_OPERATIONAL);
        break;

    case NMT_CMD_RESET_COMM:
        LOG_INFO("CANOPEN", "NMT: Reset Communication");
        co_nmt_set_state(CO_NMT_INITIALIZING);
        co_nmt_set_state(CO_NMT_PRE_OPERATIONAL);
        break;

    default:
        LOG_DEBUG("CANOPEN", "NMT: Unknown cmd=0x%02X", cmd);
        break;
    }
}

/*===========================================================================
 * 状态管理
 *===========================================================================*/
co_nmt_state_t co_nmt_get_state(void)
{
    return g_nmt_state;
}

void co_nmt_set_state(co_nmt_state_t state)
{
    g_nmt_state = state;

    /* 状态切换时同步 PDO 使能 */
    switch (state) {
    case CO_NMT_OPERATIONAL:
        /* 使能 PDO 通信 */
        break;
    case CO_NMT_PRE_OPERATIONAL:
        /* 仅 SDO 可通 */
        break;
    case CO_NMT_STOPPED:
        /* 仅 NMT 可通 */
        break;
    default:
        break;
    }
}

/*===========================================================================
 * Heartbeat 发送
 *
 * COB-ID: 0x700 + NodeID, Data[1] = NMT 状态
 *===========================================================================*/
void co_nmt_heartbeat_send(void)
{
    co_comm_params_t *params = canopen_get_comm_params();

    uint8_t data[1];
    data[0] = (uint8_t)g_nmt_state;

    uint32_t cob_id = 0x700 + params->node_id;
    canopen_tx_frame(cob_id, data, 1);
}

/*===========================================================================
 * Heartbeat Consumer 检查（作为消费者监测主机心跳）
 *===========================================================================*/
void co_nmt_heartbeat_consumer_check(void)
{
    /* 可扩展: 监控主机 Heartbeat (0x700+HostID) */
    /* 如果连续 3× 周期未收到 → 通信丢失 → 触发保护 */
}
