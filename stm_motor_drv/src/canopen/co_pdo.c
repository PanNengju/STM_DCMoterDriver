/**
 * @file    co_pdo.c
 * @brief   CANopen PDO 实现
 *          RPDO 接收 (控制) + TPDO 发送 (反馈)
 */

#include "co_pdo.h"
#include "co_cia402.h"
#include "state_machine.h"
#include "uart_cli.h"

/*===========================================================================
 * PDO 初始化
 *===========================================================================*/
void co_pdo_init(void)
{
    co_pdo_configure_mapping();
}

/*===========================================================================
 * PDO 映射配置
 *
 * RPDO1 (0x200+ID): Controlword(16) + TargetValue1(32) = 6 bytes
 * RPDO2 (0x300+ID): ModesOfOperation(8) + TargetValue2(32) = 5 bytes
 *
 * TPDO1 (0x180+ID): Statusword(16) + ActualSpeed(32) = 6 bytes
 * TPDO2 (0x280+ID): ActualPosition(32) + ActualCurrent(32) = 8 bytes
 * TPDO3 (0x380+ID): ActualTorque(16) + TempDriver(8) + BusVoltage(16) = 5 bytes
 *===========================================================================*/
void co_pdo_configure_mapping(void)
{
    /* 映射在 co_cia402 中实现 */
}

/*===========================================================================
 * RPDO 接收处理
 *===========================================================================*/
void co_pdo_process_rx(uint32_t cob_id, const uint8_t *data, uint8_t len)
{
    co_comm_params_t *params = canopen_get_comm_params();
    uint8_t node_id = params->node_id;

    if (cob_id == (uint32_t)(0x200 + node_id)) {
        /* RPDO1: Controlword + Target Value 1 */
        co_cia402_parse_rpdo1(data);
        LOG_DEBUG("PDO", "RPDO1 received");
    } else if (cob_id == (uint32_t)(0x300 + node_id)) {
        /* RPDO2: Modes of Operation + Target Value 2 */
        co_cia402_parse_rpdo2(data);
        LOG_DEBUG("PDO", "RPDO2 received");
    }
}

/*===========================================================================
 * TPDO 发送处理 (周期调用)
 *===========================================================================*/
void co_pdo_process_tx(void)
{
    co_comm_params_t *params = canopen_get_comm_params();
    uint8_t data[8];

    /* TPDO1: Statusword + Actual Speed */
    co_cia402_pack_tpdo1(data);
    canopen_tx_frame(0x180 + params->node_id, data, 6);

    /* TPDO2: Actual Position + Actual Current */
    co_cia402_pack_tpdo2(data);
    canopen_tx_frame(0x280 + params->node_id, data, 8);

    /* TPDO3: Actual Torque + Driver Temp + Bus Voltage */
    co_cia402_pack_tpdo3(data);
    canopen_tx_frame(0x380 + params->node_id, data, 5);
}

/*===========================================================================
 * SYNC 触发同步 PDO
 *===========================================================================*/
void co_pdo_process_sync(void)
{
    /* SYNC 触发后发送同步类 TPDO */
}

/*===========================================================================
 * 事件驱动 TPDO 立即发送
 *===========================================================================*/
void co_pdo_event_trigger(uint8_t tpdo_num)
{
    co_comm_params_t *params = canopen_get_comm_params();
    uint8_t data[8];
    uint32_t cob_id;

    switch (tpdo_num) {
    case PDO_NUM_TPDO1:
        co_cia402_pack_tpdo1(data);
        cob_id = 0x180 + params->node_id;
        canopen_tx_frame(cob_id, data, 6);
        break;
    case PDO_NUM_TPDO2:
        co_cia402_pack_tpdo2(data);
        cob_id = 0x280 + params->node_id;
        canopen_tx_frame(cob_id, data, 8);
        break;
    case PDO_NUM_TPDO3:
        co_cia402_pack_tpdo3(data);
        cob_id = 0x380 + params->node_id;
        canopen_tx_frame(cob_id, data, 5);
        break;
    }
}
