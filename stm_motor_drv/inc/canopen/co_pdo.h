/**
 * @file    co_pdo.h
 * @brief   CANopen PDO (过程数据对象) 模块
 *          RPDO (接收-控制指令) + TPDO (发送-状态反馈)
 *          支持: 同步(触发/非触发) / 异步事件驱动 / 异步定时
 */

#ifndef __CO_PDO_H
#define __CO_PDO_H

#include "canopen.h"

/* PDO 传输类型 */
#define PDO_TRANS_SYNC_ACYCLIC    0x00    /* 同步⋅非触发(收到SYNC后发送) */
#define PDO_TRANS_SYNC_CYCLIC(n)  (n)     /* 同步⋅触发(n=1~240, 每n个SYNC发送) */
#define PDO_TRANS_ASYNC_MFG       0xFE    /* 异步⋅制造商事件 */
#define PDO_TRANS_ASYNC_PROFILE   0xFF    /* 异步⋅设备行规事件 */

/* PDO 编号 (1-based) */
#define PDO_NUM_RPDO1   0
#define PDO_NUM_RPDO2   1
#define PDO_NUM_RPDO3   2
#define PDO_NUM_RPDO4   3
#define PDO_NUM_TPDO1   0
#define PDO_NUM_TPDO2   1
#define PDO_NUM_TPDO3   2
#define PDO_NUM_TPDO4   3

/* PDO 模块初始化 */
void co_pdo_init(void);

/* RPDO 接收处理 */
void co_pdo_process_rx(uint32_t cob_id, const uint8_t *data, uint8_t len);

/* TPDO 发送处理 (在定时循环中调用) */
void co_pdo_process_tx(void);

/* SYNC 触发同步 PDO */
void co_pdo_process_sync(void);

/* 事件驱动 TPDO 立即发送 */
void co_pdo_event_trigger(uint8_t tpdo_num);

/* PDO 映射配置 */
void co_pdo_configure_mapping(void);

#endif /* __CO_PDO_H */
