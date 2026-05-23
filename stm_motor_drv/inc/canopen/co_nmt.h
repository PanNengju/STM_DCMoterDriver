/**
 * @file    co_nmt.h
 * @brief   CANopen NMT (网络管理) 模块
 *          NMT 状态机 / Heartbeat Producer / Heartbeat Consumer
 */

#ifndef __CO_NMT_H
#define __CO_NMT_H

#include "canopen.h"

/* NMT 指令码 */
#define NMT_CMD_START_REMOTE_NODE       0x01
#define NMT_CMD_STOP_REMOTE_NODE        0x02
#define NMT_CMD_ENTER_PRE_OP            0x80
#define NMT_CMD_RESET_NODE              0x81
#define NMT_CMD_RESET_COMM              0x82

/* NMT 初始化 */
void co_nmt_init(void);

/* NMT 消息处理 */
void co_nmt_process(uint8_t cmd, uint8_t node_id);

/* 获取 NMT 状态 */
co_nmt_state_t co_nmt_get_state(void);

/* 设置 NMT 状态 */
void co_nmt_set_state(co_nmt_state_t state);

/* Heartbeat 发送 */
void co_nmt_heartbeat_send(void);

/* Heartbeat 消费者检查 */
void co_nmt_heartbeat_consumer_check(void);

#endif /* __CO_NMT_H */
