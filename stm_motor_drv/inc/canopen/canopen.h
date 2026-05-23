/**
 * @file    canopen.h
 * @brief   CANopen 协议栈主模块 (CiA 301 + CiA 402)
 *          NMT / SDO / PDO / EMCY / SYNC / Heartbeat / Object Dictionary
 */

#ifndef __CANOPEN_H
#define __CANOPEN_H

#include "motor_config.h"

/* CANopen NMT 状态 */
typedef enum {
    CO_NMT_INITIALIZING    = 0x00,
    CO_NMT_PRE_OPERATIONAL = 0x04,
    CO_NMT_OPERATIONAL     = 0x05,
    CO_NMT_STOPPED         = 0x7F,
} co_nmt_state_t;

/* CANopen 通信参数 */
typedef struct {
    uint8_t   node_id;
    uint32_t  baudrate;
    uint16_t  heartbeat_period_ms;      /* 心跳周期 */
    uint16_t  sync_period_ms;           /* SYNC 周期 */
    uint16_t  pdo_event_timer_ms;       /* PDO 事件定时周期 */
    uint16_t  emcy_inhibit_time_ms;     /* EMCY 抑制时间 */
    uint16_t  sdo_timeout_ms;           /* SDO 超时 */
} co_comm_params_t;

/* CANopen 模块初始化 */
void canopen_init(void);

/* CANopen 主任务 (FreeRTOS task) */
void canopen_task(void *pvParameters);

/* CAN 帧接收处理 */
void canopen_rx_callback(CAN_RxHeaderTypeDef *header, uint8_t *data, uint32_t len);

/* CAN 帧发送 */
uint8_t canopen_tx_frame(uint32_t cob_id, uint8_t *data, uint8_t len);

/* NMT 状态获取 */
co_nmt_state_t canopen_get_nmt_state(void);

/* Heartbeat 发送 */
void canopen_heartbeat_send(void);

/* 通信参数获取 */
co_comm_params_t *canopen_get_comm_params(void);

/* 更新通信超时计时 */
void canopen_update_comm_timestamp(void);

#endif /* __CANOPEN_H */
