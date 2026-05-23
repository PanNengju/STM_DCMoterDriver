/**
 * @file    co_sync.h
 * @brief   CANopen SYNC (同步对象) 模块
 */

#ifndef __CO_SYNC_H
#define __CO_SYNC_H

#include "canopen.h"

/* SYNC 初始化 */
void co_sync_init(void);

/* SYNC 发送 (在 1ms 定时器中调用) */
void co_sync_process(void);

/* SYNC 接收处理 (作为消费者时由主机发送) */
void co_sync_rx_callback(void);

#endif /* __CO_SYNC_H */
