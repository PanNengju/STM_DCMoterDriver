/**
 * @file    co_emcy.h
 * @brief   CANopen EMCY (紧急报文) 模块
 *          故障主动上报 / EMCY 复位 / 错误历史存储
 */

#ifndef __CO_EMCY_H
#define __CO_EMCY_H

#include "canopen.h"

/* EMCY 初始化 */
void co_emcy_init(void);

/* EMCY 告警发送 (事件驱动) */
void co_emcy_send_alarm(uint16_t error_code, const uint8_t *mfg_data, uint8_t mfg_len);

/* EMCY 复位发送 (故障消除) */
void co_emcy_send_reset(void);

/* EMCY 历史记录读取 (SDO Index 0x2050) */
uint8_t co_emcy_get_history(uint8_t idx, uint16_t *error_code,
                            uint8_t *mfg_data, uint8_t *mfg_len);

/* EMCY 抑制时间检查 */
uint8_t co_emcy_inhibit_check(void);

#endif /* __CO_EMCY_H */
