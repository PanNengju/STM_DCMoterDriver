/**
 * @file    co_cia402.h
 * @brief   CANopen CiA 402 驱动行规映射
 *          PDS 状态机 / 控制模式管理 / 运动控制指令处理
 */

#ifndef __CO_CIA402_H
#define __CO_CIA402_H

#include "canopen.h"
#include "state_machine.h"

/* CiA 402 模块初始化 */
void co_cia402_init(void);

/* CiA 402 控制字处理 */
void co_cia402_process_controlword(uint16_t cw);

/* CiA 402 模式切换 */
void co_cia402_set_mode(int8_t mode);
int8_t co_cia402_get_mode(void);

/* 目标值写入 */
void co_cia402_set_target_speed(int32_t rpm_x100);
void co_cia402_set_target_position(int32_t pulses);
void co_cia402_set_target_torque(int16_t nm_x100);

/* 状态字构造 */
uint16_t co_cia402_build_statusword(void);

/* PDO 数据打包 */
void co_cia402_pack_tpdo1(uint8_t *data);
void co_cia402_pack_tpdo2(uint8_t *data);
void co_cia402_pack_tpdo3(uint8_t *data);

/* RPDO 数据解析 */
void co_cia402_parse_rpdo1(const uint8_t *data);
void co_cia402_parse_rpdo2(const uint8_t *data);

#endif /* __CO_CIA402_H */
