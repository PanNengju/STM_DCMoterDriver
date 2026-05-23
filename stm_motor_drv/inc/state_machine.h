/**
 * @file    state_machine.h
 * @brief   直流电机驱动 — 系统状态机 + CiA 402 PDS 状态机
 */

#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "motor_config.h"

/* 系统状态机初始化 */
void state_machine_init(void);

/* 系统状态机主处理 */
system_state_t state_machine_process(void);

/* 状态转换 */
void state_machine_transition(system_state_t new_state);

/* 获取当前状态 */
system_state_t state_machine_get_state(void);

/* CiA 402 PDS 状态机 */
typedef enum {
    PDS_STATE_NOT_READY         = 0,
    PDS_STATE_SWITCH_ON_DISABLED= 1,
    PDS_STATE_READY_TO_SWITCH_ON= 2,
    PDS_STATE_SWITCHED_ON       = 3,
    PDS_STATE_OPERATION_ENABLED = 4,
    PDS_STATE_QUICK_STOP_ACTIVE = 5,
    PDS_STATE_FAULT_REACTION    = 6,
    PDS_STATE_FAULT             = 7,
} pds_state_t;

/* CiA 402 控制字指令 */
#define CO_CONTROLWORD_SHUTDOWN          0x0006U
#define CO_CONTROLWORD_SWITCH_ON         0x0007U
#define CO_CONTROLWORD_ENABLE_OPERATION  0x000FU
#define CO_CONTROLWORD_QUICK_STOP        0x0002U
#define CO_CONTROLWORD_DISABLE_VOLTAGE   0x0000U
#define CO_CONTROLWORD_FAULT_RESET       0x0080U

/* CiA 402 PDS 状态机处理 */
void pds_state_machine_process(void);
void pds_state_machine_handle_controlword(uint16_t cw);
pds_state_t pds_get_state(void);

/* 状态字构造 (CiA 402 Statusword) */
uint16_t pds_build_statusword(void);

/* 获取 CiA 402 Modes of Operation Display */
int8_t pds_get_mode_display(void);

#endif /* __STATE_MACHINE_H */
