/**
 * @file    state_machine.c
 * @brief   系统状态机 + CiA 402 PDS 状态机实现
 */

#include "state_machine.h"
#include "pwm.h"
#include "encoder.h"
#include "protection.h"
#include "uart_cli.h"

/*===========================================================================
 * PDS 状态机实例
 *===========================================================================*/
static pds_state_t g_pds_state = PDS_STATE_NOT_READY;

/*===========================================================================
 * 系统状态机初始化
 *===========================================================================*/
void state_machine_init(void)
{
    g_motor.state = SYS_STATE_INIT;
    g_pds_state   = PDS_STATE_NOT_READY;
}

/*===========================================================================
 * 系统状态机主处理
 *
 * 状态流转:
 *   INIT → DISABLE → READY → RUNNING
 *                     ↑        ↓
 *                   FAULT ← QUICK_STOP
 *===========================================================================*/
system_state_t state_machine_process(void)
{
    switch (g_motor.state) {
    case SYS_STATE_INIT:
        /* 参数加载完成后自动转入 DISABLE */
        if (g_motor.is_enabled == 0) {
            state_machine_transition(SYS_STATE_DISABLE);
        }
        break;

    case SYS_STATE_DISABLE:
        /* 等待使能指令 */
        break;

    case SYS_STATE_READY:
        /* 等待运动指令 */
        break;

    case SYS_STATE_RUNNING:
        /* 运行中 — 控制循环在 FreeRTOS 任务中执行 */
        break;

    case SYS_STATE_QUICK_STOP:
        /* 快速停止中 */
        if (pwm_get_duty() == 0) {
            state_machine_transition(SYS_STATE_READY);
        }
        break;

    case SYS_STATE_FAULT:
        /* 锁定，等待手动复位 */
        break;

    case SYS_STATE_ESTOP:
        /* 急停锁定，需手动复位 */
        break;
    }
    return g_motor.state;
}

/*===========================================================================
 * 状态转换
 *===========================================================================*/
void state_machine_transition(system_state_t new_state)
{
    system_state_t old = g_motor.state;

    if (old == new_state) return;

    switch (new_state) {
    case SYS_STATE_DISABLE:
        pwm_enable(0);
        g_motor.is_running = 0;
        g_motor.is_enabled = 0;
        break;
    case SYS_STATE_READY:
        pwm_enable(1);
        g_motor.is_enabled = 1;
        g_motor.is_running = 0;
        break;
    case SYS_STATE_RUNNING:
        g_motor.is_running = 1;
        break;
    case SYS_STATE_FAULT:
    case SYS_STATE_ESTOP:
        pwm_emergency_stop(STOP_MODE_COAST);
        g_motor.is_running = 0;
        break;
    default:
        break;
    }

    g_motor.state = new_state;
    LOG_DEBUG("SM", "State: %d → %d", old, new_state);
}

system_state_t state_machine_get_state(void)
{
    return g_motor.state;
}

/*===========================================================================
 * CiA 402 PDS 状态机处理
 *
 * Controlword 命令:
 *   0x06 — Shutdown
 *   0x07 — Switch On
 *   0x0F — Enable Operation
 *   0x02 — Quick Stop
 *   0x00 — Disable Voltage
 *   0x80 — Fault Reset
 *===========================================================================*/
void pds_state_machine_process(void)
{
    /* PDS 状态与系统状态联动 */
    switch (g_motor.state) {
    case SYS_STATE_INIT:
        g_pds_state = PDS_STATE_NOT_READY;
        break;
    case SYS_STATE_DISABLE:
        g_pds_state = PDS_STATE_SWITCH_ON_DISABLED;
        break;
    case SYS_STATE_READY:
        if (g_pds_state == PDS_STATE_FAULT) {
            /* 保持故障直到复位 */
        } else {
            g_pds_state = PDS_STATE_SWITCHED_ON;
        }
        break;
    case SYS_STATE_RUNNING:
        g_pds_state = PDS_STATE_OPERATION_ENABLED;
        break;
    case SYS_STATE_QUICK_STOP:
        g_pds_state = PDS_STATE_QUICK_STOP_ACTIVE;
        break;
    case SYS_STATE_FAULT:
    case SYS_STATE_ESTOP:
        g_pds_state = PDS_STATE_FAULT;
        break;
    }
}

void pds_state_machine_handle_controlword(uint16_t cw)
{
    switch (cw) {
    case CO_CONTROLWORD_SHUTDOWN:
        if (g_pds_state == PDS_STATE_SWITCHED_ON
            || g_pds_state == PDS_STATE_OPERATION_ENABLED) {
            state_machine_transition(SYS_STATE_READY);
            pds_state_machine_process();
        }
        break;

    case CO_CONTROLWORD_SWITCH_ON:
        if (g_pds_state == PDS_STATE_READY_TO_SWITCH_ON
            || g_pds_state == PDS_STATE_SWITCHED_ON
            || g_pds_state == PDS_STATE_OPERATION_ENABLED
            || g_pds_state == PDS_STATE_QUICK_STOP_ACTIVE) {
            state_machine_transition(SYS_STATE_READY);
            pds_state_machine_process();
        }
        break;

    case CO_CONTROLWORD_ENABLE_OPERATION:
        if (g_pds_state == PDS_STATE_SWITCHED_ON
            || g_pds_state == PDS_STATE_OPERATION_ENABLED
            || g_pds_state == PDS_STATE_QUICK_STOP_ACTIVE) {
            state_machine_transition(SYS_STATE_RUNNING);
            pds_state_machine_process();
        }
        break;

    case CO_CONTROLWORD_QUICK_STOP:
        state_machine_transition(SYS_STATE_QUICK_STOP);
        pds_state_machine_process();
        break;

    case CO_CONTROLWORD_DISABLE_VOLTAGE:
        state_machine_transition(SYS_STATE_DISABLE);
        pds_state_machine_process();
        break;

    case CO_CONTROLWORD_FAULT_RESET:
        if (g_pds_state == PDS_STATE_FAULT) {
            if (protection_fault_reset()) {
                state_machine_transition(SYS_STATE_READY);
                pds_state_machine_process();
            }
        }
        break;

    default:
        break;
    }
}

pds_state_t pds_get_state(void)
{
    return g_pds_state;
}

/*===========================================================================
 * 状态字构造
 *
 * CiA 402 Statusword (Index 0x6041):
 *   Bit0: Ready to switch on
 *   Bit1: Switched on
 *   Bit2: Operation enabled
 *   Bit3: Fault
 *   Bit4: Voltage enabled
 *   Bit5: Quick stop
 *   Bit6: Switch on disabled
 *   Bit7: Warning
 *   Bit8: Manufacturer specific
 *   Bit9: Remote
 *   Bit10: Target reached
 *   Bit11: Internal limit active
 *   Bit12-13: Operation mode specific
 *   Bit14-15: Manufacturer specific
 *===========================================================================*/
uint16_t pds_build_statusword(void)
{
    uint16_t sw = 0;

    switch (g_pds_state) {
    case PDS_STATE_NOT_READY:
        sw = 0x0000;
        break;
    case PDS_STATE_SWITCH_ON_DISABLED:
        sw = 0x0040; /* Bit6: Switch on disabled */
        break;
    case PDS_STATE_READY_TO_SWITCH_ON:
        sw = 0x0021; /* Bit0 + Bit5 (Quick stop) */
        break;
    case PDS_STATE_SWITCHED_ON:
        sw = 0x0023; /* Bit0 + Bit1 + Bit5 */
        break;
    case PDS_STATE_OPERATION_ENABLED:
        sw = 0x0027; /* Bit0 + Bit1 + Bit2 + Bit5 */
        break;
    case PDS_STATE_QUICK_STOP_ACTIVE:
        sw = 0x0007; /* Bit0 + Bit1 + Bit2 (Quick stop bit5=0) */
        break;
    case PDS_STATE_FAULT_REACTION:
        sw = 0x000F; /* Bit0~3 */
        break;
    case PDS_STATE_FAULT:
        sw = 0x0008; /* Bit3: Fault */
        break;
    }

    /* Bit4: Voltage enabled */
    if (g_pds_state >= PDS_STATE_READY_TO_SWITCH_ON
        && g_pds_state != PDS_STATE_FAULT) {
        sw |= 0x0010;
    }

    /* Bit7: Warning */
    if (g_motor.alarm_level == ALARM_LVL_WARNING) {
        sw |= 0x0080;
    }

    /* Bit9: Remote (CANopen控制) */
    sw |= 0x0200;

    /* Bit10: Target reached */
    if (g_motor.ctrl_mode == CTRL_MODE_SPEED
        && ABS(g_motor.feedback.speed_rpm_x100 - g_motor.target_speed_rpm_x100) < 100) {
        sw |= 0x0400;
    } else if (g_motor.ctrl_mode == CTRL_MODE_POSITION
        && ABS(g_motor.feedback.position_pulses - g_motor.target_position_pulses) < 2) {
        sw |= 0x0400;
    }

    /* Bit11: Internal limit active */
    if (g_motor.ctrl_mode == CTRL_MODE_POSITION) {
        int32_t pos = encoder_get_position();
        if (pos <= g_motor.sw_position_min || pos >= g_motor.sw_position_max) {
            sw |= 0x0800;
        }
    }

    return sw;
}

int8_t pds_get_mode_display(void)
{
    switch (g_motor.ctrl_mode) {
    case CTRL_MODE_SPEED:    return CO_MODE_PV;
    case CTRL_MODE_POSITION: return CO_MODE_PP;
    case CTRL_MODE_TORQUE:   return CO_MODE_TQ;
    case CTRL_MODE_HOMING:   return CO_MODE_HOMING;
    default:                 return 0;
    }
}
