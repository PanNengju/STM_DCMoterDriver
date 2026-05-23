/**
 * @file    co_cia402.c
 * @brief   CANopen CiA 402 驱动行规实现
 *          PDS 状态机联动 / PDO 数据打包与解析
 */

#include "co_cia402.h"
#include "state_machine.h"
#include "pwm.h"
#include "encoder.h"
#include "adc.h"
#include "protection.h"
#include <string.h>

/*===========================================================================
 * CiA 402 模块初始化
 *===========================================================================*/
void co_cia402_init(void)
{
    /* PDS 状态机由 state_machine 管理 */
}

/*===========================================================================
 * 控制字处理 — 驱动系统状态机
 *===========================================================================*/
void co_cia402_process_controlword(uint16_t cw)
{
    /* Bit7 = Fault Reset */
    if (cw & 0x0080) {
        pds_state_machine_handle_controlword(CO_CONTROLWORD_FAULT_RESET);
        return;
    }

    /* Bits[3:0] + Bit7 判断指令 */
    uint16_t cmd_bits = cw & 0x008F;

    switch (cmd_bits) {
    case 0x0006:
        pds_state_machine_handle_controlword(CO_CONTROLWORD_SHUTDOWN);
        break;
    case 0x0007:
        pds_state_machine_handle_controlword(CO_CONTROLWORD_SWITCH_ON);
        break;
    case 0x000F:
        pds_state_machine_handle_controlword(CO_CONTROLWORD_ENABLE_OPERATION);
        break;
    case 0x0002:
        pds_state_machine_handle_controlword(CO_CONTROLWORD_QUICK_STOP);
        break;
    case 0x0000:
        pds_state_machine_handle_controlword(CO_CONTROLWORD_DISABLE_VOLTAGE);
        break;
    default:
        break;
    }
}

/*===========================================================================
 * 模式切换
 *===========================================================================*/
void co_cia402_set_mode(int8_t mode)
{
    switch (mode) {
    case CO_MODE_PV:
        g_motor.ctrl_mode = CTRL_MODE_SPEED;
        break;
    case CO_MODE_PP:
        g_motor.ctrl_mode = CTRL_MODE_POSITION;
        break;
    case CO_MODE_TQ:
        g_motor.ctrl_mode = CTRL_MODE_TORQUE;
        break;
    case CO_MODE_HOMING:
        g_motor.ctrl_mode = CTRL_MODE_HOMING;
        break;
    default:
        break;
    }
}

int8_t co_cia402_get_mode(void)
{
    switch (g_motor.ctrl_mode) {
    case CTRL_MODE_SPEED:    return CO_MODE_PV;
    case CTRL_MODE_POSITION: return CO_MODE_PP;
    case CTRL_MODE_TORQUE:   return CO_MODE_TQ;
    case CTRL_MODE_HOMING:   return CO_MODE_HOMING;
    default:                 return 0;
    }
}

/*===========================================================================
 * 目标值写入
 *===========================================================================*/
void co_cia402_set_target_speed(int32_t rpm_x100)
{
    g_motor.target_speed_rpm_x100 = CLAMP(rpm_x100, -300000, 300000);
}

void co_cia402_set_target_position(int32_t pulses)
{
    g_motor.target_position_pulses = pulses;
}

void co_cia402_set_target_torque(int16_t nm_x100)
{
    g_motor.target_torque_nm_x100 = nm_x100;
}

/*===========================================================================
 * 状态字构造
 *===========================================================================*/
uint16_t co_cia402_build_statusword(void)
{
    return pds_build_statusword();
}

/*===========================================================================
 * TPDO1 打包: Statusword(16) + Actual Speed RPM×100(32) = 6 bytes
 *===========================================================================*/
void co_cia402_pack_tpdo1(uint8_t *data)
{
    uint16_t sw = pds_build_statusword();
    int32_t speed = encoder_get_speed_rpm_x100();

    data[0] = sw & 0xFF;
    data[1] = (sw >> 8) & 0xFF;
    memcpy(&data[2], &speed, 4);
}

/*===========================================================================
 * TPDO2 打包: Actual Position(32) + Actual Current mA(32) = 8 bytes
 *===========================================================================*/
void co_cia402_pack_tpdo2(uint8_t *data)
{
    int32_t pos = encoder_get_position();
    int32_t current_ma = (int32_t)(adc_get_current_a() * 1000.0f);

    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &current_ma, 4);
}

/*===========================================================================
 * TPDO3 打包: Actual Torque Nm×100(16) + DriverTemp offset40(8) + BusVoltage mV(16) = 5 bytes
 *===========================================================================*/
void co_cia402_pack_tpdo3(uint8_t *data)
{
    float current  = adc_get_current_a();
    float torque   = current * g_motor.motor_params.torque_constant;
    int16_t tor_x100 = (int16_t)(torque * 100.0f);
    uint8_t temp    = (uint8_t)(adc_get_temp_driver_c() + 40.0f);
    uint16_t voltage_mv = (uint16_t)(adc_get_voltage_v() * 1000.0f);

    memcpy(&data[0], &tor_x100, 2);
    data[2] = temp;
    memcpy(&data[3], &voltage_mv, 2);
}

/*===========================================================================
 * RPDO1 解析: Controlword(16) + TargetValue1(32)
 *===========================================================================*/
void co_cia402_parse_rpdo1(const uint8_t *data)
{
    uint16_t cw = data[0] | ((uint16_t)data[1] << 8);
    int32_t target_val;
    memcpy(&target_val, &data[2], 4);

    co_cia402_process_controlword(cw);

    /* 根据当前控制模式解释 target_val */
    switch (g_motor.ctrl_mode) {
    case CTRL_MODE_SPEED:
        co_cia402_set_target_speed(target_val);
        break;
    case CTRL_MODE_POSITION:
        co_cia402_set_target_position(target_val);
        break;
    default:
        break;
    }
}

/*===========================================================================
 * RPDO2 解析: Mode(8) + TargetValue2(32)
 *===========================================================================*/
void co_cia402_parse_rpdo2(const uint8_t *data)
{
    int8_t mode = (int8_t)data[0];
    int32_t target_val;
    memcpy(&target_val, &data[1], 4);

    co_cia402_set_mode(mode);

    switch (mode) {
    case CO_MODE_PV:
        co_cia402_set_target_speed(target_val);
        break;
    case CO_MODE_TQ:
        co_cia402_set_target_torque((int16_t)target_val);
        break;
    case CO_MODE_PP:
        co_cia402_set_target_position(target_val);
        break;
    default:
        break;
    }
}
