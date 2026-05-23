/**
 * @file    co_objdict.c
 * @brief   CANopen 对象字典实现
 *          CiA 301 标准区域 (0x1000~0x1FFF)
 *          CiA 402 驱动行规区域
 *          制造商自定义区域 (0x2000~0x5FFF)
 *          支持 SDO 读写访问
 */

#include "co_objdict.h"
#include "state_machine.h"
#include "canopen.h"
#include "co_cia402.h"
#include "flash.h"
#include "uart_cli.h"
#include <string.h>

/*===========================================================================
 * 全局对象字典表
 *
 * 使用静态数组存储所有可访问的对象字典条目。
 * 数据指针指向 g_motor 中的实际变量或静态存储。
 *===========================================================================*/

/* 辅助: 获取通信参数指针 */
static co_comm_params_t *od_comm_params;

/* 辅助静态变量 (不属于 g_motor 的变量) */
static int8_t  od_mode_of_operation;          /* 0x6060 */
static int8_t  od_mode_display;               /* 0x6061 */
static uint16_t od_controlword;               /* 0x6040 */
static uint16_t od_statusword;                /* 0x6041 */
static uint32_t od_firmware_version;          /* 0x100A */
static uint32_t od_serial_number;             /* 0x2100 */
static uint32_t od_runtime_hours;             /* 0x2101 */
static char     od_manufacturer[16];          /* 0x2108 */
static char     od_product_name[16];          /* 0x1008 */
static uint32_t od_sync_period;               /* 0x1006 */
static uint16_t od_heartbeat_period;          /* 0x1017 */

#define OD_ENTRY_COUNT  80

static co_objdict_entry_t g_objdict[OD_ENTRY_COUNT];
static uint16_t g_objdict_count = 0;

/*===========================================================================
 * 添加条目辅助
 *===========================================================================*/
static void od_add(uint16_t idx, uint8_t sub, uint8_t access,
                   uint8_t type, uint32_t size, void *ptr, uint32_t def,
                   const char *name)
{
    if (g_objdict_count >= OD_ENTRY_COUNT) return;
    g_objdict[g_objdict_count].index       = idx;
    g_objdict[g_objdict_count].sub_index   = sub;
    g_objdict[g_objdict_count].access      = access;
    g_objdict[g_objdict_count].data_type   = type;
    g_objdict[g_objdict_count].data_size   = size;
    g_objdict[g_objdict_count].data_ptr    = ptr;
    g_objdict[g_objdict_count].default_val = def;
    g_objdict[g_objdict_count].name        = name;
    g_objdict_count++;
}

/*===========================================================================
 * 对象字典初始化
 *===========================================================================*/
void co_objdict_init(void)
{
    g_objdict_count = 0;
    od_comm_params = canopen_get_comm_params();

    od_mode_of_operation  = 0;
    od_mode_display       = 0;
    od_controlword        = 0;
    od_statusword         = 0;
    od_firmware_version   = 0x00000101; /* V1.1 */
    od_serial_number      = 0x00000001;
    od_runtime_hours      = 0;
    strcpy(od_manufacturer, "AI_Learning");
    strcpy(od_product_name,  "DC_Motor_Driver");
    od_sync_period        = 1000;     /* us */
    od_heartbeat_period   = 200;      /* ms */

    /*=======================================================================
     * CiA 301 标准区域 (0x1000 ~ 0x1FFF)
     *=======================================================================*/

    /* --- 设备标识 (0x1000) --- */
    od_add(0x1000, 0, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_firmware_version, 0, "Device Type");
    od_add(0x1001, 0, CO_ACCESS_RO, CO_TYPE_U8,  1, NULL, 0, "Error Register");

    /* --- 通信参数 (0x1005 ~ 0x1017) --- */
    od_add(0x1005, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &od_comm_params->sync_period_ms, 1000, "COB-ID SYNC");
    od_add(0x1006, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &od_sync_period, 1000, "Communication Cycle Period");
    od_add(0x1008, 0, CO_ACCESS_CONST, CO_TYPE_STRING, 16, od_product_name, 0, "Manufacturer Device Name");
    od_add(0x100A, 0, CO_ACCESS_CONST, CO_TYPE_STRING, 16, od_manufacturer, 0, "Manufacturer Software Version");
    od_add(0x1017, 0, CO_ACCESS_RW, CO_TYPE_U16, 2, &od_heartbeat_period, 200, "Producer Heartbeat Time");

    /* --- 设备标识 (0x1018) --- */
    od_add(0x1018, 0, CO_ACCESS_RO, CO_TYPE_U8,  1, NULL, 4, "Identity Object SubCount");
    od_add(0x1018, 1, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_serial_number, 0, "Vendor ID");
    od_add(0x1018, 2, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_serial_number, 0, "Product Code");
    od_add(0x1018, 3, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_firmware_version, 0, "Revision Number");

    /*=======================================================================
     * CiA 402 区域 (0x6040 ~ 0x60FF)
     *=======================================================================*/

    /* Controlword / Statusword */
    od_add(0x6040, 0, CO_ACCESS_RW, CO_TYPE_U16, 2, &od_controlword,  0, "Controlword");
    od_add(0x6041, 0, CO_ACCESS_RO, CO_TYPE_U16, 2, &od_statusword,   0, "Statusword");

    /* 速度模式对象 */
    od_add(0x6042, 0, CO_ACCESS_RW, CO_TYPE_I16, 2, &g_motor.target_speed_rpm_x100, 0, "vl Target Velocity");
    od_add(0x6048, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.profile_accel, 1000, "vl Velocity Acceleration");
    od_add(0x6049, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.profile_decel, 1000, "vl Velocity Deceleration");
    od_add(0x60A3, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.profile_jerk, 0, "Profile Jerk");

    /* 位置模式对象 */
    od_add(0x6062, 0, CO_ACCESS_RO, CO_TYPE_I32, 4, &g_motor.feedback.position_pulses, 0, "Position Demand Value");
    od_add(0x6064, 0, CO_ACCESS_RO, CO_TYPE_I32, 4, &g_motor.feedback.position_pulses, 0, "Position Actual Value");
    od_add(0x606B, 0, CO_ACCESS_RW, CO_TYPE_I32, 4, &g_motor.target_speed_rpm_x100, 0, "Velocity Demand");
    od_add(0x606C, 0, CO_ACCESS_RO, CO_TYPE_I32, 4, &g_motor.feedback.speed_rpm_x100, 0, "Velocity Actual Value");
    od_add(0x607A, 0, CO_ACCESS_RW, CO_TYPE_I32, 4, &g_motor.target_position_pulses, 0, "Target Position");
    od_add(0x607D, 0, CO_ACCESS_RW, CO_TYPE_I32, 4, &g_motor.sw_position_min, -1000000, "Software Position Limit Min");
    od_add(0x607E, 0, CO_ACCESS_RW, CO_TYPE_I32, 4, &g_motor.sw_position_max,  1000000, "Software Position Limit Max");
    od_add(0x6081, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.profile_accel, 1000, "Profile Velocity");
    od_add(0x6083, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.profile_accel, 5000, "Profile Acceleration");
    od_add(0x6084, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.profile_decel, 5000, "Profile Deceleration");
    od_add(0x60F2, 0, CO_ACCESS_RW, CO_TYPE_U16, 2, &g_motor.position_mode_is_relative, 0, "Positioning Option Code");

    /* 力矩模式对象 */
    od_add(0x6071, 0, CO_ACCESS_RW, CO_TYPE_I16, 2, &g_motor.target_torque_nm_x100, 0, "Target Torque");
    od_add(0x6072, 0, CO_ACCESS_RW, CO_TYPE_U16, 2, &g_motor.prot_thresholds.overload_torque, 0, "Max Torque");
    od_add(0x6077, 0, CO_ACCESS_RO, CO_TYPE_I16, 2, &g_motor.feedback.torque_nm_x100, 0, "Torque Actual Value");
    od_add(0x6087, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.torque_slope, 100, "Torque Slope");

    /* 控制模式 */
    od_add(0x6060, 0, CO_ACCESS_RW, CO_TYPE_I8, 1, &od_mode_of_operation, 0, "Modes of Operation");
    od_add(0x6061, 0, CO_ACCESS_RO, CO_TYPE_I8, 1, &od_mode_display, 0, "Modes of Operation Display");

    /* 归零模式对象 */
    od_add(0x6098, 0, CO_ACCESS_RW, CO_TYPE_I8,  1, &g_motor.homing_method,       -1, "Homing Method");
    od_add(0x6099, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.homing_speed_switch, 500, "Homing Speed Switch");
    od_add(0x609A, 0, CO_ACCESS_RW, CO_TYPE_U32, 4, &g_motor.homing_speed_zero,   100, "Homing Speed Zero");
    od_add(0x60E3, 0, CO_ACCESS_RW, CO_TYPE_I32, 4, &g_motor.home_offset, 0, "Home Offset");

    /*=======================================================================
     * 制造商自定义区域 (0x2000 ~ 0x5FFF)
     *=======================================================================*/

    /* --- 电机参数 (0x2000 ~ 0x200B) --- */
    od_add(0x2000, 1, CO_ACCESS_RW, CO_TYPE_U16, 2, &g_motor.motor_params.encoder_ppr,      1000, "Encoder PPR");
    od_add(0x2000, 2, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.rated_voltage,   0, "Rated Voltage");
    od_add(0x2000, 3, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.rated_current,   0, "Rated Current");
    od_add(0x2000, 4, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.peak_current,    0, "Peak Current");
    od_add(0x2000, 5, CO_ACCESS_RW, CO_TYPE_I16,  2, &g_motor.motor_params.rated_speed,      3000, "Rated Speed");
    od_add(0x2000, 6, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.rated_torque,    0, "Rated Torque");
    od_add(0x2000, 7, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.torque_constant, 0, "Torque Constant");
    od_add(0x2000, 8, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.bemf_constant,   0, "BEMF Constant");
    od_add(0x2000, 9, CO_ACCESS_RW, CO_TYPE_U8,   1, &g_motor.motor_params.pole_pairs,       4, "Pole Pairs");
    od_add(0x2000, 10, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.phase_resistance, 0, "Phase Resistance");
    od_add(0x2000, 11, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.motor_params.phase_inductance, 0, "Phase Inductance");

    /* --- PID 参数 (0x2020 ~ 0x2029) --- */
    od_add(0x2020, 1, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.current.kp,  0, "CUR Kp");
    od_add(0x2020, 2, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.current.ki,  0, "CUR Ki");
    od_add(0x2020, 3, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.current.kd,  0, "CUR Kd");
    od_add(0x2021, 1, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.speed.kp,    0, "SPD Kp");
    od_add(0x2021, 2, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.speed.ki,    0, "SPD Ki");
    od_add(0x2021, 3, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.speed.kd,    0, "SPD Kd");
    od_add(0x2022, 1, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.position.kp, 0, "POS Kp");
    od_add(0x2022, 2, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.position.ki, 0, "POS Ki");
    od_add(0x2022, 3, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.pid_params.position.kd, 0, "POS Kd");

    /* --- 保护阈值 (0x2040 ~ 0x204A) --- */
    od_add(0x2040, 1, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.overvoltage_threshold,  0, "OV Threshold");
    od_add(0x2040, 2, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.undervoltage_threshold, 0, "UV Threshold");
    od_add(0x2040, 3, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.overcurrent_threshold,  0, "OC Threshold");
    od_add(0x2040, 4, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.overload_torque,        0, "OL Torque");
    od_add(0x2040, 5, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.overload_time,          0, "OL Time");
    od_add(0x2040, 6, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.stall_current,          0, "Stall Current");
    od_add(0x2040, 7, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.stall_time,             0, "Stall Time");
    od_add(0x2040, 8, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.overtemp_driver,        0, "OT Driver");
    od_add(0x2040, 9, CO_ACCESS_RW, CO_TYPE_FLOAT, 4, &g_motor.prot_thresholds.overtemp_motor,         0, "OT Motor");
    od_add(0x2040, 10, CO_ACCESS_RW, CO_TYPE_U16, 2, &g_motor.prot_thresholds.comm_timeout_ms, 500, "Comm Timeout");

    /* --- 校准数据 (0x2100 ~ 0x2110) --- */
    od_add(0x2100, 1, CO_ACCESS_RW, CO_TYPE_I32, 4, &g_motor.zero_offset_pulses, 0, "Zero Offset");

    /* --- 系统信息 (0x2101 ~ 0x2108) --- */
    od_add(0x2101, 0, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_serial_number,   0, "Serial Number");
    od_add(0x2102, 0, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_runtime_hours,   0, "Runtime Hours");
    od_add(0x2103, 0, CO_ACCESS_RO, CO_TYPE_U32, 4, &od_firmware_version, 0, "Firmware Version");

    LOG_INFO("OBJDICT", "Object dictionary initialized: %d entries", g_objdict_count);
}

/*===========================================================================
 * SDO 读
 *===========================================================================*/
uint8_t co_objdict_read(uint16_t index, uint8_t sub_index,
                        uint8_t *data, uint32_t *size)
{
    const co_objdict_entry_t *entry = co_objdict_find(index, sub_index);
    if (!entry) return 0;

    if (entry->access == CO_ACCESS_WO) return 0; /* 只写不可读 */

    uint32_t copy_size = MIN(*size, entry->data_size);
    if (entry->data_ptr) {
        memcpy(data, entry->data_ptr, copy_size);
    }
    *size = copy_size;
    return 1;
}

/*===========================================================================
 * SDO 写
 *
 * 特别注意: CiA 402 控制模式切换时同步更新内部状态
 *===========================================================================*/
uint8_t co_objdict_write(uint16_t index, uint8_t sub_index,
                         const uint8_t *data, uint32_t size)
{
    const co_objdict_entry_t *entry = co_objdict_find(index, sub_index);
    if (!entry) return 0;

    if (entry->access == CO_ACCESS_RO || entry->access == CO_ACCESS_CONST) {
        return 0; /* 只读/常量拒绝写入 */
    }

    /* 类型大小检查 */
    if (size > entry->data_size) return 0;

    /* 写入数据 */
    if (entry->data_ptr) {
        memcpy(entry->data_ptr, data, size);
    }

    /* 特殊处理: CiA 402 控制字 */
    if (index == 0x6040 && sub_index == 0) {
        co_cia402_process_controlword(*(uint16_t *)entry->data_ptr);
    }

    /* 特殊处理: 控制模式切换 */
    if (index == 0x6060 && sub_index == 0) {
        co_cia402_set_mode(*(int8_t *)entry->data_ptr);
        od_mode_display = *(int8_t *)entry->data_ptr;
    }

    return 1;
}

/*===========================================================================
 * 查找条目
 *===========================================================================*/
const co_objdict_entry_t *co_objdict_find(uint16_t index, uint8_t sub_index)
{
    for (uint16_t i = 0; i < g_objdict_count; i++) {
        if (g_objdict[i].index == index && g_objdict[i].sub_index == sub_index) {
            return &g_objdict[i];
        }
    }
    return NULL;
}

uint16_t co_objdict_get_count(void)
{
    return g_objdict_count;
}
