/**
 * @file    flash.h
 * @brief   直流电机驱动 — Flash 参数持久化存储（磨损均衡）
 */

#ifndef __FLASH_H
#define __FLASH_H

#include "motor_config.h"

/* 参数存储头 */
typedef struct {
    uint32_t magic;                 /* 魔数 0x4D4F544F ("MOTO") */
    uint16_t version;               /* 版本号 */
    uint16_t crc16;                 /* CRC16 校验 */
    uint32_t write_count;           /* 写入次数 */
    uint8_t  slot_index;            /* 当前槽位 */
    uint8_t  reserved[3];
} flash_header_t;

/* Flash 存储槽（一个完整参数集） */
typedef struct {
    flash_header_t  header;
    motor_params_t  motor;
    motor_pid_params_t pid;
    protection_thresholds_t prot;
    int32_t         zero_offset;
    uint8_t         can_node_id;
    uint32_t        can_baudrate;
    uint8_t         padding[32];
} flash_param_slot_t;

/* Flash 模块初始化 */
void flash_init(void);

/* 参数加载（上电时调用） */
uint8_t flash_params_load(void);

/* 参数保存 */
uint8_t flash_params_save(void);

/* 只保存零点偏移 */
uint8_t flash_save_zero_offset(int32_t offset);

/* 恢复出厂设置 */
void flash_restore_factory(void);

/* 获取参数槽指针 */
flash_param_slot_t *flash_get_slot(void);

#endif /* __FLASH_H */
