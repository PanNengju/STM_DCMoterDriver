/**
 * @file    co_objdict.h
 * @brief   CANopen 对象字典 (Object Dictionary)
 *          支持 CiA 301 标准区域 + CiA 402 驱动行规 + 制造商自定义区域
 */

#ifndef __CO_OBJDICT_H
#define __CO_OBJDICT_H

#include "motor_config.h"

/* 对象访问权限 */
#define CO_ACCESS_RO    0       /* 只读 */
#define CO_ACCESS_WO    1       /* 只写 */
#define CO_ACCESS_RW    2       /* 读写 */
#define CO_ACCESS_CONST 3       /* 常量 */

/* 对象数据类型 */
#define CO_TYPE_U8      0x05
#define CO_TYPE_U16     0x06
#define CO_TYPE_U32     0x07
#define CO_TYPE_I8      0x02
#define CO_TYPE_I16     0x03
#define CO_TYPE_I32     0x04
#define CO_TYPE_FLOAT   0x08
#define CO_TYPE_STRING  0x09

/* 对象字典条目 */
typedef struct {
    uint16_t  index;                /* 索引 */
    uint8_t   sub_index;            /* 子索引 */
    uint8_t   access;               /* 访问权限 */
    uint8_t   data_type;            /* 数据类型 */
    uint32_t  data_size;            /* 数据大小 (字节) */
    void     *data_ptr;             /* 数据指针 (指向 g_motor 或静态变量) */
    uint32_t  default_val;          /* 默认值（用于恢复出厂） */
    const char *name;               /* 名称 */
} co_objdict_entry_t;

/* 对象字典初始化 */
void co_objdict_init(void);

/* SDO 读 */
uint8_t co_objdict_read(uint16_t index, uint8_t sub_index,
                        uint8_t *data, uint32_t *size);

/* SDO 写 */
uint8_t co_objdict_write(uint16_t index, uint8_t sub_index,
                         const uint8_t *data, uint32_t size);

/* 对象字典条目数量 */
uint16_t co_objdict_get_count(void);

/* 查找条目 */
const co_objdict_entry_t *co_objdict_find(uint16_t index, uint8_t sub_index);

#endif /* __CO_OBJDICT_H */
