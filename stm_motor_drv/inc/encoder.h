/**
 * @file    encoder.h
 * @brief   直流电机驱动 — 编码器接口（M/T 测速法）
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "motor_config.h"

/* 编码器数据结构 */
typedef struct {
    int32_t  raw_count;             /* 原始脉冲计数 */
    int32_t  last_count;            /* 上一次脉冲计数 */
    int32_t  position_pulses;       /* 位置 (脉冲): 原始值 - 零点偏移 */
    int32_t  speed_rpm_x100;        /* 速度 RPM×100 */
    int16_t  speed_rpm;             /* 速度 RPM (用于快速读取) */
    uint32_t last_capture_tick;     /* 上次捕获时间 (us) */

    /* M/T 法测速 */
    uint32_t m1;                    /* 定时周期内编码器脉冲数 */
    uint32_t m2;                    /* 定时周期内高频时钟脉冲数 */
    uint32_t mt_prev_count;
    uint32_t mt_prev_tick;
    uint32_t speed_update_tick_us;
} encoder_data_t;

/* 编码器初始化 */
void encoder_init(void);

/* 原始计数读取（在中断/PID 循环中调用） */
int32_t encoder_get_raw_count(void);

/* 位置获取 (已减去零点偏移) */
int32_t encoder_get_position(void);

/* 速度获取 RPM×100 */
int32_t encoder_get_speed_rpm_x100(void);

/* M/T 法速度更新 (在速度环 1ms 定时器中调用) */
void encoder_speed_update(void);

/* 设置零点偏移 */
void encoder_set_zero_offset(int32_t offset);

/* 获取零点偏移 */
int32_t encoder_get_zero_offset(void);

/* 编码器故障检测（AB 相同时跳变） */
uint8_t encoder_check_fault(void);

/* 获取编码器数据指针 */
encoder_data_t *encoder_get_data(void);

#endif /* __ENCODER_H */
