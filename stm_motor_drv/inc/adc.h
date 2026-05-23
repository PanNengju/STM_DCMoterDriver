/**
 * @file    adc.h
 * @brief   直流电机驱动 — ADC 采样模块（DMA多通道）
 */

#ifndef __ADC_H
#define __ADC_H

#include "motor_config.h"

/* ADC 采样数据结构 */
typedef struct {
    uint16_t raw[ADC_CH_COUNT];     /* 原始 ADC 值 */
    float    current_a;             /* 电流 (A) */
    float    voltage_v;             /* 母线电压 (V) */
    float    temp_driver_c;         /* 驱动器温度 (℃) */
    float    temp_motor_c;          /* 电机温度 (℃) */
    uint32_t last_update_tick;
} adc_data_t;

/* ADC DMA 初始化 */
void adc_init(void);

/* 获取最新 ADC 数据 */
adc_data_t *adc_get_data(void);

/* 获取电流 (A) */
float adc_get_current_a(void);

/* 获取电压 (V) */
float adc_get_voltage_v(void);

/* 获取驱动器温度 (℃) */
float adc_get_temp_driver_c(void);

/* 获取电机温度 (℃) */
float adc_get_temp_motor_c(void);

/* ADC DMA 回调 — 在 HAL_ADC_ConvCpltCallback 中调用 */
void adc_conv_complete_callback(ADC_HandleTypeDef *hadc);

/* 电流零漂校准 */
void adc_calibrate_current_offset(void);

#endif /* __ADC_H */
