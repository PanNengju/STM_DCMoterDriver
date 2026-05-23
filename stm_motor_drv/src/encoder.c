/**
 * @file    encoder.c
 * @brief   编码器接口 — TIM2 编码器模式 + M/T 法测速
 *          F1/F4 兼容: 编码器计数 + 1MHz 辅助定时器
 */

#include "encoder.h"
#include <string.h>

static encoder_data_t g_encoder;
static TIM_HandleTypeDef htim_encoder;
static TIM_HandleTypeDef htim_mt;

void encoder_init(void)
{
    memset(&g_encoder, 0, sizeof(encoder_data_t));

    /* 编码器定时器 (TIM2) */
    ENC_TIM_CLK_ENABLE();
    ENC_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin       = ENC_GPIO_PIN_A | ENC_GPIO_PIN_B;
    gpio_init.Mode      = GPIO_MODE_INPUT;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ENC_GPIO_PORT, &gpio_init);

    htim_encoder.Instance               = ENC_TIM;
    htim_encoder.Init.Prescaler         = 0;
    htim_encoder.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim_encoder.Init.Period            = 0xFFFF;
    htim_encoder.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_encoder.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim_encoder);

#if STM32_PLATFORM == STM32_PLATFORM_F1
    /* F1 的 HAL_TIM_Encoder_Init 需要单独实现或使用 LL 驱动 */
    /* 这里直接操作寄存器: 设置 SMCR 为编码器模式3 */
    htim_encoder.Instance->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    htim_encoder.Instance->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0
                                 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1
                                 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_3
                                 | TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1
                                 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_3;
    htim_encoder.Instance->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
#else
    TIM_Encoder_InitTypeDef enc_config = {0};
    enc_config.EncoderMode    = TIM_ENCODERMODE_TI12;
    enc_config.IC1Polarity    = TIM_ICPOLARITY_RISING;
    enc_config.IC1Selection   = TIM_ICSELECTION_DIRECTTI;
    enc_config.IC1Prescaler   = TIM_ICPSC_DIV1;
    enc_config.IC1Filter      = 0x0F;
    enc_config.IC2Polarity    = TIM_ICPOLARITY_RISING;
    enc_config.IC2Selection   = TIM_ICSELECTION_DIRECTTI;
    enc_config.IC2Prescaler   = TIM_ICPSC_DIV1;
    enc_config.IC2Filter      = 0x0F;
    HAL_TIM_Encoder_Init(&htim_encoder, &enc_config);
#endif

    __HAL_TIM_SET_COUNTER(&htim_encoder, 32767);
    HAL_TIM_Encoder_Start(&htim_encoder, TIM_CHANNEL_ALL);

    /* M/T 法辅助定时器 (1MHz) */
    ENC_MT_TIM_CLK_ENABLE();
    htim_mt.Instance = ENC_MT_TIM;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    htim_mt.Init.Prescaler   = (APB1_CLOCK_FREQ_HZ * 2 / 1000000) - 1;
#else
    htim_mt.Init.Prescaler   = (APB1_CLOCK_FREQ_HZ / 1000000) - 1;
#endif
    htim_mt.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_mt.Init.Period      = 0xFFFF;
    htim_mt.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim_mt);
    HAL_TIM_Base_Start(&htim_mt);

    g_encoder.last_capture_tick = HAL_GetTick();
    g_encoder.speed_update_tick_us = 1000;
}

int32_t encoder_get_raw_count(void)
{
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim_encoder);
    int32_t delta = (int32_t)(cnt - g_encoder.last_count);

    if (delta > 32767)      delta -= 65536;
    else if (delta < -32768) delta += 65536;

    g_encoder.raw_count += delta;
    g_encoder.last_count = cnt;
    return g_encoder.raw_count;
}

int32_t encoder_get_position(void)
{
    encoder_get_raw_count();
    g_encoder.position_pulses = g_encoder.raw_count - g_motor.zero_offset_pulses;
    return g_encoder.position_pulses;
}

void encoder_speed_update(void)
{
    int32_t raw = encoder_get_raw_count();
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - g_encoder.last_capture_tick;
    if (elapsed_ms == 0) return;

    int32_t delta_enc = raw - g_encoder.mt_prev_count;
    uint32_t delta_clk = elapsed_ms * 1000;

    g_encoder.m1 = (uint32_t)((delta_enc >= 0) ? delta_enc : -delta_enc);
    g_encoder.m2 = delta_clk;

    if (delta_clk > 0) {
        int64_t speed_x100 = (int64_t)delta_enc * 1000000 * 60 * 100
                           / ((int64_t)ENC_CPR * (int64_t)delta_clk);
        g_encoder.speed_rpm_x100 = (int32_t)speed_x100;
    } else {
        g_encoder.speed_rpm_x100 = 0;
    }

    g_encoder.speed_rpm = (int16_t)(g_encoder.speed_rpm_x100 / 100);
    g_encoder.mt_prev_count = raw;
    g_encoder.last_capture_tick = now;
}

int32_t encoder_get_speed_rpm_x100(void)   { return g_encoder.speed_rpm_x100; }

void encoder_set_zero_offset(int32_t offset)
{
    g_motor.zero_offset_pulses = offset;
    g_encoder.position_pulses = g_encoder.raw_count - offset;
}

int32_t encoder_get_zero_offset(void)      { return g_motor.zero_offset_pulses; }

uint8_t encoder_check_fault(void)
{
    if (ENC_TIM->SR & TIM_SR_TIF) {
        g_motor.encoder_fault_count++;
        ENC_TIM->SR = (uint16_t)(~(uint32_t)TIM_SR_TIF);
    }
    return (g_motor.encoder_fault_count > 5) ? 1 : 0;
}

encoder_data_t *encoder_get_data(void)     { return &g_encoder; }
