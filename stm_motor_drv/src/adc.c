/**
 * @file    adc.c
 * @brief   ADC 采样 — ADC1+DMA 多通道 (电流/电压/温度)
 *          F1: DMA1_Channel1    F4: DMA2_Stream0
 */

#include "adc.h"
#include <math.h>
#include <string.h>

static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma_adc;
static adc_data_t g_adc_data;
static float g_current_offset_mv = CURRENT_ZERO_OFFSET_MV;

void adc_init(void)
{
    memset(&g_adc_data, 0, sizeof(adc_data_t));

    ADC_CLK_ENABLE();
    ADC_GPIO_CLK_ENABLE();

#if STM32_PLATFORM == STM32_PLATFORM_F1
    __HAL_RCC_DMA1_CLK_ENABLE();
#else
    __HAL_RCC_DMA2_CLK_ENABLE();
#endif

    /* GPIO 模拟输入 */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio_init.Mode  = GPIO_MODE_ANALOG;
    gpio_init.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_GPIO_PORT, &gpio_init);

    /* ADC 通用配置 */
    hadc.Instance                   = ADC_INSTANCE;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    hadc.Init.ScanConvMode          = ADC_SCAN_ENABLE;
#else
    hadc.Init.ScanConvMode          = ADC_SCAN_ENABLE;
#endif
    hadc.Init.ContinuousConvMode    = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion       = ADC_CH_COUNT;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    hadc.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    HAL_ADC_Init(&hadc);

    /* F1: 通道采样时间配置 */
    ADC_ChannelConfTypeDef ch_conf = {0};
    ch_conf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    ch_conf.Channel = ADC_CH_CURRENT;  ch_conf.Rank = 1; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
    ch_conf.Channel = ADC_CH_VOLTAGE;   ch_conf.Rank = 2; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
    ch_conf.Channel = ADC_CH_TEMP_DRV;  ch_conf.Rank = 3; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
    ch_conf.Channel = ADC_CH_TEMP_MOT;  ch_conf.Rank = 4; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
#else
    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc);

    ADC_ChannelConfTypeDef ch_conf = {0};
    ch_conf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    ch_conf.Channel = ADC_CH_CURRENT;  ch_conf.Rank = 1; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
    ch_conf.Channel = ADC_CH_VOLTAGE;   ch_conf.Rank = 2; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
    ch_conf.Channel = ADC_CH_TEMP_DRV;  ch_conf.Rank = 3; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
    ch_conf.Channel = ADC_CH_TEMP_MOT;  ch_conf.Rank = 4; HAL_ADC_ConfigChannel(&hadc, &ch_conf);
#endif

    /* DMA 配置 */
#if STM32_PLATFORM == STM32_PLATFORM_F1
    hdma_adc.Instance                 = ADC_DMA_INSTANCE; /* DMA1 */
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode                = DMA_CIRCULAR;
    hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc);
    __HAL_LINKDMA(&hadc, DMA_Handle, hdma_adc);
#else
    hdma_adc.Instance                 = ADC_DMA_INSTANCE; /* DMA2 */
    hdma_adc.Init.Channel             = ADC_DMA_CHANNEL;
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode                = DMA_CIRCULAR;
    hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc);
    __HAL_LINKDMA(&hadc, DMA_Handle, hdma_adc);
#endif

    /* 启动 ADC + DMA */
#if STM32_PLATFORM == STM32_PLATFORM_F1
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)g_adc_data.raw, ADC_CH_COUNT);
#else
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)g_adc_data.raw, ADC_CH_COUNT);
#endif
}

/* ADC 转换完成回调 */
void adc_conv_complete_callback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC_INSTANCE) return;

    float mv_current = (float)g_adc_data.raw[0] * ADC_VREF_MV / ADC_RESOLUTION;
    g_adc_data.current_a = (mv_current - g_current_offset_mv) / CURRENT_SENSE_MV_PER_A;

    float mv_voltage = (float)g_adc_data.raw[1] * ADC_VREF_MV / ADC_RESOLUTION;
    g_adc_data.voltage_v = mv_voltage * VOLTAGE_DIVIDER_RATIO / 1000.0f;

    /* 驱动器温度 NTC */
    float mv_temp_drv = (float)g_adc_data.raw[2] * ADC_VREF_MV / ADC_RESOLUTION;
    if (mv_temp_drv > 10.0f && mv_temp_drv < (ADC_VREF_MV - 10.0f)) {
        float r_ntc = TEMP_SERIES_RESISTOR * mv_temp_drv / (ADC_VREF_MV - mv_temp_drv);
        g_adc_data.temp_driver_c = 1.0f / (1.0f / 298.15f +
            logf(r_ntc / TEMP_NTC_R25) / TEMP_NTC_BETA) - 273.15f;
    }

    /* 电机温度 NTC */
    float mv_temp_mot = (float)g_adc_data.raw[3] * ADC_VREF_MV / ADC_RESOLUTION;
    if (mv_temp_mot > 10.0f && mv_temp_mot < (ADC_VREF_MV - 10.0f)) {
        float r_ntc = TEMP_SERIES_RESISTOR * mv_temp_mot / (ADC_VREF_MV - mv_temp_mot);
        g_adc_data.temp_motor_c = 1.0f / (1.0f / 298.15f +
            logf(r_ntc / TEMP_NTC_R25) / TEMP_NTC_BETA) - 273.15f;
    }

    g_adc_data.last_update_tick = HAL_GetTick();
}

adc_data_t *adc_get_data(void)            { return &g_adc_data; }
float adc_get_current_a(void)             { return g_adc_data.current_a; }
float adc_get_voltage_v(void)             { return g_adc_data.voltage_v; }
float adc_get_temp_driver_c(void)         { return g_adc_data.temp_driver_c; }
float adc_get_temp_motor_c(void)          { return g_adc_data.temp_motor_c; }

void adc_calibrate_current_offset(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 128; i++) {
        sum += g_adc_data.raw[0];
        HAL_Delay(1);
    }
    g_current_offset_mv = (float)(sum / 128) * ADC_VREF_MV / ADC_RESOLUTION;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc_conv_complete_callback(hadc);
}
