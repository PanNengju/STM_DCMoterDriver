/**
 * @file    stm32f1xx_hal_conf.h
 * @brief   STM32F1xx HAL 配置头文件
 */

#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* 必须最先包含 HAL 通用类型定义 (HAL_StatusTypeDef 等) */
#include "stm32f1xx_hal_def.h"

/* 使能的 HAL 模块 */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

/* HSE 晶振频率 */
#if !defined(HSE_VALUE)
#define HSE_VALUE   8000000UL
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT   100U
#endif

/* HSI */
#if !defined(HSI_VALUE)
#define HSI_VALUE   8000000UL
#endif

/* LSI */
#if !defined(LSI_VALUE)
#define LSI_VALUE   40000UL
#endif

/* LSE */
#if !defined(LSE_VALUE)
#define LSE_VALUE   32768UL
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT   5000U
#endif

/* SysTick */
#define TICK_INT_PRIORITY   0x0FU

/* 使能 assert */
#define USE_FULL_ASSERT     0U

#if (USE_FULL_ASSERT == 1U)
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */
