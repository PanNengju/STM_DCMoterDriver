/**
 * @file    motor_config.h
 * @brief   直流电机驱动 — 参数配置与宏定义
 * @note    兼容 STM32F1/F4/H7 系列
 */

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

/*===========================================================================
 * 平台选择 (编译时通过 -D 定义)
 *===========================================================================*/
#define STM32_PLATFORM_F1   1
#define STM32_PLATFORM_F4   2
#define STM32_PLATFORM_H7   3

#ifndef STM32_PLATFORM
#define STM32_PLATFORM      STM32_PLATFORM_F1
#endif

/*===========================================================================
 * HAL 头文件 — 必须按依赖顺序包含
 * 注意: stm32f1xx_hal.h 只含系统 tick/init, 模块驱动需单独包含
 *       DMA 必须在 ADC 之前 (ADC 使用 DMA_HandleTypeDef)
 *===========================================================================*/
#if STM32_PLATFORM == STM32_PLATFORM_F1
#include "stm32f1xx.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_can.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_iwdg.h"
#elif STM32_PLATFORM == STM32_PLATFORM_F4
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_wwdg.h"
#elif STM32_PLATFORM == STM32_PLATFORM_H7
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

/*===========================================================================
 * 系统时钟配置
 *===========================================================================*/
#if STM32_PLATFORM == STM32_PLATFORM_F1
#define SYS_CLOCK_FREQ_HZ           72000000UL
#define APB1_CLOCK_FREQ_HZ          36000000UL
#define APB2_CLOCK_FREQ_HZ          72000000UL
#define FLASH_LATENCY               FLASH_LATENCY_2
#define RCC_PLL_MUL                 9       /* 8MHz * 9 = 72MHz */
#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define SYS_CLOCK_FREQ_HZ           168000000UL
#define APB1_CLOCK_FREQ_HZ          42000000UL
#define APB2_CLOCK_FREQ_HZ          84000000UL
#define FLASH_LATENCY               FLASH_LATENCY_5
#define RCC_PLL_MUL                 336
#endif

#define TICK_FREQ_HZ                1000U

/*===========================================================================
 * 控制周期 (Hz)
 *===========================================================================*/
#define CURRENT_LOOP_FREQ_HZ        20000U
#define SPEED_LOOP_FREQ_HZ          1000U
#define POSITION_LOOP_FREQ_HZ       200U
#define SYSTEM_MONITOR_FREQ_HZ      100U

#define CURRENT_LOOP_PERIOD_US      (1000000UL / CURRENT_LOOP_FREQ_HZ)
#define SPEED_LOOP_PERIOD_US        (1000000UL / SPEED_LOOP_FREQ_HZ)
#define POSITION_LOOP_PERIOD_US     (1000000UL / POSITION_LOOP_FREQ_HZ)

/*===========================================================================
 * PWM 配置 (TIM1 高级定时器 — APB2 总线)
 * F1: TIM1 在 APB2(72MHz), F4: TIM1 在 APB2(168MHz)
 *===========================================================================*/
#define PWM_TIM                      TIM1
#define PWM_TIM_CLK_ENABLE()         __HAL_RCC_TIM1_CLK_ENABLE()
#define PWM_TIM_CHANNEL_POS          TIM_CHANNEL_1
#define PWM_TIM_CHANNEL_NEG          TIM_CHANNEL_2
#define PWM_FREQ_HZ                  20000U

#if STM32_PLATFORM == STM32_PLATFORM_F1
#define PWM_PERIOD                   ((APB2_CLOCK_FREQ_HZ) / (PWM_FREQ_HZ * 2) - 1)
#define PWM_DEAD_TIME_NS             1000U
#define PWM_GPIO_PORT               GPIOA
#define PWM_GPIO_PIN_POS             GPIO_PIN_8
#define PWM_GPIO_PIN_NEG             GPIO_PIN_9
#define PWM_GPIO_PORT_BRIDGE         GPIOB
#define PWM_GPIO_PIN_CH1N             GPIO_PIN_13
#define PWM_GPIO_PIN_CH2N             GPIO_PIN_14
#define PWM_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define PWM_GPIOB_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define PWM_PERIOD                   ((APB2_CLOCK_FREQ_HZ) / (PWM_FREQ_HZ * 2) - 1)
#define PWM_DEAD_TIME_NS             1000U
#define PWM_GPIO_PORT               GPIOA
#define PWM_GPIO_PIN_POS             GPIO_PIN_8
#define PWM_GPIO_PIN_NEG             GPIO_PIN_9
#define PWM_GPIO_AF                  GPIO_AF1_TIM1
#define PWM_GPIO_PORT_BRIDGE         GPIOB
#define PWM_GPIO_PIN_CH1N             GPIO_PIN_13
#define PWM_GPIO_PIN_CH2N             GPIO_PIN_14
#define PWM_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define PWM_GPIOB_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

/*===========================================================================
 * 编码器配置 (TIM2 — APB1 总线)
 * F1: TIM2 不支持中心对齐编码器模式, 使用 T1/T2 模式
 *===========================================================================*/
#define ENC_TIM                      TIM2
#define ENC_TIM_CLK_ENABLE()         __HAL_RCC_TIM2_CLK_ENABLE()
#define ENC_GPIO_PORT               GPIOA
#define ENC_GPIO_PIN_A               GPIO_PIN_0
#define ENC_GPIO_PIN_B               GPIO_PIN_1
#define ENC_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define ENC_PPR                      1000U
#define ENC_CPR                      (ENC_PPR * 4U)

/* M/T 法辅助定时器 */
#if STM32_PLATFORM == STM32_PLATFORM_F1
#define ENC_MT_TIM                   TIM4
#define ENC_MT_TIM_CLK_ENABLE()      __HAL_RCC_TIM4_CLK_ENABLE()
#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define ENC_MT_TIM                   TIM4
#define ENC_MT_TIM_CLK_ENABLE()      __HAL_RCC_TIM4_CLK_ENABLE()
#endif

/*===========================================================================
 * ADC 配置
 * F1: ADC1 + DMA1_Channel1
 * F4: ADC1 + DMA2_Stream0
 *===========================================================================*/
#define ADC_INSTANCE                 ADC1
#define ADC_CLK_ENABLE()             __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_GPIO_PORT               GPIOA
#define ADC_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADC_CH_CURRENT               ADC_CHANNEL_0    /* PA0 */
#define ADC_CH_VOLTAGE               ADC_CHANNEL_1    /* PA1 */
#define ADC_CH_TEMP_DRV              ADC_CHANNEL_2    /* PA2 */
#define ADC_CH_TEMP_MOT              ADC_CHANNEL_3    /* PA3 */
#define ADC_CH_COUNT                 4U

#define ADC_VREF_MV                  3300U
#define ADC_RESOLUTION               4096U

/* DMA 配置 */
#if STM32_PLATFORM == STM32_PLATFORM_F1
#define ADC_DMA_INSTANCE             DMA1_Channel1
#define ADC_DMA_CHANNEL              DMA1_Channel1
#define ADC_DMA_TC_FLAG             DMA1_FLAG_TC1
#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define ADC_DMA_INSTANCE             DMA2
#define ADC_DMA_STREAM              DMA2_Stream0
#define ADC_DMA_CHANNEL              DMA_CHANNEL_0
#define ADC_DMA_TC_FLAG             DMA_FLAG_TCIF0
#endif

/* 电流/电压/温度传感器参数 */
#define CURRENT_SENSE_MV_PER_A       185U
#define CURRENT_ZERO_OFFSET_MV       1650U
#define VOLTAGE_DIVIDER_RATIO        11.0f
#define TEMP_NTC_BETA                3950.0f
#define TEMP_NTC_R25                 10000.0f
#define TEMP_SERIES_RESISTOR         10000.0f

/*===========================================================================
 * 急停按钮 (外部中断)
 *===========================================================================*/
#define ESTOP_GPIO_PORT             GPIOB
#define ESTOP_GPIO_PIN               GPIO_PIN_0
#define ESTOP_EXTI_LINE              EXTI_LINE_0
#define ESTOP_EXTI_IRQn              EXTI0_IRQn
#define ESTOP_EXTI_IRQHandler        EXTI0_IRQHandler
#define ESTOP_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/*===========================================================================
 * 限位开关
 *===========================================================================*/
#define LIMIT_MIN_GPIO_PORT          GPIOB
#define LIMIT_MIN_GPIO_PIN           GPIO_PIN_12
#define LIMIT_MAX_GPIO_PORT          GPIOB
#define LIMIT_MAX_GPIO_PIN           GPIO_PIN_13

/*===========================================================================
 * LED 指示
 *===========================================================================*/
#define LED_GREEN_GPIO_PORT          GPIOC
#define LED_GREEN_GPIO_PIN           GPIO_PIN_13
#define LED_YELLOW_GPIO_PORT         GPIOC
#define LED_YELLOW_GPIO_PIN          GPIO_PIN_14
#define LED_RED_GPIO_PORT            GPIOC
#define LED_RED_GPIO_PIN             GPIO_PIN_15
#define LED_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

/*===========================================================================
 * CAN 接口
 * F1: CAN1, PB8(RX)/PB9(TX), AFIO Remap
 * F4: CAN1, PD0(RX)/PD1(TX)
 *===========================================================================*/
#define CAN_INSTANCE                  CAN1
#define CAN_CLK_ENABLE()              __HAL_RCC_CAN1_CLK_ENABLE()

#if STM32_PLATFORM == STM32_PLATFORM_F1
#define CAN_GPIO_PORT                GPIOB
#define CAN_GPIO_PIN_RX               GPIO_PIN_8
#define CAN_GPIO_PIN_TX               GPIO_PIN_9
#define CAN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAN_GPIO_REMAP()             __HAL_AFIO_REMAP_CAN1_2()
#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define CAN_GPIO_PORT                GPIOD
#define CAN_GPIO_PIN_RX               GPIO_PIN_0
#define CAN_GPIO_PIN_TX               GPIO_PIN_1
#define CAN_GPIO_AF                   GPIO_AF9_CAN1
#define CAN_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE()
#endif

#define CAN_BAUDRATE                  1000000U

/*===========================================================================
 * UART 调试接口
 * F1: USART2, PA2(TX)/PA3(RX)
 * F4: USART2, PA2(TX)/PA3(RX)
 *===========================================================================*/
#define UART_DEBUG_INSTANCE           USART2
#define UART_DEBUG_CLK_ENABLE()       __HAL_RCC_USART2_CLK_ENABLE()
#define UART_DEBUG_GPIO_PORT          GPIOA
#define UART_DEBUG_GPIO_PIN_TX        GPIO_PIN_2
#define UART_DEBUG_GPIO_PIN_RX        GPIO_PIN_3
#define UART_DEBUG_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART_DEBUG_BAUDRATE           115200U
#define UART_DEBUG_IRQn               USART2_IRQn
#define UART_DEBUG_IRQHandler         USART2_IRQHandler

/*===========================================================================
 * Flash 参数存储
 *===========================================================================*/
#if STM32_PLATFORM == STM32_PLATFORM_F1
/* F1: 使用最后 2 页 (每页 1KB, 0x0800FC00 / 0x0800F800) */
#define FLASH_PARAM_ADDR_SLOT0       0x0800F800U
#define FLASH_PARAM_ADDR_SLOT1       0x0800FA00U
#define FLASH_PARAM_PAGE0            63
#define FLASH_PARAM_PAGE1            62
#define FLASH_WEAR_LEVEL_COUNT       2U
/* FLASH_PAGE_SIZE 由 HAL stm32f1xx_hal_flash_ex.h 提供 (0x400 = 1024) */

#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define FLASH_PARAM_START_ADDR       0x08008000U
#define FLASH_PARAM_SECTOR           FLASH_SECTOR_2
#define FLASH_SLOT_SIZE              8192U
#define FLASH_WEAR_LEVEL_COUNT       2U
#endif

/*===========================================================================
 * FreeRTOS 任务配置
 *===========================================================================*/
#define TASK_STACK_CURRENT_LOOP      192U
#define TASK_STACK_SPEED_LOOP        256U
#define TASK_STACK_POSITION_LOOP     256U
#define TASK_STACK_CANOPEN           512U
#define TASK_STACK_UART_CLI          384U
#define TASK_STACK_MONITOR           192U

#define TASK_PRIO_CURRENT_LOOP       (configMAX_PRIORITIES - 1)
#define TASK_PRIO_SPEED_LOOP         (configMAX_PRIORITIES - 2)
#define TASK_PRIO_POSITION_LOOP      (configMAX_PRIORITIES - 3)
#define TASK_PRIO_CANOPEN            (configMAX_PRIORITIES - 4)
#define TASK_PRIO_MONITOR            (configMAX_PRIORITIES - 5)
#define TASK_PRIO_UART_CLI           (configMAX_PRIORITIES - 6)

/*===========================================================================
 * 通用宏
 *===========================================================================*/
#define ABS(x)                       ((x) >= 0 ? (x) : -(x))
#define MIN(a, b)                    ((a) < (b) ? (a) : (b))
#define MAX(a, b)                    ((a) > (b) ? (a) : (b))
#define CLAMP(val, lo, hi)           (MIN(MAX((val), (lo)), (hi)))
#define ARRAY_SIZE(arr)              (sizeof(arr) / sizeof((arr)[0]))

/*===========================================================================
 * 电机参数结构体
 *===========================================================================*/
typedef struct {
    uint16_t encoder_ppr;
    float    rated_voltage;
    float    rated_current;
    float    peak_current;
    int16_t  rated_speed;
    float    rated_torque;
    float    torque_constant;
    float    bemf_constant;
    uint8_t  pole_pairs;
    float    phase_resistance;
    float    phase_inductance;
} motor_params_t;

/*===========================================================================
 * PID 参数/三段
 *===========================================================================*/
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} pid_params_t;

typedef struct {
    pid_params_t current;
    pid_params_t speed;
    pid_params_t position;
} motor_pid_params_t;

/*===========================================================================
 * 保护阈值
 *===========================================================================*/
typedef struct {
    float overvoltage_threshold;
    float undervoltage_threshold;
    float overcurrent_threshold;
    float overload_torque;
    float overload_time;
    float stall_current;
    float stall_time;
    float overtemp_driver;
    float overtemp_motor;
    uint16_t comm_timeout_ms;
} protection_thresholds_t;

/*===========================================================================
 * 枚举定义
 *===========================================================================*/
typedef enum {
    CTRL_MODE_NONE      = 0,
    CTRL_MODE_SPEED     = 1,
    CTRL_MODE_POSITION  = 2,
    CTRL_MODE_TORQUE    = 3,
    CTRL_MODE_HOMING    = 4,
} control_mode_t;

#define CO_MODE_PP        1U
#define CO_MODE_PV        3U
#define CO_MODE_TQ        4U
#define CO_MODE_HOMING    6U

typedef enum {
    SYS_STATE_INIT          = 0,
    SYS_STATE_DISABLE       = 1,
    SYS_STATE_READY         = 2,
    SYS_STATE_RUNNING       = 3,
    SYS_STATE_QUICK_STOP    = 4,
    SYS_STATE_FAULT         = 5,
    SYS_STATE_ESTOP         = 6,
} system_state_t;

typedef enum {
    ALARM_LVL_NORMAL        = 0,
    ALARM_LVL_WARNING       = 1,
    ALARM_LVL_RECOVERABLE   = 2,
    ALARM_LVL_CRITICAL      = 3,
} alarm_level_t;

typedef enum {
    ERR_NONE                = 0x00,
    ERR_OVERLOAD            = 0x01,
    ERR_OVERCURRENT         = 0x02,
    ERR_STALL               = 0x03,
    ERR_OVERTEMP_DRIVER     = 0x04,
    ERR_OVERTEMP_MOTOR      = 0x05,
    ERR_OVERVOLTAGE         = 0x06,
    ERR_UNDERVOLTAGE        = 0x07,
    ERR_ENCODER_FAULT       = 0x08,
    ERR_COMM_TIMEOUT        = 0x09,
    ERR_ESTOP_TRIGGERED     = 0x0A,
    ERR_INTERNAL            = 0x0B,
} error_code_t;

#define CO_EMCY_OVERCURRENT         0x2310U
#define CO_EMCY_OVERVOLTAGE         0x3210U
#define CO_EMCY_UNDERVOLTAGE        0x3220U
#define CO_EMCY_DRIVER_OVERTEMP     0x4310U
#define CO_EMCY_OVERLOAD            0x8000U
#define CO_EMCY_STALL               0x8611U
#define CO_EMCY_ENCODER_FAULT       0x7305U
#define CO_EMCY_COMM_TIMEOUT        0x8100U

/*===========================================================================
 * 系统反馈数据结构
 *===========================================================================*/
typedef struct {
    int32_t   speed_rpm_x100;
    int32_t   position_pulses;
    int32_t   current_ma;
    int16_t   torque_nm_x100;
    uint16_t  bus_voltage_mv;
    uint8_t   temp_driver_c;
    uint8_t   temp_motor_c;
    uint8_t   status_byte;
    uint8_t   error_code;
} system_feedback_t;

/*===========================================================================
 * 系统全局控制结构体
 *===========================================================================*/
typedef struct motor_system {
    system_state_t      state;
    control_mode_t      ctrl_mode;
    alarm_level_t       alarm_level;
    error_code_t        active_errors[8];
    uint8_t             error_count;

    int32_t             target_speed_rpm_x100;
    int32_t             target_position_pulses;
    int32_t             target_torque_nm_x100;
    int32_t             position_mode_is_relative;

    int32_t             pos_sequence[8];
    uint8_t             pos_sequence_len;
    uint8_t             pos_sequence_idx;

    uint32_t            profile_accel;
    uint32_t            profile_decel;
    uint32_t            profile_jerk;
    int32_t             speed_demand_rpm_x100;

    uint32_t            torque_slope;

    int32_t             sw_position_min;
    int32_t             sw_position_max;

    int8_t              homing_method;
    uint32_t            homing_speed_switch;
    uint32_t            homing_speed_zero;
    int32_t             home_offset;

    int32_t             zero_offset_pulses;

    system_feedback_t   feedback;

    motor_params_t      motor_params;
    motor_pid_params_t  pid_params;
    protection_thresholds_t prot_thresholds;

    uint32_t            overload_start_tick;
    uint32_t            stall_start_tick;
    uint32_t            last_comm_tick;
    uint8_t             encoder_fault_count;

    uint8_t             is_enabled;
    uint8_t             is_running;
    uint8_t             is_estopped;
    uint8_t             need_fault_reset;

    SemaphoreHandle_t   mutex;

#if STM32_PLATFORM == STM32_PLATFORM_F1
    IWDG_HandleTypeDef  hiwdg;
#else
    WWDG_HandleTypeDef  hwwdg;
#endif
} motor_system_t;

/*===========================================================================
 * 全局实例声明
 *===========================================================================*/
extern motor_system_t g_motor;

#endif /* __MOTOR_CONFIG_H */
