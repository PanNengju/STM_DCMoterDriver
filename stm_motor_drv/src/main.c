/**
 * @file    main.c
 * @brief   直流电机驱动 — 主入口
 *          系统时钟 / 外设初始化 / FreeRTOS 任务创建 / 异常处理
 *          motor_drv 模块只含电机控制逻辑，所有 MCU 配置在此文件
 */

#include "motor_drv.h"

/* FreeRTOS port.c 中的实际处理函数 */
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

/*===========================================================================
 * 电流环定时器配置 (TIM3 — APB1, 20kHz)
 * F1: TIM3 clk = 72MHz (APB1×2), Period = 72M/20k-1 = 3599
 * F4: TIM3 clk = 84MHz (APB1×2), Period = 84M/20k-1 = 4199
 *===========================================================================*/
#define CURRENT_LOOP_TIM               TIM3
#define CURRENT_LOOP_TIM_CLK_ENABLE()  __HAL_RCC_TIM3_CLK_ENABLE()
#define CURRENT_LOOP_TIM_IRQn          TIM3_IRQn
#define CURRENT_LOOP_TIM_IRQHandler    TIM3_IRQHandler

#if STM32_PLATFORM == STM32_PLATFORM_F1
#define CURRENT_LOOP_TIM_PERIOD        (72000000UL / CURRENT_LOOP_FREQ_HZ - 1)  /* 3599 */
#elif STM32_PLATFORM == STM32_PLATFORM_F4
#define CURRENT_LOOP_TIM_PERIOD        (84000000UL / CURRENT_LOOP_FREQ_HZ - 1)  /* 4199 */
#endif

/*===========================================================================
 * 看门狗配置
 * F1: IWDG, LSI=40kHz, /32 prescaler, reload=1250 → ~1s
 * F4: WWDG, PCLK1=42MHz, /4096/8 prescaler, window=0x7F, reload=0x7F
 *===========================================================================*/
#if STM32_PLATFORM == STM32_PLATFORM_F1
#define IWDG_PRESCALER                IWDG_PRESCALER_32
#define IWDG_RELOAD                   1250
#define IWDG_TIMEOUT_MS               1000
#else
#define WWDG_TIMEOUT_CYCLES           0x7F
#endif

/*===========================================================================
 * 私有函数声明
 *===========================================================================*/
static void system_clock_config(void);
static void current_loop_timer_init(void);
static void wdg_init(void);

/*===========================================================================
 * main
 *===========================================================================*/
int main(void)
{
    /* 1. HAL 库初始化 */
    HAL_Init();

    /* 2. 系统时钟配置 */
    system_clock_config();

    /* 3. 外设初始化 */
    pwm_init();
    encoder_init();
    adc_init();
    protection_init();
    uart_cli_init();
    canopen_init();

    /* 4. 电流环定时器 (20kHz 中断) */
    current_loop_timer_init();

    /* 5. 看门狗 */
    wdg_init();

    /* 6. 电机驱动初始化 (Flash / PID / 状态机 / 互斥锁) */
    motor_drv_init();

    /* 7. 创建 FreeRTOS 任务 */
    xTaskCreate(motor_task_speed_loop,
                "SpeedLoop",
                TASK_STACK_SPEED_LOOP,
                NULL,
                TASK_PRIO_SPEED_LOOP,
                NULL);

    xTaskCreate(motor_task_position_loop,
                "PosLoop",
                TASK_STACK_POSITION_LOOP,
                NULL,
                TASK_PRIO_POSITION_LOOP,
                NULL);

    xTaskCreate(canopen_task,
                "CANopen",
                TASK_STACK_CANOPEN,
                NULL,
                TASK_PRIO_CANOPEN,
                NULL);

    xTaskCreate(uart_cli_task,
                "UartCLI",
                TASK_STACK_UART_CLI,
                NULL,
                TASK_PRIO_UART_CLI,
                NULL);

    xTaskCreate(motor_task_monitor,
                "Monitor",
                TASK_STACK_MONITOR,
                NULL,
                TASK_PRIO_MONITOR,
                NULL);

    /* 8. 启动调度器 */
    vTaskStartScheduler();

    /* never reach here */
    for (;;) {}
}

/*===========================================================================
 * 系统时钟配置
 * F1: HSE 8MHz → PLL×9 → 72MHz, APB1/36MHz, APB2/72MHz
 * F4: HSE 8MHz → PLL×336/M÷P → 168MHz, APB1/42MHz, APB2/84MHz
 *===========================================================================*/
static void system_clock_config(void)
{
#if STM32_PLATFORM == STM32_PLATFORM_F1
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState       = RCC_HSE_ON;
    osc_init.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc_init.PLL.PLLState   = RCC_PLL_ON;
    osc_init.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc_init.PLL.PLLMUL     = RCC_PLL_MUL;
    HAL_RCC_OscConfig(&osc_init);

    clk_init.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY);

#elif STM32_PLATFORM == STM32_PLATFORM_F4
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState       = RCC_HSE_ON;
    osc_init.PLL.PLLState   = RCC_PLL_ON;
    osc_init.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc_init.PLL.PLLM       = 8;
    osc_init.PLL.PLLN       = 336;
    osc_init.PLL.PLLP       = RCC_PLLP_DIV2;
    osc_init.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&osc_init);

    clk_init.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY);
#endif
}

/*===========================================================================
 * 电流环定时器 — 20kHz 触发电流环 PID
 *===========================================================================*/
static void current_loop_timer_init(void)
{
    CURRENT_LOOP_TIM_CLK_ENABLE();

    g_htim_current.Instance               = CURRENT_LOOP_TIM;
    g_htim_current.Init.Prescaler         = 0;
    g_htim_current.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g_htim_current.Init.Period            = CURRENT_LOOP_TIM_PERIOD;
    g_htim_current.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    g_htim_current.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&g_htim_current);

    HAL_NVIC_SetPriority(CURRENT_LOOP_TIM_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CURRENT_LOOP_TIM_IRQn);

    __HAL_TIM_ENABLE_IT(&g_htim_current, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&g_htim_current);
}

/*===========================================================================
 * 看门狗初始化
 * F1: IWDG (独立看门狗, LSI 40kHz)
 * F4: WWDG (窗口看门狗)
 *===========================================================================*/
static void wdg_init(void)
{
#if STM32_PLATFORM == STM32_PLATFORM_F1
    g_motor.hiwdg.Instance       = IWDG;
    g_motor.hiwdg.Init.Prescaler = IWDG_PRESCALER;
    g_motor.hiwdg.Init.Reload    = IWDG_RELOAD;
    HAL_IWDG_Init(&g_motor.hiwdg);
#else
    __HAL_RCC_WWDG_CLK_ENABLE();
    g_motor.hwwdg.Instance       = WWDG;
    g_motor.hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
    g_motor.hwwdg.Init.Window    = WWDG_TIMEOUT_CYCLES;
    g_motor.hwwdg.Init.Counter   = WWDG_TIMEOUT_CYCLES;
    g_motor.hwwdg.Init.EWIMode   = WWDG_EWI_ENABLE;
    HAL_WWDG_Init(&g_motor.hwwdg);
#endif
}

/*===========================================================================
 * 电流环中断服务函数 — 委托给 motor_drv 的 ISR
 *===========================================================================*/
void CURRENT_LOOP_TIM_IRQHandler(void)
{
    CURRENT_LOOP_IRQHandler();
}

/*===========================================================================
 * 异常处理 — 紧急停止 PWM 后死循环
 *===========================================================================*/
void HardFault_Handler(void)
{
    pwm_emergency_stop(STOP_MODE_COAST);
    for (;;) {}
}

void MemManage_Handler(void)
{
    pwm_emergency_stop(STOP_MODE_COAST);
    for (;;) {}
}

void BusFault_Handler(void)
{
    pwm_emergency_stop(STOP_MODE_COAST);
    for (;;) {}
}

void UsageFault_Handler(void)
{
    pwm_emergency_stop(STOP_MODE_COAST);
    for (;;) {}
}

/*===========================================================================
 * FreeRTOS 所需系统定时器
 *===========================================================================*/
void SysTick_Handler(void)
{
    HAL_IncTick();
    xPortSysTickHandler();
}

__attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile ("b vPortSVCHandler");
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile ("b xPortPendSVHandler");
}

/*===========================================================================
 * FreeRTOS Hook 函数
 *===========================================================================*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    pwm_emergency_stop(STOP_MODE_COAST);
    for (;;) {}
}

void vApplicationMallocFailedHook(void)
{
    pwm_emergency_stop(STOP_MODE_COAST);
    for (;;) {}
}
