/**
 * @file    pwm.c
 * @brief   PWM 输出 — TIM1 互补输出 (H桥驱动)
 *          F1: CH1(PA8)+CH1N(PB13) / CH2(PA9)+CH2N(PB14)
 *          F4: CH1(PA8)+CH1N(PB13) / CH2(PA9)+CH2N(PB14)
 */

#include "pwm.h"

static TIM_HandleTypeDef htim_pwm;
static int16_t g_pwm_duty;
static uint8_t g_pwm_enabled = 0;

void pwm_init(void)
{
    g_pwm_duty = 0;

    PWM_TIM_CLK_ENABLE();
    PWM_GPIO_CLK_ENABLE();
    PWM_GPIOB_CLK_ENABLE();

    /* GPIO 配置 */
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLDOWN;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;

#if STM32_PLATFORM == STM32_PLATFORM_F1
    /* F1: 无需 AF 编号，复用推挽即可 */
    gpio_init.Pin = PWM_GPIO_PIN_POS;
    HAL_GPIO_Init(PWM_GPIO_PORT, &gpio_init);
    gpio_init.Pin = PWM_GPIO_PIN_NEG;
    HAL_GPIO_Init(PWM_GPIO_PORT, &gpio_init);
    gpio_init.Pin = PWM_GPIO_PIN_CH1N;
    HAL_GPIO_Init(PWM_GPIO_PORT_BRIDGE, &gpio_init);
    gpio_init.Pin = PWM_GPIO_PIN_CH2N;
    HAL_GPIO_Init(PWM_GPIO_PORT_BRIDGE, &gpio_init);
#else
    gpio_init.Alternate = PWM_GPIO_AF;
    gpio_init.Pin = PWM_GPIO_PIN_POS;
    HAL_GPIO_Init(PWM_GPIO_PORT, &gpio_init);
    gpio_init.Pin = PWM_GPIO_PIN_NEG;
    HAL_GPIO_Init(PWM_GPIO_PORT, &gpio_init);
    gpio_init.Pin = PWM_GPIO_PIN_CH1N;
    HAL_GPIO_Init(PWM_GPIO_PORT_BRIDGE, &gpio_init);
    gpio_init.Pin = PWM_GPIO_PIN_CH2N;
    HAL_GPIO_Init(PWM_GPIO_PORT_BRIDGE, &gpio_init);
#endif

    /* TIM1 中心对齐 PWM 配置 */
    htim_pwm.Instance               = PWM_TIM;
    htim_pwm.Init.Prescaler         = 0;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    htim_pwm.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
#else
    htim_pwm.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED3;
#endif
    htim_pwm.Init.Period            = PWM_PERIOD;
    htim_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim_pwm.Init.RepetitionCounter  = 0;
    htim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim_pwm);

    /* 死区配置 */
    TIM_BreakDeadTimeConfigTypeDef bdt = {0};
    bdt.OffStateRunMode  = TIM_OSSR_ENABLE;
    bdt.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bdt.LockLevel        = TIM_LOCKLEVEL_1;
    bdt.DeadTime         = (PWM_DEAD_TIME_NS * (APB2_CLOCK_FREQ_HZ / 1000000UL) / 1000UL);
    bdt.BreakState       = TIM_BREAK_DISABLE;
    bdt.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    bdt.BreakFilter      = 0;
#if STM32_PLATFORM != STM32_PLATFORM_F1
    bdt.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
#endif
    HAL_TIMEx_ConfigBreakDeadTime(&htim_pwm, &bdt);

    /* PWM 通道配置 */
    TIM_OC_InitTypeDef oc_init = {0};
    oc_init.OCMode       = TIM_OCMODE_PWM1;
    oc_init.Pulse        = 0;
    oc_init.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc_init.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc_init.OCFastMode   = TIM_OCFAST_ENABLE;
    oc_init.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc_init.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim_pwm, &oc_init, PWM_TIM_CHANNEL_POS);
    HAL_TIM_PWM_ConfigChannel(&htim_pwm, &oc_init, PWM_TIM_CHANNEL_NEG);

    /* 启动 PWM (初始占空比=0) */
    HAL_TIM_PWM_Start(&htim_pwm, PWM_TIM_CHANNEL_POS);
    HAL_TIMEx_PWMN_Start(&htim_pwm, PWM_TIM_CHANNEL_POS);
    HAL_TIM_PWM_Start(&htim_pwm, PWM_TIM_CHANNEL_NEG);
    HAL_TIMEx_PWMN_Start(&htim_pwm, PWM_TIM_CHANNEL_NEG);

    g_pwm_enabled = 1;
}

void pwm_set_duty(int16_t duty)
{
    if (!g_pwm_enabled) return;

    duty = CLAMP(duty, -1000, 1000);
    g_pwm_duty = duty;

    uint32_t pulse = (uint32_t)((uint32_t)PWM_PERIOD * (uint32_t)(ABS(duty)) / 1000UL);

    if (duty > 1) {
        __HAL_TIM_SET_COMPARE(&htim_pwm, PWM_TIM_CHANNEL_POS, pulse);
        __HAL_TIM_SET_COMPARE(&htim_pwm, PWM_TIM_CHANNEL_NEG, 0);
    } else if (duty < -1) {
        __HAL_TIM_SET_COMPARE(&htim_pwm, PWM_TIM_CHANNEL_POS, 0);
        __HAL_TIM_SET_COMPARE(&htim_pwm, PWM_TIM_CHANNEL_NEG, pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim_pwm, PWM_TIM_CHANNEL_POS, 0);
        __HAL_TIM_SET_COMPARE(&htim_pwm, PWM_TIM_CHANNEL_NEG, 0);
    }
}

void pwm_emergency_stop(stop_mode_t mode)
{
    HAL_TIM_PWM_Stop(&htim_pwm, PWM_TIM_CHANNEL_POS);
    HAL_TIMEx_PWMN_Stop(&htim_pwm, PWM_TIM_CHANNEL_POS);
    HAL_TIM_PWM_Stop(&htim_pwm, PWM_TIM_CHANNEL_NEG);
    HAL_TIMEx_PWMN_Stop(&htim_pwm, PWM_TIM_CHANNEL_NEG);

    if (mode == STOP_MODE_BRAKE) {
        TIM1->CCER |= TIM_CCER_CC1NE;
        TIM1->CCER |= TIM_CCER_CC2NE;
    }

    g_pwm_duty = 0;
    g_pwm_enabled = 0;
}

void pwm_stop(stop_mode_t mode)
{
    pwm_set_duty(0);
    if (mode == STOP_MODE_BRAKE) {
        HAL_Delay(100);
    }
    HAL_TIM_PWM_Stop(&htim_pwm, PWM_TIM_CHANNEL_POS);
    HAL_TIMEx_PWMN_Stop(&htim_pwm, PWM_TIM_CHANNEL_POS);
    HAL_TIM_PWM_Stop(&htim_pwm, PWM_TIM_CHANNEL_NEG);
    HAL_TIMEx_PWMN_Stop(&htim_pwm, PWM_TIM_CHANNEL_NEG);
    g_pwm_enabled = 0;
}

void pwm_enable(uint8_t enable)
{
    if (enable && !g_pwm_enabled) {
        HAL_TIM_PWM_Start(&htim_pwm, PWM_TIM_CHANNEL_POS);
        HAL_TIMEx_PWMN_Start(&htim_pwm, PWM_TIM_CHANNEL_POS);
        HAL_TIM_PWM_Start(&htim_pwm, PWM_TIM_CHANNEL_NEG);
        HAL_TIMEx_PWMN_Start(&htim_pwm, PWM_TIM_CHANNEL_NEG);
        g_pwm_enabled = 1;
    } else if (!enable && g_pwm_enabled) {
        HAL_TIM_PWM_Stop(&htim_pwm, PWM_TIM_CHANNEL_POS);
        HAL_TIMEx_PWMN_Stop(&htim_pwm, PWM_TIM_CHANNEL_POS);
        HAL_TIM_PWM_Stop(&htim_pwm, PWM_TIM_CHANNEL_NEG);
        HAL_TIMEx_PWMN_Stop(&htim_pwm, PWM_TIM_CHANNEL_NEG);
        g_pwm_enabled = 0;
    }
}

int16_t pwm_get_duty(void)
{
    return g_pwm_duty;
}
