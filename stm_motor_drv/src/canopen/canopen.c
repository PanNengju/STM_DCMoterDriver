/**
 * @file    canopen.c
 * @brief   CANopen 主模块 — CAN 硬件驱动 + 帧收发分发
 *          F1: bxCAN (PB8/PB9, AFIO Remap)
 *          F4: bxCAN (PD0/PD1)
 */

#include "canopen.h"
#include "co_nmt.h"
#include "co_sdo.h"
#include "co_pdo.h"
#include "co_emcy.h"
#include "co_sync.h"
#include "co_objdict.h"
#include "co_cia402.h"
#include "protection.h"
#include "uart_cli.h"

static CAN_HandleTypeDef hcan;
static co_nmt_state_t g_nmt_state = CO_NMT_INITIALIZING;
static co_comm_params_t g_comm_params;
static uint32_t g_last_heartbeat_ms;
static uint32_t g_last_sync_ms;
static uint32_t g_can_tx_mailbox;

static void can_hw_init(void)
{
    CAN_CLK_ENABLE();
    CAN_GPIO_CLK_ENABLE();

#if STM32_PLATFORM == STM32_PLATFORM_F1
    /* F1: 开启 AFIO 时钟用于 GPIO 复用和 CAN 重映射 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    CAN_GPIO_REMAP();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin   = CAN_GPIO_PIN_RX;
    gpio_init.Mode  = GPIO_MODE_INPUT;
    gpio_init.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(CAN_GPIO_PORT, &gpio_init);

    gpio_init.Pin   = CAN_GPIO_PIN_TX;
    gpio_init.Mode  = GPIO_MODE_AF_PP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN_GPIO_PORT, &gpio_init);
#else
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin       = CAN_GPIO_PIN_RX | CAN_GPIO_PIN_TX;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = CAN_GPIO_AF;
    HAL_GPIO_Init(CAN_GPIO_PORT, &gpio_init);
#endif

    hcan.Instance                  = CAN_INSTANCE;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    /* F1 CAN: APB1=36MHz, 1Mbps → 36/(1+BS1+BS2)/Prescaler → Prescaler=3, BS1=BS1_8TQ, BS2=BS2_3TQ */
    hcan.Init.Prescaler            = 3;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_8TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_3TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = ENABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
#else
    hcan.Init.Prescaler            = (APB1_CLOCK_FREQ_HZ) / (CAN_BAUDRATE * 16) - 1;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = ENABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
#endif
    HAL_CAN_Init(&hcan);

    /* CAN 过滤器: 接收所有帧 */
    CAN_FilterTypeDef filter = {0};
#if STM32_PLATFORM == STM32_PLATFORM_F1
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
#else
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
#endif
    HAL_CAN_ConfigFilter(&hcan, &filter);
}

void canopen_init(void)
{
    g_comm_params.node_id              = 1;
    g_comm_params.baudrate             = CAN_BAUDRATE;
    g_comm_params.heartbeat_period_ms  = 200;
    g_comm_params.sync_period_ms       = 1;
    g_comm_params.pdo_event_timer_ms   = 10;
    g_comm_params.emcy_inhibit_time_ms = 100;
    g_comm_params.sdo_timeout_ms       = 500;

    can_hw_init();
    co_objdict_init();
    co_nmt_init();
    co_sdo_init();
    co_pdo_init();
    co_emcy_init();
    co_sync_init();
    co_cia402_init();

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    g_nmt_state = CO_NMT_PRE_OPERATIONAL;
    g_last_heartbeat_ms = HAL_GetTick();
    g_last_sync_ms      = HAL_GetTick();

    LOG_INFO("CANOPEN", "Init OK: NodeID=%d, %lubps",
             g_comm_params.node_id, g_comm_params.baudrate);
}

void canopen_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
        uint32_t now = HAL_GetTick();

        if (g_nmt_state == CO_NMT_OPERATIONAL) {
            if (now - g_last_sync_ms >= g_comm_params.sync_period_ms) {
                co_sync_process();
                g_last_sync_ms = now;
            }
        }

        if (now - g_last_heartbeat_ms >= g_comm_params.heartbeat_period_ms) {
            co_nmt_heartbeat_send();
            g_last_heartbeat_ms = now;
        }

        static uint32_t last_pdo_tx;
        if (now - last_pdo_tx >= g_comm_params.pdo_event_timer_ms) {
            co_pdo_process_tx();
            last_pdo_tx = now;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    if (hcan_ptr->Instance != CAN_INSTANCE) return;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &header, data);
    canopen_update_comm_timestamp();
    canopen_rx_callback(&header, data, header.DLC);
}

void canopen_rx_callback(CAN_RxHeaderTypeDef *header, uint8_t *data, uint32_t len)
{
    uint32_t cob_id = header->StdId;
    uint8_t  node_id = g_comm_params.node_id;

    if (cob_id == 0x000) {
        if (len >= 2) co_nmt_process(data[0], data[1]);
        return;
    }
    if (cob_id == 0x080) {
        co_sync_rx_callback();
        co_pdo_process_sync();
        return;
    }
    if (cob_id == (uint32_t)(0x600 + node_id)) { co_sdo_process_request(cob_id, data, len); return; }
    if (cob_id == (uint32_t)(0x200 + node_id)) { co_pdo_process_rx(cob_id, data, len); return; }
    if (cob_id == (uint32_t)(0x300 + node_id)) { co_pdo_process_rx(cob_id, data, len); return; }
    if (cob_id == (uint32_t)(0x400 + node_id)) { co_pdo_process_rx(cob_id, data, len); return; }
    if (cob_id == (uint32_t)(0x500 + node_id)) { co_pdo_process_rx(cob_id, data, len); return; }
}

uint8_t canopen_tx_frame(uint32_t cob_id, uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;
    CAN_TxHeaderTypeDef header;
    header.StdId    = cob_id;
    header.ExtId    = 0;
    header.IDE      = CAN_ID_STD;
    header.RTR      = CAN_RTR_DATA;
    header.DLC      = len;
    header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan, &header, data, &g_can_tx_mailbox) == HAL_OK) ? 1 : 0;
}

co_nmt_state_t canopen_get_nmt_state(void)     { return g_nmt_state; }
co_comm_params_t *canopen_get_comm_params(void) { return &g_comm_params; }
void canopen_update_comm_timestamp(void)        { protection_reset_comm_timeout(); }
