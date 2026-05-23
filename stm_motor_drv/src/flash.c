/**
 * @file    flash.c
 * @brief   Flash 参数存储 — F1 页擦除 / F4 扇区擦除
 */

#include "flash.h"
#include "uart_cli.h"
#include <string.h>

static flash_param_slot_t g_flash_slot;
static uint32_t g_active_slot_addr;

static uint16_t crc16_calc(const uint8_t *data, uint32_t len)
{
    static const uint16_t table[256] = {
        0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
        0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
        0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
        0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
        0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
        0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
        0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
        0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
        0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
        0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
        0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
        0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
        0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
        0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
        0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
        0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
        0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
        0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
        0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
        0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
        0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
        0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
        0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
        0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
        0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
        0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
        0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
        0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
        0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
        0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
        0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
        0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0,
    };
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = (crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFF];
    return crc;
}

void flash_init(void)
{
    memset(&g_flash_slot, 0, sizeof(flash_param_slot_t));
#if STM32_PLATFORM == STM32_PLATFORM_F1
    g_active_slot_addr = FLASH_PARAM_ADDR_SLOT0;
#else
    g_active_slot_addr = FLASH_PARAM_START_ADDR;
#endif
}

uint8_t flash_params_load(void)
{
    uint32_t slot_addrs[2];
#if STM32_PLATFORM == STM32_PLATFORM_F1
    slot_addrs[0] = FLASH_PARAM_ADDR_SLOT0;
    slot_addrs[1] = FLASH_PARAM_ADDR_SLOT1;
#else
    slot_addrs[0] = FLASH_PARAM_START_ADDR;
    slot_addrs[1] = FLASH_PARAM_START_ADDR + FLASH_SLOT_SIZE;
#endif

    uint32_t best_addr = 0, best_wr = 0;
    uint8_t found = 0;

    for (int i = 0; i < 2; i++) {
        flash_param_slot_t *slot = (flash_param_slot_t *)slot_addrs[i];
        if (slot->header.magic != 0x4D4F544F) continue;

        uint16_t crc = crc16_calc((uint8_t *)slot + sizeof(flash_header_t),
                                  sizeof(flash_param_slot_t) - sizeof(flash_header_t));
        if (crc != slot->header.crc16) continue;

        if (slot->header.write_count >= best_wr) {
            best_wr = slot->header.write_count;
            best_addr = slot_addrs[i];
            found = 1;
        }
    }

    if (found) {
        memcpy(&g_flash_slot, (void *)best_addr, sizeof(flash_param_slot_t));
        g_active_slot_addr = best_addr;
        memcpy(&g_motor.motor_params,    &g_flash_slot.motor, sizeof(motor_params_t));
        memcpy(&g_motor.pid_params,      &g_flash_slot.pid,   sizeof(motor_pid_params_t));
        memcpy(&g_motor.prot_thresholds, &g_flash_slot.prot,  sizeof(protection_thresholds_t));
        g_motor.zero_offset_pulses = g_flash_slot.zero_offset;
        LOG_INFO("FLASH", "Loaded slot%d, count=%lu", (best_addr == slot_addrs[0]) ? 0 : 1, best_wr);
    } else {
        flash_restore_factory();
        LOG_WARN("FLASH", "No valid params, using defaults");
    }
    return found;
}

uint8_t flash_params_save(void)
{
    uint32_t target_addr;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    if (g_active_slot_addr == FLASH_PARAM_ADDR_SLOT0)
        target_addr = FLASH_PARAM_ADDR_SLOT1;
    else
        target_addr = FLASH_PARAM_ADDR_SLOT0;
#else
    if (g_active_slot_addr == FLASH_PARAM_START_ADDR)
        target_addr = FLASH_PARAM_START_ADDR + FLASH_SLOT_SIZE;
    else
        target_addr = FLASH_PARAM_START_ADDR;
#endif

    memcpy(&g_flash_slot.motor, &g_motor.motor_params, sizeof(motor_params_t));
    memcpy(&g_flash_slot.pid,   &g_motor.pid_params,   sizeof(motor_pid_params_t));
    memcpy(&g_flash_slot.prot,  &g_motor.prot_thresholds, sizeof(protection_thresholds_t));
    g_flash_slot.zero_offset   = g_motor.zero_offset_pulses;
    g_flash_slot.can_node_id    = 1;
    g_flash_slot.can_baudrate   = CAN_BAUDRATE;

    g_flash_slot.header.magic       = 0x4D4F544F;
    g_flash_slot.header.version     = 0x0101;
    g_flash_slot.header.write_count++;
#if STM32_PLATFORM == STM32_PLATFORM_F1
    g_flash_slot.header.slot_index  =
        (target_addr == FLASH_PARAM_ADDR_SLOT0) ? 0 : 1;
#else
    g_flash_slot.header.slot_index  =
        (target_addr == FLASH_PARAM_START_ADDR) ? 0 : 1;
#endif
    g_flash_slot.header.crc16 = crc16_calc(
        (uint8_t *)&g_flash_slot + sizeof(flash_header_t),
        sizeof(flash_param_slot_t) - sizeof(flash_header_t));

    HAL_FLASH_Unlock();

#if STM32_PLATFORM == STM32_PLATFORM_F1
    /* F1: 页擦除 */
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_PAGES;
    erase.PageAddress  = target_addr;
    erase.NbPages      = 1;

    uint32_t page_error;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    /* 按 16bit 半字写入 (F1 Flash 为 16bit 位宽) */
    uint16_t *src = (uint16_t *)&g_flash_slot;
    for (uint32_t i = 0; i < sizeof(flash_param_slot_t) / 2; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              target_addr + i * 2, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
    }
#else
    /* F4: 扇区擦除 + 字写入 */
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = FLASH_PARAM_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    uint32_t *src = (uint32_t *)&g_flash_slot;
    for (uint32_t i = 0; i < sizeof(flash_param_slot_t) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              target_addr + i * 4, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
    }
#endif

    HAL_FLASH_Lock();
    g_active_slot_addr = target_addr;
    LOG_INFO("FLASH", "Saved slot%d, count=%lu",
             (target_addr == FLASH_PARAM_ADDR_SLOT0) ? 0 : 1,
             g_flash_slot.header.write_count);
    return 1;
}

uint8_t flash_save_zero_offset(int32_t offset)
{
    g_flash_slot.zero_offset = offset;
    return flash_params_save();
}

void flash_restore_factory(void)
{
    motor_params_t def_motor = {
        1000, 24.0f, 3.0f, 6.0f, 3000, 0.5f, 0.167f, 8.3f, 4, 0.8f, 1.2f
    };
    motor_pid_params_t def_pid = {
        { 0.5f, 0.02f, 0.0f,  500.0f,  1000.0f },
        { 1.2f, 0.08f, 0.01f, 3000.0f, 6000.0f },
        { 8.0f, 0.0f,  0.5f,  10000.0f,3000.0f },
    };
    protection_thresholds_t def_prot = {
        28.8f, 16.8f, 6.0f, 0.75f, 3.0f, 2.0f, 0.5f, 85.0f, 120.0f, 500
    };

    memcpy(&g_motor.motor_params,    &def_motor, sizeof(motor_params_t));
    memcpy(&g_motor.pid_params,      &def_pid,   sizeof(motor_pid_params_t));
    memcpy(&g_motor.prot_thresholds, &def_prot,  sizeof(protection_thresholds_t));
    g_motor.zero_offset_pulses = 0;
    LOG_INFO("FLASH", "Factory defaults loaded");
}

flash_param_slot_t *flash_get_slot(void) { return &g_flash_slot; }
