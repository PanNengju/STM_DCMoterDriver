/**
 * @file    co_sdo.c
 * @brief   CANopen SDO 实现
 *          加速传输 (≤4字节) + 分段传输 (>4字节)
 */

#include "co_sdo.h"
#include "co_objdict.h"
#include "uart_cli.h"
#include <string.h>

/*===========================================================================
 * SDO 初始化
 *===========================================================================*/
void co_sdo_init(void)
{
    /* SDO 无状态，直接使用对象字典 */
}

/*===========================================================================
 * SDO 请求处理
 *
 * SDO 加速传输帧格式 (8字节):
 *   Byte0: 命令码
 *      下载: 0x22 (写2B) / 0x23 (写4B) / 0x2B (写1B)
 *      上传: 0x40 (读请求)
 *   Byte1-2: 对象索引 (小端)
 *   Byte3: 子索引
 *   Byte4-7: 数据 (小端)
 *===========================================================================*/
void co_sdo_process_request(uint32_t cob_id, const uint8_t *data, uint8_t len)
{
    if (len < 8) return;

    uint8_t  cmd       = data[0];
    uint16_t index     = data[1] | ((uint16_t)data[2] << 8);
    uint8_t  sub_index = data[3];
    uint32_t sdo_data  = data[4] | ((uint32_t)data[5] << 8)
                       | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

    co_comm_params_t *params = canopen_get_comm_params();
    uint32_t resp_cob_id = 0x580 + params->node_id;
    uint8_t  resp[8];

    /* 仅预操作/操作状态下处理 SDO */
    co_nmt_state_t nmt = canopen_get_nmt_state();
    if (nmt != CO_NMT_PRE_OPERATIONAL && nmt != CO_NMT_OPERATIONAL) {
        return;
    }

    /* 判断: 下载 (写) 还是上传 (读) */
    uint8_t is_upload = (cmd == 0x40);

    if (is_upload) {
        /* --- SDO 上传 (读) --- */
        uint8_t read_data[4] = {0};
        uint32_t read_size = 4;

        if (co_objdict_read(index, sub_index, read_data, &read_size)) {
            resp[0] = 0x43; /* Upload Response, 4 bytes */
            if (read_size == 1) resp[0] = 0x4F;
            else if (read_size == 2) resp[0] = 0x4B;

            resp[1] = data[1]; resp[2] = data[2]; /* Index */
            resp[3] = sub_index;
            resp[4] = read_data[0]; resp[5] = read_data[1];
            resp[6] = read_data[2]; resp[7] = read_data[3];

            canopen_tx_frame(resp_cob_id, resp, 8);

            LOG_DEBUG("SDO", "Read 0x%04X/%d = 0x%08lX", index, sub_index,
                      *(uint32_t *)read_data);
        } else {
            /* 对象不存在 */
            resp[0] = 0x80;
            resp[1] = data[1]; resp[2] = data[2];
            resp[3] = sub_index;
            uint32_t abort = SDO_ABORT_OBJECT_NOT_EXIST;
            memcpy(&resp[4], &abort, 4);
            canopen_tx_frame(resp_cob_id, resp, 8);

            LOG_WARN("SDO", "Read failed: 0x%04X/%d not exist", index, sub_index);
        }
    } else {
        /* --- SDO 下载 (写) --- */
        uint8_t write_data[4];
        uint32_t write_size = 4;

        /* 解析写入大小 */
        switch (cmd) {
        case 0x2F: write_size = 1; break;  /* 1 byte */
        case 0x2B: write_size = 2; break;  /* 2 bytes */
        case 0x23: write_size = 4; break;  /* 4 bytes */
        case 0x22: write_size = 2; break;
        default:  write_size = 4; break;
        }

        memcpy(write_data, &sdo_data, write_size);

        if (co_objdict_write(index, sub_index, write_data, write_size)) {
            resp[0] = 0x60; /* Download Response */
            resp[1] = data[1]; resp[2] = data[2];
            resp[3] = sub_index;
            memset(&resp[4], 0, 4);
            canopen_tx_frame(resp_cob_id, resp, 8);

            LOG_DEBUG("SDO", "Write 0x%04X/%d = 0x%08lX", index, sub_index, sdo_data);
        } else {
            /* 写失败 */
            resp[0] = 0x80;
            resp[1] = data[1]; resp[2] = data[2];
            resp[3] = sub_index;
            uint32_t abort = SDO_ABORT_READ_ONLY;
            memcpy(&resp[4], &abort, 4);
            canopen_tx_frame(resp_cob_id, resp, 8);

            LOG_WARN("SDO", "Write failed: 0x%04X/%d", index, sub_index);
        }
    }
}
