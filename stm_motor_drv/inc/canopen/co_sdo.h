/**
 * @file    co_sdo.h
 * @brief   CANopen SDO (服务数据对象) 模块
 *          加速传输 (≤4字节) + 分段传输 (>4字节)
 */

#ifndef __CO_SDO_H
#define __CO_SDO_H

#include "canopen.h"

/* SDO 命令码 */
#define SDO_CCS_DOWNLOAD_INITIATE      0x20
#define SDO_CCS_DOWNLOAD_SEGMENT       0x00
#define SDO_CCS_UPLOAD_INITIATE        0x40
#define SDO_CCS_UPLOAD_SEGMENT         0x60
#define SDO_CCS_ABORT                  0x80

#define SDO_SCS_DOWNLOAD_RESPONSE      0x60
#define SDO_SCS_DOWNLOAD_SEGMENT_RSP   0x20
#define SDO_SCS_UPLOAD_RESPONSE        0x40
#define SDO_SCS_UPLOAD_SEGMENT_RSP     0x00
#define SDO_SCS_ABORT                  0x80

/* SDO 中止码 */
#define SDO_ABORT_OBJECT_NOT_EXIST     0x06020000
#define SDO_ABORT_READ_ONLY            0x06010001
#define SDO_ABORT_WRITE_ONLY           0x06010002
#define SDO_ABORT_TYPE_MISMATCH        0x06070010
#define SDO_ABORT_VALUE_RANGE          0x06090030
#define SDO_ABORT_GENERAL_ERROR        0x08000000

/* SDO 初始化 */
void co_sdo_init(void);

/* SDO 请求处理 */
void co_sdo_process_request(uint32_t cob_id, const uint8_t *data, uint8_t len);

#endif /* __CO_SDO_H */
