/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#define CMD_OFFSET 1
#define ID_OFFSET 2
#define TIMESTAMP_OFFSET 6
#define DRONE_ID_OFFSET 6
#define POS_OFFSET 10
#define TOF_OFFSET 22

#include "spi_cmds_decode.h"

/* Packet structure:
        'S'
        CMD
        ID
        TIMESTAMP
        POS
        TOF
        CHECKSUM
        'E'
*/
spi_transfer_t spi_decode_pose(uint8_t *buffer, pcl_pose_t *pose) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != POSE_CMD) return SPI_WRONG_CMD;

    memset(pose, 0, sizeof(pcl_pose_t));
    pose->id = *((int16_t *)&buffer[ID_OFFSET]);
    pose->timestamp = *((int32_t *)&buffer[TIMESTAMP_OFFSET]);
    memcpy(pose->pos, &buffer[POS_OFFSET], 3 * sizeof(float));
    memcpy(pose->tof, &buffer[TOF_OFFSET], 4 * 8 * sizeof(int16_t));

    uint32_t checksum = 0;
    for (int16_t i = 0; i < sizeof(pcl_pose_t); i++)
        checksum += *((uint8_t *)pose + i);

    uint32_t checksum_stm32 = *((uint32_t *)&buffer[sizeof(pcl_pose_t) + 2]);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_ext_pose(uint8_t *buffer, ext_aug_pose_t *pose) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != EXT_POSE_CMD) return SPI_WRONG_CMD;

    memset(pose, 0, sizeof(ext_aug_pose_t));
    pose->id = *((int16_t *)&buffer[ID_OFFSET]);
    pose->drone_id = *((int32_t *)&buffer[DRONE_ID_OFFSET]);
    memcpy(pose->pos, &buffer[POS_OFFSET], 3 * sizeof(float));
    memcpy(pose->tof, &buffer[TOF_OFFSET], 4 * 8 * sizeof(int16_t));

    uint32_t checksum = 0;
    for (int16_t i = 0; i < sizeof(ext_aug_pose_t); i++)
        checksum += *((uint8_t *)pose + i);

    uint32_t checksum_stm32 = *((uint32_t *)&buffer[sizeof(ext_aug_pose_t) + 2]);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_ext_pose_upd(uint8_t *buffer, ext_pose_upd_t *pos_upd) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != UPD_EXT_POSE_CMD) return SPI_WRONG_CMD;

    memset(pos_upd, 0, sizeof(ext_pose_upd_t));
    pos_upd->pose_id = *((int16_t *)&buffer[ID_OFFSET]);
    pos_upd->drone_id = *((int16_t *)&buffer[ID_OFFSET + 2]);
    memcpy(pos_upd->pos, &buffer[ID_OFFSET + 4], 3 * sizeof(float));

    uint32_t checksum = 0;
    for (int16_t i = 0; i < 3 * sizeof(float); i++)
        checksum += *((uint8_t *)pos_upd->pos + i);

    uint32_t checksum_stm32 = *((uint32_t *)&buffer[18]);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_scan_info(uint8_t *buffer, int16_t *scan_info) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != SCAN_CMD) return SPI_WRONG_CMD;

    scan_info[0] = *((int16_t *)&buffer[2]);

    int32_t checksum = 0;
    for (int16_t i = 2; i < 4; i++) checksum += buffer[i];

    int32_t checksum_stm32 = *((int16_t *)&buffer[4]);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}

spi_transfer_t spi_decode_pose_req(uint8_t *buffer, int16_t *pose_request) {
    uint8_t cmd_type = buffer[CMD_OFFSET];
    if (cmd_type != POSE_REQ_CMD) return SPI_WRONG_CMD;

    pose_request[0] = *((int16_t *)&buffer[2]);
    pose_request[1] = *((int16_t *)&buffer[4]);

    int32_t checksum = 0;
    for (int16_t i = 2; i < 6; i++) checksum += buffer[i];

    int32_t checksum_stm32 = *((int16_t *)&buffer[6]);
    if (checksum != checksum_stm32) return SPI_CHECKSUM_ERR;

    return SPI_OK;
}
