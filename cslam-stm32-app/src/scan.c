/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "scan.h"

#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "task.h"

#include "tof_matrix.h"

#define DEBUG_MODULE __FILE__

#include "debug.h"

static int16_t pose_cnt = 0;

int compare_int16(const void *a, const void *b) {
    return (*(int16_t *)a - *(int16_t *)b);
}

uint8_t get_aug_pose(int16_t tof_data_buf[4][8][8], aug_pose_t *complete_pose) {
    memset(complete_pose, 0, sizeof(aug_pose_t));
    // Read measurements from each sensor
    for (int16_t dir = 0; dir < 4; dir++) {
        extract_measurements(tof_data_buf[dir], complete_pose->tof[dir]);
    }

    complete_pose->pos[0] = logGetFloat(logGetVarId("stateEstimate", "x"));
    complete_pose->pos[1] = logGetFloat(logGetVarId("stateEstimate", "y"));
    complete_pose->pos[2] =
        logGetFloat(logGetVarId("stateEstimate", "yaw")) / 180.0f * (float)M_PI;
    complete_pose->timestamp = (int32_t)xTaskGetTickCount();
    complete_pose->id = pose_cnt;
    pose_cnt++;
    return 1;
}

void extract_measurements(const int16_t matrix[8][8], int16_t measurements[8]) {

    for (int col = 0; col < 8; col++) {

        // Extract center four pixels of each column discarding invalid pixels
        int16_t rows[4];
        uint8_t num_rows = 0;
        for (int i = 2; i < 6; i++) {
            if (matrix[i][col] >= 0) {
                rows[num_rows++] = matrix[i][col];
            }
        }
        if (num_rows == 0) {
            measurements[col] = -1;
            continue;
        }

        // Find median
        qsort(rows, num_rows, sizeof(int16_t), compare_int16);
        if (num_rows & 1) {
            measurements[col] = rows[num_rows >> 1];
        } else {
            measurements[col] =
                (rows[num_rows >> 1] + rows[(num_rows >> 1) - 1]) >> 1;
        }
    }
}

int16_t get_last_pose_id() { return pose_cnt - 1; }