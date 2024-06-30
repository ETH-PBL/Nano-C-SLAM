/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "math.h"
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"

#include "scan_manager.h"
#include "spi_comm.h"

static float traveled_distance = 0.0f;
static float x_prev = 0.0f;
static float y_prev = 0.0f;
static QueueHandle_t newScanQueue;

// Keep track of the last 20 poses
typedef struct aug_pose_history_t {
    uint16_t head;
    aug_pose_t poses[20];
} aug_pose_history_t;

typedef struct gap9_ext_aug_pose_buf_t {
    uint16_t count;
    ext_aug_pose_t poses[100];
} gap9_ext_aug_pose_buf_t;

typedef struct gap9_pose_buf_t {
    uint16_t count;
    float poses[200][3];
    int16_t ids[200];
    int16_t drone_ids[200];
} gap9_pose_buf_t;

gap9_ext_aug_pose_buf_t gap9_ext_aug_pose_buf = {0};
gap9_pose_buf_t gap9_pose_buf = {0};

static aug_pose_history_t pose_history = {0};
static uint8_t locked = 0;

void update_traveled_distance(aug_pose_t *aug_pose) {
    float x_now = aug_pose->pos[0];
    float y_now = aug_pose->pos[1];

    traveled_distance += sqrtf((x_now - x_prev) * (x_now - x_prev) +
                               (y_now - y_prev) * (y_now - y_prev));

    x_prev = x_now;
    y_prev = y_now;
}

void reset_traveled_distance() { traveled_distance = 0.0f; }

uint8_t traveled_more_than(float value) {
    if (traveled_distance > value) {
        return 1;
    } else {
        return 0;
    }
}

void init_scan_queue() { newScanQueue = xQueueCreate(1, sizeof(int16_t)); }

uint8_t get_scan_id_from_queue(int16_t *scan_id, TickType_t timeout_ms) {
    if (newScanQueue != NULL) {
        if (xQueueReceive(newScanQueue, scan_id, (TickType_t)timeout_ms) ==
            pdTRUE) {
            return 1;
        }
        return 0;
    }
    return 0;
}

uint8_t send_scan_id_to_queue(int16_t scan_id) {
    UBaseType_t available_space;
    if (newScanQueue != NULL) {
        available_space = uxQueueSpacesAvailable(newScanQueue);
        if (available_space == 0)
            xQueueReset(newScanQueue);

        if (xQueueSendToBack(newScanQueue, (void *)&scan_id, (TickType_t)0) ==
            pdTRUE) {
            return 1;
        }
    }
    return 0;
}

void lock_scan_acquisition() { locked = 1; }

void unlock_scan_acquisition() { locked = 0; }

uint8_t is_new_scan_available() { return locked; }

void add_pose_to_scan_buffer(aug_pose_t *pose) {
    if (locked)
        return;

    memcpy(&pose_history.poses[pose_history.head], pose, sizeof(aug_pose_t));
    pose_history.head = (pose_history.head + 1) % 20;
}

aug_pose_t *get_pose_from_scan_buffer(uint8_t idx) {
    uint8_t buffer_idx = (pose_history.head + idx) % 20;
    aug_pose_t *pose = &pose_history.poses[buffer_idx];
    return pose;
}

void add_ext_aug_pose_to_gap9_buffer(ext_aug_pose_t *ext_aug_pose0) {
    memcpy(&gap9_ext_aug_pose_buf.poses[gap9_ext_aug_pose_buf.count++],
           ext_aug_pose0, sizeof(ext_aug_pose_t));
}

void add_pose_upd_to_gap9_buffer(float *pose, int16_t id, int16_t drone_id) {
    gap9_pose_buf.drone_ids[gap9_pose_buf.count] = drone_id;
    gap9_pose_buf.ids[gap9_pose_buf.count] = id;
    memcpy(&gap9_pose_buf.poses[gap9_pose_buf.count++][0], pose,
           3 * sizeof(float));
}

void send_ext_aug_poses_to_gap9() {
    // Send scans
    for (int16_t i = 0; i < gap9_ext_aug_pose_buf.count; i++) {
        // DEBUG_PRINT("GAP: Sent ext pose %d\n",
        //             gap9_ext_aug_pose_buf.poses[i].id);
        spi_send_ext_aug_pose(&gap9_ext_aug_pose_buf.poses[i]);
        vTaskDelay(M2T(2));
    }
    gap9_ext_aug_pose_buf.count = 0;

    // Send updated poses
    for (int16_t i = 0; i < gap9_pose_buf.count; i++) {
        DEBUG_PRINT("GAP: Sent pose upd %d\n", gap9_pose_buf.ids[i]);
        spi_send_ext_pose_upd(&gap9_pose_buf.poses[i][0], gap9_pose_buf.ids[i],
                              gap9_pose_buf.drone_ids[i]);
        vTaskDelay(M2T(2));
    }
    gap9_pose_buf.count = 0;
}