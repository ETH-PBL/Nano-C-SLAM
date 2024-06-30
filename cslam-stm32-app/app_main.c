/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "FreeRTOS.h"
#include "debug.h"
#include "estimator_kalman.h"
#include "log.h"
#include "param.h"
#include "queue.h"
#include "semphr.h"
#include "static_mem.h"
#include "task.h"

#include "deck.h"
#include "exploration.h"
#include "scan.h"
#include "tof_daq.h"
#include "tof_matrix.h"

#include "configblock.h"
#include "exploration.h"
#include "flight_commander.h"
#include "radio_comm.h"
#include "scan_manager.h"
#include "uwb_api.h"
#include "uwb_ranging.h"

SemaphoreHandle_t tofSemaphore0;
SemaphoreHandle_t tofSemaphore1;
QueueHandle_t poseQueue;

uint8_t ready = 0;
uint8_t land = 0;

float x_init = 0.0f;
float y_init = 0.0f;
float yaw_init = 0.0f;

PARAM_GROUP_START(cmds)
PARAM_ADD(PARAM_UINT8, ready, &ready)
PARAM_ADD(PARAM_UINT8, land, &land)
PARAM_GROUP_STOP(cmds)

PARAM_GROUP_START(initial)
PARAM_ADD(PARAM_FLOAT, xinit, &x_init)
PARAM_ADD(PARAM_FLOAT, yinit, &y_init)
PARAM_ADD(PARAM_FLOAT, yawinit, &yaw_init)
PARAM_GROUP_STOP(initial)

void mission_task(void *parameters);

STATIC_MEM_TASK_ALLOC(mission_task, 2000);
STATIC_MEM_TASK_ALLOC(flight_task, 1000);
STATIC_MEM_TASK_ALLOC(tof_task, 500);

extern TaskHandle_t uwbTaskHandle;

void update_poses_pc();

#define FIRST_NODE_RADIO 0xA
uint8_t NODE_ID;
static uint8_t broadcast_scan = 0;

void appMain() {
    vSemaphoreCreateBinary(tofSemaphore0);
    vSemaphoreCreateBinary(tofSemaphore1);
    xSemaphoreTake(tofSemaphore0, 0);
    xSemaphoreTake(tofSemaphore1, 0);
    poseQueue = xQueueCreate(100, sizeof(aug_pose_t));

    init_scan_queue();

    STATIC_MEM_TASK_CREATE(mission_task, mission_task, "mission", NULL, 2);
    STATIC_MEM_TASK_CREATE(tof_task, tof_task, "tof", NULL, 3);

    // Declare UWB part
    NODE_ID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) -
                        FIRST_NODE_RADIO);
    uwb_api_init(NODE_ID);
    uwb_responder_off();
    xTaskCreate(uwb_ranging_task, "uwb_task", 5 * configMINIMAL_STACK_SIZE,
                NULL, 4, &uwbTaskHandle);
    DEBUG_PRINT("DRONE ID: %d\n", NODE_ID);

    while (1) vTaskDelay(M2T(1000));
}

void show_optimized_poses() {
    int16_t pose_count = get_last_pose_id();
    for (int16_t i=0; i<pose_count; i++) {
        float poses[3];
        spi_rcv_poses(poses, i, 1);
        DEBUG_PRINT("Pose %d is: %.3f, %.3f, %.3f\n");
        vTaskDelay(M2T(2));
    }
}

void mission_task(void *parameters) {
    // Wait for ready command from client
    DEBUG_PRINT("Waiting for ready command...\n");
    while (!ready)
        vTaskDelay(M2T(100));
    while (!tof_is_init())
        vTaskDelay(M2T(10));
    DEBUG_PRINT("Start\n");
    STATIC_MEM_TASK_CREATE(flight_task, flight_task, "fly", NULL, 2);

    // Start mission
    DEBUG_PRINT("Init: %.2f %.2f\n", x_init, y_init);
    estimatorKalmanInitToPose(x_init, y_init, 0.0f, yaw_init);
    vTaskDelay(M2T(50));
    enable_commander();

    int16_t loop_cnt = 0;

    // Define structs
    aug_pose_t aug_pose = {0};

    vTaskDelay(M2T(2000));
    int16_t scan_id = -1;
    TickType_t prev_time = 0;

    TickType_t mission_start_time = xTaskGetTickCount();
    while (1) {
        if (xTaskGetTickCount() - mission_start_time > 4 * 60 * 1000) {
            show_optimized_poses();
            while (1) vTaskDelay(M2T(1));
        }

        // Get ToF data and send it to PC
        xSemaphoreTake(tofSemaphore0, portMAX_DELAY);
        if (loop_cnt++ % 2 > 0)
            continue;

        if (get_min_dtd_dist() < 0.9f && !is_drone_spinning())
            continue;

        if (get_min_dtd_dist() < 0.4f)
            continue;

        TickType_t loop_time = xTaskGetTickCount() - prev_time;
        if (loop_time > 150)
            DEBUG_PRINT("Loop time is %d\n", loop_time);
        prev_time = xTaskGetTickCount();

        get_aug_pose(tof_data_buf, &aug_pose);
        update_traveled_distance(&aug_pose);
        send_burst_data(0, &aug_pose, sizeof(aug_pose_t)); // to PC

        send_aug_pose_to_queue(aug_pose);
        add_pose_to_scan_buffer(&aug_pose);

        if (get_scan_id_from_queue(&scan_id, 1)) {
            DEBUG_PRINT("Scan added! %d\n", scan_id);
            send_burst_data(1, &scan_id, sizeof(int16_t));
            reset_traveled_distance();
            broadcast_scan = 1;
        }

        if (broadcast_scan && get_last_pose_id() - scan_id >= 19) {
            broadcast_scan = 0;
            lock_scan_acquisition();
        }
    }

    while (1)
        vTaskDelay(M2T(1000));
}

