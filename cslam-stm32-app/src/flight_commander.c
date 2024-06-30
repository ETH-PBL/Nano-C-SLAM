/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "commander.h"
#include "debug.h"
#include "estimator_kalman.h"
#include "log.h"
#include "task.h"

#include "config_params.h"
#include "crtp_commander_high_level.h"
#include "exploration.h"
#include "limit_actuation.h"
#include "line_detector.h"
#include "tof_daq.h"
#include "uwb_ranging.h"

#define RAD_TO_DEG 57.3f
static uint8_t commander_en = 0;
extern uint8_t land;

static setpoint_t setpoint = {
    .mode =
        {
            .x = modeVelocity,
            .y = modeVelocity,
            .z = modeAbs,
            .yaw = modeVelocity,
        },
    .position =
        {
            .x = 0,
            .y = 0,
            .z = EXP_HEIGHT,
        },
    .velocity_body = true,
    .attitudeRate =
        {
            .yaw = 0,
        },
};
void update_flight_commands(int16_t tof_data_buf[4][8][8], float *fly_cmds);

void flight_task(void *parameters) {
    while (!commander_en)
        vTaskDelay(M2T(100));
    // crtpCommanderHighLevelTakeoffWithVelocity(0.6f, 0.3f, false);
    DEBUG_PRINT("Take-off: %.2f  %.2f  %.2f\n",
                logGetFloat(logGetVarId("kalman", "stateX")),
                logGetFloat(logGetVarId("kalman", "stateY")),
                logGetFloat(logGetVarId("stateEstimate", "yaw")));
    vTaskDelay(M2T(2500));
    while (1) {
        float fly_cmds[3];
        update_flight_commands(tof_data_buf, fly_cmds);
        setpoint.velocity.x = fly_cmds[0];
        setpoint.velocity.y = fly_cmds[1];
        setpoint.attitudeRate.yaw = fly_cmds[2];
        // commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
        vTaskDelay(M2T(20));
        if (land) {
            crtpCommanderHighLevelLand(0.0f, 2);
            vTaskDelay(M2T(2000));
            break;
        }
    }

    while (1) vTaskDelay(M2T(10));
}

void process_line(int16_t *line_in, float *line_out) {
    for (uint8_t i = 0; i < 8; i++) {
        if (line_in[i] > 0) {
            line_out[i] = (float)line_in[i] / 1000.0f;
        } else {
            line_out[i] = -1.0f;
        }
    }
}

float distance_mm_to_m(int16_t distance_mm) {
    float distance_m;
    if (distance_mm > 0) {
        distance_m = (float)distance_mm / 1000.0f;
    } else {
        distance_m = 10.0f;
    }
    return distance_m;
}

void update_flight_commands(int16_t tof_data_buf[4][8][8], float *fly_cmds) {
    uint16_t tof_data_front[8 * 8];

    // Process front matrix
    for (uint16_t i = 0; i < 8; i++) {
        for (uint16_t j = 0; j < 8; j++) {
            if (tof_data_buf[0][i][j] < 0)
                tof_data_front[i * 8 + j] = UINT16_MAX;
            else
                tof_data_front[i * 8 + j] = (uint16_t)(tof_data_buf[0][i][j]);
        }
    }

    // Get minimum side distance
    float dist_left = distance_mm_to_m(tof_data_buf[2][3][4]);
    float dist_right = distance_mm_to_m(tof_data_buf[3][3][4]);
    float dist_back = distance_mm_to_m(tof_data_buf[1][3][4]);

    float front_line[8], left_line[8], right_line[8];
    process_line(tof_data_buf[0][3], front_line);
    process_line(tof_data_buf[2][3], left_line);
    process_line(tof_data_buf[3][3], right_line);

    // Get line slopes
    float lines[3];
    lines[0] = line_detector(front_line, 8);
    lines[1] = line_detector(left_line, 8);
    lines[2] = line_detector(right_line, 8);

    // Get side distances
    float sides[2];
    sides[0] = dist_left;
    sides[1] = dist_right;

    float dtd_distance;
    dtd_distance = get_min_dtd_dist();

    explore(tof_data_front, lines, sides, dist_back, fly_cmds, dtd_distance);

    // Limit actuation
    fly_cmds[0] = limit_velocity(fly_cmds[0]);
    fly_cmds[1] = fly_cmds[1];
    fly_cmds[2] = fly_cmds[2] * RAD_TO_DEG;
}

void enable_commander() { commander_en = 1; }