/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "exploration.h"
#include "FreeRTOS.h"
#include "debug.h"
#include "log.h"
#include "math.h"
#include "scan.h"
#include "scan_manager.h"
#include "task.h"
#include "uwb_ranging.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define SIDE_TH 0.5f
#define SIDE_VEL 0.1f
#define INTER_DRONE_DISTANCE 1.0f
#define ALIGN_SPIN_RATE 0.1f

typedef enum {
    CRUISE_STATE,
    SPINNING_STATE,
    WAIT_TO_STOP_STATE,
    CAUTION_STATE,
} state_t;

static state_t state = CRUISE_STATE;
static int8_t spin_direction = 1;
static float vel_scale = 1.0f;
static uint32_t spinning_start_time = 0;
static int32_t scan_pose_id;
static uint8_t scan_token = 1;
static uint32_t time_to_spin = 0;
static uint32_t caution_time = 0;
static int8_t danger_lock = 1;

uint8_t is_angle_valid(float angle) {
    if (angle > -1.6f && angle < 1.6f) {
        return 1;
    } else {
        return 0;
    }
}

// intors spre dreapta -> unghiul creste negativ
int8_t get_spinning_direction(float angle, float *sides, int8_t front_dir) {
    float left_side = sides[0];
    float right_side = sides[1];
    float side_th = 1.2f;

    // if approaching the wall almost perpendicularly
    if (!is_angle_valid(angle) || (angle < 0.6f && angle > -0.6f)) {
        if (left_side < side_th && right_side > side_th) {
            return CW;
        } else if (left_side > side_th && right_side < side_th) {
            return CCW;
        } // Tunnel
        else if (left_side < side_th && right_side < side_th) {
            return front_dir;
        } else {
            return choose_left_right();
        }
    } // if not approaching perpendicularly
    else if (angle > 0.0f && angle <= 1.0f) {
        return CCW;
    } else {
        return CW;
    }
}

void align_sides(float *angles, float *sides, float *side_cmds) {
    float angle_left = angles[1];
    float angle_right = angles[2];
    float dist_left = sides[0];
    float dist_right = sides[1];

    side_cmds[0] = side_cmds[1] = 0.0f;

    if (dist_right < 1.0f && is_angle_valid(angle_right)) {
        if (angle_right > 0.0f)
            side_cmds[1] = -ALIGN_SPIN_RATE;
        else
            side_cmds[1] = ALIGN_SPIN_RATE;
    }

    if (dist_left < 1.0f && is_angle_valid(angle_left)) {
        if (angle_left > 0.0f)
            side_cmds[1] = -ALIGN_SPIN_RATE;
        else
            side_cmds[1] = ALIGN_SPIN_RATE;
    }

    if (dist_right < SIDE_TH)
        side_cmds[0] = SIDE_VEL;

    if (dist_left < SIDE_TH)
        side_cmds[0] = -SIDE_VEL;

    if (dist_right < SIDE_TH && dist_left < SIDE_TH) {
        if (dist_right > dist_left) {
            side_cmds[0] = -SIDE_VEL;
        } else {
            side_cmds[0] = SIDE_VEL;
        }
    }
}

uint16_t get_front_distance(uint16_t *tof_front) {
    uint16_t min_dist = UINT16_MAX;
    for (int16_t i = 2; i < 6; i++)
        for (int16_t j = 1; j < 7; j++)
            if (*(tof_front + 8 * i + j) < min_dist)
                min_dist = *(tof_front + 8 * i + j);

    return min_dist;
}

int8_t get_front_direction(uint16_t *tof_front) {
    uint32_t left = tof_front[0] + tof_front[1] + tof_front[2] + tof_front[3];
    uint32_t right = tof_front[4] + tof_front[5] + tof_front[6] + tof_front[7];

    if (left > right)
        return CCW;
    else
        return CW;
}

void danger_check(float dist_front, float dist_back, float dist_left,
                  float dist_right, float *vx, float *vy) {
    float fear_velocity = 0.2f;
    float threshold = 0.3f;

    int8_t zones[4] = {0, 0, 0, 0};
    if (dist_front < threshold)
        zones[0] = 1;
    if (dist_back < threshold)
        zones[1] = 1;
    if (dist_left < threshold)
        zones[2] = 1;
    if (dist_right < threshold)
        zones[3] = 1;

    *vx = *vy = 0.0f;
    if (zones[0] + zones[1] + zones[2] + zones[3] == 0) {
        return;
    }

    *vx = (float)(zones[1] - zones[0]) * fear_velocity;
    *vy = (float)(zones[3] - zones[2]) * fear_velocity;
}

uint8_t is_scan_valid(uint32_t start_time, float dtd_dist) {
    uint32_t time_now = xTaskGetTickCount();

    if (time_now - start_time < 2000) {
        return 0;
    }

    if (!traveled_more_than(1.0f)) {
        return 0;
    }

    if (dtd_dist < 1.0f || scan_token != 1) {
        return 0;
    }

    return 1;
}

void explore(uint16_t *front_matrix, float *lines, float *side_distances,
             float back_dist, float *fly_cmds, float dtd_dist) {
    uint16_t distance;
    distance = get_front_distance(front_matrix);

    float cmd_vel_x = 0.0f;
    float cmd_vel_y = 0.0f;
    float cmd_turn = 0.0f;

    switch (state) {
    case CRUISE_STATE:
        if (distance < DIS_STOP) {
            state = WAIT_TO_STOP_STATE;
            int8_t front_dir =
                get_front_direction(front_matrix + 24); // middle row
            spin_direction =
                get_spinning_direction(lines[0], side_distances, front_dir);
            // DEBUG_PRINT("-> WAIT TO STOP\n");
        }
        break;

    case SPINNING_STATE: // spinning state
        if (distance > DIS_STOP + 300) {
            state = CRUISE_STATE;
            // DEBUG_PRINT("-> CRUISE\n");
        } else {
            if (is_scan_valid(spinning_start_time, dtd_dist)) {
                send_scan_id_to_queue(scan_pose_id);
                scan_token = 0;
            }
        }
        break;

    case CAUTION_STATE:
        if (xTaskGetTickCount() - caution_time > time_to_spin) {
            state = CRUISE_STATE;
        }
        break;

    case WAIT_TO_STOP_STATE:;
        float vx = logGetFloat(logGetVarId("stateEstimate", "vx"));
        float vy = logGetFloat(logGetVarId("stateEstimate", "vy"));
        if (fabs(vx) < 0.04f && fabs(vy) < 0.04f) {
            spinning_start_time = xTaskGetTickCount();
            scan_pose_id = get_last_pose_id();
            state = SPINNING_STATE;

            if (side_distances[0] > 1.0f && side_distances[1] > 1.0f) {
                scan_token = 0;
            } else {
                scan_token = 1;
            }
            // DEBUG_PRINT("-> SPIN\n");
        }
        break;
    }

    // Manage inter-drone small distance
    if (dtd_dist < INTER_DRONE_DISTANCE && state != CAUTION_STATE &&
        state != SPINNING_STATE) {
        scan_token = 0;
        zones_t zones = {0};
        check_for_neigh_drones(&zones, INTER_DRONE_DISTANCE);
        if (danger_lock && zones.front > 0) {
            danger_lock = 0;
            time_to_spin = 5000;
            caution_time = xTaskGetTickCount();
            state = CAUTION_STATE;
        }
    }

    if (dtd_dist < INTER_DRONE_DISTANCE) {
        vel_scale = 0.5f;
    } else if (dtd_dist > INTER_DRONE_DISTANCE + 0.3f) {
        vel_scale = 1.0f;
        danger_lock = 1;
    }

    switch (state) {
    case CRUISE_STATE:
        // v = v_cruise * (d - d_stop) / (d_H - d_stop)
        cmd_vel_x =
            VEL_CRUISE * (distance - DIS_STOP + 100) / (DIS_HIGH - DIS_STOP);
        cmd_vel_x *= vel_scale;

        if (cmd_vel_x > VEL_CRUISE)
            cmd_vel_x = VEL_CRUISE;
        if (cmd_vel_x < 0.2f)
            cmd_vel_x = 0.2f;

        float side_cmds[2];
        align_sides(lines, side_distances, side_cmds);
        cmd_turn = side_cmds[1];
        cmd_vel_y = side_cmds[0];

        break;

    case WAIT_TO_STOP_STATE:
        cmd_vel_x = cmd_vel_y = cmd_turn = 0;
        break;

    case SPINNING_STATE:
        cmd_turn = TURN_RATE * (float)spin_direction;
        break;

    case CAUTION_STATE:
        cmd_turn = 1.5f * TURN_RATE * (float)spin_direction;
        danger_check((float)distance / 1000.0f, back_dist, side_distances[0],
                     side_distances[1], &cmd_vel_x, &cmd_vel_y);

        break;
    }

    fly_cmds[0] = cmd_vel_x;
    fly_cmds[1] = cmd_vel_y;
    fly_cmds[2] = cmd_turn;
}

uint8_t is_drone_spinning() {
    if (state == SPINNING_STATE)
        return 1;
    else
        return 0;
}
