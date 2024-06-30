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
#include "log.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <math.h>

#include "uwb_collision_det.h"

#define FREE_AREA 0
#define FRONT 1
#define BACK 2
#define RIGHT 3
#define LEFT 4

static uint8_t is_in_fwd_box(float delta_x, float delta_y, float yaw) {
    float x_rot = cosf(-yaw) * delta_x - sinf(-yaw) * delta_y;
    float y_rot = sinf(-yaw) * delta_x + cosf(-yaw) * delta_y;

    if (0.0f < x_rot && x_rot < 0.25f && fabs(y_rot) < 0.25f) {
        return 1;
    } else {
        return 0;
    }
}

void check_all_sides(float x, float y, zones_t *zones) {
    float x_cur = logGetFloat(logGetVarId("kalman", "stateX"));
    float y_cur = logGetFloat(logGetVarId("kalman", "stateY"));

    float current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));

    float delta_x = x_cur - x;
    float delta_y = y_cur - y;

    if (delta_x == 0.0f)
        delta_x = 0.0001; // avoid divid by zero

    float theta = atan(delta_y / delta_x) * 180 / M_PI;

    if ((delta_x) < 0 && (delta_y) >= 0) // 4nd quadrant
        theta += 360 - current_yaw;
    else if ((delta_x) > 0 && (delta_y) >= 0) // 3d quadrant
        theta += 180 - current_yaw;
    else if ((delta_x) > 0 && (delta_y) <= 0) // 2nd quadrant
        theta += 180 - current_yaw;
    else // 1st quadrant
        theta += -current_yaw;

    theta = (int)(theta + 360) % 360;

    if (is_in_fwd_box(-delta_x, -delta_y, current_yaw * M_PI / 180.0f)) {
        zones->front++;
        return;
    }

    if (theta <= 45 || theta >= 315) {
        zones->front++;
        return;
    } else if (theta > 45 && theta <= 135) {
        zones->left++;
        return;
    } else if (theta > 135 && theta <= 225) {
        zones->back++;
        return;
    } else if (theta > 225 && theta < 315) {
        zones->right++;
        return;
    }

    return; // error code
}

int8_t get_side_lr(float x, float y) {
    float x_cur = logGetFloat(logGetVarId("kalman", "stateX"));
    float y_cur = logGetFloat(logGetVarId("kalman", "stateY"));

    float current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));

    float delta_x = x_cur - x;
    float delta_y = y_cur - y;

    if (delta_x == 0.0f)
        delta_x = 0.0001; // avoid divid by zero

    float theta = atan(delta_y / delta_x) * 180 / M_PI;

    if ((delta_x) < 0 && (delta_y) >= 0) // 4nd quadrant
        theta += 360 - current_yaw;
    else if ((delta_x) > 0 && (delta_y) >= 0) // 3d quadrant
        theta += 180 - current_yaw;
    else if ((delta_x) > 0 && (delta_y) <= 0) // 2nd quadrant
        theta += 180 - current_yaw;
    else // 1st quadrant
        theta += -current_yaw /*+0*/;

    theta = (int)(theta + 360) % 360;

    // determine the side
    if (theta >= 0 && theta <= 180) {
        return 0; // obstacle on the left
    } else
        return 1; // obstacle on the right
}
