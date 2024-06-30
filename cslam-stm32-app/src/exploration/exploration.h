/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef TOF_PROCESS
#define TOF_PROCESS

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define VEL_STOP 0.0f
#define VEL_FEAR -0.2f
#define VEL_CRUISE 0.4f

#define TURN_RATE 0.4f

#define DIS_HIGH 1300 // mm
#define DIS_STOP 500  // mm
#define DIS_FEAR 150  // mm

#define CCW 1
#define CW -1

#define TOF_FPS (30.0f)
#define TOF_PERIOD (1.0f / TOF_FPS)

/**
 * @brief Performs one iteration of the exploration algorithm and provides the
 * flights commands based on the distances to the other drones, their positions
 * and also the previous exploration state of the current drone.
 *
 * @param front_matrix Distance ToF amtrix of the front sensor.
 * @param lines Array containing the slopes of the lines determiend by flat
 * walls for the front/left/right sensors. If no line is detected by a sensor i,
 * then lines[i] is -10.
 * @param side_distances Three dimensional array containing left/right/back
 * distances.
 * @param fly_cmds Array where to write the computed flight commands.
 * @param dtd_dist Minimum distance from another drone.
 */
void explore(uint16_t *front_matrix, float *lines, float *side_distances,
             float back_dist, float *fly_cmds, float dtd_dist);

/**
 * @brief Checks if the drone is currently spinning.
 *
 * @return 1 means yes, 0 means no.
 */
uint8_t is_drone_spinning();

#endif /* TOF_PROCESS */