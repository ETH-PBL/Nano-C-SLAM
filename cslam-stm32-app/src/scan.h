/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SCAN_H
#define SCAN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

typedef struct {
    int16_t num;
    float x[32];
    float y[32];
} scan_frame_t;

typedef struct {
    int16_t id;
    int32_t timestamp;
    float pos[3];
    int16_t tof[4][8];
} aug_pose_t;

typedef struct {
    int16_t id;
    int32_t drone_id;
    float pos[3];
    int16_t tof[4][8];
} ext_aug_pose_t;

#define SCAN_SIZE 15

int compare_int16(const void *a, const void *b);

/**
 * @brief Fetches the 2D position, heading, and the ToF measurements and
 *        puts it in an augmented pose structure.
 * @param tof_data_buf Array containing the distance matrices from four
 *        ToF sensors.
 * @param complete_pose Pointer to the output augmented pose.
 */
uint8_t get_aug_pose(int16_t tof_data_buf[4][8][8], aug_pose_t *complete_pose);

/**
 * @brief Reduces a ToF matrix to a row vector by selecting the median pixel
 *        from each column.
 * @param matrix Array containing the distance matrix.
 * @param measurements Output array.
 */
void extract_measurements(const int16_t matrix[8][8], int16_t measurements[8]);

/**
 * @brief Keeps track of the augmented pose count.
 * @return Number of the acquired augmented poses.
 */
int16_t get_last_pose_id();

#endif // SCAN_H
