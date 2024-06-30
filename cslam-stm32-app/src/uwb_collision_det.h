/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef UWB_COL_DET_H
#define UWB_COL_DET_H

typedef struct {
    uint8_t front;
    uint8_t back;
    uint8_t left;
    uint8_t right;
} zones_t;

/**
 * @brief Determine if a robot located at (x,y) is in one of the four zones
 * and increment the counter of that zone (front, back, left, right).
 *
 * @param x x-coordinate of the robot.
 * @param y y-coordinate of the robot.
 * @param zones Pointer to a zones structure.
 * @return 1 - point is to the right, 0 to the left.
 */
void check_all_sides(float x, float y, zones_t *zones);

/**
 * @brief Determine if a particular 2D point is towards the right or left of the
 * current drone.
 *
 * @param x Coordinate of the point.
 * @param y Coordinate of the point.
 * @return 1 - point is to the right, 0 to the left.
 */
int8_t get_side_lr(float x, float y);

#endif