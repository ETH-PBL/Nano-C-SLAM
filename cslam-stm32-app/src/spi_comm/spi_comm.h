/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SPI_COMM_H
#define SPI_COMM_H

#include "scan.h"

typedef struct {
    float error;
    int16_t time_ms;
    int16_t has_nan;
    float result[3];
} icp_job_t;

typedef struct {
    int16_t number_of_poses;
    int16_t main_graph_size;
    int16_t max_subgraph_size;
    int16_t time_ms;
    int16_t has_nan;
} slam_job_t;

/**
 * @brief Sends augmented pose via SPI to the GAP9.
 *
 * @param pose Pointer to the augmented pose to send.
 */
void spi_send_aug_pose(aug_pose_t *pose);

/**
 * @brief Sends external augmented pose via SPI to the GAP9.
 *
 * @param pose Pointer to the external augmented pose to send.
 */
void spi_send_ext_aug_pose(ext_aug_pose_t *pose);

/**
 * @brief Sends pose update via SPI to the GAP9.
 *
 * @param pose Pointer to the pose to send.
 * @param if The ID of the pose.
 * @param drone_id The ID of the drone that sent the psoe update.
 */
void spi_send_ext_pose_upd(float *pose, int16_t id, int16_t drone_id);

/**
 * @brief Sends the ID of the new internal scan pose.
 *
 * @param scan_pose_id The ID of the scan pose.
 */
void spi_send_scan_info(int16_t scan_pose_id);

/**
 * @brief Sends a command to fetch poses from the GAP9.
 *
 * @param from_id The ID of the first pose.
 * @param nr_of_poses Number of poses to receive, starting from the ID
 * "from_id".
 */
void spi_rcv_poses(float poses[][3], int16_t from_id, int16_t nr_of_poses);

/**
 * @brief Inform GAP9 to perform graph optimization.
 *
 * @param job Pointer to the job structure. Contains the infromation about the
 * optimization job
 */
void spi_send_opt_cmd(slam_job_t *job);

#endif