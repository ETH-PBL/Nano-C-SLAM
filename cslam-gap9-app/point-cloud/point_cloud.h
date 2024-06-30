/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "../icp/icp.h"
#include "pmsis.h"

// Augmented pose structure
typedef struct {
    int16_t id;
    int32_t timestamp;
    float pos[3];
    int16_t tof[4][8];
} pcl_pose_t;

// Structure that stores the optimization information
typedef struct {
    int16_t number_of_poses;
    int16_t main_graph_size;
    int16_t max_subgraph_size;
    int16_t time_ms;
    int16_t has_nan;
} slam_job_t;

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

/**
 * @brief Add a new augmented pose to the pose table.
 *
 * @param pose The new augmented pose to be added containing the timestamp,
 * drone's (x,y,yaw) and the ToF line information from 4 sensors
 */
void pcl_add_augmented_pose(pcl_pose_t pose);

/**
 * @brief Transform a scan from pose table entries to a point cloud.
 *
 * @param start_id The id where the scan starts in the pose table
 * @param end_id The id where the scan ends in the pose table
 * @param points Pointer to the point cloud where the scan should be stored
 * @param range_limit Only project distance measurements below this threshold
 */
void pcl_extract_scan(int16_t start_id, int16_t end_id, icp_points_t *points,
                      int16_t range_limit);

/**
 * @brief Add a loop closure constraint to the pose graph.
 *
 * @param from_node The constrained is from the node with this id
 * @param to_node The constrained is to the node with this id
 * @param icp Pointer to the ICP result that is used to derive the constraint
 */
void pcl_add_constraint(int16_t from_node, int16_t to_node, float *icp);

/**
 * @brief Start hierarchical pose graph optimization.
 *
 * @param job_info Pointer to where to store information about the graph
 * optimization
 */
void pcl_optimize(slam_job_t *job_info);

/**
 * @brief Move only the pose information (i.e., x, y, yaw) from the pose table
 * to an array.
 *
 * @param x The extracted poses will be stored here, and each pose occupies 3
 * entries in the array
 * @param from_id Start extracting from this id
 * @param nr_of_poses How many poses to extract starting from the given id
 */
void pcl_to_poses(float *x, int16_t from_id, int16_t nr_of_poses);

/**
 * @brief Get the number of poses in the pose table.
 *
 * @return Returns the number of poses currently in the graph / pose table
 */
int16_t pcl_get_pose_count();

/**
 * @brief Tries to match the scan associated to the pose with ID "cur_pose_id"
 * with one of the previously acquired internal scans and derive an intra-drone
 * loop closure constraint.
 *
 * @param cur_pose_id The ID of the scan pose to be matched
 */
void check_internal_constraints(uint16_t cur_pose_id);

/**
 * @brief Add the scan pose ID to the history buffer.
 *
 * @param pose_id The ID of the scan pose
 */
void add_scan_pose_id(uint16_t pose_id);

/**
 * @brief Get the idx-th scan pose id stored.
 *
 * @param idx The index of the scan pose id in the buffer
 */
int16_t pcl_get_scan_pose_id(uint16_t idx);

/**
 * @brief Returns the scan pose count.
 *
 * @return Number of stored scan poses
 */
uint16_t pcl_get_scan_pose_count();

/**
 * @brief Prints the stored augmented poses (only for debugging).
 */
void print_poses();

#endif
