/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef EXTERNAL_POINT_CLOUD_H
#define EXTERNAL_POINT_CLOUD_H

#include "graph-based-slam.h"
#include "icp.h"
#include "point_cloud.h"

typedef struct {
    int16_t id;
    int32_t drone_id;
    float pos[3];
    int16_t tof[4][8];
} ext_aug_pose_t;

typedef struct {
    uint8_t taken;
    int16_t from_drone;
    int32_t pose_id;
    ext_aug_pose_t ext_aug_poses[20];
} ext_scan_t;

typedef struct {
    int16_t from_drone;
    int16_t ref_pose_id;
    int16_t cur_pose_id;
    float t[3];
    float z[3];
} ext_const_info_t;

typedef struct {
    int16_t pose_id;
    int32_t drone_id;
    float pos[3];
} ext_pose_upd_t;

/**
 * @brief Stores an external augmented pose received from other drone. The
 * external augmented poses come in batches of 20. Once all 20 were received,
 * the external scan can be computed (and matched it with an internal scan).
 *
 * @param ext_pose The external augmented pose
 */
void add_ext_aug_pose_to_buf(ext_aug_pose_t ext_pose);

/**
 * @brief Compute the external scan out of the received external augmented
 * poses.
 *
 * @param ext_scan_idx The index of the external scan in the external scan table
 * @param points Output scan
 * @param range_limit Maximim distance to consider for the ToF measurements.
 * Distance measurements higher than this value are ignored
 */
void get_external_scan(int16_t ext_scan_idx, icp_points_t *points,
                       int16_t range_limit);

/**
 * @brief Looks in the external scan table and tries to match the existing
 * external scans with the internal ones to derive inter-drone loop closures.
 */
void check_for_external_constraints();

/**
 * @brief Stores the external constraints discovered by the function
 * check_for_external_constraints().
 *
 * @param ext_scan Pointer to the external scan
 * @param cur_scan_id ID of the internal pose to match with
 * @param icp_res Pointer to the ICP result
 */
void add_external_constraint(ext_scan_t *ext_scan, int16_t cur_scan_id,
                             float *icp_res);

/**
 * @brief Determines the number of external constraints (i.e., inter-drone loop
 * closures).
 *
 * @return The number of added external constraints
 */
uint16_t external_constraints_count();

/**
 * @brief Fetches the idx-th external constraint from the external constraint
 * table.
 *
 * @return The constraint structure
 */
constraint get_ext_constraint(int16_t idx);

/**
 * @brief Add new pose update to the external pose update table.
 *
 * @param pose_upd The pose to update with
 */
void add_pose_upd_to_buf(ext_pose_upd_t pose_upd);

/**
 * @brief Parses the external pose update table and implements the external pose
 * updates.
 */
void update_external_constraints();

/**
 * @brief Enforces a pose update. Only used for debugging.
 *
 * @param ID of the internal pose
 * @param Constraint value
 */
void force_external_constraint(int16_t pose_id, float *z);

#endif