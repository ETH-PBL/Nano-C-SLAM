/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef UWB_RANGING_H
#define UWB_RANGING_H

#include "scan.h"
#include "uwb_api.h"
#include "uwb_api_message_utils.h"
#include "uwb_collision_det.h"

typedef struct Measurement {
    uint32_t time;
    float range;
    float x;
    float y;
} Measurement;

typedef struct scan_poses_t {
    uint16_t count;
    int16_t poses[30];
} scan_poses_t;

void uwb_ranging_task(void *parameters);

/**
 * @brief Calculate the minimum inter-drone distance
 *
 * @return Minimum distance in meters.
 */
float get_min_dtd_dist();

/**
 * @brief Check if there is another drone ahead closer than dist_threshold
 * meters.
 *
 * @param zones Pointer to a structure of zones.
 * @param dist_threshold Threshold on the distance. Another drone further than
 * this distance is not considered.
 */
void check_for_neigh_drones(zones_t *zones, float dist_threshold);

/**
 * @brief Chooses the steering direction according to how many drones are to the
 * left and right.
 *
 * @return 1 for CCW, -1 for CW.
 */
int8_t choose_left_right();

/**
 * @brief Sent UWB packet with acknowledgement.
 *
 * @param data Pointer to the data array.
 * @param data_len Length of the data array in bytes.
 * @param dest_id ID of the target UWB node.
 * @param attempts Number of re-attempts in case of failed transmission.
 * @return 1 - point is to the right, 0 to the left.
 */
uwb_err_code_e uwb_send_data_packet(uint8_t *data, uint8_t data_len,
                                    uint8_t dest_id, uint8_t attempts);

/**
 * @brief Break a scan into 20 UWB packets and send it packet by packet.
 *
 * @param dest_id ID of the target UWB node.
 */
void uwb_send_scan_to_drone(uint8_t dest_id);

/**
 * @brief Send the updated scan poses after optimization.
 *
 * @param dest_id ID of the target UWB node.
 */
void uwb_send_updated_poses(uint8_t dest_id);

/**
 * @brief Add the id of the scan pose to the scan pose buffer.
 *
 * @param dest_id ID of the target UWB node.
 */
void add_scan_pose(int16_t pose_id);

/**
 * @brief Buffer a newly acquired augmented pose in the mission task.
 *
 * @param aug_pose The augmented pose to be queued.
 * @return 1 for succesful operation, 0 otherwise.
 */
uint8_t send_aug_pose_to_queue(aug_pose_t aug_pose);

/**
 * @brief Buffer a newly acquired augmented pose in the mission task.
 *
 * @param aug_pose Pointer where to store the augmented pose popped from the
 * queue.
 * @param timeout_ms How long to wait in case queue is empty.
 * @return 1 for succesful operation, 0 otherwise.
 */
uint8_t get_aug_pose_from_queue(aug_pose_t *aug_pose, TickType_t timeout_ms);

#endif