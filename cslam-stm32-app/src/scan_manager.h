/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SCAN_MANAGER_H
#define SCAN_MANAGER_H

#include "FreeRTOS.h"
#include "queue.h"
#include "scan.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Keep track of the travelled distance.
 *
 * @param aug_pose Pointer to the augmented pose.
 */
void update_traveled_distance(aug_pose_t *aug_pose);

/**
 * @brief Set the travelled distance to zero.
 */
void reset_traveled_distance();

/**
 * @brief Check if traveled more than "value" meters from the last reset.
 *
 * @param value Distance threshold to compare with.
 * @return 1 for yes, 0 for no.
 */
uint8_t traveled_more_than(float value);

/**
 * @brief Initialize the scan queue. This queue stores potential candiates
 *        for scan poses.
 */
void init_scan_queue();

/**
 * @brief Fetch scan pose id in the mission task and valdiate it.
 *
 * @param scan_id Pointer where to store the scan pose id.
 * @param timeout_ms Maximum time to wait for a new queue element.
 * @return 1 for pop successful, 0 otherwise.
 */
uint8_t get_scan_id_from_queue(int16_t *scan_id, TickType_t timeout_ms);

/**
 * @brief Add new scan pose id to queue in the flight task.
 *
 * @param scan_id Scan pose id to be added.
 * @return 1 for push successful, 0 otherwise.
 */
uint8_t send_scan_id_to_queue(int16_t scan_id);

/**
 * @brief Add new scan pose id to queue in the flight task.
 *
 * @param scan_id Scan pose id to be added.
 * @return 1 for push successful, 0 otherwise.
 */
void add_pose_to_scan_buffer(aug_pose_t *pose);

/**
 * @brief The scan buffer always stores the last 20 augmented poses.
 *        In case a scan pose id is validated, the scan is obtained from
 *        the history and it is not necessary to fetch the scan from the GAP9.
 *        This function fetches the idx-th augmented pose in the buffer.
 *
 * @param idx Index of the pose to be fetched.
 * @return 1 Pointer to where the augmented pose is in the buffer.
 */
aug_pose_t *get_pose_from_scan_buffer(uint8_t);

/**
 * @brief Once the mission task validates a scan, it notifies the ranging
 *        task to process and share the scan with other drones. This
 *        function locks the acquisition of a new scan while the current
 *        one is being sent
 */
void lock_scan_acquisition();

/**
 * @brief Re-enables scan acqusition.
 */
void unlock_scan_acquisition();

/**
 * @brief Checks if a new scan was validated. The check happens in the ranging
 *        task.
 * #return 1 for yes, 0 otherwise.
 */
uint8_t is_new_scan_available();

/**
 * @brief Checks if there are buffered external augmented poses received from
 *        other drones and sends them to the GAP9.
 */
void send_ext_aug_poses_to_gap9();

/**
 * @brief After receiving a new external augmented pose via UWB, store it in
 *        the dedicated buffer.
 * @param ext_aug_pose0 Pointer to the pose to be stored
 */
void add_ext_aug_pose_to_gap9_buffer(ext_aug_pose_t *ext_aug_pose0);

/**
 * @brief After receiving a new pose update via UWB, store it in
 *        the dedicated buffer.
 * @param pose Pointer to the 3-dimensional pose.
 * @param id The id of the external pose.
 * @param drone_id The ID of the drone which sent the pose.
 */
void add_pose_upd_to_gap9_buffer(float *pose, int16_t id, int16_t drone_id);

#endif
