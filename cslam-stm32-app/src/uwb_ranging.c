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
#include "estimator_kalman.h"
#include "log.h"
#include "param.h"
#include "semphr.h"
#include "static_mem.h"
#include "task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config_params.h"
#include "exploration.h"
#include "scan.h"
#include "scan_manager.h"
#include "spi_comm.h"
#include "uwb_api.h"
#include "uwb_api_message_utils.h"
#include "uwb_collision_det.h"
#include "uwb_ranging.h"

extern uint8_t NODE_ID;
extern QueueHandle_t poseQueue;

static volatile uint32_t uwb_pkt_loss = 0;
static Measurement measurements[NR_OF_NODES];

static scan_poses_t scan_poses = {0};
static volatile uint8_t optimize = 0;
static volatile uint8_t send_token_with_opt = 0;
uint8_t opt_cnt = 1;

uint8_t get_aug_pose_from_queue(aug_pose_t *aug_pose, TickType_t timeout_ms) {
    if (poseQueue != NULL) {
        if (xQueueReceive(poseQueue, aug_pose, (TickType_t)timeout_ms) ==
            pdTRUE) {
            return 1;
        } else
            return 0;
    } else
        return 0;
}

uint8_t send_aug_pose_to_queue(aug_pose_t aug_pose) {
    UBaseType_t available_space;
    if (poseQueue != NULL) {
        available_space = uxQueueSpacesAvailable(poseQueue);
        if (available_space == 0)
            xQueueReset(poseQueue);

        if (xQueueSendToBack(poseQueue, (void *)&aug_pose, (TickType_t)0) ==
            pdTRUE) {
            return 1;
        }
    }
    return 0;
}

uwb_node_coordinates_t get_current_pos_mm(void) {
    uwb_node_coordinates_t pos;
    pos.x = (int16_t)(1000 * logGetFloat(logGetVarId("kalman", "stateX")));
    pos.y = (int16_t)(1000 * logGetFloat(logGetVarId("kalman", "stateY")));
    pos.z = (int16_t)(1000 * logGetFloat(logGetVarId("kalman", "stateZ")));
    return pos;
}

uint8_t get_next_node(uint8_t node, uint8_t node_nr) {
    return (node + 1) % node_nr;
}

uwb_err_code_e uwb_4way_ranging_with_retry(uint8_t dest_id,
                                           uint32_t *range_dst_mm,
                                           uint8_t attempts) {
    uwb_node_coordinates_t node_pos = get_current_pos_mm();

    for (uint8_t i = 0; i < attempts; i++) {
        uwb_err_code_e e =
            uwb_do_4way_ranging_with_node(dest_id, node_pos, range_dst_mm);
        if (e == UWB_SUCCESS) {
            return UWB_SUCCESS;
        } else {
            uwb_pkt_loss++;
        }
        vTaskDelay(M2T(RETRY_TIME_MS));
    }

    return UWB_RX_TIMEOUT;
}

uwb_err_code_e uwb_send_data_packet(uint8_t *data, uint8_t data_len,
                                    uint8_t dest_id, uint8_t attempts) {
    UWB_message data_msg = {0};

    // Define token message
    data_msg.ctrl = 0xDE;
    data_msg.src = NODE_ID;
    data_msg.dest = dest_id;
    data_msg.code = UWB_ACK2_MSG;

    data_msg.data_len = data_len;
    memcpy(data_msg.data, data, data_len);

    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];

    for (uint8_t i = 0; i < attempts; i++) {
        uwb_err_code_e e =
            uwb_send_msg_wait_rsp(data_msg, 0, rx_buffer, &rx_buffer_len);
        UWB_message ack_message =
            uwb_message_from_array(rx_buffer, rx_buffer_len);

        if (e == UWB_SUCCESS && ack_message.data[0] == data_len) {
            return UWB_SUCCESS;
        }
    }
    return UWB_RX_TIMEOUT;
}

uint8_t is_token(uint8_t *array) {
    char key[] = "TOKEN";
    int res = strncmp(array, key, 5);
    if (res == 0) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t is_scan_pose(uint8_t data_len) {
    if (data_len == sizeof(aug_pose_t)) {
        return 1;
    } else {
        return 0;
    }
}

uwb_err_code_e pass_token(uint8_t target_id, uint8_t attempts) {
    UWB_message token_msg = {0};
    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];

    // Define token message
    token_msg.ctrl = 0xDE;
    token_msg.src = NODE_ID;
    token_msg.dest = target_id;
    token_msg.code = UWB_ACK_MSG;

    token_msg.data_len = 5;
    token_msg.data[0] = 'T';
    token_msg.data[1] = 'O';
    token_msg.data[2] = 'K';
    token_msg.data[3] = 'E';
    token_msg.data[4] = 'N';

    for (uint8_t i = 0; i < attempts; i++) {
        uwb_err_code_e e =
            uwb_send_msg_wait_rsp(token_msg, 0, rx_buffer, &rx_buffer_len);
        UWB_message ack_message =
            uwb_message_from_array(rx_buffer, rx_buffer_len);

        if (e == UWB_SUCCESS && ack_message.data_len == 1 &&
            ack_message.data[0] == 'T' && ack_message.dest == NODE_ID &&
            ack_message.src == target_id) {
            return UWB_SUCCESS;
        } else {
            uwb_pkt_loss++;
        }

        vTaskDelay(M2T(RETRY_TIME_MS));
    }

    return UWB_RX_TIMEOUT;
}

uwb_err_code_e pass_token_with_opt(uint8_t target_id, uint8_t attempts) {
    UWB_message token_msg = {0};
    uint8_t rx_buffer_len;
    uint8_t rx_buffer[RX_BUF_LEN];

    // Define token message
    token_msg.ctrl = 0xDE;
    token_msg.src = NODE_ID;
    token_msg.dest = target_id;
    token_msg.code = UWB_ACK_MSG;

    token_msg.data_len = 8;
    token_msg.data[0] = 'T';
    token_msg.data[1] = 'O';
    token_msg.data[2] = 'K';
    token_msg.data[3] = 'E';
    token_msg.data[4] = 'N';
    token_msg.data[5] = 'O';
    token_msg.data[6] = 'P';
    token_msg.data[7] = 'T';

    for (uint8_t i = 0; i < attempts; i++) {
        uwb_err_code_e e =
            uwb_send_msg_wait_rsp(token_msg, 0, rx_buffer, &rx_buffer_len);
        UWB_message ack_message =
            uwb_message_from_array(rx_buffer, rx_buffer_len);

        if (e == UWB_SUCCESS && ack_message.data_len == 1 &&
            ack_message.data[0] == 'T' && ack_message.dest == NODE_ID &&
            ack_message.src == target_id) {
            return UWB_SUCCESS;
        } else {
            uwb_pkt_loss++;
        }

        vTaskDelay(M2T(RETRY_TIME_MS));
    }

    return UWB_RX_TIMEOUT;
}

float filter_dist(float x, float y_prev, float alpha) {
    return (alpha * y_prev + (1 - alpha) * x);
}

void update_inter_drone_dist(float range, uint8_t drone_id) {
    float alpha = 0.4f;
    measurements[drone_id].range =
        filter_dist(range, measurements[drone_id].range, alpha);
    measurements[drone_id].time = xTaskGetTickCount();
}

void update_inter_drone_pos(float x_source, float y_source, uint8_t drone_id) {
    measurements[drone_id].x = x_source;
    measurements[drone_id].y = y_source;
}

float get_min_dtd_dist() {
    float min = 100.0f;

    for (uint8_t i = 0; i < NR_OF_NODES; i++) {
        uint32_t time_dif = xTaskGetTickCount() - measurements[i].time;
        if (time_dif < 3000 && measurements[i].range < min && i != NODE_ID) {
            min = measurements[i].range;
        }
    }
    return min;
}

int8_t choose_left_right() {
    uint8_t left = 0;
    uint8_t right = 0;

    for (uint8_t i = 0; i < NR_OF_NODES; i++) {
        uint32_t time_dif = xTaskGetTickCount() - measurements[i].time;
        if (time_dif < 3000 && i != NODE_ID) {
            if (get_side_lr(measurements[i].x, measurements[i].y) == 0)
                left++;
            else
                right++;
        }
    }

    if (left == right)
        return (2 * (rand() % 2) - 1);
    else if (left > right)
        return CW;
    else
        return CCW;
}

void check_for_neigh_drones(zones_t *zones, float dist_threshold) {
    for (uint8_t i = 0; i < NR_OF_NODES; i++) {
        if (i == NODE_ID)
            continue;

        int32_t time_dif = xTaskGetTickCount() - measurements[i].time;
        float x_neigh = measurements[i].x;
        float y_neigh = measurements[i].y;
        if (time_dif < 3000 && measurements[i].range < dist_threshold) {
            check_all_sides(x_neigh, y_neigh, zones);
        }
    }
    return;
}

void uwb_send_scan_to_drone(uint8_t dest_id) {
    // Send a scan
    aug_pose_t *pose;
    for (uint8_t i = 0; i < 20; i++) {
        pose = get_pose_from_scan_buffer(i);
        uwb_err_code_e e = uwb_send_data_packet((uint8_t *)pose,
                                                sizeof(aug_pose_t), dest_id, 3);
        if (e != UWB_SUCCESS) {
            DEBUG_PRINT("Error in sending a scan at packet %d\n", i);
        } else {
            // DEBUG_PRINT("UWB: Sent aug pose %d to D%d\n", pose->id, dest_id);
        }
    }
}

void uwb_send_updated_poses(uint8_t dest_id) {
    uint8_t buffer[113];
    uint8_t index = 1;
    DEBUG_PRINT("Scan pose count %d\n", scan_poses.count);
    for (int16_t i = 0; i < scan_poses.count; i++) {
        int16_t pose_id = scan_poses.poses[i];

        float poses[3];
        spi_rcv_poses(poses, pose_id, 1);
        memcpy(&buffer[index], &pose_id, sizeof(int16_t));
        index += sizeof(int16_t);

        memcpy(&buffer[index], poses, sizeof(poses));
        index += sizeof(poses);

        if (index == 113 || i == scan_poses.count - 1) {
            buffer[0] = (uint8_t)(index / 14);
            uwb_err_code_e e = uwb_send_data_packet((uint8_t *)buffer,
                                                    sizeof(buffer), dest_id, 3);
            if (e != UWB_SUCCESS) {
                DEBUG_PRINT("Updated pose packet failed!\n");
                return;
            } else {
                DEBUG_PRINT("UWB: Sent %d pose upd!\n", buffer[0]);
            }
            index = 1;
        }

        vTaskDelay(M2T(2));
    }
}

void add_scan_pose(int16_t pose_id) {
    scan_poses.poses[scan_poses.count] = pose_id;
    scan_poses.count++;
}

void ekf_correct_position(float *pose_unopt) {
    float x_cur = logGetFloat(logGetVarId("stateEstimate", "x"));
    float y_cur = logGetFloat(logGetVarId("stateEstimate", "y"));
    float z_cur = logGetFloat(logGetVarId("stateEstimate", "z"));
    float yaw_cur = logGetFloat(logGetVarId("stateEstimate", "yaw"));
    yaw_cur = yaw_cur / 180.0f * (float)M_PI;

    int16_t pose_id = get_last_pose_id();
    float pose_opt[3];
    spi_rcv_poses(pose_opt, pose_id, 1);

    float dx = pose_opt[0] - pose_unopt[0];
    float dy = pose_opt[1] - pose_unopt[1];
    float dyaw = pose_opt[2] - pose_unopt[2];

    vTaskDelay(M2T(2));
    estimatorKalmanInitToPose(x_cur + dx, y_cur + dy, z_cur, yaw_cur + dyaw);
}

void uwb_ranging_task(void *parameters) {
    UWB_measurement uwb_msrm = {0};
    UWB_message uwb_msg = {0};

    uint8_t agent_has_token = 0;

    if (NODE_ID == 0) {
        uwb_responder_off();
        agent_has_token = 1;
        uwb_set_gpio0(1);
    } else {
        uwb_responder_on();
    }

    vTaskDelay(M2T(100));

    uint32_t token_rcv_time = 0;
    const TickType_t xPeriod = 5;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t last_tick_count = xTaskGetTickCount();

    while (1) {
        if (agent_has_token) { // Transmit mode: Execute while having the token
            // Fetch the internally acquired augmented poses and send them to
            // the GAP9
            aug_pose_t aug_pose0 = {0};
            while (get_aug_pose_from_queue(&aug_pose0, 0)) {
                spi_send_aug_pose(&aug_pose0);
                vTaskDelay(M2T(1));
            }

            // 1. Do ranging with the other drones and get their positions
            for (uint8_t i = 0; i < NR_OF_NODES; i++) {
                if (i == NODE_ID)
                    continue;
                vTaskDelayUntil(&xLastWakeTime, xPeriod);
                uint32_t range_mm = 0;
                // Do ranging
                uwb_err_code_e e = uwb_4way_ranging_with_retry(i, &range_mm, 4);
                // Update the psoitions of drone i
                update_inter_drone_dist((float)range_mm / 1000.0f, i);
            }

            // 2. Check if there is a new internal scan and send it to other
            // drones
            if (is_new_scan_available()) {
                for (uint8_t i = NODE_ID + 1; i < NR_OF_NODES; i++) {
                    uwb_send_scan_to_drone(i);
                    DEBUG_PRINT("UWB: Sent scan to D%d\n", i);
                }

                // Notify GAP9 about the scan
                aug_pose_t *aug_pose = get_pose_from_scan_buffer(0);
                spi_send_scan_info(aug_pose->id);
                DEBUG_PRINT("GAP9: Scan ID %d sent\n", aug_pose->id);

                // Decide if to perform graph optimization this round
                if ((scan_poses.count == 4 * opt_cnt - 1) && NODE_ID == 0) {
                    opt_cnt++;
                    optimize = 1;
                    send_token_with_opt = 1;
                }

                // Store the id of the just acquired scan pose
                add_scan_pose(aug_pose->id);

                // Re-enable scan acquisition
                unlock_scan_acquisition();
            }

            // 3. Check for received augmented poses and send them to GAP9
            send_ext_aug_poses_to_gap9();

            // 4. Perform pose-graph optimization
            if (optimize) {
                DEBUG_PRINT("Optimizing...\n");
                // Get the value of the last unoptimize pose
                int16_t pose_id = get_last_pose_id();
                float pose_unopt[3];
                spi_rcv_poses(pose_unopt, pose_id, 1);

                // Optimize
                slam_job_t slam_job = {0};
                spi_send_opt_cmd(&slam_job);

                // Broadcast updated scan poses to other higher ID drones
                for (uint8_t i = NODE_ID + 1; i < NR_OF_NODES; i++) {
                    uwb_send_updated_poses(i);
                    DEBUG_PRINT("UWB: Sent upd poses to drone %d\n", i);
                }

                // Correct the drone's state estimate
                ekf_correct_position(pose_unopt);

                optimize = 0;

                // When passing the token, notify the next drone to also perform
                // optimization
                send_token_with_opt = 1;
            }

            // 6. Pass token and switch to listening mode
            uwb_err_code_e token_pass_ok = UWB_RX_TIMEOUT;
            uint8_t target_id = NODE_ID;
            vTaskDelay(M2T(2));
            while (token_pass_ok != UWB_SUCCESS) {
                target_id = get_next_node(target_id, NR_OF_NODES);
                if (target_id == NODE_ID) {
                    // DEBUG_PRINT("Error: Could not forward the token!\n");
                    vTaskDelay(M2T(10));
                    continue;
                }

                if (send_token_with_opt && target_id > NODE_ID) {
                    token_pass_ok = pass_token_with_opt(target_id, 5);
                } else {
                    token_pass_ok = pass_token(target_id, 5);
                }
                send_token_with_opt = 0;
            }

            agent_has_token = 0;
            uwb_responder_on();
        } else { // Listening mode: Execute while not having the token
            // Perform ranging with other drones that request it
            if (get_msrm_from_queue(&uwb_msrm, 1) == UWB_SUCCESS) {
                token_rcv_time = 0;
                float range_m = uwb_msrm.range;
                float x_source = uwb_msrm.posx / 1000.0f;
                float y_source = uwb_msrm.posy / 1000.0f;
                update_inter_drone_dist(range_m, uwb_msrm.src_id);
                update_inter_drone_pos(x_source, y_source, uwb_msrm.src_id);
            }

            // Check new UWB messages
            if (get_msg_from_queue(&uwb_msg, 1) == UWB_SUCCESS) {
                token_rcv_time = 0;
                // Received token and do optimization in the next loop
                if (is_token(uwb_msg.data) && uwb_msg.dest == NODE_ID &&
                    uwb_msg.data_len == 8) {
                    token_rcv_time = xTaskGetTickCount();
                    optimize = 1;
                } else if (is_token(uwb_msg.data) &&
                           uwb_msg.dest == NODE_ID) { // Received token
                    token_rcv_time = xTaskGetTickCount();
                } else if (uwb_msg.data_len == sizeof(aug_pose_t) &&
                           uwb_msg.dest == NODE_ID) { // Ext augmented pose
                    aug_pose_t *pose = (aug_pose_t *)uwb_msg.data;

                    ext_aug_pose_t *ext_pose = (ext_aug_pose_t *)pose;
                    ext_pose->drone_id = uwb_msg.src;
                    add_ext_aug_pose_to_gap9_buffer(ext_pose);
                }

                else if (uwb_msg.data_len == 113 &&
                         uwb_msg.dest == NODE_ID) { // Updated ext poses
                    uint8_t pose_count = uwb_msg.data[0];
                    uint8_t *base = &uwb_msg.data[1];
                    float *pose_upd;
                    for (uint8_t i = 0; i < pose_count; i++) {
                        int16_t pose_id = *(int16_t *)(base + 14 * i);
                        int16_t drone_id = uwb_msg.src;
                        pose_upd = (base + 14 * i + 2);
                        add_pose_upd_to_gap9_buffer(pose_upd, pose_id,
                                                    drone_id);
                    }
                }
            }

            if (xTaskGetTickCount() - token_rcv_time > (2 * RETRY_TIME_MS) &&
                token_rcv_time > 0) {
                token_rcv_time = 0;
                agent_has_token = 1;
                uwb_responder_off();
                vTaskDelay(M2T(1));
            }
        }

        if (xTaskGetTickCount() - last_tick_count > 1000) {
            last_tick_count = xTaskGetTickCount();
            uwb_pkt_loss = 0;
        }
    }
}

LOG_GROUP_START(UWB)
LOG_ADD(LOG_UINT32, packet_loss, &uwb_pkt_loss)
LOG_GROUP_STOP(UWB)
