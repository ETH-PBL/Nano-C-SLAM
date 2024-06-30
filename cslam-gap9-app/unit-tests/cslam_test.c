/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "../icp/icp.h"
#include "pmsis.h"
#include "point_cloud.h"
#include "rcm-sparse-matrix.h"
#include "cslam-data/pos0.h"
#include "cslam-data/tof0.h"
#include "cslam-data/pos1.h"
#include "cslam-data/tof1.h"
#include "spi_cmds_decode.h"
#include "spi_driver.h"
#include "external_point_cloud.h"

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

static int16_t scans0[16] = {81, 157, 234, 505, 613, 691, 780, 850, 999, 1103, 1180, 1230, 1341, 1411, 1608, 1678};
static int16_t scans1[10] = {128, 236, 318, 394, 591, 639, 1170, 1230, 1359, 1634};

static PI_L2 icp_points_t ref_scan, cur_scan;

void add_internal_constraint(int16_t from_id, int16_t to_id) {
    icp_job_t icp_job = {0};
    float icp_res[3];
    // Extract first scan
    pcl_extract_scan(from_id, from_id + 20, &cur_scan, 1400);
    // Extract second scan
    pcl_extract_scan(to_id, to_id + 20, &ref_scan, 1400);
    // Run ICP
    icp(&ref_scan, &cur_scan, 25, 0.3f, icp_res, &icp_job);
    if (!icp_job.has_nan) {
        // printf("ICP added LC edge from %d to %d Tr: %.3f %.3f  "
        //        "Rot:%.3f \n",
        //        from_id, to_id, icp_res[0], icp_res[1], icp_res[2]);
        pcl_add_constraint(from_id, to_id, icp_res);
    }
}

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

void extract_scan(int16_t start_id, icp_points_t *points, int16_t range_limit, float pos[][3], int16_t tof[][32]) {

    const float offsets[4] = {0.025f, -0.025f, 0.02f, -0.02f};
    const float step = -((float)M_PI_4) / 8;

    points->num = 0;

    for (int16_t pose_id = start_id; pose_id < start_id + 20; pose_id++) {

        float frame_x = pos[pose_id][0];
        float frame_y = pos[pose_id][1];
        float frame_yaw = pos[pose_id][2];

        for (int dir = 0; dir < 4; dir++) {

            float dir_yaw = frame_yaw;
            if (dir == DIR_BACK)
                dir_yaw += (float)M_PI;
            else if (dir == DIR_LEFT)
                dir_yaw += (float)M_PI_2;
            else if (dir == DIR_RIGHT)
                dir_yaw -= (float)M_PI_2;
            float angle = -4 * step + step / 2;
            for (int col = 0; col < 8; angle += step, col++) {

                // Check if measurement is valid
                int16_t measurement = tof[pose_id][dir * 8 + col];
                if (measurement < 0 || measurement > range_limit) continue;

                // Apply rotation and add point
                float dist_x = (float)measurement / 1000.0f;
                float dist_y = tanf(angle) * dist_x;
                dist_x += offsets[dir];
                icp_point_t p = {
                    .x = frame_x + dist_x * cosf(dir_yaw) -
                         dist_y * sinf(dir_yaw),
                    .y = frame_y + dist_x * sinf(dir_yaw) +
                         dist_y * cosf(dir_yaw),
                };
                points->items[points->num++] = p;
            }
        }
    }
}

PI_L2 ext_scan_t ext_scan;
void match_with_external(int16_t cur_id, int16_t ext_ref_id) {
    printf("Matching self %d with external %d\n", cur_id, ext_ref_id);
	extract_scan(ext_ref_id, &ref_scan, 1400, pos0, tof0);
	pcl_extract_scan(cur_id, cur_id + 20, &cur_scan, 1400);

    float icp_res[3];
    icp_job_t icp_job = {0};
    icp(&ref_scan, &cur_scan, 25, 0.4f, icp_res, &icp_job);
    printf("ICP %.2f  %.2f  %.2f\n", icp_res[0], icp_res[1], icp_res[2]);
    // printf("lengths %d %d\n", ref_scan.num, cur_scan.num);
    if ((icp_job.has_nan == 0) && (icp_job.error < 0.01)) {
    	memset(&ext_scan, 0, sizeof(ext_scan_t));
    	ext_scan.from_drone = 0;
    	ext_scan.pose_id = ext_ref_id;

    	add_external_constraint(&ext_scan, cur_id, icp_res, pos0[ext_ref_id]);
        printf("Match ok!\n\n");
    }

}

#define SIZE_POSES 1699

void unit_test_cslam() {
    pcl_pose_t pose0 = {0};

    // Add poses to the graph
    for (int16_t i = 0; i < SIZE_POSES; i++) {
        pose0.id = i;
        memcpy(pose0.pos, pos1[i], 3 * sizeof(float));
        memcpy(pose0.tof, tof1[i], 32 * sizeof(int16_t));
        pcl_add_augmented_pose(pose0);
    }

    // Start the mission
    float new_ref_pose[3];

	// Round 0
    printf("Round 0\n");
	match_with_external(236, 81);
	match_with_external(318, 157);
	match_with_external(128, 505);
    slam_job_t job = {0};
    pcl_optimize(&job);

    printf("\n\n");

	// Round 1
    printf("Round 1\n");
    // float pose[3];
    // pcl_to_poses(pose, 394, 1);
    // printf("%.3f %.3f %.3f\n", pose[0], pose[1], pose[2]);
	match_with_external(394, 234);

	new_ref_pose[0] = 3.419; new_ref_pose[1] = 2.119; new_ref_pose[2] = 1.579;
	update_external_constraint(318, new_ref_pose);

	new_ref_pose[0] = -0.331; new_ref_pose[1] = -0.048; new_ref_pose[2] = -1.469;
	update_external_constraint(128, new_ref_pose);
    pcl_optimize(&job);
    printf("SLAM execution time [ms]: %d \n", job.time_ms);
    printf("Main graph: %d \n", job.main_graph_size);
    printf("Largest subgraph: %d \n", job.max_subgraph_size);
    printf("\n\n");

	// Round 2
    printf("Round 2\n");
	add_internal_constraint(1170, 591);
	add_internal_constraint(1230, 639);
	add_internal_constraint(1359, 394);
	add_internal_constraint(1634, 128);

	new_ref_pose[0] = 3.395; new_ref_pose[1] = 2.112; new_ref_pose[2] = 1.584;
	update_external_constraint(318, new_ref_pose);

	new_ref_pose[0] = -0.338; new_ref_pose[1] = -0.085; new_ref_pose[2] = -1.462;
	update_external_constraint(128, new_ref_pose);


	new_ref_pose[0] = 1.345; new_ref_pose[1] = 2.376; new_ref_pose[2] = 3.147;
	update_external_constraint(394, new_ref_pose);

	match_with_external(1170, 1341);
	match_with_external(639, 1411);

    pcl_optimize(&job);
    printf("SLAM execution time [ms]: %d \n", job.time_ms);
    printf("Main graph: %d \n", job.main_graph_size);
    printf("Largest subgraph: %d \n", job.max_subgraph_size);
    printf("\n\n");

	// Mission end

    // // Optimize graph
    // slam_job_t slam_job = {0};
    // pcl_optimize(&slam_job);
    // pcl_to_poses(pos_test, 0, SIZE_POSES);
    // printf("SLAM execution time [ms]: %d \n", slam_job.time_ms);
    // printf("Main graph: %d \n", slam_job.main_graph_size);
    // printf("Largest subgraph: %d \n", slam_job.max_subgraph_size);

    // // Evaluate results
    // float xy_error = 0.0f;
    // float yaw_error = 0.0f;
    // for (int16_t i = 0; i < SIZE_POSES; i++) {
    //     float error0 = fabs(pos_test[i][0] - pos_gt[i][0]);
    //     float error1 = fabs(pos_test[i][1] - pos_gt[i][1]);
    //     float error2 = fabs(pos_test[i][2] - pos_gt[i][2]);
    //     if (error0 + error1 > xy_error) xy_error = error0 + error1;
    //     if (error2 > yaw_error) yaw_error = error2;
    // }
    // if (xy_error < 0.001f && yaw_error < 0.001f) {
    //     printf("Unit test passed!");
    // } else {
    //     printf("Unit test failed!");
    // }
}

int main_func(void) {
    pi_freq_set(PI_FREQ_DOMAIN_FC, FREQU_FC);
    pi_freq_set(PI_FREQ_DOMAIN_CL, FREQU_CL);
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);

    printf("FC frequency : %ld\nCL frequency : %ld\n", cur_fc_freq,
           cur_cl_freq);

    uint32_t errors = 0;

    /* Init cluster configuration structure. */
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0; /* Set cluster ID. */

    /* Configure & open cluster. */
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev)) {
        printf("Cluster open failed !\n");
        pmsis_exit(-1);
    }

    unit_test_cslam();

    return errors;
}

/* Program Entry. */
int main(void) { return pmsis_kickoff((void *)main_func); }
