/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "graph-based-slam.h"
#include "pmsis.h"

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

#include "external_point_cloud.h"
#include "gpio_config.h"
#include "icp/icp.h"
#include "pmsis.h"
#include "point_cloud.h"
#include "rcm-sparse-matrix.h"
#include "spi_cmds_decode.h"
#include "spi_driver.h"

static uint8_t *spi_buf, *spi_buf_t;
PI_L2 icp_points_t ref_scan, cur_scan;

int get_spi_command(uint8_t *spi_cmd, uint8_t len) {
    if (spi_cmd[0] == 'S' && spi_cmd[len - 1] == 'E')
        return spi_cmd[1];
    else
        return -1;
}

uint8_t verbose = 1;

void slam_loop() {
    gpio_init();
    spi_buf = pi_l2_malloc(SPIBUF_SIZE);
    spi_buf_t = pi_l2_malloc(3 * SPIBUF_SIZE + 1);
    spi_slave_driver_init();

    pcl_pose_t pose0 = {0};
    ext_aug_pose_t ext_pose0 = {0};
    ext_pose_upd_t ext_pose_upd0 = {0};

    int16_t scan_info[2];
    int16_t pose_request[2];
    float icp_res[3];
    slam_job_t slam_job = {0};
    icp_job_t icp_job = {0};

    while (1) {
        // Wait new SPI packet
        spi_rcv_blocking(spi_buf, SPI_PKT_SIZE);
        pi_time_wait_us(100);
        int cmd = get_spi_command(spi_buf, SPI_PKT_SIZE);

        switch (cmd) {
        case POSE_CMD: // Received augmented pose: timestamp + id + pose value +
                       // 4 x ToF
            if (spi_decode_pose(spi_buf, &pose0) == SPI_OK) {
                pcl_add_augmented_pose(pose0);
            }
            break;

        case EXT_POSE_CMD: // Received external augmented pose
            if (spi_decode_ext_pose(spi_buf, &ext_pose0) == SPI_OK) {
                add_ext_aug_pose_to_buf(ext_pose0);
            }
            break;

        case UPD_EXT_POSE_CMD: // Received external pose update
            if (spi_decode_ext_pose_upd(spi_buf, &ext_pose_upd0) == SPI_OK) {
                add_pose_upd_to_buf(ext_pose_upd0);
                // printf("Pose %d from %d: %.1f %.1f %.1f\n",
                // ext_pose_upd0.pose_id, ext_pose_upd0.drone_id,
                // ext_pose_upd0.pos[0], ext_pose_upd0.pos[1],
                // ext_pose_upd0.pos[2]);
            }
            break;

        case SCAN_CMD: // New internal scan
            if (spi_decode_scan_info(spi_buf, scan_info) == SPI_OK) {
                set_cf_gpio(1); // GAP9 busy
                int16_t scan_pose_id = scan_info[0];
                add_scan_pose_id(scan_pose_id);
                if (verbose)
                    printf("Int scan %d rcv! %d poses\n\n", scan_pose_id,
                           pcl_get_pose_count());
                // print_poses();
                set_cf_gpio(0); // GAP9 free
            }
            break;

        case OPTIMIZE_CMD:  // Optimize pose graph
            set_cf_gpio(1); // GAP9 busy
            if (verbose) printf("Opt..\n");

            // Implement new ext constraints
            check_for_external_constraints();

            // Update modified ext constraints
            update_external_constraints();

            // Optimize graph
            pcl_optimize(&slam_job); // Optimize graph
            set_cf_gpio(0);          // GAP9 free

            // Send feedback to the STM32
            memcpy(spi_buf_t, &slam_job, sizeof(slam_job_t));
            spi_send_blocking(spi_buf_t, sizeof(slam_job_t));
            if (verbose) printf("\n");
            break;

        case POSE_REQ_CMD: // Send poses back to STM32
            if (spi_decode_pose_req(spi_buf, pose_request) == SPI_OK) {
                int16_t from_id = pose_request[0];
                int16_t nr_of_poses = pose_request[1];
                if (nr_of_poses > 100) break;
                pcl_to_poses((float *)(spi_buf_t), from_id, nr_of_poses);

                spi_send_blocking(spi_buf_t, 3 * 4 * nr_of_poses);
            }
            break;
        }
    }
}

int main_func(void) {
    pi_freq_set(PI_FREQ_DOMAIN_FC, FREQU_FC);
    pi_freq_set(PI_FREQ_DOMAIN_CL, FREQU_CL);
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);

    if (verbose)
        printf("FC frequency : %ld\nCL frequency : %ld\n", cur_fc_freq,
               cur_cl_freq);

    /* Init cluster configuration structure. */
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;

    /* Configure & open cluster. */
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev)) {
        if (verbose) printf("Cluster open failed !\n");
        pmsis_exit(-1);
    }

    slam_loop();

    return 0;
}

int main(void) { return pmsis_kickoff((void *)main_func); }
