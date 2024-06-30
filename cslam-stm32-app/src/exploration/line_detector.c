/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug.h"

#define MIN_POINTS_ON_LINE 6
#define NO_LINE -10.0f
#define DIST_TOLERANCE 0.16f
#define MAX_DIST 2.0f

typedef struct {
    int16_t num;
    float x[32];
    float y[32];
} scan_frame_t;

static uint8_t get_index(float *array, uint8_t len, float val) {
    for (uint8_t i = 0; i < len; i++) {
        if (val >= array[i] && val < array[i + 1])
            return i;
    }

    return 255;
}

int cmpfunc(const void *a, const void *b) {
    const float *float1 = (const float *)a;
    const float *float2 = (const float *)b;

    if (*float1 < *float2)
        return -1;
    if (*float1 > *float2)
        return 1;
    return 0;
}

uint8_t linear_regression2(float *x, float *y, int16_t n, float *slope,
                           float *intercept) {
    *slope = 0.0f;
    *intercept = 0.0f;

    float XTX[2][2];
    memset(XTX, 0, 4 * sizeof(float));

    XTX[0][0] = (float)n;
    for (int16_t i = 0; i < n; i++) {
        XTX[0][1] += x[i];
        XTX[1][1] += x[i] * x[i];
    }
    XTX[1][0] = XTX[0][1];

    float inv[2][2];
    float det_denom = (XTX[0][0] * XTX[1][1] - XTX[1][0] * XTX[0][1]);
    if (det_denom <= 0.0001f)
        return 0;
    float det = 1.0f / det_denom;

    inv[0][0] = det * XTX[1][1];
    inv[0][1] = inv[1][0] = -det * XTX[0][1];
    inv[1][1] = det * XTX[0][0];

    float XTY[2] = {0.0f, 0.0f};

    for (int16_t i = 0; i < n; i++) {
        XTY[0] += y[i];
        XTY[1] += x[i] * y[i];
    }

    float beta[2];
    beta[0] = inv[0][0] * XTY[0] + inv[0][1] * XTY[1];
    beta[1] = inv[1][0] * XTY[0] + inv[1][1] * XTY[1];

    *slope = beta[1];
    *intercept = beta[0];

    return 1;
}

float dist_line_point(float slope, float intercept, float x, float y) {
    float nom = slope * x - y + intercept;
    if (nom < 0.0f)
        nom *= -1.0f;

    float denom = sqrtf(slope * slope + 1);

    return nom / denom;
}

float line_detector(float *array, int n) {
    scan_frame_t frame = {0};
    scan_frame_t *scan_frame = &frame;

    const float step = -((float)M_PI_4) / 8;
    float angle = -4 * step + step / 2;
    for (int16_t i = 0; i < 8; i++) {
        if ((array[i] > 0) && (array[i] < MAX_DIST)) {
            scan_frame->y[scan_frame->num] = array[i];
            scan_frame->x[scan_frame->num] = tanf(angle) * array[i];
            scan_frame->num++;
        }
        angle += step;
    }

    if (scan_frame->num == 0)
        return NO_LINE;

    float slope, intercept;
    uint8_t reg_res = linear_regression2(scan_frame->x, scan_frame->y,
                                         scan_frame->num, &slope, &intercept);
    if (!reg_res)
        return NO_LINE;

    int16_t point_cnt = 0;
    for (int16_t i = 0; i < scan_frame->num; i++) {
        float dist = dist_line_point(slope, intercept, scan_frame->x[i],
                                     scan_frame->y[i]);
        if (dist < DIST_TOLERANCE)
            point_cnt++;
    }

    if (point_cnt >= MIN_POINTS_ON_LINE)
        return atan(slope);
    else
        return NO_LINE;
}
