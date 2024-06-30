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

void v2t(float *pose, float T[3][3]) {
    float c = cosf(pose[2]);
    float s = sinf(pose[2]);
    T[0][0] = c;
    T[0][1] = -s;
    T[0][2] = pose[0];

    T[1][0] = s;
    T[1][1] = c;
    T[1][2] = pose[1];

    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = 1;
}

void t2v(float T[3][3], float *pose) {
    pose[0] = T[0][2];
    pose[1] = T[1][2];
    pose[2] = atan2f(T[1][0], T[0][0]);
}