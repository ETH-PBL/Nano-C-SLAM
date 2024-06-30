/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

/**
 * @brief Pose to transformation matrix.
 *
 * @param pose Input pose
 * @param T Output matrix
 */
void v2t(float *, float T[3][3]);

/**
 * @brief Transformation matrix to pose.
 *
 * @param T Input matrix
 * @param pose Output pose
 */
void t2v(float T[3][3], float *pose);

#endif