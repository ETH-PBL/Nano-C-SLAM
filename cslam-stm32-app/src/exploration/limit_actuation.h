/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef LIMIT_ACTUATION
#define LIMIT_ACTUATION

/**
 * @brief Limits the increase in velocity to avoid unstable flight.
 *
 * @param x_vel_in Input velocity.
 * @return Output velocity.
 */
float limit_velocity(float x_vel_in);

#endif
