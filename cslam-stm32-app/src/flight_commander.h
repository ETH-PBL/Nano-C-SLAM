/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef FLIGHT_CMD_H
#define FLIGHT_CMD_H

void flight_task(void *parameters);

/**
 * @brief Enable flight and start mission.
 */
void enable_commander();

#endif