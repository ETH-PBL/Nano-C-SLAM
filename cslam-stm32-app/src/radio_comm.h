/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef RADIO_COMM_H
#define RADIO_COMM_H

/**
 * @brief Sends data to the computer over the Crazyradio.
 * @param type Packet type.
 * @param data Pointer to the data array.
 * @param len Length of the array to be sent.
 */
void send_burst_data(uint8_t type, void *data, size_t len);

#endif