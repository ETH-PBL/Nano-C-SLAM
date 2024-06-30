/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Carl Friess
 */

#include "FreeRTOS.h"
#include "crtp.h"
#include "task.h"
#include <assert.h>
#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

void send_burst_data(uint8_t type, void *data, size_t len) {
    assert(len < 0xff * 28);
    for (size_t i = 0, offset = 0; offset < len; i++, offset += 28) {
        CRTPPacket pk = {
            .header = CRTP_HEADER(1, 0),
            .size = MIN(len - offset, 28) + 2,
        };
        pk.data[0] = (type & 0x7f) | (offset + 28 >= len ? 0x80 : 0);
        pk.data[1] = i;
        memcpy(&(pk.data[2]), data + offset, pk.size - 2);
        crtpSendPacketBlock(&pk);
        vTaskDelay(M2T(1));
    }
}
