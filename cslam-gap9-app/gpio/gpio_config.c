/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */
 
#include "pmsis.h"

static pi_device_t gpio_dummy0, gpio_dummy1;
static pi_device_t gpio_cf;

void gpio_init() {

    struct pi_gpio_conf gpio_conf;

    // GPIO 60
    pi_gpio_conf_init(&gpio_conf);
    gpio_conf.port = PI_PAD_060 / 32;

    pi_open_from_conf(&gpio_dummy0, &gpio_conf);
    pi_gpio_open(&gpio_dummy0);

    /* set pad to gpio mode */
    pi_pad_set_function(PI_PAD_060, PI_PAD_FUNC1);

    pi_gpio_flags_e flags = PI_GPIO_INPUT | PI_GPIO_PULL_ENABLE;
    pi_gpio_pin_configure(&gpio_dummy0, PI_PAD_060, flags);


    // GPIO 61
    pi_gpio_conf_init(&gpio_conf);
    gpio_conf.port = PI_PAD_061 / 32;

    pi_open_from_conf(&gpio_dummy1, &gpio_conf);
    pi_gpio_open(&gpio_dummy1);

    /* set pad to gpio mode */
    pi_pad_set_function(PI_PAD_061, PI_PAD_FUNC1);

    flags = PI_GPIO_INPUT;
    pi_gpio_pin_configure(&gpio_dummy1, PI_PAD_061, flags);


    // GPIO BUSY

    pi_gpio_conf_init(&gpio_conf);

    gpio_conf.port = PI_PAD_063 / 32;

    pi_open_from_conf(&gpio_cf, &gpio_conf);

    pi_gpio_open(&gpio_cf);

    /* set pad to gpio mode */
    pi_pad_set_function(PI_PAD_063, PI_PAD_FUNC1);

    /* configure gpio output */
    flags = PI_GPIO_OUTPUT;
    pi_gpio_pin_configure(&gpio_cf, PI_PAD_063, flags);

    pi_gpio_pin_write(&gpio_cf, PI_PAD_063, 0);
}

void set_cf_gpio(int8_t value) {
	pi_gpio_pin_write(&gpio_cf, PI_PAD_063, value);
}
