/*
 * Copyright (c) 2018 Oticon A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pseudo-random entropy generator for the ARCH_POSIX architecture:
 * Following the principle of reproducibility of the native_posix board
 * this entropy device will always generate the same random sequence when
 * initialized with the same seed
 *
 * This entropy source should only be used for testing.
 */

#define DT_DRV_COMPAT zephyr_qemu_rng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/random/random.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <stdlib.h>
#include <string.h>


static int entropy_qemu_get_entropy(const struct device *dev,
					    uint8_t *buffer,
					    uint16_t length)
{
	ARG_UNUSED(dev);

    sys_rand_get(buffer, length);
	return 0;
}

static int entropy_qemu_get_entropy_isr(const struct device *dev,
						uint8_t *buf,
						uint16_t len, uint32_t flags)
{
	ARG_UNUSED(flags);
	entropy_qemu_get_entropy(dev, buf, len);
	return len;
}

static int entropy_qemu_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static const struct entropy_driver_api entropy_qemu_api_funcs = {
	.get_entropy     = entropy_qemu_get_entropy,
	.get_entropy_isr = entropy_qemu_get_entropy_isr
};

DEVICE_DT_INST_DEFINE(0,
		    entropy_qemu_init, NULL,
		    NULL, NULL,
		    PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
		    &entropy_qemu_api_funcs);




