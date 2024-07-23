/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>



#define SLEEP_TIME_MS   1000

int main(void)
{
	//printk("hello main \n");
	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}