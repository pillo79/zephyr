/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

int main(void)
{
	const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bhy2);
	struct sensor_value gas_res;

	if (!device_is_ready(dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	printf("Device %p name is %s\n", dev, dev->name);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRIV_START+131);
	while (1) {
		k_sleep(K_MSEC(3000));

		sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
		sensor_channel_get(dev, SENSOR_CHAN_PRIV_START+131, &gas_res);

		printf("G: %d.%06d\n", gas_res.val1, gas_res.val2);
	}
	return 0;
}
