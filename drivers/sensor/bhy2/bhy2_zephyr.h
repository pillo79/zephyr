/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 * Copyright (c) 2022, Leonard Pollak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_DRIVERS_SENSOR_BHY2_H__
#define __ZEPHYR_DRIVERS_SENSOR_BHY2_H__

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define DT_DRV_COMPAT bosch_bhy2

#define BHY2_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#include "bosch/bhy2_defs.h"

#define WORK_BUFFER_SIZE        2048

#define SENSOR_DATA_FIXED_LENGTH (10)
#define SENSOR_QUEUE_SIZE       10

// With the introduction of the new sensor 116 ( SENSOR_ID_BSEC2_COLLECTOR), the max sensor packet size is now 21.
#define SENSOR_LONG_DATA_FIXED_LENGTH (21)
#define LONG_SENSOR_QUEUE_SIZE  4

#endif /* __ZEPHYR_DRIVERS_SENSOR_BHY2_H__ */
