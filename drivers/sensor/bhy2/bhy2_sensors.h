#ifndef __BHY2_SENSORS__
#define __BHY2_SENSORS__

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#include "bosch/bhy2.h"

int bhy2_sensors_init(struct bhy2_dev *dev);

int bhy2_sensor_subchans(uint8_t sid);

int bhy2_sensor_parse_data(uint8_t sid, const uint8_t *data_buf, int data_len);

int bhy2_sensor_get_values(enum sensor_channel chan, struct sensor_value *values);

#endif
