#ifndef __BHY2_DATASTORE_H__
#define __BHY2_DATASTORE_H__

#include <zephyr/drivers/sensor.h>

#include "bhy2_sensors.h"

void bhy2_datastore_init(void);

int bhy2_datastore_register(int channel);
int bhy2_datastore_put(uint8_t id, const void *data_buf, int data_len);
int bhy2_datastore_get(int channel, struct sensor_value *out);

#endif /* __BHY2_DATASTORE_H__ */
