#ifndef __BHY2_DATASTORE_H__
#define __BHY2_DATASTORE_H__

#include <zephyr/drivers/sensor.h>

#include "bhy2_sensors.h"

void bhy2_datastore_init(void);

int bhy2_datastore_register(enum bhy2_sensor_id sid);
int bhy2_datastore_put(enum bhy2_sensor_id sid, const void *data_buf, int data_len);
int bhy2_datastore_get(enum bhy2_sensor_id sid, struct sensor_value *out);

#endif /* __BHY2_DATASTORE_H__ */
