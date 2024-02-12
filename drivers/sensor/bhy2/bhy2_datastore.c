#include "bhy2_sensors.h"
#include "bhy2_datastore.h"

#include <zephyr/sys/sflist.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bhy2, CONFIG_SENSOR_LOG_LEVEL);

#define FLAG_HAS_DATA 0x01

struct bhy2_sensor_data {
	sys_sfnode_t node;
	uint8_t id;
	struct sensor_value values[];
};

static sys_sflist_t data_list;
static struct k_mutex data_mutex;

static inline struct bhy2_sensor_data *find_sensor_data(int id)
{
	struct bhy2_sensor_data *data;

	SYS_SFLIST_FOR_EACH_CONTAINER(&data_list, data, node) {
		if (data->id == id) {
			return data;
		}
	}
	return NULL;
}

void bhy2_datastore_init(void)
{
	sys_sflist_init(&data_list);
	k_mutex_init(&data_mutex);
}

int bhy2_datastore_register(int channel)
{
	struct bhy2_sensor_data *data;
	int subchannels = 0;

	/* Check if the id is known */
	if (bhy2_sensors_get_info(BHY2_ID(channel), NULL, &subchannels) < 0) {
		LOG_ERR("Unknown sensor id %d", BHY2_ID(channel));
		return -EINVAL;
	}

	k_mutex_lock(&data_mutex, K_FOREVER);

	/* Check if the BHY2_ID(channel) is already registered */
	data = find_sensor_data(BHY2_ID(channel));
	if (data) {
		k_mutex_unlock(&data_mutex);
		LOG_DBG("Sensor id %d already registered", BHY2_ID(channel));
		return -EEXIST;
	}

	/* Allocate memory for the new data */
	data = k_malloc(sizeof(struct bhy2_sensor_data) + subchannels*sizeof(struct sensor_value));
	sys_sfnode_init(&data->node, 0);
	data->id = BHY2_ID(channel);
	sys_sflist_append(&data_list, &data->node);

	k_mutex_unlock(&data_mutex);
	return 0;
}

int bhy2_datastore_put(uint8_t id, const void *data_buf, int data_len)
{
	struct bhy2_sensor_data *data = NULL;

	k_mutex_lock(&data_mutex, K_FOREVER);

	/* Find the sensor data object */
	data = find_sensor_data(id);
	if (!data) {
		k_mutex_unlock(&data_mutex);
		LOG_DBG("Sensor id %d not registered", id);
		return 0;
	}

	int ret = bhy2_sensors_parse_data(id, data_buf, data_len, data->values);
	if (ret < 0) {
		k_mutex_unlock(&data_mutex);
		LOG_ERR("Failed to parse sensor data for id %d: %d", id, ret);
		return ret;
	}

	sys_sfnode_flags_set(&data->node, FLAG_HAS_DATA);
	k_mutex_unlock(&data_mutex);
	return 0;
}

int bhy2_datastore_get(int channel, struct sensor_value *out)
{
	struct bhy2_sensor_data *data = NULL;
	int subchannels = 0;
	int flags;

	if (bhy2_sensors_get_info(BHY2_ID(channel), NULL, &subchannels) < 0) {
		LOG_ERR("Failed to get sensor info for channel %d", BHY2_ID(channel));
		return -EINVAL;
	}
	if (BHY2_SUBCH(channel) >= subchannels) {
		LOG_ERR("Invalid subchannel %d for sensor channel %d", BHY2_SUBCH(channel), BHY2_ID(channel));
		return -EINVAL;
	}

	k_mutex_lock(&data_mutex, K_FOREVER);

	/* Find the sensor data object */
	data = find_sensor_data(BHY2_ID(channel));
	if (!data) {
		k_mutex_unlock(&data_mutex);
		LOG_ERR("Sensor id %d not registered", BHY2_ID(channel));
		return -EINVAL;
	}

	flags = sys_sfnode_flags_get(&data->node);
	if (!flags & FLAG_HAS_DATA) {
		k_mutex_unlock(&data_mutex);
		return -EAGAIN;
	}

	/* Copy the data */
	*out = data->values[BHY2_SUBCH(channel)];

	k_mutex_unlock(&data_mutex);
	return 0;
}
