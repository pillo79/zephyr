/* bhy2.c - Driver for Bosch Sensortec's BHY2 temperature, pressure,
 * humidity and gas sensor
 *
 * https://www.bosch-sensortec.com/bst/products/all_products/bhy2
 */

/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 * Copyright (c) 2022, Leonard Pollak
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include "bhy2_zephyr.h"
#include "bhy2_datastore.h"
#include "bosch_parser.h"

#include "bosch/bhy2.h"
#include "bosch/bhy2_strings.h"

LOG_MODULE_REGISTER(bhy2, CONFIG_SENSOR_LOG_LEVEL);

#define MAX_READ_WRITE_LEN 256

#define BHY2_SPI_READ_BIT	      0x80

#define BHY2_INT_THREAD_STACK_SIZE 2048
#define BHY2_INT_THREAD_PRIORITY 5

struct bhy2_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec int_gpio;
};

struct bhy2_data {
	struct gpio_callback int_gpio_cb;
	struct k_thread int_thread;
	k_thread_stack_t *int_thread_stack;
	struct k_sem int_sem;

	uint8_t _workBuffer[WORK_BUFFER_SIZE];
	uint8_t _acknowledgment;

	struct bhy2_dev _bhy2;

	uint8_t _sensorsPresent[32];

	uint32_t calc_gas_resistance;
};

int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	struct device *dev = intf_ptr;
	const struct bhy2_config *config = dev->config;
	uint8_t addr = reg_addr | BHY2_SPI_READ_BIT;

	const struct spi_buf tx_buf = {
			.buf = &addr,
			.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf[] = {
		{
			.buf = NULL,
			.len = 1
		},
		{
			.buf = reg_data,
			.len = length
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf)
	};

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	struct device *dev = intf_ptr;
	const struct bhy2_config *config = dev->config;

	const struct spi_buf tx_buf[2] = {
		{
			.buf = (uint8_t *)&reg_addr,
			.len = 1
		},
		{
			.buf = (uint8_t *)reg_data,
			.len = length
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)
	};

	return spi_transceive_dt(&config->spi, &tx, NULL);
}

void bhy2_delay_us(uint32_t us, void *private_data)
{
	k_sleep(K_USEC(us));
}

#define RETURN_ON_ERR(ret)								\
	if (ret != BHY2_OK) {								\
		LOG_ERR("%s:%i: %s", __FILE__, __LINE__, bhy2_get_api_error(ret));	\
		return ret;								\
	}

static int bhy2_attr_set(const struct device *dev,
			 enum sensor_channel chan,
			 enum sensor_attribute attr,
			 const struct sensor_value *val)
{
	struct bhy2_data *data = dev->data;
	int ret;

	if (chan < SENSOR_CHAN_PRIV_START) {
		LOG_WRN("attr_set() not supported on channel %d", chan);
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
                ret = bhy2_set_virt_sensor_cfg(chan - SENSOR_CHAN_PRIV_START, val[0].val1, val[1].val1*1000+val[1].val2/1000, &data->_bhy2);
		bhy2_datastore_register(chan - SENSOR_CHAN_PRIV_START);
		break;
	default:
		LOG_WRN("attr_set(): attribute %d not supported", attr);
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bhy2_sample_fetch(const struct device *dev,
			     enum sensor_channel chan)
{
	struct bhy2_data *data = dev->data;
	const struct bhy2_config *config = dev->config;

	if (chan == SENSOR_CHAN_ALL) {
		if (gpio_pin_get_dt(&config->int_gpio)) {
			/* new data available */
			int ret = bhy2_get_and_process_fifo(data->_workBuffer, WORK_BUFFER_SIZE, &data->_bhy2);
			RETURN_ON_ERR(ret);
		}
		return 0;
	} else if (chan >= SENSOR_CHAN_PRIV_START) {
		return bhy2_datastore_register(chan - SENSOR_CHAN_PRIV_START);
	} else {
		return -ENOTSUP;
	}
}

static int bhy2_channel_get(const struct device *dev,
			    enum sensor_channel chan,
			    struct sensor_value *val)
{
	return bhy2_datastore_get(chan - SENSOR_CHAN_PRIV_START, val);
}

static void bhy2_int_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct bhy2_data *data = dev->data;

	while (1) {
		k_sem_take(&data->int_sem, K_FOREVER);
		bhy2_sample_fetch(dev, SENSOR_CHAN_ALL);
	}
}

static void bhy2_int_gpio_callback(const struct device *dev,
				      struct gpio_callback *cb, uint32_t pins)
{
	struct bhy2_data *data =
		CONTAINER_OF(cb, struct bhy2_data, int_gpio_cb);

	k_sem_give(&data->int_sem);
}

static int bhy2_int_setup(const struct device *dev)
{
	struct bhy2_data *data = dev->data;
	const struct bhy2_config *config = dev->config;
	k_tid_t tid;

	k_sem_init(&data->int_sem, 0, 1);

	/* Initialize interrupt handling  */
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("Interrupt GPIO port not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT)) {
		LOG_ERR("Unable to configure interrupt GPIO");
		return -EINVAL;
	}

	gpio_init_callback(&(data->int_gpio_cb), bhy2_int_gpio_callback,
			   BIT(config->int_gpio.pin));

	if (gpio_add_callback(config->int_gpio.port,
			      &(data->int_gpio_cb))) {
		return -EINVAL;
	}

	if (gpio_pin_interrupt_configure_dt(&config->int_gpio,
					    GPIO_INT_EDGE_TO_ACTIVE)) {
		return -EINVAL;
	}

	tid = k_thread_create(&data->int_thread, data->int_thread_stack,
			      BHY2_INT_THREAD_STACK_SIZE,
			      bhy2_int_thread, (void *)dev,
			      NULL, NULL, K_PRIO_COOP(BHY2_INT_THREAD_PRIORITY),
			      0, K_NO_WAIT);
	(void)k_thread_name_set(tid, "bhy2");

	return 0;
}

static int bhy2_zephyr_init(const struct device *dev)
{
	struct bhy2_data *data = dev->data;

	int ret;
	uint8_t stat = 0;

	bhy2_datastore_init();

	ret = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us,
			MAX_READ_WRITE_LEN, (void*) dev, &data->_bhy2);
	RETURN_ON_ERR(ret);

	ret = bhy2_soft_reset(&data->_bhy2);
	RETURN_ON_ERR(ret);

	ret = bhy2_get_product_id(&stat, &data->_bhy2);
	RETURN_ON_ERR(ret);
	LOG_DBG("Chip ID: %02x", stat);

	ret = bhy2_get_boot_status(&stat, &data->_bhy2);
	RETURN_ON_ERR(ret);
	LOG_DBG("Boot status: %02x", stat);

	ret = bhy2_boot_from_flash(&data->_bhy2);
	RETURN_ON_ERR(ret);
	if (ret != BHY2_OK) return ret;

	ret = bhy2_get_boot_status(&stat, &data->_bhy2);
	RETURN_ON_ERR(ret);
	LOG_DBG("Boot status: %02x", stat);

	ret = bhy2_get_host_interrupt_ctrl(&stat, &data->_bhy2);
	RETURN_ON_ERR(ret);
	LOG_DBG("Interrupt ctrl: %02x", stat);

	ret = bhy2_get_host_intf_ctrl(&stat, &data->_bhy2);
	RETURN_ON_ERR(ret);
	LOG_DBG("Interface ctrl: %02x", stat);

	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parseMetaEvent, (void*) dev, &data->_bhy2);
	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parseMetaEvent, (void*) dev, &data->_bhy2);
	bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, parseDebugMessage, (void*) dev, &data->_bhy2);

	// All sensors' data are handled in the same generic way
	for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++) {
		bhy2_register_fifo_parse_callback(i, parseData, (void*) dev, &data->_bhy2);
	}

	bhy2_update_virtual_sensor_list(&data->_bhy2);
	bhy2_get_virt_sensor_list(data->_sensorsPresent, &data->_bhy2);

	if (IS_ENABLED(CONFIG_SENSOR_LOG_LEVEL_DBG)) {
		LOG_DBG("Present sensors: ");
		for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++) {
			if (data->_sensorsPresent[i >> 3] & BIT(i & 0x07)) {
				LOG_DBG("%i - %s", i, bhy2_get_sensor_name(i));
				log_process();
			}
		}
	}

	ret = bhy2_int_setup(dev);
	RETURN_ON_ERR(ret);

	ret = bhy2_get_and_process_fifo(data->_workBuffer, WORK_BUFFER_SIZE, &data->_bhy2);

	return ret;
}

static const struct sensor_driver_api bhy2_api_funcs = {
	.attr_set = bhy2_attr_set,
	.sample_fetch = bhy2_sample_fetch,
	.channel_get = bhy2_channel_get,
};

#define BHY2_SPI_OP                                                     \
	(SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER)

/*
 * Main instantiation macro.
 */
#define BHY2_DEFINE(inst)						\
	static K_KERNEL_STACK_DEFINE(bhy2_int_thread_stack_##inst,	\
				     BHY2_INT_THREAD_STACK_SIZE);       \
	static struct bhy2_data bhy2_data_##inst = {			\
		.int_thread_stack = bhy2_int_thread_stack_##inst,	\
	};								\
	static const struct bhy2_config bhy2_config_##inst = {		\
		.spi = SPI_DT_SPEC_INST_GET(inst, BHY2_SPI_OP, 0),	\
		.int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),	\
	};		                                                \
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			 bhy2_zephyr_init,				\
			 NULL,						\
			 &bhy2_data_##inst,				\
			 &bhy2_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &bhy2_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(BHY2_DEFINE)
