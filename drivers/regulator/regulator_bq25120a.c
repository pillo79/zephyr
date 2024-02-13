/*
 * Copyright (c) 2024 Arduino SA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq25120a_regulator

#include <errno.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>

/* BQ25120A regulator related registers */
#define BQ25120A_SYS_VOUT_CTL_REG	0x06
#define BQ25120A_LS_LDO_CTL_REG		0x07

/* Buck related registers */
#define BQ25120A_EN_SYS_OUT	BIT(7)
#define BQ25120A_SYS_VOUT	GENMASK(6, 1) /* merged SYS_SEL and SYS_VOUT */

/* LDO related registers */
#define BQ25120A_EN_LS_LDO	BIT(7)
#define BQ25120A_LS_LDO		GENMASK(6, 2)

struct regulator_bq25120a_desc {
	uint8_t ctl_reg;
	uint8_t en_mask;
	uint8_t vout_mask;
	const struct linear_range *ranges;
	uint8_t nranges;
};

static const struct linear_range buck_ranges[] = {
	LINEAR_RANGE_INIT(1300000, 100000U, 0x10U, 0x1FU), /* 1.3V - 2.8V */
	LINEAR_RANGE_INIT(1500000,  83333U, 0x20U, 0x2FU), /* 1.5V - 2.75V */
	LINEAR_RANGE_INIT(1800000, 100000U, 0x30U, 0x3FU), /* 1.8V - 3.3V */
};

static const struct regulator_bq25120a_desc buck_desc = {
	.ctl_reg = BQ25120A_SYS_VOUT_CTL_REG,
	.en_mask = BQ25120A_EN_SYS_OUT,
	.vout_mask = BQ25120A_SYS_VOUT,
	.ranges = buck_ranges,
	.nranges = ARRAY_SIZE(buck_ranges),
};

static const struct linear_range ldo_ranges[] = {
	LINEAR_RANGE_INIT(800000, 100000U, 0x0U, 0x19U), /* 0.8 - 3.3V */
};

static const struct regulator_bq25120a_desc ldo_desc = {
	.ctl_reg = BQ25120A_LS_LDO_CTL_REG,
	.en_mask = BQ25120A_EN_LS_LDO,
	.vout_mask = BQ25120A_LS_LDO,
	.ranges = ldo_ranges,
	.nranges = ARRAY_SIZE(ldo_ranges),
};

struct regulator_bq25120a_config {
	struct regulator_common_config common;
	struct i2c_dt_spec i2c;
	const struct regulator_bq25120a_desc *desc;
};

struct regulator_bq25120a_data {
	struct regulator_common_data data;
};

static unsigned int regulator_bq25120a_count_voltages(const struct device *dev)
{
	const struct regulator_bq25120a_config *config = dev->config;

	return linear_range_group_values_count(config->desc->ranges, config->desc->nranges);
}

static int regulator_bq25120a_list_voltage(const struct device *dev, unsigned int idx,
					  int32_t *volt_uv)
{
	const struct regulator_bq25120a_config *config = dev->config;

	return linear_range_group_get_value(config->desc->ranges, config->desc->nranges, idx,
					    volt_uv);
}

static int regulator_bq25120a_set_voltage(const struct device *dev, int32_t min_uv, int32_t max_uv)
{
	const struct regulator_bq25120a_config *config = dev->config;
	uint16_t idx;
	int ret;

	ret = linear_range_group_get_win_index(config->desc->ranges, config->desc->nranges, min_uv,
					       max_uv, &idx);
	if (ret == -EINVAL) {
		return ret;
	}

	return i2c_reg_update_byte_dt(&config->i2c, config->desc->ctl_reg,
				      config->desc->vout_mask,
				      FIELD_PREP(config->desc->vout_mask, idx));
}

static int regulator_bq25120a_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct regulator_bq25120a_config *config = dev->config;
	int ret;
	uint8_t raw_reg;

	ret = i2c_reg_read_byte_dt(&config->i2c, config->desc->ctl_reg, &raw_reg);
	if (ret < 0) {
		return ret;
	}

	raw_reg = FIELD_GET(config->desc->vout_mask, raw_reg);

	return linear_range_group_get_value(config->desc->ranges, config->desc->nranges, raw_reg,
					    volt_uv);
}

static int regulator_bq25120a_enable(const struct device *dev)
{
	const struct regulator_bq25120a_config *config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, config->desc->ctl_reg,
				      config->desc->en_mask,
				      -1);
}

static int regulator_bq25120a_disable(const struct device *dev)
{
	const struct regulator_bq25120a_config *config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, config->desc->ctl_reg,
				      config->desc->en_mask,
				      0U);
}

static int regulator_bq25120a_init(const struct device *dev)
{
	const struct regulator_bq25120a_config *config = dev->config;
	int ret;
	uint8_t val;

	regulator_common_data_init(dev);

	if (!i2c_is_ready_dt(&config->i2c)) {
		return -ENODEV;
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, config->desc->ctl_reg, &val);
	if (ret < 0) {
		return ret;
	}


	return regulator_common_init(dev, (val & config->desc->en_mask) != 0U);
}

static const struct regulator_driver_api api = {
	.enable = regulator_bq25120a_enable,
	.disable = regulator_bq25120a_disable,
	.count_voltages = regulator_bq25120a_count_voltages,
	.list_voltage = regulator_bq25120a_list_voltage,
	.set_voltage = regulator_bq25120a_set_voltage,
	.get_voltage = regulator_bq25120a_get_voltage,
};

#define REGULATOR_BQ25120A_DEFINE(node_id, id, name)						\
	static struct regulator_bq25120a_data data_##id;					\
												\
	static const struct regulator_bq25120a_config config_##id = {				\
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),				\
		.i2c = I2C_DT_SPEC_GET(DT_GPARENT(node_id)),					\
		.desc = &name##_desc,								\
	};											\
												\
	DEVICE_DT_DEFINE(node_id, regulator_bq25120a_init, NULL, &data_##id, &config_##id,	\
			 POST_KERNEL, CONFIG_REGULATOR_BQ25120A_INIT_PRIORITY, &api);

#define REGULATOR_BQ25120A_DEFINE_COND(inst, child)						\
	COND_CODE_1(DT_NODE_EXISTS(DT_INST_CHILD(inst, child)),					\
		    (REGULATOR_BQ25120A_DEFINE(DT_INST_CHILD(inst, child), child##inst, child)),\
		    ())

#define REGULATOR_BQ25120A_DEFINE_ALL(inst)							\
	REGULATOR_BQ25120A_DEFINE_COND(inst, buck)						\
	REGULATOR_BQ25120A_DEFINE_COND(inst, ldo)

DT_INST_FOREACH_STATUS_OKAY(REGULATOR_BQ25120A_DEFINE_ALL)
