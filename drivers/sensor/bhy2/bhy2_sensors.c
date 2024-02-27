#include "bhy2_sensors.h"
#include "bosch/bhy2.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bhy2, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT bosch_bhy2

struct bhy2_chan_config;

/* Define all parsers */
#define DECLARE_PARSER_FN(name) \
	int name(const struct bhy2_chan_config *info,	\
		 const uint8_t *data_buf, int data_len, \
		 struct sensor_value *values)

typedef DECLARE_PARSER_FN(bhy2_parser_fn);

struct bhy2_chan_config {
	uint8_t sid;
	enum sensor_channel chan;
	int rate;
	int interval;
	int latency;
	bhy2_parser_fn *parser_fn;
	struct sensor_value *data;
	int subchans;
	float scale_factor;
};

/*
 * The following functions are referenced by the sensor parser map, and
 * that depends on the sensors that are enabled in the user's devicetree.
 */
static __maybe_unused DECLARE_PARSER_FN(parse_3axis_s16);
static __maybe_unused DECLARE_PARSER_FN(parse_bsec);
static __maybe_unused DECLARE_PARSER_FN(parse_bsec2);
static __maybe_unused DECLARE_PARSER_FN(parse_bsec2_collector);
static __maybe_unused DECLARE_PARSER_FN(parse_bsec_legacy);
static __maybe_unused DECLARE_PARSER_FN(parse_euler);
static __maybe_unused DECLARE_PARSER_FN(parse_quaternion);
static __maybe_unused DECLARE_PARSER_FN(parse_s16_as_float);
static __maybe_unused DECLARE_PARSER_FN(parse_scalar_event);
static __maybe_unused DECLARE_PARSER_FN(parse_scalar_u16);
static __maybe_unused DECLARE_PARSER_FN(parse_scalar_u32);
static __maybe_unused DECLARE_PARSER_FN(parse_scalar_u8);
static __maybe_unused DECLARE_PARSER_FN(parse_u24_as_float);

/* Get the actual map of sensor name to parse information */
#include "sensor_parser_map.h"

/* Get the BHY2 sensor id from the node identifier */
#define BHY2_CH_EXPAND(node_id, m_body)					\
	BHY2_CH_EXPAND2(node_id, m_body,				\
			DT_STRING_UNQUOTED(node_id, sensor_id),		\
		        DT_STRING_UNQUOTED_OR(node_id, channel,))	\

/* Use the sensor name to compute all the macro names, and pass it to
 * the actual macro body */
#define BHY2_CH_EXPAND2(node_id, m_body, s_ident, c_ident)		\
	m_body(node_id,							\
	       DT_CAT3(BHY2_SENSOR_ID_, s_ident,),			\
	       DT_CAT3(BHY2_, s_ident, _PARSER),			\
	       DT_CAT3(BHY2_, s_ident, _LENGTH),			\
	       DT_CAT3(BHY2_, s_ident, _FACTOR),			\
	       DT_CAT3(bhy2_, s_ident, _data),				\
	       c_ident)

/* Define the channel information using the macro names */
#define BHY2_CH_DATA(node_id, m_sid, m_parser, m_length, m_factor, m_data, m_chan) \
	static struct sensor_value m_data[m_length];

/* Define the constant channel information using the macro names */
#define BHY2_CH_CONST(node_id, m_sid, m_parser, m_length, m_factor, m_data, m_chan) \
{									\
	.sid = m_sid,							\
	.chan = COND_CODE_1(DT_NODE_HAS_PROP(node_id, channel),		\
			    (SENSOR_CHAN_ ## m_chan),			\
			    (SENSOR_CHAN_PRIV_START + m_sid)),		\
	.data = m_data,							\
	.parser_fn = &m_parser,						\
	.subchans = m_length,						\
	.rate = DT_PROP_OR(node_id, sampling_rate, 0),		        \
	.interval = DT_PROP_OR(node_id, sampling_interval, 0),	        \
	.latency = DT_PROP(node_id, sampling_latency_ms),		\
	.scale_factor = DT_STRING_UNQUOTED_OR(node_id,			\
					      scale_factor,		\
					      m_factor),		\
},

/* Define the constant channel information table and data storages */
#define BHY2_DEFINE(inst)						\
	static struct k_mutex data_mutex;                               \
	DT_FOREACH_CHILD_VARGS(DT_DRV_INST(inst),			\
			       BHY2_CH_EXPAND,				\
			       BHY2_CH_DATA)				\
	static const struct bhy2_chan_config BHY2_CHAN_CONFIG[] = {	\
		DT_FOREACH_CHILD_SEP_VARGS(DT_DRV_INST(inst),		\
				       BHY2_CH_EXPAND,			\
				       (,),				\
				       BHY2_CH_CONST)			\
	};

DT_INST_FOREACH_STATUS_OKAY(BHY2_DEFINE)


static int getFloat(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	float result = 0;
	uint8_t res_len = sizeof(result);
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	result = (float)(result * scale_factor);
	dest->val1 = (int32_t)result;
	dest->val2 = (int32_t)((result - dest->val1) * 1000000);
	return 0;
}

static int getUint8(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	uint8_t result = 0;
	if (index >= data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	result = data_buf[index];
	if (scale_factor) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

static int getUint16(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	uint16_t result = 0;
	uint8_t res_len = sizeof(result);
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

static int getUint24(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	uint32_t result = 0;
	uint8_t res_len = 3;
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

static int getUint32(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	uint32_t result = 0;
	uint8_t res_len = sizeof(result);
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

static int getUint64(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	uint64_t result = 0;
	uint8_t res_len = sizeof(result);
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = (int32_t)result;
		dest->val2 = 0;
	}
	return 0;
}

static int getInt16(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	int16_t result = 0;
	uint8_t res_len = sizeof(result);
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

#define BHY2_ASSERT(cmd) do { int res = cmd; if (res) return res; } while (0)

static int parse_3axis_s16(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getInt16(&values[0], data_buf, data_len, 0, info->scale_factor));	/* x */
	BHY2_ASSERT(getInt16(&values[1], data_buf, data_len, 2, info->scale_factor));	/* y */
	BHY2_ASSERT(getInt16(&values[2], data_buf, data_len, 4, info->scale_factor));	/* z */
	return 0;
}

static int parse_euler(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getInt16(&values[0], data_buf, data_len, 0, info->scale_factor));	/* heading */
	BHY2_ASSERT(getInt16(&values[1], data_buf, data_len, 2, info->scale_factor));	/* pitch */
	BHY2_ASSERT(getInt16(&values[2], data_buf, data_len, 4, info->scale_factor));	/* roll */
	return 0;
}

static int parse_quaternion(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getInt16(&values[0], data_buf, data_len, 0, info->scale_factor));	/* x */
	BHY2_ASSERT(getInt16(&values[1], data_buf, data_len, 2, info->scale_factor));	/* y */
	BHY2_ASSERT(getInt16(&values[2], data_buf, data_len, 4, info->scale_factor));	/* z */
	BHY2_ASSERT(getInt16(&values[3], data_buf, data_len, 6, info->scale_factor));	/* w */
	BHY2_ASSERT(getUint16(&values[4], data_buf, data_len, 8, info->scale_factor));	/* accuracy */
	return 0;
}

static int parse_bsec(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	const float SCALE_BSEC_BVOC_EQ = 0.01f;
	const float SCALE_BSEC_COMP_T = 1.0f / 256;
	const float SCALE_BSEC_COMP_H = 1.0f / 500;

	BHY2_ASSERT(getUint16(&values[0], data_buf, data_len, 0, 1.0));			/* iaq */
	BHY2_ASSERT(getUint16(&values[1], data_buf, data_len, 2, 1.0));			/* iaq_s */
	BHY2_ASSERT(getUint16(&values[2], data_buf, data_len, 4, SCALE_BSEC_BVOC_EQ));	/* b_voc_eq */
	BHY2_ASSERT(getUint24(&values[3], data_buf, data_len, 6, 1.0));			/* co2_eq */
	BHY2_ASSERT(getUint8(&values[4], data_buf, data_len, 9, 1.0));			/* accuracy */
	BHY2_ASSERT(getInt16(&values[5], data_buf, data_len, 10, SCALE_BSEC_COMP_T));	/* comp_t */
	BHY2_ASSERT(getUint16(&values[6], data_buf, data_len, 12, SCALE_BSEC_COMP_H));	/* comp_h */
	BHY2_ASSERT(getFloat(&values[7], data_buf, data_len, 14, 1.0));			/* comp_g */
	return 0;
}

static int parse_bsec_legacy(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getFloat(&values[0], data_buf, data_len, 0, 1.0));	/* comp_t */
	BHY2_ASSERT(getFloat(&values[1], data_buf, data_len, 4, 1.0));	/* comp_h */
	//note that: SENSOR_DATA_FIXED_LENGTH is defined as 10 by default,
	//so all the fields below are 0 unless it's redefined to 29 and above
	BHY2_ASSERT(getFloat(&values[2], data_buf, data_len, 8, 1.0));	/* comp_g */
	BHY2_ASSERT(getFloat(&values[3], data_buf, data_len, 12, 1.0));	/* iaq */
	BHY2_ASSERT(getFloat(&values[4], data_buf, data_len, 16, 1.0));	/* iaq_s */
	BHY2_ASSERT(getFloat(&values[5], data_buf, data_len, 20, 1.0));	/* co2_eq */
	BHY2_ASSERT(getFloat(&values[6], data_buf, data_len, 24, 1.0));	/* b_voc_eq */
	BHY2_ASSERT(getUint8(&values[7], data_buf, data_len, 28, 1.0));	/* accuracy */
	return 0;
}

static int parse_bsec2(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getUint8(&values[0], data_buf, data_len, 0, 1.0));	/* gas_estimates[0] */
	BHY2_ASSERT(getUint8(&values[1], data_buf, data_len, 1, 1.0));	/* gas_estimates[1] */
	BHY2_ASSERT(getUint8(&values[2], data_buf, data_len, 2, 1.0));	/* gas_estimates[2] */
	BHY2_ASSERT(getUint8(&values[3], data_buf, data_len, 3, 1.0));	/* gas_estimates[3] */
	BHY2_ASSERT(getUint8(&values[4], data_buf, data_len, 4, 1.0));	/* accuracy */
	return 0;
}

static int parse_bsec2_collector(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	const float SCALE_BSEC_TS = 1.0E-9f;
	const float SCALE_BSEC_COMP_T = 1.0f / 256;
	const float SCALE_BSEC_COMP_H = 1.0f / 500;

	BHY2_ASSERT(getUint64(&values[0], data_buf, data_len, 0, SCALE_BSEC_TS));	/* timestamp */
	BHY2_ASSERT(getInt16(&values[1], data_buf, data_len, 8, SCALE_BSEC_COMP_T));	/* raw_temp */
	BHY2_ASSERT(getFloat(&values[2], data_buf, data_len, 10, 1.0));			/* raw_pressure */
	BHY2_ASSERT(getUint16(&values[3], data_buf, data_len, 14, SCALE_BSEC_COMP_H));	/* raw_hum */
	BHY2_ASSERT(getFloat(&values[4], data_buf, data_len, 16, 1.0));			/* raw_gas */
	BHY2_ASSERT(getUint8(&values[5], data_buf, data_len, 20, 1.0));			/* gas_index */
	return 0;
}

static int parse_scalar_u32(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getUint32(values, data_buf, data_len, 0, info->scale_factor));
	return 0;
}

static int parse_scalar_u16(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getUint16(values, data_buf, data_len, 0, info->scale_factor));
	return 0;
}

static int parse_scalar_u8(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	BHY2_ASSERT(getUint8(values, data_buf, data_len, 0, info->scale_factor));
	return 0;
}

static int parse_scalar_event(const struct bhy2_chan_config *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	ARG_UNUSED(info);
	ARG_UNUSED(data_buf);
	ARG_UNUSED(data_len);

	values->val1 = 1;
	values->val2 = 0;
	return 0;
}

static inline const struct bhy2_chan_config *find_chan_config_by_sid(uint8_t sid)
{
	for (int i = 0; i < ARRAY_SIZE(BHY2_CHAN_CONFIG); i++) {
		if (BHY2_CHAN_CONFIG[i].sid == sid) {
			return &BHY2_CHAN_CONFIG[i];
		}
	}

	return NULL;
}

static inline const struct bhy2_chan_config *find_chan_config_by_chan(enum sensor_channel chan)
{
	for (int i = 0; i < ARRAY_SIZE(BHY2_CHAN_CONFIG); i++) {
		if (BHY2_CHAN_CONFIG[i].chan == chan) {
			return &BHY2_CHAN_CONFIG[i];
		}
	}

	return NULL;
}

int bhy2_sensor_subchans(uint8_t sid)
{
	const struct bhy2_chan_config *info = find_chan_config_by_sid(sid);
	return info ? info->subchans : -EINVAL;
}

int bhy2_sensor_parse_data(uint8_t sid, const uint8_t *data_buf, int data_len)
{
	int ret;
	const struct bhy2_chan_config *info = find_chan_config_by_sid(sid);

	if (info) {
		k_mutex_lock(&data_mutex, K_FOREVER);
		ret = info->parser_fn(info, data_buf, data_len, info->data);
		k_mutex_unlock(&data_mutex);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

int bhy2_sensor_get_values(enum sensor_channel chan, struct sensor_value *values)
{
	const struct bhy2_chan_config *info = find_chan_config_by_chan(chan);

	if (!info) {
		return -EINVAL;
	}

	k_mutex_lock(&data_mutex, K_FOREVER);
	memcpy(values, info->data, info->subchans * sizeof(struct sensor_value));
	k_mutex_unlock(&data_mutex);
	return 0;
}

int bhy2_sensors_init(struct bhy2_dev *dev)
{
	for (int i = 0; i < ARRAY_SIZE(BHY2_CHAN_CONFIG); i++) {
		const struct bhy2_chan_config *info = &BHY2_CHAN_CONFIG[i];
		float rate_hz;

		memset(info->data, 0, info->subchans * sizeof(struct sensor_value));
		if (info->rate && !info->interval) {
			rate_hz = info->rate;
		} else if (info->interval) {
			rate_hz = 1.0f / info->interval;
		} else {
			LOG_ERR("Invalid sampling rate and/or interval for sensor %d", info->sid);
			continue;
		}
		
		BHY2_ASSERT(bhy2_set_virt_sensor_cfg(info->sid, rate_hz, info->latency, dev));
	}

	return 0;
}
