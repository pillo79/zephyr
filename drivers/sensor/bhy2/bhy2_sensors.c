#include "bhy2_sensors.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bhy2, CONFIG_SENSOR_LOG_LEVEL);

struct bhy2_sensor_info
{
	enum bhy2_sensor_id id;
	enum bhy2_sensor_payload payload;
	float scaleFactor;
};

struct bhy2_payload_info
{
	int subchannels;
	int (*parser_fn)(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values);
};

static const struct bhy2_sensor_info BHY2_SENSOR_INFO[] = {
	{ SENSOR_ID_ACC_PASS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_ACC_RAW,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_ACC,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_ACC_BIAS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_ACC_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_ACC_RAW_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO_PASS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO_RAW,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO_BIAS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO_RAW_WU,	VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG_PASS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG_RAW,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG_BIAS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG_RAW_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GRA,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_GRA_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_LACC,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_LACC_WU,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_RV,			PQUATERNION,	    1.0 },
	{ SENSOR_ID_RV_WU,		PQUATERNION,	    1.0 },
	{ SENSOR_ID_GAMERV,		PQUATERNION,	    1.0 },
	{ SENSOR_ID_GAMERV_WU,		PQUATERNION,	    1.0 },
	{ SENSOR_ID_GEORV,		PQUATERNION,	    1.0 },
	{ SENSOR_ID_GEORV_WU,		PQUATERNION,	    1.0 },
	{ SENSOR_ID_ORI,		PEULER,		    0.01098 },
	{ SENSOR_ID_ORI_WU,		PEULER,		    0.01098 },
	{ SENSOR_ID_TILT_DETECTOR,	PEVENT,		    1.0 },
	{ SENSOR_ID_STD,		PEVENT,		    1.0 },
	{ SENSOR_ID_STC,		P32BITUNSIGNED,     1.0 },
	{ SENSOR_ID_STC_WU,		P32BITUNSIGNED,     1.0 },
	{ SENSOR_ID_SIG,		PEVENT,		    1.0 },
	{ SENSOR_ID_WAKE_GESTURE,	PEVENT,		    1.0 },
	{ SENSOR_ID_GLANCE_GESTURE,	PEVENT,		    1.0 },
	{ SENSOR_ID_PICKUP_GESTURE,	PEVENT,		    1.0 },
	{ SENSOR_ID_AR,			ACTIVITY,	    1.0 },
	{ SENSOR_ID_WRIST_TILT_GESTURE,	PEVENT,		    1.0 },
	{ SENSOR_ID_DEVICE_ORI,		P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_DEVICE_ORI_WU,	P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_STATIONARY_DET,	PEVENT,		    1.0 },
	{ SENSOR_ID_MOTION_DET,		PEVENT,		    1.0 },
	{ SENSOR_ID_ACC_BIAS_WU,	VECTOR3D,	    1.0 },
	{ SENSOR_ID_GYRO_BIAS_WU,	VECTOR3D,	    1.0 },
	{ SENSOR_ID_MAG_BIAS_WU,	VECTOR3D,	    1.0 },
	{ SENSOR_ID_STD_WU,		PEVENT,		    1.0 },
//	{ SENSOR_ID_KLIO,		KLIO,		    1.0 },
	{ SENSOR_ID_BSEC,		BSEC,		    1.0 },
	{ SENSOR_ID_BSEC2,		BSEC2,		    1.0 },
	{ SENSOR_ID_BSEC2_COLLECTOR,	BSEC2_COLLECTOR,    1.0 },
	{ SENSOR_ID_TEMP,		P16BITSIGNED,	    0.01 },
	{ SENSOR_ID_BARO,		P24BITUNSIGNED,     0.0078 },
	{ SENSOR_ID_HUM,		P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_GAS,		P32BITUNSIGNED,     1.0 },
	{ SENSOR_ID_TEMP_WU,		P16BITSIGNED,	    0.01 },
	{ SENSOR_ID_BARO_WU,		P24BITUNSIGNED,     0.0078 },
	{ SENSOR_ID_HUM_WU,		P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_GAS_WU,		P32BITUNSIGNED,     1.0 },
	{ SENSOR_ID_STC_HW,		P32BITUNSIGNED,     1.0 },
	{ SENSOR_ID_STD_HW,		PEVENT,		    1.0 },
	{ SENSOR_ID_SIG_HW,		PEVENT,		    1.0 },
	{ SENSOR_ID_STC_HW_WU,		P32BITUNSIGNED,     1.0 },
	{ SENSOR_ID_STD_HW_WU,		PEVENT,		    1.0 },
	{ SENSOR_ID_SIG_HW_WU,		PEVENT,		    1.0 },
	{ SENSOR_ID_ANY_MOTION,		PEVENT,		    1.0 },
	{ SENSOR_ID_ANY_MOTION_WU,	PEVENT,		    1.0 },
	{ SENSOR_ID_EXCAMERA,		P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_GPS,		VECTOR3D,	    1.0 },
	{ SENSOR_ID_LIGHT,		P16BITUNSIGNED,     46.296 },
	{ SENSOR_ID_PROX,		P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_LIGHT_WU,		P16BITUNSIGNED,     46.296 },
	{ SENSOR_ID_PROX_WU,		P8BITUNSIGNED,      1.0 },
	{ SENSOR_ID_BSEC_LEGACY,	BSEC,		    1.0 },
	{ DEBUG_DATA_EVENT,		DEBUG_DATA,	    1.0 },
	{ TIMESTAMP_SMALL_DELTA,	P8BITUNSIGNED,      0.000015625 },
	{ TIMESTAMP_SMALL_DELTA_WU,	P8BITUNSIGNED,      0.000015625 },
	{ TIMESTAMP_LARGE_DELTA,	P8BITUNSIGNED,      0.000015625 },
	{ TIMESTAMP_LARGE_DELTA_WU,	P8BITUNSIGNED,      0.000015625 },
	{ TIMESTAMP_FULL,		P8BITUNSIGNED,      0.000015625 },
	{ TIMESTAMP_FULL_WU,		P8BITUNSIGNED,      0.000015625 }
};

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
	if (scale_factor != 1.0f) {
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
	if (scale_factor != 1.0f) {
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
	if (scale_factor != 1.0f) {
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
	if (scale_factor != 1.0f) {
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
	if (scale_factor != 1.0f) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = (int32_t)result;
		dest->val2 = 0;
	}
	return 0;
}

static int getInt8(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	int8_t result = 0;
	if (index >= data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	result = (int8_t)data_buf[index];
	if (scale_factor != 1.0f) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
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
	if (scale_factor != 1.0f) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

static __unused int getInt24(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	int32_t result = 0;
	uint8_t res_len = 3;
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor != 1.0f) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

static int getInt32(struct sensor_value *dest, const uint8_t *data_buf, int data_len, uint8_t index, float scale_factor)
{
	int32_t result = 0;
	uint8_t res_len = sizeof(result);
	if (index + res_len > data_len) {
		dest->val1 = dest->val2 = 0;
		return -EINVAL;
	}

	memcpy(&result, &data_buf[index], res_len);
	if (scale_factor != 1.0f) {
		float scaled = (float)(result * scale_factor);
		dest->val1 = (int32_t)scaled;
		dest->val2 = (int32_t)((scaled - dest->val1) * 1000000);
	} else {
		dest->val1 = result;
		dest->val2 = 0;
	}
	return 0;
}

#define RET_ON_ERR(cmd) do { res = cmd; if (res) return res; } while (0)

static int parse3DVector(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	RET_ON_ERR(getInt16(&values[0], data_buf, data_len, 0, 1.0));	/* x */
	RET_ON_ERR(getInt16(&values[1], data_buf, data_len, 2, 1.0));	/* y */
	RET_ON_ERR(getInt16(&values[2], data_buf, data_len, 4, 1.0));	/* z */
	return 0;
}

static int parseEuler(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	RET_ON_ERR(getInt16(&values[0], data_buf, data_len, 0, info->scaleFactor));	/* heading */
	RET_ON_ERR(getInt16(&values[1], data_buf, data_len, 2, info->scaleFactor));	/* pitch */
	RET_ON_ERR(getInt16(&values[2], data_buf, data_len, 4, info->scaleFactor));	/* roll */
	return 0;
}

static int parseQuaternion(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	RET_ON_ERR(getInt16(&values[0], data_buf, data_len, 0, info->scaleFactor));     /* x */
	RET_ON_ERR(getInt16(&values[1], data_buf, data_len, 2, info->scaleFactor));	/* y */
	RET_ON_ERR(getInt16(&values[2], data_buf, data_len, 4, info->scaleFactor));	/* z */
	RET_ON_ERR(getInt16(&values[3], data_buf, data_len, 6, info->scaleFactor));	/* w */
	RET_ON_ERR(getUint16(&values[4], data_buf, data_len, 8, info->scaleFactor));	/* accuracy */
	return 0;
}

static __unused int parseBSEC(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	const float SCALE_BSEC_BVOC_EQ = 0.01f;
	const float SCALE_BSEC_COMP_T = 1.0f / 256;
	const float SCALE_BSEC_COMP_H = 1.0f / 500;

	RET_ON_ERR(getUint16(&values[0], data_buf, data_len, 0, 1.0));			/* iaq */
	RET_ON_ERR(getUint16(&values[1], data_buf, data_len, 2, 1.0));			/* iaq_s */
	RET_ON_ERR(getUint16(&values[2], data_buf, data_len, 4, SCALE_BSEC_BVOC_EQ));	/* b_voc_eq */
	RET_ON_ERR(getUint24(&values[3], data_buf, data_len, 6, 1.0));			/* co2_eq */
	RET_ON_ERR(getUint8(&values[4], data_buf, data_len, 9, 1.0));			/* accuracy */
	RET_ON_ERR(getInt16(&values[5], data_buf, data_len, 10, SCALE_BSEC_COMP_T));	/* comp_t */
	RET_ON_ERR(getUint16(&values[6], data_buf, data_len, 12, SCALE_BSEC_COMP_H));	/* comp_h */
	RET_ON_ERR(getFloat(&values[7], data_buf, data_len, 14, 1.0));	                /* comp_g */
	return 0;
}

static int parseBSEC2(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	RET_ON_ERR(getUint8(&values[0], data_buf, data_len, 0, 1.0));	/* gas_estimates[0] */
	RET_ON_ERR(getUint8(&values[1], data_buf, data_len, 1, 1.0));	/* gas_estimates[1] */
	RET_ON_ERR(getUint8(&values[2], data_buf, data_len, 2, 1.0));	/* gas_estimates[2] */
	RET_ON_ERR(getUint8(&values[3], data_buf, data_len, 3, 1.0));	/* gas_estimates[3] */
	RET_ON_ERR(getUint8(&values[4], data_buf, data_len, 4, 1.0));	/* accuracy */
	return 0;
}

static int parseBSEC2Collector(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	const float SCALE_BSEC_TS = 1.0E-9f;
	const float SCALE_BSEC_COMP_T = 1.0f / 256;
	const float SCALE_BSEC_COMP_H = 1.0f / 500;

	RET_ON_ERR(getUint64(&values[0], data_buf, data_len, 0, SCALE_BSEC_TS));	/* timestamp */
	RET_ON_ERR(getInt16(&values[1], data_buf, data_len, 8, SCALE_BSEC_COMP_T));	/* raw_temp */
	RET_ON_ERR(getFloat(&values[2], data_buf, data_len, 10, 1.0));			/* raw_pressure */
	RET_ON_ERR(getUint16(&values[3], data_buf, data_len, 14, SCALE_BSEC_COMP_H));	/* raw_hum */
	RET_ON_ERR(getFloat(&values[4], data_buf, data_len, 16, 1.0));			/* raw_gas */
	RET_ON_ERR(getUint8(&values[5], data_buf, data_len, 20, 1.0));			/* gas_index */
	return 0;
}

static int parseBSECLegacy(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res;
	RET_ON_ERR(getFloat(&values[0], data_buf, data_len, 0, 1.0));	/* comp_t */
	RET_ON_ERR(getFloat(&values[1], data_buf, data_len, 4, 1.0));	/* comp_h */
	//note that: SENSOR_DATA_FIXED_LENGTH is defined as 10 by default,
	//so all the fields below are 0 unless it's redefined to 29 and above
	RET_ON_ERR(getFloat(&values[2], data_buf, data_len, 8, 1.0));   /* comp_g */
	RET_ON_ERR(getFloat(&values[3], data_buf, data_len, 12, 1.0));	/* iaq */
	RET_ON_ERR(getFloat(&values[4], data_buf, data_len, 16, 1.0));	/* iaq_s */
	RET_ON_ERR(getFloat(&values[5], data_buf, data_len, 20, 1.0));	/* co2_eq */
	RET_ON_ERR(getFloat(&values[6], data_buf, data_len, 24, 1.0));	/* b_voc_eq */
	RET_ON_ERR(getUint8(&values[7], data_buf, data_len, 28, 1.0));	/* accuracy */
	return 0;
}

static int parseSingleData(const struct bhy2_sensor_info *info, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	int res = 0;
	switch (info->payload) {
	case P8BITSIGNED:
		res = getInt8(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case P8BITUNSIGNED:
		res = getUint8(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case P16BITSIGNED:
		res = getInt16(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case P16BITUNSIGNED:
		res = getUint16(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case P24BITUNSIGNED:
		res = getUint24(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case P32BITSIGNED:
		res = getInt32(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case P32BITUNSIGNED:
		res = getUint32(values, data_buf, data_len, 0, info->scaleFactor);
		break;
	case PEVENT:
		values->val1 = 1;
		values->val2 = 0;
		break;
	case ACTIVITY:
		res = getUint16(values, data_buf, data_len, 0, 1.0);
		break;
	default:
		res = -EINVAL;
		break;
	}
	return res;
}

static const struct bhy2_payload_info BHY2_PAYLOAD_INFO[NUM_PAYLOADS] = {
	[PQUATERNION]		= { 5, parseQuaternion },
	[VECTOR3D]		= { 3, parse3DVector },
	[PEULER]		= { 3, parseEuler },
	[P8BITSIGNED]		= { 1, parseSingleData },
	[P8BITUNSIGNED]	        = { 1, parseSingleData },
	[P16BITSIGNED]		= { 1, parseSingleData },
	[P16BITUNSIGNED]	= { 1, parseSingleData },
	[P32BITSIGNED]		= { 1, parseSingleData },
	[P32BITUNSIGNED]	= { 1, parseSingleData },
	[P24BITUNSIGNED]	= { 1, parseSingleData },
	[P40BITUNSIGNED]	= { 1, parseSingleData },
	[PEVENT]		= { 1, parseSingleData },
	[ACTIVITY]		= { 1, parseSingleData },
	[BSEC]			= { 8, parseBSECLegacy },
	[BSEC2]			= { 5, parseBSEC2 },
	[BSEC2_COLLECTOR]	= { 6, parseBSEC2Collector },
};

int bhy2_sensors_get_info(enum bhy2_sensor_id id, enum bhy2_sensor_payload *payload, int *subchannels)
{
	for (int i = 0; i < ARRAY_SIZE(BHY2_SENSOR_INFO); i++) {
		const struct bhy2_sensor_info *info = &BHY2_SENSOR_INFO[i];
		if (info->id == id) {
			if (payload) {
				*payload = info->payload;
			}
			if (subchannels) {
				*subchannels = BHY2_PAYLOAD_INFO[info->payload].subchannels;
			}
			return 0;
		}
	}

	return -EINVAL;
}

int bhy2_sensors_parse_data(enum bhy2_sensor_id id, const uint8_t *data_buf, int data_len, struct sensor_value *values)
{
	for (int i = 0; i < ARRAY_SIZE(BHY2_SENSOR_INFO); i++) {
		const struct bhy2_sensor_info *info = &BHY2_SENSOR_INFO[i];
		if (info->id == id) {
			return BHY2_PAYLOAD_INFO[info->payload].parser_fn(info, data_buf, data_len, values);
		}
	}

	return -EINVAL;
}
