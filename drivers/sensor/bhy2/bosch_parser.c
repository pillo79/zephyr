#include "bosch_parser.h"

#include "bhy2_sensors.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(bhy2, CONFIG_SENSOR_LOG_LEVEL);

void convertTime(uint64_t time_ticks, uint32_t *s, uint32_t *ns)
{
	uint64_t timestamp = time_ticks; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* timestamp is now in nanoseconds */
	*s = (uint32_t)(timestamp / UINT64_C(1000000000));
	*ns = (uint32_t)(timestamp - ((*s) * UINT64_C(1000000000)));
}

void parseData(const struct bhy2_fifo_parse_data_info *fifoData, void *arg)
{
	int8_t sz = fifoData->data_size - 1;

	LOG_DBG("Sensor: %i size: %i  value: ", fifoData->sensor_id, fifoData->data_size);
	for (uint8_t i = 0; i < (fifoData->data_size - 1); i++)
	{
		LOG_DBG("%02x ", fifoData->data_ptr[i]);
	}

	bhy2_sensor_parse_data(fifoData->sensor_id, fifoData->data_ptr, sz);
}

void parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	uint8_t meta_event_type = callback_info->data_ptr[0];
	uint8_t byte1 = callback_info->data_ptr[1];
	uint8_t byte2 = callback_info->data_ptr[2];
	uint32_t s, ns;
	const char *event_text;

	if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
	{
		event_text = "[META EVENT]";
	}
	else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
	{
		event_text = "[META EVENT WAKE UP]";
	}
	else
	{
		return;
	}

	convertTime(*callback_info->time_stamp, &s, &ns);

	struct parse_ref *parse_table = (struct parse_ref*)callback_ref;
	(void)parse_table;

	switch (meta_event_type)
	{
		case BHY2_META_EVENT_FLUSH_COMPLETE:
			//printf("%s; T: %u.%09u; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
			LOG_DBG("%s Flush complete for sensor id %u", event_text, byte1);
			break;
		case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
			//printf("%s; T: %u.%09u; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
			LOG_DBG("%s Sample rate changed for sensor id %u", event_text, byte1);
			break;
		case BHY2_META_EVENT_POWER_MODE_CHANGED:
			//printf("%s; T: %u.%09u; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
			LOG_DBG("%s Power mode changed for sensor id %u", event_text, byte1);
			break;
		case BHY2_META_EVENT_ALGORITHM_EVENTS:
			//printf("%s; T: %u.%09u; Algorithm event\r\n", event_text, s, ns);
			LOG_DBG("%s Algorithm event", event_text);
			break;
		case BHY2_META_EVENT_SENSOR_STATUS:
			//printf("%s; T: %u.%09u; Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
			LOG_DBG("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
			break;
		case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
			//printf("%s; T: %u.%09u; BSX event (do steps main)\r\n", event_text, s, ns);
			LOG_DBG("%s Algorithm event", event_text);
			break;
		case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
			//printf("%s; T: %u.%09u; BSX event (do steps calib)\r\n", event_text, s, ns);
			LOG_DBG("%s BSX event (do steps calib)", event_text);
			break;
		case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
			//printf("%s; T: %u.%09u; BSX event (get output signal)\r\n", event_text, s, ns);
			LOG_DBG("%s BSX event (get output signal)", event_text);
			break;
		case BHY2_META_EVENT_SENSOR_ERROR:
			//printf("%s; T: %u.%09u; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
			LOG_DBG("%s Sensor id %u reported error %u", event_text, byte1, byte2);
			break;
		case BHY2_META_EVENT_FIFO_OVERFLOW:
			//printf("%s; T: %u.%09u; FIFO overflow\r\n", event_text, s, ns);
			LOG_DBG("%s FIFO overflow", event_text);
			break;
		case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
			//printf("%s; T: %u.%09u; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
			LOG_DBG("%s Dynamic range changed for sensor id %u", event_text, byte1);
			break;
		case BHY2_META_EVENT_FIFO_WATERMARK:
			//printf("%s; T: %u.%09u; FIFO watermark reached\r\n", event_text, s, ns);
			LOG_DBG("%s FIFO watermark reached", event_text);
			break;
		case BHY2_META_EVENT_INITIALIZED:
			//printf("%s; T: %u.%09u; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
			//((uint16_t )byte2 << 8) | byte1);
			LOG_DBG("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t )byte2 << 8) | byte1);
			break;
		case BHY2_META_TRANSFER_CAUSE:
			//printf("%s; T: %u.%09u; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
			LOG_DBG("%s Transfer cause for sensor id %u", event_text, byte1);
			break;
		case BHY2_META_EVENT_SENSOR_FRAMEWORK:
			//printf("%s; T: %u.%09u; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
			LOG_DBG("%s Sensor framework event for sensor id %u", event_text, byte1);
			break;
		case BHY2_META_EVENT_RESET:
			//printf("%s; T: %u.%09u; Reset event\r\n", event_text, s, ns);
			LOG_DBG("%s Reset event", event_text);
			break;
		case BHY2_META_EVENT_SPACER:
			break;
		default:
			//printf("%s; T: %u.%09u; Unknown meta event with id: %u\r\n", event_text, s, ns, meta_event_type);
			LOG_DBG("%s Unknown meta event with id: %u", event_text, meta_event_type);
			break;
	}
}

void parseGeneric(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	uint32_t s, ns;
	convertTime(*callback_info->time_stamp, &s, &ns);

	printf("SID: %u; T: %u.%09u; ", callback_info->sensor_id, (unsigned int)s, (unsigned int)ns);
	for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
	{
		printf("%X ", callback_info->data_ptr[i]);
	}
	printf("\r\n");
}

void parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	uint32_t s, ns;
	convertTime(*callback_info->time_stamp, &s, &ns);
	//LOG_DBG("Debug message: ");
	LOG_DBG("[DEBUG MSG]; flag: 0x%02x data: %s", callback_info->data_ptr[0], (char*)&callback_info->data_ptr[1]);
	//printf("[DEBUG MSG]; T: %u.%09u; flag: 0x%x; data: %s\r\n",
	//s,
	//ns,
	//callback_info->data_ptr[0],
	//&callback_info->data_ptr[1]);
}

#if BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT
void bhy2_get_callback_info_delegate(uint8_t sensor_id,
		struct bhy2_fifo_parse_callback_table *info,
		const struct bhy2_dev *dev)
{
	info->callback_ref = NULL;
	if (sensor_id < BHY2_SENSOR_ID_MAX) {
		info->callback = parseData;
	} else {
		switch (sensor_id) {
			case BHY2_SYS_ID_META_EVENT:
			case BHY2_SYS_ID_META_EVENT_WU:
				info->callback = parseMetaEvent;
				break;
			case BHY2_SYS_ID_DEBUG_MSG:
				info->callback = parseDebugMessage;
				break;
			default:
				info->callback = NULL;
		}
	}
}
#endif
