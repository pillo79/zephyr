#ifndef __BHY2_STRINGS_H__
#define __BHY2_STRINGS_H__

#include <stdint.h>

const char *bhy2_get_api_error(int8_t error_code);
const char *bhy2_get_sensor_error_text(uint8_t sensor_error);
const char *bhy2_get_sensor_name(uint8_t sensor_id);

#endif /* __BHY2_STRINGS_H__ */
