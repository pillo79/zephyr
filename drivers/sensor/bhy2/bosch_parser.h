#ifndef BOSCH_PARSER_H_
#define BOSCH_PARSER_H_

#include "bosch/bhy2.h"

/**
* @brief Convert ticks of the 64 MHz oscillator to time
*
* @param time_ticks Timestamp of the 64 MHz oscillator in ticks
* @param s          Timestamp in seconds
* @param ns         Timestamp in nanoseconds
*/
void convertTime(uint64_t time_ticks, uint32_t *s, uint32_t *ns);

/**
* @brief Data payload is stored in FIFO buffer
*
* @param fifoData Pointer to data to be parsed
* @param arg      Reserved for future use.
*/
void parseData(const struct bhy2_fifo_parse_data_info *fifoData, void *arg);

/**
* @brief Parse Meta Events from the Bosch BHI260 Sensor
*
* @param callback_info Pointer containing information to be parsed
* @param callback_ref Reserved for future use
*/
void parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Extracts the timestamp information and prints it with the sensor ID. Also prints contents of callback_info.
*
* @param callback_info Pointer containing information to be parsed
* @param callback_ref  Reserved for future use
*/
void parseGeneric(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Parse debug message
*
* @param callback_info Pointer containing information to be parsed
* @param callback_ref  Reserved for future use
*/
void parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

#endif
