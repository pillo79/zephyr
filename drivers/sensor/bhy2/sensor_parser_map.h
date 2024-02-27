/*
 * Copyright 2024 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define BHY2_ACC_BIAS_PARSER parse_3axis_s16
#define BHY2_ACC_BIAS_LENGTH 3
#define BHY2_ACC_BIAS_FACTOR (1.0f / 4096.0f)

#define BHY2_ACC_BIAS_WU_PARSER parse_3axis_s16
#define BHY2_ACC_BIAS_WU_LENGTH 3
#define BHY2_ACC_BIAS_WU_FACTOR (1.0f / 4096.0f)

#define BHY2_ACC_PARSER parse_3axis_s16
#define BHY2_ACC_LENGTH 3
#define BHY2_ACC_FACTOR (1.0f / 4096.0f)

#define BHY2_ACC_PASS_PARSER parse_3axis_s16
#define BHY2_ACC_PASS_LENGTH 3
#define BHY2_ACC_PASS_FACTOR (1.0f / 4096.0f)

#define BHY2_ACC_RAW_PARSER parse_3axis_s16
#define BHY2_ACC_RAW_LENGTH 3
#define BHY2_ACC_RAW_FACTOR (1.0f / 4096.0f)

#define BHY2_ACC_RAW_WU_PARSER parse_3axis_s16
#define BHY2_ACC_RAW_WU_LENGTH 3
#define BHY2_ACC_RAW_WU_FACTOR (1.0f / 4096.0f)

#define BHY2_ACC_WU_PARSER parse_3axis_s16
#define BHY2_ACC_WU_LENGTH 3
#define BHY2_ACC_WU_FACTOR (1.0f / 4096.0f)

#define BHY2_GRA_PARSER parse_3axis_s16
#define BHY2_GRA_LENGTH 3
#define BHY2_GRA_FACTOR (1.0f / 4096.0f)

#define BHY2_GRA_WU_PARSER parse_3axis_s16
#define BHY2_GRA_WU_LENGTH 3
#define BHY2_GRA_WU_FACTOR (1.0f / 4096.0f)

#define BHY2_LACC_PARSER parse_3axis_s16
#define BHY2_LACC_LENGTH 3
#define BHY2_LACC_FACTOR (1.0f / 4096.0f)

#define BHY2_LACC_WU_PARSER parse_3axis_s16
#define BHY2_LACC_WU_LENGTH 3
#define BHY2_LACC_WU_FACTOR (1.0f / 4096.0f)

#define BHY2_MAG_BIAS_PARSER parse_3axis_s16
#define BHY2_MAG_BIAS_LENGTH 3
#define BHY2_MAG_BIAS_FACTOR (2500.0f / 32768.0f)

#define BHY2_MAG_BIAS_WU_PARSER parse_3axis_s16
#define BHY2_MAG_BIAS_WU_LENGTH 3
#define BHY2_MAG_BIAS_WU_FACTOR (2500.0f / 32768.0f)

#define BHY2_MAG_PARSER parse_3axis_s16
#define BHY2_MAG_LENGTH 3
#define BHY2_MAG_FACTOR (2500.0f / 32768.0f)

#define BHY2_MAG_PASS_PARSER parse_3axis_s16
#define BHY2_MAG_PASS_LENGTH 3
#define BHY2_MAG_PASS_FACTOR (2500.0f / 32768.0f)

#define BHY2_MAG_RAW_PARSER parse_3axis_s16
#define BHY2_MAG_RAW_LENGTH 3
#define BHY2_MAG_RAW_FACTOR (2500.0f / 32768.0f)

#define BHY2_MAG_RAW_WU_PARSER parse_3axis_s16
#define BHY2_MAG_RAW_WU_LENGTH 3
#define BHY2_MAG_RAW_WU_FACTOR (2500.0f / 32768.0f)

#define BHY2_MAG_WU_PARSER parse_3axis_s16
#define BHY2_MAG_WU_LENGTH 3
#define BHY2_MAG_WU_FACTOR (2500.0f / 32768.0f)

#define BHY2_GYRO_BIAS_PARSER parse_3axis_s16
#define BHY2_GYRO_BIAS_LENGTH 3
#define BHY2_GYRO_BIAS_FACTOR (2000.0f / 32768.0f)

#define BHY2_GYRO_BIAS_WU_PARSER parse_3axis_s16
#define BHY2_GYRO_BIAS_WU_LENGTH 3
#define BHY2_GYRO_BIAS_WU_FACTOR (2000.0f / 32768.0f)

#define BHY2_GYRO_PARSER parse_3axis_s16
#define BHY2_GYRO_LENGTH 3
#define BHY2_GYRO_FACTOR (2000.0f / 32768.0f)

#define BHY2_GYRO_PASS_PARSER parse_3axis_s16
#define BHY2_GYRO_PASS_LENGTH 3
#define BHY2_GYRO_PASS_FACTOR (2000.0f / 32768.0f)

#define BHY2_GYRO_RAW_PARSER parse_3axis_s16
#define BHY2_GYRO_RAW_LENGTH 3
#define BHY2_GYRO_RAW_FACTOR (2000.0f / 32768.0f)

#define BHY2_GYRO_RAW_WU_PARSER parse_3axis_s16
#define BHY2_GYRO_RAW_WU_LENGTH 3
#define BHY2_GYRO_RAW_WU_FACTOR (2000.0f / 32768.0f)

#define BHY2_GYRO_WU_PARSER parse_3axis_s16
#define BHY2_GYRO_WU_LENGTH 3
#define BHY2_GYRO_WU_FACTOR (2000.0f / 32768.0f)

#define BHY2_ORI_PARSER parse_euler
#define BHY2_ORI_LENGTH 3
#define BHY2_ORI_FACTOR (360.0f / 32768.0f)

#define BHY2_ORI_WU_PARSER parse_euler
#define BHY2_ORI_WU_LENGTH 3
#define BHY2_ORI_WU_FACTOR (360.0f / 32768.0f)

#define BHY2_RV_PARSER parse_quaternion
#define BHY2_RV_LENGTH 5
#define BHY2_RV_FACTOR (1.0f / 16384.0f)

#define BHY2_RV_WU_PARSER parse_quaternion
#define BHY2_RV_WU_LENGTH 5
#define BHY2_RV_WU_FACTOR (1.0f / 16384.0f)

#define BHY2_GAMERV_PARSER parse_quaternion
#define BHY2_GAMERV_LENGTH 5
#define BHY2_GAMERV_FACTOR (1.0f / 16384.0f)

#define BHY2_GAMERV_WU_PARSER parse_quaternion
#define BHY2_GAMERV_WU_LENGTH 5
#define BHY2_GAMERV_WU_FACTOR (1.0f / 16384.0f)

#define BHY2_GEORV_PARSER parse_quaternion
#define BHY2_GEORV_LENGTH 5
#define BHY2_GEORV_FACTOR (1.0f / 16384.0f)

#define BHY2_GEORV_WU_PARSER parse_quaternion
#define BHY2_GEORV_WU_LENGTH 5
#define BHY2_GEORV_WU_FACTOR (1.0f / 16384.0f)

#define BHY2_DEVICE_ORI_PARSER parse_scalar_u8
#define BHY2_DEVICE_ORI_LENGTH 1
#define BHY2_DEVICE_ORI_FACTOR 0.0f

#define BHY2_DEVICE_ORI_WU_PARSER parse_scalar_u8
#define BHY2_DEVICE_ORI_WU_LENGTH 1
#define BHY2_DEVICE_ORI_WU_FACTOR 0.0f

#define BHY2_HUM_PARSER parse_scalar_u8
#define BHY2_HUM_LENGTH 1
#define BHY2_HUM_FACTOR 1.0f

#define BHY2_HUM_WU_PARSER parse_scalar_u8
#define BHY2_HUM_WU_LENGTH 1
#define BHY2_HUM_WU_FACTOR 1.0f

#define BHY2_PROX_PARSER parse_scalar_u8
#define BHY2_PROX_LENGTH 1
#define BHY2_PROX_FACTOR 1.0f

#define BHY2_PROX_WU_PARSER parse_scalar_u8
#define BHY2_PROX_WU_LENGTH 1
#define BHY2_PROX_WU_FACTOR 1.0f

#define BHY2_EXCAMERA_PARSER parse_scalar_u8
#define BHY2_EXCAMERA_LENGTH 1
#define BHY2_EXCAMERA_FACTOR 1.0f

#define BHY2_GAS_PARSER parse_scalar_u32
#define BHY2_GAS_LENGTH 1
#define BHY2_GAS_FACTOR 1.0f

#define BHY2_GAS_WU_PARSER parse_scalar_u32
#define BHY2_GAS_WU_LENGTH 1
#define BHY2_GAS_WU_FACTOR 1.0f

#define BHY2_STC_HW_PARSER parse_scalar_u32
#define BHY2_STC_HW_LENGTH 1
#define BHY2_STC_HW_FACTOR 1.0

#define BHY2_STC_HW_WU_PARSER parse_scalar_u32
#define BHY2_STC_HW_WU_LENGTH 1
#define BHY2_STC_HW_WU_FACTOR 1.0

#define BHY2_STC_PARSER parse_scalar_u32
#define BHY2_STC_LENGTH 1
#define BHY2_STC_FACTOR 1.0

#define BHY2_STC_WU_PARSER parse_scalar_u32
#define BHY2_STC_WU_LENGTH 1
#define BHY2_STC_WU_FACTOR 1.0

#define BHY2_BARO_PARSER parse_u24_as_float
#define BHY2_BARO_LENGTH 1
#define BHY2_BARO_FACTOR (100.0f / 128.0f)

#define BHY2_BARO_WU_PARSER parse_u24_as_float
#define BHY2_BARO_WU_LENGTH 1
#define BHY2_BARO_WU_FACTOR (100.0f / 128.0f)

#define BHY2_LIGHT_PARSER parse_s16_as_float
#define BHY2_LIGHT_LENGTH 1
#define BHY2_LIGHT_FACTOR (10000.0f / 216.0f)

#define BHY2_LIGHT_WU_PARSER parse_s16_as_float
#define BHY2_LIGHT_WU_LENGTH 1
#define BHY2_LIGHT_WU_FACTOR (10000.0f / 216.0f)

#define BHY2_TEMP_PARSER parse_s16_as_float
#define BHY2_TEMP_LENGTH 1
#define BHY2_TEMP_FACTOR (1.0f / 100.0f)

#define BHY2_TEMP_WU_PARSER parse_s16_as_float
#define BHY2_TEMP_WU_LENGTH 1
#define BHY2_TEMP_WU_FACTOR (1.0f / 100.0f)

#define BHY2_ANY_MOTION_PARSER parse_scalar_event
#define BHY2_ANY_MOTION_LENGTH 1
#define BHY2_ANY_MOTION_FACTOR 0.0f

#define BHY2_ANY_MOTION_WU_PARSER parse_scalar_event
#define BHY2_ANY_MOTION_WU_LENGTH 1
#define BHY2_ANY_MOTION_WU_FACTOR 0.0f

#define BHY2_GLANCE_GESTURE_PARSER parse_scalar_event
#define BHY2_GLANCE_GESTURE_LENGTH 1
#define BHY2_GLANCE_GESTURE_FACTOR 0.0f

#define BHY2_MOTION_DET_PARSER parse_scalar_event
#define BHY2_MOTION_DET_LENGTH 1
#define BHY2_MOTION_DET_FACTOR 0.0f

#define BHY2_PICKUP_GESTURE_PARSER parse_scalar_event
#define BHY2_PICKUP_GESTURE_LENGTH 1
#define BHY2_PICKUP_GESTURE_FACTOR 0.0f

#define BHY2_SIG_HW_PARSER parse_scalar_event
#define BHY2_SIG_HW_LENGTH 1
#define BHY2_SIG_HW_FACTOR 0.0f

#define BHY2_SIG_HW_WU_PARSER parse_scalar_event
#define BHY2_SIG_HW_WU_LENGTH 1
#define BHY2_SIG_HW_WU_FACTOR 0.0f

#define BHY2_SIG_PARSER parse_scalar_event
#define BHY2_SIG_LENGTH 1
#define BHY2_SIG_FACTOR 0.0f

#define BHY2_STATIONARY_DET_PARSER parse_scalar_event
#define BHY2_STATIONARY_DET_LENGTH 1
#define BHY2_STATIONARY_DET_FACTOR 0.0f

#define BHY2_STD_HW_PARSER parse_scalar_event
#define BHY2_STD_HW_LENGTH 1
#define BHY2_STD_HW_FACTOR 0.0f

#define BHY2_STD_HW_WU_PARSER parse_scalar_event
#define BHY2_STD_HW_WU_LENGTH 1
#define BHY2_STD_HW_WU_FACTOR 0.0f

#define BHY2_STD_PARSER parse_scalar_event
#define BHY2_STD_LENGTH 1
#define BHY2_STD_FACTOR 0.0f

#define BHY2_STD_WU_PARSER parse_scalar_event
#define BHY2_STD_WU_LENGTH 1
#define BHY2_STD_WU_FACTOR 0.0f

#define BHY2_TILT_DETECTOR_PARSER parse_scalar_event
#define BHY2_TILT_DETECTOR_LENGTH 1
#define BHY2_TILT_DETECTOR_FACTOR 0.0f

#define BHY2_WAKE_GESTURE_PARSER parse_scalar_event
#define BHY2_WAKE_GESTURE_LENGTH 1
#define BHY2_WAKE_GESTURE_FACTOR 0.0f

#define BHY2_WRIST_TILT_GESTURE_PARSER parse_scalar_event
#define BHY2_WRIST_TILT_GESTURE_LENGTH 1
#define BHY2_WRIST_TILT_GESTURE_FACTOR 0.0f

#define BHY2_AR_PARSER parse_scalar_u16
#define BHY2_AR_LENGTH 1
#define BHY2_AR_FACTOR 0.0f

#define BHY2_BSEC_PARSER parse_bsec
#define BHY2_BSEC_LENGTH 8
#define BHY2_BSEC_FACTOR 0.0f

#define BHY2_BSEC_LEGACY_PARSER parse_bsec_legacy
#define BHY2_BSEC_LEGACY_LENGTH 8
#define BHY2_BSEC_LEGACY_FACTOR 0.0f

#define BHY2_BSEC2_PARSER parse_bsec2
#define BHY2_BSEC2_LENGTH 5
#define BHY2_BSEC2_FACTOR 0.0f

#define BHY2_BSEC2_COLLECTOR_PARSER parse_bsec2_collector
#define BHY2_BSEC2_COLLECTOR_LENGTH 6
#define BHY2_BSEC2_COLLECTOR_FACTOR 0.0f

