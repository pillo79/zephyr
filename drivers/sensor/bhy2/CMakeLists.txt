# Copyright (c) 2018 Bosch Sensortec GmbH
# Copyright (c) 2022, Leonard Pollak
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources(
	bhy2_datastore.c
	bhy2_sensors.c
	bhy2_zephyr.c
	bosch_parser.c
	bosch/bhy2.c
	bosch/bhy2_hif.c
	bosch/bhy2_parse.c
	bosch/bhy2_strings.c
)

zephyr_library_include_directories(.)
