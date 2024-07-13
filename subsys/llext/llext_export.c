/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/llext/symbol.h>

#define FORCE_EXPORT_SYM(name) \
	extern void name(void); \
	EXPORT_SYMBOL(name);


EXPORT_SYMBOL(strcpy);
EXPORT_SYMBOL(strncpy);
EXPORT_SYMBOL(strlen);
EXPORT_SYMBOL(strcmp);
EXPORT_SYMBOL(strncmp);
EXPORT_SYMBOL(memcmp);
EXPORT_SYMBOL(memcpy);
EXPORT_SYMBOL(memset);

FORCE_EXPORT_SYM(abort);
FORCE_EXPORT_SYM(cbvprintf);
EXPORT_SYMBOL(free);
FORCE_EXPORT_SYM(rand);
FORCE_EXPORT_SYM(ring_buf_get);
FORCE_EXPORT_SYM(ring_buf_peek);
FORCE_EXPORT_SYM(ring_buf_put);
FORCE_EXPORT_SYM(srand);
FORCE_EXPORT_SYM(sys_clock_cycle_get_32);

#include <zephyr/syscall_export_llext.c>
