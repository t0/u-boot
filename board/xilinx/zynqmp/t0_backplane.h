// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2026 t0.technology, Inc.
 */

#ifndef __T0_BACKPLANE_H
#define __T0_BACKPLANE_H

#include <linux/types.h>

/* A FRU type/length field holds at most 63 bytes; keep 32 plus a terminator */
#define T0_BP_FIELD_LEN		33

struct t0_backplane_desc {
	/* Chassis info area */
	bool crate_valid;
	u8 chassis_type;
	char crate_part[T0_BP_FIELD_LEN];
	char crate_serial[T0_BP_FIELD_LEN];
	int slot;

	/* Board info area */
	char manufacturer[T0_BP_FIELD_LEN];
	char product[T0_BP_FIELD_LEN];
	char serial[T0_BP_FIELD_LEN];
	char part[T0_BP_FIELD_LEN];
};

int t0_backplane_init(void);

#endif /* __T0_BACKPLANE_H */
