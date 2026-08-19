// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2026 t0.technology, Inc.
 */

#include <dm.h>
#include <env.h>
#include <i2c.h>
#include <i2c_eeprom.h>
#include <log.h>
#include <malloc.h>
#include <vsprintf.h>
#include <dm/uclass.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "t0_backplane.h"

#define FRU_HDR_LEN		8
#define FRU_AREA_UNIT		8
#define FRU_AREA_HDR_LEN	2	/* an area's own version and length bytes */
#define FRU_VERSION_MASK	0x0f
#define FRU_VERSION_1		1

/* Type/length byte: type in bits 7:6, length in bits 5:0 */
#define FRU_TYPELEN_TYPE_SHIFT	6
#define FRU_TYPELEN_LEN_MASK	0x3f
#define FRU_TYPE_ASCII8		3
#define FRU_TYPELEN_EOF		0xc1

/* Bytes preceding the first type/length field in each area */
#define FRU_CHASSIS_PROLOGUE_LEN	3	/* version, length, chassis type */
#define FRU_BOARD_PROLOGUE_LEN	6	/* version, length, language, date[3] */

#define T0_SLOT_PREFIX		"slot="

/* offsets are multiples of 8 bytes */
struct fru_hdr {
	u8 version;
	u8 internal_offset;
	u8 chassis_offset;
	u8 board_offset;
	u8 product_offset;
	u8 multirec_offset;
	u8 padding;
	u8 checksum;
} __packed;

static struct t0_backplane_desc backplane;

/*
 * FRU regions (header and areas) carry a "zero checksum"
 * The sum of every byte including the trailing checksum byte
 * modulo 256 is 0.
 */
static bool fru_checksum_ok(const u8 *p, size_t len)
{
	u8 sum = 0;

	while (len--)
		sum += *p++;

	return !sum;
}

/*
 * Consumes one type/length-encoded field, copying it out as a
 * NULL-terminated string, and returns where the next field starts.
 * Only 8-bit ASCII is copied out. Fields in other encodings yield an
 * empty string, but still advance the pointer.
 */
static const u8 *fru_field(const u8 *p, const u8 *end, char *out,
			   size_t out_len)
{
	size_t copy = 0;
	u8 type, len;

	if (!p || p >= end || *p == FRU_TYPELEN_EOF)
		return NULL;

	type = *p >> FRU_TYPELEN_TYPE_SHIFT;
	len = *p & FRU_TYPELEN_LEN_MASK;
	p++;

	if (p + len > end)
		return NULL;

	if (type == FRU_TYPE_ASCII8)
		copy = min_t(size_t, len, out_len - 1);

	memcpy(out, p, copy);
	out[copy] = '\0';

	return p + len;
}

static int fru_locate_area(const u8 *fru, size_t size, u8 offset, size_t prologue,
		    const u8 **start, const u8 **end)
{
	const u8 *area;
	size_t len;

	if (!offset)
		return -ENOENT;

	/* ensure that at least the area version and length fields exist */
	if ((size_t)offset * FRU_AREA_UNIT + FRU_AREA_HDR_LEN > size)
		return -EINVAL;

	area = fru + (size_t)offset * FRU_AREA_UNIT;
	len = (size_t)area[1] * FRU_AREA_UNIT;

	/* Must hold its prologue and a checksum, and stay inside the image */
	if (len < prologue + 1 || (size_t)(area - fru) + len > size)
		return -EINVAL;

	if (!fru_checksum_ok(area, len))
		return -EBADMSG;

	*start = area;
	*end = area + len - 1;

	return 0;
}

static void t0_parse_chassis(const u8 *area, const u8 *end,
			     struct t0_backplane_desc *d)
{
	char custom[T0_BP_FIELD_LEN];
	const u8 *p;

	d->chassis_type = area[2];

	p = area + FRU_CHASSIS_PROLOGUE_LEN;
	p = fru_field(p, end, d->crate_part, sizeof(d->crate_part));
	p = fru_field(p, end, d->crate_serial, sizeof(d->crate_serial));

	while ((p = fru_field(p, end, custom, sizeof(custom)))) {
		if (!strncmp(custom, T0_SLOT_PREFIX, strlen(T0_SLOT_PREFIX))) {
			d->slot = dectoul(custom + strlen(T0_SLOT_PREFIX), NULL);
			break;
		}
	}

	d->crate_valid = true;
}

static void t0_parse_board(const u8 *area, const u8 *end,
			   struct t0_backplane_desc *d)
{
	const u8 *p = area + FRU_BOARD_PROLOGUE_LEN;

	p = fru_field(p, end, d->manufacturer, sizeof(d->manufacturer));
	p = fru_field(p, end, d->product, sizeof(d->product));
	p = fru_field(p, end, d->serial, sizeof(d->serial));
	p = fru_field(p, end, d->part, sizeof(d->part));
}

static int t0_backplane_parse(const u8 *fru, size_t size,
			      struct t0_backplane_desc *d)
{
	const struct fru_hdr *hdr = (const struct fru_hdr *)fru;
	const u8 *area, *end;

	if (size < FRU_HDR_LEN)
		return -EINVAL;

	if ((hdr->version & FRU_VERSION_MASK) != FRU_VERSION_1)
		return -EBADMSG;

	if (!fru_checksum_ok(fru, FRU_HDR_LEN))
		return -EBADMSG;

	memset(d, 0, sizeof(*d));
	d->slot = -1;

	if (fru_locate_area(fru, size, hdr->board_offset, FRU_BOARD_PROLOGUE_LEN,
			&area, &end))
		return -EBADMSG;

	t0_parse_board(area, end, d);

	if (fru_locate_area(fru, size, hdr->chassis_offset, FRU_CHASSIS_PROLOGUE_LEN,
			&area, &end) == 0)
		t0_parse_chassis(area, end, d);
	else
		debug("%s: no usable chassis area\n", __func__);

	return 0;
}

static int t0_backplane_read(struct t0_backplane_desc *d)
{
	struct udevice *dev;
	ofnode node;
	u8 *buf;
	int size, ret;

	node = ofnode_get_aliases_node("backplane0");
	if (!ofnode_valid(node))
		return -ENOENT;

	ret = uclass_get_device_by_ofnode(UCLASS_I2C_EEPROM, node, &dev);
	if (ret)
		return ret;

	size = i2c_eeprom_size(dev);
	if (size <= 0)
		return size < 0 ? size : -EINVAL;

	buf = calloc(1, size);
	if (!buf)
		return -ENOMEM;

	ret = dm_i2c_read(dev, 0, buf, size);
	if (!ret)
		ret = t0_backplane_parse(buf, size, d);

	free(buf);

	return ret;
}

/*
 * The environment may have been saved while this card sat in a
 * different slot or a different crate.
 */
static void t0_backplane_env_clear(void)
{
	int i;

	static const char * const t0_backplane_vars[] = {
		"backplane_manufacturer",
		"backplane_product",
		"backplane_serial",
		"backplane_part",
		"backplane_slot",
		"crate_type",
		"crate_part",
		"crate_serial",
	};

	for (i = 0; i < ARRAY_SIZE(t0_backplane_vars); i++)
		env_set(t0_backplane_vars[i], NULL);
}

int t0_backplane_init(void)
{
	struct t0_backplane_desc *d = &backplane;
	int ret;

	t0_backplane_env_clear();

	ret = t0_backplane_read(d);
	switch (ret) {
	case 0:
		break;
	case -ENOENT:
		debug("Backplane:\tno backplane0 alias in DT\n");
		return 0;
	case -ENODEV:
		printf("Backplane:\tnot present\n");
		return 0;
	default:
		printf("Backplane:\tpresent, FRU unreadable (%d)\n", ret);
		return 0;
	}

	env_set("backplane_manufacturer", d->manufacturer);
	env_set("backplane_product", d->product);
	env_set("backplane_serial", d->serial);
	env_set("backplane_part", d->part);

	if (d->crate_valid) {
		env_set_ulong("crate_type", d->chassis_type);
		env_set("crate_part", d->crate_part);
		env_set("crate_serial", d->crate_serial);
		if (d->slot >= 0)
			env_set_ulong("backplane_slot", d->slot);
	}

	printf("Backplane:\t%s SN %s", d->product, d->serial);
	if (d->crate_valid) {
		printf(", crate SN %s", d->crate_serial);
		if (d->slot >= 0)
			printf(", slot %d", d->slot);
	}
	printf("\n");

	return 0;
}
