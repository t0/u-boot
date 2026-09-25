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

#define FRU_HDR_LEN			8
#define FRU_AREA_UNIT		8
#define FRU_AREA_HDR_LEN	2
#define FRU_VERSION_MASK	0x0f
#define FRU_VERSION_1		1

/* [7:6]: type code [5:0]: length in bytes */
#define FRU_TYPELEN_TYPE_SHIFT	6
#define FRU_TYPELEN_LEN_MASK	0x3f
#define FRU_TYPE_ASCII8			0b11 /* only supported type code here */
#define FRU_TYPELEN_EOF			0xc1

/* Bytes preceding the first type/length field in each area */
#define FRU_CHASSIS_PROLOGUE_LEN	3	/* version, length, chassis type */
#define FRU_BOARD_PROLOGUE_LEN		6	/* version, length, language, date[3] */
#define FRU_PRODUCT_PROLOGUE_LEN	3	/* version, length, language*/

#define T0_SLOT_PREFIX		"slot="
#define T0_REVISION_PREFIX	"revision="

#define FRU_FIELD_LEN	64
#define FRU_MAX_CUSTOM	 4

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

struct fru_custom_fields {
	int count;
	char field[FRU_MAX_CUSTOM][FRU_FIELD_LEN];
};

struct fru_chassis_info {
	char part_number[FRU_FIELD_LEN];
	char serial_number[FRU_FIELD_LEN];
	struct fru_custom_fields custom;
};

struct fru_board_info {
	char manufacturer[FRU_FIELD_LEN];
	char product_name[FRU_FIELD_LEN];
	char serial_number[FRU_FIELD_LEN];
	char part_number[FRU_FIELD_LEN];
	char file_id[FRU_FIELD_LEN];
	struct fru_custom_fields custom;
};

struct fru_product_info {
	char manufacturer[FRU_FIELD_LEN];
	char product_name[FRU_FIELD_LEN];
	char part_number[FRU_FIELD_LEN];
	char product_version[FRU_FIELD_LEN];
	char serial_number[FRU_FIELD_LEN];
	char asset_tag[FRU_FIELD_LEN];
	char file_id[FRU_FIELD_LEN];
	struct fru_custom_fields custom;
};

struct fru_info {
	struct fru_chassis_info chassis_info;
	struct fru_board_info board_info;
	struct fru_product_info product_info;
};

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
 * NUL-terminated string, and returns where the next field starts.
 * Only 8-bit ASCII is copied out. Fields in other encodings yield an
 * empty string, but still advance the pointer. The copy stops at the
 * first non-printable byte (including e.g. newlines). */
static const u8 *fru_parse_field(const u8 *p, const u8 *end, char *out,
				 size_t out_len)
{
	size_t copy = 0;
	u8 type, len;

	out[0] = '\0'; /* terminated, even on early bailout */

	if (!p || p >= end || *p == FRU_TYPELEN_EOF)
		return NULL;

	type = *p >> FRU_TYPELEN_TYPE_SHIFT;
	len = *p & FRU_TYPELEN_LEN_MASK;
	p++;

	if (p + len > end)
		return NULL;

	if (type == FRU_TYPE_ASCII8) {
		for (int i = 0; i < len && copy < out_len - 1; i++) {
			if (p[i] < 0x20 || p[i] > 0x7e)
				break; /* bail on unprintable */
			out[copy++] = p[i];
		}
	}
	out[copy] = '\0';

	return p + len;
}

static void fru_parse_custom(const u8 *p, const u8 *end,
			     struct fru_custom_fields *custom)
{
	custom->count = 0;
	while (custom->count < FRU_MAX_CUSTOM) {
		p = fru_parse_field(p, end, custom->field[custom->count], FRU_FIELD_LEN);
		if (!p)
			break;
		custom->count++;
	}
}

static const char *fru_find_custom(const struct fru_custom_fields *c,
				   const char *prefix)
{
	const size_t plen = strlen(prefix);

	for (int i = 0; i < c->count; i++) {
		if (strncmp(c->field[i], prefix, plen) == 0)
			return c->field[i] + plen;
	}
	return NULL;
}

/*
 * Populates pointers for the start and end of the area at the offset of
 * the fru blob with a given prologue length. *end points at the area
 * checksum, which is one byte past the last field byte.
 */
static int fru_locate_area(const u8 *fru, size_t size, u8 offset,
			   size_t prologue, const u8 **start, const u8 **end)
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

static void fru_parse_chassis(const u8 *area, const u8 *end,
			      struct fru_chassis_info *ci)
{
	const u8 *p = area + FRU_CHASSIS_PROLOGUE_LEN;

	p = fru_parse_field(p, end, ci->part_number, sizeof(ci->part_number));
	p = fru_parse_field(p, end, ci->serial_number, sizeof(ci->serial_number));
	fru_parse_custom(p, end, &ci->custom);
}

static void fru_parse_board(const u8 *area, const u8 *end,
			    struct fru_board_info *bi)
{
	const u8 *p = area + FRU_BOARD_PROLOGUE_LEN;

	p = fru_parse_field(p, end, bi->manufacturer, sizeof(bi->manufacturer));
	p = fru_parse_field(p, end, bi->product_name, sizeof(bi->product_name));
	p = fru_parse_field(p, end, bi->serial_number, sizeof(bi->serial_number));
	p = fru_parse_field(p, end, bi->part_number, sizeof(bi->part_number));
	p = fru_parse_field(p, end, bi->file_id, sizeof(bi->file_id));
	fru_parse_custom(p, end, &bi->custom);
}

static void fru_parse_product(const u8 *area, const u8 *end,
			      struct fru_product_info *pi)
{
	const u8 *p = area + FRU_PRODUCT_PROLOGUE_LEN;

	p = fru_parse_field(p, end, pi->manufacturer, sizeof(pi->manufacturer));
	p = fru_parse_field(p, end, pi->product_name, sizeof(pi->product_name));
	p = fru_parse_field(p, end, pi->part_number, sizeof(pi->part_number));
	p = fru_parse_field(p, end, pi->product_version, sizeof(pi->product_version));
	p = fru_parse_field(p, end, pi->serial_number, sizeof(pi->serial_number));
	p = fru_parse_field(p, end, pi->asset_tag, sizeof(pi->asset_tag));
	p = fru_parse_field(p, end, pi->file_id, sizeof(pi->file_id));
	fru_parse_custom(p, end, &pi->custom);
}

static int fru_parse(const u8 *fru, size_t size, struct fru_info *fru_info)
{
	const struct fru_hdr *hdr = (const struct fru_hdr *)fru;
	const u8 *area, *end;
	int ret;

	if (size < FRU_HDR_LEN)
		return -EINVAL;

	if ((hdr->version & FRU_VERSION_MASK) != FRU_VERSION_1)
		return -EBADMSG;

	if (!fru_checksum_ok(fru, FRU_HDR_LEN))
		return -EBADMSG;

	memset(fru_info, 0, sizeof(*fru_info));

	ret = fru_locate_area(fru, size, hdr->board_offset, FRU_BOARD_PROLOGUE_LEN,
			      &area, &end);
	if (!ret)
		fru_parse_board(area, end, &fru_info->board_info);
	else if (ret != -ENOENT)
		return ret;

	ret = fru_locate_area(fru, size, hdr->chassis_offset, FRU_CHASSIS_PROLOGUE_LEN,
			      &area, &end);
	if (!ret)
		fru_parse_chassis(area, end, &fru_info->chassis_info);
	else if (ret != -ENOENT)
		return ret;

	ret = fru_locate_area(fru, size, hdr->product_offset, FRU_PRODUCT_PROLOGUE_LEN,
			      &area, &end);
	if (!ret)
		fru_parse_product(area, end, &fru_info->product_info);
	else if (ret != -ENOENT)
		return ret;

	return 0;
}

static int fru_read(struct fru_info *f)
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
		ret = fru_parse(buf, size, f);

	free(buf);

	return ret;
}

static void t0_backplane_env_clear(void)
{
	static const char * const t0_backplane_vars[] = {
		"crate_model",
		"crate_serial",
		"crate_slot",
		"backplane_manufacturer",
		"backplane_serial",
		"backplane_model",
		"backplane_revision",
		"crate_manufacturer",
		"crate_revision"
	};

	for (int i = 0; i < ARRAY_SIZE(t0_backplane_vars); i++)
		env_set(t0_backplane_vars[i], NULL);
}

int t0_backplane_init(void)
{
	struct fru_info fru_info;
	const struct fru_chassis_info *chassis;
	const struct fru_board_info *board;
	const struct fru_product_info *product;
	const char *slot;
	ulong slot_num;
	const char *revision;
	int ret;

	t0_backplane_env_clear();

	ret = fru_read(&fru_info);
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

	chassis = &fru_info.chassis_info;
	board = &fru_info.board_info;
	product = &fru_info.product_info;

	env_set("crate_model", chassis->part_number);
	env_set("crate_serial", chassis->serial_number);
	slot = fru_find_custom(&chassis->custom, T0_SLOT_PREFIX);
	if (slot && strict_strtoul(slot, 10, &slot_num) == 0)
		env_set_ulong("crate_slot", slot_num);

	env_set("backplane_manufacturer", board->manufacturer);
	env_set("backplane_serial", board->serial_number);
	env_set("backplane_model", board->part_number);
	revision = fru_find_custom(&board->custom, T0_REVISION_PREFIX);
	if (revision)
		env_set("backplane_revision", revision);

	env_set("crate_manufacturer", product->manufacturer);
	env_set("crate_revision", product->product_version);

	return 0;
}
