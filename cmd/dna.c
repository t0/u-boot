// SPDX-License-Identifier: GPL-2.0+ OR MIT
/*
 * Simple command to fetch the EFUSE DNA registers.
 * These 96 bits are the unique identifier for the
 * xilinx ZynqMP SoC.
 */

#include <command.h>
#include <zynqmp_firmware.h>

#define ZYNQMP_EFUSE_CACHE_DNA_0	0xffcc100c
#define ZYNQMP_EFUSE_CACHE_DNA_1	0xffcc1010
#define ZYNQMP_EFUSE_CACHE_DNA_2	0xffcc1014

static int do_dna(struct cmd_tbl *cmdtp, int flag, int argc,
		  char *const argv[])
{
	u32 dna0, dna1, dna2;
	int ret;

	ret = zynqmp_mmio_read(ZYNQMP_EFUSE_CACHE_DNA_0, &dna0);
	if (!ret)
		ret = zynqmp_mmio_read(ZYNQMP_EFUSE_CACHE_DNA_1, &dna1);
	if (!ret)
		ret = zynqmp_mmio_read(ZYNQMP_EFUSE_CACHE_DNA_2, &dna2);
	if (ret) {
		return CMD_RET_FAILURE;
	}

	printf("%08x%08x%08x\n", dna2, dna1, dna0);

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	dna, 1, 1, do_dna,
	"print ZynqMP eFUSE device DNA",
	"- print the 96-bit unique device identifier"
);
