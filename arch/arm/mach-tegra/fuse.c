/*
 * arch/arm/mach-tegra/fuse.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010-2011 NVIDIA Corp.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/tegra-soc.h>
#include <linux/init.h>
#include <linux/string.h>

#include "fuse.h"
#include "iomap.h"
#include "apbio.h"

#define FUSE_SKU_INFO		0x110

#define TEGRA20_FUSE_SPARE_BIT		0x200
#define TEGRA30_FUSE_SPARE_BIT		0x244

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define FUSE_UID_LOW		0x108
#define FUSE_UID_HIGH		0x10c
#else
#define FUSE_VENDOR_CODE	0x200
#define FUSE_VENDOR_CODE_MASK	0xf
#define FUSE_FAB_CODE		0x204
#define FUSE_FAB_CODE_MASK	0x3f
#define FUSE_LOT_CODE_0		0x208
#define FUSE_LOT_CODE_1		0x20c
#define FUSE_WAFER_ID		0x210
#define FUSE_WAFER_ID_MASK	0x3f
#define FUSE_X_COORDINATE	0x214
#define FUSE_X_COORDINATE_MASK	0x1ff
#define FUSE_Y_COORDINATE	0x218
#define FUSE_Y_COORDINATE_MASK	0x1ff
#endif

int tegra_sku_id;
int tegra_cpu_process_id;
int tegra_core_process_id;
int tegra_chip_id;
int tegra_cpu_speedo_id;		/* only exist in Tegra30 and later */
int tegra_soc_speedo_id;
enum tegra_revision tegra_revision;
static unsigned int tegra_chip_major;
static unsigned int tegra_chip_minor;
static unsigned int tegra_chip_netlist;
static unsigned int tegra_chip_patch;
static char *tegra_chip_priv;

static int tegra_fuse_spare_bit;
static void (*tegra_init_speedo_data)(void);

/* The BCT to use at boot is specified by board straps that can be read
 * through a APB misc register and decoded. 2 bits, i.e. 4 possible BCTs.
 */
int tegra_bct_strapping;

#define STRAP_OPT 0x008
#define GMI_AD0 (1 << 4)
#define GMI_AD1 (1 << 5)
#define RAM_ID_MASK (GMI_AD0 | GMI_AD1)
#define RAM_CODE_SHIFT 4

static const char *tegra_revision_name[TEGRA_REVISION_MAX] = {
	[TEGRA_REVISION_UNKNOWN] = "unknown",
	[TEGRA_REVISION_A01]     = "A01",
	[TEGRA_REVISION_A02]     = "A02",
	[TEGRA_REVISION_A03]     = "A03",
	[TEGRA_REVISION_A03p]    = "A03 prime",
	[TEGRA_REVISION_A04]     = "A04",
};

u32 tegra_fuse_readl(unsigned long offset)
{
	return tegra_apb_readl(TEGRA_FUSE_BASE + offset);
}

void tegra_fuse_writel(u32 value, unsigned long offset)
{
	tegra_apb_writel(value, TEGRA_FUSE_BASE + offset);
}

bool tegra_spare_fuse(int bit)
{
	return tegra_fuse_readl(tegra_fuse_spare_bit + bit * 4);
}

static enum tegra_revision tegra_get_revision()
{
	if (tegra_chip_id == TEGRA30) {
		switch (tegra_chip_major) {
		case 0:
			if (tegra_chip_minor != 1)
				return TEGRA_REVISION_UNKNOWN;
			else if (tegra_chip_netlist == 12 && (tegra_chip_patch & 0xf) == 12)
				return TEGRA_REVISION_A01;
			else if (tegra_chip_netlist == 12 && (tegra_chip_patch & 0xf) > 12)
				return TEGRA_REVISION_A02;
			else if (tegra_chip_netlist > 12)
				return TEGRA_REVISION_A02;
			else
				return TEGRA_REVISION_UNKNOWN;
		case 1:
			break;
		default:
			return TEGRA_REVISION_UNKNOWN;
	}

	switch (tegra_chip_minor) {
	case 1:
		BUG_ON(tegra_chip_id == TEGRA20);
		return TEGRA_REVISION_A01;
	case 2:
		return TEGRA_REVISION_A02;
	case 3:
		if (tegra_chip_id == TEGRA20 &&
			(*tegra_chip_priv) == 'p')
			return TEGRA_REVISION_A03p;
		else
			return TEGRA_REVISION_A03;
	case 4:
		return TEGRA_REVISION_A04;
	default:
		return TEGRA_REVISION_UNKNOWN;
	}
}

static void tegra_get_process_id(void)
{
	u32 reg;

	reg = tegra_fuse_readl(tegra_fuse_spare_bit);
	tegra_cpu_process_id = (reg >> 6) & 3;
	reg = tegra_fuse_readl(tegra_fuse_spare_bit);
	tegra_core_process_id = (reg >> 12) & 3;
}

u32 tegra_read_chipid(void)
{
	return readl_relaxed(IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x804);
}

void tegra_init_fuse(void)
{
	u32 id, netlist;

	u32 reg = readl(IO_ADDRESS(TEGRA_CLK_RESET_BASE + 0x48));
	reg |= 1 << 28;
	writel(reg, IO_ADDRESS(TEGRA_CLK_RESET_BASE + 0x48));

	reg = tegra_fuse_readl(FUSE_SKU_INFO);
	tegra_sku_id = reg & 0xFF;

	reg = tegra_apb_readl(TEGRA_APB_MISC_BASE + STRAP_OPT);
	tegra_bct_strapping = (reg & RAM_ID_MASK) >> RAM_CODE_SHIFT;

	if (!tegra_chip_id) {
		id = tegra_read_chipid();
		netlist = readl_relaxed(IO_ADDRESS(TEGRA_APB_MISC_BASE) + 0x860);
		tegra_chip_id = (id >> 8) & 0xff;
		tegra_chip_major = (id >> 4) & 0xf;
		tegra_chip_minor = (id >> 16) & 0xf;
		tegra_chip_netlist = (netlist >> 0) & 0xffff;
		tegra_chip_patch = (netlist >> 16) & 0xffff;

		if (tegra_chip_id == TEGRA20 &&
		    (tegra_spare_fuse(18) || tegra_spare_fuse(19)))
			tegra_chip_priv = "p";
	}

	switch (tegra_chip_id) {
	case TEGRA20:
		tegra_fuse_spare_bit = TEGRA20_FUSE_SPARE_BIT;
		tegra_init_speedo_data = &tegra20_init_speedo_data;
		break;
	case TEGRA30:
		tegra_fuse_spare_bit = TEGRA30_FUSE_SPARE_BIT;
		tegra_init_speedo_data = &tegra30_init_speedo_data;
		break;
	default:
		pr_warn("Tegra: unknown chip id %d\n", tegra_chip_id);
		tegra_fuse_spare_bit = TEGRA20_FUSE_SPARE_BIT;
		tegra_init_speedo_data = &tegra_get_process_id;
	}

	tegra_revision = tegra_get_revision();
#ifndef CONFIG_TEGRA_FPGA_PLATFORM
	tegra_init_speedo_data();
#endif

	pr_info("Tegra Revision: %s SKU: %d CPU Process: %d Core Process: %d "
		"Speedo ID: %d\n",
		tegra_revision_name[tegra_revision],
		tegra_sku_id, tegra_cpu_process_id,
		tegra_core_process_id, tegra_soc_speedo_id);
}

unsigned long long tegra_chip_uid(void)
{
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
	unsigned long long lo, hi;

	lo = tegra_fuse_readl(FUSE_UID_LOW);
	hi = tegra_fuse_readl(FUSE_UID_HIGH);
	return (hi << 32ull) | lo;
#else
	u64 uid = 0ull;
	u32 reg;
	u32 cid;
	u32 vendor;
	u32 fab;
	u32 lot;
	u32 wafer;
	u32 x;
	u32 y;
	u32 i;

	/* This used to be so much easier in prior chips. Unfortunately, there
	   is no one-stop shopping for the unique id anymore. It must be
	   constructed from various bits of information burned into the fuses
	   during the manufacturing process. The 64-bit unique id is formed
	   by concatenating several bit fields. The notation used for the
	   various fields is <fieldname:size_in_bits> with the UID composed
	   thusly:

	   <CID:4><VENDOR:4><FAB:6><LOT:26><WAFER:6><X:9><Y:9>

	   Where:

		Field    Bits  Position Data
		-------  ----  -------- ----------------------------------------
		CID        4     60     Chip id (encoded as zero for T30)
		VENDOR     4     56     Vendor code
		FAB        6     50     FAB code
		LOT       26     24     Lot code (5-digit base-36-coded-decimal,
					re-encoded to 26 bits binary)
		WAFER      6     18     Wafer id
		X          9      9     Wafer X-coordinate
		Y          9      0     Wafer Y-coordinate
		-------  ----
		Total     64
	*/

	/* Get the chip id and encode each chip variant as a unique value. */
	reg = readl(IO_TO_VIRT(TEGRA_APB_MISC_BASE + 0x804));
	reg = (reg & 0xFF00) >> 8;

	switch (reg) {
	case TEGRA30:
		cid = 0;
		break;

	default:
		BUG();
		break;
	}

	vendor = tegra_fuse_readl(FUSE_VENDOR_CODE) & FUSE_VENDOR_CODE_MASK;
	fab = tegra_fuse_readl(FUSE_FAB_CODE) & FUSE_FAB_CODE_MASK;

	/* Lot code must be re-encoded from a 5 digit base-36 'BCD' number
	   to a binary number. */
	lot = 0;
	reg = tegra_fuse_readl(FUSE_LOT_CODE_1) << 2;

	for (i = 0; i < 5; ++i) {
		u32 digit = (reg & 0xFC000000) >> 26;
		BUG_ON(digit >= 36);
		lot *= 36;
		lot += digit;
		reg <<= 6;
	}

	wafer = tegra_fuse_readl(FUSE_WAFER_ID) & FUSE_WAFER_ID_MASK;
	x = tegra_fuse_readl(FUSE_X_COORDINATE) & FUSE_X_COORDINATE_MASK;
	y = tegra_fuse_readl(FUSE_Y_COORDINATE) & FUSE_Y_COORDINATE_MASK;

	uid = ((unsigned long long)cid  << 60ull)
	    | ((unsigned long long)vendor << 56ull)
	    | ((unsigned long long)fab << 50ull)
	    | ((unsigned long long)lot << 24ull)
	    | ((unsigned long long)wafer << 18ull)
	    | ((unsigned long long)x << 9ull)
	    | ((unsigned long long)y << 0ull);
	return uid;
#endif
}
EXPORT_SYMBOL(tegra_chip_uid);

static char chippriv[16]; /* Permanent buffer for private string */
static int __init tegra_bootloader_tegraid(char *str)
{
	u32 id[5];
	int i = 0;
	char *priv = NULL;

	do {
		id[i++] = simple_strtoul(str, &str, 16);
	} while (*str++ && i < ARRAY_SIZE(id));

	if (*(str - 1) == '.') {
		strncpy(chippriv, str, sizeof(chippriv) - 1);
		priv = chippriv;
		if (strlen(str) > sizeof(chippriv) - 1)
			pr_err("### tegraid.priv in kernel arg truncated\n");
	}

	while (i < ARRAY_SIZE(id))
		id[i++] = 0;

	tegra_chip_id = id[0];
	tegra_chip_major = id[1];
	tegra_chip_minor = id[2];
	tegra_chip_netlist = id[3];
	tegra_chip_patch = id[4];
	tegra_chip_priv = priv;
	return 0;
}

/* tegraid=chipid.major.minor.netlist.patch[.priv] */
early_param("tegraid", tegra_bootloader_tegraid);
