/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/tegra_smmu.h>
#include <linux/dma-contiguous.h>
#include <linux/tegra-soc.h>
#include <linux/platform_data/tegra_bpmp.h>

#include <asm/dma-iommu.h>

#include <mach/tegra-swgid.h>

#include "../../../arch/arm/mach-tegra/iomap.h"
#include "../../../arch/arm/mach-tegra/board.h"

static struct resource tegra_smmu_resources[] = {
	{
		.name	= "mc",
		.flags	= IORESOURCE_MEM,
		.start	= TEGRA_MC_BASE,
		.end	= TEGRA_MC_BASE + TEGRA_MC_SIZE - 1,
	},
	{
		.name   = "ahbarb",
		.flags  = IORESOURCE_MEM,
		.start  = TEGRA_AHB_ARB_BASE,
		.end    = TEGRA_AHB_ARB_BASE + TEGRA_AHB_ARB_SIZE - 1,
	},
};

struct platform_device tegra_smmu_device = {
	.name		= "tegra_smmu",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tegra_smmu_resources),
	.resource	= tegra_smmu_resources
};

static struct resource tegra_smmu[] = {
	[0] = {
		.start	= TEGRA_SMMU_BASE,
		.end	= TEGRA_SMMU_BASE + TEGRA_SMMU_SIZE - 1,
	},
};

struct resource *tegra_smmu_window(int wnum)
{
	return &tegra_smmu[wnum];
}
EXPORT_SYMBOL(tegra_smmu_window);

int tegra_smmu_window_count(void)
{
	return ARRAY_SIZE(tegra_smmu);
}

static struct iommu_linear_map tegra_fb_linear_map[16]; /* Terminated with 0 */

#ifdef CONFIG_TEGRA_BPMP
static struct iommu_linear_map tegra_bpmp_linear_map[2];
static void tegra_bpmp_linear_set(void)
{
	tegra_bpmp_get_smmu_data(&tegra_bpmp_linear_map[0].start,
			&tegra_bpmp_linear_map[0].size);
}
#else
#define tegra_bpmp_linear_map NULL
static inline void tegra_bpmp_linear_set(void) {}
#endif

#define LINEAR_MAP_ADD(n) \
do { \
	if (n##_start && n##_size) { \
		map[i].start = n##_start; \
		map[i++].size = n##_size; \
	} \
} while (0)

void tegra_fb_linear_set(struct iommu_linear_map *map)
{
	int i = 0;

	map = tegra_fb_linear_map;

	LINEAR_MAP_ADD(tegra_fb);
	LINEAR_MAP_ADD(tegra_fb2);
	LINEAR_MAP_ADD(tegra_bootloader_fb);
	LINEAR_MAP_ADD(tegra_bootloader_fb2);
#ifndef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	LINEAR_MAP_ADD(tegra_vpr);
	LINEAR_MAP_ADD(tegra_carveout);
#endif
}
EXPORT_SYMBOL(tegra_fb_linear_set);

#ifdef CONFIG_CMA
void carveout_linear_set(struct device *cma_dev)
{
	struct dma_contiguous_stats stats;
	struct iommu_linear_map *map = &tegra_fb_linear_map[0];

	if (dma_get_contiguous_stats(cma_dev, &stats))
		return;

	/* get the free slot at end and add carveout entry */
	while (map && map->size)
		map++;
	map->start = stats.base;
	map->size = stats.size;
}
EXPORT_SYMBOL(carveout_linear_set);
#endif

struct swgid_fixup {
	const char * const name;
	u64 swgids;
	struct iommu_linear_map *linear_map;
};

#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
#define DUMMY_DEV_NAME "dummy_dev"
#define DUMMY_DEV_MAX_NAME_SIZE 100
static char dummy_name[DUMMY_DEV_MAX_NAME_SIZE] = DUMMY_DEV_NAME;
#endif

/*
 * FIXME: They should have a DT entry with swgroup IDs.
 */
struct swgid_fixup tegra_swgid_fixup[] = {
	{ .name = "540c0000.epp",	.swgids = SWGID(EPP), },
	{ .name = "epp",	.swgids = SWGID(EPP), },
	{ .name = "54200000.dc",	.swgids = SWGID(DC),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "54240000.dc",	.swgids = SWGID(DCB), },
	{ .name = "dc",	.swgids = SWGID(DC) | SWGID(DCB) },
	{ .name = "gr2d",	.swgids = SWGID(G2), },
	{ .name = "gr3d",	.swgids = SWGID(NV) | SWGID(NV2), },
	{ .name = "host1x",	.swgids = SWGID(HC) | SWGID(VDE) |
					  SWGID(EPP) | SWGID(HDA), },
	{ .name = "isp",	.swgids = SWGID(ISP), },
	{ .name = "max77660",	.swgids = SWGID(PPCS), },
	{ .name = "max8831",	.swgids = SWGID(PPCS), },
	{ .name = "msenc",	.swgids = SWGID(MSENC), },
	{ .name = "mpe",	.swgids = SWGID(MPE), },
	{ .name = "tegra-aes",	.swgids = SWGID(VDE), },
	{ .name = "nvavp",	.swgids = SWGID(AVPC), },
	{ .name = "sdhci-tegra.0",	.swgids = SWGID(PPCS1) },
	{ .name = "sdhci-tegra.1",	.swgids = SWGID(PPCS1) },
	{ .name = "sdhci-tegra.2",	.swgids = SWGID(PPCS1) },
	{ .name = "sdhci-tegra.3",	.swgids = SWGID(PPCS1) },
	{ .name = "serial8250",	.swgids = SWGID(PPCS), },
	{ .name = "serial-tegra",      .swgids = SWGID(PPCS), },
	{ .name = "snd-soc-dummy",	.swgids = SWGID(PPCS), },
	{ .name = "spdif-dit",	.swgids = SWGID(PPCS), },
	{ .name = "tegra11-se",	.swgids = SWGID(PPCS), },
	{ .name = "tegra11-spi",	.swgids = SWGID(PPCS), },
	{ .name = "tegra14-i2c",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-ahub",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-dam",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-hda",	.swgids = SWGID(HDA), },
	{ .name = "tegra30-i2s",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-spdif",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-avp-audio",	.swgids = SWGID(AVPC), },
	{ .name = "tegradc.0", .swgids = SWGID(DC),
	  .linear_map = tegra_fb_linear_map},
	{ .name = "tegradc.1", .swgids = SWGID(DCB), },
	{ .name = "tegra_bb",	.swgids = SWGID(PPCS), },
	{ .name = "tegra_dma",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-ehci",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-fuse",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-i2c",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-carveouts",	.swgids = SWGID(HC) | SWGID(AVPC), },
	{ .name = "tegra-otg",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-pcm-audio",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-rtc",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-sata",	.swgids = SWGID(SATA), },
	{ .name = "tegra-se",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-snd",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-tzram",	.swgids = SWGID(VDE), },
	{ .name = "tegra_uart",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-udc",	.swgids = SWGID(PPCS), },
	{ .name = "tegra_usb_modem_power",	.swgids = SWGID(PPCS), },
	{ .name = "tsec",	.swgids = SWGID(TSEC), },
	{ .name = "vi",	.swgids = SWGID(VI), },
	{ .name = "therm_est",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-xhci",	.swgids = SWGID(XUSB_HOST), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = SWGID(PPCS) },
#endif
	{},
};

struct swgid_fixup tegra_swgid_fixup_t124[] = {
	{ .name = "54200000.dc",	.swgids = SWGID(DC), },
	{ .name = "54240000.dc",	.swgids = SWGID(DCB), },
	{ .name = "dc",	.swgids = SWGID(DC) | SWGID(DCB) },
	{ .name = "host1x",	.swgids = SWGID(HC) | SWGID(VDE) |
	  SWGID(EPP) | SWGID(HDA), },
	{ .name = "isp",	.swgids = SWGID(ISP2) | SWGID(ISP2B), },
	{ .name = "max77660",	.swgids = SWGID(PPCS), },
	{ .name = "max8831",	.swgids = SWGID(PPCS), },
	{ .name = "msenc",	.swgids = SWGID(MSENC), },
	{ .name = "mpe",	.swgids = SWGID(MPE), },
	{ .name = "tegra-aes",	.swgids = SWGID(VDE), },
	{ .name = "nvavp",	.swgids = SWGID(AVPC) | SWGID(A9AVP), },
	{ .name = "sdhci-tegra.1",	.swgids = SWGID(SDMMC2A) },
	{ .name = "sdhci-tegra.2",	.swgids = SWGID(SDMMC3A) },
	{ .name = "serial8250",	.swgids = SWGID(PPCS), },
	{ .name = "serial-tegra",	.swgids = SWGID(PPCS), },
	{ .name = "dtv",	.swgids = SWGID(PPCS), },
	{ .name = "snd-soc-dummy",	.swgids = SWGID(PPCS), },
	{ .name = "spdif-dit",	.swgids = SWGID(PPCS), },
	{ .name = "tegra12-se",	.swgids = SWGID(PPCS), },
	{ .name = "spi-tegra114",	.swgids = SWGID(PPCS), },
	{ .name = "tegra14-i2c",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-ahub",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-dam",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-hda",	.swgids = SWGID(HDA), },
	{ .name = "tegra30-i2s",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-spdif",	.swgids = SWGID(PPCS), },
	{ .name = "tegra30-avp-audio",	.swgids = SWGID(AVPC) | SWGID(A9AVP), },
	{ .name = "tegradc.0", .swgids = SWGID(DC) | SWGID(DC12),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "tegradc.1", .swgids = SWGID(DCB),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "tegra_bb",	.swgids = SWGID(PPCS), },
	{ .name = "tegra_dma",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-ehci",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-fuse",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-i2c",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-carveouts",	.swgids = SWGID(HC) | SWGID(AVPC), },
	/*
	 * PPCS1 selection for USB2 needs AHB_ARBC register program
	 * in warm boot and cold boot paths in BL as it needs
	 * secure write.
	 */
	{ .name = "tegra-otg",	.swgids = SWGID(PPCS1), },
	{ .name = "tegra-pcm-audio",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-rtc",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-sata",	.swgids = SWGID(SATA2), },
	{ .name = "tegra-se",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-snd",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-tzram",	.swgids = SWGID(PPCS), },
	{ .name = "tegra_uart",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-udc",	.swgids = SWGID(PPCS), },
	{ .name = "tegra_usb_modem_power",	.swgids = SWGID(PPCS), },
	{ .name = "tsec",	.swgids = SWGID(TSEC), },
	/* vic must be before vi to prevent incorrect matching */
	{ .name = "vic",	.swgids = SWGID(VIC), },
	{ .name = "vi",	.swgids = SWGID(VI), },
	{ .name = "therm_est",	.swgids = SWGID(PPCS), },
	{ .name = "gk20a",	.swgids = SWGID(GPU) | SWGID(GPUB), },
	{ .name = "tegra124-apbdma",	.swgids = SWGID(PPCS), },
	{ .name = "tegra-nor",	.swgids = SWGID(PPCS), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = SWGID(PPCS) },
#endif
	{ .name = "tegra-xhci",	.swgids = SWGID(XUSB_HOST), },
	{},
};

struct swgid_fixup tegra_swgid_fixup_t210[] = {
	{ .name = "54200000.dc",	.swgids = SWGID(DC), },
	{ .name = "54240000.dc",	.swgids = SWGID(DCB), },
	{ .name = "dc",	.swgids = SWGID(DC) | SWGID(DCB) },
	{ .name = "max77660",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "max8831",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "ape",	.swgids = SWGID(APE), },
	{ .name = "tegra-aes",	.swgids = SWGID(NVDEC), },
	{ .name = "nvavp",	.swgids = SWGID(AVPC), },
	{
		.name = "bpmp",
		.swgids = SWGID(AVPC),
		.linear_map = tegra_bpmp_linear_map
	},
	{ .name = "sdhci-tegra.0",	.swgids = SWGID(SDMMC1A) },
	{ .name = "sdhci-tegra.1",	.swgids = SWGID(SDMMC2A) },
	{ .name = "sdhci-tegra.2",	.swgids = SWGID(SDMMC3A) },
	{ .name = "sdhci-tegra.3",	.swgids = SWGID(SDMMC4A) },
	{ .name = "serial8250",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "serial-tegra",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "snd-soc-dummy",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "spdif-dit",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra21-se",	.swgids = SWGID(PPCS) | SWGID(SE) |
	  SWGID(SE1), },
	{ .name = "spi-tegra114",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra14-i2c",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra30-ahub",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra30-dam",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra30-hda",	.swgids = SWGID(HDA), },
	{ .name = "tegra30-i2s",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra30-spdif",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegradc.0", .swgids = SWGID(DC) | SWGID(DCB) |
	 SWGID(DC12), .linear_map = tegra_fb_linear_map, },
	{ .name = "tegradc.1", .swgids = SWGID(DC) | SWGID(DCB) |
	 SWGID(DC12), },
	{ .name = "tegra_bb",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra_dma",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-ehci",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-fuse",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-i2c",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-carveouts",	.swgids = SWGID(HC) | SWGID(AVPC), },
	{ .name = "tegra-otg",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-pcm-audio",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-rtc",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-sata",	.swgids = SWGID(SATA2), },
	{ .name = "tegra-se",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-snd",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-tzram",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra_uart",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra-udc",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "tegra_usb_modem_power",	.swgids = SWGID(PPCS) |
	  SWGID(PPCS1) | SWGID(PPCS2), },
	{ .name = "therm_est",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
	{ .name = "gk20a",	.swgids = SWGID(GPU) | SWGID(GPUB), },
	{ .name = "tegra124-apbdma",	.swgids = SWGID(PPCS) | SWGID(PPCS1) |
	  SWGID(PPCS2), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = SWGID(PPCS) },
#endif
	{ .name = "tegra-xhci",	.swgids = SWGID(XUSB_HOST), },
	{ .name = "vi",		.swgids = SWGID(VI) },
	{},
};

u64 tegra_smmu_fixup_swgids(struct device *dev, struct iommu_linear_map **map)
{
	const char *s;
	struct swgid_fixup *table;

	if (!dev)
		return 0;

	switch (tegra_get_chipid()) {
	case TEGRA_CHIPID_TEGRA12:
	case TEGRA_CHIPID_TEGRA13:
		table = tegra_swgid_fixup_t124;
		break;
	case TEGRA_CHIPID_TEGRA21:
		table = tegra_swgid_fixup_t210;
		break;
	default:
		table = tegra_swgid_fixup;
		break;
	}

	while ((s = table->name) != NULL) {
		if (!strncmp(s, dev_name(dev), strlen(s))) {
			if (map)
				*map = table->linear_map;

			return table->swgids;
		}
		table++;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_smmu_fixup_swgids);

#ifdef CONFIG_PLATFORM_ENABLE_IOMMU

struct tegra_iommu_mapping {
	dma_addr_t base;
	size_t size;
	struct dma_iommu_mapping *map;
};

static struct tegra_iommu_mapping smmu_default_map[] = {
	[SYSTEM_DEFAULT] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
	[SYSTEM_PROTECTED] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
	[PPCS1_ASID] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
	[SYSTEM_DC] = {0x10000, (u32)~0},
	[SYSTEM_DCB] = {0x10000, (u32)~0},
	/* Non-zero base to account for gk20a driver's assumptions */
	[SYSTEM_GK20A] = {0x100000, (u32)~0},
	[SDMMC1A_ASID] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
	[SDMMC2A_ASID] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
	[SDMMC3A_ASID] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
	[SDMMC4A_ASID] = {TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE},
#if defined(CONFIG_ARCH_TEGRA_APE)
	[SYSTEM_ADSP] = {TEGRA_APE_DRAM_MAP2_BASE, TEGRA_APE_DRAM_MAP2_SIZE},
#endif
};

static void tegra_smmu_map_init(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(smmu_default_map); i++) {
		struct tegra_iommu_mapping *m = &smmu_default_map[i];
		struct dma_iommu_mapping *map;
		int order = 0;

		if (IS_ENABLED(CONFIG_TEGRA_ERRATA_1053704))
			order = get_order(SZ_16K);

		map = arm_iommu_create_mapping(&platform_bus_type,
					       m->base, m->size, 0);

		if (IS_ERR(map))
			dev_err(&pdev->dev,
				"Failed create IOVA map for ASID[%d]\n", i);

		m->map = map;
	}
}

void tegra_smmu_map_misc_device(struct device *dev)
{
	struct dma_iommu_mapping *map = smmu_default_map[SYSTEM_PROTECTED].map;
	if (!strncmp(dummy_name, DUMMY_DEV_NAME, strlen(dummy_name))) {
		strncpy(dummy_name, dev_name(dev),
			DUMMY_DEV_MAX_NAME_SIZE);
		arm_iommu_attach_device(dev, map);
		dev_info(dev, "Mapped the misc device\n");
		return;
	}
	dev_err(dev, "Can't Map device\n");
}
EXPORT_SYMBOL(tegra_smmu_map_misc_device);

void tegra_smmu_unmap_misc_device(struct device *dev)
{
	if (!strncmp(dummy_name, dev_name(dev), strlen(dummy_name))) {
		arm_iommu_detach_device(dev);
		strncpy(dummy_name, DUMMY_DEV_NAME,
			DUMMY_DEV_MAX_NAME_SIZE);
		dev_info(dev, "Un-mapped the misc device\n");
		return;
	}
	dev_err(dev, "Can't Unmap device\n");
}
EXPORT_SYMBOL(tegra_smmu_unmap_misc_device);

int tegra_smmu_get_asid(struct device *dev)
{
	u64 swgids = tegra_smmu_fixup_swgids(dev, NULL);
	return _tegra_smmu_get_asid(swgids);
}

struct dma_iommu_mapping *tegra_smmu_get_map(struct device *dev, u64 swgids)
{
	if (!swgids)
		swgids = tegra_smmu_fixup_swgids(dev, NULL);

	if (!swgids)
		return NULL;

	return smmu_default_map[_tegra_smmu_get_asid(swgids)].map;
}
#else
static inline void tegra_smmu_map_init(struct platform_device *pdev)
{
}
#endif

static int __init tegra_smmu_init(void)
{
	tegra_bpmp_linear_set();
	platform_device_register(&tegra_smmu_device);
	tegra_smmu_map_init(&tegra_smmu_device);
	return 0;
}
postcore_initcall(tegra_smmu_init);
