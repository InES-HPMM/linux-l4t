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

#include <dt-bindings/memory/tegra-swgroup.h>

#include "../../../arch/arm/mach-tegra/iomap.h"
#include "../../../arch/arm/mach-tegra/board.h"
#include "../../../arch/arm/mach-tegra/common.h"

struct tegra_iommu_mapping {
	dma_addr_t base;
	size_t size;
	int asid;
	struct dma_iommu_mapping *map;
};

static struct dma_iommu_mapping *__tegra_smmu_map_init_dev(struct device *dev,
					   struct tegra_iommu_mapping *info);

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

#ifdef CONFIG_CMA
static void carveout_linear_set(struct device *cma_dev)
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
#endif

static void cma_carveout_linear_set(void)
{
#ifdef CONFIG_CMA
	if (tegra_vpr_resize) {
		carveout_linear_set(&tegra_generic_cma_dev);
		carveout_linear_set(&tegra_vpr_cma_dev);
	}
#endif
}

void tegra_fb_linear_set(struct iommu_linear_map *map)
{
	int i = 0;

	map = tegra_fb_linear_map;

	LINEAR_MAP_ADD(tegra_fb);
	LINEAR_MAP_ADD(tegra_fb2);
	LINEAR_MAP_ADD(tegra_bootloader_fb);
	LINEAR_MAP_ADD(tegra_bootloader_fb2);
	if (!tegra_vpr_resize) {
		LINEAR_MAP_ADD(tegra_vpr);
		LINEAR_MAP_ADD(tegra_carveout);
	}
}
EXPORT_SYMBOL(tegra_fb_linear_set);

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
	{ .name = "540c0000.epp",	.swgids = TEGRA_SWGROUP_BIT(EPP), },
	{ .name = "epp",	.swgids = TEGRA_SWGROUP_BIT(EPP), },
	{ .name = "54200000.dc",	.swgids = TEGRA_SWGROUP_BIT(DC),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "54240000.dc",	.swgids = TEGRA_SWGROUP_BIT(DCB), },
	{ .name = "dc",	.swgids = TEGRA_SWGROUP_BIT(DC) |
				  TEGRA_SWGROUP_BIT(DCB) },
	{ .name = "gr2d",	.swgids = TEGRA_SWGROUP_BIT(G2), },
	{ .name = "gr3d",	.swgids = TEGRA_SWGROUP_BIT(NV) |
					  TEGRA_SWGROUP_BIT(NV2), },
	{ .name = "host1x",	.swgids = TEGRA_SWGROUP_BIT(HC) |
					  TEGRA_SWGROUP_BIT(VDE) |
					  TEGRA_SWGROUP_BIT(EPP) |
					  TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "isp",	.swgids = TEGRA_SWGROUP_BIT(ISP), },
	{ .name = "max77660",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "max8831",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "msenc",	.swgids = TEGRA_SWGROUP_BIT(MSENC), },
	{ .name = "mpe",	.swgids = TEGRA_SWGROUP_BIT(MPE), },
	{ .name = "tegra-aes",	.swgids = TEGRA_SWGROUP_BIT(VDE), },
	{ .name = "nvavp",	.swgids = TEGRA_SWGROUP_BIT(AVPC), },
	{ .name = "sdhci-tegra.0",	.swgids = TEGRA_SWGROUP_BIT(PPCS1) },
	{ .name = "sdhci-tegra.1",	.swgids = TEGRA_SWGROUP_BIT(PPCS1) },
	{ .name = "sdhci-tegra.2",	.swgids = TEGRA_SWGROUP_BIT(PPCS1) },
	{ .name = "sdhci-tegra.3",	.swgids = TEGRA_SWGROUP_BIT(PPCS1) },
	{ .name = "serial8250",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "serial-tegra",      .swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "snd-soc-dummy",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "spdif-dit",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra11-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra11-spi",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra14-i2c",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-ahub",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-dam",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-hda",	.swgids = TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "tegra30-i2s",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-spdif",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-avp-audio",	.swgids = TEGRA_SWGROUP_BIT(AVPC), },
	{ .name = "tegradc.0", .swgids = TEGRA_SWGROUP_BIT(DC),
	  .linear_map = tegra_fb_linear_map},
	{ .name = "tegradc.1", .swgids = TEGRA_SWGROUP_BIT(DCB), },
	{ .name = "tegra_bb",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra_dma",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-ehci",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-fuse",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-i2c",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-carveouts",	.swgids = TEGRA_SWGROUP_BIT(HC) |
						  TEGRA_SWGROUP_BIT(AVPC), },
	{ .name = "tegra-otg",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-pcm-audio",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-rtc",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-sata",	.swgids = TEGRA_SWGROUP_BIT(SATA), },
	{ .name = "tegra-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-snd",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-tzram",	.swgids = TEGRA_SWGROUP_BIT(VDE), },
	{ .name = "tegra_uart",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-udc",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra_usb_modem_power",
				.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tsec",	.swgids = TEGRA_SWGROUP_BIT(TSEC), },
	{ .name = "vi",	.swgids = TEGRA_SWGROUP_BIT(VI), },
	{ .name = "therm_est",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-xhci",	.swgids = TEGRA_SWGROUP_BIT(XUSB_HOST), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = TEGRA_SWGROUP_BIT(PPCS) },
#endif
	{},
};

struct swgid_fixup tegra_swgid_fixup_t124[] = {
	{ .name = "54200000.dc",	.swgids = TEGRA_SWGROUP_BIT(DC), },
	{ .name = "54240000.dc",	.swgids = TEGRA_SWGROUP_BIT(DCB), },
	{ .name = "dc",	.swgids = TEGRA_SWGROUP_BIT(DC) |
				  TEGRA_SWGROUP_BIT(DCB) },
	{ .name = "host1x",	.swgids = TEGRA_SWGROUP_BIT(HC) |
					  TEGRA_SWGROUP_BIT(VDE) |
	  TEGRA_SWGROUP_BIT(EPP) | TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "isp",	.swgids = TEGRA_SWGROUP_BIT(ISP2) |
					  TEGRA_SWGROUP_BIT(ISP2B), },
	{ .name = "max77660",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "max8831",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "msenc",	.swgids = TEGRA_SWGROUP_BIT(MSENC), },
	{ .name = "mpe",	.swgids = TEGRA_SWGROUP_BIT(MPE), },
	{ .name = "tegra-aes",	.swgids = TEGRA_SWGROUP_BIT(VDE), },
	{ .name = "nvavp",	.swgids = TEGRA_SWGROUP_BIT(AVPC) |
					  TEGRA_SWGROUP_BIT(A9AVP), },
	{ .name = "sdhci-tegra.1",	.swgids = TEGRA_SWGROUP_BIT(SDMMC2A) },
	{ .name = "sdhci-tegra.2",	.swgids = TEGRA_SWGROUP_BIT(SDMMC3A) },
	{ .name = "serial8250",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "serial-tegra",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "dtv",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "snd-soc-dummy",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "spdif-dit",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra12-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "spi-tegra114",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra14-i2c",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-ahub",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-dam",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-hda",	.swgids = TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "tegra30-i2s",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-spdif",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-avp-audio",	.swgids = TEGRA_SWGROUP_BIT(AVPC) |
						  TEGRA_SWGROUP_BIT(A9AVP), },
	{ .name = "tegradc.0", .swgids = TEGRA_SWGROUP_BIT(DC) |
					 TEGRA_SWGROUP_BIT(DC12),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "tegradc.1", .swgids = TEGRA_SWGROUP_BIT(DCB),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "tegra_bb",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra_dma",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-ehci",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-fuse",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-i2c",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-carveouts",	.swgids = TEGRA_SWGROUP_BIT(HC) |
						  TEGRA_SWGROUP_BIT(AVPC), },
	/*
	 * PPCS1 selection for USB2 needs AHB_ARBC register program
	 * in warm boot and cold boot paths in BL as it needs
	 * secure write.
	 */
	{ .name = "tegra-otg",	.swgids = TEGRA_SWGROUP_BIT(PPCS1), },
	{ .name = "tegra-pcm-audio",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-rtc",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-sata",	.swgids = TEGRA_SWGROUP_BIT(SATA2), },
	{ .name = "tegra-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-snd",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-tzram",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra_uart",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-udc",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra_usb_modem_power",
				.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tsec",	.swgids = TEGRA_SWGROUP_BIT(TSEC), },
	/* vic must be before vi to prevent incorrect matching */
	{ .name = "vic",	.swgids = TEGRA_SWGROUP_BIT(VIC), },
	{ .name = "vi",	.swgids = TEGRA_SWGROUP_BIT(VI), },
	{ .name = "therm_est",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "gk20a",	.swgids = TEGRA_SWGROUP_BIT(GPU) |
					  TEGRA_SWGROUP_BIT(GPUB), },
	{ .name = "tegra124-apbdma",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-nor",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = TEGRA_SWGROUP_BIT(PPCS) },
#endif
	{ .name = "tegra-xhci",	.swgids = TEGRA_SWGROUP_BIT(XUSB_HOST), },
	{},
};

struct swgid_fixup tegra_swgid_fixup_t210[] = {
	{ .name = "54200000.dc",	.swgids = TEGRA_SWGROUP_BIT(DC), },
	{ .name = "54240000.dc",	.swgids = TEGRA_SWGROUP_BIT(DCB), },
	{ .name = "dc",	.swgids = TEGRA_SWGROUP_BIT(DC) | TEGRA_SWGROUP_BIT(DCB) },
	{ .name = "max77660",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "max8831",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "ape",	.swgids = TEGRA_SWGROUP_BIT(APE), },
	{ .name = "tegra-aes",	.swgids = TEGRA_SWGROUP_BIT(NVDEC), },
	{ .name = "nvavp",	.swgids = TEGRA_SWGROUP_BIT(AVPC), },
	{
		.name = "bpmp",
		.swgids = TEGRA_SWGROUP_BIT(AVPC),
		.linear_map = tegra_bpmp_linear_map
	},
	{ .name = "serial8250",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "serial-tegra",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "snd-soc-dummy",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "spdif-dit",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra21-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(SE) |
	  TEGRA_SWGROUP_BIT(SE1), },
	{ .name = "spi-tegra114",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra14-i2c",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra30-ahub",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra30-dam",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra30-hda",	.swgids = TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "tegra30-i2s",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra30-spdif",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegradc.0", .swgids = TEGRA_SWGROUP_BIT(DC) |
					 TEGRA_SWGROUP_BIT(DCB) |
	 TEGRA_SWGROUP_BIT(DC12), .linear_map = tegra_fb_linear_map, },
	{ .name = "tegradc.1", .swgids = TEGRA_SWGROUP_BIT(DC) |
					 TEGRA_SWGROUP_BIT(DCB) |
	 TEGRA_SWGROUP_BIT(DC12), },
	{ .name = "tegra_bb",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra_dma",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-ehci",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-fuse",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-i2c",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-carveouts",	.swgids = TEGRA_SWGROUP_BIT(HC) |
						  TEGRA_SWGROUP_BIT(AVPC), },
	{ .name = "tegra-otg",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-pcm-audio",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-rtc",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-sata",	.swgids = TEGRA_SWGROUP_BIT(SATA2), },
	{ .name = "tegra-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-snd",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-tzram",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra_uart",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-udc",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra_usb_modem_power",
				.swgids = TEGRA_SWGROUP_BIT(PPCS) |
	  TEGRA_SWGROUP_BIT(PPCS1) | TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "therm_est",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "gk20a",	.swgids = TEGRA_SWGROUP_BIT(GPU) |
					  TEGRA_SWGROUP_BIT(GPUB), },
	{ .name = "tegra124-apbdma",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = TEGRA_SWGROUP_BIT(PPCS) },
#endif
	{ .name = "tegra-xhci",	.swgids = TEGRA_SWGROUP_BIT(XUSB_HOST), },
	{},
};

u64 tegra_smmu_fixup_swgids(struct device *dev, struct iommu_linear_map **map)
{
	const char *s;
	struct swgid_fixup *table;

	if (!dev)
		return SWGIDS_ERROR_CODE;

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

	return SWGIDS_ERROR_CODE;
}
EXPORT_SYMBOL(tegra_smmu_fixup_swgids);

#ifdef CONFIG_PLATFORM_ENABLE_IOMMU

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

/* XXX: Remove this function once all client devices moved to DT */
static struct dma_iommu_mapping *__tegra_smmu_map_init_dev(struct device *dev,
					   struct tegra_iommu_mapping *info)
{
	struct dma_iommu_mapping *map;

	map = arm_iommu_create_mapping(&platform_bus_type,
				       info->base, info->size, 0);
	if (IS_ERR(map)) {
		dev_err(dev, "%s: Failed create IOVA map for ASID[%d]\n",
			__func__, info->asid);
		return NULL;
	}

	BUG_ON(cmpxchg(&info->map, NULL, map));
	dev_info(dev, "Created a new map %p(asid=%d)\n", map, info->asid);
	return map;
}

struct dma_iommu_mapping *tegra_smmu_map_init_dev(struct device *dev,
						  u64 swgids)
{
	int asid;
	struct tegra_iommu_mapping *info;

	BUG_ON(_tegra_smmu_get_asid(swgids) >= ARRAY_SIZE(smmu_default_map));

	asid = _tegra_smmu_get_asid(swgids);
	info = &smmu_default_map[asid];
	info->asid = asid;
	if (!info->size)
		return NULL;

	if (info->map) {
		dev_info(dev, "Use an existing map %p(asid=%d)\n",
			 info->map, info->asid);
		return info->map;
	}

	return __tegra_smmu_map_init_dev(dev, info);
}

#else
static inline void tegra_smmu_map_init(struct platform_device *pdev)
{
}
#endif

static int __init tegra_smmu_init(void)
{
	tegra_bpmp_linear_set();
	cma_carveout_linear_set();
	return 0;
}
postcore_initcall(tegra_smmu_init);
