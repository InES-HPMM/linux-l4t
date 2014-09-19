/*
 * arch/arm/mach-tegra/board-dt-t210.c
 *
 * NVIDIA Tegra210 device tree board support
 *
 * Copyright (C) 2013-2014 NVIDIA Corporation. All rights reserved.
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
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/tegra_nvadsp.h>
#include <linux/tegra-pm.h>

#include <asm/mach/arch.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/nvmap.h>

#include "board.h"
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/common.h>
#include "iomap.h"
#include "board-common.h"
#include "board-t210.h"
#include "board-t210ref.h"

#if defined(CONFIG_TEGRA_NVADSP) && \
		!defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
static struct nvadsp_platform_data nvadsp_plat_data;
#endif

static void __init tegra_t210_reserve(void)
{
#if defined(CONFIG_TEGRA_NVADSP) && \
		!defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	nvadsp_plat_data.co_pa = tegra_reserve_adsp(SZ_32M);
	nvadsp_plat_data.co_size = SZ_32M;
#endif

#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	tegra_reserve(0, SZ_16M + SZ_2M, SZ_4M);
#else
	tegra_reserve(SZ_128M, SZ_16M + SZ_2M, SZ_4M);
#endif
}

struct of_dev_auxdata t210_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC1_BASE,
			"sdhci-tegra.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC2_BASE,
			"sdhci-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC3_BASE,
			"sdhci-tegra.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC4_BASE,
			"sdhci-tegra.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-udc", TEGRA_USB_BASE,
			"tegra-udc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-otg", TEGRA_USB_BASE,
			"tegra-otg", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-host1x", TEGRA_HOST1X_BASE, "host1x",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-gm20b", TEGRA_GK20A_BAR0_BASE,
				"gpu.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvenc", TEGRA_NVENC_BASE, "msenc",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vi", TEGRA_VI_BASE, "vi",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-isp", TEGRA_ISP_BASE, "isp.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-isp", TEGRA_ISPB_BASE, "isp.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-tsec", TEGRA_TSEC_BASE, "tsec",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-tsec", TEGRA_TSECB_BASE, "tsecb",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vic", TEGRA_VIC_BASE, "vic03",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-se", 0x70012000, "tegra21-se",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dtv", 0x7000c300, "dtv", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvdec", TEGRA_NVDEC_BASE, "nvdec",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvjpg", TEGRA_NVJPG_BASE, "nvjpg",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-axbar",  TEGRA_AXBAR_BASE,
				"tegra210-axbar", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-adma",  TEGRA_ADMA_BASE,
				"tegra210-adma", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c000,
				"tegra21-i2c.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c400,
				"tegra21-i2c.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c500,
				"tegra21-i2c.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c700,
				"tegra21-i2c.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000d000,
				"tegra21-i2c.4", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000d100,
				"tegra21-i2c.5", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000d400, "spi-tegra114.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000d600, "spi-tegra114.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000d800, "spi-tegra114.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000da00, "spi-tegra114.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-qspi", 0x70410000, "qspi",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006000, "serial8250.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006040, "serial8250.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006200, "serial8250.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006300, "serial8250.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006000, "serial-tegra.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006040, "serial-tegra.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006200, "serial-tegra.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006300, "serial-tegra.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahci-sata", 0x70027000, "tegra-sata.0",
				NULL),
#if defined(CONFIG_TEGRA_NVADSP) && \
		!defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	OF_DEV_AUXDATA("nvidia,tegra210-adsp", TEGRA_APE_AMC_BASE,
				"tegra210-adsp", &nvadsp_plat_data),
#endif
	OF_DEV_AUXDATA("nvidia,tegra210-adsp-audio", 0, "adsp_audio.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra-bluedroid_pm", 0, "bluedroid_pm",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra-bcmdhd-wlan", 0, "bcm4329_wlan",
				NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 0, "spdif-dit.0", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 1, "spdif-dit.1", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 2, "spdif-dit.2", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 3, "spdif-dit.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dfll", 0x70110000, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-xhci", 0x70090000, "tegra-xhci",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-xudc", 0x700D0000, "tegra-xudc",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra30-hda", 0x70030000, "tegra30-hda", NULL),
	{}
};

static struct tegra_suspend_platform_data tegra21_suspend_data = {
	.cpu_timer      = 500,
	.cpu_off_timer  = 300,
	.suspend_mode   = TEGRA_SUSPEND_LP0,
	.core_timer     = 0x157e,
	.core_off_timer = 10,
	.corereq_high   = true,
	.sysclkreq_high = true,
};

static void __init tegra210_dt_init(void)
{
	tegra_soc_device_init("granada");
	of_platform_populate(NULL,
			of_default_bus_match_table,
			t210_auxdata_lookup,
			&platform_bus);

	t210_emc_init();
	t210ref_panel_init();

	tegra_init_suspend(&tegra21_suspend_data);
}

static const char * const tegra210_dt_board_compat[] = {
	"nvidia,tegra210",
	NULL
};

DT_MACHINE_START(TEGRA210_DT, "t210")
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_t210_reserve,
	.init_early	= tegra21x_init_early,
	.init_late	= tegra_init_late,
	.init_machine	= tegra210_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tegra210_dt_board_compat,
MACHINE_END

void tegra_pd_in_idle(bool enable) {}
