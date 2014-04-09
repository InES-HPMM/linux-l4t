/*
 * arch/arm/mach-tegra/board-p2360-sdhci.c
 * Based on arch/arm/mach-tegra/board-vcm30_t124-sdhci.c
 *
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/platform_data/mmc-sdhci-tegra.h>

#include "board.h"
#include "board-p2360.h"
#include "devices.h"

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data4 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = true,
	.tap_delay = 0x4,
	.trim_delay = 0x4,
	.ddr_trim_delay = 0x4,
	.mmc_data = {
		.built_in = 1,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
	.uhs_mask = MMC_MASK_HS200,
	.ddr_clk_limit = 51000000,
	.max_clk_limit = 102000000,
	/*      .max_clk = 12000000, */
};

int __init p2360_sdhci_init(void)
{
	tegra_sdhci_device4.dev.platform_data = &tegra_sdhci_platform_data4;
	platform_device_register(&tegra_sdhci_device4);

	return 0;
}
