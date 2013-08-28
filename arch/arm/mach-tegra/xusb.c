/*
 * arch/arm/mach-tegra/xusb.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Ajay Gupta <ajayg@nvidia.com>
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
#include <linux/types.h>
#include <mach/xusb.h>
#include "devices.h"
#include "fuse.h"

#ifdef CONFIG_TEGRA_XUSB_PLATFORM
static struct tegra_xusb_platform_data tegra_xusb_plat_data = {};

static void tegra_xusb_read_usb_calib(void)
{
	u32 usb_calib0 = tegra_fuse_readl(FUSE_SKU_USB_CALIB_0);

	pr_info("tegra_xusb_read_usb_calib: usb_calib0 = 0x%08x\n", usb_calib0);
	/*
	 * read from usb_calib0 and pass to driver
	 * set HS_CURR_LEVEL (PAD0)	= usb_calib0[5:0]
	 * set TERM_RANGE_ADJ		= usb_calib0[10:7]
	 * set HS_SQUELCH_LEVEL		= usb_calib0[12:11]
	 * set HS_IREF_CAP		= usb_calib0[14:13]
	 * set HS_CURR_LEVEL (PAD1)	= usb_calib0[20:15]
	 */

	tegra_xusb_plat_data.hs_curr_level_pad0 = (usb_calib0 >> 0) & 0x3f;
	tegra_xusb_plat_data.hs_term_range_adj = (usb_calib0 >> 7) & 0xf;
	tegra_xusb_plat_data.hs_squelch_level = (usb_calib0 >> 11) & 0x3;
	tegra_xusb_plat_data.hs_iref_cap = (usb_calib0 >> 13) & 0x3;
	tegra_xusb_plat_data.hs_curr_level_pad1 = (usb_calib0 >> 15) & 0x3f;
	tegra_xusb_plat_data.hs_curr_level_pad2 = (usb_calib0 >> 15) & 0x3f;
}

void tegra_xusb_init(struct tegra_xusb_board_data *bdata)
{
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	tegra_xusb_plat_data.pmc_portmap = (TEGRA_XUSB_UTMIP_PMC_PORT0 << 0) |
			(TEGRA_XUSB_UTMIP_PMC_PORT2 << 4);
	tegra_xusb_plat_data.quirks |= TEGRA_XUSB_USE_HS_SRC_CLOCK2;
	tegra_xusb_plat_data.rx_wander = (0x3 << 4);
	tegra_xusb_plat_data.rx_eq = (0x3928 << 8);
	tegra_xusb_plat_data.cdr_cntl = (0x26 << 24);
	tegra_xusb_plat_data.dfe_cntl = 0x002008EE;
	tegra_xusb_plat_data.hs_slew = (0xE << 6);
	tegra_xusb_plat_data.ls_rslew_pad0 = (0x3 << 14);
	tegra_xusb_plat_data.ls_rslew_pad1 = (0x0 << 14);
	tegra_xusb_plat_data.hs_disc_lvl = (0x5 << 2);
	tegra_xusb_plat_data.spare_in = 0x0;
#elif defined(CONFIG_ARCH_TEGRA_12x_SOC)
	tegra_xusb_plat_data.pmc_portmap = (TEGRA_XUSB_UTMIP_PMC_PORT0 << 0) |
			(TEGRA_XUSB_UTMIP_PMC_PORT1 << 4) |
			(TEGRA_XUSB_UTMIP_PMC_PORT2 << 8);
	tegra_xusb_plat_data.rx_wander = (0xF << 4);
	tegra_xusb_plat_data.rx_eq = (0xF070 << 8);
	tegra_xusb_plat_data.cdr_cntl = (0x26 << 24);
	tegra_xusb_plat_data.dfe_cntl = 0x002008EE;
	tegra_xusb_plat_data.hs_slew = (0xE << 6);
	tegra_xusb_plat_data.ls_rslew_pad0 = (0x3 << 14);
	tegra_xusb_plat_data.ls_rslew_pad1 = (0x0 << 14);
	tegra_xusb_plat_data.ls_rslew_pad2 = (0x0 << 14);
	tegra_xusb_plat_data.hs_disc_lvl = (0x5 << 2);
	tegra_xusb_plat_data.spare_in = 0x1;
#endif
	tegra_xusb_read_usb_calib();
	tegra_xusb_plat_data.bdata = bdata;
	tegra_xhci_device.dev.platform_data = &tegra_xusb_plat_data;
	platform_device_register(&tegra_xhci_device);
}
#else
void tegra_xusb_init(struct tegra_xusb_board_data *bdata) {}
#endif
