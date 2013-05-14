/*
 * arch/arm/mach-tegra/include/mach/xusb.h
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
#ifndef _XUSB_H
#define _XUSB_H

#include <linux/platform_data/tegra_xusb.h>

/*
 * BIT0 - BIT7 : SS ports
 * BIT8 - BIT15 : USB2 UTMI ports
 * BIT16 - BIT23 : HSIC ports
 * BIT24 - BIT31 : ULPI ports
 */
#define TEGRA_XUSB_SS_P0	(1 << 0)
#define TEGRA_XUSB_SS_P1	(1 << 1)
#define TEGRA_XUSB_USB2_P0	(1 << 8)
#define TEGRA_XUSB_USB2_P1	(1 << 9)
#define TEGRA_XUSB_HSIC_P0	(1 << 16)
#define TEGRA_XUSB_HSIC_P1	(1 << 17)
#define TEGRA_XUSB_ULPI_P0	(1 << 24)
#define TEGRA_XUSB_SS_PORT_MAP_USB2_P0 (0x0)
#define TEGRA_XUSB_SS_PORT_MAP_USB2_P1 (0x1)
#define TEGRA_XUSB_SS0_PORT_MAP	(0xf)
#define TEGRA_XUSB_SS1_PORT_MAP	(0xf0)

struct tegra_xusb_board_data {
	u32	portmap;
	/*
	 * SS0 or SS1 port may be mapped either to USB2_P0 or USB2_P1
	 * ss_portmap[0:3] = SS0 map, ss_portmap[4:7] = SS1 map
	 */
	u8	ss_portmap;
	struct tegra_xusb_pad_data *padctl_data;
};

struct tegra_xusb_platform_data {
	struct tegra_xusb_board_data *bdata;
	u32 hs_curr_level_pad0;
	u32 hs_curr_level_pad1;
	u32 hs_iref_cap;
	u32 hs_term_range_adj;
	u32 hs_squelch_level;
};

extern void tegra_xusb_init(struct tegra_xusb_board_data *bdata);
#endif /* _XUSB_H */
