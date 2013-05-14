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

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
static struct tegra_xusb_platform_data tegra_xusb_plat_data = {};
void tegra_xusb_init(struct tegra_xusb_board_data *bdata)
{
	tegra_xusb_plat_data.bdata = bdata;
	tegra_xhci_device.dev.platform_data = &tegra_xusb_plat_data;
	platform_device_register(&tegra_xhci_device);
}
#endif
