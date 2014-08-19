/*
 * drivers/platform/tegra/asim.c
 *
 * Copyright (C) 2010-2014 NVIDIA Corporation. All rights reserved.
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

#include <linux/printk.h>
#include <linux/io.h>
#include <linux/tegra-soc.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>

#include "iomap.h"

#define ASIM_SHUTDOWN_REG_PHYS       0x538f0ffc


static void __iomem *asim_shutdown_reg;


static void asim_power_off(void)
{
	pr_err("ASIM Powering off the device\n");
	*(unsigned int *)(asim_shutdown_reg) = 1;
	while (1);
}

static int __init asim_power_off_init(void)
{
	if (tegra_cpu_is_asim()) {
		pm_power_off = asim_power_off;
		asim_shutdown_reg = ioremap(ASIM_SHUTDOWN_REG_PHYS, 4);
	}

	return 0;
}

arch_initcall(asim_power_off_init);

#if defined(CONFIG_SMC91X)
static struct resource tegra_asim_smc91x_resources[] = {
	[0] = {
		.start	= TEGRA_SIM_ETH_BASE,
		.end	= TEGRA_SIM_ETH_BASE + TEGRA_SIM_ETH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_ETH,
		.end	= IRQ_ETH,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_asim_smc91x_device = {
	.name	   = "smc91x",
	.id	     = 0,
	.num_resources  = ARRAY_SIZE(tegra_asim_smc91x_resources),
	.resource       = tegra_asim_smc91x_resources,
};

static int __init asim_enet_smc91x_init(void)
{
	if (tegra_cpu_is_asim() && !tegra_cpu_is_dsim())
		platform_device_register(&tegra_asim_smc91x_device);
	return 0;
}

rootfs_initcall(asim_enet_smc91x_init);
#endif
