/*
 * arch/arm/mach-tegra/board-curacao-power.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "pm.h"
#include "board.h"

static int ac_online(void)
{
	return 1;
}

static struct resource curacao_pda_resources[] = {
	[0] = {
		.name	= "ac",
	},
};

static struct pda_power_pdata curacao_pda_data = {
	.is_ac_online	= ac_online,
};

static struct platform_device curacao_pda_power_device = {
	.name		= "pda-power",
	.id		= -1,
	.resource	= curacao_pda_resources,
	.num_resources	= ARRAY_SIZE(curacao_pda_resources),
	.dev	= {
		.platform_data	= &curacao_pda_data,
	},
};

static struct tegra_suspend_platform_data curacao_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= false,
	.sysclkreq_high	= true,
};

int __init curacao_regulator_init(void)
{
	platform_device_register(&curacao_pda_power_device);
	return 0;
}

int __init curacao_suspend_init(void)
{
	tegra_init_suspend(&curacao_suspend_data);
	return 0;
}

#ifdef CONFIG_TEGRA_SIMULATION_PLATFORM

#define COSIM_SHUTDOWN_REG         0x538f0ffc

static void curacao_power_off(void)
{
	pr_err("Curacao Powering off the device\n");
	writel(1, IO_ADDRESS(COSIM_SHUTDOWN_REG));
	while (1)
		;
}

int __init curacao_power_off_init(void)
{
	pm_power_off = curacao_power_off;
	return 0;
}
#endif
