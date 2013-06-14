/*
 *  linux/arch/arm64/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpumask.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/soc.h>

#include "common.h"
#include "reset.h"

void tegra_smp_clear_power_mask()
{
}

static void __init tegra_smp_prepare_cpus(unsigned int max_cpus)
{
	tegra_cpu_reset_handler_init();
}

struct smp_operations tegra_smp_ops __initdata = {
	.smp_prepare_cpus	= tegra_smp_prepare_cpus,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= tegra_cpu_die,
#endif
};
