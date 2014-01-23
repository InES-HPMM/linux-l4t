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
#include <linux/tegra-powergate.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/soc.h>

#include "common.h"
#include "reset.h"
#include "iomap.h"

#define PWRGATE_TOGGLE		0x30
#define PMC_TOGGLE_START	0x100

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
#define pmc_writel(value, reg)	writel(value, pmc + (reg))
#define pmc_readl(reg)		readl(pmc + (reg))

static void __init tegra_smp_prepare_cpus(unsigned int max_cpus)
{
	tegra_cpu_reset_handler_init();
}

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
static int tegra_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	u32 reg;

	BUG_ON(cpu == smp_processor_id());

	cpu = cpu_logical_map(cpu);

	reg = PMC_TOGGLE_START | TEGRA_CPU_POWERGATE_ID(cpu);
	pmc_writel(reg, PWRGATE_TOGGLE);

	return 0;
}
#endif

struct smp_operations tegra_smp_ops __initdata = {
	.smp_prepare_cpus	= tegra_smp_prepare_cpus,
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	.smp_boot_secondary	= tegra_boot_secondary,
#endif
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= tegra_cpu_die,
#endif
};
