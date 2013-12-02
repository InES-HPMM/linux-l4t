/*
 *  linux/arch/arm64/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2013-2014, NVIDIA Corporation. All rights reserved.
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
#include "flowctrl.h"

#define PWRGATE_TOGGLE		0x30
#define PMC_TOGGLE_START	0x100

static DECLARE_BITMAP(tegra_cpu_power_up_by_fc, CONFIG_NR_CPUS) __read_mostly;
struct cpumask *tegra_cpu_power_mask =
				to_cpumask(tegra_cpu_power_up_by_fc);
#define tegra_cpu_power_map	(*(cpumask_t *)tegra_cpu_power_mask)

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
#define pmc_writel(value, reg)	writel(value, pmc + (reg))
#define pmc_readl(reg)		readl(pmc + (reg))

static void __init tegra_smp_prepare_cpus(unsigned int max_cpus)
{
	/* Always mark the boot CPU as initially powered up */
	cpumask_set_cpu(0, tegra_cpu_power_mask);

	tegra_cpu_reset_handler_init();
}

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
static void tegra_secondary_init(unsigned int cpu)
{
	cpumask_set_cpu(cpu, tegra_cpu_power_mask);
}

static int tegra_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	u32 reg;

	cpu = cpu_logical_map(cpu);

	if (cpu_isset(cpu, tegra_cpu_power_map)) {
		/* set SCLK as event trigger for flow conroller */
		flowctrl_write_cpu_csr(cpu, 0x1);
		flowctrl_write_cpu_halt(cpu, 0x48000000);
	} else {
		reg = PMC_TOGGLE_START | TEGRA_CPU_POWERGATE_ID(cpu);
		pmc_writel(reg, PWRGATE_TOGGLE);
	}

	return 0;
}
#endif

struct smp_operations tegra_smp_ops __initdata = {
	.smp_prepare_cpus	= tegra_smp_prepare_cpus,
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	.smp_secondary_init	= tegra_secondary_init,
	.smp_boot_secondary	= tegra_boot_secondary,
#endif
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= tegra_cpu_die,
#endif
};
