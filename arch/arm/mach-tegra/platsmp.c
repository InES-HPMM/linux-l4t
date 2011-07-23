/*
 *  linux/arch/arm/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>

#include <asm/smp_scu.h>

#include <mach/powergate.h>

#include "fuse.h"
#include "flowctrl.h"
#include "reset.h"
#include "pm.h"
#include "clock.h"

#include "common.h"
#include "iomap.h"

#define EVP_CPU_RESET_VECTOR \
	(IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE) + 0x100)

static unsigned int available_cpus(void);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
static inline int is_g_cluster_available(unsigned int cpu)
{ return -EPERM; }
static inline bool is_cpu_powered(unsigned int cpu)
{ return true; }
static inline int power_up_cpu(unsigned int cpu)
{ return 0; }

/* For Tegra2 use the software-written value of the reset regsiter for status.*/
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET

#else
static int is_g_cluster_available(unsigned int cpu);
static bool is_cpu_powered(unsigned int cpu);
static int power_up_cpu(unsigned int cpu);

#define CAR_BOND_OUT_V \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x390)
#define CAR_BOND_OUT_V_CPU_G	(1<<0)
#define CLK_RST_CONTROLLER_CPU_CMPLX_STATUS \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x470)

#endif

extern void tegra_secondary_startup(void);

static void __iomem *scu_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE);

static void __cpuinit tegra_secondary_init(unsigned int cpu)
{
}

static int tegra20_power_up_cpu(unsigned int cpu)
{
	int status;

	if (is_lp_cluster()) {
		struct clk *cpu_clk, *cpu_g_clk;

		/* The G CPU may not be available for a variety of reasons. */
		status = is_g_cluster_available(cpu);
		if (status)
			goto done;

		cpu_clk = tegra_get_clock_by_name("cpu");
		cpu_g_clk = tegra_get_clock_by_name("cpu_g");

		/* Switch to G CPU before continuing. */
		if (!cpu_clk || !cpu_g_clk) {
			/* Early boot, clock infrastructure is not initialized
			   - CPU mode switch is not allowed */
			status = -EINVAL;
		} else
			status = clk_set_parent(cpu_clk, cpu_g_clk);

		if (status)
			goto done;
	}

	/* Enable the CPU clock. */
	tegra_enable_cpu_clock(cpu);

	/* Clear flow controller CSR. */
	flowctrl_write_cpu_csr(cpu, 0);

	return 0;
}

static int tegra30_power_up_cpu(unsigned int cpu)
{
	int ret, pwrgateid;
	unsigned long timeout;

	pwrgateid = tegra_cpu_powergate_id(cpu);
	if (pwrgateid < 0)
		return pwrgateid;

	/* If this is the first boot, toggle powergates directly. */
	if (!tegra_powergate_is_powered(pwrgateid)) {
		ret = tegra_powergate_power_on(pwrgateid);
		if (ret)
			return ret;

		/* Wait for the power to come up. */
		timeout = jiffies + 10*HZ;
		while (tegra_powergate_is_powered(pwrgateid)) {
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
			udelay(10);
		}
	}

	/* CPU partition is powered. Enable the CPU clock. */
	tegra_enable_cpu_clock(cpu);
	udelay(10);

	/* Remove I/O clamps. */
	ret = tegra_powergate_remove_clamping(pwrgateid);
	if (ret)
		return ret;

	udelay(10);

	/* Clear flow controller CSR. */
	flowctrl_write_cpu_csr(cpu, 0);

done:
	return status;
}

static int __cpuinit tegra_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	int status;

	/*
	 * Force the CPU into reset. The CPU must remain in reset when the
	 * flow controller state is cleared (which will cause the flow
	 * controller to stop driving reset if the CPU has been power-gated
	 * via the flow controller). This will have no effect on first boot
	 * of the CPU since it should already be in reset.
	 */
	tegra_put_cpu_in_reset(cpu);

	/*
	 * Unhalt the CPU. If the flow controller was used to power-gate the
	 * CPU this will cause the flow controller to stop driving reset.
	 * The CPU will remain in reset because the clock and reset block
	 * is now driving reset.
	 */
	flowctrl_write_cpu_halt(cpu, 0);

	switch (tegra_chip_id) {
	case TEGRA20:
		status = tegra20_power_up_cpu(cpu);
		break;
	case TEGRA30:
		status = tegra30_power_up_cpu(cpu);
		break;
	default:
		status = -EINVAL;
		break;
	}

	if (status)
		goto done;

	/* Take the CPU out of reset. */
	tegra_cpu_out_of_reset(cpu);
done:
	return status;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init tegra_smp_init_cpus(void)
{
	unsigned int ncores = available_cpus();
	unsigned int i;

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

static void __init tegra_smp_prepare_cpus(unsigned int max_cpus)
{
	tegra_cpu_reset_handler_init();
	scu_enable(scu_base);
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)

static bool is_cpu_powered(unsigned int cpu)
{
	if (is_lp_cluster())
		return true;
	else
		return tegra_powergate_is_powered(TEGRA_CPU_POWERGATE_ID(cpu));
}

static int power_up_cpu(unsigned int cpu)
{
	int ret;
	u32 reg;
	unsigned long timeout;

	BUG_ON(cpu == smp_processor_id());
	BUG_ON(is_lp_cluster());

	/* If this cpu has booted this function is entered after
	 * CPU has been already un-gated by flow controller. Wait
	 * for confirmation that cpu is powered and remove clamps.
	 * On first boot entry do not wait - go to direct ungate.
	 */
#if 0 /* FIXME! */
	if (cpu_isset(cpu,*(cpumask_t*)&tegra_cpu_init_map))
	{
		timeout = jiffies + HZ;
		do {
			if (is_cpu_powered(cpu))
				goto remove_clamps;
			udelay(10);
		} while (time_before(jiffies, timeout));
	}
#endif
	/* 1'st boot or Flow controller did not work as expected - try directly toggle
	   power gates. Bail out if direct power on also failed */
	if (!is_cpu_powered(cpu))
	{
		ret = tegra_powergate_power_on(TEGRA_CPU_POWERGATE_ID(cpu));
		if (ret)
			goto fail;

		/* Wait for the power to come up. */
		timeout = jiffies + 10*HZ;

		do {
			if (is_cpu_powered(cpu))
				goto remove_clamps;
			udelay(10);
		} while (time_before(jiffies, timeout));
		ret = -ETIMEDOUT;
		goto fail;
	}

remove_clamps:
	/* now CPU is up: enable clock, propagate reset, and remove clamps */
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg & ~CPU_CLOCK(cpu), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	barrier();
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	udelay(10);
	ret = tegra_powergate_remove_clamping(TEGRA_CPU_POWERGATE_ID(cpu));
fail:
	return ret;
}

static int is_g_cluster_available(unsigned int cpu)
{
	u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
	u32 bond_out = readl(CAR_BOND_OUT_V);

	/* Does the G CPU complex exist at all? */
	if ((fuse_sku & FUSE_SKU_DISABLE_ALL_CPUS) ||
	    (bond_out & CAR_BOND_OUT_V_CPU_G))
		return -EPERM;

	if (cpu >= available_cpus())
		return -EPERM;

	/* FIXME: The G CPU can be unavailable for a number of reasons
	 *	  (e.g., low battery, over temperature, etc.). Add checks for
	 *	  these conditions. */

	return 0;
}
#endif

static unsigned int available_cpus(void)
{
	static unsigned int ncores = 0;

	if (ncores == 0) {
		ncores = scu_get_core_count(scu_base);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		if (ncores > 1) {
			u32 fuse_sku = readl(FUSE_SKU_DIRECT_CONFIG);
			ncores -= FUSE_SKU_NUM_DISABLED_CPUS(fuse_sku);
			BUG_ON((int)ncores <= 0);
		}
#endif
	}
	return ncores;
}

struct smp_operations tegra_smp_ops __initdata = {
	.smp_init_cpus		= tegra_smp_init_cpus,
	.smp_prepare_cpus	= tegra_smp_prepare_cpus,
	.smp_secondary_init	= tegra_secondary_init,
	.smp_boot_secondary	= tegra_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= tegra_cpu_kill,
	.cpu_die		= tegra_cpu_die,
#endif
};
