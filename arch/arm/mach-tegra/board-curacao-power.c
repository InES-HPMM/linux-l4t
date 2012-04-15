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
#include "tegra_cl_dvfs.h"

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

/* FIXME: the cl_dvfs data below is bogus, just to enable s/w
   debugging on FPGA */

/* Board characterization parameters */
static struct tegra_cl_dvfs_cfg_param curacao_cl_dvfs_param = {
	.sample_rate = 2250,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 16,
	.ci = 7,
	.cg = -12,

	.droop_cut_value = 0x8,
	.droop_restore_ramp = 0x10,
	.scale_out_ramp = 0x2,
};

/* PMU data : assume fixed 12.5mV steps from 600mV to 1400mV starting
   with reg value 3 */
#define PMU_CPU_VDD_MAP_SIZE ((1400000 - 600000) / 12500 + 1)
static struct voltage_reg_map pmu_cpu_vdd_map[PMU_CPU_VDD_MAP_SIZE];
static void fill_reg_map(void)
{
	int i;
	for (i = 0; i < PMU_CPU_VDD_MAP_SIZE; i++) {
		pmu_cpu_vdd_map[i].reg_value = i + 3;
		pmu_cpu_vdd_map[i].reg_uV = 600000 + 12500 * i;
	}
}

static struct tegra_cl_dvfs_platform_data curacao_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 50000,
		.hs_master_code = 0, /* no hs mode support */
		.slave_addr = 0x2D,
		.reg = 0x27,
	},
	.vdd_map = pmu_cpu_vdd_map,
	.vdd_map_size = PMU_CPU_VDD_MAP_SIZE,

	.cfg_param = &curacao_cl_dvfs_param,
};

int __init curacao_regulator_init(void)
{
	platform_device_register(&curacao_pda_power_device);
	fill_reg_map();
	tegra_cl_dvfs_set_plarform_data(&curacao_cl_dvfs_data);
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
