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
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/gpio-regulator.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "pm.h"
#include "board.h"
#include "tegra_cl_dvfs.h"
#include "gpio-names.h"

static int ac_online(void)
{
	return 1;
}

static struct regulator_consumer_supply gpio_reg_sdmmc3_vdd_sel_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};

static struct gpio_regulator_state gpio_reg_sdmmc3_vdd_sel_states[] = {
	{
		.gpios = 0,
		.value = 1800000,
	},
	{
		.gpios = 1,
		.value = 3300000,
	},
};

static struct gpio gpio_reg_sdmmc3_vdd_sel_gpios[] = {
	{
		.gpio = TEGRA_GPIO_PV1,
		.flags = 0,
		.label = "sdmmc3_vdd_sel",
	},
};

/* Macro for defining gpio regulator device data */
#define GPIO_REG(_id, _name, _input_supply, _active_high,	\
		_boot_state, _delay_us, _minmv, _maxmv)		\
	static struct regulator_init_data ri_data_##_name =	\
{								\
	.supply_regulator = NULL,				\
	.num_consumer_supplies =				\
		ARRAY_SIZE(gpio_reg_##_name##_supply),		\
	.consumer_supplies = gpio_reg_##_name##_supply,		\
	.constraints = {					\
		.name = "gpio_reg_"#_name,			\
		.min_uV = (_minmv)*1000,			\
		.max_uV = (_maxmv)*1000,			\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
				REGULATOR_MODE_STANDBY),	\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
				REGULATOR_CHANGE_STATUS |	\
				REGULATOR_CHANGE_VOLTAGE),	\
	},							\
};								\
static struct gpio_regulator_config gpio_reg_##_name##_pdata =	\
{								\
	.supply_name = "vddio_sdmmc",				\
	.enable_gpio = -EINVAL,					\
	.enable_high = _active_high,				\
	.enabled_at_boot = _boot_state,				\
	.startup_delay = _delay_us,				\
	.gpios = gpio_reg_##_name##_gpios,			\
	.nr_gpios = ARRAY_SIZE(gpio_reg_##_name##_gpios),	\
	.states = gpio_reg_##_name##_states,			\
	.nr_states = ARRAY_SIZE(gpio_reg_##_name##_states),	\
	.type = REGULATOR_VOLTAGE,				\
	.init_data = &ri_data_##_name,				\
};								\
static struct platform_device gpio_reg_##_name##_dev = {	\
	.name   = "gpio-regulator",				\
	.id = _id,						\
	.dev    = {						\
		.platform_data = &gpio_reg_##_name##_pdata,	\
	},							\
}
GPIO_REG(4, sdmmc3_vdd_sel, NULL,
		true, true, 0, 1000, 3300);

#define ADD_GPIO_REG(_name) (&gpio_reg_##_name##_dev)
static struct platform_device *gpio_regs_devices[] = {
	ADD_GPIO_REG(sdmmc3_vdd_sel),
};

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

static int __init curacao_gpio_regulator_init(void)
{
	int i, j;
	for (i = 0; i < ARRAY_SIZE(gpio_regs_devices); ++i) {
		struct gpio_regulator_config *gpio_reg_pdata =
			gpio_regs_devices[i]->dev.platform_data;
		for (j = 0; j < gpio_reg_pdata->nr_gpios; ++j) {
			if (gpio_reg_pdata->gpios[j].gpio < TEGRA_NR_GPIOS)
				tegra_gpio_enable(gpio_reg_pdata->gpios[j].gpio);
		}
	}
	return platform_add_devices(gpio_regs_devices,
			ARRAY_SIZE(gpio_regs_devices));
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
	return curacao_gpio_regulator_init();
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
