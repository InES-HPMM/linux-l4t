/*
 * Copyright (C) 2011 NVIDIA, Inc.
 * Copyright (c) 2013 NVIDIA CORPORATION. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
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
#include <linux/regulator/fixed.h>

#include <mach/gpio-tegra.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"
#include "iomap.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"


static int __init ardbeg_cl_dvfs_init(u16 pmu_board_id);

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

GPIO_REG(4, sdmmc3_vdd_sel, NULL, true, true, 0, 1000, 3300);

#define ADD_GPIO_REG(_name) (&gpio_reg_##_name##_dev)
static struct platform_device *gpio_regs_devices[] = {
	ADD_GPIO_REG(sdmmc3_vdd_sel),
};

static struct resource bonaire_pda_resources[] = {
	[0] = {
		.name	= "ac",
	},
};

static struct pda_power_pdata bonaire_pda_data = {
	.is_ac_online	= ac_online,
};

static struct platform_device bonaire_pda_power_device = {
	.name		= "pda-power",
	.id		= -1,
	.resource	= bonaire_pda_resources,
	.num_resources	= ARRAY_SIZE(bonaire_pda_resources),
	.dev	= {
		.platform_data	= &bonaire_pda_data,
	},
};

static struct regulator_consumer_supply fixed_reg_en_battery_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

#define FIXED_SUPPLY(_name) "fixed_reg_en"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _open_drain, _active_high, _boot_state, _millivolts,	\
	_sdelay)							\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_en_##_name##_supply),	\
		.consumer_supplies = fixed_reg_en_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
	};								\
	static struct fixed_voltage_config fixed_reg_en_##_var##_pdata = \
	{								\
		.supply_name = FIXED_SUPPLY(_name),			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.gpio_is_open_drain = _open_drain,			\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
		.startup_delay = _sdelay				\
	};								\
	static struct platform_device fixed_reg_en_##_var##_dev = {	\
		.name = "reg-fixed-voltage",				\
		.id = _id,						\
		.dev = {						\
			.platform_data = &fixed_reg_en_##_var##_pdata,	\
		},							\
	}

FIXED_REG(0,	battery,	battery,
	NULL,	0,	0,
	-1,	false, true,	0,	3300,	0);

static struct platform_device *pfixed_reg_devs[] = {
	&fixed_reg_en_battery_dev,
};

#ifndef CONFIG_ARCH_TEGRA_13x_SOC
static struct tegra_suspend_platform_data bonaire_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= false,
	.sysclkreq_high	= true,
};
#endif

int __init bonaire_regulator_init(void)
{
	platform_device_register(&bonaire_pda_power_device);
	platform_add_devices(pfixed_reg_devs, ARRAY_SIZE(pfixed_reg_devs));

	struct board_info pmu_board_info;
	tegra_get_pmu_board_info(&pmu_board_info);
	ardbeg_cl_dvfs_init(pmu_board_info.board_id);

	return platform_add_devices(gpio_regs_devices,
		ARRAY_SIZE(gpio_regs_devices));
}

int __init bonaire_suspend_init(void)
{
#ifndef CONFIG_ARCH_TEGRA_13x_SOC
	tegra_init_suspend(&bonaire_suspend_data);
#endif
	return 0;
}


#define COSIM_SHUTDOWN_REG         0x538f0ffc

static void bonaire_power_off(void)
{
	pr_err("Bonaire: Powering off the device\n");
	writel(1, IO_ADDRESS(COSIM_SHUTDOWN_REG));
	while (1)
		;
}

int __init bonaire_power_off_init(void)
{
	pm_power_off = bonaire_power_off;
	return 0;
}

/************************ ARDBEG CL-DVFS DATA *********************/
#define E1735_CPU_VDD_MAP_SIZE		33
#define E1735_CPU_VDD_MIN_UV		675000
#define E1735_CPU_VDD_STEP_UV		18750
#define E1735_CPU_VDD_STEP_US		80
#define ARDBEG_DEFAULT_CVB_ALIGNMENT	10000

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* Macro definition of dfll bypass device */
#define DFLL_BYPASS(_board, _min, _step, _size, _us_sel)		       \
static struct regulator_init_data _board##_dfll_bypass_init_data = {	       \
	.num_consumer_supplies = ARRAY_SIZE(_board##_dfll_bypass_consumers),   \
	.consumer_supplies = _board##_dfll_bypass_consumers,		       \
	.constraints = {						       \
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |		       \
				REGULATOR_MODE_STANDBY),		       \
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |		       \
				REGULATOR_CHANGE_STATUS |		       \
				REGULATOR_CHANGE_VOLTAGE),		       \
		.min_uV = (_min),					       \
		.max_uV = ((_size) - 1) * (_step) + (_min),		       \
		.always_on = 1,						       \
		.boot_on = 1,						       \
	},								       \
};									       \
static struct tegra_dfll_bypass_platform_data _board##_dfll_bypass_pdata = {   \
	.reg_init_data = &_board##_dfll_bypass_init_data,		       \
	.uV_step = (_step),						       \
	.linear_min_sel = 0,						       \
	.n_voltages = (_size),						       \
	.voltage_time_sel = _us_sel,					       \
};									       \
static struct platform_device e1735_dfll_bypass_dev = {			       \
	.name = "tegra_dfll_bypass",					       \
	.id = -1,							       \
	.dev = {							       \
		.platform_data = &_board##_dfll_bypass_pdata,		       \
	},								       \
}

/* E1735 board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param e1735_cl_dvfs_param = {
	.sample_rate = 50000,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};

/* E1735 RT8812C volatge map */
static struct voltage_reg_map e1735_cpu_vdd_map[E1735_CPU_VDD_MAP_SIZE];
static inline int e1735_fill_reg_map(int nominal_mv)
{
	int i, uv, nominal_uv = 0;
	for (i = 0; i < E1735_CPU_VDD_MAP_SIZE; i++) {
		e1735_cpu_vdd_map[i].reg_value = i;
		e1735_cpu_vdd_map[i].reg_uV = uv =
			E1735_CPU_VDD_MIN_UV + E1735_CPU_VDD_STEP_UV * i;
		if (!nominal_uv && uv >= nominal_mv * 1000)
			nominal_uv = uv;
	}
	return nominal_uv;
}

/* E1735 dfll bypass device for legacy dvfs control */
static struct regulator_consumer_supply e1735_dfll_bypass_consumers[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};
// DFLL_BYPASS(e1735, E1735_CPU_VDD_MIN_UV, E1735_CPU_VDD_STEP_UV,
//	    E1735_CPU_VDD_MAP_SIZE, E1735_CPU_VDD_STEP_US);

static struct tegra_cl_dvfs_platform_data e1735_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_PWM,
	.u.pmu_pwm = {
		.pwm_rate = 12750000,
		.out_gpio = TEGRA_GPIO_PS5,
		.out_enable_high = false,
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
//		.dfll_bypass_dev = &e1735_dfll_bypass_dev,
#endif
	},
	.vdd_map = e1735_cpu_vdd_map,
	.vdd_map_size = E1735_CPU_VDD_MAP_SIZE,

	.cfg_param = &e1735_cl_dvfs_param,
};

static void e1735_suspend_dfll_bypass(void)
{
	__gpio_set_value(TEGRA_GPIO_PS5, 1); /* tristate external PWM buffer */
}

static void e1735_resume_dfll_bypass(void)
{
	__gpio_set_value(TEGRA_GPIO_PS5, 0); /* enable PWM buffer operations */
}

static struct tegra_cl_dvfs_cfg_param e1733_ardbeg_cl_dvfs_param = {
	.sample_rate = 12500,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};

/* E1733 volatge map. Fixed 10mv steps from 700mv to 1400mv */
#define E1733_CPU_VDD_MAP_SIZE ((1400000 - 700000) / 10000 + 1)
static struct voltage_reg_map e1733_cpu_vdd_map[E1733_CPU_VDD_MAP_SIZE];
static inline void e1733_fill_reg_map(void)
{
        int i;
        for (i = 0; i < E1733_CPU_VDD_MAP_SIZE; i++) {
                e1733_cpu_vdd_map[i].reg_value = i + 0xa;
                e1733_cpu_vdd_map[i].reg_uV = 700000 + 10000 * i;
        }
}

static struct tegra_cl_dvfs_platform_data e1733_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0x80,
		.reg = 0x00,
	},
	.vdd_map = e1733_cpu_vdd_map,
	.vdd_map_size = E1733_CPU_VDD_MAP_SIZE,

	.cfg_param = &e1733_ardbeg_cl_dvfs_param,
};

static struct tegra_cl_dvfs_cfg_param e1736_ardbeg_cl_dvfs_param = {
	.sample_rate = 12500, /* i2c freq */

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};

/* E1736 volatge map. Fixed 10mv steps from 700mv to 1400mv */
#define E1736_CPU_VDD_MAP_SIZE ((1400000 - 700000) / 10000 + 1)
static struct voltage_reg_map e1736_cpu_vdd_map[E1736_CPU_VDD_MAP_SIZE];
static inline void e1736_fill_reg_map(void)
{
	int i;
	for (i = 0; i < E1736_CPU_VDD_MAP_SIZE; i++) {
		/* 0.7V corresponds to 0b0011010 = 26 */
		/* 1.4V corresponds to 0b1100000 = 96 */
		e1736_cpu_vdd_map[i].reg_value = i + 26;
		e1736_cpu_vdd_map[i].reg_uV = 700000 + 10000 * i;
	}
}

static struct tegra_cl_dvfs_platform_data e1736_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0xb0, /* pmu i2c address */
		.reg = 0x23,        /* vdd_cpu rail reg address */
	},
	.vdd_map = e1736_cpu_vdd_map,
	.vdd_map_size = E1736_CPU_VDD_MAP_SIZE,

	.cfg_param = &e1736_ardbeg_cl_dvfs_param,
};
static int __init ardbeg_cl_dvfs_init(u16 pmu_board_id)
{
	struct tegra_cl_dvfs_platform_data *data = NULL;
//	int v = tegra_dvfs_rail_get_nominal_millivolts(tegra_cpu_rail);


        e1733_fill_reg_map();
        data = &e1733_cl_dvfs_data;

	if (data) {
		data->flags = TEGRA_CL_DVFS_DYN_OUTPUT_CFG;
		tegra_cl_dvfs_device.dev.platform_data = data;
		platform_device_register(&tegra_cl_dvfs_device);
	}
	return 0;
}
#else
static inline int ardbeg_cl_dvfs_init(u16 pmu_board_id)
{ return 0; }
#endif
