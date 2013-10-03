/*
 * arch/arm/mach-tegra/board-vcm30_t124-power.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/regulator/max15569-regulator.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/tegra-soc.h>

#include <mach/edp.h>
#include <mach/gpio-tegra.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"
#include "board-common.h"
#include "board-vcm30_t124.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"
#include "tegra11_soctherm.h"
#include "tegra3_tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

/* FIXME: remove power supplies not in use. */

/* MAX77663 consumer rails */
static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr0", NULL),
	REGULATOR_SUPPLY("vddio_ddr1", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	/* REGULATOR_SUPPLY("vdd_core", NULL), */
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_cam", "vi"),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.1"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.1"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vdd_emmc", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_hv", "tegradc.1"),
	REGULATOR_SUPPLY("pwrdet_hv", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("hvdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("hvdd_usb", "tegra-xhci"),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
};

static struct regulator_consumer_supply max77663_sd4_supply[] = {
	REGULATOR_SUPPLY("avdd_csi_dsi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_csi_dsi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_csi_dsi", "vi"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "vi"),
};


static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sys", NULL),
};

/* FIXME: PCI rails should be added? */
static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-xhci"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-xhci"),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
};

static struct max77663_regulator_fps_cfg max77663_fps_cfgs[] = {
	{
		.src = FPS_SRC_0,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_1,
		.en_src = FPS_EN_SRC_EN1,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_2,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
};

#define MAX77663_PDATA_INIT(_rid, _id, _min_uV, _max_uV, _supply_reg, \
		_always_on, _boot_on, _apply_uV, \
		_fps_src, _fps_pu_period, _fps_pd_period, _flags) \
	static struct regulator_init_data max77663_regulator_idata_##_id = { \
		.supply_regulator = _supply_reg, \
		.constraints = { \
			.name = max77663_rails(_id), \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.valid_modes_mask = (REGULATOR_MODE_NORMAL | \
					REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE | \
					REGULATOR_CHANGE_STATUS | \
					REGULATOR_CHANGE_VOLTAGE), \
			.always_on = _always_on, \
			.boot_on = _boot_on, \
			.apply_uV = _apply_uV, \
		}, \
		.num_consumer_supplies = \
		ARRAY_SIZE(max77663_##_id##_supply), \
		.consumer_supplies = max77663_##_id##_supply, \
	}; \
	static struct max77663_regulator_platform_data \
		max77663_regulator_pdata_##_id = \
	{ \
		.reg_init_data = &max77663_regulator_idata_##_id, \
		.id = MAX77663_REGULATOR_ID_##_rid, \
		.fps_src = _fps_src, \
		.fps_pu_period = _fps_pu_period, \
		.fps_pd_period = _fps_pd_period, \
		.fps_cfgs = max77663_fps_cfgs, \
		.flags = _flags, \
	}

/* FIXME: Revisit maximum constraint value for all supplies */

MAX77663_PDATA_INIT(SD0, sd0,  600000, 3387500, NULL, 1, 0, 0,
		FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(SD1, sd1,  800000, 1587500, NULL, 1, 1, 0,
		FPS_SRC_1, FPS_POWER_PERIOD_1, FPS_POWER_PERIOD_6, 0);

MAX77663_PDATA_INIT(SD2, sd2,  1800000, 1800000, NULL, 1, 1, 0,
		FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(SD3, sd3,  600000, 3387500, NULL, 1, 1, 0,
		FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(SD4, sd4,  1200000, 1200000, NULL, 1, 1, 0,
		FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(LDO0, ldo0, 800000, 2350000, NULL, 1, 1, 0,
		FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO1, ldo1, 800000, 2350000, NULL, 0, 0, 0,
		FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO2, ldo2, 800000, 3950000, NULL, 1, 1, 0,
		FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO3, ldo3, 800000, 3950000, NULL, 1, 1, 0,
		FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO4, ldo4, 1000000, 1000000, NULL, 0, 1, 0,
		FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(LDO5, ldo5, 800000, 3950000, NULL, 0, 1, 0,
		FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO6, ldo6, 800000, 3950000, NULL, 0, 0, 0,
		FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO7, ldo7, 800000, 3950000, NULL, 0, 0, 0,
		FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO8, ldo8, 800000, 3950000, NULL, 0, 1, 0,
		FPS_SRC_1, -1, -1, 0);

#define MAX77663_REG(_id, _data) (&max77663_regulator_pdata_##_data)

static struct max77663_regulator_platform_data *max77663_reg_pdata[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
	MAX77663_REG(SD4, sd4),
	MAX77663_REG(LDO0, ldo0),
	MAX77663_REG(LDO1, ldo1),
	MAX77663_REG(LDO2, ldo2),
	MAX77663_REG(LDO3, ldo3),
	MAX77663_REG(LDO4, ldo4),
	MAX77663_REG(LDO5, ldo5),
	MAX77663_REG(LDO6, ldo6),
	MAX77663_REG(LDO7, ldo7),
	MAX77663_REG(LDO8, ldo8),
};

static struct max77663_gpio_config max77663_gpio_cfgs[] = {
	{
		.gpio = MAX77663_GPIO0,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO5,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
};

static struct max77663_platform_data max77663_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs	= ARRAY_SIZE(max77663_gpio_cfgs),
	.gpio_cfgs	= max77663_gpio_cfgs,

	.regulator_pdata = max77663_reg_pdata,
	.num_regulator_pdata = ARRAY_SIZE(max77663_reg_pdata),

	.rtc_i2c_addr	= 0x68,

	.use_power_off	= false,
};

static struct i2c_board_info __initdata max77663_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x3c),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max77663_pdata,
	},
};

/* MAX15569 switching regulator for vdd_cpu */
static struct regulator_consumer_supply max15569_vddcpu_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_init_data max15569_vddcpu_init_data = {
	.constraints = {
		.min_uV = 500000,
		.max_uV = 1520000,
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY),
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					 REGULATOR_CHANGE_CONTROL |
					REGULATOR_CHANGE_VOLTAGE),
		.always_on = 1,
		.boot_on =  1,
		.apply_uV = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(max15569_vddcpu_supply),
		.consumer_supplies = max15569_vddcpu_supply,
};

static struct max15569_regulator_platform_data max15569_vddcpu_pdata = {
	.reg_init_data = &max15569_vddcpu_init_data,
	.max_voltage_uV = 1520000,
	.base_voltage_uV = 1200000,
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddcpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x3a),
		.platform_data	= &max15569_vddcpu_pdata,
	},
};

/* MAX15569 switching regulator for vdd_core */
static struct regulator_consumer_supply max15569_vddcore_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data max15569_vddcore_init_data = {
	.constraints = {
		.min_uV = 500000,
		.max_uV = 1520000,
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY),
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					 REGULATOR_CHANGE_CONTROL |
					REGULATOR_CHANGE_VOLTAGE),
		.always_on = 1,
		.boot_on =  1,
		.apply_uV = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(max15569_vddcore_supply),
		.consumer_supplies = max15569_vddcore_supply,
};

static struct max15569_regulator_platform_data max15569_vddcore_pdata = {
	.reg_init_data = &max15569_vddcore_init_data,
	.max_voltage_uV = 1300000,
	.base_voltage_uV = 1150000,
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddcore_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x39),
		.platform_data	= &max15569_vddcore_pdata,
	},
};

/* MAX15569 switching regulator for vdd_gpu */
static struct regulator_consumer_supply max15569_vddgpu_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};

static struct regulator_init_data max15569_vddgpu_init_data = {
	.constraints = {
		.min_uV = 500000,
		.max_uV = 1520000,
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY),
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					 REGULATOR_CHANGE_CONTROL |
					REGULATOR_CHANGE_VOLTAGE),
		.always_on = 0,
		.boot_on =  0,
		.apply_uV = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(max15569_vddgpu_supply),
		.consumer_supplies = max15569_vddgpu_supply,
};

static struct max15569_regulator_platform_data max15569_vddgpu_pdata = {
	.reg_init_data = &max15569_vddgpu_init_data,
	.max_voltage_uV = 1400000,
	.base_voltage_uV = 1200000,
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddgpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x38),
		.platform_data	= &max15569_vddgpu_pdata,
	},
};

static struct tegra_suspend_platform_data vcm30_t124_suspend_data = {
	.cpu_timer	= 500,
	.cpu_off_timer	= 300,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x157e,
	.core_off_timer = 2000,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 1000,
	.min_residency_crail = 20000,
#ifdef CONFIG_TEGRA_LP1_LOW_COREVOLTAGE
	.lp1_lowvolt_support = false,
	.i2c_base_addr = 0,
	.pmuslave_addr = 0,
	.core_reg_addr = 0,
	.lp1_core_volt_low = 0,
	.lp1_core_volt_high = 0,
#endif
};

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param vcm30_t124_cl_dvfs_param = {
	.sample_rate = 12500,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};
#endif

/* MAX15569: fixed 10mV steps from 600mV to 1400mV, with offset 0x0b */
#define PMU_CPU_VDD_MAP_SIZE ((1400000 - 600000) / 10000 + 1)
static struct voltage_reg_map pmu_cpu_vdd_map[PMU_CPU_VDD_MAP_SIZE];
static inline void fill_reg_map(void)
{
	int i;
	for (i = 0; i < PMU_CPU_VDD_MAP_SIZE; i++) {
		pmu_cpu_vdd_map[i].reg_value = i + 0x0b;
		pmu_cpu_vdd_map[i].reg_uV = 600000 + 10000 * i;
	}
}

/* FIXME: Needed? */
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
static struct tegra_cl_dvfs_platform_data vcm30_t124_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0x3a,
		.reg = 0x07,
	},
	.vdd_map = pmu_cpu_vdd_map,
	.vdd_map_size = PMU_CPU_VDD_MAP_SIZE,

	.cfg_param = &vcm30_t124_cl_dvfs_param,
};

static int __init vcm30_t124_cl_dvfs_init(void)
{
	fill_reg_map();
#if 0
	if (tegra_revision < TEGRA_REVISION_A02)
		vcm30_t124_cl_dvfs_data.out_quiet_then_disable = true;
#endif
	tegra_cl_dvfs_device.dev.platform_data = &vcm30_t124_cl_dvfs_data;
	platform_device_register(&tegra_cl_dvfs_device);

	return 0;
}
#endif

static int __init vcm30_t124_max77663_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	i2c_register_board_info(4, max77663_regulators,
				ARRAY_SIZE(max77663_regulators));

	return 0;
}

int __init vcm30_t124_regulator_init(void)
{
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	vcm30_t124_cl_dvfs_init();
#endif
	vcm30_t124_max77663_regulator_init();

	i2c_register_board_info(4, max15569_vddcpu_boardinfo, 1);
	i2c_register_board_info(4, max15569_vddcore_boardinfo, 1);
	i2c_register_board_info(4, max15569_vddgpu_boardinfo, 1);
	return 0;
}

int __init vcm30_t124_suspend_init(void)
{
	tegra_init_suspend(&vcm30_t124_suspend_data);
	return 0;
}

/* FIXME: Should this be called? */
int __init vcm30_t124_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 14000;

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	regulator_mA = get_maximum_core_current_supported();
	if (!regulator_mA)
		regulator_mA = 14000;

	pr_info("%s: core regulator %d mA\n", __func__, regulator_mA);
	tegra_init_core_edp_limits(regulator_mA);

	return 0;
}

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
};

static struct tegra_tsensor_pmu_data tpdata_palmas = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x58,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0xa0,
	.poweroff_reg_data = 0x0,
};

static struct tegra_tsensor_pmu_data tpdata_max77663 = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x3c,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0x41,
	.poweroff_reg_data = 0x80,
};

static struct soctherm_platform_data vcm30_t124_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
		},
	},
	.throttle = {
		[THROTTLE_HEAVY] = {
			.priority = 100,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 80,
				},
			},
		},
	},
	.tshut_pmu_trip_data = &tpdata_palmas,
};

/* FIXME: Needed? */
int __init vcm30_t124_soctherm_init(void)
{

	vcm30_t124_soctherm_data.tshut_pmu_trip_data = &tpdata_max77663;

	tegra_platform_edp_init(vcm30_t124_soctherm_data.therm[THERM_CPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_CPU].num_trips,
			8000); /* edp temperature margin */
	tegra_add_tj_trips(vcm30_t124_soctherm_data.therm[THERM_CPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_CPU].num_trips);
	/*tegra_add_vc_trips(vcm30_t124_soctherm_data.therm[THERM_CPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_CPU].num_trips);
*/
	return tegra11_soctherm_init(&vcm30_t124_soctherm_data);
}
