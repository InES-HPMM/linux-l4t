/*
 * arch/arm/mach-tegra/board-vcm30_t124-power.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/io.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/regulator/max15569-regulator.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/tegra-pmc.h>
#include <linux/pid_thermal_gov.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"
#include "board-common.h"
#include "board-vcm30_t124.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"
#include "tegra11_soctherm.h"
#include <mach/board_id.h>

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
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

MAX77663_PDATA_INIT(LDO5, ldo5, 800000, 3950000, NULL, 1, 1, 0,
		FPS_SRC_1, FPS_POWER_PERIOD_7, FPS_POWER_PERIOD_0, 0);

#define MAX77663_REG(_id, _data) (&max77663_regulator_pdata_##_data)

static struct max77663_regulator_platform_data *max77663_reg_pdata[] = {
	MAX77663_REG(LDO5, ldo5),
};

static struct max77663_gpio_config max77663_gpio_cfgs[] = {
	{
		.gpio = MAX77663_GPIO5,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
};

static struct max77663_platform_data max77663_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,
	.regulator_pdata = max77663_reg_pdata,
	.num_regulator_pdata = ARRAY_SIZE(max77663_reg_pdata),
	.rtc_i2c_addr	= 0x68,
	.use_power_off	= false,
};

static struct i2c_board_info __initdata max77663_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x3c),
		.irq		= -1,
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
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddcpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x3a),
		.platform_data  = &max15569_vddcpu_pdata,
	},
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

static struct tegra_cl_dvfs_platform_data vcm30_t124_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0x74,
		.reg = 0x07,
	},
	.vdd_map = pmu_cpu_vdd_map,
	.vdd_map_size = PMU_CPU_VDD_MAP_SIZE,
	.flags = TEGRA_CL_DVFS_DYN_OUTPUT_CFG,
	.cfg_param = &vcm30_t124_cl_dvfs_param,
};

static int __init vcm30_t124_cl_dvfs_init(void)
{
	fill_reg_map();
	tegra_cl_dvfs_device.dev.platform_data = &vcm30_t124_cl_dvfs_data;
	platform_device_register(&tegra_cl_dvfs_device);

	return 0;
}
#endif

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
		.enable_time = 210, /* for ramp up time in usec */
	},
	.num_consumer_supplies = ARRAY_SIZE(max15569_vddgpu_supply),
	.consumer_supplies = max15569_vddgpu_supply,
};

static struct max15569_regulator_platform_data max15569_vddgpu_pdata = {
	.reg_init_data = &max15569_vddgpu_init_data,
	.max_voltage_uV = 1400000,
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddgpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x38),
		.platform_data  = &max15569_vddgpu_pdata,
	},
};

int __init vcm30_t124_regulator_init(void)
{
	int sku_rev;
	sku_rev = tegra_board_get_skurev("61859");

	tegra_pmc_pmu_interrupt_polarity(true);

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	vcm30_t124_cl_dvfs_init();
#endif

	/* C01 boards have tegra gpio for gpu_pwr_req and
	 *   boards before C01 have PMU gpio for gpu_pwr_req
	 */
	if (sku_rev >= 300) {
		max15569_vddgpu_pdata.ena_gpio = TEGRA_GPIO_PR2;
	} else {
		max77663_pdata.num_gpio_cfgs = ARRAY_SIZE(max77663_gpio_cfgs);
		max77663_pdata.gpio_cfgs = max77663_gpio_cfgs;
	}

	i2c_register_board_info(4, max77663_regulators,
				ARRAY_SIZE(max77663_regulators));

	i2c_register_board_info(4, max15569_vddcpu_boardinfo, 1);
	i2c_register_board_info(4, max15569_vddgpu_boardinfo, 1);

	return 0;
}

static struct tegra_suspend_platform_data vcm30_t124_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 2000,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0xfefe,
	.core_off_timer = 2000,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 1000,
};

int __init vcm30_t124_suspend_init(void)
{
	tegra_init_suspend(&vcm30_t124_suspend_data);
	return 0;
}

static struct pid_thermal_gov_params soctherm_pid_params = {
	.max_err_temp = 9000,
	.max_err_gain = 1000,
	.gain_p = 1000,
	.gain_d = 0,
	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &soctherm_pid_params,
};

static struct tegra_thermtrip_pmic_data tpdata_max77663 = {
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
			.num_trips = 0,
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 0,
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
		},
	},
};

/* FIXME: Needed? */
int __init vcm30_t124_soctherm_init(void)
{
	vcm30_t124_soctherm_data.tshut_pmu_trip_data = &tpdata_max77663;

	tegra_add_cpu_clk_switch_trips(
			vcm30_t124_soctherm_data.therm[THERM_CPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_CPU].num_trips);

	tegra_add_tgpu_trips(
			vcm30_t124_soctherm_data.therm[THERM_GPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_GPU].num_trips);

	return tegra11_soctherm_init(&vcm30_t124_soctherm_data);
}

/*
 * GPIO init table for PCA9539 MISC IO GPIOs
 * related to DAP_D_SEL and DAP_D_EN.
 */
static struct gpio vcm30_t124_system_1_gpios[] = {
	{MISCIO_MUX_DAP_D_SEL,	GPIOF_OUT_INIT_LOW,	"dap_d_sel"},
	{MISCIO_MUX_DAP_D_EN,	GPIOF_OUT_INIT_LOW,	"dap_d_en"},
};

static struct gpio vcm30_t124_system_2_gpios[] = {
	{MISCIO_MDM_EN,		GPIOF_OUT_INIT_HIGH,	"mdm_en"},
	{MISCIO_MDM_COLDBOOT,	GPIOF_IN,		"mdm_coldboot"},
	{MISCIO_AP_MDM_RESET,	GPIOF_OUT_INIT_HIGH|GPIOF_EXPORT, "ap_mdm_rst"}
};

static int __init vcm30_t124_system_1_gpio_init(void)
{
	int ret, pin_count = 0;
	struct gpio *gpios_info = NULL;
	gpios_info = vcm30_t124_system_1_gpios;
	pin_count = ARRAY_SIZE(vcm30_t124_system_1_gpios);

	/* Set required system GPIOs to initial bootup values */
	ret = gpio_request_array(gpios_info, pin_count);

	if (ret)
		pr_err("%s gpio_request_array failed(%d)\r\n",
				 __func__, ret);

	gpio_free_array(gpios_info, pin_count);

	return ret;
}

static int __init vcm30_t124_system_2_gpio_init(void)
{
	int ret, pin_count = 0;
	struct gpio *gpios_info = NULL;
	gpios_info = vcm30_t124_system_2_gpios;
	pin_count = ARRAY_SIZE(vcm30_t124_system_2_gpios);

	/* Set required system GPIOs to initial bootup values */
	ret = gpio_request_array(gpios_info, pin_count);

	if (ret)
		pr_err("%s gpio_request_array failed(%d)\r\n",
				 __func__, ret);

	/* We need to keep MDM_RESET exported, controled by modem RIL*/
	while (pin_count--) {
		if (gpios_info->gpio != MISCIO_AP_MDM_RESET)
			gpio_free(gpios_info->gpio);

		gpios_info++;
	}

	return ret;
}


/*
 * TODO: Check for the correct pca953x before invoking client
 *  init functions
 */
static int pca953x_client_setup(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context)
{
	int ret = 0;
	int system = (int)context;

	switch (system) {
	case 1:
		ret = vcm30_t124_system_1_gpio_init();
		break;
	case 2:
		ret = vcm30_t124_system_2_gpio_init();
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		goto fail;

	return 0;
fail:
	pr_err("%s failed(%d)\r\n", __func__, ret);
	return ret;
}

static struct pca953x_platform_data vcm30_t124_miscio_1_pca9539_data = {
	.gpio_base  = PCA953X_MISCIO_1_GPIO_BASE,
	.setup = pca953x_client_setup,
	.context = (void *)1,
};

static struct pca953x_platform_data vcm30_t124_miscio_2_pca9539_data = {
	.gpio_base  = PCA953X_MISCIO_2_GPIO_BASE,
	.setup = pca953x_client_setup,
	.context = (void *)2,
};

static struct i2c_board_info vcm30_t124_i2c2_board_info_pca9539_1 = {
	I2C_BOARD_INFO("pca9539", PCA953X_MISCIO_1_ADDR),
	.platform_data = &vcm30_t124_miscio_1_pca9539_data,
};

static struct i2c_board_info vcm30_t124_i2c2_board_info_pca9539_2 = {
	I2C_BOARD_INFO("pca9539", PCA953X_MISCIO_2_ADDR),
	.platform_data = &vcm30_t124_miscio_2_pca9539_data,
};

int __init vcm30_t124_pca953x_init(void)
{
	int is_e1860_b00 = 0;

	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);

	if (is_e1860_b00) {

		i2c_register_board_info(1,
			&vcm30_t124_i2c2_board_info_pca9539_1, 1);

	/* some pins of vcm30_t124_i2c2_board_info_pca9539_2 may be used
	later on some other board versions, not only b00.*/
		i2c_register_board_info(1,
			&vcm30_t124_i2c2_board_info_pca9539_2, 1);
	}
	return 0;
}
