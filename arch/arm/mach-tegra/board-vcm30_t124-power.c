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

#include <mach/edp.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"
#include "board-common.h"
#include "board-vcm30_t124.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"
#include "tegra11_soctherm.h"
#include "tegra3_tsensor.h"
#include <mach/board_id.h>

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)


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
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddcpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x3a),
		.platform_data  = &max15569_vddcpu_pdata,
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
	.slew_rate_mv_per_us = 44,
};

static struct i2c_board_info __initdata max15569_vddgpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("max15569", 0x38),
		.platform_data  = &max15569_vddgpu_pdata,
	},
};

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
	vcm30_t124_max77663_regulator_init();
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
	tegra_add_cpu_vmax_trips(vcm30_t124_soctherm_data.therm[THERM_CPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_CPU].num_trips);
	/*tegra_add_vc_trips(vcm30_t124_soctherm_data.therm[THERM_CPU].trips,
			&vcm30_t124_soctherm_data.therm[THERM_CPU].num_trips);
*/
	return tegra11_soctherm_init(&vcm30_t124_soctherm_data);
}

/*
 * GPIO init table for PCA9539 MISC IO GPIOs
 * that have to be brought up to a known good state
 * except for WiFi as it is handled via the
 * WiFi stack.
 */
static struct gpio vcm30_t124_system_0_gpios[] = {
	{MISCIO_BT_WAKEUP_GPIO,	GPIOF_OUT_INIT_HIGH,	"bt_wk"},
	{MISCIO_ABB_RST_GPIO,	GPIOF_OUT_INIT_HIGH,	"ebb_rst"},
	{MISCIO_USER_LED2_GPIO,	GPIOF_OUT_INIT_LOW,	"usr_led2"},
	{MISCIO_USER_LED1_GPIO, GPIOF_OUT_INIT_LOW,	"usr_led1"},
};

/*
 * GPIO init table for PCA9539 MISC IO GPIOs
 * related to DAP_D_SEL and DAP_D_EN.
 */
static struct gpio vcm30_t124_system_1_gpios[] = {
	{MISCIO_MUX_DAP_D_SEL,	GPIOF_OUT_INIT_LOW,	"dap_d_sel"},
	{MISCIO_MUX_DAP_D_EN,	GPIOF_OUT_INIT_LOW,	"dap_d_en"},
};

static int __init vcm30_t124_system_0_gpio_init(void)
{
	int ret, pin_count = 0;
	struct gpio *gpios_info = NULL;
	gpios_info = vcm30_t124_system_0_gpios;
	pin_count = ARRAY_SIZE(vcm30_t124_system_0_gpios);

	/* Set required system GPIOs to initial bootup values */
	ret = gpio_request_array(gpios_info, pin_count);

	if (ret)
		pr_err("%s gpio_request_array failed(%d)\r\n",
				 __func__, ret);

	/* Export the LED GPIOs to userland for any check */
	gpio_export(MISCIO_USER_LED2_GPIO, false);
	gpio_export(MISCIO_USER_LED1_GPIO, false);

	return ret;
}

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
	case 0:
		ret = vcm30_t124_system_0_gpio_init();
		break;
	case 1:
		ret = vcm30_t124_system_1_gpio_init();
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


static struct pca953x_platform_data vcm30_t124_miscio_0_pca9539_data = {
	.gpio_base  = PCA953X_MISCIO_0_GPIO_BASE,
	.setup = pca953x_client_setup,
	.context = (void *)0,
};

static struct pca953x_platform_data vcm30_t124_miscio_1_pca9539_data = {
	.gpio_base  = PCA953X_MISCIO_1_GPIO_BASE,
	.setup = pca953x_client_setup,
	.context = (void *)1,
};

static struct i2c_board_info vcm30_t124_i2c2_board_info_pca9539_0 = {
	I2C_BOARD_INFO("pca9539", PCA953X_MISCIO_0_ADDR),
	.platform_data = &vcm30_t124_miscio_0_pca9539_data,
};

static struct i2c_board_info vcm30_t124_i2c2_board_info_pca9539_1 = {
	I2C_BOARD_INFO("pca9539", PCA953X_MISCIO_1_ADDR),
	.platform_data = &vcm30_t124_miscio_1_pca9539_data,
};

int __init vcm30_t124_pca953x_init(void)
{
	int is_e1860_b00 = 0;

	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);

	i2c_register_board_info(1, &vcm30_t124_i2c2_board_info_pca9539_0, 1);

	if (is_e1860_b00) {
		i2c_register_board_info(1,
			&vcm30_t124_i2c2_board_info_pca9539_1, 1);
	}

	return 0;
}
