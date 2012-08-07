/*
 * arch/arm/mach-tegra/board-dalmore-power.c
 *
 * Copyright (C) 2012 NVIDIA Corporation.
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
#include <linux/regulator/driver.h>
#include <linux/mfd/max77663-core.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/regulator/tps51632-regulator.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>

#include "pm.h"
#include "board.h"
#include "board-dalmore.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_core", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu", NULL),
	REGULATOR_SUPPLY("avdd_hdmmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_sensor_2v8", NULL),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb3_pll", NULL),
	REGULATOR_SUPPLY("avddio_usb3", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_cam1", NULL),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_cam2", NULL),
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

#define MAX77663_PDATA_INIT(_id, _min_uV, _max_uV, _supply_reg,		\
		_always_on, _boot_on, _apply_uV,			\
		_init_apply, _init_enable, _init_uV,			\
		_fps_src, _fps_pu_period, _fps_pd_period, _flags)	\
static struct max77663_regulator_platform_data max77663_regulator_pdata_##_id =\
{									\
	.init_data = {							\
			.constraints = {				\
				.min_uV = _min_uV,			\
				.max_uV = _max_uV,			\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uV,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(max77663_##_id##_supply),	\
			.consumer_supplies = max77663_##_id##_supply,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_apply = _init_apply,				\
		.init_enable = _init_enable,				\
		.init_uV = _init_uV,					\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.fps_cfgs = max77663_fps_cfgs,				\
		.flags = _flags,					\
	}

MAX77663_PDATA_INIT(sd0,  600000, 3387500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(sd1,  800000, 1587500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(sd2,  1800000, 1800000, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(sd3,  600000, 3387500, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo0, 800000, 2350000, max77663_rails(sd2), 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(ldo1, 800000, 2350000, max77663_rails(sd2), 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo2, 2850000, 2850000, NULL, 1, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(ldo3, 800000, 3950000, max77663_rails(sd2), 1, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo4, 800000, 1587500, NULL, 0, 0, 0,
		    1, 1, 1000000, FPS_SRC_NONE, -1, -1, LDO4_EN_TRACKING);

MAX77663_PDATA_INIT(ldo5, 800000, 2800000, max77663_rails(sd2), 0, 0, 0,
		    1, 1, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo6, 800000, 3950000, NULL, 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo7, 800000, 3950000, NULL, 0, 0, 0,
		    0, 0, -1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(ldo8, 800000, 3950000, NULL, 0, 0, 0,
		    1, 1, -1, FPS_SRC_1, -1, -1, 0);

#define MAX77663_REG(_id, _data)					\
	{								\
		.name = "max77663-regulator",				\
		.id = MAX77663_REGULATOR_ID_##_id,			\
		.platform_data = &max77663_regulator_pdata_##_data,	\
		.pdata_size = sizeof(max77663_regulator_pdata_##_data),	\
	}

static struct mfd_cell max77663_subdevs[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
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
		.gpio = MAX77663_GPIO1,
		.dir = GPIO_DIR_IN,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.pull_up = GPIO_PU_ENABLE,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO2,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.pull_up = GPIO_PU_ENABLE,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO3,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.pull_up = GPIO_PU_ENABLE,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO4,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_ENABLE,
	},
	{
		.gpio = MAX77663_GPIO5,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO6,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO7,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
};

static struct max77663_platform_data max7763_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs	= ARRAY_SIZE(max77663_gpio_cfgs),
	.gpio_cfgs	= max77663_gpio_cfgs,


	.num_subdevs	= ARRAY_SIZE(max77663_subdevs),
	.sub_devices	= max77663_subdevs,

	.rtc_i2c_addr	= 0x68,

	.use_power_off	= false,
};


static struct i2c_board_info __initdata max77663_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x3c),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max7763_pdata,
	},
};

/* TPS51632 DC-DC converter */
static struct regulator_consumer_supply tps51632_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_init_data tps51632_init_data = {
	.constraints = {						\
		.min_uV = 500000,					\
			.max_uV = 1520000,				\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = 1,					\
			.boot_on =  1,					\
			.apply_uV = 0,					\
	},								\
	.num_consumer_supplies = ARRAY_SIZE(tps51632_dcdc_supply),	\
		.consumer_supplies = tps51632_dcdc_supply,		\
};

static struct tps51632_regulator_platform_data tps51632_pdata = {
	.reg_init_data = &tps51632_init_data,		\
	.enable_pwm = false,				\
	.max_voltage_uV = 1520000,			\
	.base_voltage_uV = 500000,			\
	.slew_rate_uv_per_us = 6000,			\
};

static struct i2c_board_info __initdata tps51632_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps51632", 0x43),
		.platform_data	= &tps51632_pdata,
	},
};

static int ac_online(void)
{
	return 1;
}

static struct resource dalmore_pda_resources[] = {
	[0] = {
		.name	= "ac",
	},
};

static struct pda_power_pdata dalmore_pda_data = {
	.is_ac_online	= ac_online,
};

static struct platform_device dalmore_pda_power_device = {
	.name		= "pda-power",
	.id		= -1,
	.resource	= dalmore_pda_resources,
	.num_resources	= ARRAY_SIZE(dalmore_pda_resources),
	.dev	= {
		.platform_data	= &dalmore_pda_data,
	},
};

static struct tegra_suspend_platform_data dalmore_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= false,
	.sysclkreq_high	= true,
};

static int __init dalmore_max77663_regulator_init(void)
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

int __init dalmore_regulator_init(void)
{
	dalmore_max77663_regulator_init();
	i2c_register_board_info(4, tps51632_boardinfo, 1);
	platform_device_register(&dalmore_pda_power_device);
	return 0;
}

int __init dalmore_suspend_init(void)
{
	tegra_init_suspend(&dalmore_suspend_data);
	return 0;
}

