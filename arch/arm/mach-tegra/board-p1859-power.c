/*
 * arch/arm/mach-tegra/board-p1859-power.c
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
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/tegra-pmc.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"
#include "board-common.h"
#include "board-p1859.h"
#include "devices.h"
#include <mach/board_id.h>
#include "vcm30_t124.h"

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
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

int __init p1859_regulator_init(void)
{
	int sku_rev;
	sku_rev = tegra_board_get_skurev("61859");

	/* C01 boards have tegra gpio for gpu_pwr_req and
	 *   boards before C01 have PMU gpio for gpu_pwr_req
	 */
	if (sku_rev < 300) {
		max77663_pdata.num_gpio_cfgs = ARRAY_SIZE(max77663_gpio_cfgs);
		max77663_pdata.gpio_cfgs = max77663_gpio_cfgs;
	}

	i2c_register_board_info(4, max77663_regulators,
				ARRAY_SIZE(max77663_regulators));

	return 0;
}

