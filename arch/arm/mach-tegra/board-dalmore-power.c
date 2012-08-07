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
#include <linux/regulator/tps51632-regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "pm.h"
#include "board.h"
#include "board-dalmore.h"


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


int __init dalmore_regulator_init(void)
{
	i2c_register_board_info(4, tps51632_boardinfo, 1);
	platform_device_register(&dalmore_pda_power_device);
	return 0;
}

int __init dalmore_suspend_init(void)
{
	tegra_init_suspend(&dalmore_suspend_data);
	return 0;
}

