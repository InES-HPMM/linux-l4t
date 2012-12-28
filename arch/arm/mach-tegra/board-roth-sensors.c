/*
 * arch/arm/mach-tegra/board-roth-sensors.c
 *
 * Copyright (c) 2012 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mpu.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/nct1008.h>
#include <mach/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-common.h"
#include "board-roth.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"
#include "dvfs.h"

static struct board_info board_info;

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = 19,
	.throt_tab = {
		{      0, 1000 },
		{  51000, 1000 },
		{ 102000, 1000 },
		{ 204000, 1000 },
		{ 252000, 1000 },
		{ 288000, 1000 },
		{ 372000, 1000 },
		{ 468000, 1000 },
		{ 510000, 1000 },
		{ 612000, 1000 },
		{ 714000, 1050 },
		{ 816000, 1050 },
		{ 918000, 1050 },
		{1020000, 1100 },
		{1122000, 1100 },
		{1224000, 1100 },
		{1326000, 1100 },
		{1428000, 1100 },
		{1530000, 1100 },
	},
};

static int __init roth_throttle_init(void)
{
	if (machine_is_roth())
		balanced_throttle_register(&tj_throttle, "roth-nct");
	return 0;
}
module_init(roth_throttle_init);

static struct nct1008_platform_data roth_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 0,
	.shutdown_ext_limit = 85, /* C */
	.shutdown_local_limit = 120, /* C */
	.loc_name = "soc",

	.passive_delay = 2000,

	.num_trips = 1,
	.trips = {
		/* Thermal Throttling */
		[0] = {
			.cdev_type = "roth-nct",
			.trip_temp = 75000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.state = THERMAL_NO_LIMIT,
			.hysteresis = 0,
		},
	},
};

static struct nct1008_platform_data roth_nct1008_left_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 0,
	.loc_name = "left",
	.shutdown_ext_limit = 90, /* C */
	.shutdown_local_limit = 120, /* C */
};

static struct nct1008_platform_data roth_nct1008_right_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 0,
	.loc_name = "right",
	.shutdown_ext_limit = 90, /* C */
	.shutdown_local_limit = 120, /* C */
};

static struct i2c_board_info roth_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &roth_nct1008_pdata,
		.irq = -1,
	}
};

static struct i2c_board_info roth_i2c4_nct1008_lr_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &roth_nct1008_left_pdata,
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("nct1008", 0x4D),
		.platform_data = &roth_nct1008_right_pdata,
		.irq = -1,
	}
};

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

/* MPU board file definition	*/
static struct mpu_platform_data mpu6050_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct i2c_board_info __initdata inv_mpu6050_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu6050_gyro_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu6050_i2c2_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu6050_i2c2_board_info,
		ARRAY_SIZE(inv_mpu6050_i2c2_board_info));
}

static int roth_nct1008_init(void)
{
	int nct1008_port = TEGRA_GPIO_PX6;
	int ret = 0;
	struct nct1008_platform_data *data = &roth_nct1008_pdata;

#ifdef CONFIG_TEGRA_EDP_LIMITS
		const struct tegra_edp_limits *cpu_edp_limits;
		int cpu_edp_limits_size;
		int i;
		int trip;
		struct nct_trip_temp *trip_state;

		/* edp capping */
		tegra_get_cpu_edp_limits(&cpu_edp_limits, &cpu_edp_limits_size);

		if (cpu_edp_limits_size > MAX_THROT_TABLE_SIZE)
			BUG();

		for (i = 0; i < cpu_edp_limits_size-1; i++) {
			trip = data->num_trips;
			trip_state = &data->trips[trip];

			trip_state->cdev_type = "edp";
			trip_state->trip_temp =
					cpu_edp_limits[i].temperature * 1000;
			trip_state->trip_type = THERMAL_TRIP_ACTIVE;
			trip_state->state = i + 1;
			trip_state->hysteresis = 1000;

			data->num_trips++;

			if (data->num_trips >= NCT_MAX_TRIPS)
				BUG();
		}
#endif

	nct1008_add_cdev_trips(data, tegra_core_edp_get_cdev());
	nct1008_add_cdev_trips(data, tegra_dvfs_get_cpu_dfll_cdev());
	nct1008_add_cdev_trips(data, tegra_dvfs_get_cpu_pll_cdev());
	nct1008_add_cdev_trips(data, tegra_dvfs_get_core_cdev());

	roth_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
	pr_info("%s: roth nct1008 irq %d", __func__, \
				roth_i2c4_nct1008_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
	}

	/* roth has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, roth_i2c4_nct1008_board_info,
		ARRAY_SIZE(roth_i2c4_nct1008_board_info));

	i2c_register_board_info(1, roth_i2c4_nct1008_lr_board_info,
		ARRAY_SIZE(roth_i2c4_nct1008_lr_board_info));
	return ret;
}

static struct i2c_board_info __initdata bq20z45_pdata[] = {
	{
		I2C_BOARD_INFO("sbs-battery", 0x0B),
	},
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int tegra_skin_match(struct thermal_zone_device *thz, void *data)
{
	return strcmp((char *)data, thz->type) == 0;
}

static int tegra_skin_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, tegra_skin_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static struct therm_est_data skin_data = {
	.cdev_type = "roth-skin",
	.toffset = 9793,
	.polling_period = 1100,
	.ndevs = 2,
	.tc1 = 5,
	.tc2 = 1,
	.devs = {
			{
				.dev_data = "nct_ext",
				.get_temp = tegra_skin_get_temp,
				.coeffs = {
					2, 1, 1, 1,
					1, 1, 1, 1,
					1, 1, 1, 0,
					1, 1, 0, 0,
					0, 0, -1, -7
				},
			},
			{
				.dev_data = "nct_int",
				.get_temp = tegra_skin_get_temp,
				.coeffs = {
					-11, -7, -5, -3,
					-3, -2, -1, 0,
					0, 0, 1, 1,
					1, 2, 2, 3,
					4, 6, 11, 18
				},
			},
	},
	.trip_temp = 43000,
	.passive_delay = 5000,
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = 19,
	.throt_tab = {
		{      0, 1000 },
		{  51000, 1000 },
		{ 102000, 1000 },
		{ 204000, 1000 },
		{ 252000, 1000 },
		{ 288000, 1000 },
		{ 372000, 1000 },
		{ 468000, 1000 },
		{ 510000, 1000 },
		{ 612000, 1000 },
		{ 714000, 1050 },
		{ 816000, 1050 },
		{ 918000, 1050 },
		{1020000, 1100 },
		{1122000, 1100 },
		{1224000, 1100 },
		{1326000, 1100 },
		{1428000, 1100 },
		{1530000, 1100 },
	},
};

static int __init roth_skin_init(void)
{
	if (machine_is_roth()) {
		balanced_throttle_register(&skin_throttle, "roth-skin");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(roth_skin_init);
#endif

static int roth_fan_est_match(struct thermal_zone_device *thz, void *data)
{
	return (strcmp((char *)data, thz->type) == 0);
}

static int roth_fan_est_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, roth_fan_est_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static struct therm_fan_est_data fan_est_data = {
	.toffset = 0,
	.polling_period = 1100,
	.ndevs = 2,
	.devs = {
			{
				.dev_data = "nct_ext_soc",
				.get_temp = roth_fan_est_get_temp,
				.coeffs = {
					100, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
			{
				.dev_data = "nct_int_soc",
				.get_temp = roth_fan_est_get_temp,
				.coeffs = {
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
	},
	.active_trip_temps = {57000, 58000, 59000, 60000, 61000, 62000, 63000,
		64000, 65000, 68000},
};

static struct platform_device roth_fan_therm_est_device = {
	.name   = "therm-fan-est",
	.id     = -1,
	.num_resources  = 0,
	.dev = {
		.platform_data = &fan_est_data,
	},
};

static int __init roth_fan_est_init(void)
{
	platform_device_register(&roth_fan_therm_est_device);
	return 0;
}
int __init roth_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	err = roth_nct1008_init();
	if (err)
		return err;

	mpuirq_init();

	roth_fan_est_init();

	if (0)
		i2c_register_board_info(0, bq20z45_pdata,
			ARRAY_SIZE(bq20z45_pdata));

	return 0;
}
