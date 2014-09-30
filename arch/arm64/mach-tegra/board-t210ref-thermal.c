/*
 * arch/arm/mach-tegra/board-t210ref-thermal.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/pid_thermal_gov.h>
#include <linux/gpio.h>
#include <linux/nct1008.h>

#include <mach/edp.h>

#include "board.h"
#include "tegra-board-id.h"
#include "board-t210ref.h"

int __init t210ref_edp_init(void)
{
	unsigned int regulator_mA;
	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 14000;
	}

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	/* gpu maximum current */
	regulator_mA = 12000;
	pr_info("%s: GPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_gpu_edp_limits(regulator_mA);

	return 0;
}

static struct pid_thermal_gov_params cpu_pid_params = {
	.max_err_temp = 10000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params cpu_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &cpu_pid_params,
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_zone_params board_tzp = {
	.governor_name = "pid_thermal_gov"
};
#endif

static struct nct1008_platform_data t210ref_nct72_pdata = {
	.loc_name = "tegra",
	.supported_hwrev = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.extended_range = true,

	.sensors = {
		[LOC] = {
			.shutdown_limit = 120, /* C */
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
			.tzp = &board_tzp,
			.passive_delay = 1000,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "therm_est_activ",
					.trip_temp = 40000,
					.trip_type = THERMAL_TRIP_ACTIVE,
					.hysteresis = 1000,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 1,
				},
			},
#endif
		},
		[EXT] = {
			.tzp = &cpu_tzp,
			.shutdown_limit = 93, /* C */
			.passive_delay = 1000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 92000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 0,
				},
				{
					.cdev_type = "shutdown_warning",
					.trip_temp = 85000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 0,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 82000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.hysteresis = 1000,
					.mask = 1,
				},
			}
		}
	}
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct pid_thermal_gov_params skin_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params skin_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &skin_pid_params,
};

static struct nct1008_platform_data t210ref_nct72_tskin_pdata = {
	.loc_name = "skin",

	.supported_hwrev = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.extended_range = true,

	.sensors = {
		[LOC] = {
			.shutdown_limit = 95, /* C */
			.num_trips = 0,
			.tzp = NULL,
		},
		[EXT] = {
			.shutdown_limit = 85, /* C */
			.passive_delay = 10000,
			.polling_delay = 1000,
			.tzp = &skin_tzp,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "skin-balanced",
					.trip_temp = 50000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 1,
				},
			},
		}
	}
};
#endif

static struct i2c_board_info t210ref_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4c),
		.platform_data = &t210ref_nct72_pdata,
		.irq = -1,
	},
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct i2c_board_info t210ref_skin_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4d),
		.platform_data = &t210ref_nct72_tskin_pdata,
		.irq = -1,
	}
};
#endif

int __init t210ref_thermal_sensors_init(void)
{
	int nct72_port = TEGRA_GPIO_PX4;
	int ret = 0;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

#if 0 /* EDP and DVFS thermals NOT YET ENABLED on T210 */
	tegra_platform_edp_init(
		t210ref_nct72_pdata.sensors[EXT].trips,
		&t210ref_nct72_pdata.sensors[EXT].num_trips,
				12000); /* edp temperature margin */
	tegra_add_cpu_vmax_trips(
		t210ref_nct72_pdata.sensors[EXT].trips,
		&t210ref_nct72_pdata.sensors[EXT].num_trips);
	tegra_add_tgpu_trips(
		t210ref_nct72_pdata.sensors[EXT].trips,
		&t210ref_nct72_pdata.sensors[EXT].num_trips);
	tegra_add_vc_trips(
		t210ref_nct72_pdata.sensors[EXT].trips,
		&t210ref_nct72_pdata.sensors[EXT].num_trips);
	tegra_add_core_vmax_trips(
		t210ref_nct72_pdata.sensors[EXT].trips,
		&t210ref_nct72_pdata.sensors[EXT].num_trips);
#endif

	/* T210_interposer use GPIO_PC7 for alert*/
	if (board_info.board_id == BOARD_E2141)
		nct72_port = TEGRA_GPIO_PC7;

	t210ref_i2c_nct72_board_info[0].irq = gpio_to_irq(nct72_port);

	ret = gpio_request(nct72_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct72_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct72_port)", __func__);
		gpio_free(nct72_port);
	}

	/* t210ref has thermal sensor on GEN2-I2C i.e. instance 1 */
	if (board_info.board_id == BOARD_E2141) {
	/* E2141 has thermal sensor on GEN1-I2C, skin temp sensor on GEN2-I2C */
		i2c_register_board_info(0, t210ref_i2c_nct72_board_info,
			ARRAY_SIZE(t210ref_i2c_nct72_board_info));
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
		i2c_register_board_info(1, t210ref_skin_i2c_nct72_board_info,
			ARRAY_SIZE(t210ref_skin_i2c_nct72_board_info));
#endif
	} else if (board_info.board_id == BOARD_P2530) {
	/* E2530 has thermal sensor on GEN1-I2C i.e. instance 0 */
		i2c_register_board_info(0, t210ref_i2c_nct72_board_info,
				ARRAY_SIZE(t210ref_i2c_nct72_board_info));
	} else {
	/* E2220 has thermal sensor on GEN3-I2C i.e. instance 2 */
		i2c_register_board_info(2, t210ref_i2c_nct72_board_info,
				ARRAY_SIZE(t210ref_i2c_nct72_board_info));
	}

	return ret;
}
