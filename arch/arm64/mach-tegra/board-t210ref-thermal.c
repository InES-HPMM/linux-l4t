/*
 * arch/arm/mach-tegra/board-t210ref-power.c
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
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <mach/edp.h>
#include <mach/irqs.h>
#include <linux/edp.h>
#include <linux/of_platform.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/pid_thermal_gov.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/tegra-dfll-bypass-regulator.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-pmc.h>
#include <linux/tegra_soctherm.h>
#include <linux/pinctrl/pinconf-tegra.h>
#include <linux/nct1008.h>

#include <asm/mach-types.h>

#include "pm.h"
#include <linux/platform/tegra/dvfs.h>
#include "board.h"
#include <linux/platform/tegra/common.h>
#include "tegra-board-id.h"
#include "board-pmu-defines.h"
#include "board-common.h"
#include "board-t210ref.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include "iomap.h"
#include <linux/platform/tegra/tegra_cl_dvfs.h>
#include <linux/platform/tegra/cpu-tegra.h>

static u32 tegra_chip_id;

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

static struct tegra_thermtrip_pmic_data tpdata_max77620 = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x3c,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0x41,
	.poweroff_reg_data = 0x80,
};

static struct soctherm_platform_data t210ref_soctherm_data = {
	.oc_irq_base = TEGRA_SOC_OC_IRQ_BASE,
	.num_oc_irqs = TEGRA_SOC_OC_NUM_IRQ,
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
#if 0 /* no trip points yet */
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
#endif
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
#if 0 /* no trip points yet */
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "gpu-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
#endif
		},
		[THERM_MEM] = {
			.zone_enable = true,
#if 0 /* no trip points yet */
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000, /* = GPU shut */
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
#endif
		},
		[THERM_PLL] = {
			.zone_enable = true,
			.tzp = &soctherm_tzp,
		},
	},
	.throttle = {
#if 0 /* no HW heavy throttling yet */
		[THROTTLE_HEAVY] = {
			.priority = 100,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 80,
					.throttling_depth = "heavy_throttling",
				},
				[THROTTLE_DEV_GPU] = {
					.enable = true,
					.throttling_depth = "heavy_throttling",
				},
			},
		},
#endif
	},
};

int __init t210ref_soctherm_init(void)
{
	int cp_rev;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	tegra_chip_id = tegra_get_chip_id();

	cp_rev = tegra_fuse_calib_base_get_cp(NULL, NULL);
	if (cp_rev < 0) {
		pr_info("%s: ERROR: cp_rev %d.\n", __func__, cp_rev);
		return -EINVAL;
	}

	t210ref_soctherm_data.tshut_pmu_trip_data = &tpdata_max77620;

	return tegra_soctherm_init(&t210ref_soctherm_data);
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
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	int i;
	struct thermal_trip_info *trip_state;
#endif
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* raise NCT's thresholds if soctherm CP,FT fuses are ok */
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	if ((tegra_fuse_calib_base_get_cp(NULL, NULL) >= 0) &&
	    (tegra_fuse_calib_base_get_ft(NULL, NULL) >= 0)) {
		t210ref_nct72_pdata.sensors[EXT].shutdown_limit += 20;
		for (i = 0; i < t210ref_nct72_pdata.sensors[EXT].num_trips;
			 i++) {
			trip_state = &t210ref_nct72_pdata.sensors[EXT].trips[i];
			if (!strncmp(trip_state->cdev_type, "cpu-balanced",
					THERMAL_NAME_LENGTH)) {
				trip_state->cdev_type = "_none_";
				break;
			}
		}
	} else {
#endif
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
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	}
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
