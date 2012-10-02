/*
 * arch/arm/mach-tegra/tegra3_tsensor.h
 *
 * Tegra tsensor header file
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_TEGRA3_SOCTHERM_H
#define __MACH_TEGRA_TEGRA3_SOCTHERM_H

#include <linux/thermal.h>

enum soctherm_sense {
	TSENSE_CPU0 = 0,
	TSENSE_CPU1 = 1,
	TSENSE_CPU2 = 2,
	TSENSE_CPU3 = 3,
	TSENSE_MEM0 = 4,
	TSENSE_MEM1 = 5,
	TSENSE_GPU  = 6,
	TSENSE_PLLX = 7,
	TSENSE_SIZE = 8,
};

enum soctherm_therm_id {
	THERM_CPU = 0,
	THERM_MEM = 1,
	THERM_GPU = 2,
	THERM_PLL = 3,
	THERM_SIZE = 4,
};

struct soctherm_sensor {
	bool enable;
	int tall;
	int tiddq;
	int ten_count;
	int tsample;
	s16 therm_a;
	s16 therm_b;
	u8 pdiv;
};

struct soctherm_therm {
	s8 thermtrip;
	s8 hw_backstop;

	struct thermal_cooling_device *cdev;
	int trip_temp;
	int tc1;
	int tc2;
	int passive_delay;
};

enum soctherm_throttle_id {
	THROTTLE_LITE = 0,
	THROTTLE_HEAVY = 1,
	THROTTLE_SIZE = 2,
};

enum soctherm_throttle_dev_id {
	THROTTLE_DEV_CPU = 0,
	THROTTLE_DEV_GPU = 1,
	THROTTLE_DEV_SIZE = 2,
};

struct soctherm_throttle_dev {
	bool enable;
	u8 dividend;
	u8 divisor;
	u16 duration;
	u8 step;
};

struct soctherm_throttle {
	u8 priority;
	struct soctherm_throttle_dev devs[THROTTLE_DEV_SIZE];
};

struct soctherm_platform_data {
	struct soctherm_sensor sensor_data[TSENSE_SIZE];
	struct soctherm_therm therm[THERM_SIZE];
	struct soctherm_throttle throttle[THROTTLE_SIZE];

	int edp_weights[12];
	int edp_threshold;
};

#ifdef CONFIG_TEGRA_SOCTHERM
int __init tegra11_soctherm_init(struct soctherm_platform_data *data);
#else
static inline int tegra11_soctherm_init(struct soctherm_platform_data *data)
{
	return 0;
}
#endif

#endif
