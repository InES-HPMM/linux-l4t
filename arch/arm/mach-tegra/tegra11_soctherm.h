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

struct soctherm_cdev {
	struct thermal_cooling_device *cdev;
	int trip_temp;
	int tc1;
	int tc2;
	int passive_delay;
};

struct soctherm_sensor {
	int tall;
	int tiddq;
	int ten_count;
	int tsample;
	int therm_a;
	int therm_b;
};

struct soctherm_platform_data {
	int therm_trip; /* in celcius */

	int hw_backstop; /* in celcius */
	int dividend;
	int divisor;
	int duration;
	int step;

	struct soctherm_sensor sensor;
	struct soctherm_cdev passive;
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
