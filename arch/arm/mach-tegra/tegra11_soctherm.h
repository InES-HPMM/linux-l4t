/*
 * arch/arm/mach-tegra/tegra11_soctherm.h
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

enum soctherm_sense {
	TSENSE_CPU0 = 0,
	TSENSE_CPU1,
	TSENSE_CPU2,
	TSENSE_CPU3,
	TSENSE_MEM0,
	TSENSE_MEM1,
	TSENSE_GPU,
	TSENSE_PLLX,
	TSENSE_SIZE,
};

enum soctherm_therm_id {
	THERM_CPU = 0,
	THERM_MEM,
	THERM_GPU,
	THERM_PLL,
	THERM_SIZE,
};

struct soctherm_sensor {
	bool sensor_enable;
	bool zone_enable;
	int tall;
	int tiddq;
	int ten_count;
	int tsample;
	u8 pdiv;
};

struct soctherm_therm {
	bool zone_enable;
	s8 thermtrip;
	s8 hw_backstop;

	char *cdev_type;
	int trip_temp;
	int passive_delay;
	int etemp;
	int hysteresis;
};

enum soctherm_throttle_id {
	THROTTLE_LITE = 0,
	THROTTLE_HEAVY,
	THROTTLE_SIZE,
};

enum soctherm_throttle_dev_id {
	THROTTLE_DEV_CPU = 0,
	THROTTLE_DEV_GPU,
	THROTTLE_DEV_SIZE,
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
	unsigned long soctherm_clk_rate;
	unsigned long tsensor_clk_rate;

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
