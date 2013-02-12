/*
 * arch/arm/mach-tegra/include/mach/edp.h
 *
 * Copyright (c) 2011-2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MACH_EDP_H
#define __MACH_EDP_H

#include <linux/debugfs.h>
#include <linux/edp.h>
#include <linux/thermal.h>
#include <linux/platform_data/thermal_sensors.h>

struct tegra_edp_vdd_cpu_entry {
	char speedo_id;
	char regulator_100mA;
	char temperature;
	char freq_limits[4];
};

struct tegra_edp_limits {
	int temperature;
	unsigned int freq_limits[4];
};

struct tegra_system_edp_entry {
	char speedo_id;
	char power_limit_100mW;
	unsigned int freq_limits[4];
};

struct tegra_edp_cpu_leakage_params {
	int cpu_speedo_id;
	int dyn_consts_n[4];	 /* pre-multiplied by 1,000,000 */
	int leakage_consts_n[4];	 /* pre-multiplied by 1,000,000 */
	int leakage_consts_ijk[4][4][4]; /* pre-multiplied by 100,000 */
	unsigned int safety_cap[4];
};

struct tegra_edp_freq_voltage_table {
	unsigned int freq;
	int voltage_mV;
};

enum tegra_core_edp_profiles {
	CORE_EDP_PROFILE_FAVOR_EMC = 0,
	CORE_EDP_PROFILE_BALANCED,
	CORE_EDP_PROFILE_FAVOR_GPU,

	CORE_EDP_PROFILES_NUM,
};

struct tegra_core_edp_limits {
	int sku;
	struct clk **cap_clocks;
	int cap_clocks_num;
	int *temperatures;
	int temperature_ranges;
	int core_modules_states;
	unsigned long *cap_rates_scpu_on;
	unsigned long *cap_rates_scpu_off;
};

#ifdef CONFIG_TEGRA_EDP_LIMITS
struct thermal_cooling_device *edp_cooling_device_create(void *v);
void tegra_init_cpu_edp_limits(unsigned int regulator_mA);
void tegra_recalculate_cpu_edp_limits(void);
void tegra_init_system_edp_limits(unsigned int power_limit_mW);
void tegra_get_cpu_edp_limits(const struct tegra_edp_limits **limits, int *size);
unsigned int tegra_get_edp_limit(int *get_edp_thermal_index);
void tegra_get_system_edp_limits(const unsigned int **limits);
int tegra_system_edp_alarm(bool alarm);
void tegra_platform_edp_init(struct thermal_trip_info *trips,
					int *num_trips, int margin);
struct tegra_system_edp_entry *tegra_get_system_edp_entries(int *size);
#else
static inline struct thermal_cooling_device *edp_cooling_device_create(
	int index)
{ return NULL; }
static inline void tegra_init_cpu_edp_limits(int regulator_mA)
{}
static inline void tegra_recalculate_cpu_edp_limits(void)
{}
static inline void tegra_init_system_edp_limits(int power_limit_mW)
{}
static inline void tegra_get_cpu_edp_limits(struct tegra_edp_limits **limits,
					    int *size)
{}
static inline unsigned int tegra_get_edp_limit(int *get_edp_thermal_index)
{ return -1; }
static inline void tegra_get_system_edp_limits(unsigned int **limits)
{}
static inline int tegra_system_edp_alarm(bool alarm)
{ return -1; }
static inline void tegra_platform_edp_init(struct thermal_trip_info *trips,
					   int *num_trips, int margin)
{}
static inline struct tegra_system_edp_entry
		*tegra_get_system_edp_entries(int *size) { return NULL; }
#endif

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static inline void tegra_edp_throttle_cpu_now(u8 factor)
{}
#else
void tegra_edp_throttle_cpu_now(u8 factor);
#endif

#ifdef CONFIG_TEGRA_CORE_EDP_LIMITS
void tegra_init_core_edp_limits(unsigned int regulator_mA);
int tegra_core_edp_debugfs_init(struct dentry *edp_dir);
int tegra_core_edp_cpu_state_update(bool scpu_state);
struct tegra_cooling_device *tegra_core_edp_get_cdev(void);
#else
static inline void tegra_init_core_edp_limits(unsigned int regulator_mA)
{}
static inline int tegra_core_edp_debugfs_init(struct dentry *edp_dir)
{ return 0; }
static inline int tegra_core_edp_cpu_state_update(bool scpu_state)
{ return 0; }
static inline struct tegra_cooling_device *tegra_core_edp_get_cdev(void)
{ return NULL; }
#endif

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
int tegra11x_select_core_edp_table(unsigned int regulator_mA,
				   struct tegra_core_edp_limits *limits);
#else
static inline int tegra11x_select_core_edp_table(
	unsigned int regulator_mA, struct tegra_core_edp_limits *limits)
{ return -ENOSYS; }
#endif

#endif	/* __MACH_EDP_H */
