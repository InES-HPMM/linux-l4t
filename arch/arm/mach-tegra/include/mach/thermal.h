/*
 * arch/arm/mach-tegra/thermal.h
 *
 * Copyright (C) 2010-2012 NVIDIA Corporation.
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

#ifndef __MACH_THERMAL_H
#define __MACH_THERMAL_H

#include <linux/therm_est.h>
#include <linux/thermal.h>

struct tegra_cooling_device {
	char *cdev_type;
	int *trip_temperatures;
	int trip_temperatures_num;
};

#define MAX_THROT_TABLE_SIZE	(64)

struct throttle_table {
	unsigned int cpu_freq;
	int core_cap_level;
};

struct balanced_throttle {
	struct thermal_cooling_device *cdev;
	struct list_head node;
	int is_throttling;
	int throttle_count;
	int throttle_index;
	int throt_tab_size;
	struct throttle_table *throt_tab;
};

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
int tegra_throttle_init(struct mutex *cpu_lock);
struct thermal_cooling_device *balanced_throttle_register(
		struct balanced_throttle *bthrot,
		char *type);
void tegra_throttle_exit(void);
bool tegra_is_throttling(int *count);
unsigned int tegra_throttle_governor_speed(unsigned int requested_speed);
#else
static inline int tegra_throttle_init(struct mutex *cpu_lock)
{ return 0; }
static inline struct thermal_cooling_device *balanced_throttle_register(
		struct balanced_throttle *bthrot,
		char *type)
{ return ERR_PTR(-ENODEV); }
static inline void tegra_throttle_exit(void)
{}
static inline bool tegra_is_throttling(int *count)
{ return false; }
static inline unsigned int tegra_throttle_governor_speed(
	unsigned int requested_speed)
{ return requested_speed; }
#endif /* CONFIG_TEGRA_THERMAL_THROTTLE */

#endif	/* __MACH_THERMAL_H */
