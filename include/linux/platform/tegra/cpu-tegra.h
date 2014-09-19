/*
 * linux/platform/tegra/cpu-tegra.h
 *
 * Copyright (c) 2011-2014, NVIDIA Corporation. All rights reserved.
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

#ifndef __MACH_TEGRA_CPU_TEGRA_H
#define __MACH_TEGRA_CPU_TEGRA_H

#include <linux/fs.h>
#include <linux/tegra_throttle.h>

unsigned int tegra_getspeed(unsigned int cpu);
int tegra_update_cpu_speed(unsigned long rate);
int tegra_cpu_set_speed_cap(unsigned int *speed_cap);
int tegra_cpu_set_speed_cap_locked(unsigned int *speed_cap);
void tegra_cpu_set_volt_cap(unsigned int cap);
unsigned int tegra_count_slow_cpus(unsigned long speed_limit);
unsigned int tegra_get_slowest_cpu_n(void);
unsigned long tegra_cpu_lowest_speed(void);
unsigned long tegra_cpu_highest_speed(void);

#if defined(CONFIG_TEGRA_CPUQUIET) && defined(CONFIG_TEGRA_CLUSTER_CONTROL)
int tegra_auto_hotplug_init(struct mutex *cpulock);
void tegra_auto_hotplug_exit(void);
void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend);
#else
static inline int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{ return 0; }
static inline void tegra_auto_hotplug_exit(void)
{ }
static inline void tegra_auto_hotplug_governor(unsigned int cpu_freq,
					       bool suspend)
{ }
#endif

#ifdef CONFIG_CPU_FREQ
int tegra_suspended_target(unsigned int target_freq);
#else
static inline int tegra_suspended_target(unsigned int target_freq)
{ return -ENOSYS; }
#endif

#if defined(CONFIG_CPU_FREQ) && defined(CONFIG_TEGRA_EDP_LIMITS)
int tegra_update_cpu_edp_limits(void);
int tegra_cpu_reg_mode_force_normal(bool force);
unsigned int tegra_get_cpu_tegra_thermal_index(void);
int tegra_cpu_edp_get_max_state(struct thermal_cooling_device *cdev,
			    unsigned long *max_state);
int tegra_cpu_edp_get_cur_state(struct thermal_cooling_device *cdev,
			    unsigned long *cur_state);
int tegra_cpu_edp_set_cur_state(struct thermal_cooling_device *cdev,
			    unsigned long cur_state);
#else
static inline int tegra_update_cpu_edp_limits(void)
{ return 0; }
static inline int tegra_cpu_reg_mode_force_normal(bool force)
{ return 0; }
static inline unsigned int tegra_get_cpu_tegra_thermal_index(void)
{ return -1; }
static inline int tegra_cpu_edp_get_max_state(
	struct thermal_cooling_device *cdev, unsigned long *max_state)
{ return -1; }
static inline int tegra_cpu_edp_get_cur_state(
	struct thermal_cooling_device *cdev, unsigned long *cur_state)
{ return -1; }
static inline int tegra_cpu_edp_set_cur_state(
	struct thermal_cooling_device *cdev, unsigned long cur_state)
{ return -1; }
#endif
#ifdef CONFIG_TEGRA_CPU_VOLT_CAP
struct tegra_cooling_device *tegra_vc_get_cdev(void);
#else
static inline struct tegra_cooling_device *tegra_vc_get_cdev(void)
{ return NULL; }
#endif

#endif /* __MACH_TEGRA_CPU_TEGRA_H */
