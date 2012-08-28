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

enum thermal_device_id {
	THERMAL_DEVICE_ID_NULL = 0,
	THERMAL_DEVICE_ID_NCT_EXT = 1,
	THERMAL_DEVICE_ID_NCT_INT = 2,
	THERMAL_DEVICE_ID_TSENSOR = 3,
	THERMAL_DEVICE_ID_THERM_EST_SKIN = 4,
};

#define THERMAL_DEVICE_MAX	(5)

enum cooling_device_id {
	CDEV_BTHROT_ID_TJ      = 0x00010000,
	CDEV_EDPTABLE_ID_EDP   = 0x00030000,
	CDEV_EDPTABLE_ID_EDP_0 = 0x00030000,
	CDEV_EDPTABLE_ID_EDP_1 = 0x00030001,
	CDEV_EDPTABLE_ID_EDP_2 = 0x00030002,
	CDEV_EDPTABLE_ID_EDP_3 = 0x00030003,
	CDEV_EDPTABLE_ID_EDP_4 = 0x00030004,
};

struct tegra_thermal_bind {
	enum thermal_device_id tdev_id;
	enum cooling_device_id cdev_id;
	int type;
	int (*get_trip_temp) (void *, long);
	int (*get_trip_size) (void);
	struct passive_params {
		long trip_temp;
		int tc1;
		int tc2;
		long passive_delay;
	} passive;
};

struct tegra_thermal_device {
	char *name;
	enum thermal_device_id id;
	void *data;
	int (*get_temp) (void *, long *);
	int (*set_limits) (void *, long, long);
	int (*set_alert)(void *, void (*)(void *), void *);
	struct thermal_zone_device *thz;
	struct list_head node;
};

struct tegra_cooling_device {
	enum cooling_device_id id;
};

struct throttle_table {
	unsigned int cpu_freq;
	int core_cap_level;
};

#define MAX_THROT_TABLE_SIZE	(32)

struct balanced_throttle {
	struct tegra_cooling_device tegra_cdev;
	struct thermal_cooling_device *cdev;
	struct list_head node;
	int is_throttling;
	int throttle_index;
	int throt_tab_size;
	struct throttle_table throt_tab[MAX_THROT_TABLE_SIZE];
};

#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
struct thermal_cooling_device *balanced_throttle_register(
	struct balanced_throttle *bthrot);
#else
static inline struct thermal_cooling_device *balanced_throttle_register(
	struct balanced_throttle *bthrot)
{ return ERR_PTR(-EINVAL); }
#endif

#ifdef CONFIG_TEGRA_THERMAL
int tegra_thermal_init(struct tegra_thermal_bind *thermal_binds);
int tegra_thermal_device_register(struct tegra_thermal_device *device);
int tegra_thermal_exit(void);
#else
static inline int tegra_thermal_init(struct tegra_thermal_bind *thermal_binds)
{ return 0; }
static inline int tegra_thermal_device_register(struct tegra_thermal_device *device)
{ return 0; }
static inline int tegra_thermal_exit(void)
{ return 0; }
#endif

#endif	/* __MACH_THERMAL_H */
