/*
 * arch/arm/mach-tegra/tegra3_thermal.c
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

#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <linux/module.h>
#include <mach/thermal.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include "clock.h"
#include "cpu-tegra.h"
#include "dvfs.h"

static struct tegra_thermal_data *therm;
static struct tegra_skin_data *skin_therm;
static LIST_HEAD(tegra_therm_list);
static DEFINE_MUTEX(tegra_therm_mutex);

static struct balanced_throttle *throttle_list;
static int throttle_list_size;

#define MAX_TEMP (120000)

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int skin_devs_bitmap;
static struct therm_est_subdevice *skin_devs[THERMAL_DEVICE_MAX];
static int skin_devs_count;
#endif
static bool tegra_thermal_suspend;

#ifdef CONFIG_DEBUG_FS
static struct dentry *thermal_debugfs_root;
#endif

static inline long dev2tj(struct tegra_thermal_device *dev,
				long dev_temp)
{
	return dev_temp + dev->offset;
}

static inline long tj2dev(struct tegra_thermal_device *dev,
				long tj_temp)
{
	return tj_temp - dev->offset;
}

static int tegra_thermal_get_temp_unlocked(long *tj_temp, bool offsetted)
{
	struct tegra_thermal_device *dev = NULL;
	int ret = 0;

	list_for_each_entry(dev, &tegra_therm_list, node)
		if (dev->id == therm->throttle_edp_device_id)
			break;

	if (dev) {
		dev->get_temp(dev->data, tj_temp);
		if (offsetted)
			*tj_temp = dev2tj(dev, *tj_temp);
	} else {
		*tj_temp = 0xdeadbeef;
		ret = -1;
	}

	return ret;
}

static int tegra_thermal_zone_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice)
{
	int i, j, trip=0, trip_size, index;
	struct tegra_cooling_device *tegra_cdev = cdevice->devdata;
	struct tegra_thermal_device *device = thz->devdata;

	for (i=0; therm->binds[i].tdev_id; i++) {
		if(device->id == therm->binds[i].tdev_id) {
			trip_size = therm->binds[i].get_trip_size();
			index = tegra_cdev->id & 0xffff;

			if (therm->binds[i].cdev_id  == (tegra_cdev->id & 0xffff0000)) {
				for (j=0; j < trip_size; j++) {
					thermal_zone_bind_cooling_device(
							thz,
							trip + index,
							cdevice);
				}
			}

			trip += trip_size;
		}
	}

	return 0;
}

static int tegra_thermal_zone_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice) {
	int i, trip = 0;
	struct tegra_cooling_device *tegra_cdev = cdevice->devdata;
	struct tegra_thermal_device *device = thz->devdata;

	for (i=0; therm->binds[i].tdev_id; i++) {
		if(device->id == therm->binds[i].tdev_id) {
			if (tegra_cdev->id == therm->binds[i].cdev_id)
				thermal_zone_unbind_cooling_device(thz, trip, cdevice);
			trip += therm->binds[i].get_trip_size();
		}
	}

	return 0;
}

static int tegra_thermal_zone_get_temp(struct thermal_zone_device *thz,
					unsigned long *temp)
{
	struct tegra_thermal_device *device = thz->devdata;

	if (!tegra_thermal_suspend)
		device->get_temp(device->data, temp);

	return 0;
}

static int tegra_thermal_zone_get_trip_type(
			struct thermal_zone_device *thz,
			int trip,
			enum thermal_trip_type *type) {
	int i, trip_count = 0;
	struct tegra_thermal_device *device = thz->devdata;

	for (i=0; therm->binds[i].tdev_id; i++) {
		if(device->id == therm->binds[i].tdev_id) {
			trip_count += therm->binds[i].get_trip_size();
			if (trip < trip_count) {
				*type = therm->binds[i].type;
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int tegra_thermal_zone_get_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long *temp) {
	int i, j, trip_size, trip_count = 0;
	struct tegra_thermal_device *device = thz->devdata;

	for (i=0; therm->binds[i].tdev_id; i++) {
		if(device->id == therm->binds[i].tdev_id) {
			trip_size = therm->binds[i].get_trip_size();
			for (j=0; j < trip_size; j++) {
				if (trip == trip_count) {
					*temp = therm->binds[i].get_trip_temp(
							&therm->binds[i], j);
					return 0;
				}
				trip_count++;
			}
		}
	}

	return -EINVAL;
}

static struct thermal_zone_device_ops tegra_thermal_zone_ops = {
	.bind = tegra_thermal_zone_bind,
	.unbind = tegra_thermal_zone_unbind,
	.get_temp = tegra_thermal_zone_get_temp,
	.get_trip_type = tegra_thermal_zone_get_trip_type,
	.get_trip_temp = tegra_thermal_zone_get_trip_temp,
};

static int tegra_thermal_pm_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		tegra_thermal_suspend = true;
		break;
	case PM_POST_SUSPEND:
		tegra_thermal_suspend = false;
		break;
	}

	return NOTIFY_OK;
};

static struct notifier_block tegra_thermal_nb = {
	.notifier_call = tegra_thermal_pm_notify,
};

static void tegra_thermal_alert(void *data)
{
	struct tegra_thermal_device *device = data;
	long temp_tj;
	long lo_limit_throttle_tj, hi_limit_throttle_tj;
	long lo_limit_edp_tj = 0, hi_limit_edp_tj = 0;
	long temp_low_dev, temp_low_tj;
	int lo_limit_tj = 0, hi_limit_tj = 0;
#ifdef CONFIG_TEGRA_EDP_LIMITS
	const struct tegra_edp_limits *z;
	int zones_sz;
	int i;
#endif

	mutex_lock(&tegra_therm_mutex);

	if (device->thz)
		if ((!device->thz->passive) && (!tegra_thermal_suspend))
			thermal_zone_device_update(device->thz);

	/* Convert all temps to tj and then do all work/logic in terms of
	   tj in order to avoid confusion */
	if (tegra_thermal_get_temp_unlocked(&temp_tj, true))
		goto done;

	device->get_temp_low(device, &temp_low_dev);
	temp_low_tj = dev2tj(device, temp_low_dev);

	lo_limit_throttle_tj = temp_low_tj;
	hi_limit_throttle_tj = dev2tj(device, MAX_TEMP);

	hi_limit_throttle_tj = dev2tj(device, therm->temp_throttle);

	if (temp_tj > dev2tj(device, therm->temp_throttle)) {
		lo_limit_throttle_tj = dev2tj(device, therm->temp_throttle);
		hi_limit_throttle_tj = dev2tj(device, MAX_TEMP);
	}

#ifdef CONFIG_TEGRA_EDP_LIMITS
#define EDP_TEMP_TJ(_index) (z[_index].temperature * 1000 + therm->edp_offset)
	tegra_get_cpu_edp_limits(&z, &zones_sz);
	if (temp_tj < EDP_TEMP_TJ(0)) {
		lo_limit_edp_tj = temp_low_tj;
		hi_limit_edp_tj = EDP_TEMP_TJ(0);
	} else if (temp_tj >= EDP_TEMP_TJ(zones_sz-1)) {
		lo_limit_edp_tj = EDP_TEMP_TJ(zones_sz-1) -
					therm->hysteresis_edp;
		hi_limit_edp_tj = dev2tj(device, MAX_TEMP);
	} else {
		for (i = 0; (i + 1) < zones_sz; i++) {
			if ((temp_tj >= EDP_TEMP_TJ(i)) &&
				(temp_tj < EDP_TEMP_TJ(i+1))) {
				lo_limit_edp_tj = EDP_TEMP_TJ(i) -
							therm->hysteresis_edp;
				hi_limit_edp_tj = EDP_TEMP_TJ(i+1);
				break;
			}
		}
	}
#undef EDP_TEMP_TJ
#else
	lo_limit_edp_tj = temp_low_tj;
	hi_limit_edp_tj = dev2tj(device, MAX_TEMP);
#endif

	/* Get smallest window size */
	lo_limit_tj = max(lo_limit_throttle_tj, lo_limit_edp_tj);
	hi_limit_tj = min(hi_limit_throttle_tj, hi_limit_edp_tj);

	device->set_limits(device->data,
				tj2dev(device, lo_limit_tj),
				tj2dev(device, hi_limit_tj));
}

done:
	mutex_unlock(&tegra_therm_mutex);
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static void tegra_skin_thermal_alert(void *data)
{
	struct tegra_thermal_device *dev = data;

	if (!dev->thz->passive)
		thermal_zone_device_update(dev->thz);
}

static int tegra_skin_device_register(struct tegra_thermal_device *device)
{
	int i;
	struct therm_est_subdevice *skin_dev =
		kzalloc(sizeof(struct therm_est_subdevice), GFP_KERNEL);

	for (i = 0; i < skin_therm->skin_devs_size; i++) {
		if (skin_therm->skin_devs[i].id == device->id) {
			memcpy(skin_dev->coeffs,
				skin_therm->skin_devs[i].coeffs,
				sizeof(skin_devs[i]->coeffs));
			break;
		}
	}

	skin_dev->dev_data = device->data;
	skin_dev->get_temp = device->get_temp;

	skin_devs[skin_devs_count++] = skin_dev;

	/* Create skin thermal device */
	if (skin_devs_count == skin_therm->skin_devs_size) {
		struct tegra_thermal_device *thermal_skin_device;
		struct therm_estimator *skin_estimator;

		skin_estimator = therm_est_register(
					skin_devs,
					skin_devs_count,
					skin_therm->skin_temp_offset,
					skin_therm->skin_period);
		thermal_skin_device = kzalloc(sizeof(struct tegra_thermal_device),
							GFP_KERNEL);
		thermal_skin_device->name = "skin_pred";
		thermal_skin_device->id = THERMAL_DEVICE_ID_SKIN;
		thermal_skin_device->data = skin_estimator;
		thermal_skin_device->get_temp =
			(int (*)(void *, long *)) therm_est_get_temp;
		thermal_skin_device->set_limits =
			(int (*)(void *, long, long)) therm_est_set_limits;
		thermal_skin_device->set_alert =
			(int (*)(void *, void (*)(void *), void *))
				therm_est_set_alert;

		tegra_thermal_device_register(thermal_skin_device);
	}

	return 0;
}
#endif

static int passive_get_trip_temp(void *data, long trip)
{
	struct tegra_thermal_bind *bind = data;
	return bind->passive.trip_temp;
}

static int passive_get_trip_size(void)
{
	return 1;
}

int tegra_thermal_device_register(struct tegra_thermal_device *device)
{
	struct tegra_thermal_device *dev;
	struct thermal_zone_device *thz;
	int i, t1 = 0, t2 = 0, pdelay = 0, trips=0;

	mutex_lock(&tegra_therm_mutex);
	list_for_each_entry(dev, &tegra_therm_list, node) {
		if (dev->id == device->id) {
			mutex_unlock(&tegra_therm_mutex);
			return -EINVAL;
		}
	}

	for (i=0; therm->binds[i].tdev_id; i++) {
		if(device->id == therm->binds[i].tdev_id) {
			switch(therm->binds[i].type) {
			case THERMAL_TRIP_PASSIVE:
				/* only one passive type allowed for now */
				if (pdelay)
					return -EINVAL;

				/* These should be set only for ACTIVE types */
				if (therm->binds[i].get_trip_temp ||
					therm->binds[i].get_trip_size)
					return -EINVAL;

				t1 = therm->binds[i].passive.tc1;
				t2 = therm->binds[i].passive.tc2;
				pdelay = therm->binds[i].passive.passive_delay;

				therm->binds[i].get_trip_temp = passive_get_trip_temp;
				therm->binds[i].get_trip_size = passive_get_trip_size;
				break;
			}

			trips += therm->binds[i].get_trip_size();
		}
	}

	if (trips) {
		thz = thermal_zone_device_register(
					device->name,
					trips, /* trips */
					device,
					&tegra_thermal_zone_ops,
					t1, /* dT/dt */
					t2, /* throttle */
					pdelay,
					0); /* polling delay */
		if (IS_ERR_OR_NULL(thz))
			return -ENODEV;

		device->thz = thz;
	}

	list_add(&device->node, &tegra_therm_list);
	mutex_unlock(&tegra_therm_mutex);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if (device->id == skin_therm->skin_device_id) {
		if (create_thz)
			device->set_alert(device->data,
				tegra_skin_thermal_alert,
				device);
		device->set_limits(device->data, 0, skin_therm->temp_throttle_skin);
	}
#endif

	if (device->id == therm->throttle_edp_device_id) {
		device->set_alert(device->data, tegra_thermal_alert, device);

		/* initialize limits */
		tegra_thermal_alert(device);
	}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	if ((skin_therm->skin_device_id == THERMAL_DEVICE_ID_SKIN) &&
		device->id && skin_devs_bitmap)
		tegra_skin_device_register(device);
#endif

	return 0;
}

/* This needs to be inialized later hand */
static int __init throttle_list_init(void)
{
	int i;
	for (i = 0; i < throttle_list_size; i++)
		if (balanced_throttle_register(&throttle_list[i]))
			return -ENODEV;

	return 0;
}
late_initcall(throttle_list_init);

int __init tegra_thermal_init(struct tegra_thermal_data *data,
				struct tegra_skin_data *skin_data,
				struct balanced_throttle *tlist,
				int tlist_size)
{
	therm = data;
	skin_therm = skin_data;
#ifdef CONFIG_DEBUG_FS
	thermal_debugfs_root = debugfs_create_dir("tegra_thermal", 0);
#endif

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		int i;
		for (i = 0; i < skin_therm->skin_devs_size; i++)
			skin_devs_bitmap |= skin_therm->skin_devs[i].id;
	}
#endif

	throttle_list = tlist;
	throttle_list_size = tlist_size;

	register_pm_notifier(&tegra_thermal_nb);

	return 0;
}

int tegra_thermal_exit(void)
{
	struct tegra_thermal_device *dev;
	mutex_lock(&tegra_therm_mutex);
	list_for_each_entry(dev, &tegra_therm_list, node) {
		thermal_zone_device_unregister(dev->thz);
	}
	mutex_unlock(&tegra_therm_mutex);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int tegra_thermal_temp_tj_get(void *data, u64 *val)
{
	long temp_tj;

	mutex_lock(&tegra_therm_mutex);
	if (tegra_thermal_get_temp_unlocked(&temp_tj, false))
		temp_tj = -1;
	mutex_unlock(&tegra_therm_mutex);

	*val = (u64)temp_tj;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(temp_tj_fops,
			tegra_thermal_temp_tj_get,
			NULL,
			"%llu\n");

static int __init temp_tj_debug_init(void)
{
	debugfs_create_file("temp_tj", 0644, thermal_debugfs_root,
		NULL, &temp_tj_fops);
	return 0;
}
late_initcall(temp_tj_debug_init);

#define THERM_DEBUGFS(_name) \
	static int tegra_thermal_##_name##_set(void *data, u64 val) \
	{ \
		struct tegra_thermal_device *dev; \
		mutex_lock(&tegra_therm_mutex); \
		list_for_each_entry(dev, &tegra_therm_list, node) \
			if (dev->id == therm->throttle_edp_device_id) \
				break; \
		if (dev) \
			dev->thz->_name = val; \
		mutex_unlock(&tegra_therm_mutex); \
		return 0; \
	} \
	static int tegra_thermal_##_name##_get(void *data, u64 *val) \
	{ \
		struct tegra_thermal_device *dev; \
		mutex_lock(&tegra_therm_mutex); \
		list_for_each_entry(dev, &tegra_therm_list, node) \
			if (dev->id == therm->throttle_edp_device_id) \
				break; \
		if (dev) \
			*val = (u64)dev->thz->_name; \
		mutex_unlock(&tegra_therm_mutex); \
		return 0; \
	} \
	DEFINE_SIMPLE_ATTRIBUTE(_name##_fops, \
			tegra_thermal_##_name##_get, \
			tegra_thermal_##_name##_set, \
			"%llu\n"); \
	static int __init _name##_debug_init(void) \
	{ \
		debugfs_create_file(#_name, 0644, thermal_debugfs_root, \
			NULL, &_name##_fops); \
		return 0; \
	} \
	late_initcall(_name##_debug_init);


THERM_DEBUGFS(tc1);
THERM_DEBUGFS(tc2);
THERM_DEBUGFS(passive_delay);
#endif
