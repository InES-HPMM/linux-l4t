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

static struct tegra_thermal_bind *thermal_binds;

static struct tegra_thermal_device *devices[THERMAL_DEVICE_MAX];
static bool tegra_thermal_suspend;

#define MAX_TEMP (120000)

static bool tegra_thermal_suspend;

static int tegra_thermal_zone_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice)
{
	int i, j, trip=0, trip_size, index;
	struct tegra_cooling_device *tegra_cdev = cdevice->devdata;
	struct tegra_thermal_device *device = thz->devdata;

	for (i=0; thermal_binds[i].tdev_id; i++) {
		if(device->id == thermal_binds[i].tdev_id) {
			trip_size = thermal_binds[i].get_trip_size();
			index = tegra_cdev->id & 0xffff;

			if (thermal_binds[i].cdev_id  == (tegra_cdev->id & 0xffff0000)) {
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

	for (i=0; thermal_binds[i].tdev_id; i++) {
		if(device->id == thermal_binds[i].tdev_id) {
			if (tegra_cdev->id == thermal_binds[i].cdev_id)
				thermal_zone_unbind_cooling_device(thz, trip, cdevice);
			trip += thermal_binds[i].get_trip_size();
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

	for (i=0; thermal_binds[i].tdev_id; i++) {
		if(device->id == thermal_binds[i].tdev_id) {
			trip_count += thermal_binds[i].get_trip_size();
			if (trip < trip_count) {
				*type = thermal_binds[i].type;
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

	for (i=0; thermal_binds[i].tdev_id; i++) {
		if(device->id == thermal_binds[i].tdev_id) {
			trip_size = thermal_binds[i].get_trip_size();
			for (j=0; j < trip_size; j++) {
				if (trip == trip_count) {
					*temp = thermal_binds[i].get_trip_temp(
							&thermal_binds[i], j);
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
	.get_trip_type = tegra_thermal_zone_get_trip_type,
	.get_trip_temp = tegra_thermal_zone_get_trip_temp,
	.get_temp = tegra_thermal_zone_get_temp,
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
	long temp, trip_temp, low_temp = 0, high_temp = MAX_TEMP;
	int count;

	if (!device->thz)
		return;

	if ((!device->thz->passive) && (!tegra_thermal_suspend))
		thermal_zone_device_update(device->thz);

	device->thz->ops->get_temp(device->thz, &temp);

	for (count = 0; count < device->thz->trips; count++) {
		device->thz->ops->get_trip_temp(device->thz, count, &trip_temp);

		if ((trip_temp >= temp) && (trip_temp < high_temp))
			high_temp = trip_temp;

		if ((trip_temp < temp) && (trip_temp > low_temp))
			low_temp = trip_temp;
	}

	if (device->set_limits && device->thz->trips)
		device->set_limits(device->data, low_temp, high_temp);
}

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
	struct thermal_zone_device *thz = NULL;
	int i, t1 = 0, t2 = 0, pdelay = 0, trips=0;

	if (device->id >= THERMAL_DEVICE_MAX || devices[device->id])
		return -EINVAL;

	for (i=0; thermal_binds[i].tdev_id; i++) {
		if(device->id == thermal_binds[i].tdev_id) {
			switch(thermal_binds[i].type) {
			case THERMAL_TRIP_PASSIVE:
				/* only one passive type allowed for now */
				if (pdelay)
					return -EINVAL;

				/* These should be set only for ACTIVE types */
				if (thermal_binds[i].get_trip_temp ||
					thermal_binds[i].get_trip_size)
					return -EINVAL;

				t1 = thermal_binds[i].passive.tc1;
				t2 = thermal_binds[i].passive.tc2;
				pdelay = thermal_binds[i].passive.passive_delay;

				thermal_binds[i].get_trip_temp = passive_get_trip_temp;
				thermal_binds[i].get_trip_size = passive_get_trip_size;
				break;
			}

			trips += thermal_binds[i].get_trip_size();
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
		if (IS_ERR_OR_NULL(thz)) {
			return -ENODEV;
		}

		device->thz = thz;
	}

	devices[device->id] = device;

	if (thz) {
		device->set_alert(device->data, tegra_thermal_alert, device);
		tegra_thermal_alert(device);
	}

	return 0;
}


int __init tegra_thermal_init(struct tegra_thermal_bind *binds)
{
	thermal_binds = binds;

	register_pm_notifier(&tegra_thermal_nb);

	return 0;
}

int tegra_thermal_exit(void)
{
	return 0;
}

