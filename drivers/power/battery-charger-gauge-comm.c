/*
 * battery-charger-gauge-comm.c -- Communication between battery charger and
 *	battery gauge driver.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/thermal.h>
#include <linux/list.h>
#include <linux/power/battery-charger-gauge-comm.h>

#define JETI_TEMP_COLD		0
#define JETI_TEMP_COOL		10
#define JETI_TEMP_WARM		45
#define JETI_TEMP_HOT		60

static DEFINE_MUTEX(charger_gauge_list_mutex);
static LIST_HEAD(charger_list);
static LIST_HEAD(gauge_list);
static LIST_HEAD(charger_thermal_map_list);

struct battery_charger_dev {
	int				cell_id;
	struct device			*parent_dev;
	struct battery_charging_ops	*ops;
	struct list_head		list;
	void				*drv_data;
	struct delayed_work		restart_charging_wq;
};

struct battery_gauge_dev {
	int				cell_id;
	char				tz_name[THERMAL_NAME_LENGTH];
	struct device			*parent_dev;
	struct battery_gauge_ops	*ops;
	struct list_head		list;
	void				*drv_data;
	struct thermal_zone_device	*battery_tz;
};

struct battery_charger_thermal_dev {
	int					cell_id;
	char					tz_name[THERMAL_NAME_LENGTH];
	struct device				*parent_dev;
	struct battery_charger_thermal_ops	*ops;
	struct list_head			list;
	void					*drv_data;
	struct delayed_work			poll_temp_monitor_wq;
	int					polling_time_sec;
	struct thermal_zone_device		*battery_tz;
	bool					start_monitoring;
};

static void battery_charger_restart_charging_wq(struct work_struct *work)
{
	struct battery_charger_dev *bc_dev;

	bc_dev = container_of(work, struct battery_charger_dev,
					restart_charging_wq.work);
	if (!bc_dev->ops->restart_charging) {
		dev_err(bc_dev->parent_dev,
				"No callback for restart charging\n");
		return;
	}
	bc_dev->ops->restart_charging(bc_dev);
}

static void battery_thermal_check_temperature(struct work_struct *work)
{
	struct battery_charger_thermal_dev *bct_dev;
	struct device *dev;
	long temperature;
	bool charger_enable_state;
	bool charger_current_half;
	int battery_thersold_voltage;
	int ret;

	bct_dev = container_of(work, struct battery_charger_thermal_dev,
					poll_temp_monitor_wq.work);
	dev = bct_dev->parent_dev;

	if (!bct_dev->battery_tz)
		bct_dev->battery_tz =
			thermal_zone_device_find_by_name(bct_dev->tz_name);

	if (!bct_dev->battery_tz) {
		dev_info(dev, "Battery thermal zone %s is not registered yet\n",
					bct_dev->tz_name);
		schedule_delayed_work(&bct_dev->poll_temp_monitor_wq,
			msecs_to_jiffies(bct_dev->polling_time_sec * HZ));
		return;
	}

	ret = thermal_zone_get_temp(bct_dev->battery_tz, &temperature);
	if (ret < 0) {
		dev_err(dev, "Temperature read failed: %d\n ", ret);
		goto exit;
	}

	temperature = temperature / 1000;
	charger_enable_state = true;
	charger_current_half = false;
	battery_thersold_voltage = 4250;

	if (temperature <= JETI_TEMP_COLD || temperature >= JETI_TEMP_HOT) {
		charger_enable_state = false;
	} else if (temperature <= JETI_TEMP_COOL &&
				temperature >= JETI_TEMP_WARM) {
		charger_current_half = true;
		battery_thersold_voltage = 4100;
	}

	if (bct_dev->ops->thermal_configure)
		bct_dev->ops->thermal_configure(bct_dev, temperature,
			charger_enable_state, charger_current_half,
			battery_thersold_voltage);

exit:
	if (bct_dev->start_monitoring)
		schedule_delayed_work(&bct_dev->poll_temp_monitor_wq,
			msecs_to_jiffies(bct_dev->polling_time_sec * HZ));
	return;
}

struct battery_charger_thermal_dev *battery_charger_thermal_register(
	struct device *dev, struct battery_charger_thermal_info *bcti,
	void *drv_data)
{
	struct battery_charger_thermal_dev *bct_dev;

	dev_info(dev, "Registering battery charger thermal manager\n");

	if (!dev || !bcti) {
		dev_err(dev, "Invalid parameters\n");
		return ERR_PTR(-EINVAL);
	}

	bct_dev = kzalloc(sizeof(*bct_dev), GFP_KERNEL);
	if (!bct_dev) {
		dev_err(dev, "Memory alloc for bc_dev failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&charger_gauge_list_mutex);

	INIT_LIST_HEAD(&bct_dev->list);
	bct_dev->cell_id = bcti->cell_id;
	bct_dev->ops = bcti->bct_ops;
	bct_dev->polling_time_sec = bcti->polling_time_sec;
	bct_dev->parent_dev = dev;
	bct_dev->drv_data = drv_data;
	strcpy(bct_dev->tz_name, bcti->tz_name ? : "");
	bct_dev->battery_tz =
			thermal_zone_device_find_by_name(bct_dev->tz_name);
	if (!bct_dev->battery_tz)
		dev_info(dev, "Battery thermal zone %s is not registered yet\n",
			bct_dev->tz_name);
	INIT_DELAYED_WORK(&bct_dev->poll_temp_monitor_wq,
			battery_thermal_check_temperature);
	list_add(&bct_dev->list, &charger_thermal_map_list);
	mutex_unlock(&charger_gauge_list_mutex);
	return bct_dev;
}
EXPORT_SYMBOL(battery_charger_thermal_register);

void battery_charger_thermal_unregister(
	struct battery_charger_thermal_dev *bct_dev)
{
	if (!bct_dev)
		return;

	mutex_lock(&charger_gauge_list_mutex);
	cancel_delayed_work(&bct_dev->poll_temp_monitor_wq);
	list_del(&bct_dev->list);
	mutex_unlock(&charger_gauge_list_mutex);
	kfree(bct_dev);
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_unregister);

int battery_charger_thermal_start_monitoring(
	struct battery_charger_thermal_dev *bct_dev)
{
	if (!bct_dev)
		return -EINVAL;

	bct_dev->start_monitoring = true;
	schedule_delayed_work(&bct_dev->poll_temp_monitor_wq,
			msecs_to_jiffies(bct_dev->polling_time_sec * HZ));
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_start_monitoring);

int battery_charger_thermal_stop_monitoring(
	struct battery_charger_thermal_dev *bct_dev)
{
	if (!bct_dev)
		return -EINVAL;

	bct_dev->start_monitoring = false;
	cancel_delayed_work(&bct_dev->poll_temp_monitor_wq);
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_stop_monitoring);

struct battery_charger_dev *battery_charger_register(struct device *dev,
	struct battery_charger_info *bci)
{
	struct battery_charger_dev *bc_dev;

	dev_info(dev, "Registering battery charger driver\n");

	if (!dev || !bci) {
		dev_err(dev, "Invalid parameters\n");
		return ERR_PTR(-EINVAL);
	}

	bc_dev = kzalloc(sizeof(*bc_dev), GFP_KERNEL);
	if (!bc_dev) {
		dev_err(dev, "Memory alloc for bc_dev failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&charger_gauge_list_mutex);

	INIT_LIST_HEAD(&bc_dev->list);
	bc_dev->cell_id = bci->cell_id;
	bc_dev->ops = bci->bc_ops;
	bc_dev->parent_dev = dev;
	list_add(&bc_dev->list, &charger_list);

	INIT_DELAYED_WORK(&bc_dev->restart_charging_wq,
			battery_charger_restart_charging_wq);
	mutex_unlock(&charger_gauge_list_mutex);
	return bc_dev;
}
EXPORT_SYMBOL_GPL(battery_charger_register);

void battery_charger_unregister(struct battery_charger_dev *bc_dev)
{
	mutex_lock(&charger_gauge_list_mutex);
	list_del(&bc_dev->list);
	mutex_unlock(&charger_gauge_list_mutex);
	kfree(bc_dev);
}
EXPORT_SYMBOL_GPL(battery_charger_unregister);

int battery_charging_restart(struct battery_charger_dev *bc_dev, int after_sec)
{
	if (!bc_dev->ops->restart_charging) {
		dev_err(bc_dev->parent_dev,
			"No callback for restart charging\n");
		return -EINVAL;
	}
	schedule_delayed_work(&bc_dev->restart_charging_wq,
			msecs_to_jiffies(after_sec * HZ));
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charging_restart);

int battery_gauge_get_battery_temperature(struct battery_gauge_dev *bg_dev,
	int *temp)
{
	int ret;
	long temperature;

	if (!bg_dev)
		return -EINVAL;

	if (!bg_dev->battery_tz)
		bg_dev->battery_tz =
			thermal_zone_device_find_by_name(bg_dev->tz_name);

	if (!bg_dev->battery_tz) {
		dev_info(bg_dev->parent_dev,
			"Battery thermal zone %s is not registered yet\n",
			bg_dev->tz_name);
		return -ENODEV;
	}

	ret = thermal_zone_get_temp(bg_dev->battery_tz, &temperature);
	if (ret < 0)
		return ret;

	*temp = temperature / 1000;
	return 0;
}
EXPORT_SYMBOL_GPL(battery_gauge_get_battery_temperature);

struct battery_gauge_dev *battery_gauge_register(struct device *dev,
	struct battery_gauge_info *bgi)
{
	struct battery_gauge_dev *bg_dev;

	dev_info(dev, "Registering battery gauge driver\n");

	if (!dev || !bgi) {
		dev_err(dev, "Invalid parameters\n");
		return ERR_PTR(-EINVAL);
	}

	bg_dev = kzalloc(sizeof(*bg_dev), GFP_KERNEL);
	if (!bg_dev) {
		dev_err(dev, "Memory alloc for bg_dev failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&charger_gauge_list_mutex);

	INIT_LIST_HEAD(&bg_dev->list);
	bg_dev->cell_id = bgi->cell_id;
	bg_dev->ops = bgi->bg_ops;
	bg_dev->parent_dev = dev;
	strcpy(bg_dev->tz_name, bgi->tz_name ? : "");
	bg_dev->battery_tz = thermal_zone_device_find_by_name(bg_dev->tz_name);
	if (!bg_dev->battery_tz)
		dev_info(dev, "Battery thermal zone %s is not registered yet\n",
			bg_dev->tz_name);
	list_add(&bg_dev->list, &gauge_list);
	mutex_unlock(&charger_gauge_list_mutex);
	return bg_dev;
}
EXPORT_SYMBOL_GPL(battery_gauge_register);

void battery_gauge_unregister(struct battery_gauge_dev *bg_dev)
{
	mutex_lock(&charger_gauge_list_mutex);
	list_del(&bg_dev->list);
	mutex_unlock(&charger_gauge_list_mutex);
	kfree(bg_dev);
}
EXPORT_SYMBOL_GPL(battery_gauge_unregister);

int battery_charging_status_update(struct battery_charger_dev *bc_dev,
	enum battery_charger_status status)
{
	struct battery_gauge_dev *node;
	int ret = -EINVAL;

	if (!bc_dev) {
		dev_err(bc_dev->parent_dev, "Invalid parameters\n");
		return -EINVAL;
	}

	mutex_lock(&charger_gauge_list_mutex);

	list_for_each_entry(node, &gauge_list, list) {
		if (node->cell_id != bc_dev->cell_id)
			continue;
		if (node->ops && node->ops->update_battery_status)
			ret = node->ops->update_battery_status(node, status);
	}

	mutex_unlock(&charger_gauge_list_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(battery_charging_status_update);

void *battery_charger_get_drvdata(struct battery_charger_dev *bc_dev)
{
	if (bc_dev)
		return bc_dev->drv_data;
	return NULL;
}
EXPORT_SYMBOL_GPL(battery_charger_get_drvdata);

void battery_charger_set_drvdata(struct battery_charger_dev *bc_dev, void *data)
{
	if (bc_dev)
		bc_dev->drv_data = data;
}
EXPORT_SYMBOL_GPL(battery_charger_set_drvdata);

void *battery_charger_thermal_get_drvdata(
		struct battery_charger_thermal_dev *bct_dev)
{
	if (bct_dev)
		return bct_dev->drv_data;
	return NULL;
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_get_drvdata);

void battery_charger_thermal_set_drvdata(
	struct battery_charger_thermal_dev *bct_dev, void *data)
{
	if (bct_dev)
		bct_dev->drv_data = data;
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_set_drvdata);

void *battery_gauge_get_drvdata(struct battery_gauge_dev *bg_dev)
{
	if (bg_dev)
		return bg_dev->drv_data;
	return NULL;
}
EXPORT_SYMBOL_GPL(battery_gauge_get_drvdata);

void battery_gauge_set_drvdata(struct battery_gauge_dev *bg_dev, void *data)
{
	if (bg_dev)
		bg_dev->drv_data = data;
}
EXPORT_SYMBOL_GPL(battery_gauge_set_drvdata);
