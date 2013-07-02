/*
 * battery-charger-gauge-comm.h -- Communication APIS between battery charger
 *		and battery gauge driver.
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

#ifndef TEGRA_CHARGER_CORE_H
#define TEGRA_CHARGER_CORE_H

enum battery_charger_status {
	BATTERY_DISCHARGING,
	BATTERY_CHARGING,
	BATTERY_CHARGING_DONE,
};

struct battery_gauge_dev;
struct battery_charger_dev;
struct battery_charger_thermal_dev;

struct battery_gauge_ops {
	int (*update_battery_status)(struct battery_gauge_dev *bg_device,
				enum battery_charger_status status);
};

struct battery_charging_ops {
	int (*get_charging_status)(struct battery_charger_dev *bc_dev);
	int (*restart_charging)(struct battery_charger_dev *bc_dev);
};

struct battery_charger_thermal_ops {
	int (*thermal_configure)(struct battery_charger_thermal_dev *bct_dev,
		int temp, bool enable_charger, bool enable_charg_half_current,
		int battery_voltage);
};

struct battery_charger_info {
	int cell_id;
	struct battery_charging_ops *bc_ops;
};

struct battery_charger_thermal_info {
	int cell_id;
	const char *tz_name;
	int polling_time_sec;
	struct battery_charger_thermal_ops *bct_ops;
};

struct battery_gauge_info {
	int cell_id;
	const char *tz_name;
	struct battery_gauge_ops *bg_ops;
};

struct battery_charger_dev *battery_charger_register(struct device *dev,
		struct battery_charger_info *bci, void *drv_data);
void battery_charger_unregister(struct battery_charger_dev *bc_dev);
int battery_charging_status_update(struct battery_charger_dev *bc_dev,
		enum battery_charger_status status);
int battery_charging_restart(struct battery_charger_dev *bc_dev, int after_sec);

struct battery_charger_thermal_dev *battery_charger_thermal_register(
	struct device *dev, struct battery_charger_thermal_info *bci,
	void *drv_data);
void battery_charger_thermal_unregister(
	struct battery_charger_thermal_dev *bct_dev);
int battery_charger_thermal_start_monitoring(
	struct battery_charger_thermal_dev *bct_dev);
int battery_charger_thermal_stop_monitoring(
	struct battery_charger_thermal_dev *bct_dev);

int battery_gauge_get_battery_temperature(struct battery_gauge_dev *bg_dev,
	int *temp);
struct battery_gauge_dev *battery_gauge_register(struct device *dev,
		struct battery_gauge_info *bgi, void *drv_data);
void battery_gauge_unregister(struct battery_gauge_dev *bg_dev);

void *battery_charger_get_drvdata(struct battery_charger_dev *bc_dev);
void battery_charger_set_drvdata(struct battery_charger_dev *bc_dev,
			void *data);
void *battery_charger_thermal_get_drvdata(
		struct battery_charger_thermal_dev *bct_dev);
void battery_charger_thermal_set_drvdata(
		struct battery_charger_thermal_dev *bct_dev, void *data);
void *battery_gauge_get_drvdata(struct battery_gauge_dev *bg_dev);
void battery_gauge_set_drvdata(struct battery_gauge_dev *bg_dev, void *data);
#endif
