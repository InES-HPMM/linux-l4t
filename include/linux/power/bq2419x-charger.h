/*
 * bq2419x-charger.h -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger driver
 *
 * Copyright (C) 2012 - 2013 NVIDIA Corporation

 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 * Author: Syed Rafiuddin <srafiuddin@nvidia.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 *
 */

#ifndef __LINUX_POWER_BQ2419X_CHARGER_H
#define __LINUX_POWER_BQ2419X_CHARGER_H

#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>

/* Register definitions */
#define BQ2419X_INPUT_SRC_REG		0x00
#define BQ2419X_PWR_ON_REG		0x01
#define BQ2419X_CHRG_CTRL_REG		0x02
#define BQ2419X_CHRG_TERM_REG		0x03
#define BQ2419X_VOLT_CTRL_REG		0x04
#define BQ2419X_TIME_CTRL_REG		0x05
#define BQ2419X_THERM_REG		0x06
#define BQ2419X_MISC_OPER_REG		0x07
#define BQ2419X_SYS_STAT_REG		0x08
#define BQ2419X_FAULT_REG		0x09
#define BQ2419X_REVISION_REG		0x0a

#define BQ24190_IC_VER			0x40
#define BQ24192_IC_VER			0x28
#define BQ24192i_IC_VER			0x18

#define BQ2419X_ENABLE_CHARGE_MASK	0x30
#define BQ2419X_ENABLE_CHARGE		0x10
#define BQ2419X_ENABLE_VBUS		0x20

#define BQ2419X_REG0			0x0
#define BQ2419X_EN_HIZ			BIT(7)

#define BQ2419X_CHRG_CTRL_REG_3A	0xC0
#define BQ2419x_OTP_CURRENT_500MA	0x32

#define BQ2419X_WD			0x5
#define BQ2419X_WD_MASK			0x30
#define BQ2419X_WD_DISABLE		0x00
#define BQ2419X_WD_40ms			0x10
#define BQ2419X_WD_80ms			0x20
#define BQ2419X_WD_160ms		0x30

#define BQ2419x_VBUS_STAT		0xc0
#define BQ2419x_VBUS_UNKNOWN		0x00
#define BQ2419x_VBUS_USB		0x40
#define BQ2419x_VBUS_AC			0x80

#define BQ2419x_CHRG_STATE_MASK			0x30
#define BQ2419x_CHRG_STATE_NOTCHARGING		0x00
#define BQ2419x_CHRG_STATE_PRE_CHARGE		0x10
#define BQ2419x_CHRG_STATE_POST_CHARGE		0x20
#define BQ2419x_CHRG_STATE_CHARGE_DONE		0x30

#define BQ2419x_FAULT_WATCHDOG_FAULT		BIT(7)
#define BQ2419x_FAULT_BOOST_FAULT		BIT(6)
#define BQ2419x_FAULT_CHRG_FAULT_MASK		0x30
#define BQ2419x_FAULT_CHRG_NORMAL		0x00
#define BQ2419x_FAULT_CHRG_INPUT		0x10
#define BQ2419x_FAULT_CHRG_THERMAL		0x20
#define BQ2419x_FAULT_CHRG_SAFTY		0x30

#define BQ2419x_FAULT_NTC_FAULT			0x07

#define BQ2419x_CONFIG_MASK		0x7
#define BQ2419x_INPUT_VOLTAGE_MASK	0x78
#define BQ2419x_NVCHARGER_INPUT_VOL_SEL	0x40
#define BQ2419x_DEFAULT_INPUT_VOL_SEL	0x30

#define BQ2419X_MAX_REGS		(BQ2419X_REVISION_REG + 1)

/*
 * struct bq2419x_vbus_platform_data - bq2419x VBUS platform data.
 *
 * @gpio_otg_iusb: GPIO number for OTG/IUSB
 * @num_consumer_supplies: Number fo consumer for vbus regulators.
 * @consumer_supplies: List of consumer suppliers.
 */
struct bq2419x_vbus_platform_data {
	int gpio_otg_iusb;
	int num_consumer_supplies;
	struct regulator_consumer_supply *consumer_supplies;
};

/*
 * struct bq2419x_charger_platform_data - bq2419x charger platform data.
 */
struct bq2419x_charger_platform_data {
	unsigned use_mains:1;
	unsigned use_usb:1;
	void (*update_status)(int, int);
	int (*battery_check)(void);

	int max_charge_volt_mV;
	int max_charge_current_mA;
	int charging_term_current_mA;
	int wdt_timeout;
	int rtc_alarm_time;
	int num_consumer_supplies;
	struct regulator_consumer_supply *consumer_supplies;
	int chg_restart_time;
	int is_battery_present;
};

/*
 * struct bq2419x_platform_data - bq2419x platform data.
 */
struct bq2419x_platform_data {
	struct bq2419x_vbus_platform_data *vbus_pdata;
	struct bq2419x_charger_platform_data *bcharger_pdata;
};

struct bq2419x_chip {
	struct device                   *dev;
	struct regmap                   *regmap;
	int                             irq;
	int                             gpio_otg_iusb;
	int                             wdt_refresh_timeout;
	int                             wdt_time_sec;

	struct power_supply             ac;
	struct power_supply             usb;
	struct mutex                    mutex;
	int                             ac_online;
	int                             usb_online;
	int                             in_current_limit;
	unsigned                        use_mains:1;
	unsigned                        use_usb:1;
	int                             rtc_alarm_time;
	void                            (*update_status)(int, int);

	struct regulator_dev            *chg_rdev;
	struct regulator_desc           chg_reg_desc;
	struct regulator_init_data      chg_reg_init_data;

	struct regulator_dev            *vbus_rdev;
	struct regulator_desc           vbus_reg_desc;
	struct regulator_init_data      vbus_reg_init_data;

	struct kthread_worker           bq_kworker;
	struct task_struct              *bq_kworker_task;
	struct kthread_work             bq_wdt_work;
	struct rtc_device               *rtc;
	int                             stop_thread;
	int                             suspended;
	int                             chg_restart_timeout;
	int                             chg_restart_time;
	int                             use_regmap;
	struct palmas                   *palmas;
};

enum bq2419x_chip_id {
	PALMAS_CHARGER,
};

int bq2419x_hw_init(struct bq2419x_chip *bq2419x,
		struct bq2419x_platform_data *pdata);

int bq2419x_resource_cleanup(struct bq2419x_chip *bq2419x);
#endif /* __LINUX_POWER_BQ2419X_CHARGER_H */
