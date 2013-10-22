/*
 * bq2419x-charger.c -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 * Author: Syed Rafiuddin <srafiuddin@nvidia.com>
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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/regmap.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/power/battery-charger-gauge-comm.h>

#define MAX_STR_PRINT 50

/* input current limit */
static const unsigned int iinlim[] = {
	100, 150, 500, 900, 1200, 1500, 2000, 3000,
};

/* Kthread scheduling parameters */
struct sched_param bq2419x_param = {
	.sched_priority = MAX_RT_PRIO - 1,
};

static const struct regmap_config bq2419x_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= BQ2419X_MAX_REGS,
};

struct bq2419x_chip {
	struct device			*dev;
	struct regmap			*regmap;
	int				irq;
	int				gpio_otg_iusb;
	int				wdt_refresh_timeout;
	int				wdt_time_sec;

	struct mutex			mutex;
	int				in_current_limit;
	int				rtc_alarm_time;

	struct regulator_dev		*chg_rdev;
	struct regulator_desc		chg_reg_desc;
	struct regulator_init_data	chg_reg_init_data;

	struct regulator_dev		*vbus_rdev;
	struct regulator_desc		vbus_reg_desc;
	struct regulator_init_data	vbus_reg_init_data;

	struct battery_charger_dev	*bc_dev;
	int				chg_status;

	struct kthread_worker		bq_kworker;
	struct task_struct		*bq_kworker_task;
	struct kthread_work		bq_wdt_work;
	struct rtc_device		*rtc;
	int				stop_thread;
	int				suspended;
	int				chg_restart_timeout;
	int				chg_restart_time;
	int				battery_presense;
	bool				cable_connected;
	int				last_charging_current;
};

static int current_to_reg(const unsigned int *tbl,
			size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

static int bq2419x_charger_enable(struct bq2419x_chip *bq2419x)
{
	int ret;

	if (bq2419x->battery_presense) {
		dev_info(bq2419x->dev, "Charging enabled\n");
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_PWR_ON_REG,
				 BQ2419X_ENABLE_CHARGE_MASK, 0);
		if (ret < 0) {
			dev_err(bq2419x->dev,
				"register update failed, err %d\n", ret);
			return ret;
		}

		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_PWR_ON_REG,
			 BQ2419X_ENABLE_CHARGE_MASK, BQ2419X_ENABLE_CHARGE);
	} else {
		dev_info(bq2419x->dev, "Charging disabled\n");
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_PWR_ON_REG,
				 BQ2419X_ENABLE_CHARGE_MASK,
				 BQ2419X_DISABLE_CHARGE);
	}
	if (ret < 0)
		dev_err(bq2419x->dev, "register update failed, err %d\n", ret);
	return ret;
}

static int bq2419x_vbus_regulator_enable_time(struct regulator_dev *rdev)
{
	return 8000;
}

static int bq2419x_vbus_enable(struct regulator_dev *rdev)
{
	struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
	int ret;

	dev_info(bq2419x->dev, "VBUS enabled, charging disabled\n");

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_PWR_ON_REG,
			BQ2419X_ENABLE_CHARGE_MASK, BQ2419X_ENABLE_VBUS);
	if (ret < 0)
		dev_err(bq2419x->dev, "PWR_ON_REG update failed %d", ret);

	return ret;
}

static int bq2419x_vbus_disable(struct regulator_dev *rdev)
{
	struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
	int ret;

	dev_info(bq2419x->dev, "VBUS disabled, charging enabled\n");
	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger enable failed %d", ret);
		return ret;
	}

	return ret;
}

static int bq2419x_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
	int ret;
	unsigned int data;

	ret = regmap_read(bq2419x->regmap, BQ2419X_PWR_ON_REG, &data);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG read failed %d", ret);
		return ret;
	}
	return (data & BQ2419X_ENABLE_CHARGE_MASK) == BQ2419X_ENABLE_VBUS;
}

static struct regulator_ops bq2419x_vbus_ops = {
	.enable		= bq2419x_vbus_enable,
	.disable	= bq2419x_vbus_disable,
	.is_enabled	= bq2419x_vbus_is_enabled,
	.enable_time	= bq2419x_vbus_regulator_enable_time,
};

static int bq2419x_init(struct bq2419x_chip *bq2419x)
{
	int val = 0;
	int ret = 0;
	int floor = 0;

	/* Configure input voltage to 4.52 in case of NV charger */
	if (bq2419x->in_current_limit == 2000)
		val |= BQ2419x_NVCHARGER_INPUT_VOL_SEL;
	else
		val |= BQ2419x_DEFAULT_INPUT_VOL_SEL;

	/* Clear EN_HIZ */
	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
			BQ2419X_EN_HIZ | BQ2419x_INPUT_VOLTAGE_MASK, val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "INPUT_SRC_REG update failed %d\n", ret);
		return ret;
	}

	/* Configure input current limit */
	val = current_to_reg(iinlim, ARRAY_SIZE(iinlim),
				bq2419x->in_current_limit);

	/* Start from 500mA and then step to val */
	floor = current_to_reg(iinlim, ARRAY_SIZE(iinlim), 500);
	if (val < 0 || floor < 0)
		return 0;

	for (; floor <= val; floor++) {
		udelay(BQ2419x_CHARGING_CURRENT_STEP_DELAY_US);
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
				BQ2419x_CONFIG_MASK, floor);
		if (ret < 0)
			dev_err(bq2419x->dev,
				"INPUT_SRC_REG update failed: %d\n", ret);
	}
	return ret;
}

static int bq2419x_charger_init(struct bq2419x_chip *bq2419x)
{
	int ret;

	/* Configure Output Current Control to 2.25A*/
	ret = regmap_write(bq2419x->regmap, BQ2419X_CHRG_CTRL_REG, 0x6c);
	if (ret < 0) {
		dev_err(bq2419x->dev, "CHRG_CTRL_REG write failed %d\n", ret);
		return ret;
	}

	/*
	 * Configure Input voltage limit reset to OTP value,
	 * and charging current to 500mA.
	 */
	ret = regmap_write(bq2419x->regmap, BQ2419X_INPUT_SRC_REG, 0x32);
	if (ret < 0)
		dev_err(bq2419x->dev, "INPUT_SRC_REG write failed %d\n", ret);

	return ret;
}

static int bq2419x_set_charging_current(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct bq2419x_chip *bq_charger = rdev_get_drvdata(rdev);
	int ret = 0;
	int val;

	dev_info(bq_charger->dev, "Setting charging current %d\n", max_uA/1000);
	msleep(200);
	bq_charger->chg_status = BATTERY_DISCHARGING;

	ret = bq2419x_charger_enable(bq_charger);
	if (ret < 0) {
		dev_err(bq_charger->dev, "Charger enable failed %d", ret);
		return ret;
	}

	ret = regmap_read(bq_charger->regmap, BQ2419X_SYS_STAT_REG, &val);
	if (ret < 0)
		dev_err(bq_charger->dev, "error reading reg: 0x%x\n",
				BQ2419X_SYS_STAT_REG);

	if (max_uA == 0 && val != 0)
		return ret;

	bq_charger->last_charging_current = max_uA;
	bq_charger->in_current_limit = max_uA/1000;
	if ((val & BQ2419x_VBUS_STAT) == BQ2419x_VBUS_UNKNOWN) {
		bq_charger->cable_connected = 0;
		bq_charger->in_current_limit = 500;
		bq_charger->chg_status = BATTERY_DISCHARGING;
		battery_charger_release_wake_lock(bq_charger->bc_dev);
	} else {
		bq_charger->cable_connected = 1;
		bq_charger->chg_status = BATTERY_CHARGING;
		battery_charger_acquire_wake_lock(bq_charger->bc_dev);
	}
	ret = bq2419x_init(bq_charger);
	if (ret < 0)
		goto error;
	battery_charging_status_update(bq_charger->bc_dev,
					bq_charger->chg_status);
	return 0;
error:
	dev_err(bq_charger->dev, "Charger enable failed, err = %d\n", ret);
	return ret;
}

static struct regulator_ops bq2419x_tegra_regulator_ops = {
	.set_current_limit = bq2419x_set_charging_current,
};

static int bq2419x_reset_wdt(struct bq2419x_chip *bq2419x, const char *from)
{
	int ret = 0;
	unsigned int reg01;

	mutex_lock(&bq2419x->mutex);
	if (bq2419x->suspended)
		goto scrub;

	dev_info(bq2419x->dev, "%s() from %s()\n", __func__, from);

	/* Clear EN_HIZ */
	ret = regmap_update_bits(bq2419x->regmap,
			BQ2419X_INPUT_SRC_REG, BQ2419X_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev, "INPUT_SRC_REG update failed:%d\n", ret);
		goto scrub;
	}

	ret = regmap_read(bq2419x->regmap, BQ2419X_PWR_ON_REG, &reg01);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG read failed: %d\n", ret);
		goto scrub;
	}

	reg01 |= BIT(6);

	/* Write two times to make sure reset WDT */
	ret = regmap_write(bq2419x->regmap, BQ2419X_PWR_ON_REG, reg01);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG write failed: %d\n", ret);
		goto scrub;
	}
	ret = regmap_write(bq2419x->regmap, BQ2419X_PWR_ON_REG, reg01);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG write failed: %d\n", ret);
		goto scrub;
	}

scrub:
	mutex_unlock(&bq2419x->mutex);
	return ret;
}

static int bq2419x_fault_clear_sts(struct bq2419x_chip *bq2419x)
{
	int ret;
	unsigned int reg09;

	ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &reg09);
	if (ret < 0) {
		dev_err(bq2419x->dev, "FAULT_REG read failed: %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &reg09);
	if (ret < 0)
		dev_err(bq2419x->dev, "FAULT_REG read failed: %d\n", ret);

	return ret;
}

static int bq2419x_watchdog_init(struct bq2419x_chip *bq2419x,
			int timeout, const char *from)
{
	int ret, val;
	unsigned int reg05;

	if (!timeout) {
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
				BQ2419X_WD_MASK, 0);
		if (ret < 0)
			dev_err(bq2419x->dev,
				"TIME_CTRL_REG read failed: %d\n", ret);
		return ret;
	}

	if (timeout <= 60) {
		val = BQ2419X_WD_40ms;
		bq2419x->wdt_refresh_timeout = 25;
	} else if (timeout <= 120) {
		val = BQ2419X_WD_80ms;
		bq2419x->wdt_refresh_timeout = 50;
	} else {
		val = BQ2419X_WD_160ms;
		bq2419x->wdt_refresh_timeout = 125;
	}

	ret = regmap_read(bq2419x->regmap, BQ2419X_TIME_CTRL_REG, &reg05);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"TIME_CTRL_REG read failed:%d\n", ret);
		return ret;
	}

	if ((reg05 & BQ2419X_WD_MASK) != val) {
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
				BQ2419X_WD_MASK, val);
		if (ret < 0) {
			dev_err(bq2419x->dev,
				"TIME_CTRL_REG read failed: %d\n", ret);
			return ret;
		}
	}

	ret = bq2419x_reset_wdt(bq2419x, from);
	if (ret < 0)
		dev_err(bq2419x->dev, "bq2419x_reset_wdt failed: %d\n", ret);

	return ret;
}

static void bq2419x_work_thread(struct kthread_work *work)
{
	struct bq2419x_chip *bq2419x = container_of(work,
			struct bq2419x_chip, bq_wdt_work);
	int ret;
	int val = 0;

	for (;;) {
		if (bq2419x->stop_thread)
			return;

		if (bq2419x->chg_restart_timeout) {
			mutex_lock(&bq2419x->mutex);
			bq2419x->chg_restart_timeout--;
			if (!bq2419x->chg_restart_timeout) {
				ret = bq2419x_charger_enable(bq2419x);
				if (ret < 0)
					dev_err(bq2419x->dev,
					"Charger enable failed %d", ret);
				ret = regmap_read(bq2419x->regmap,
					BQ2419X_SYS_STAT_REG, &val);
				if (ret < 0)
					dev_err(bq2419x->dev,
					"SYS_STAT_REG read failed %d\n", ret);
				/*
				* Update Charging status based on STAT register
				*/
				if ((val & BQ2419x_CHRG_STATE_MASK) ==
					BQ2419x_CHRG_STATE_NOTCHARGING) {
					bq2419x->chg_status = BATTERY_DISCHARGING;
					battery_charging_status_update(bq2419x->bc_dev,
						bq2419x->chg_status);

					bq2419x->chg_restart_timeout =
						bq2419x->chg_restart_time /
						bq2419x->wdt_refresh_timeout;
				} else {
					bq2419x->chg_status = BATTERY_CHARGING;
					battery_charging_status_update(bq2419x->bc_dev,
						bq2419x->chg_status);
				}

			}

			if (bq2419x->suspended)
				bq2419x->chg_restart_timeout = 0;

			mutex_unlock(&bq2419x->mutex);
		}

		ret = bq2419x_reset_wdt(bq2419x, "THREAD");
		if (ret < 0)
			dev_err(bq2419x->dev,
				"bq2419x_reset_wdt failed: %d\n", ret);

		msleep(bq2419x->wdt_refresh_timeout * 1000);
	}
}

static int bq2419x_reset_safety_timer(struct bq2419x_chip *bq2419x)
{
	int ret;

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
			BQ2419X_EN_SFT_TIMER_MASK, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev,
				"TIME_CTRL_REG update failed: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
			BQ2419X_EN_SFT_TIMER_MASK, BQ2419X_EN_SFT_TIMER_MASK);
	if (ret < 0)
		dev_err(bq2419x->dev,
				"TIME_CTRL_REG update failed: %d\n", ret);
	return ret;
}

static irqreturn_t bq2419x_irq(int irq, void *data)
{
	struct bq2419x_chip *bq2419x = data;
	int ret;
	unsigned int val;
	int check_chg_state = 0;

	ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "FAULT_REG read failed %d\n", ret);
		return ret;
	}

	dev_info(bq2419x->dev, "%s() Irq %d status 0x%02x\n",
		__func__, irq, val);

	if (val & BQ2419x_FAULT_BOOST_FAULT)
		dev_err(bq2419x->dev, "Charging Fault: VBUS Overloaded\n");

	if (!bq2419x->battery_presense)
		return IRQ_HANDLED;

	if (val & BQ2419x_FAULT_WATCHDOG_FAULT) {
		dev_err(bq2419x->dev,
			"Charging Fault: Watchdog Timer Expired\n");
		ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec,
						"ISR");
		if (ret < 0) {
			dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
			return ret;
		}

		ret = bq2419x_charger_init(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
			return ret;
		}

		ret = bq2419x_init(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "bq2419x init failed: %d\n", ret);
			return ret;
		}
	}

	switch (val & BQ2419x_FAULT_CHRG_FAULT_MASK) {
	case BQ2419x_FAULT_CHRG_INPUT:
		dev_err(bq2419x->dev, "Charging Fault: "
				"Input Fault (VBUS OVP or VBAT<VBUS<3.8V)\n");
		break;
	case BQ2419x_FAULT_CHRG_THERMAL:
		dev_err(bq2419x->dev, "Charging Fault: Thermal shutdown\n");
		check_chg_state = 1;
		break;
	case BQ2419x_FAULT_CHRG_SAFTY:
		dev_err(bq2419x->dev,
			"Charging Fault: Safety timer expiration\n");
		bq2419x->chg_restart_timeout = bq2419x->chg_restart_time /
						bq2419x->wdt_refresh_timeout;
		ret = bq2419x_reset_safety_timer(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "Reset safety timer failed %d\n",
							ret);
			return ret;
		}

		check_chg_state = 1;
		break;
	default:
		break;
	}

	if (val & BQ2419x_FAULT_NTC_FAULT) {
		dev_err(bq2419x->dev, "Charging Fault: NTC fault %d\n",
				val & BQ2419x_FAULT_NTC_FAULT);
		check_chg_state = 1;
	}

	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		return ret;
	}

	ret = regmap_read(bq2419x->regmap, BQ2419X_SYS_STAT_REG, &val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "SYS_STAT_REG read failed %d\n", ret);
		return ret;
	}

	if ((val & BQ2419x_CHRG_STATE_MASK) ==
				BQ2419x_CHRG_STATE_CHARGE_DONE) {
		bq2419x->chg_restart_timeout = bq2419x->chg_restart_time /
						bq2419x->wdt_refresh_timeout;
		dev_info(bq2419x->dev, "Charging completed\n");
		bq2419x->chg_status = BATTERY_CHARGING_DONE;
		battery_charging_status_update(bq2419x->bc_dev,
					bq2419x->chg_status);
		battery_charging_restart(bq2419x->bc_dev,
					bq2419x->chg_restart_time);
		battery_charger_release_wake_lock(bq2419x->bc_dev);
	}

	if ((val & BQ2419x_VSYS_STAT_MASK) ==
				BQ2419x_VSYS_STAT_BATT_LOW)
		dev_info(bq2419x->dev,
			"In VSYSMIN regulation, battery is too low\n");

	/*
	* Update Charging status based on STAT register
	*/
	if (check_chg_state) {
		if ((val & BQ2419x_CHRG_STATE_MASK) ==
				BQ2419x_CHRG_STATE_NOTCHARGING) {
			bq2419x->chg_status = BATTERY_DISCHARGING;
			battery_charging_status_update(bq2419x->bc_dev,
					bq2419x->chg_status);
			battery_charging_restart(bq2419x->bc_dev,
					bq2419x->chg_restart_time);
		}
	}

	return IRQ_HANDLED;
}

static int bq2419x_init_charger_regulator(struct bq2419x_chip *bq2419x,
		struct bq2419x_platform_data *pdata)
{
	int ret = 0;
	struct regulator_config rconfig = { };

	if (!pdata->bcharger_pdata) {
		dev_err(bq2419x->dev, "No charger platform data\n");
		return 0;
	}

	bq2419x->chg_reg_desc.name  = "bq2419x-charger";
	bq2419x->chg_reg_desc.ops   = &bq2419x_tegra_regulator_ops;
	bq2419x->chg_reg_desc.type  = REGULATOR_CURRENT;
	bq2419x->chg_reg_desc.owner = THIS_MODULE;

	bq2419x->chg_reg_init_data.supply_regulator	= NULL;
	bq2419x->chg_reg_init_data.regulator_init	= NULL;
	bq2419x->chg_reg_init_data.num_consumer_supplies =
				pdata->bcharger_pdata->num_consumer_supplies;
	bq2419x->chg_reg_init_data.consumer_supplies	=
				pdata->bcharger_pdata->consumer_supplies;
	bq2419x->chg_reg_init_data.driver_data		= bq2419x;
	bq2419x->chg_reg_init_data.constraints.name	= "bq2419x-charger";
	bq2419x->chg_reg_init_data.constraints.min_uA	= 0;
	bq2419x->chg_reg_init_data.constraints.max_uA	=
			pdata->bcharger_pdata->max_charge_current_mA * 1000;

	bq2419x->chg_reg_init_data.constraints.valid_modes_mask =
						REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_STANDBY;

	bq2419x->chg_reg_init_data.constraints.valid_ops_mask =
						REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_CURRENT;

	rconfig.dev = bq2419x->dev;
	rconfig.of_node = NULL;
	rconfig.init_data = &bq2419x->chg_reg_init_data;
	rconfig.driver_data = bq2419x;
	bq2419x->chg_rdev = regulator_register(&bq2419x->chg_reg_desc,
					&rconfig);
	if (IS_ERR(bq2419x->chg_rdev)) {
		ret = PTR_ERR(bq2419x->chg_rdev);
		dev_err(bq2419x->dev,
			"vbus-charger regulator register failed %d\n", ret);
	}
	return ret;
}

static int bq2419x_init_vbus_regulator(struct bq2419x_chip *bq2419x,
		struct bq2419x_platform_data *pdata)
{
	int ret = 0;
	struct regulator_config rconfig = { };

	if (!pdata->vbus_pdata) {
		dev_err(bq2419x->dev, "No vbus platform data\n");
		return 0;
	}

	bq2419x->gpio_otg_iusb = pdata->vbus_pdata->gpio_otg_iusb;
	bq2419x->vbus_reg_desc.name = "bq2419x-vbus";
	bq2419x->vbus_reg_desc.ops = &bq2419x_vbus_ops;
	bq2419x->vbus_reg_desc.type = REGULATOR_VOLTAGE;
	bq2419x->vbus_reg_desc.owner = THIS_MODULE;

	bq2419x->vbus_reg_init_data.supply_regulator	= NULL;
	bq2419x->vbus_reg_init_data.regulator_init	= NULL;
	bq2419x->vbus_reg_init_data.num_consumer_supplies	=
				pdata->vbus_pdata->num_consumer_supplies;
	bq2419x->vbus_reg_init_data.consumer_supplies	=
				pdata->vbus_pdata->consumer_supplies;
	bq2419x->vbus_reg_init_data.driver_data		= bq2419x;

	bq2419x->vbus_reg_init_data.constraints.name	= "bq2419x-vbus";
	bq2419x->vbus_reg_init_data.constraints.min_uV	= 0;
	bq2419x->vbus_reg_init_data.constraints.max_uV	= 5000000,
	bq2419x->vbus_reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;
	bq2419x->vbus_reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_VOLTAGE;

	if (gpio_is_valid(bq2419x->gpio_otg_iusb)) {
		ret = gpio_request_one(bq2419x->gpio_otg_iusb,
				GPIOF_OUT_INIT_HIGH, dev_name(bq2419x->dev));
		if (ret < 0) {
			dev_err(bq2419x->dev, "gpio request failed  %d\n", ret);
			return ret;
		}
	}

	/* Register the regulators */
	rconfig.dev = bq2419x->dev;
	rconfig.of_node = NULL;
	rconfig.init_data = &bq2419x->vbus_reg_init_data;
	rconfig.driver_data = bq2419x;
	bq2419x->vbus_rdev = regulator_register(&bq2419x->vbus_reg_desc,
					&rconfig);
	if (IS_ERR(bq2419x->vbus_rdev)) {
		ret = PTR_ERR(bq2419x->vbus_rdev);
		dev_err(bq2419x->dev,
			"VBUS regulator register failed %d\n", ret);
		goto scrub;
	}

	/* Disable the VBUS regulator and enable charging */
	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charging enable failed %d", ret);
		goto scrub_reg;
	}
	return ret;

scrub_reg:
	regulator_unregister(bq2419x->vbus_rdev);
	bq2419x->vbus_rdev = NULL;
scrub:
	if (gpio_is_valid(bq2419x->gpio_otg_iusb))
		gpio_free(bq2419x->gpio_otg_iusb);
	return ret;
}

static int bq2419x_show_chip_version(struct bq2419x_chip *bq2419x)
{
	int ret;
	unsigned int val;

	ret = regmap_read(bq2419x->regmap, BQ2419X_REVISION_REG, &val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "REVISION_REG read failed: %d\n", ret);
		return ret;
	}

	if ((val & BQ24190_IC_VER) == BQ24190_IC_VER)
		dev_info(bq2419x->dev, "chip type BQ24190 detected\n");
	else if ((val & BQ24192_IC_VER) == BQ24192_IC_VER)
		dev_info(bq2419x->dev, "chip type BQ2419X/3 detected\n");
	else if ((val & BQ24192i_IC_VER) == BQ24192i_IC_VER)
		dev_info(bq2419x->dev, "chip type BQ2419Xi detected\n");
	return 0;
}

static int bq2419x_wakealarm(struct bq2419x_chip *bq2419x, int time_sec)
{
	int ret;
	unsigned long now;
	struct rtc_wkalrm alm;
	int alarm_time = time_sec;

	if (!alarm_time)
		return 0;

	alm.enabled = true;
	ret = rtc_read_time(bq2419x->rtc, &alm.time);
	if (ret < 0) {
		dev_err(bq2419x->dev, "RTC read time failed %d\n", ret);
		return ret;
	}
	rtc_tm_to_time(&alm.time, &now);

	rtc_time_to_tm(now + alarm_time, &alm.time);
	ret = rtc_set_alarm(bq2419x->rtc, &alm);
	if (ret < 0) {
		dev_err(bq2419x->dev, "RTC set alarm failed %d\n", ret);
		alm.enabled = false;
		return ret;
	}
	alm.enabled = false;
	return 0;
}

static int fchg_curr_to_reg(int curr)
{
	int ret;

	ret = (curr - 500) / 64;
	ret = ret << 2;

	return ret;
}

static int fchg_reg_to_curr(int reg_val)
{
	int ret;

	ret = (reg_val * 64) + 500;

	return ret;
}

static ssize_t bq2419x_show_output_charging_current(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);
	int reg_val, curr_val, ret;

	ret = regmap_read(bq2419x->regmap, BQ2419X_CHRG_CTRL_REG, &reg_val);

	reg_val &= (~3);
	reg_val = reg_val >> 2;

	curr_val = fchg_reg_to_curr(reg_val);

	return snprintf(buf, MAX_STR_PRINT, "%d mA\n", curr_val);
}

static ssize_t bq2419x_set_output_charging_current(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);
	int curr_val, ret, i;

	if (kstrtouint(buf, 0, &curr_val)) {
		dev_err(dev, "\nfile: %s, line=%d return %s()",
					__FILE__, __LINE__, __func__);
		return -EINVAL;
	}

	for (i = 0; i <= 63; i++) {
		if (curr_val == fchg_reg_to_curr(i))
			break;
	}

	if (i == 64) {
		dev_err(dev, "Invalid output current value..\n");
		return -EINVAL;
	}

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_CHRG_CTRL_REG,
				BQ2419X_CHARGE_CURRENT_MASK,
				fchg_curr_to_reg(curr_val));

	return count;
}

static ssize_t bq2419x_show_output_charging_current_values(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int i, ret = 0;

	for (i = 0; i <= 63; i++) {
		ret +=
		snprintf(buf + strlen(buf), MAX_STR_PRINT,
				"%d mA\n", fchg_reg_to_curr(i));
	}

	return ret;
}

static DEVICE_ATTR(output_charging_current, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		bq2419x_show_output_charging_current,
		bq2419x_set_output_charging_current);

static DEVICE_ATTR(output_current_allowed_values, S_IRUGO,
		bq2419x_show_output_charging_current_values, NULL);

static struct attribute *bq2419x_attributes[] = {
	&dev_attr_output_charging_current.attr,
	&dev_attr_output_current_allowed_values.attr,
	NULL
};

static const struct attribute_group bq2419x_attr_group = {
	.attrs = bq2419x_attributes,
};

static int bq2419x_charger_get_status(struct battery_charger_dev *bc_dev)
{
	struct bq2419x_chip *bq2419x = battery_charger_get_drvdata(bc_dev);

	return bq2419x->chg_status;
}

static int bq2419x_charging_restart(struct battery_charger_dev *bc_dev)
{
        struct bq2419x_chip *bq2419x = battery_charger_get_drvdata(bc_dev);
        int ret;

        if (!bq2419x->cable_connected)
                return 0;

        dev_info(bq2419x->dev, "Restarting the charging\n");
        ret = bq2419x_set_charging_current(bq2419x->chg_rdev,
                        bq2419x->last_charging_current,
                        bq2419x->last_charging_current);
        if (ret < 0) {
                dev_err(bq2419x->dev, "restarting of charging failed: %d\n", ret);
                battery_charging_restart(bq2419x->bc_dev,
                                bq2419x->chg_restart_time);
        }
        return ret;
}


static struct battery_charging_ops bq2419x_charger_bci_ops = {
	.get_charging_status = bq2419x_charger_get_status,
	.restart_charging = bq2419x_charging_restart,
};

static struct battery_charger_info bq2419x_charger_bci = {
	.cell_id = 0,
	.bc_ops = &bq2419x_charger_bci_ops,
};

static struct bq2419x_platform_data *
bq2419x_dt_parse(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct bq2419x_platform_data *pdata;
	struct device_node *batt_reg_node;
	struct device_node *vbus_reg_node;
	int wdt_timeout;
	int rtc_alarm_time;
	int num_consumer_supplies;
	int chg_restart_time;
	struct regulator_init_data *batt_init_data;
	struct regulator_init_data *vbus_init_data;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return PTR_ERR(-ENOMEM);

	pdata->vbus_pdata = devm_kzalloc(&client->dev,
			sizeof(*(pdata->vbus_pdata)), GFP_KERNEL);
	if (!pdata->vbus_pdata)
		return NULL;

	batt_reg_node = of_find_node_by_name(np, "charger");
	if (!batt_reg_node)
		return NULL;

	if (batt_reg_node) {
		of_property_read_u32(batt_reg_node, "watchdog-timeout", &wdt_timeout);
		of_property_read_u32(batt_reg_node, "rtc-alarm-time", &rtc_alarm_time);
		of_property_read_u32(batt_reg_node, "auto-recharge-time", &chg_restart_time);
		pdata->bcharger_pdata = devm_kzalloc(&client->dev,
				sizeof(*(pdata->bcharger_pdata)), GFP_KERNEL);
		if (!pdata->bcharger_pdata)
			return PTR_ERR(-ENOMEM);
		batt_init_data = of_get_regulator_init_data(&client->dev,
								batt_reg_node);
		if (!batt_init_data)
			return NULL;

		pdata->bcharger_pdata->consumer_supplies =
					batt_init_data->consumer_supplies;
		pdata->bcharger_pdata->num_consumer_supplies =
					batt_init_data->num_consumer_supplies;
		pdata->bcharger_pdata->max_charge_current_mA =
						batt_init_data->constraints.max_uA;
		pdata->bcharger_pdata->wdt_timeout = wdt_timeout;
		pdata->bcharger_pdata->rtc_alarm_time = rtc_alarm_time;
		pdata->bcharger_pdata->chg_restart_time = chg_restart_time;
	}

	vbus_reg_node = of_find_node_by_name(np, "vbus");
	if (!vbus_reg_node)
		return NULL;

	vbus_init_data = of_get_regulator_init_data(&client->dev,
							vbus_reg_node);
	if (!vbus_init_data)
		return NULL;

	pdata->vbus_pdata->consumer_supplies =
				vbus_init_data->consumer_supplies;
	pdata->vbus_pdata->num_consumer_supplies =
				vbus_init_data->num_consumer_supplies;

	return pdata;
}

static int bq2419x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bq2419x_chip *bq2419x;
	struct bq2419x_platform_data *pdata;
	int ret = 0;

	if (client->dev.platform_data)
		pdata = client->dev.platform_data;
	else if (client->dev.of_node)
		pdata = bq2419x_dt_parse(client);

	if (!pdata) {
		dev_err(&client->dev, "No Platform data");
		return -EINVAL;
	}

	bq2419x = devm_kzalloc(&client->dev, sizeof(*bq2419x), GFP_KERNEL);
	if (!bq2419x) {
		dev_err(&client->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	bq2419x->regmap = devm_regmap_init_i2c(client, &bq2419x_regmap_config);
	if (IS_ERR(bq2419x->regmap)) {
		ret = PTR_ERR(bq2419x->regmap);
		dev_err(&client->dev, "regmap init failed with err %d\n", ret);
		return ret;
	}

	bq2419x->dev = &client->dev;

	if (pdata->bcharger_pdata) {
		bq2419x->rtc_alarm_time	= pdata->bcharger_pdata->rtc_alarm_time;
		bq2419x->wdt_time_sec	= pdata->bcharger_pdata->wdt_timeout;
		bq2419x->chg_restart_time =
					pdata->bcharger_pdata->chg_restart_time;
		bq2419x->battery_presense	= true;
	}

	bq2419x->wdt_refresh_timeout = 25;
	i2c_set_clientdata(client, bq2419x);
	bq2419x->irq = client->irq;

	if (bq2419x->rtc_alarm_time)
		bq2419x->rtc = alarmtimer_get_rtcdev();

	ret = sysfs_create_group(&client->dev.kobj, &bq2419x_attr_group);

	mutex_init(&bq2419x->mutex);
	bq2419x->suspended = 0;
	bq2419x->chg_restart_timeout = 0;

	ret = bq2419x_show_chip_version(bq2419x);
	if (ret < 0) {
		dev_err(&client->dev, "version read failed %d\n", ret);
		goto scrub_mutex;
	}

	ret = bq2419x_init_vbus_regulator(bq2419x, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "VBUS regulator init failed %d\n", ret);
		goto scrub_mutex;
	}

	if (!pdata->bcharger_pdata) {
		dev_info(&client->dev, "No battery charger supported\n");
		ret = bq2419x_watchdog_init(bq2419x, 0, "PROBE");
		if (ret < 0) {
			dev_err(bq2419x->dev, "WDT disable failed: %d\n", ret);
			goto scrub_vbus_reg;
		}
		goto skip_bcharger_init;
	}

	ret = bq2419x_charger_init(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
		goto scrub_vbus_reg;
	}

	ret = bq2419x_init_charger_regulator(bq2419x, pdata);
	if (ret < 0) {
		dev_err(&client->dev,
			"Charger regualtor init failed %d\n", ret);
		goto scrub_vbus_reg;
	}

	bq2419x_charger_bci.tz_name = pdata->bcharger_pdata->tz_name;
	bq2419x->bc_dev = battery_charger_register(bq2419x->dev,
			&bq2419x_charger_bci, bq2419x);
	if (IS_ERR(bq2419x->bc_dev)) {
		ret = PTR_ERR(bq2419x->bc_dev);
		dev_err(bq2419x->dev, "battery charger register failed: %d\n",
			ret);
		goto scrub_chg_reg;
	}
	init_kthread_worker(&bq2419x->bq_kworker);
	bq2419x->bq_kworker_task = kthread_run(kthread_worker_fn,
				&bq2419x->bq_kworker,
				dev_name(bq2419x->dev));
	if (IS_ERR(bq2419x->bq_kworker_task)) {
		ret = PTR_ERR(bq2419x->bq_kworker_task);
		dev_err(&client->dev, "Kworker task creation failed %d\n", ret);
		goto scrub_bchg_reg;
	}

	init_kthread_work(&bq2419x->bq_wdt_work, bq2419x_work_thread);
	sched_setscheduler(bq2419x->bq_kworker_task,
			SCHED_FIFO, &bq2419x_param);
	queue_kthread_work(&bq2419x->bq_kworker, &bq2419x->bq_wdt_work);

	ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec, "PROBE");
	if (ret < 0) {
		dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
		goto scrub_kthread;
	}

skip_bcharger_init:
	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		goto scrub_kthread;
	}

	ret = request_threaded_irq(bq2419x->irq, NULL,
		bq2419x_irq, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			dev_name(bq2419x->dev), bq2419x);
	if (ret < 0) {
		dev_warn(bq2419x->dev, "request IRQ %d fail, err = %d\n",
				bq2419x->irq, ret);
		dev_info(bq2419x->dev,
			"Supporting bq driver without intierrupt\n");
		ret = 0;
	}

	if (!pdata->bcharger_pdata) {
		/* enable charging */
		ret = bq2419x_charger_enable(bq2419x);
		if (ret < 0)
			goto scrub_irq;
	}

	return 0;
scrub_irq:
	if (bq2419x->irq)
		free_irq(bq2419x->irq, bq2419x);
scrub_kthread:
	if (pdata->bcharger_pdata) {
		bq2419x->stop_thread = true;
		flush_kthread_worker(&bq2419x->bq_kworker);
		kthread_stop(bq2419x->bq_kworker_task);
	}
scrub_bchg_reg:
	if (pdata->bcharger_pdata)
		battery_charger_unregister(bq2419x->bc_dev);
scrub_chg_reg:
	if (pdata->bcharger_pdata)
		regulator_unregister(bq2419x->chg_rdev);
scrub_vbus_reg:
	regulator_unregister(bq2419x->vbus_rdev);
scrub_mutex:
	mutex_destroy(&bq2419x->mutex);
	return ret;
}

static int bq2419x_remove(struct i2c_client *client)
{
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);

	if (bq2419x->irq)
		free_irq(bq2419x->irq, bq2419x);
	if (bq2419x->battery_presense) {
		battery_charger_unregister(bq2419x->bc_dev);
		regulator_unregister(bq2419x->chg_rdev);
		bq2419x->stop_thread = true;
		flush_kthread_worker(&bq2419x->bq_kworker);
		kthread_stop(bq2419x->bq_kworker_task);
	}
	regulator_unregister(bq2419x->vbus_rdev);
	mutex_destroy(&bq2419x->mutex);
	return 0;
}

static void bq2419x_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);
	int alarm_time = bq2419x->rtc_alarm_time;

	if (!bq2419x->battery_presense)
		return;

	if (bq2419x->irq)
		disable_irq(bq2419x->irq);

	if (alarm_time && !bq2419x->rtc)
		bq2419x->rtc = alarmtimer_get_rtcdev();

	if (alarm_time && (bq2419x->in_current_limit > 500)) {
		dev_info(bq2419x->dev, "HighCurrent %dmA charger is connectd\n",
			bq2419x->in_current_limit);
		ret = bq2419x_reset_wdt(bq2419x, "shutdown");
		if (ret < 0)
			dev_err(bq2419x->dev,
				"bq2419x_reset_wdt failed: %d\n", ret);
		alarm_time = 20;
	}

	mutex_lock(&bq2419x->mutex);
	bq2419x->suspended = 1;
	mutex_unlock(&bq2419x->mutex);

	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0)
		dev_err(bq2419x->dev, "Charger enable failed %d", ret);

	if (alarm_time && (bq2419x->in_current_limit <= 500)) {
		/* Configure charging current to 500mA */
		ret = regmap_write(bq2419x->regmap,
				BQ2419X_INPUT_SRC_REG, 0x32);
		if (ret < 0)
			dev_err(bq2419x->dev,
				"INPUT_SRC_REG write failed %d\n", ret);
	}

	ret = bq2419x_wakealarm(bq2419x, alarm_time);
	if (ret < 0)
		dev_err(bq2419x->dev, "RTC wake alarm config failed %d\n", ret);
}

#ifdef CONFIG_PM_SLEEP
static int bq2419x_suspend(struct device *dev)
{
	int ret = 0;
	struct bq2419x_chip *bq2419x = dev_get_drvdata(dev);

	if (!bq2419x->battery_presense)
		return 0;

	mutex_lock(&bq2419x->mutex);
	bq2419x->suspended = 1;
	mutex_unlock(&bq2419x->mutex);
	disable_irq(bq2419x->irq);
	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger enable failed %d", ret);
		return ret;
	}

	/* Configure charging current to 500mA */
	ret = regmap_write(bq2419x->regmap, BQ2419X_INPUT_SRC_REG, 0x32);
	if (ret < 0) {
		dev_err(bq2419x->dev, "INPUT_SRC_REG write failed %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq2419x_resume(struct device *dev)
{
	int ret = 0;
	struct bq2419x_chip *bq2419x = dev_get_drvdata(dev);
	unsigned int val;

	if (!bq2419x->battery_presense)
		return 0;

	ret = regmap_read(bq2419x->regmap, BQ2419X_FAULT_REG, &val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "FAULT_REG read failed %d\n", ret);
		return ret;
	}

	if (val & BQ2419x_FAULT_WATCHDOG_FAULT) {
		dev_err(bq2419x->dev,
			"Charging Fault: Watchdog Timer Expired\n");

		ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec,
						"RESUME");
		if (ret < 0) {
			dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
			return ret;
		}
	}

	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		return ret;
	}

	ret = bq2419x_charger_init(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
		return ret;
	}

	ret = bq2419x_init(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "bq2419x init failed: %d\n", ret);
		return ret;
	}

	mutex_lock(&bq2419x->mutex);
	bq2419x->suspended = 0;
	mutex_unlock(&bq2419x->mutex);
	if (gpio_is_valid(bq2419x->gpio_otg_iusb))
		gpio_set_value(bq2419x->gpio_otg_iusb, 1);

	enable_irq(bq2419x->irq);
	return 0;
};
#endif

static const struct dev_pm_ops bq2419x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bq2419x_suspend,
				bq2419x_resume)
};

static const struct i2c_device_id bq2419x_id[] = {
	{.name = "bq2419x",},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2419x_id);

static struct i2c_driver bq2419x_i2c_driver = {
	.driver = {
		.name	= "bq2419x",
		.owner	= THIS_MODULE,
		.pm = &bq2419x_pm_ops,
	},
	.probe		= bq2419x_probe,
	.remove		= bq2419x_remove,
	.shutdown	= bq2419x_shutdown,
	.id_table	= bq2419x_id,
};

static int __init bq2419x_module_init(void)
{
	return i2c_add_driver(&bq2419x_i2c_driver);
}
subsys_initcall(bq2419x_module_init);

static void __exit bq2419x_cleanup(void)
{
	i2c_del_driver(&bq2419x_i2c_driver);
}
module_exit(bq2419x_cleanup);

MODULE_DESCRIPTION("BQ24190/BQ24192/BQ24192i/BQ24193 battery charger driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com");
MODULE_LICENSE("GPL v2");
