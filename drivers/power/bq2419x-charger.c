/*
 * bq2419x-charger.c -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger driver
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/regmap.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/alarmtimer.h>
#include <linux/power/battery-charger-gauge-comm.h>
#include <linux/workqueue.h>

#define MAX_STR_PRINT 50

#define bq_chg_err(bq, fmt, ...)			\
		dev_err(bq->dev, "Charging Fault: " fmt, ##__VA_ARGS__)

enum charging_states {
	ENABLED_HALF_IBAT = 1,
	ENABLED_FULL_IBAT,
	DISABLED,
};

static const unsigned int bq2419x_maxcharge_voltage_lookup[] = {
	3504, 3520, 3536, 3552, 3568, 3584, 3600, 3616,
	3632, 3648, 3664, 3680, 3696, 3712, 3728, 3744,
	3760, 3776, 3792, 3808, 3824, 3840, 3856, 3872,
	3888, 3904, 3920, 3936, 3952, 3968, 3984, 4000,
	4016, 4032, 4048, 4064, 4080, 4096, 4112, 4128,
	4144, 4160, 4176, 4192, 4208, 4224, 4240, 4256,
	4272, 4288, 4304, 4320, 4336, 4352, 4368, 4384,
	4400, 4416, 4432, 4448, 4464, 4480, 4496, 4512,
};

/* input current limit */
static const unsigned int iinlim[] = {
	100, 150, 500, 900, 1200, 1500, 2000, 3000,
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

	struct regulator_dev		*chg_rdev;
	struct regulator_desc		chg_reg_desc;
	struct regulator_init_data	chg_reg_init_data;

	struct regulator_dev		*vbus_rdev;
	struct regulator_desc		vbus_reg_desc;
	struct regulator_init_data	vbus_reg_init_data;

	struct battery_charger_dev	*bc_dev;
	int				chg_status;

	struct delayed_work		wdt_restart_wq;

	int				chg_restart_time;
	int				battery_presense;
	bool				cable_connected;
	int				last_charging_current;
	bool				disable_suspend_during_charging;
	int				charging_state;
};

static inline int convert_to_reg(int x)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bq2419x_maxcharge_voltage_lookup); i++) {
		if (x < bq2419x_maxcharge_voltage_lookup[i])
			break;
	}
	return i > 0 ? i - 1 : -EINVAL;
}

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
		/* set default Charge regulation voltage */
		ret = regmap_write(bq2419x->regmap, BQ2419X_VOLT_CTRL_REG,
				BQ2419x_DEFAULT_CHARGE_VOLTAGE);
		if (ret < 0) {
			dev_err(bq2419x->dev,
				"VOLT_CTRL_REG write failed %d\n", ret);
			return ret;
		}
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_PWR_ON_REG,
				BQ2419X_ENABLE_CHARGE_MASK,
				BQ2419X_ENABLE_CHARGE);
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
	if (ret < 0)
		dev_err(bq2419x->dev, "Charger enable failed %d", ret);
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
};

static int bq2419x_charger_init(struct bq2419x_chip *bq2419x)
{
	int ret;

	/* Configure Output Current Control to 2.25A*/
	ret = regmap_write(bq2419x->regmap, BQ2419X_CHRG_CTRL_REG, 0xFC);
	if (ret < 0) {
		dev_err(bq2419x->dev, "CHRG_CTRL_REG write failed %d\n", ret);
		return ret;
	}

	/*
	 * Configure Pre-charge current limit to 768mA,
	 * and Termination current limit to 384mA.
	*/
	ret = regmap_write(bq2419x->regmap, BQ2419X_CHRG_TERM_REG, 0x52);
	if (ret < 0) {
		dev_err(bq2419x->dev, "CHRG_TERM_REG write failed %d\n", ret);
		return ret;
	}

	/*
	 * Configure Input voltage limit reset to OTP value,
	 * and charging current to 500mA.
	 */
	ret = regmap_write(bq2419x->regmap, BQ2419X_INPUT_SRC_REG, 0x22);
	if (ret < 0)
		dev_err(bq2419x->dev, "INPUT_SRC_REG write failed %d\n", ret);

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_THERM_REG,
			BQ2419x_TREG, BQ2419x_TREG_100_C);
	if (ret < 0)
		dev_err(bq2419x->dev, "THERM_REG write failed: %d\n", ret);

	return ret;
}

static int bq2419x_configure_charging_current(struct bq2419x_chip *bq2419x,
	int in_current_limit)
{
	int val = 0;
	int ret = 0;
	int floor = 0;

	/* Clear EN_HIZ */
	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
			BQ2419X_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev, "INPUT_SRC_REG update failed %d\n", ret);
		return ret;
	}

	/* Configure input current limit in steps */
	val = current_to_reg(iinlim, ARRAY_SIZE(iinlim), in_current_limit);
	floor = current_to_reg(iinlim, ARRAY_SIZE(iinlim), 500);
	if (val < 0 || floor < 0)
		return 0;

	for (; floor <= val; floor++) {
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_INPUT_SRC_REG,
				BQ2419x_CONFIG_MASK, floor);
		if (ret < 0)
			dev_err(bq2419x->dev,
				"INPUT_SRC_REG update failed: %d\n", ret);
		udelay(BQ2419x_CHARGING_CURRENT_STEP_DELAY_US);
	}
	bq2419x->in_current_limit = in_current_limit;
	return ret;
}

int bq2419x_full_current_enable(struct bq2419x_chip *bq2419x)
{
	int ret;
	int in_current_limit;

	in_current_limit = bq2419x->last_charging_current/1000;
	ret = bq2419x_configure_charging_current(bq2419x, in_current_limit);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"Failed to initialise full current charging\n");
		return ret;
	}

	bq2419x->charging_state = ENABLED_FULL_IBAT;

	return 0;
}

int bq2419x_half_current_enable(struct bq2419x_chip *bq2419x)
{
	int ret;
	int in_current_limit;

	in_current_limit = bq2419x->last_charging_current/2000;
	ret = bq2419x_configure_charging_current(bq2419x, in_current_limit);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"Failed to initialise half current charging\n");
		return ret;
	}
	bq2419x->charging_state = ENABLED_HALF_IBAT;

	return 0;
}

static int bq2419x_set_charging_current(struct regulator_dev *rdev,
			int min_uA, int max_uA)
{
	struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
	int in_current_limit;
	int ret = 0;
	int val;

	dev_info(bq2419x->dev, "Setting charging current %d\n", max_uA/1000);
	msleep(200);
	bq2419x->chg_status = BATTERY_DISCHARGING;

	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger enable failed %d", ret);
		return ret;
	}

	ret = regmap_read(bq2419x->regmap, BQ2419X_SYS_STAT_REG, &val);
	if (ret < 0)
		dev_err(bq2419x->dev, "SYS_STAT_REG read failed: %d\n", ret);

	if (max_uA == 0 && val != 0)
		return ret;

	bq2419x->last_charging_current = max_uA;
	if ((val & BQ2419x_VBUS_STAT) == BQ2419x_VBUS_UNKNOWN) {
		battery_charging_restart_cancel(bq2419x->bc_dev);
		in_current_limit = 500;
		bq2419x->cable_connected = 0;
		bq2419x->chg_status = BATTERY_DISCHARGING;
		battery_charger_thermal_stop_monitoring(
				bq2419x->bc_dev);
	} else {
		in_current_limit = max_uA/1000;
		bq2419x->cable_connected = 1;
		bq2419x->chg_status = BATTERY_CHARGING;
		bq2419x->charging_state = ENABLED_FULL_IBAT;
		battery_charger_thermal_start_monitoring(
				bq2419x->bc_dev);
	}
	ret = bq2419x_configure_charging_current(bq2419x, in_current_limit);
	if (ret < 0)
		goto error;

	battery_charging_status_update(bq2419x->bc_dev, bq2419x->chg_status);
	if (bq2419x->disable_suspend_during_charging) {
		if (bq2419x->cable_connected) {
			if (in_current_limit > 500)
				battery_charger_acquire_wake_lock
							(bq2419x->bc_dev);
		} else {
			battery_charger_release_wake_lock(bq2419x->bc_dev);
		}
	}
	return 0;
error:
	dev_err(bq2419x->dev, "Charger enable failed, err = %d\n", ret);
	return ret;
}

static struct regulator_ops bq2419x_tegra_regulator_ops = {
	.set_current_limit = bq2419x_set_charging_current,
};

static int bq2419x_set_charging_current_suspend(struct bq2419x_chip *bq2419x,
			int in_current_limit)
{
	int ret;
	int val;

	dev_info(bq2419x->dev, "Setting charging current %d mA\n",
			in_current_limit);

	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger enable failed %d", ret);
		return ret;
	}

	ret = regmap_read(bq2419x->regmap, BQ2419X_SYS_STAT_REG, &val);
	if (ret < 0)
		dev_err(bq2419x->dev, "SYS_STAT_REG read failed: %d\n", ret);

	if (!bq2419x->cable_connected) {
		battery_charging_restart_cancel(bq2419x->bc_dev);
		ret = bq2419x_configure_charging_current(bq2419x,
				in_current_limit);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int bq2419x_reset_wdt(struct bq2419x_chip *bq2419x, const char *from)
{
	int ret = 0;
	unsigned int reg01;
	int timeout;

	mutex_lock(&bq2419x->mutex);
	dev_info(bq2419x->dev, "%s() from %s()\n", __func__, from);

	/* Clear EN_HIZ */
	ret = regmap_update_bits(bq2419x->regmap,
			BQ2419X_INPUT_SRC_REG, BQ2419X_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev, "INPUT_SRC_REG update failed:%d\n", ret);
		goto scrub;
	}

	if (!bq2419x->wdt_refresh_timeout)
		goto scrub;

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

	timeout = bq2419x->wdt_refresh_timeout ? : 100;
	schedule_delayed_work(&bq2419x->wdt_restart_wq, timeout * HZ);
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
			dev_err(bq2419x->dev, "TIME_CTRL_REG read failed: %d\n",
				ret);
		bq2419x->wdt_refresh_timeout = 0;
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
		dev_err(bq2419x->dev, "TIME_CTRL_REG read failed:%d\n", ret);
		return ret;
	}

	/* Reset WDT to be safe if about to end */
	ret = bq2419x_reset_wdt(bq2419x, from);
	if (ret < 0)
		dev_err(bq2419x->dev, "bq2419x_reset_wdt failed: %d\n", ret);

	if ((reg05 & BQ2419X_WD_MASK) != val) {
		ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
				BQ2419X_WD_MASK, val);
		if (ret < 0) {
			dev_err(bq2419x->dev, "TIME_CTRL_REG read failed: %d\n",
				ret);
			return ret;
		}
	}

	ret = bq2419x_reset_wdt(bq2419x, from);
	if (ret < 0)
		dev_err(bq2419x->dev, "bq2419x_reset_wdt failed: %d\n", ret);

	return ret;
}

static void bq2419x_wdt_restart_wq(struct work_struct *work)
{
	struct bq2419x_chip *bq2419x;
	int ret;

	bq2419x = container_of(work, struct bq2419x_chip, wdt_restart_wq.work);
	ret = bq2419x_reset_wdt(bq2419x, "THREAD");
	if (ret < 0)
		dev_err(bq2419x->dev, "bq2419x_reset_wdt failed: %d\n", ret);

}

static int bq2419x_reset_safety_timer(struct bq2419x_chip *bq2419x)
{
	int ret;

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
			BQ2419X_EN_SFT_TIMER_MASK, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev, "TIME_CTRL_REG update failed: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(bq2419x->regmap, BQ2419X_TIME_CTRL_REG,
			BQ2419X_EN_SFT_TIMER_MASK, BQ2419X_EN_SFT_TIMER_MASK);
	if (ret < 0)
		dev_err(bq2419x->dev, "TIME_CTRL_REG update failed: %d\n", ret);
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
		bq_chg_err(bq2419x, "VBUS Overloaded\n");

	if (!bq2419x->battery_presense)
		return IRQ_HANDLED;

	if (val & BQ2419x_FAULT_WATCHDOG_FAULT) {
		bq_chg_err(bq2419x, "WatchDog Expired\n");
		ret = bq2419x_watchdog_init(bq2419x,
					bq2419x->wdt_time_sec, "ISR");
		if (ret < 0) {
			dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
			return ret;
		}

		ret = bq2419x_charger_init(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
			return ret;
		}

		ret = bq2419x_configure_charging_current(bq2419x,
					bq2419x->in_current_limit);
		if (ret < 0) {
			dev_err(bq2419x->dev, "bq2419x init failed: %d\n", ret);
			return ret;
		}
	}

	switch (val & BQ2419x_FAULT_CHRG_FAULT_MASK) {
	case BQ2419x_FAULT_CHRG_INPUT:
		bq_chg_err(bq2419x,
			"Input Fault (VBUS OVP or VBAT<VBUS<3.8V)\n");
		break;
	case BQ2419x_FAULT_CHRG_THERMAL:
		bq_chg_err(bq2419x, "Thermal shutdown\n");
		check_chg_state = 1;
		break;
	case BQ2419x_FAULT_CHRG_SAFTY:
		bq_chg_err(bq2419x, "Safety timer expiration\n");
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
		bq_chg_err(bq2419x, "NTC fault %d\n",
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

	if ((val & BQ2419x_CHRG_STATE_MASK) == BQ2419x_CHRG_STATE_CHARGE_DONE) {
		dev_info(bq2419x->dev, "Charging completed\n");
		bq2419x->chg_status = BATTERY_CHARGING_DONE;
		battery_charging_status_update(bq2419x->bc_dev,
					bq2419x->chg_status);
		battery_charging_restart(bq2419x->bc_dev,
					bq2419x->chg_restart_time);
		if (bq2419x->disable_suspend_during_charging)
			battery_charger_release_wake_lock(bq2419x->bc_dev);
		battery_charger_release_wake_lock(bq2419x->bc_dev);
		battery_charger_thermal_stop_monitoring(
				bq2419x->bc_dev);
	}

	if ((val & BQ2419x_VSYS_STAT_MASK) == BQ2419x_VSYS_STAT_BATT_LOW)
		dev_info(bq2419x->dev,
			"In VSYSMIN regulation, battery is too low\n");

	/* Update Charging status based on STAT register */
	if (check_chg_state &&
	  ((val & BQ2419x_CHRG_STATE_MASK) == BQ2419x_CHRG_STATE_NOTCHARGING)) {
		bq2419x->chg_status = BATTERY_DISCHARGING;
		battery_charging_status_update(bq2419x->bc_dev,
				bq2419x->chg_status);
		battery_charging_restart(bq2419x->bc_dev,
					bq2419x->chg_restart_time);
		if (bq2419x->disable_suspend_during_charging)
			battery_charger_release_wake_lock(bq2419x->bc_dev);
		battery_charger_release_wake_lock(bq2419x->bc_dev);
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
	bq2419x->chg_rdev = devm_regulator_register(bq2419x->dev,
				&bq2419x->chg_reg_desc, &rconfig);
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
	bq2419x->vbus_reg_desc.enable_time = 8000;

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
	bq2419x->vbus_rdev = devm_regulator_register(bq2419x->dev,
				&bq2419x->vbus_reg_desc, &rconfig);
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
		goto scrub;
	}
	return ret;

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
			struct device_attribute *attr, char *buf)
{
	int i, ret = 0;

	for (i = 0; i <= 63; i++)
		ret += snprintf(buf + strlen(buf), MAX_STR_PRINT,
				"%d mA\n", fchg_reg_to_curr(i));

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

static int bq2419x_charger_thermal_configure(
		struct battery_charger_dev *bc_dev,
		int temp, bool enable_charger, bool enable_charg_half_current,
		int battery_voltage)
{
	struct bq2419x_chip *bq2419x = battery_charger_get_drvdata(bc_dev);
	int temperature;
	int battery_threshold_voltage;
	int in_current_limit;
	int ret;

	if (!bq2419x->cable_connected)
		return 0;

	temperature = temp;
	dev_info(bq2419x->dev, "Battery temp %d\n", temperature);
	if (enable_charger) {
		if (!enable_charg_half_current &&
			bq2419x->charging_state != ENABLED_FULL_IBAT) {
			bq2419x_full_current_enable(bq2419x);
			battery_charging_status_update(bq2419x->bc_dev,
				BATTERY_CHARGING);
		} else if (enable_charg_half_current &&
			bq2419x->charging_state != ENABLED_HALF_IBAT) {
			bq2419x_half_current_enable(bq2419x);
			/*Set charge voltage */
			battery_threshold_voltage =
					convert_to_reg(battery_voltage);
			ret = regmap_update_bits(bq2419x->regmap,
			BQ2419X_VOLT_CTRL_REG, BQ2419x_VOLTAGE_CTRL_MASK,
			battery_threshold_voltage << 2);
			if (ret < 0)
				return ret;
			battery_charging_status_update(bq2419x->bc_dev,
							BATTERY_CHARGING);
		}
	} else {
		if (bq2419x->charging_state != DISABLED) {
			in_current_limit = 500;
			ret = bq2419x_configure_charging_current(bq2419x,
				in_current_limit);
			if (ret < 0)
				return ret;
			bq2419x->charging_state = DISABLED;
			battery_charging_status_update(bq2419x->bc_dev,
				BATTERY_DISCHARGING);
		}
	}
	return 0;
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
		dev_err(bq2419x->dev,
			"Restarting of charging failed: %d\n", ret);
		battery_charging_restart(bq2419x->bc_dev,
				bq2419x->chg_restart_time);
	}
	return ret;
}


static struct battery_charging_ops bq2419x_charger_bci_ops = {
	.get_charging_status = bq2419x_charger_get_status,
	.restart_charging = bq2419x_charging_restart,
	.thermal_configure = bq2419x_charger_thermal_configure,
};

static struct battery_charger_info bq2419x_charger_bci = {
	.cell_id = 0,
	.bc_ops = &bq2419x_charger_bci_ops,
};

static struct bq2419x_platform_data *bq2419x_dt_parse(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct bq2419x_platform_data *pdata;
	struct device_node *batt_reg_node;
	struct device_node *vbus_reg_node;
	int ret;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	batt_reg_node = of_find_node_by_name(np, "charger");
	if (batt_reg_node) {
		int wdt_timeout;
		int chg_restart_time;
		int disable_suspend_during_charging;
		int temp_polling_time;
		bool enable_thermal_monitor;
		struct regulator_init_data *batt_init_data;
		const char *status_str;

		status_str = of_get_property(batt_reg_node, "status", NULL);
		if (status_str && !(!strcmp(status_str, "okay"))) {
			dev_info(&client->dev,
				"charger node status is disabled\n");
			goto  vbus_node;
		}

		pdata->bcharger_pdata = devm_kzalloc(&client->dev,
				sizeof(*(pdata->bcharger_pdata)), GFP_KERNEL);
		if (!pdata->bcharger_pdata)
			return ERR_PTR(-ENOMEM);

		batt_init_data = of_get_regulator_init_data(&client->dev,
								batt_reg_node);
		if (!batt_init_data)
			return ERR_PTR(-EINVAL);

		disable_suspend_during_charging =
				of_property_read_bool(batt_reg_node,
				"ti,disbale-suspend-during-charging");

		pdata->bcharger_pdata->disable_suspend_during_charging
					= disable_suspend_during_charging;

		ret = of_property_read_u32(batt_reg_node,
				"watchdog-timeout", &wdt_timeout);
		if (!ret)
			pdata->bcharger_pdata->wdt_timeout = wdt_timeout;

		ret = of_property_read_u32(batt_reg_node,
				"auto-recharge-time", &chg_restart_time);
		if (!ret)
			pdata->bcharger_pdata->chg_restart_time =
							chg_restart_time;

		enable_thermal_monitor = of_property_read_bool(batt_reg_node,
				"ti,enable-thermal-monitor");
		pdata->bcharger_pdata->enable_thermal_monitor =
						enable_thermal_monitor;

		ret = of_property_read_u32(batt_reg_node,
			"ti,temp-polling-time-sec", &temp_polling_time);
		if (!ret)
			pdata->bcharger_pdata->temp_polling_time_sec =
							temp_polling_time;

		pdata->bcharger_pdata->consumer_supplies =
					batt_init_data->consumer_supplies;
		pdata->bcharger_pdata->num_consumer_supplies =
					batt_init_data->num_consumer_supplies;
		pdata->bcharger_pdata->max_charge_current_mA =
					batt_init_data->constraints.max_uA;
	}

vbus_node:
	vbus_reg_node = of_find_node_by_name(np, "vbus");
	if (vbus_reg_node) {
		struct regulator_init_data *vbus_init_data;

		pdata->vbus_pdata = devm_kzalloc(&client->dev,
			sizeof(*(pdata->vbus_pdata)), GFP_KERNEL);
		if (!pdata->vbus_pdata)
			return ERR_PTR(-ENOMEM);

		vbus_init_data = of_get_regulator_init_data(
					&client->dev, vbus_reg_node);
		if (!vbus_init_data)
			return ERR_PTR(-EINVAL);

		pdata->vbus_pdata->consumer_supplies =
				vbus_init_data->consumer_supplies;
		pdata->vbus_pdata->num_consumer_supplies =
				vbus_init_data->num_consumer_supplies;
		pdata->vbus_pdata->gpio_otg_iusb =
				of_get_named_gpio(vbus_reg_node,
					"otg-iusb-gpio", 0);
	}

	return pdata;
}

static int bq2419x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bq2419x_chip *bq2419x;
	struct bq2419x_platform_data *pdata = NULL;
	int ret = 0;

	if (client->dev.platform_data)
		pdata = client->dev.platform_data;

	if (!pdata && client->dev.of_node) {
		pdata = bq2419x_dt_parse(client);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			dev_err(&client->dev, "Parsing of node failed, %d\n",
				ret);
			return ret;
		}
	}

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
	i2c_set_clientdata(client, bq2419x);
	bq2419x->irq = client->irq;

	ret = bq2419x_show_chip_version(bq2419x);
	if (ret < 0) {
		dev_err(&client->dev, "version read failed %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &bq2419x_attr_group);
	if (ret < 0) {
		dev_err(&client->dev, "sysfs create failed %d\n", ret);
		return ret;
	}

	mutex_init(&bq2419x->mutex);

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
			goto scrub_mutex;
		}
		goto skip_bcharger_init;
	}

	bq2419x->wdt_time_sec = pdata->bcharger_pdata->wdt_timeout;
	bq2419x->chg_restart_time = pdata->bcharger_pdata->chg_restart_time;
	bq2419x->battery_presense = true;
	bq2419x->disable_suspend_during_charging =
			pdata->bcharger_pdata->disable_suspend_during_charging;

	ret = bq2419x_charger_init(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
		goto scrub_mutex;
	}

	ret = bq2419x_init_charger_regulator(bq2419x, pdata);
	if (ret < 0) {
		dev_err(&client->dev,
			"Charger regualtor init failed %d\n", ret);
		goto scrub_mutex;
	}

	bq2419x_charger_bci.enable_thermal_monitor =
			pdata->bcharger_pdata->enable_thermal_monitor;
	bq2419x_charger_bci.polling_time_sec =
			pdata->bcharger_pdata->temp_polling_time_sec;
	bq2419x_charger_bci.tz_name = pdata->bcharger_pdata->tz_name;
	bq2419x->bc_dev = battery_charger_register(bq2419x->dev,
			&bq2419x_charger_bci, bq2419x);
	if (IS_ERR(bq2419x->bc_dev)) {
		ret = PTR_ERR(bq2419x->bc_dev);
		dev_err(bq2419x->dev, "battery charger register failed: %d\n",
			ret);
		goto scrub_mutex;
	}

	INIT_DELAYED_WORK(&bq2419x->wdt_restart_wq, bq2419x_wdt_restart_wq);
	ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec, "PROBE");
	if (ret < 0) {
		dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
		goto scrub_wq;
	}

skip_bcharger_init:
	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		goto scrub_wq;
	}

	ret = devm_request_threaded_irq(bq2419x->dev, bq2419x->irq, NULL,
		bq2419x_irq, IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			dev_name(bq2419x->dev), bq2419x);
	if (ret < 0) {
		dev_warn(bq2419x->dev, "request IRQ %d fail, err = %d\n",
				bq2419x->irq, ret);
		dev_info(bq2419x->dev,
			"Supporting bq driver without interrupt\n");
		ret = 0;
	}

	/* enable charging */
	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0)
		goto scrub_wq;

	return 0;
scrub_wq:
	if (pdata->bcharger_pdata) {
		cancel_delayed_work(&bq2419x->wdt_restart_wq);
		battery_charger_unregister(bq2419x->bc_dev);
	}
scrub_mutex:
	mutex_destroy(&bq2419x->mutex);
	return ret;
}

static int bq2419x_remove(struct i2c_client *client)
{
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);

	if (bq2419x->battery_presense) {
		battery_charger_unregister(bq2419x->bc_dev);
		cancel_delayed_work(&bq2419x->wdt_restart_wq);
	}
	mutex_destroy(&bq2419x->mutex);
	return 0;
}

static void bq2419x_shutdown(struct i2c_client *client)
{
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);
	int ret;

	if (!bq2419x->battery_presense)
		return;

	ret = bq2419x_reset_wdt(bq2419x, "SHUTDOWN");
	if (ret < 0)
		dev_err(bq2419x->dev, "Reset WDT failed: %d\n", ret);

	if (bq2419x->cable_connected && bq2419x->in_current_limit > 500 &&
			bq2419x->wdt_refresh_timeout) {
		ret = battery_charging_system_reset_after(bq2419x->bc_dev,
				bq2419x->wdt_refresh_timeout);
		if (ret < 0)
			dev_err(bq2419x->dev,
				"System reset after %d config failed %d\n",
				bq2419x->wdt_refresh_timeout, ret);
	}

	battery_charging_system_power_on_usb_event(bq2419x->bc_dev);
}

#ifdef CONFIG_PM_SLEEP
static int bq2419x_suspend(struct device *dev)
{
	struct bq2419x_chip *bq2419x = dev_get_drvdata(dev);
	int ret;

	if (!bq2419x->battery_presense)
		return 0;

	battery_charging_restart_cancel(bq2419x->bc_dev);

	ret = bq2419x_reset_wdt(bq2419x, "Suspend");
	if (ret < 0)
		dev_err(bq2419x->dev, "Reset WDT failed: %d\n", ret);

	battery_charging_wakeup(bq2419x->bc_dev,
				bq2419x->wdt_refresh_timeout);

	ret = bq2419x_set_charging_current_suspend(bq2419x, 500);
	if (ret < 0)
		dev_err(bq2419x->dev,
			"Configuration of charging failed: %d\n", ret);

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

	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		return ret;
	}

	if (val & BQ2419x_FAULT_WATCHDOG_FAULT) {
		bq_chg_err(bq2419x, "Watchdog Timer Expired\n");

		ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec,
						"RESUME");
		if (ret < 0) {
			dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
			return ret;
		}

		ret = bq2419x_charger_init(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
			return ret;
		}

		ret = bq2419x_set_charging_current(bq2419x->chg_rdev,
				bq2419x->last_charging_current,
				bq2419x->last_charging_current);
		if (ret < 0) {
			dev_err(bq2419x->dev,
				"Set charging current failed: %d\n", ret);
			return ret;
		}
	} else {
		ret = bq2419x_reset_wdt(bq2419x, "Resume");
		if (ret < 0)
			dev_err(bq2419x->dev, "Reset WDT failed: %d\n", ret);
	}

	return 0;
};
#endif

static const struct dev_pm_ops bq2419x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bq2419x_suspend, bq2419x_resume)
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
