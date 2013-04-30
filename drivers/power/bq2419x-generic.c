/*
 * bq2419x-generic.c -- BQ24190/BQ24192/BQ24192i/BQ24193 generic functions
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
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/platform_device.h>
#include <linux/mfd/palmas.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>

#define VBUS_REGULATOR_ENABLE_TIME	500000
#define NV_CHARGER_CURRENT_LIMIT	2000

/* input current limit */
static const unsigned int iinlim[] = {
	100, 150, 500, 900, 1200, 1500, 2000, 3000,
};

static const struct regmap_config bq2419x_regmap_config = {
	.reg_bits               = 8,
	.val_bits               = 8,
	.max_register           = BQ2419X_MAX_REGS,
};

static enum power_supply_property bq2419x_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

/* Kthread scheduling parameters */
struct sched_param bq2419x_param = {
	.sched_priority = MAX_RT_PRIO - 1,
};

int current_to_reg(const unsigned int *tbl,
			size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

int bq2419x_read(struct bq2419x_chip *bq2419x,
		unsigned int reg, unsigned int *val)
{
	if (bq2419x->use_regmap)
		return regmap_read(bq2419x->regmap, reg, val);
	else
		return palmas_read(bq2419x->palmas,
		PALMAS_CHARGER_BASE,
		reg, val);

}

int bq2419x_write(struct bq2419x_chip *bq2419x,
		unsigned int reg, unsigned int val)
{
	if (bq2419x->use_regmap)
		return regmap_write(bq2419x->regmap, reg, val);
	else
		return palmas_write(bq2419x->palmas,
			PALMAS_CHARGER_BASE,
			reg, val);
}

int bq2419x_update_bits(struct bq2419x_chip *bq2419x, unsigned int reg,
		unsigned int mask, unsigned int val)
{
	if (bq2419x->use_regmap)
		return regmap_update_bits(bq2419x->regmap, reg, mask, val);
	else
		return palmas_update_bits(bq2419x->palmas,
			PALMAS_CHARGER_BASE,
			reg, mask, val);
}

int bq2419x_charger_enable(struct bq2419x_chip *bq2419x)
{
	int ret;

	dev_info(bq2419x->dev, "Charging enabled\n");
	ret = bq2419x_update_bits(bq2419x, BQ2419X_PWR_ON_REG,
			BQ2419X_ENABLE_CHARGE_MASK, BQ2419X_ENABLE_CHARGE);
	if (ret < 0)
		dev_err(bq2419x->dev, "register update failed, err %d\n", ret);
	return ret;
}

int bq2419x_vbus_regulator_enable_time(struct regulator_dev *rdev)
{
	return VBUS_REGULATOR_ENABLE_TIME;
}

int bq2419x_vbus_enable(struct regulator_dev *rdev)
{
	struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
	int ret;

	dev_info(bq2419x->dev, "VBUS enabled, charging disabled\n");

	ret = bq2419x_update_bits(bq2419x, BQ2419X_PWR_ON_REG,
			BQ2419X_ENABLE_CHARGE_MASK, BQ2419X_ENABLE_VBUS);
	if (ret < 0)
		dev_err(bq2419x->dev, "PWR_ON_REG update failed %d", ret);

	return ret;
}

int bq2419x_vbus_disable(struct regulator_dev *rdev)
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

int bq2419x_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct bq2419x_chip *bq2419x = rdev_get_drvdata(rdev);
	int ret;
	unsigned int data;

	ret = bq2419x_read(bq2419x, BQ2419X_PWR_ON_REG, &data);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG read failed %d", ret);
		return ret;
	}
	return (data & BQ2419X_ENABLE_CHARGE_MASK) == BQ2419X_ENABLE_VBUS;
}

struct regulator_ops bq2419x_vbus_ops = {
	.enable         = bq2419x_vbus_enable,
	.disable        = bq2419x_vbus_disable,
	.is_enabled     = bq2419x_vbus_is_enabled,
	.enable_time    = bq2419x_vbus_regulator_enable_time,
};

int bq2419x_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq2419x_chip *bq2419x;

	bq2419x = container_of(psy, struct bq2419x_chip, ac);
	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = bq2419x->ac_online;
	else
		return -EINVAL;
	return 0;
}

int bq2419x_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq2419x_chip *bq2419x;

	bq2419x = container_of(psy, struct bq2419x_chip, usb);
	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = bq2419x->usb_online;
	else
		return -EINVAL;
	return 0;
}

static int bq2419x_set_usbsuspend(struct bq2419x_chip *bq2419x,
			unsigned int enable)
{
	struct palmas *palmas = bq2419x->palmas;
	int ret;
	int reg;

	reg = enable << 2;
	ret = palmas_update_bits(palmas,
			PALMAS_PMU_CONTROL_BASE,
			PALMAS_USB_CHGCTL1,
			PALMAS_USB_CHGCTL1_USB_SUSPEND, reg);

	if (ret < 0) {
		dev_err(bq2419x->dev, "Unable to update usb suspend\n");
		return ret;
	}

	return 0;
}

int bq2419x_init(struct bq2419x_chip *bq2419x)
{
	int val, ret = 0;

	/* Clear EN_HIZ */
	ret = bq2419x_update_bits(bq2419x,
			BQ2419X_INPUT_SRC_REG, BQ2419X_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev, "error reading reg: 0x%x\n",
			BQ2419X_INPUT_SRC_REG);
		return ret;
	}

	/* Configure input current limit */
	val = current_to_reg(iinlim, ARRAY_SIZE(iinlim),
				bq2419x->in_current_limit);
	if (val < 0)
		return 0;

	val &= ~(BQ2419x_INPUT_VOLTAGE_MASK);
	/* Configure inout voltage to 4.52 in case of NV
	*  NV charger.
	*/
	if (bq2419x->in_current_limit == 2000)
		val |= BQ2419x_NVCHARGER_INPUT_VOL_SEL;
	else
		val |= BQ2419x_DEFAULT_INPUT_VOL_SEL;

	ret = bq2419x_update_bits(bq2419x,
			BQ2419X_INPUT_SRC_REG, BQ2419x_CONFIG_MASK |
			BQ2419x_INPUT_VOLTAGE_MASK, val);
	if (ret < 0)
		dev_err(bq2419x->dev, "error reading reg: 0x%x\n",
			BQ2419X_INPUT_SRC_REG);

	bq2419x_set_usbsuspend(bq2419x, 0);

	return ret;
}

int bq2419x_charger_init(struct bq2419x_chip *bq2419x)
{
	int ret;

	/* Configure Output Current Control to 3A*/
	ret = bq2419x_write(bq2419x, BQ2419X_CHRG_CTRL_REG,
				BQ2419X_CHRG_CTRL_REG_3A);
	if (ret < 0) {
		dev_err(bq2419x->dev, "CHRG_CTRL_REG write failed %d\n", ret);
		return ret;
	}

	/*
	 * Configure Input voltage limit reset to OTP value,
	 * and charging current to 500mA.
	 */
	ret = bq2419x_write(bq2419x, BQ2419X_INPUT_SRC_REG,
				BQ2419x_OTP_CURRENT_500MA);
	if (ret < 0)
		dev_err(bq2419x->dev, "INPUT_SRC_REG write failed %d\n", ret);

	return ret;
}

int bq2419x_reset_wdt(struct bq2419x_chip *bq2419x, const char *from)
{
	int ret = 0;
	unsigned int reg01;

	mutex_lock(&bq2419x->mutex);
	if (bq2419x->suspended)
		goto scrub;

	dev_info(bq2419x->dev, "%s() from %s()\n", __func__, from);

	/* Clear EN_HIZ */
	ret = bq2419x_update_bits(bq2419x,
			BQ2419X_INPUT_SRC_REG, BQ2419X_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(bq2419x->dev, "INPUT_SRC_REG update failed:%d\n", ret);
		goto scrub;
	}

	ret = bq2419x_read(bq2419x, BQ2419X_PWR_ON_REG, &reg01);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG read failed: %d\n", ret);
		goto scrub;
	}

	reg01 |= BIT(6);

	/* Write two times to make sure reset WDT */
	ret = bq2419x_write(bq2419x, BQ2419X_PWR_ON_REG, reg01);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG write failed: %d\n", ret);
		goto scrub;
	}
	ret = bq2419x_write(bq2419x, BQ2419X_PWR_ON_REG, reg01);
	if (ret < 0) {
		dev_err(bq2419x->dev, "PWR_ON_REG write failed: %d\n", ret);
		goto scrub;
	}

scrub:
	mutex_unlock(&bq2419x->mutex);
	return ret;
}

int bq2419x_set_charging_current(struct regulator_dev *rdev,
			int min_uA, int max_uA)
{
	struct bq2419x_chip *bq_charger = rdev_get_drvdata(rdev);
	int ret = 0;
	int val;
	int status;

	dev_info(bq_charger->dev, "Setting charging current %d\n", max_uA/1000);
	/* System status register gets updated after a delay of about 200ms*/

	bq_charger->usb_online = 0;
	bq_charger->ac_online = 0;
	status = 0;
	msleep(200);

	ret = bq2419x_charger_enable(bq_charger);
	if (ret < 0) {
		dev_err(bq_charger->dev, "Charger enable failed %d", ret);
		return ret;
	}

	ret = bq2419x_read(bq_charger, BQ2419X_SYS_STAT_REG, &val);
	if (ret < 0) {
		dev_err(bq_charger->dev, "error reading reg: 0x%x\n",
				BQ2419X_SYS_STAT_REG);
	}

	if (max_uA == 0 && val != 0) {
		bq2419x_set_usbsuspend(bq_charger, 1);
		return ret;
	}

	bq_charger->in_current_limit = max_uA/1000;

	if ((val & BQ2419x_VBUS_STAT) == BQ2419x_VBUS_UNKNOWN) {
		bq_charger->usb_online = 0;
		bq_charger->in_current_limit = 500;
		ret = bq2419x_init(bq_charger);
		if (ret < 0)
			goto error;
		if (bq_charger->update_status)
			bq_charger->update_status
				(status, 0);
	} else if (bq_charger->in_current_limit == 500) {
		status = 1;
		bq_charger->usb_online = 1;
		ret = bq2419x_init(bq_charger);
		if (ret < 0)
			goto error;
		if (bq_charger->update_status)
			bq_charger->update_status
				(status, 2);
	} else {
		status = 1;
		bq_charger->ac_online = 1;
		ret = bq2419x_init(bq_charger);
		if (ret < 0)
			goto error;
		if (bq_charger->update_status)
			bq_charger->update_status
				(status, 1);
	}
	if (ret == 0) {
		if (bq_charger->use_mains)
			power_supply_changed(&bq_charger->ac);
		if (bq_charger->use_usb)
			power_supply_changed(&bq_charger->usb);
	}
	return 0;
error:
	dev_err(bq_charger->dev, "Charger enable failed, err = %d\n", ret);
	return ret;
}

struct regulator_ops bq2419x_tegra_regulator_ops = {
	.set_current_limit = bq2419x_set_charging_current,
};

int bq2419x_fault_clear_sts(struct bq2419x_chip *bq2419x)
{
	int ret;
	unsigned int reg09;

	ret = bq2419x_read(bq2419x, BQ2419X_FAULT_REG, &reg09);
	if (ret < 0)
		dev_err(bq2419x->dev, "FAULT_REG read failed: %d\n", ret);

	return ret;
}

int bq2419x_watchdog_init(struct bq2419x_chip *bq2419x,
			int timeout, const char *from)
{
	int ret, val;
	unsigned int reg05;

	if (!timeout) {
		ret = bq2419x_update_bits(bq2419x,
				BQ2419X_TIME_CTRL_REG,
				BQ2419X_WD_MASK, 0);
		if (ret < 0)
			dev_err(bq2419x->dev,
				"TIME_CTRL_REG read failed: %d\n", ret);
		return ret;
	}

	/*
	 * Choose a kernel wdt refresh thread timeout value below the
	 * watchdog expiry timeout
	 */
	if (timeout <= 60) {
		val = BQ2419X_WD_40ms;
		bq2419x->wdt_refresh_timeout = 15;

	} else if (timeout <= 120) {
		val = BQ2419X_WD_80ms;
		bq2419x->wdt_refresh_timeout = 40;
	} else {
		val = BQ2419X_WD_160ms;
		bq2419x->wdt_refresh_timeout = 105;
	}

	ret = bq2419x_read(bq2419x, BQ2419X_TIME_CTRL_REG, &reg05);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"TIME_CTRL_REG read failed:%d\n", ret);
		return ret;
	}

	if ((reg05 & BQ2419X_WD_MASK) != val) {
		ret = bq2419x_update_bits(bq2419x,
				BQ2419X_TIME_CTRL_REG,
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

void bq2419x_work_thread(struct kthread_work *work)
{
	struct bq2419x_chip *bq2419x = container_of(work,
			struct bq2419x_chip, bq_wdt_work);
	int ret;

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

irqreturn_t bq2419x_irq(int irq, void *data)
{
	struct bq2419x_chip *bq2419x = data;
	irqreturn_t ret;
	unsigned int val;

	ret = bq2419x_read(bq2419x, BQ2419X_FAULT_REG, &val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "FAULT_REG read failed %d\n", ret);
		return ret;
	}

	dev_info(bq2419x->dev, "%s() Irq %d status 0x%02x\n",
		__func__, irq, val);

	if (val & BQ2419x_FAULT_WATCHDOG_FAULT) {
		dev_err(bq2419x->dev,
			"Charging Fault: Watchdog Timer Expired\n");
		ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec,
						"ISR");
		if (ret < 0) {
			dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
			return IRQ_NONE;
		}

		ret = bq2419x_charger_init(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
			return IRQ_NONE;
		}

		ret = bq2419x_init(bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "bq2419x init failed: %d\n", ret);
			return IRQ_NONE;
		}
	}

	if (val & BQ2419x_FAULT_BOOST_FAULT)
		dev_err(bq2419x->dev, "Charging Fault: VBUS Overloaded\n");

	switch (val & BQ2419x_FAULT_CHRG_FAULT_MASK) {
	case BQ2419x_FAULT_CHRG_INPUT:
		dev_err(bq2419x->dev, "Charging Fault: "
				"Input Fault (VBUS OVP or VBAT<VBUS<3.8V)\n");
		break;
	case BQ2419x_FAULT_CHRG_THERMAL:
		dev_err(bq2419x->dev, "Charging Fault: Thermal shutdown\n");
		break;
	case BQ2419x_FAULT_CHRG_SAFTY:
		dev_err(bq2419x->dev,
			"Charging Fault: Safety timer expiration\n");
		bq2419x->chg_restart_timeout = bq2419x->chg_restart_time /
						bq2419x->wdt_refresh_timeout;
		break;
	default:
		break;
	}

	if (val & BQ2419x_FAULT_NTC_FAULT)
		dev_err(bq2419x->dev, "Charging Fault: NTC fault %d\n",
				val & BQ2419x_FAULT_NTC_FAULT);

	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		return IRQ_NONE;
	}

	ret = bq2419x_read(bq2419x, BQ2419X_SYS_STAT_REG, &val);
	if (ret < 0) {
		dev_err(bq2419x->dev, "SYS_STAT_REG read failed %d\n", ret);
		return IRQ_NONE;
	}

	if ((val & BQ2419x_CHRG_STATE_MASK) == BQ2419x_CHRG_STATE_CHARGE_DONE)
		dev_info(bq2419x->dev, "Charging completed\n");

	return IRQ_HANDLED;
}

int bq2419x_init_charger_regulator(struct bq2419x_chip *bq2419x,
		struct bq2419x_platform_data *pdata)
{
	int ret = 0;

	if (!pdata->bcharger_pdata) {
		dev_err(bq2419x->dev, "No charger platform data\n");
		return 0;
	}

	bq2419x->chg_reg_desc.name  = "bq2419x-charger";
	bq2419x->chg_reg_desc.ops   = &bq2419x_tegra_regulator_ops;
	bq2419x->chg_reg_desc.type  = REGULATOR_CURRENT;
	bq2419x->chg_reg_desc.owner = THIS_MODULE;

	bq2419x->chg_reg_init_data.supply_regulator     = NULL;
	bq2419x->chg_reg_init_data.regulator_init       = NULL;
	bq2419x->chg_reg_init_data.num_consumer_supplies =
				pdata->bcharger_pdata->num_consumer_supplies;
	bq2419x->chg_reg_init_data.consumer_supplies    =
				pdata->bcharger_pdata->consumer_supplies;
	bq2419x->chg_reg_init_data.driver_data          = bq2419x;
	bq2419x->chg_reg_init_data.constraints.name     = "bq2419x-charger";
	bq2419x->chg_reg_init_data.constraints.min_uA   = 0;
	bq2419x->chg_reg_init_data.constraints.max_uA   =
			pdata->bcharger_pdata->max_charge_current_mA * 1000;

	bq2419x->chg_reg_init_data.constraints.valid_modes_mask =
						REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_STANDBY;

	bq2419x->chg_reg_init_data.constraints.valid_ops_mask =
						REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_CURRENT;

	bq2419x->chg_rdev = regulator_register(&bq2419x->chg_reg_desc,
				bq2419x->dev, &bq2419x->chg_reg_init_data,
				bq2419x, NULL);
	if (IS_ERR(bq2419x->chg_rdev)) {
		ret = PTR_ERR(bq2419x->chg_rdev);
		dev_err(bq2419x->dev,
			"vbus-charger regulator register failed %d\n", ret);
	}
	return ret;
}

int bq2419x_init_vbus_regulator(struct bq2419x_chip *bq2419x,
		struct bq2419x_platform_data *pdata)
{
	int ret = 0;

	if (!pdata->vbus_pdata) {
		dev_err(bq2419x->dev, "No vbus platform data\n");
		return 0;
	}

	bq2419x->gpio_otg_iusb = pdata->vbus_pdata->gpio_otg_iusb;
	bq2419x->vbus_reg_desc.name = "bq2419x-vbus";
	bq2419x->vbus_reg_desc.ops = &bq2419x_vbus_ops;
	bq2419x->vbus_reg_desc.type = REGULATOR_VOLTAGE;
	bq2419x->vbus_reg_desc.owner = THIS_MODULE;

	bq2419x->vbus_reg_init_data.supply_regulator    = NULL;
	bq2419x->vbus_reg_init_data.regulator_init      = NULL;
	bq2419x->vbus_reg_init_data.num_consumer_supplies       =
				pdata->vbus_pdata->num_consumer_supplies;
	bq2419x->vbus_reg_init_data.consumer_supplies   =
				pdata->vbus_pdata->consumer_supplies;
	bq2419x->vbus_reg_init_data.driver_data         = bq2419x;

	bq2419x->vbus_reg_init_data.constraints.name    = "bq2419x-vbus";
	bq2419x->vbus_reg_init_data.constraints.min_uV  = 0;
	bq2419x->vbus_reg_init_data.constraints.max_uV  = 5000000,
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
	bq2419x->vbus_rdev = regulator_register(&bq2419x->vbus_reg_desc,
			bq2419x->dev, &bq2419x->vbus_reg_init_data,
			bq2419x, NULL);
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

int bq2419x_psy_init(struct bq2419x_chip *bq2419x)
{
	int ret = 0;

	bq2419x->ac_online = 0;
	bq2419x->usb_online = 0;
	if (bq2419x->use_mains) {
		bq2419x->ac.name                = "bq2419x-ac";
		bq2419x->ac.type                = POWER_SUPPLY_TYPE_MAINS;
		bq2419x->ac.get_property        = bq2419x_ac_get_property;
		bq2419x->ac.properties          = bq2419x_psy_props;
		bq2419x->ac.num_properties      = ARRAY_SIZE(bq2419x_psy_props);
		ret = power_supply_register(bq2419x->dev, &bq2419x->ac);
		if (ret < 0) {
			dev_err(bq2419x->dev,
				"AC power supply register failed %d\n", ret);
			return ret;
		}
	}

	if (bq2419x->use_usb) {
		bq2419x->usb.name               = "bq2419x-usb";
		bq2419x->usb.type               = POWER_SUPPLY_TYPE_USB;
		bq2419x->usb.get_property       = bq2419x_usb_get_property;
		bq2419x->usb.properties         = bq2419x_psy_props;
		bq2419x->usb.num_properties     = ARRAY_SIZE(bq2419x_psy_props);
		ret = power_supply_register(bq2419x->dev, &bq2419x->usb);
		if (ret) {
			dev_err(bq2419x->dev,
				"usb power supply register failed %d\n", ret);
			goto scrub;
		}
	}
	return ret;
scrub:
	if (bq2419x->use_mains)
		power_supply_unregister(&bq2419x->ac);
	return ret;
}

int bq2419x_show_chip_version(struct bq2419x_chip *bq2419x)
{
	int ret;
	unsigned int val;

	ret = bq2419x_read(bq2419x, BQ2419X_REVISION_REG, &val);
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

int bq2419x_wakealarm(struct bq2419x_chip *bq2419x, int time_sec)
{
	int ret;
	unsigned long now;
	struct rtc_wkalrm alm;
	int alarm_time = time_sec;

	alm.enabled = true;
	ret = rtc_read_time(bq2419x->rtc, &alm.time);
	if (ret < 0) {
		dev_err(bq2419x->dev, "RTC read time failed %d\n", ret);
		return ret;
	}
	rtc_tm_to_time(&alm.time, &now);

	if (!alarm_time)
		alarm_time = 3600;
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

int bq2419x_hw_init(struct bq2419x_chip *bq2419x,
		struct bq2419x_platform_data *pdata)
{
	int ret = 0;

	bq2419x->use_usb = pdata->bcharger_pdata->use_usb;
	bq2419x->use_mains =  pdata->bcharger_pdata->use_mains;
	bq2419x->update_status =  pdata->bcharger_pdata->update_status;
	bq2419x->rtc_alarm_time =  pdata->bcharger_pdata->rtc_alarm_time;
	bq2419x->wdt_time_sec = pdata->bcharger_pdata->wdt_timeout;
	bq2419x->chg_restart_time = pdata->bcharger_pdata->chg_restart_time;
	bq2419x->rtc = alarmtimer_get_rtcdev();
	mutex_init(&bq2419x->mutex);
	bq2419x->suspended = 0;
	bq2419x->chg_restart_timeout = 0;

	ret = bq2419x_show_chip_version(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "version read failed %d\n", ret);
		return ret;
	}

	ret = bq2419x_charger_init(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "Charger init failed: %d\n", ret);
		return ret;
	}

	ret = bq2419x_init_charger_regulator(bq2419x, pdata);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"Charger regualtor init failed %d\n", ret);
		return ret;
	}

	ret = bq2419x_psy_init(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"Charger power supply init failed %d\n", ret);
				goto scrub_chg_reg;
	}

	ret = bq2419x_init_vbus_regulator(bq2419x, pdata);
	if (ret < 0) {
		dev_err(bq2419x->dev,
			"VBUS regualtor init failed %d\n", ret);
		goto scrub_psy;
	}

	init_kthread_worker(&bq2419x->bq_kworker);
	bq2419x->bq_kworker_task = kthread_run(kthread_worker_fn,
			&bq2419x->bq_kworker,
			dev_name(bq2419x->dev));
	if (IS_ERR(bq2419x->bq_kworker_task)) {
		ret = PTR_ERR(bq2419x->bq_kworker_task);
		dev_err(bq2419x->dev, "Kworker task creation failed %d\n", ret);
		goto scrub_vbus_reg;
	}

	init_kthread_work(&bq2419x->bq_wdt_work, bq2419x_work_thread);
	sched_setscheduler(bq2419x->bq_kworker_task,
			SCHED_FIFO, &bq2419x_param);
	queue_kthread_work(&bq2419x->bq_kworker, &bq2419x->bq_wdt_work);

	ret = bq2419x_watchdog_init(bq2419x, bq2419x->wdt_time_sec, "PROBE");
	if (ret < 0) {
		dev_err(bq2419x->dev, "BQWDT init failed %d\n", ret);
		return ret;
	}

	ret = bq2419x_fault_clear_sts(bq2419x);
	if (ret < 0) {
		dev_err(bq2419x->dev, "fault clear status failed %d\n", ret);
		return ret;
	}

	if (bq2419x->irq < 0)
		ret = palmas_update_bits(bq2419x->palmas,
				PALMAS_INTERRUPT_BASE,
				PALMAS_INT6_MASK, PALMAS_INT6_MASK_CHARGER,
				PALMAS_INT6_MASK_CHARGER);
	else {
		ret = request_threaded_irq(bq2419x->irq, NULL,
			bq2419x_irq, IRQF_TRIGGER_FALLING,
				dev_name(bq2419x->dev), bq2419x);
		if (ret < 0) {
			dev_err(bq2419x->dev, "request IRQ %d fail, err = %d\n",
					bq2419x->irq, ret);
			goto scrub_kthread;
		}
	}

	/* enable charging */
	ret = bq2419x_charger_enable(bq2419x);
	if (ret < 0)
		goto scrub_irq;

	return 0;
scrub_irq:
	free_irq(bq2419x->irq, bq2419x);
scrub_kthread:
	bq2419x->stop_thread = true;
	flush_kthread_worker(&bq2419x->bq_kworker);
	kthread_stop(bq2419x->bq_kworker_task);
scrub_vbus_reg:
	regulator_unregister(bq2419x->vbus_rdev);
scrub_psy:
	if (bq2419x->use_usb)
		power_supply_unregister(&bq2419x->usb);
	if (bq2419x->use_mains)
		power_supply_unregister(&bq2419x->ac);
scrub_chg_reg:
	regulator_unregister(bq2419x->chg_rdev);
	mutex_destroy(&bq2419x->mutex);
	return ret;
}

int bq2419x_resource_cleanup(struct bq2419x_chip *bq2419x)
{
	free_irq(bq2419x->irq, bq2419x);
	bq2419x->stop_thread = true;
	flush_kthread_worker(&bq2419x->bq_kworker);
	kthread_stop(bq2419x->bq_kworker_task);
	regulator_unregister(bq2419x->vbus_rdev);
	if (bq2419x->use_usb)
		power_supply_unregister(&bq2419x->usb);
	if (bq2419x->use_mains)
		power_supply_unregister(&bq2419x->ac);
	regulator_unregister(bq2419x->chg_rdev);
	mutex_destroy(&bq2419x->mutex);
	return 0;
}

