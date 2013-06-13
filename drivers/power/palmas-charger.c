/*
 * palmas-charger.c -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Darbha Sriharsha <dsriharsha@nvidia.com>
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
#include <linux/platform_device.h>
#include <linux/mfd/palmas.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/power/battery-charger-gauge-comm.h>

#define VBUS_REGULATOR_ENABLE_TIME	500000
#define NV_CHARGER_CURRENT_LIMIT	2000

/* input current limit */
static const unsigned int iinlim[] = {
	100, 150, 500, 900, 1200, 1500, 2000, 3000,
};

/* Kthread scheduling parameters */
struct sched_param palmas_param = {
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

int palmas_charger_reg_read(struct palmas_charger_chip *palmas_chip,
		unsigned int reg, unsigned int *val)
{
	if (palmas_chip->use_regmap)
		return regmap_read(palmas_chip->regmap, reg, val);
	else
		return palmas_read(palmas_chip->palmas,
		PALMAS_CHARGER_BASE,
		reg, val);

}

int palmas_charger_reg_write(struct palmas_charger_chip *palmas_chip,
unsigned int reg, unsigned int val)
{
	if (palmas_chip->use_regmap)
		return regmap_write(palmas_chip->regmap, reg, val);
	else
		return palmas_write(palmas_chip->palmas,
			PALMAS_CHARGER_BASE,
			reg, val);
}

int palmas_charger_update_bits(struct palmas_charger_chip *palmas_chip,
		unsigned int reg, unsigned int mask, unsigned int val)
{
	if (palmas_chip->use_regmap)
		return regmap_update_bits(palmas_chip->regmap, reg, mask, val);
	else
		return palmas_update_bits(palmas_chip->palmas,
			PALMAS_CHARGER_BASE,
			reg, mask, val);
}

int palmas_charger_enable(struct palmas_charger_chip *palmas_chip)
{
	int ret;

	dev_info(palmas_chip->dev, "Charging enabled\n");
	ret = palmas_charger_update_bits(palmas_chip, PALMAS_CHARGER_REG01,
			PALMAS_ENABLE_CHARGE_MASK, PALMAS_ENABLE_CHARGE);
	if (ret < 0)
		dev_err(palmas_chip->dev,
			"register update failed, err %d\n", ret);
	return ret;
}

int palmas_vbus_regulator_enable_time(struct regulator_dev *rdev)
{
	return VBUS_REGULATOR_ENABLE_TIME;
}

int palmas_vbus_enable(struct regulator_dev *rdev)
{
	struct palmas_charger_chip *palmas_chip = rdev_get_drvdata(rdev);
	int ret;

	dev_info(palmas_chip->dev, "VBUS enabled, charging disabled\n");


	ret = palmas_charger_update_bits(palmas_chip, PALMAS_CHARGER_REG01,
			PALMAS_ENABLE_CHARGE_MASK, PALMAS_ENABLE_VBUS);
	if (ret < 0)
		dev_err(palmas_chip->dev, "PWR_ON_REG update failed %d", ret);

	return ret;
}

int palmas_vbus_disable(struct regulator_dev *rdev)
{
	struct palmas_charger_chip *palmas_chip = rdev_get_drvdata(rdev);
	int ret;

	dev_info(palmas_chip->dev, "VBUS disabled, charging enabled\n");
	ret = palmas_charger_enable(palmas_chip);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "Charger enable failed %d", ret);
		return ret;
	}

	return ret;
}

int palmas_vbus_is_enabled(struct regulator_dev *rdev)
{
	struct palmas_charger_chip *palmas_chip = rdev_get_drvdata(rdev);
	int ret;
	unsigned int data;

	ret = palmas_charger_reg_read(palmas_chip, PALMAS_CHARGER_REG01, &data);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "PWR_ON_REG read failed %d", ret);
		return ret;
	}
	return (data & PALMAS_ENABLE_CHARGE_MASK) == PALMAS_ENABLE_VBUS;
}

struct regulator_ops palmas_vbus_ops = {
	.enable         = palmas_vbus_enable,
	.disable        = palmas_vbus_disable,
	.is_enabled     = palmas_vbus_is_enabled,
	.enable_time    = palmas_vbus_regulator_enable_time,
};

static int palmas_set_usbsuspend(struct palmas_charger_chip *palmas_chip,
			unsigned int enable)
{
	struct palmas *palmas = palmas_chip->palmas;
	int ret;
	int reg;

	reg = enable << 2;

	ret = palmas_update_bits(palmas,
			PALMAS_PMU_CONTROL_BASE,
			PALMAS_USB_CHGCTL1,
			PALMAS_USB_CHGCTL1_USB_SUSPEND, reg);

	if (ret < 0) {
		dev_err(palmas_chip->dev, "Unable to update usb suspend\n");
		return ret;
	}

	return 0;
}

int palmas_init(struct palmas_charger_chip *palmas_chip)
{
	int val, ret = 0;

	/* Clear EN_HIZ */
	ret = palmas_charger_update_bits(palmas_chip,
			PALMAS_CHARGER_REG00, PALMAS_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "error reading reg: 0x%x\n",
			PALMAS_CHARGER_REG00);
		return ret;
	}

	/* Configure input current limit */
	val = current_to_reg(iinlim, ARRAY_SIZE(iinlim),
				palmas_chip->in_current_limit);
	if (val < 0)
		return 0;

	val &= ~(PALMAS_INPUT_VOLTAGE_MASK);
	/* Configure inout voltage to 4.52 in case of NV
	*  NV charger.
	*/
	if (palmas_chip->in_current_limit == 2000)
		val |= PALMAS_NVCHARGER_INPUT_VOL_SEL;
	else
		val |= PALMAS_DEFAULT_INPUT_VOL_SEL;

	ret = palmas_charger_update_bits(palmas_chip,
			PALMAS_CHARGER_REG00, PALMAS_CONFIG_MASK |
			PALMAS_INPUT_VOLTAGE_MASK, val);
	if (ret < 0)
		dev_err(palmas_chip->dev, "error reading reg: 0x%x\n",
			PALMAS_CHARGER_REG00);

	palmas_set_usbsuspend(palmas_chip, 0);

	return ret;
}

int palmas_charger_init(struct palmas_charger_chip *palmas_chip)
{
	int ret;

	/* Configure Output Current Control to 3A*/
	ret = palmas_charger_reg_write(palmas_chip, PALMAS_CHARGER_REG02,
				PALMAS_CHRG_CTRL_REG_3A);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"CHRG_CTRL_REG write failed %d\n", ret);
		return ret;
	}

	/*
	 * Configure Input voltage limit reset to OTP value,
	 * and charging current to 500mA.
	 */
	ret = palmas_charger_reg_write(palmas_chip, PALMAS_CHARGER_REG00,
				PALMAS_OTP_CURRENT_500MA);
	if (ret < 0)
		dev_err(palmas_chip->dev,
			"INPUT_SRC_REG write failed %d\n", ret);

	ret = palmas_update_bits(palmas_chip->palmas,
			PALMAS_PMU_CONTROL_BASE,
			PALMAS_USB_CHGCTL2, PALMAS_USB_CHGCTL2_BOOST_EN,
			PALMAS_USB_CHGCTL2_BOOST_EN);

	if (ret < 0)
		dev_err(palmas_chip->dev, "BOOST mode enable failed\n");

	return ret;
}

int palmas_reset_wdt(struct palmas_charger_chip *palmas_chip, const char *from)
{
	int ret = 0;
	unsigned int reg01;

	mutex_lock(&palmas_chip->mutex);
	if (palmas_chip->suspended)
		goto scrub;

	dev_info(palmas_chip->dev, "%s() from %s()\n", __func__, from);

	/* Clear EN_HIZ */
	ret = palmas_charger_update_bits(palmas_chip,
			PALMAS_CHARGER_REG00, PALMAS_EN_HIZ, 0);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"INPUT_SRC_REG update failed:%d\n", ret);
		goto scrub;
	}

	ret = palmas_charger_reg_read(palmas_chip,
				PALMAS_CHARGER_REG01, &reg01);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "PWR_ON_REG read failed: %d\n", ret);
		goto scrub;
	}

	reg01 |= BIT(6);

	/* Write two times to make sure reset WDT */
	ret = palmas_charger_reg_write(palmas_chip,
				PALMAS_CHARGER_REG01, reg01);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"PWR_ON_REG write failed: %d\n", ret);
		goto scrub;
	}
	ret = palmas_charger_reg_write(palmas_chip, PALMAS_CHARGER_REG01, reg01);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "PWR_ON_REG write failed: %d\n", ret);
		goto scrub;
	}

scrub:
	mutex_unlock(&palmas_chip->mutex);
	return ret;
}

int palmas_set_charging_current(struct regulator_dev *rdev,
			int min_uA, int max_uA)
{
	struct palmas_charger_chip *bq_charger = rdev_get_drvdata(rdev);
	int ret = 0;
	int val;

	dev_info(bq_charger->dev, "Setting charging current %d\n", max_uA/1000);
	/* System status register gets updated after a delay of about 200ms*/

	bq_charger->chg_status = BATTERY_DISCHARGING;
	msleep(200);

	ret = palmas_charger_enable(bq_charger);
	if (ret < 0) {
		dev_err(bq_charger->dev, "Charger enable failed %d", ret);
		return ret;
	}

	ret = palmas_charger_reg_read(bq_charger, PALMAS_CHARGER_REG08, &val);
	if (ret < 0) {
		dev_err(bq_charger->dev, "error reading reg: 0x%x\n",
				PALMAS_CHARGER_REG08);
	}

	if (max_uA == 0 && val != 0) {
		palmas_set_usbsuspend(bq_charger, 1);
		return ret;
	}

	bq_charger->in_current_limit = max_uA/1000;

	if ((val & PALMAS_VBUS_STAT) == PALMAS_VBUS_UNKNOWN) {
		bq_charger->in_current_limit = 500;
		ret = palmas_init(bq_charger);
		if (ret < 0)
			goto error;
		battery_charging_status_update(bq_charger->bc_dev,
					BATTERY_DISCHARGING);
	} else {
		bq_charger->chg_status = BATTERY_CHARGING;
		ret = palmas_init(bq_charger);
		if (ret < 0)
			goto error;
		battery_charging_status_update(bq_charger->bc_dev,
				BATTERY_CHARGING);
	}
	return 0;
error:
	dev_err(bq_charger->dev, "Charger enable failed, err = %d\n", ret);
	return ret;
}

struct regulator_ops palmas_tegra_regulator_ops = {
	.set_current_limit = palmas_set_charging_current,
};

int palmas_fault_clear_sts(struct palmas_charger_chip *palmas_chip)
{
	int ret;
	unsigned int reg09;

	ret = palmas_charger_reg_read(palmas_chip,
				PALMAS_CHARGER_REG09, &reg09);
	if (ret < 0)
		dev_err(palmas_chip->dev, "FAULT_REG read failed: %d\n", ret);

	return ret;
}

int palmas_watchdog_init(struct palmas_charger_chip *palmas_chip,
			int timeout, const char *from)
{
	int ret, val;
	unsigned int reg05;

	if (!timeout) {
		ret = palmas_charger_update_bits(palmas_chip,
				PALMAS_CHARGER_REG05,
				PALMAS_WD_MASK, 0);
		if (ret < 0)
			dev_err(palmas_chip->dev,
				"TIME_CTRL_REG read failed: %d\n", ret);
		return ret;
	}

	/*
	 * Choose a kernel wdt refresh thread timeout value below the
	 * watchdog expiry timeout
	 */
	if (timeout <= 60) {
		val = PALMAS_WD_40ms;
		palmas_chip->wdt_refresh_timeout = 15;

	} else if (timeout <= 120) {
		val = PALMAS_WD_80ms;
		palmas_chip->wdt_refresh_timeout = 40;
	} else {
		val = PALMAS_WD_160ms;
		palmas_chip->wdt_refresh_timeout = 105;
	}

	ret = palmas_charger_reg_read(palmas_chip,
			PALMAS_CHARGER_REG05, &reg05);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"TIME_CTRL_REG read failed:%d\n", ret);
		return ret;
	}

	if ((reg05 & PALMAS_WD_MASK) != val) {
		ret = palmas_charger_update_bits(palmas_chip,
				PALMAS_CHARGER_REG05,
				PALMAS_WD_MASK, val);
		if (ret < 0) {
			dev_err(palmas_chip->dev,
				"TIME_CTRL_REG read failed: %d\n", ret);
			return ret;
		}
	}

	ret = palmas_reset_wdt(palmas_chip, from);
	if (ret < 0)
		dev_err(palmas_chip->dev, "palmas_reset_wdt failed: %d\n", ret);

	return ret;
}

void palmas_work_thread(struct kthread_work *work)
{
	struct palmas_charger_chip *palmas_chip = container_of(work,
			struct palmas_charger_chip, bq_wdt_work);
	int ret;

	for (;;) {
		if (palmas_chip->stop_thread)
			return;

		if (palmas_chip->chg_restart_timeout) {
			mutex_lock(&palmas_chip->mutex);
			palmas_chip->chg_restart_timeout--;
			if (!palmas_chip->chg_restart_timeout) {
				ret = palmas_charger_enable(palmas_chip);
				if (ret < 0)
					dev_err(palmas_chip->dev,
					"Charger enable failed %d", ret);
			}
			if (palmas_chip->suspended)
				palmas_chip->chg_restart_timeout = 0;

			mutex_unlock(&palmas_chip->mutex);
		}

		ret = palmas_reset_wdt(palmas_chip, "THREAD");
		if (ret < 0)
			dev_err(palmas_chip->dev,
				"palmas_reset_wdt failed: %d\n", ret);

		msleep(palmas_chip->wdt_refresh_timeout * 1000);
	}
}

irqreturn_t palmas_irq(int irq, void *data)
{
	struct palmas_charger_chip *palmas_chip = data;
	irqreturn_t ret;
	unsigned int val;

	ret = palmas_charger_reg_read(palmas_chip, PALMAS_CHARGER_REG09, &val);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "FAULT_REG read failed %d\n", ret);
		return ret;
	}

	dev_info(palmas_chip->dev, "%s() Irq %d status 0x%02x\n",
		__func__, irq, val);

	if (val & PALMAS_FAULT_WATCHDOG_FAULT) {
		dev_err(palmas_chip->dev,
			"Charging Fault: Watchdog Timer Expired\n");
		ret = palmas_watchdog_init(palmas_chip,
				palmas_chip->wdt_time_sec, "ISR");
		if (ret < 0) {
			dev_err(palmas_chip->dev,
				"BQWDT init failed %d\n", ret);
			return IRQ_NONE;
		}

		ret = palmas_charger_init(palmas_chip);
		if (ret < 0) {
			dev_err(palmas_chip->dev,
				"Charger init failed: %d\n", ret);
			return IRQ_NONE;
		}

		ret = palmas_init(palmas_chip);
		if (ret < 0) {
			dev_err(palmas_chip->dev,
				"palmas init failed: %d\n", ret);
			return IRQ_NONE;
		}
	}

	if (val & PALMAS_FAULT_BOOST_FAULT)
		dev_err(palmas_chip->dev, "Charging Fault: VBUS Overloaded\n");

	switch (val & PALMAS_FAULT_CHRG_FAULT_MASK) {
	case PALMAS_FAULT_CHRG_INPUT:
		dev_err(palmas_chip->dev, "Charging Fault: "
				"Input Fault (VBUS OVP or VBAT<VBUS<3.8V)\n");
		break;
	case PALMAS_FAULT_CHRG_THERMAL:
		dev_err(palmas_chip->dev, "Charging Fault: Thermal shutdown\n");
		break;
	case PALMAS_FAULT_CHRG_SAFTY:
		dev_err(palmas_chip->dev,
			"Charging Fault: Safety timer expiration\n");
		palmas_chip->chg_restart_timeout =
					palmas_chip->chg_restart_time /
					palmas_chip->wdt_refresh_timeout;
		break;
	default:
		break;
	}

	if (val & PALMAS_FAULT_NTC_FAULT)
		dev_err(palmas_chip->dev, "Charging Fault: NTC fault %d\n",
				val & PALMAS_FAULT_NTC_FAULT);

	ret = palmas_fault_clear_sts(palmas_chip);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"fault clear status failed %d\n", ret);
		return IRQ_NONE;
	}

	ret = palmas_charger_reg_read(palmas_chip, PALMAS_CHARGER_REG08, &val);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "SYS_STAT_REG read failed %d\n", ret);
		return IRQ_NONE;
	}

	if ((val & PALMAS_CHRG_STATE_MASK) == PALMAS_CHRG_STATE_CHARGE_DONE)
		dev_info(palmas_chip->dev, "Charging completed\n");

	return IRQ_HANDLED;
}

int palmas_init_charger_regulator(struct palmas_charger_chip *palmas_chip,
		struct palmas_charger_platform_data *pdata)
{
	int ret = 0;
	struct regulator_config rconfig = { };

	if (!pdata->bcharger_pdata) {
		dev_err(palmas_chip->dev, "No charger platform data\n");
		return 0;
	}

	palmas_chip->chg_reg_desc.name  = "palmas-charger";
	palmas_chip->chg_reg_desc.ops   = &palmas_tegra_regulator_ops;
	palmas_chip->chg_reg_desc.type  = REGULATOR_CURRENT;
	palmas_chip->chg_reg_desc.owner = THIS_MODULE;

	palmas_chip->chg_reg_init_data.supply_regulator     = NULL;
	palmas_chip->chg_reg_init_data.regulator_init       = NULL;
	palmas_chip->chg_reg_init_data.num_consumer_supplies =
				pdata->bcharger_pdata->num_consumer_supplies;
	palmas_chip->chg_reg_init_data.consumer_supplies    =
				pdata->bcharger_pdata->consumer_supplies;
	palmas_chip->chg_reg_init_data.driver_data          = palmas_chip;
	palmas_chip->chg_reg_init_data.constraints.name     = "palmas-charger";
	palmas_chip->chg_reg_init_data.constraints.min_uA   = 0;
	palmas_chip->chg_reg_init_data.constraints.max_uA   =
			pdata->bcharger_pdata->max_charge_current_mA * 1000;

	palmas_chip->chg_reg_init_data.constraints.valid_modes_mask =
						REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_STANDBY;

	palmas_chip->chg_reg_init_data.constraints.valid_ops_mask =
						REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_CURRENT;

	rconfig.dev = palmas_chip->dev;
	rconfig.of_node = NULL;
	rconfig.init_data = &palmas_chip->chg_reg_init_data;
	rconfig.driver_data = palmas_chip;
	palmas_chip->chg_rdev = regulator_register(&palmas_chip->chg_reg_desc,
					&rconfig);
	if (IS_ERR(palmas_chip->chg_rdev)) {
		ret = PTR_ERR(palmas_chip->chg_rdev);
		dev_err(palmas_chip->dev,
			"vbus-charger regulator register failed %d\n", ret);
	}
	return ret;
}

int palmas_init_vbus_regulator(struct palmas_charger_chip *palmas_chip,
		struct palmas_charger_platform_data *pdata)
{
	int ret = 0;
	struct regulator_config rconfig = { };

	if (!pdata->vbus_pdata) {
		dev_err(palmas_chip->dev, "No vbus platform data\n");
		return 0;
	}

	palmas_chip->gpio_otg_iusb = pdata->vbus_pdata->gpio_otg_iusb;
	palmas_chip->vbus_reg_desc.name = "palmas-vbus";
	palmas_chip->vbus_reg_desc.ops = &palmas_vbus_ops;
	palmas_chip->vbus_reg_desc.type = REGULATOR_VOLTAGE;
	palmas_chip->vbus_reg_desc.owner = THIS_MODULE;

	palmas_chip->vbus_reg_init_data.supply_regulator    = NULL;
	palmas_chip->vbus_reg_init_data.regulator_init      = NULL;
	palmas_chip->vbus_reg_init_data.num_consumer_supplies       =
				pdata->vbus_pdata->num_consumer_supplies;
	palmas_chip->vbus_reg_init_data.consumer_supplies   =
				pdata->vbus_pdata->consumer_supplies;
	palmas_chip->vbus_reg_init_data.driver_data         = palmas_chip;

	palmas_chip->vbus_reg_init_data.constraints.name    = "palmas-vbus";
	palmas_chip->vbus_reg_init_data.constraints.min_uV  = 0;
	palmas_chip->vbus_reg_init_data.constraints.max_uV  = 5000000,
	palmas_chip->vbus_reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;
	palmas_chip->vbus_reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_VOLTAGE;

	if (gpio_is_valid(palmas_chip->gpio_otg_iusb)) {
		ret = gpio_request_one(palmas_chip->gpio_otg_iusb,
				GPIOF_OUT_INIT_HIGH,
				dev_name(palmas_chip->dev));
		if (ret < 0) {
			dev_err(palmas_chip->dev,
				"gpio request failed  %d\n", ret);
			return ret;
		}
	}

	/* Register the regulators */
	rconfig.dev = palmas_chip->dev;
	rconfig.of_node = NULL;
	rconfig.init_data = &palmas_chip->vbus_reg_init_data;
	rconfig.driver_data = palmas_chip;
	palmas_chip->vbus_rdev = regulator_register(&palmas_chip->vbus_reg_desc,
					&rconfig);
	if (IS_ERR(palmas_chip->vbus_rdev)) {
		ret = PTR_ERR(palmas_chip->vbus_rdev);
		dev_err(palmas_chip->dev,
			"VBUS regulator register failed %d\n", ret);
		goto scrub;
	}

	/* Disable the VBUS regulator and enable charging */
	ret = palmas_charger_enable(palmas_chip);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "Charging enable failed %d", ret);
		goto scrub_reg;
	}
	return ret;

scrub_reg:
	regulator_unregister(palmas_chip->vbus_rdev);
	palmas_chip->vbus_rdev = NULL;
scrub:
	if (gpio_is_valid(palmas_chip->gpio_otg_iusb))
		gpio_free(palmas_chip->gpio_otg_iusb);
	return ret;
}

int palmas_show_chip_version(struct palmas_charger_chip *palmas_chip)
{
	int ret;
	unsigned int val;

	ret = palmas_charger_reg_read(palmas_chip, PALMAS_CHARGER_REG10, &val);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"REVISION_REG read failed: %d\n", ret);
		return ret;
	}

	if ((val & BQ24190_IC_VER) == BQ24190_IC_VER)
		dev_info(palmas_chip->dev, "chip type BQ24190 detected\n");
	else if ((val & BQ24192_IC_VER) == BQ24192_IC_VER)
		dev_info(palmas_chip->dev, "chip type PALMAS/3 detected\n");
	else if ((val & BQ24192i_IC_VER) == BQ24192i_IC_VER)
		dev_info(palmas_chip->dev, "chip type PALMASi detected\n");
	return 0;
}

int palmas_wakealarm(struct palmas_charger_chip *palmas_chip, int time_sec)
{
	int ret;
	unsigned long now;
	struct rtc_wkalrm alm;
	int alarm_time = time_sec;

	alm.enabled = true;
	ret = rtc_read_time(palmas_chip->rtc, &alm.time);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "RTC read time failed %d\n", ret);
		return ret;
	}
	rtc_tm_to_time(&alm.time, &now);

	if (!alarm_time)
		alarm_time = 3600;
	rtc_time_to_tm(now + alarm_time, &alm.time);
	ret = rtc_set_alarm(palmas_chip->rtc, &alm);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "RTC set alarm failed %d\n", ret);
		alm.enabled = false;
		return ret;
	}
	alm.enabled = false;
	return 0;
}

int palmas_hw_init(struct palmas_charger_chip *palmas_chip,
		struct palmas_charger_platform_data *pdata)
{
	int ret = 0;

	palmas_chip->rtc_alarm_time =  pdata->bcharger_pdata->rtc_alarm_time;
	palmas_chip->wdt_time_sec = pdata->bcharger_pdata->wdt_timeout;
	palmas_chip->chg_restart_time = pdata->bcharger_pdata->chg_restart_time;
	palmas_chip->rtc = alarmtimer_get_rtcdev();
	mutex_init(&palmas_chip->mutex);
	palmas_chip->suspended = 0;
	palmas_chip->chg_restart_timeout = 0;

	ret = palmas_show_chip_version(palmas_chip);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "version read failed %d\n", ret);
		return ret;
	}

	ret = palmas_charger_init(palmas_chip);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "Charger init failed: %d\n", ret);
		return ret;
	}

	ret = palmas_init_charger_regulator(palmas_chip, pdata);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"Charger regualtor init failed %d\n", ret);
		return ret;
	}

	ret = palmas_init_vbus_regulator(palmas_chip, pdata);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"VBUS regualtor init failed %d\n", ret);
		goto scrub_chg_reg;
	}

	init_kthread_worker(&palmas_chip->bq_kworker);
	palmas_chip->bq_kworker_task = kthread_run(kthread_worker_fn,
			&palmas_chip->bq_kworker,
			dev_name(palmas_chip->dev));
	if (IS_ERR(palmas_chip->bq_kworker_task)) {
		ret = PTR_ERR(palmas_chip->bq_kworker_task);
		dev_err(palmas_chip->dev,
			"Kworker task creation failed %d\n", ret);
		goto scrub_vbus_reg;
	}

	init_kthread_work(&palmas_chip->bq_wdt_work, palmas_work_thread);
	sched_setscheduler(palmas_chip->bq_kworker_task,
			SCHED_FIFO, &palmas_param);
	queue_kthread_work(&palmas_chip->bq_kworker, &palmas_chip->bq_wdt_work);

	ret = palmas_watchdog_init(palmas_chip,
			palmas_chip->wdt_time_sec, "PROBE");
	if (ret < 0) {
		dev_err(palmas_chip->dev, "BQWDT init failed %d\n", ret);
		return ret;
	}

	ret = palmas_fault_clear_sts(palmas_chip);
	if (ret < 0) {
		dev_err(palmas_chip->dev,
			"fault clear status failed %d\n", ret);
		return ret;
	}

	if (palmas_chip->irq < 0)
		ret = palmas_update_bits(palmas_chip->palmas,
			PALMAS_INTERRUPT_BASE,
			PALMAS_INT6_MASK, PALMAS_INT6_MASK_CHARGER,
			PALMAS_INT6_MASK_CHARGER);

	else {
		ret = request_threaded_irq(palmas_chip->irq, NULL,
			palmas_irq, IRQF_TRIGGER_FALLING,
				dev_name(palmas_chip->dev), palmas_chip);
		if (ret < 0) {
			dev_err(palmas_chip->dev, "request IRQ %d fail, err = %d\n",
					palmas_chip->irq, ret);
			goto scrub_kthread;
		}
	}

	/* enable charging */
	ret = palmas_charger_enable(palmas_chip);
	if (ret < 0)
		goto scrub_irq;

	return 0;
scrub_irq:
	free_irq(palmas_chip->irq, palmas_chip);
scrub_kthread:
	palmas_chip->stop_thread = true;
	flush_kthread_worker(&palmas_chip->bq_kworker);
	kthread_stop(palmas_chip->bq_kworker_task);
scrub_vbus_reg:
	regulator_unregister(palmas_chip->vbus_rdev);
scrub_chg_reg:
	regulator_unregister(palmas_chip->chg_rdev);
	mutex_destroy(&palmas_chip->mutex);
	return ret;
}

int palmas_resource_cleanup(struct palmas_charger_chip *palmas_chip)
{
	free_irq(palmas_chip->irq, palmas_chip);
	palmas_chip->stop_thread = true;
	flush_kthread_worker(&palmas_chip->bq_kworker);
	kthread_stop(palmas_chip->bq_kworker_task);
	regulator_unregister(palmas_chip->vbus_rdev);
	regulator_unregister(palmas_chip->chg_rdev);
	mutex_destroy(&palmas_chip->mutex);
	return 0;
}

static int palmas_charger_get_status(struct battery_charger_dev *bc_dev)
{
	struct palmas_charger_chip *chip = battery_charger_get_drvdata(bc_dev);

	return chip->chg_status;
}

static struct battery_charging_ops palmas_charger_bci_ops = {
	.get_charging_status = palmas_charger_get_status,
};

static struct battery_charger_info palmas_charger_bci = {
	.cell_id = 0,
	.bc_ops = &palmas_charger_bci_ops,
};

static int palmas_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct palmas_charger_chip *palmas_chip;
	struct palmas_charger_platform_data *pdata;
	struct palmas_platform_data *palmas_pdata;

	palmas_pdata = dev_get_platdata(pdev->dev.parent);
	if (!palmas_pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -ENODEV;
	}

	palmas_chip = devm_kzalloc(&pdev->dev,
			sizeof(*palmas_chip), GFP_KERNEL);
	if (!palmas_chip) {
		dev_err(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	pdata = palmas_pdata->charger_pdata;
	palmas_chip->dev = &pdev->dev;
	palmas_chip->irq = platform_get_irq(pdev, 0);
	palmas_chip->palmas = dev_get_drvdata(pdev->dev.parent);
	palmas_chip->use_regmap     = 0;

	if (!pdata->bcharger_pdata) {
		dev_err(&pdev->dev, "No battery charger platform data\n");
		return -ENODEV;
	}

	if (!pdata->bcharger_pdata->is_battery_present) {
		dev_err(&pdev->dev, "Battery not detected! Exiting driver...\n");
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, palmas_chip);

	palmas_chip->bc_dev = battery_charger_register(&pdev->dev,
					&palmas_charger_bci);
	if (IS_ERR(palmas_chip->bc_dev)) {
		ret = PTR_ERR(palmas_chip->bc_dev);
		dev_err(&pdev->dev, "battery charger register failed: %d\n",
			ret);
		return ret;
	}
	battery_charger_set_drvdata(palmas_chip->bc_dev, palmas_chip);

	ret = palmas_hw_init(palmas_chip, pdata);
	if (ret < 0) {
		dev_err(palmas_chip->dev, "hw init failed %d\n", ret);
		goto err;
	}
	return ret;
err:
	battery_charger_unregister(palmas_chip->bc_dev);
	return ret;
}

static int palmas_remove(struct platform_device *pdev)
{
	struct palmas_charger_chip *palmas_chip = dev_get_drvdata(&pdev->dev);

	battery_charger_unregister(palmas_chip->bc_dev);
	palmas_resource_cleanup(palmas_chip);
	return 0;
}

static struct platform_driver palmas_platform_driver = {
	.probe  = palmas_probe,
	.remove = palmas_remove,
	.driver = {
		.name = "palmas-charger",
		.owner = THIS_MODULE,
	},
};

static inline int palmas_charger_platform_init(void)
{
	return platform_driver_register(&palmas_platform_driver);
}

static inline void palmas_charger_platform_exit(void)
{
	return platform_driver_unregister(&palmas_platform_driver);
}

static int __init palmas_module_init(void)
{
	palmas_charger_platform_init();
	return 0;
}
subsys_initcall(palmas_module_init);

static void __exit palmas_cleanup(void)
{
	palmas_charger_platform_exit();
}
module_exit(palmas_cleanup);

MODULE_DESCRIPTION("palmas battery charger driver");
MODULE_AUTHOR("Darbha Sriharsha <dsriharsha@nvidia.com>");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_LICENSE("GPL v2");
