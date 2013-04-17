/*
 * max77665-charger.c - Battery charger driver
 *
 *  Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/alarmtimer.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/mfd/max77665.h>
#include <linux/max77665-charger.h>
#include <linux/power/max17042_battery.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>

#define CHARGER_TYPE_DETECTION_DEBOUNCE_TIME_MS 500

/* fast charge current in mA */
static const uint32_t chg_cc[]  = {
	0, 33, 66, 99, 133, 166, 199, 233, 266, 299,
	333, 366, 399, 432, 466, 499, 532, 566, 599, 632,
	666, 699, 732, 765, 799, 832, 865, 899, 932, 965,
	999, 1032, 1065, 1098, 1132, 1165, 1198, 1232, 1265,
	1298, 1332, 1365, 1398, 1421, 1465, 1498, 1531, 1565,
	1598, 1631, 1665, 1698, 1731, 1764, 1798, 1831, 1864,
	1898, 1931, 1964, 1998, 2031, 2064, 2097,
};

/* primary charge termination voltage in mV */
static const uint32_t chg_cv_prm[] = {
	3650, 3675, 3700, 3725, 3750,
	3775, 3800, 3825, 3850, 3875,
	3900, 3925, 3950, 3975, 4000,
	4025, 4050, 4075, 4100, 4125,
	4150, 4175, 4200, 4225, 4250,
	4275, 4300, 4325, 4340, 4350,
	4375, 4400,
};

/* maxim input current limit in mA*/
static const uint32_t chgin_ilim[] = {
	0, 100, 200, 300, 400, 500, 600, 700,
	800, 900, 1000, 1100, 1200, 1300, 1400,
	1500, 1600, 1700, 1800, 1900, 2000, 2100,
	2200, 2300, 2400, 2500,
};

static int max77665_bat_to_sys_oc_thres[] = {
	0, 3000, 3250, 3500, 3750, 4000, 4250, 4500
};


struct max77665_charger {
	struct device		*dev;
	int			irq;
	struct power_supply	ac;
	struct power_supply	usb;
	struct max77665_charger_plat_data *plat_data;
	uint8_t ac_online;
	uint8_t usb_online;
	uint8_t num_cables;
	struct extcon_dev *edev;
	struct alarm wdt_alarm;
	struct delayed_work wdt_ack_work;
	struct wake_lock wdt_wake_lock;
};

static enum power_supply_property max77665_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property max77665_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int max77665_write_reg(struct max77665_charger *charger,
	uint8_t reg, uint8_t value)
{
	int ret = 0;
	struct device *dev = charger->dev;

	ret = max77665_write(dev->parent, MAX77665_I2C_SLAVE_PMIC, reg, value);
	if (ret < 0)
		dev_err(charger->dev, "Failed to write to reg 0x%x\n", reg);
	return ret;
}

static int max77665_read_reg(struct max77665_charger *charger,
	uint8_t reg, uint32_t *value)
{
	int ret = 0;
	uint8_t read_val;

	struct device *dev = charger->dev;

	ret = max77665_read(dev->parent, MAX77665_I2C_SLAVE_PMIC,
			reg, &read_val);
	if (!ret)
		*value = read_val;

	return ret;
}

static int max77665_update_reg(struct max77665_charger *charger,
	uint8_t reg, uint8_t value)
{
	int ret = 0;
	uint8_t read_val;
	struct device *dev = charger->dev;

	ret = max77665_read(dev->parent, MAX77665_I2C_SLAVE_PMIC,
			reg, &read_val);
	if (ret)
		return ret;

	ret = max77665_write(dev->parent, MAX77665_I2C_SLAVE_PMIC, reg,
			read_val | value);
	if (ret)
		return ret;

	return ret;
}

/* Convert current to register value using lookup table */
static int convert_to_reg(const unsigned int *tbl, size_t size,
		unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

static int max77665_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max77665_charger *chip = container_of(psy,
				struct max77665_charger, ac);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->ac_online;
	else
		return -EINVAL;

	return 0;
}

static int max77665_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max77665_charger *chip = container_of(psy,
					struct max77665_charger, usb);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->usb_online;
	else
		return -EINVAL;

	return 0;
}

static int max77665_enable_write(struct max77665_charger *charger, int access)
{
	int ret = 0;

	if (access)
		/* enable write acces to registers */
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_06, 0x0c);
	else
		/* Disable write acces to registers */
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_06, 0x00);
	return ret;
}

static int max77665_charger_enable(struct max77665_charger *charger,
		enum max77665_mode mode)
{
	int ret;
	int flags;

	ret = max77665_enable_write(charger, true);
	if (ret < 0) {
		dev_err(charger->dev, "failed to enable write acess\n");
		return ret;
	}

	if (mode == CHARGER) {
		/* enable charging */
		flags = CHARGER_ON_OTG_OFF_BUCK_OFF_BOOST_ON | WDTEN;
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_00, flags);
		if (ret < 0)
			return ret;
	} else if (mode == OTG) {
		/* enable OTG mode */
		flags = CHARGER_OFF_OTG_ON_BUCK_OFF_BOOST_ON;
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_00, flags);
		if (ret < 0)
			return ret;
	}

	ret = max77665_enable_write(charger, false);
	if (ret < 0) {
		dev_err(charger->dev, "failed to disable write acess\n");
		return ret;
	}
	return 0;
}

static int max77665_charger_init(struct max77665_charger *charger)
{
	int ret = 0;

	ret = max77665_enable_write(charger, true);
	if (ret < 0) {
		dev_err(charger->dev, "failed to enable write acess\n");
		goto error;
	}

	ret = max77665_update_reg(charger, MAX77665_CHG_CNFG_01, 0xa4);
	if (ret < 0) {
		dev_err(charger->dev, "Failed in writing to register 0x%x\n",
			MAX77665_CHG_CNFG_01);
		goto error;
	}

	if (charger->plat_data->fast_chg_cc) {
		ret = convert_to_reg(chg_cc, ARRAY_SIZE(chg_cc),
					charger->plat_data->fast_chg_cc);
		if (ret < 0)
			goto error;

		ret = max77665_update_reg(charger, MAX77665_CHG_CNFG_02, ret);
		if (ret < 0) {
			dev_err(charger->dev, "Failed in writing to register 0x%x\n",
				MAX77665_CHG_CNFG_02);
			goto error;
		}
	}

	if (charger->plat_data->term_volt) {
		ret = convert_to_reg(chg_cv_prm, ARRAY_SIZE(chg_cv_prm),
					charger->plat_data->term_volt);
		if (ret < 0)
			goto error;

		ret = max77665_update_reg(charger,
				MAX77665_CHG_CNFG_04, ret+1);
		if (ret < 0) {
			dev_err(charger->dev, "Failed writing to reg:0x%x\n",
				MAX77665_CHG_CNFG_04);
			goto error;
		}
	}

	if (charger->plat_data->curr_lim) {
		ret = convert_to_reg(chgin_ilim, ARRAY_SIZE(chgin_ilim),
					charger->plat_data->curr_lim);
		if (ret < 0)
			goto error;

		ret = max77665_write_reg(charger,
				MAX77665_CHG_CNFG_09, ret*5);
		if (ret < 0)
			goto error;
	}
error:
	ret = max77665_enable_write(charger, false);
	if (ret < 0) {
		dev_err(charger->dev, "failed to enable write acess\n");
		return ret;
	}
	return ret;
}

static int max77665_enable_charger(struct max77665_charger *charger,
		struct extcon_dev *edev)
{
	int ret = 0;
	uint32_t val = 0;
	int ilim;

	charger->usb_online = 0;
	charger->ac_online = 0;

	ret = max77665_read_reg(charger, MAX77665_CHG_CNFG_09, &val);
	if (0 > ret)
		return ret;
	val &= 0x7F;
	ilim = max_t(int, 60, val * 20);

	if (charger->plat_data->update_status)
		charger->plat_data->update_status(false);

	if (true == extcon_get_cable_state(edev, "USB-Host")) {
		ret = max77665_charger_enable(charger, OTG);
		if (0 > ret)
			dev_err(charger->dev,
				"failed to set device to USB-host mode");
		power_supply_changed(&charger->usb);
		power_supply_changed(&charger->ac);
		return ret;
	}

	if (true == extcon_get_cable_state(edev, "USB")) {
		charger->usb_online = 1;
		charger->plat_data->curr_lim = 500;
	} else if (true == extcon_get_cable_state(edev, "Charge-downstream")) {
		charger->usb_online = 1;
	} else if (true == extcon_get_cable_state(edev, "TA")) {
		charger->ac_online = 1;
	} else if (true == extcon_get_cable_state(edev, "Fast-charger")) {
		charger->ac_online = 1;
	} else if (true == extcon_get_cable_state(edev, "Slow-charger")) {
		charger->ac_online = 1;
		charger->plat_data->curr_lim = 500;
	} else {
		/* no cable connected */
		return 0;
	}

	power_supply_changed(charger->usb_online ?
			&charger->usb : &charger->ac);

	if (charger->plat_data->update_status)
		charger->plat_data->update_status(ilim);

	ret = max77665_charger_enable(charger, CHARGER);
	if (ret < 0) {
		dev_err(charger->dev, "failed to set device to charger mode\n");
		return ret;
	}

	ret = max77665_charger_init(charger);
	if (ret < 0) {
		dev_err(charger->dev, "failed to initialize charger\n");
		return ret;
	}

	/* set the charging watchdog timer */
	alarm_start(&charger->wdt_alarm, ktime_add(ktime_get_boottime(),
			ktime_set(MAX77665_WATCHDOG_TIMER_PERIOD_S / 2, 0)));

	return 0;
}

static void charger_extcon_handle_notifier(struct work_struct *w)
{
	struct max77665_charger_cable *cable = container_of(to_delayed_work(w),
			struct max77665_charger_cable, extcon_notifier_work);

	if (cable->event == 0) {
		cable->charger->ac_online = 0;
		cable->charger->usb_online = 0;
		if (cable->charger->plat_data->update_status)
			cable->charger->plat_data->update_status(0);
		power_supply_changed(&cable->charger->usb);
		power_supply_changed(&cable->charger->ac);
	} else {
		max77665_enable_charger(cable->charger,
				cable->extcon_dev->edev);
	}
}

static void max77665_charger_wdt_ack_work_handler(struct work_struct *w)
{
	struct max77665_charger *charger = container_of(to_delayed_work(w),
			struct max77665_charger, wdt_ack_work);

	if (0 > max77665_update_reg(charger, MAX77665_CHG_CNFG_06, WDTCLR))
		dev_err(charger->dev, "fail to ack charging WDT\n");

	alarm_start(&charger->wdt_alarm,
			ktime_add(ktime_get_boottime(), ktime_set(30, 0)));
	wake_unlock(&charger->wdt_wake_lock);
}

static enum alarmtimer_restart max77665_charger_wdt_timer(struct alarm *alarm,
		ktime_t now)
{
	struct max77665_charger *charger =
		container_of(alarm, struct max77665_charger, wdt_alarm);

	wake_lock(&charger->wdt_wake_lock);
	schedule_delayed_work(&charger->wdt_ack_work, 0);
	return ALARMTIMER_NORESTART;
}

static void max77665_charger_disable_wdt(struct max77665_charger *charger)
{
	cancel_delayed_work_sync(&charger->wdt_ack_work);
	alarm_cancel(&charger->wdt_alarm);
}

static int charger_extcon_notifier(struct notifier_block *self,
		unsigned long event, void *ptr)
{
	struct max77665_charger_cable *cable = container_of(self,
		struct max77665_charger_cable, nb);

	cable->event = event;
	cancel_delayed_work(&cable->extcon_notifier_work);
	schedule_delayed_work(&cable->extcon_notifier_work,
		msecs_to_jiffies(CHARGER_TYPE_DETECTION_DEBOUNCE_TIME_MS));

	return NOTIFY_DONE;
}

static int max77665_display_charger_status(struct max77665_charger *charger,
		uint32_t val)
{
	int i;
	int bits[] = { BYP_OK, DETBAT_OK, BAT_OK, CHG_OK, CHGIN_OK };
	char *info[] = {
		"bypass",
		"main battery presence",
		"battery",
		"charger",
		"charging input"
	};

	for (i = 0; i < ARRAY_SIZE(bits); i++)
		dev_err(charger->dev, "%s %s OK", info[i],
			(val & bits[i]) ? "is" : "is not");

	return 0;
}

static int max77665_update_charger_status(struct max77665_charger *charger)
{
	int ret;
	uint32_t read_val;

	ret = max77665_read_reg(charger, MAX77665_CHG_INT, &read_val);
	if (ret < 0) {
		dev_err(charger->dev, "failed in reading register: 0x%x\n",
				MAX77665_CHG_INT);
		goto error;
	}
	dev_dbg(charger->dev, "CHG_INT = 0x%02x\n", read_val);

	if (read_val & CHG_I) {
		ret = max77665_read_reg(charger, MAX77665_CHG_INT_OK,
				&read_val);
		if (ret < 0) {
			dev_err(charger->dev, "failed to reading reg: 0x%x\n",
					MAX77665_CHG_INT_OK);
			goto error;
		}

		dev_info(charger->dev, "CHG_INT_OK = 0x%02x\n", read_val);
		max77665_display_charger_status(charger, read_val);
	}

	ret = max77665_update_bits(charger->dev->parent,
			MAX77665_I2C_SLAVE_PMIC,
			MAX77665_CHG_INT_MASK, ~0x0a, 0x0);
error:
	return ret;
}

static irqreturn_t max77665_charger_irq_handler(int irq, void *data)
{
	struct max77665_charger *charger = data;
	int ret;
	ret = max77665_update_charger_status(charger);
	return IRQ_HANDLED;
}

static ssize_t max77665_set_bat_oc_threshold(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);
	int i;
	int ret;
	int val = 0;
	int n = ARRAY_SIZE(max77665_bat_to_sys_oc_thres);
	char *p = (char *)buf;
	int oc_curr = memparse(p, &p);

	for (i = 0; i < n; ++i) {
		if (oc_curr <= max77665_bat_to_sys_oc_thres[i])
			break;
	}

	val = (i < n) ? i : n - 1;
	ret = max77665_update_bits(charger->dev->parent,
			MAX77665_I2C_SLAVE_PMIC,
			MAX77665_CHG_CNFG_12, 0x7, val);
	if (ret < 0) {
		dev_err(charger->dev, "CHG_CNFG_12 update failed: %d\n", ret);
		return ret;
	}
	return count;
}

static ssize_t max77665_show_bat_oc_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);
	uint8_t val = 0;
	int ret;

	ret = max77665_read(charger->dev->parent, MAX77665_I2C_SLAVE_PMIC,
				MAX77665_CHG_CNFG_12, &val);
	if (ret < 0) {
		dev_err(charger->dev, "CHG_CNFG_12 read failed: %d\n", ret);
		return ret;
	}
	return sprintf(buf, "%d\n", max77665_bat_to_sys_oc_thres[val & 0x7]);
}
static DEVICE_ATTR(oc_threshold,  0644,
		max77665_show_bat_oc_threshold, max77665_set_bat_oc_threshold);

static ssize_t max77665_set_battery_oc_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);
	int ret;
	bool enabled;
	unsigned int val;

	if ((*buf == 'E') || (*buf == 'e')) {
		enabled = true;
	} else if ((*buf == 'D') || (*buf == 'd')) {
		enabled = false;
	} else {
		dev_err(charger->dev, "Illegal option\n");
		return -EINVAL;
	}

	val = (enabled) ? 0x0 : 0x8;
	ret = max77665_update_bits(charger->dev->parent,
			MAX77665_I2C_SLAVE_PMIC,
			MAX77665_CHG_INT_MASK, 0x08, val);
	if (ret < 0) {
		dev_err(charger->dev, "CHG_INT_MASK update failed: %d\n", ret);
		return ret;
	}
	return count;
}

static ssize_t max77665_show_battery_oc_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);
	uint8_t val = 0;
	int ret;

	ret = max77665_read(charger->dev->parent, MAX77665_I2C_SLAVE_PMIC,
			 MAX77665_CHG_INT_MASK, &val);
	if (ret < 0) {
		dev_err(charger->dev, "CHG_CNFG_12 read failed: %d\n", ret);
		return ret;
	}
	if (val & 0x8)
		return sprintf(buf, "disabled\n");
	else
		return sprintf(buf, "enabled\n");
}
static DEVICE_ATTR(oc_state, 0644,
		max77665_show_battery_oc_state, max77665_set_battery_oc_state);

static struct attribute *max77665_chg_attributes[] = {
	&dev_attr_oc_threshold.attr,
	&dev_attr_oc_state.attr,
	NULL,
};

static const struct attribute_group max77665_chg_attr_group = {
	.attrs = max77665_chg_attributes,
};

static int max77665_add_sysfs_entry(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &max77665_chg_attr_group);
}
static void max77665_remove_sysfs_entry(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &max77665_chg_attr_group);
}

static __devinit int max77665_battery_probe(struct platform_device *pdev)
{
	int ret = 0;
	uint8_t j;
	uint32_t read_val;
	struct max77665_charger *charger;

	charger = devm_kzalloc(&pdev->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger) {
		dev_err(&pdev->dev, "failed to allocate memory status\n");
		return -ENOMEM;
	}

	charger->dev = &pdev->dev;

	charger->plat_data = pdev->dev.platform_data;
	dev_set_drvdata(&pdev->dev, charger);

	/* modify OTP setting of input current limit to 100ma */
	ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_09, 0x05);
	if (ret < 0)
		goto error;

	/* check for battery presence */
	ret = max77665_read_reg(charger, MAX77665_CHG_DTLS_01, &read_val);
	if (ret) {
		dev_err(&pdev->dev, "error in reading register 0x%x\n",
				MAX77665_CHG_DTLS_01);
		return -ENODEV;
	} else if (!(read_val & 0xe0)) {
		dev_err(&pdev->dev, "Battery not detected exiting driver..\n");
		return -ENODEV;
	}

	/* differentiate between E1236 and E1587*/
	ret = maxim_get_temp();
	if (ret == 0xff) {
		dev_err(&pdev->dev, "failed in reading temperaure\n");
		return -ENODEV;
	} else if ((ret < MIN_TEMP) || (ret > MAX_TEMP)) {
			dev_err(&pdev->dev, "E1236 detected exiting driver....\n");
			return -ENODEV;
	}

	ret = max77665_write_reg(charger, MAX77665_CHG_INT_MASK, 0x0a);
	if (ret < 0)
		goto error;

	charger->ac.name		= "ac";
	charger->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	charger->ac.get_property	= max77665_ac_get_property;
	charger->ac.properties		= max77665_ac_props;
	charger->ac.num_properties	= ARRAY_SIZE(max77665_ac_props);

	ret = power_supply_register(charger->dev, &charger->ac);
	if (ret) {
		dev_err(charger->dev, "failed: power supply register\n");
		goto error;
	}

	charger->usb.name		= "usb";
	charger->usb.type		= POWER_SUPPLY_TYPE_USB;
	charger->usb.get_property	= max77665_usb_get_property;
	charger->usb.properties		= max77665_usb_props;
	charger->usb.num_properties	= ARRAY_SIZE(max77665_usb_props);

	ret = power_supply_register(charger->dev, &charger->usb);
	if (ret) {
		dev_err(charger->dev, "failed: power supply register\n");
		goto pwr_sply_error;
	}

	for (j = 0 ; j < charger->plat_data->num_cables; j++) {
		struct max77665_charger_cable *cable =
				&charger->plat_data->cables[j];
		cable->extcon_dev =  devm_kzalloc(&pdev->dev,
			sizeof(struct extcon_specific_cable_nb), GFP_KERNEL);
		if (!cable->extcon_dev) {
			dev_err(&pdev->dev, "failed to allocate memory for extcon_dev\n");
			return -ENOMEM;
		}

		INIT_DELAYED_WORK(&cable->extcon_notifier_work,
			charger_extcon_handle_notifier);

		cable->charger = charger;
		cable->nb.notifier_call = charger_extcon_notifier;
		ret = extcon_register_interest(cable->extcon_dev,
			charger->plat_data->extcon_name,
			cable->name, &cable->nb);

		if (ret < 0) {
			dev_err(charger->dev, "Cannot register for cable: %s\n",
				cable->name);
			ret = -EINVAL;
		}
	}

	charger->edev = extcon_get_extcon_dev(charger->plat_data->extcon_name);
	if (!charger->edev)
		goto chrg_error;

	charger->irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(charger->irq, NULL,
			max77665_charger_irq_handler, 0, "charger_irq",
			charger);
	if (ret) {
		dev_err(&pdev->dev,
				"failed: irq request error :%d)\n", ret);
		goto chrg_error;
	}

	ret = max77665_add_sysfs_entry(&pdev->dev);
	if (ret < 0) {
		dev_err(charger->dev, "sysfs create failed %d\n", ret);
		goto free_irq;
	}

	ret = max77665_enable_charger(charger, charger->edev);
	if (ret < 0) {
		dev_err(charger->dev, "failed to enable charger\n");
		goto remove_sysfs;
	}

	/* Enable OC interrupt and threshold to 3250mA */
	ret = max77665_write_reg(charger, MAX77665_CHG_INT_MASK, 0x02);
	if (ret < 0) {
		dev_err(charger->dev, "CHG_INT_MASK write failed %d\n", ret);
		goto remove_sysfs;
	}

	ret = max77665_update_bits(charger->dev->parent,
			MAX77665_I2C_SLAVE_PMIC,
			MAX77665_CHG_CNFG_12, 0x7, 0x2);
	if (ret < 0) {
		dev_err(charger->dev, "CHG_CNFG_12 update failed: %d\n", ret);
		return ret;
	}

	wake_lock_init(&charger->wdt_wake_lock, WAKE_LOCK_SUSPEND,
			"max77665-charger-wdt");
	alarm_init(&charger->wdt_alarm, ALARM_BOOTTIME,
			max77665_charger_wdt_timer);
	INIT_DELAYED_WORK(&charger->wdt_ack_work,
			max77665_charger_wdt_ack_work_handler);

	return 0;

remove_sysfs:
	max77665_remove_sysfs_entry(&pdev->dev);
free_irq:
	free_irq(charger->irq, charger);
chrg_error:
	power_supply_unregister(&charger->usb);
pwr_sply_error:
	power_supply_unregister(&charger->ac);
error:
	return ret;
}

static int __devexit max77665_battery_remove(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);

	max77665_remove_sysfs_entry(&pdev->dev);
	free_irq(charger->irq, charger);
	power_supply_unregister(&charger->ac);
	power_supply_unregister(&charger->usb);

	return 0;
}
#ifdef CONFIG_PM_SLEEP
static int max77665_suspend(struct device *dev)
{
	return 0;
}
static int max77665_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77665_charger *charger = platform_get_drvdata(pdev);
	int ret;
	ret = max77665_update_charger_status(charger);
	if (ret < 0)
		dev_err(charger->dev, "error occured in resume\n");
	return ret;
}

static const struct dev_pm_ops max77665_pm = {
	.suspend = max77665_suspend,
	.resume = max77665_resume,
};
#define MAX77665_PM	(&max77665_pm)
#else
#define MAX77665_PM	NULL
#endif
static struct platform_driver max77665_battery_driver = {
	.driver = {
		.name = "max77665-charger",
		.owner = THIS_MODULE,
		.pm	= MAX77665_PM,
	},
	.probe = max77665_battery_probe,
	.remove = __devexit_p(max77665_battery_remove),

};

static int __init max77665_battery_init(void)
{
	return platform_driver_register(&max77665_battery_driver);
}

static void __exit max77665_battery_exit(void)
{
	platform_driver_unregister(&max77665_battery_driver);
}

late_initcall(max77665_battery_init);
module_exit(max77665_battery_exit);

MODULE_DESCRIPTION("MAXIM MAX77665 battery charging driver");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_LICENSE("GPL v2");
