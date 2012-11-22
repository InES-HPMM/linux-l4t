/*
 * max77665-charger.c - Battery charger driver
 *
 *  Copyright (C) 2012 nVIDIA corporation
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
#include <linux/mfd/max77665.h>
#include <linux/max77665-charger.h>
#include <linux/power/max17042_battery.h>
#include <linux/interrupt.h>

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

struct max77665_charger {
	struct device		*dev;
	struct power_supply	ac;
	struct power_supply	usb;
	struct max77665_charger_plat_data *plat_data;
	uint8_t ac_online;
	uint8_t usb_online;
	uint8_t num_cables;
	struct extcon_dev *edev;
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
		return ret;
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

	if (access) {
		/* enable write acces to registers */
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_06, 0x0c);
		if (ret < 0) {
			dev_err(charger->dev, "Failed to write to reg 0x%x\n",
				MAX77665_CHG_CNFG_06);
			return ret;
		}
	} else {
		/* Disable write acces to registers */
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_06, 0x00);
		if (ret < 0) {
			dev_err(charger->dev, "Failed to write to reg 0x%x\n",
				MAX77665_CHG_CNFG_06);
			return ret;
		}
	}
	return 0;
}

static int max77665_charger_enable(struct max77665_charger *charger,
		enum max77665_mode mode)
{
	int ret;

	ret = max77665_enable_write(charger, true);
	if (ret < 0) {
		dev_err(charger->dev, "failed to enable write acess\n");
		return ret;
	}

	if (mode == CHARGER) {
		/* enable charging */
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_00, 0x05);
		if (ret < 0) {
			dev_err(charger->dev, "Failed in wirting to register 0x%x:\n",
					MAX77665_CHG_CNFG_00);
			return ret;
		}
	} else if (mode == OTG) {
		/* enable OTG mode */
		ret = max77665_write_reg(charger, MAX77665_CHG_CNFG_00, 0x2A);
		if (ret < 0) {
			dev_err(charger->dev, "Failed in writing to register 0x%x:\n",
					 MAX77665_CHG_CNFG_00);
			return ret;
		}
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
		if (ret < 0) {
			dev_err(charger->dev, "Failed writing to reg:0x%x\n",
				MAX77665_CHG_CNFG_09);
			goto error;
		}
	}
error:
	ret = max77665_enable_write(charger, false);
	if (ret < 0) {
		dev_err(charger->dev, "failed to enable write acess\n");
		return ret;
	}
	return ret;
}

static int max77665_enable_charger(struct max77665_charger *charger)
{
	int ret = 0;

	if (charger->plat_data->update_status)
			charger->plat_data->update_status(false);

	if (extcon_get_cable_state(charger->edev, "USB")) {

		ret = max77665_charger_enable(charger, CHARGER);
		if (ret < 0)
			goto error;

		charger->usb_online = 1;
		power_supply_changed(&charger->usb);
		charger->plat_data->curr_lim = 500;
		if (charger->plat_data->update_status)
			charger->plat_data->update_status(true);
	}

	if (extcon_get_cable_state(charger->edev, "USB-Host")) {
		ret = max77665_charger_enable(charger, OTG);
		if (ret < 0)
			goto error;
	}

	if (extcon_get_cable_state(charger->edev, "TA")) {
		ret = max77665_charger_enable(charger, CHARGER);
		if (ret < 0)
			goto error;

		charger->ac_online = 1;
		power_supply_changed(&charger->ac);
		if (charger->plat_data->update_status)
			charger->plat_data->update_status(true);
	}

	ret = max77665_charger_init(charger);
	if (ret < 0) {
		dev_err(charger->dev, "failed to initialize charger\n");
		goto error;
	}

	return 0;
error:
	return ret;
}

static int charger_extcon_notifier(struct notifier_block *self,
		unsigned long event, void *ptr)
{
	return NOTIFY_DONE;
}

static irqreturn_t max77665_charger_irq_handler(int irq, void *data)
{
	struct max77665_charger *charger = data;
	int ret;
	uint8_t read_val;

	ret = max77665_read_reg(charger, MAX77665_CHG_INT_OK, &read_val);
	if (ret < 0) {
		dev_err(charger->dev, "failed to write to reg: 0x%x\n",
				MAX77665_CHG_INT_OK);
		goto error;
	}

	if (read_val & 0x40) {
		charger->usb_online = 1;
		power_supply_changed(&charger->usb);
		power_supply_changed(&charger->ac);
		ret = max77665_enable_charger(charger);
		if (ret < 0)
			goto error;
	} else {
		charger->ac_online = 0;
		charger->usb_online = 0;
		if (charger->plat_data->update_status)
			charger->plat_data->update_status(false);
		power_supply_changed(&charger->usb);
		power_supply_changed(&charger->ac);
	}

	ret = max77665_read_reg(charger, MAX77665_CHG_INT, &read_val);
	if (ret < 0) {
		dev_err(charger->dev, "failed in reading register: 0x%x\n",
				MAX77665_CHG_INT);
		goto error;
	}

	ret = max77665_write_reg(charger, MAX77665_CHG_INT_MASK, 0x0a);
	if (ret < 0) {
		dev_err(charger->dev, "failed to write to reg: 0x%x\n",
				MAX77665_CHG_INT_MASK);
		goto error;
	}
error:
	return IRQ_HANDLED;
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
	if (ret < 0) {
		dev_err(charger->dev, "failed to write to reg: 0x%x\n",
				MAX77665_CHG_CNFG_09);
		goto error;
	}

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
	if (ret < 0) {
		dev_err(charger->dev, "failed to write to reg: 0x%x\n",
				MAX77665_CHG_INT_MASK);
		goto error;
	}

	if (charger->plat_data->irq_base) {
		ret = request_threaded_irq(charger->plat_data->irq_base +
				MAX77665_IRQ_CHARGER, NULL,
				max77665_charger_irq_handler,
				0, "charger_irq",
					charger);
		if (ret) {
			dev_err(&pdev->dev,
				"failed: irq request error :%d)\n", ret);
			goto chrg_error;
		}
	}
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

		cable->nb.notifier_call = charger_extcon_notifier;
		ret = extcon_register_interest(cable->extcon_dev,
				"max77665-muic", cable->name, &cable->nb);

		if (ret < 0) {
			dev_err(charger->dev, "Cannot register for cable: %s\n",
				cable->name);
			ret = -EINVAL;
		}
	}

	charger->edev = extcon_get_extcon_dev("max77665-muic");
	if (!charger->edev)
		return -ENODEV;

	ret = max77665_enable_charger(charger);
	if (ret < 0) {
		dev_err(charger->dev, "failed to initialize charger\n");
		goto chrg_error;
	}

	return 0;

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

	power_supply_unregister(&charger->ac);
	power_supply_unregister(&charger->usb);

	return 0;
}

static struct platform_driver max77665_battery_driver = {
	.driver = {
		.name = "max77665-charger",
		.owner = THIS_MODULE,
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
