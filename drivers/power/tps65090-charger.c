/*
 * drivers/power/tps65090-charger.c
 *
 * Battery charger driver for TI's tps65090
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mfd/tps65090.h>

#define TPS65090_INTR_STS	0x00
#define TPS65090_CG_CTRL0	0x04
#define TPS65090_CG_CTRL1	0x05
#define TPS65090_CG_CTRL2	0x06
#define TPS65090_CG_CTRL3	0x07
#define TPS65090_CG_CTRL4	0x08
#define TPS65090_CG_CTRL5	0x09
#define TPS65090_CG_STATUS1	0x0a
#define TPS65090_CG_STATUS2	0x0b

#define TPS65090_NOITERM	BIT(5)
#define CHARGER_ENABLE		0x01
#define	TPS65090_VACG		0x02

struct tps65090_charger {
	struct	device	*dev;
	int	irq_base;
	int	ac_online;
	int	prev_ac_online;
	struct power_supply	ac;
};

static enum power_supply_property tps65090_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int tps65090_low_chrg_current(struct tps65090_charger *charger)
{
	int ret;

	ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL5,
			TPS65090_NOITERM);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
			__func__, TPS65090_CG_CTRL5);
		return ret;
	}
	return 0;
}

static int tps65090_enable_charging(struct tps65090_charger *charger,
	uint8_t enable)
{
	int ret;
	uint8_t retval;

	ret = tps65090_read(charger->dev->parent, TPS65090_CG_CTRL0, &retval);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}

	ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL0,
				(retval | CHARGER_ENABLE));
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}
	return 0;
}

static int tps65090_config_charger(struct tps65090_charger *charger)
{
	int ret;

	ret = tps65090_low_chrg_current(charger);
	if (ret < 0) {
		dev_err(charger->dev,
			"error configuring low charge current\n");
		return ret;
	}

	/* Disable charging if any */
	ret = tps65090_enable_charging(charger, 0);
	if (ret < 0) {
		dev_err(charger->dev, "error enabling charger\n");
		return ret;
	}

	return 0;
}

static int tps65090_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct tps65090_charger *charger = container_of(psy,
					struct tps65090_charger, ac);

	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = charger->ac_online;
		charger->prev_ac_online = charger->ac_online;
		return 0;
	}
	return -EINVAL;
}

static irqreturn_t tps65090_charger_isr(int irq, void *dev_id)
{
	struct tps65090_charger *charger = dev_id;
	int ret;
	uint8_t retval;

	ret = tps65090_read(charger->dev->parent, TPS65090_INTR_STS, &retval);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Error in reading reg 0x%x\n",
				__func__, TPS65090_INTR_STS);
		goto error;
	}
	if (retval & TPS65090_VACG) {
		ret = tps65090_enable_charging(charger, 1);
		if (ret < 0)
			goto error;
		charger->ac_online = 1;
	} else {
		charger->ac_online = 0;
	}

	if (charger->prev_ac_online != charger->ac_online)
		power_supply_changed(&charger->ac);
error:
	return IRQ_HANDLED;
}

static __devinit int tps65090_charger_probe(struct platform_device *pdev)
{
	uint8_t retval;
	int ret;
	struct tps65090_charger *charger_data;
	struct tps65090_plat_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s():no platform data available\n",
				__func__);
		return -ENODEV;
	}

	charger_data = devm_kzalloc(&pdev->dev, sizeof(*charger_data),
			GFP_KERNEL);
	if (!charger_data) {
		dev_err(&pdev->dev, "failed to allocate memory status\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, charger_data);

	charger_data->dev = &pdev->dev;

	/* Check for battery presence */
	ret = tps65090_read(charger_data->dev->parent, TPS65090_CG_STATUS1,
			&retval);
	if (ret < 0) {
		dev_err(charger_data->dev, "%s(): Error in reading reg 0x%x",
			__func__, TPS65090_CG_STATUS1);
		return -ENODEV;
	}

	if (!(retval & 0x04)) {
		dev_err(charger_data->dev, "No battery. Exiting driver\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(&pdev->dev, charger_data->irq_base,
			NULL, tps65090_charger_isr, 0, "tps65090",
			charger_data);
	if (ret) {
		dev_err(charger_data->dev, "Unable to register irq %d err %d\n",
				charger_data->irq_base, ret);
		return ret;
	}

	charger_data->ac.name		= "tps65090-ac";
	charger_data->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	charger_data->ac.get_property	= tps65090_ac_get_property;
	charger_data->ac.properties	= tps65090_ac_props;
	charger_data->ac.num_properties	= ARRAY_SIZE(tps65090_ac_props);

	ret = power_supply_register(&pdev->dev, &charger_data->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		return ret;
	}

	ret = tps65090_config_charger(charger_data);
	if (ret < 0)
		goto error;

	return 0;
error:
	power_supply_unregister(&charger_data->ac);
	return ret;
}

static int tps65090_charger_remove(struct platform_device *pdev)
{
	struct tps65090_charger *charger = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(&charger->ac);
	return 0;
}

static struct platform_driver tps65090_charger_driver = {
	.driver	= {
		.name	= "tps65090-charger",
		.owner	= THIS_MODULE,
	},
	.probe	= tps65090_charger_probe,
	.remove = __devexit_p(tps65090_charger_remove),
};

module_platform_driver(tps65090_charger_driver);

MODULE_LICENSE("GPL V2");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_DESCRIPTION("tps65090 battery charger driver");
