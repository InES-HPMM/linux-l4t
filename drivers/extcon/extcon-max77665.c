/*
 * extcon-max77665.c - MAX77665 extcon driver to support MAX77665 MUIC
 *
 * Copyright (C) 2012 nvidia corporation
 * Syed Rafiuddin <srafiuddin@nvidia.com>
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/extcon.h>
#include <linux/mfd/max77665.h>
#include <linux/max77665-charger.h>

#define	DEV_NAME	"max77665-muic"

#define MAX77665_MUIC_REG_ID		0x0
#define MAX77665_MUIC_REG_INT1		0x1
#define MAX77665_MUIC_REG_INT2		0x2
#define MAX77665_MUIC_REG_INT3		0x3
#define MAX77665_MUIC_REG_STATUS1	0x4
#define MAX77665_MUIC_REG_STATUS2	0x5
#define MAX77665_MUIC_REG_STATUS3	0x6
#define MAX77665_MUIC_REG_INTMASK1	0x7
#define MAX77665_MUIC_REG_INTMASK2	0x8
#define MAX77665_MUIC_REG_INTMASK3	0x9
#define MAX77665_MUIC_REG_CDETCTRL	0xa

#define MAX77665_MUIC_REG_CONTROL1	0xc
#define MAX77665_MUIC_REG_CONTROL2	0xd
#define MAX77665_MUIC_REG_CONTROL3	0xe

/* MAX77665-MUIC STATUS1 register */
#define STATUS1_ADC_SHIFT		0
#define STATUS1_ADCLOW_SHIFT		5
#define STATUS1_ADCERR_SHIFT		6
#define STATUS1_ADC_MASK		(0x1f << STATUS1_ADC_SHIFT)

/* MAX77665-MUIC STATUS2 register */
#define STATUS2_CHGTYP_SHIFT		0
#define STATUS2_CHGDETRUN_SHIFT		3
#define STATUS2_DCDTMR_SHIFT		4
#define STATUS2_DBCHG_SHIFT		5
#define STATUS2_VBVOLT_SHIFT		6
#define STATUS2_CHGTYP_MASK		(0x7 << STATUS2_CHGTYP_SHIFT)

/* MAX77665-MUIC CONTROL1 register */
#define COMN1SW_SHIFT			0
#define COMP2SW_SHIFT			3
#define COMN1SW_MASK			(0x7 << COMN1SW_SHIFT)
#define COMP2SW_MASK			(0x7 << COMP2SW_SHIFT)
#define SW_MASK				(COMP2SW_MASK | COMN1SW_MASK)

#define MAX77665_SW_USB		((1 << COMP2SW_SHIFT) | (1 << COMN1SW_SHIFT))
#define MAX77665_SW_OPEN	((0 << COMP2SW_SHIFT) | (0 << COMN1SW_SHIFT))

#define MAX77665_ADC_GROUND	0x00
#define MAX77665_ADC_OPEN	0x1f

enum max77665_muic_usb_type {
	MAX77665_USB_HOST,
	MAX77665_USB_DEVICE,
};

enum max77665_muic_charger_type {
	MAX77665_CHARGER_TYPE_NONE = 0,
	MAX77665_CHARGER_TYPE_USB,
	MAX77665_CHARGER_TYPE_DOWNSTREAM_PORT,
	MAX77665_CHARGER_TYPE_DEDICATED_CHG,
	MAX77665_CHARGER_TYPE_500MA,
	MAX77665_CHARGER_TYPE_1A,
};

struct max77665_muic {
	struct device *dev;
	struct max77665_muic_platform_data *pdata;

	int irq;
	struct work_struct irq_work;

	enum max77665_muic_charger_type pre_charger_type;
	int pre_adc;

	struct mutex mutex;

	struct extcon_dev	*edev;
};

const char *max77665_extcon_cable[] = {
	[0] = "USB",
	[1] = "USB-Host",
	[2] = "TA",
	[3] = "Fast-charger",
	[4] = "Slow-charger",
	[5] = "Charge-downstream",
	NULL,
};

static int max77665_read_reg(struct max77665_muic *muic,
		uint8_t reg, uint8_t *value)
{
	int ret;
	struct device *dev = muic->dev;

	ret = max77665_read(dev->parent, MAX77665_I2C_SLAVE_MUIC, reg, value);
	if (ret < 0)
		return ret;
	return 0;
}

static int max77665_update_reg(struct max77665_muic *muic,
				uint8_t reg, uint8_t value, uint8_t mask)
{
	int ret;
	uint8_t read_val;
	struct device *dev = muic->dev;

	ret = max77665_read(dev->parent, MAX77665_I2C_SLAVE_MUIC,
				reg, &read_val);
	if (ret < 0)
		return ret;

	ret = max77665_write(dev->parent, MAX77665_I2C_SLAVE_MUIC, reg,
			(value & mask) | (read_val & (~mask)));
	if (ret < 0)
		return ret;
	return 0;
}

static int max77665_muic_handle_usb(struct max77665_muic *muic,
			enum max77665_muic_usb_type usb_type, bool attached)
{
	int ret = 0;

	if (usb_type == MAX77665_USB_HOST) {
		ret = max77665_update_reg(muic, MAX77665_MUIC_REG_CONTROL1,
				attached ? MAX77665_SW_USB : MAX77665_SW_OPEN,
				SW_MASK);
		if (ret) {
			dev_err(muic->dev, "failed to update muic register\n");
			goto out;
		}
	}

	switch (usb_type) {
	case MAX77665_USB_HOST:
		extcon_set_cable_state(muic->edev, "USB-Host", attached);
		break;
	case MAX77665_USB_DEVICE:
		extcon_set_cable_state(muic->edev, "USB", attached);
		break;
	default:
		ret = -EINVAL;
		break;
	}

out:
	return ret;
}

static int max77665_muic_handle_adc_detach(struct max77665_muic *muic)
{
	int ret = 0;

	switch (muic->pre_adc) {
	case MAX77665_ADC_GROUND:
		ret = max77665_muic_handle_usb(muic, MAX77665_USB_HOST, false);
		break;
	default:
		break;
	}

	return ret;
}

static int max77665_muic_handle_adc(struct max77665_muic *muic, int adc)
{
	int ret = 0;

	switch (adc) {
	case MAX77665_ADC_GROUND:
		ret = max77665_muic_handle_usb(muic, MAX77665_USB_HOST, true);
		break;
	case MAX77665_ADC_OPEN:
		ret = max77665_muic_handle_adc_detach(muic);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	muic->pre_adc = adc;
out:
	return ret;
}

static int max77665_muic_handle_charger_type_detach(
				struct max77665_muic *muic)
{
	int ret = 0;

	switch (muic->pre_charger_type) {
	case MAX77665_CHARGER_TYPE_USB:
		extcon_set_cable_state(muic->edev, "USB", false);
		break;
	case MAX77665_CHARGER_TYPE_DOWNSTREAM_PORT:
		extcon_set_cable_state(muic->edev, "Charge-downstream", false);
		break;
	case MAX77665_CHARGER_TYPE_DEDICATED_CHG:
		extcon_set_cable_state(muic->edev, "TA", false);
		break;
	case MAX77665_CHARGER_TYPE_500MA:
		extcon_set_cable_state(muic->edev, "Slow-charger", false);
		break;
	case MAX77665_CHARGER_TYPE_1A:
		extcon_set_cable_state(muic->edev, "Fast-charger", false);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max77665_muic_handle_charger_type(struct max77665_muic *muic,
				enum max77665_muic_charger_type charger_type)
{
	uint8_t adc;
	int ret = 0;

	ret = max77665_read_reg(muic, MAX77665_MUIC_REG_STATUS1, &adc);
	if (ret) {
		dev_err(muic->dev, "failed to read muic register\n");
		goto out;
	}

	switch (charger_type) {
	case MAX77665_CHARGER_TYPE_NONE:
		ret = max77665_muic_handle_charger_type_detach(muic);
		break;
	case MAX77665_CHARGER_TYPE_USB:
		if ((adc & STATUS1_ADC_MASK) == MAX77665_ADC_OPEN) {
			max77665_muic_handle_usb(muic,
					MAX77665_USB_DEVICE, true);
		}
		break;
	case MAX77665_CHARGER_TYPE_DOWNSTREAM_PORT:
		extcon_set_cable_state(muic->edev, "Charge-downstream", true);
		break;
	case MAX77665_CHARGER_TYPE_DEDICATED_CHG:
		extcon_set_cable_state(muic->edev, "TA", true);
		break;
	case MAX77665_CHARGER_TYPE_500MA:
		extcon_set_cable_state(muic->edev, "Slow-charger", true);
		break;
	case MAX77665_CHARGER_TYPE_1A:
		extcon_set_cable_state(muic->edev, "Fast-charger", true);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	muic->pre_charger_type = charger_type;
out:
	return ret;
}

static void max77665_muic_irq_work(struct work_struct *work)
{
	struct max77665_muic *muic = container_of(work,
			struct max77665_muic, irq_work);
	uint8_t status[2];
	uint8_t adc, chg_type;

	int ret;

	mutex_lock(&muic->mutex);

	ret = max77665_bulk_read(muic->dev->parent, MAX77665_I2C_SLAVE_MUIC,
				MAX77665_MUIC_REG_STATUS1,
				2, status);
	if (ret) {
		dev_err(muic->dev, "failed to read muic register\n");
		mutex_unlock(&muic->mutex);
		return;
	}

	dev_dbg(muic->dev, "%s: STATUS1:0x%x, 2:0x%x\n", __func__,
			status[0], status[1]);

	adc = status[0] & STATUS1_ADC_MASK;
	adc >>= STATUS1_ADC_SHIFT;

	chg_type = status[1] & STATUS2_CHGTYP_MASK;
	chg_type >>= STATUS2_CHGTYP_SHIFT;

	max77665_muic_handle_charger_type(muic, chg_type);

	mutex_unlock(&muic->mutex);

	return;
}

static irqreturn_t max77665_muic_irq_handler(int irq, void *data)
{
	struct max77665_muic *muic = data;

	dev_dbg(muic->dev, "irq:%d\n", irq);
	muic->irq = irq;

	schedule_work(&muic->irq_work);

	return IRQ_HANDLED;
}

static void max77665_muic_detect_dev(struct max77665_muic *muic)
{
	int ret;
	uint8_t status[2], adc, chg_type;

	ret = max77665_bulk_read(muic->dev->parent, MAX77665_I2C_SLAVE_MUIC,
				MAX77665_MUIC_REG_STATUS1,
				2, status);
	if (ret) {
		dev_err(muic->dev, "failed to read muic register\n");
		return;
	}

	dev_info(muic->dev, "STATUS1:0x%x, STATUS2:0x%x\n",
			status[0], status[1]);

	adc = status[0] & STATUS1_ADC_MASK;
	adc >>= STATUS1_ADC_SHIFT;

	chg_type = status[1] & STATUS2_CHGTYP_MASK;
	chg_type >>= STATUS2_CHGTYP_SHIFT;

	max77665_muic_handle_charger_type(muic, chg_type);
}

static int __devinit max77665_muic_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct max77665_muic *muic;

	muic = devm_kzalloc(&pdev->dev, sizeof(struct max77665_muic),
			GFP_KERNEL);
	if (!muic) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	muic->dev = &pdev->dev;
	muic->pdata = pdev->dev.platform_data;
	if (!muic->pdata) {
		dev_err(&pdev->dev, "no platform data available\n");
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, muic);
	mutex_init(&muic->mutex);
	INIT_WORK(&muic->irq_work, max77665_muic_irq_work);

	if (muic->pdata->irq_base) {
		ret = request_threaded_irq(muic->pdata->irq_base +
					MAX77665_IRQ_MUIC, NULL,
					max77665_muic_irq_handler,
					0, "muic_irq",
					muic);
		if (ret) {
			dev_err(&pdev->dev,
				"failed: irq request error :%d)\n", ret);
			return ret;
		}
	}
	/* External connector */
	muic->edev = kzalloc(sizeof(struct extcon_dev),
			GFP_KERNEL);
	if (!muic->edev) {
		dev_err(&pdev->dev, "failed to allocate memory for extcon\n");
		ret = -ENOMEM;
		goto error;
	}
	muic->edev->name = DEV_NAME;
	muic->edev->supported_cable = max77665_extcon_cable;
	ret = extcon_dev_register(muic->edev, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		goto error1;
	}
	/* Initial device detection */
	max77665_muic_detect_dev(muic);

	return 0;
error1:
	kfree(muic->edev);
error:
	free_irq(muic->pdata->irq_base + MAX77665_IRQ_MUIC, muic);
	return ret;
}

static int __devexit max77665_muic_remove(struct platform_device *pdev)
{
	struct max77665_muic *muic = platform_get_drvdata(pdev);

	cancel_work_sync(&muic->irq_work);
	extcon_dev_unregister(muic->edev);

	return 0;
}

static struct platform_driver max77665_muic_driver = {
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= max77665_muic_probe,
	.remove		= __devexit_p(max77665_muic_remove),
};

module_platform_driver(max77665_muic_driver);

MODULE_DESCRIPTION("Maxim MAX77665 Extcon driver");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_LICENSE("GPL v2");
