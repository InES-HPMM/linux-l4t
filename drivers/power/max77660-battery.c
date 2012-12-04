/*
 * Fuel gauge driver for MAX77660 PMIC
 *
 * Copyright 2012 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/mfd/max77660/max77660-battery.h>


/* max77660 fuel gauge registers */
#define MAX77660_FG_REG_VCELL       0x02    /*  MSB(0x02), LSB(0x03) */
#define MAX77660_FG_REG_SOC         0x04    /*  MSB(0x04), LSB(0x05) */
#define MAX77660_FG_REG_MODE        0x06    /*  word  */
#define MAX77660_FG_REG_VERSION     0x08    /*  word  */
#define MAX77660_FG_REG_HIBRT       0x0A    /*  word */
#define MAX77660_FG_REG_CONFIG      0x0C    /*  MSB(0x0C), LSB(0x0D) */
#define MAX77660_FG_REG_OVC         0x0E    /*  MSB(0x0E), LSB(0x0F)  */
#define MAX77660_FG_REG_ADC         0x10    /*  MSB(0x10), LSB(0x11)  */
#define MAX77660_FG_REG_CAP         0x12    /*  byte */
#define MAX77660_FG_REG_VALRT       0x14    /*  VALRTMIN(0x14), VALRTMAX(0x15) */
#define MAX77660_FG_REG_CRATE       0x16    /*  MSB(0x16), LSB(0x17) */
#define MAX77660_FG_REG_V_RESET     0x18
#define MAX77660_FG_REG_ID          0x19
#define MAX77660_FG_REG_STATUS      0x1A
#define MAX77660_FG_REG_TABLE00     0x40
#define MAX77660_FG_REG_TABLE0F     0x5E
#define MAX77660_FG_REG_TABLE10     0x60
#define MAX77660_FG_REG_TABLE1F     0x7E
#define MAX77660_FG_REG_RCOMPSeg    0x80    /*  0x80~0x9F */
#define MAX77660_FG_REG_CMD         0xFE    /*  54h means softreset, B8 means Recall EEPROM. */


#define MAX77660_FG_Quick_Start_MASK    0x4000
#define MAX77660_FG_Quick_Start_SHIFT   14
#define MAX77660_FG_EnSleep_MASK        0x2000
#define MAX77660_FG_EnSleep_SHIFT       13
#define MAX77660_FG_HibStat_MASK        0x1000
#define MAX77660_FG_HibStat_SHIFT       12

#define MAX77660_FG_

struct max77660_fg_chip {
	struct device *dev;
	struct power_supply battery;
	struct max77660_fg_platform_data *pdata;
};

static int max77660_fg_write_reg(struct max77660_fg_chip *fg, u8 reg, u16 value)
{
	int ret;

	ret = max77660_write(fg->dev, reg, &value, 2, MAX77660_I2C_FG);
	if (ret < 0)
		dev_err(fg->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max77660_fg_read_reg(struct max77660_fg_chip *fg, u8 reg, int *value)
{
	int ret;

	ret = max77660_read(fg->dev, reg, value, 2, MAX77660_I2C_FG);

	if (ret < 0)
		dev_err(fg->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static enum power_supply_property max77660_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int max77660_fg_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	int ret;
	struct max77660_fg_chip *chip = container_of(psy,
				struct max77660_fg_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	/* unit:uV, 12bit(1.25mV/1bit) */
		ret = max77660_fg_read_reg(chip, MAX77660_FG_REG_VCELL,
							&val->intval);
		val->intval = val->intval * 1250;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max77660_fg_read_reg(chip, MAX77660_FG_REG_SOC,
							&val->intval);
		val->intval = val->intval / 256;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int __devinit max77660_fg_probe(struct platform_device *pdev)
{
	struct max77660_fg_chip *chip;
	int ret;
	unsigned int data;

	chip = kzalloc(sizeof(struct max77660_fg_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	/* Set platform data */
	dev_set_drvdata(&pdev->dev, chip);
	chip->dev = &pdev->dev;

	chip->battery.name		= "max77660-battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max77660_fg_get_property;
	chip->battery.properties	= max77660_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max77660_battery_props);

	ret = power_supply_register(chip->dev, &chip->battery);
	if (ret) {
		dev_err(chip->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}

	data = chip->pdata->valrt_max<<8 | chip->pdata->valrt_min;
	max77660_fg_write_reg(chip, MAX77660_FG_REG_VALRT, data);

	return 0;
}

static int __devexit max77660_fg_remove(struct platform_device *pdev)
{
	struct max77660_fg_chip *chip = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(&chip->battery);

	kfree(chip);
	return 0;
}

static struct platform_driver max77660_fg_driver = {
	.driver	= {
		.name	= "max77660-fg",
		.owner = THIS_MODULE,
	},
	.probe		= max77660_fg_probe,
	.remove		= __devexit_p(max77660_fg_remove),
};

static int __init max77660_fg_init(void)
{
	return platform_driver_register(&max77660_fg_driver);
}
module_init(max77660_fg_init);

static void __exit max77660_fg_exit(void)
{
	platform_driver_register(&max77660_fg_driver);
}
module_exit(max77660_fg_exit);


MODULE_DESCRIPTION("MAx77660 PMIC Fuel Gauge");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Maxim Integrated");
MODULE_ALIAS("platform:max77660-battery");
