/*
 * as3722-core.c - core driver for AS3722 PMICs
 *
 * Copyright (C) 2013 ams AG
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * Author: Florian Lobmaier <florian.lobmaier@ams.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>

#define AS3722_DRIVER_VERSION  "v0.9.0"

enum as3722_ids {
	AS3722_GPIO_ID,
	AS3722_REGULATOR_ID,
	AS3722_RTC_ID,
	AS3722_ADC,
	AS3722_POWER_OFF_ID,
	AS3722_CLK_ID,
};

static const struct resource as3722_rtc_resource[] = {
	{
		.name = "as3722-rtc-alarm",
		.start = AS3722_IRQ_RTC_ALARM,
		.end = AS3722_IRQ_RTC_ALARM,
		.flags = IORESOURCE_IRQ,
	},
};

static const struct resource as3722_adc_resource[] = {
	{
		.name = "as3722-adc",
		.start = AS3722_IRQ_ADC,
		.end = AS3722_IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell as3722_devs[] = {
	{
		.name = "as3722-gpio",
		.id = AS3722_GPIO_ID,
	},
	{
		.name = "as3722-regulator",
		.id = AS3722_REGULATOR_ID,
	},
	{
		.name = "as3722-clk",
		.id = AS3722_CLK_ID,
	},
	{
		.name = "as3722-rtc",
		.num_resources = ARRAY_SIZE(as3722_rtc_resource),
		.resources = as3722_rtc_resource,
		.id = AS3722_RTC_ID,
	},
	{
		.name = "as3722-adc-extcon",
		.num_resources = ARRAY_SIZE(as3722_adc_resource),
		.resources = as3722_adc_resource,
		.id = AS3722_ADC,
	},
	{
		.name = "as3722-power-off",
		.id = AS3722_POWER_OFF_ID,
	},
};

static const struct regmap_irq as3722_irqs[] = {
	/* INT1 IRQs */
	[AS3722_IRQ_LID] = {
		.mask = AS3722_IRQ_MASK_LID,
	},
	[AS3722_IRQ_ACOK] = {
		.mask = AS3722_IRQ_MASK_ACOK,
	},
	[AS3722_IRQ_CORE_PWRREQ] = {
		.mask = AS3722_IRQ_MASK_CORE_PWRREQ,
	},
	[AS3722_IRQ_SD0] = {
		.mask = AS3722_IRQ_MASK_SD0,
	},
	[AS3722_IRQ_ONKEY_LONG] = {
		.mask = AS3722_IRQ_MASK_ONKEY_LONG,
	},
	[AS3722_IRQ_ONKEY] = {
		.mask = AS3722_IRQ_MASK_ONKEY,
	},
	[AS3722_IRQ_OVTMP] = {
		.mask = AS3722_IRQ_MASK_OVTMP,
	},
	[AS3722_IRQ_LOWBAT] = {
		.mask = AS3722_IRQ_MASK_LOWBAT,
	},
	/* INT2 IRQs */
	[AS3722_IRQ_RTC_REP] = {
		.mask = AS3722_IRQ_MASK_RTC_REP,
		.reg_offset = 1,
	},
	/* INT3 IRQs */
	[AS3722_IRQ_RTC_ALARM] = {
		.mask = AS3722_IRQ_MASK_RTC_ALARM,
		.reg_offset = 2,
	},
	[AS3722_IRQ_RTC_GPIO1] = {
		.mask = AS3722_IRQ_MASK_GPIO_EDGE1,
		.type_rising_mask = AS3722_IRQ_MASK_GPIO_EDGE1,
		.type_falling_mask = AS3722_IRQ_MASK_GPIO_EDGE1,
		.reg_offset = 2,
	},
	[AS3722_IRQ_RTC_GPIO2] = {
		.mask = AS3722_IRQ_MASK_GPIO_EDGE2,
		.type_rising_mask = AS3722_IRQ_MASK_GPIO_EDGE2,
		.type_falling_mask = AS3722_IRQ_MASK_GPIO_EDGE2,
		.reg_offset = 2,
	},
	[AS3722_IRQ_RTC_GPIO3] = {
		.mask = AS3722_IRQ_MASK_GPIO_EDGE3,
		.type_rising_mask = AS3722_IRQ_MASK_GPIO_EDGE3,
		.type_falling_mask = AS3722_IRQ_MASK_GPIO_EDGE3,
		.reg_offset = 2,
	},
	[AS3722_IRQ_RTC_GPIO4] = {
		.mask = AS3722_IRQ_MASK_GPIO_EDGE4,
		.type_rising_mask = AS3722_IRQ_MASK_GPIO_EDGE4,
		.type_falling_mask = AS3722_IRQ_MASK_GPIO_EDGE4,
		.reg_offset = 2,
	},
	[AS3722_IRQ_RTC_GPIO5] = {
		.mask = AS3722_IRQ_MASK_GPIO_EDGE5,
		.type_rising_mask = AS3722_IRQ_MASK_GPIO_EDGE5,
		.type_falling_mask = AS3722_IRQ_MASK_GPIO_EDGE5,
		.reg_offset = 2,
	},
	[AS3722_IRQ_WATCHDOG] = {
		.mask = AS3722_IRQ_MASK_WATCHDOG,
		.reg_offset = 2,
	},
	[AS3722_IRQ_ADC] = {
		.mask = AS3722_IRQ_MASK_ADC,
		.reg_offset = 3,
	},
};

static struct regmap_irq_chip as3722_irq_chip = {
	.name = "as3722",
	.irqs = as3722_irqs,
	.num_irqs = ARRAY_SIZE(as3722_irqs),
	.num_regs = 4,
	.status_base = AS3722_INTERRUPTSTATUS1_REG,
	.mask_base = AS3722_INTERRUPTMASK1_REG,
};

static void as3722_reg_init(struct as3722 *as3722,
		struct as3722_reg_init *reg_data) {
	int ret;

	while (reg_data->reg != AS3722_REG_INIT_TERMINATE) {
		ret = as3722_reg_write(as3722, reg_data->reg, reg_data->val);
		if (ret) {
			dev_err(as3722->dev,
					"reg setup failed: %d\n", ret);
			return;
		}
		reg_data++;
	}
}

int as3722_read_adc(struct as3722 *as3722,
		enum as3722_adc_channel channel,
		enum as3722_adc_source source) {
	/*FIXME: not implemented yet*/
	return 0;
}
EXPORT_SYMBOL_GPL(as3722_read_adc);

static int as3722_init(struct as3722 *as3722,
		struct as3722_platform_data *pdata, int irq) {
	u32 reg;
	int ret;

	/* Check that this is actually a AS3722 */
	ret = regmap_read(as3722->regmap, AS3722_ADDR_ASIC_ID1, &reg);
	if (ret != 0) {
		dev_err(as3722->dev,
				"Chip ID register read failed\n");
		return -EIO;
	}
	if (reg != AS3722_DEVICE_ID) {
		dev_err(as3722->dev,
				"Device is not an AS3722, ID is 0x%x\n"
				, reg);
		return -ENODEV;
	}

	ret = regmap_read(as3722->regmap, AS3722_ADDR_ASIC_ID2, &reg);
	if (ret < 0) {
		dev_err(as3722->dev,
				"ID2 register read failed: %d\n", ret);
		return ret;
	}
	dev_info(as3722->dev, "AS3722 with revision %x found\n",
			reg);

	/* do some initial platform register setup */
	if (pdata->core_init_data)
		as3722_reg_init(as3722, pdata->core_init_data);

	/* enable pullups if required */
	if (pdata->use_internal_int_pullup == 1)
		as3722_set_bits(as3722, AS3722_IOVOLTAGE_REG,
				AS3722_INT_PULLUP_MASK, AS3722_INT_PULLUP_ON);
	else
		as3722_set_bits(as3722, AS3722_IOVOLTAGE_REG,
				AS3722_INT_PULLUP_MASK, AS3722_INT_PULLUP_OFF);
	if (pdata->use_internal_i2c_pullup == 1)
		as3722_set_bits(as3722, AS3722_IOVOLTAGE_REG,
				AS3722_I2C_PULLUP_MASK, AS3722_I2C_PULLUP_ON);
	else
		as3722_set_bits(as3722, AS3722_IOVOLTAGE_REG,
				AS3722_I2C_PULLUP_MASK, AS3722_I2C_PULLUP_OFF);

	return 0;
}

static bool as3722_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x06:
	case 0x08 ... 0x17:
	case 0x19 ... 0x24:
	case 0x27 ... 0x42:
	case 0x46:
	case 0x48 ... 0x4A:
	case 0x4D ... 0x55:
	case 0x57 ... 0x6D:
	case 0x73 ... 0x7D:
	case 0x80 ... 0x8A:
	case 0x90 ... 0x91:
	case 0xA7 ... 0xF3:
		return true;
	default:
		return false;
	}
}

static bool as3722_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x06:
	case 0x08 ... 0x17:
	case 0x19 ... 0x20:
	case 0x22 ... 0x24:
	case 0x27 ... 0x42:
	case 0x46:
	case 0x48 ... 0x4A:
	case 0x4D ... 0x55:
	case 0x57 ... 0x6D:
	case 0x74 ... 0x7D:
	case 0x80 ... 0x81:
	case 0x86 ... 0x8A:
	case 0xA7 ... 0xF3:
		return true;
	default:
		return false;
	}
}

static bool as3722_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AS3722_SD0_VOLTAGE_REG ... AS3722_LDO11_VOLTAGE_REG:
	case AS3722_SD_CONTROL_REG ... AS3722_LDOCONTROL1_REG:
		return false;
	default:
		return true;
	}
}

const struct regmap_config as3722_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_RBTREE,

	.max_register = AS3722_MAX_REGISTER,
	.readable_reg = as3722_readable,
	.writeable_reg = as3722_writeable,
	.volatile_reg = as3722_volatile,
};

static int as3722_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id) {
	struct as3722 *as3722;
	struct as3722_platform_data *pdata;
	int irq_flags;
	int ret;

	pdata = dev_get_platdata(&i2c->dev);
	if (!pdata) {
		dev_err(&i2c->dev, "as3722 requires platform data\n");
		return -EINVAL;
	}

	as3722 = devm_kzalloc(&i2c->dev, sizeof(struct as3722), GFP_KERNEL);
	if (as3722 == NULL) {
		dev_err(&i2c->dev, "mem alloc for as3722 failed\n");
		return -ENOMEM;
	}

	as3722->dev = &i2c->dev;
	as3722->chip_irq = i2c->irq;
	i2c_set_clientdata(i2c, as3722);

	as3722->regmap = devm_regmap_init_i2c(i2c, &as3722_regmap_config);
	if (IS_ERR(as3722->regmap)) {
		ret = PTR_ERR(as3722->regmap);
		dev_err(&i2c->dev, "regmap_init failed with err: %d\n", ret);
		return ret;
	}

	irq_flags = pdata->irq_type;
	irq_flags |= IRQF_ONESHOT;
	ret = regmap_add_irq_chip(as3722->regmap, as3722->chip_irq,
			irq_flags, pdata->irq_base, &as3722_irq_chip,
			&as3722->irq_data);
	if (ret < 0) {
		dev_err(as3722->dev,
				"irq allocation failed for as3722 failed\n");
		return ret;
	}

	ret = as3722_init(as3722, pdata, i2c->irq);
	if (ret < 0)
		return ret;

	ret = mfd_add_devices(&i2c->dev, -1, as3722_devs,
			ARRAY_SIZE(as3722_devs), NULL,
			pdata->irq_base,
			regmap_irq_get_domain(as3722->irq_data));
	if (ret) {
		dev_err(as3722->dev, "add mfd devices failed with err: %d\n",
				ret);
		return ret;
	}

	dev_info(as3722->dev,
			"AS3722 core driver %s initialized successfully\n",
			AS3722_DRIVER_VERSION);

	return 0;
}

static int as3722_i2c_remove(struct i2c_client *i2c)
{
	struct as3722 *as3722 = i2c_get_clientdata(i2c);

	mfd_remove_devices(as3722->dev);
	regmap_del_irq_chip(as3722->chip_irq, as3722->irq_data);

	return 0;
}

static const struct i2c_device_id as3722_i2c_id[] = {
	{"as3722", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, as3722_i2c_id);

static struct i2c_driver as3722_i2c_driver = {
	.driver = {
		.name = "as3722",
		.owner = THIS_MODULE,
	},
	.probe = as3722_i2c_probe,
	.remove = as3722_i2c_remove,
	.id_table = as3722_i2c_id,
};

static int __init as3722_i2c_init(void)
{
	return i2c_add_driver(&as3722_i2c_driver);
}

subsys_initcall(as3722_i2c_init);

static void __exit as3722_i2c_exit(void)
{
	i2c_del_driver(&as3722_i2c_driver);
}

module_exit(as3722_i2c_exit);

MODULE_DESCRIPTION("I2C support for AS3722 PMICs");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");
