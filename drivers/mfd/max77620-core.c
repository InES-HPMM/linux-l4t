/*
 * Maxim MAX77620 MFD Driver
 *
 * Copyright (C) 2014 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/mfd/max77620.h>

static struct resource gpio_resources[] = {
	{
		.start	= MAX77620_IRQ_TOP_GPIO,
		.end	= MAX77620_IRQ_TOP_GPIO,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource rtc_resources[] = {
	{
		.start	= MAX77620_IRQ_TOP_RTC,
		.end	= MAX77620_IRQ_TOP_RTC,
		.flags  = IORESOURCE_IRQ,
	}
};

static const struct regmap_irq max77620_top_irqs[] = {
	[MAX77620_IRQ_TOP_GLBL] = {
		.mask = MAX77620_IRQ_TOP_GLBL_MASK,
		.reg_offset = 0,
	},
	[MAX77620_IRQ_TOP_SD] = {
		.mask = MAX77620_IRQ_TOP_SD_MASK,
		.reg_offset = 0,
	},
	[MAX77620_IRQ_TOP_LDO] = {
		.mask = MAX77620_IRQ_TOP_LDO,
		.reg_offset = 0,
	},
	[MAX77620_IRQ_TOP_GPIO] = {
		.mask = MAX77620_IRQ_TOP_GPIO,
		.reg_offset = 0
	},
	[MAX77620_IRQ_TOP_RTC] = {
		.mask = MAX77620_IRQ_TOP_RTC,
		.reg_offset = 0,
	},
	[MAX77620_IRQ_TOP_32K] = {
		.mask = MAX77620_IRQ_TOP_32K,
		.reg_offset = 0,
	},
	[MAX77620_IRQ_TOP_ONOFF] = {
		.mask = MAX77620_IRQ_TOP_ONOFF,
		.reg_offset = 0,
	},
};

enum max77660_ids {
	MAX77620_PMIC_ID,
	MAX77620_GPIO_ID,
	MAX77620_RTC_ID,
	MAX77620_PINCTRL_ID,
};

static struct mfd_cell max77620_children[] = {
	[MAX77620_GPIO_ID] = {
		.name = "max77620-gpio",
		.num_resources	= ARRAY_SIZE(gpio_resources),
		.resources	= &gpio_resources[0],
		.id = MAX77620_GPIO_ID,
	},
	[MAX77620_PMIC_ID] = {
		.name = "max77620-pmic",
		.id = MAX77620_PMIC_ID,
	},
	[MAX77620_RTC_ID] = {
		.name = "max77620-rtc",
		.num_resources	= ARRAY_SIZE(rtc_resources),
		.resources	= &rtc_resources[0],
		.id = MAX77620_RTC_ID,
	},
	[MAX77620_PINCTRL_ID] = {
		.name = "max77620-pinctrl",
		.id = MAX77620_PINCTRL_ID,
	},
};

static struct regmap_irq_chip max77620_top_irq_chip = {
	.name = "max77620-top",
	.irqs = max77620_top_irqs,
	.num_irqs = ARRAY_SIZE(max77620_top_irqs),
	.num_regs = 1,
	.irq_reg_stride = 1,
	.status_base = MAX77620_REG_IRQTOP,
	.mask_base = MAX77620_REG_IRQTOPM,
};

static int max77620_init_irqs(struct max77620_chip *chip)
{
	int ret;

	ret = regmap_add_irq_chip(chip->rmap[MAX77620_PWR_SLAVE],
		chip->chip_irq, IRQF_ONESHOT, chip->pdata->irq_base,
		&max77620_top_irq_chip, &chip->top_irq_data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add top irq_chip %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct regmap_config max77620_regmap_config[] = {
	[MAX77620_PWR_SLAVE] = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x5d,
	},
	[MAX77620_RTC_SLAVE] = {
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = 0x1b,
	},
};

static int max77620_slave_address[MAX77620_NUM_SLAVES] = {
	MAX77620_PWR_I2C_ADDR,
	MAX77620_RTC_I2C_ADDR,
};

static void max77620_dt_to_pdata(struct i2c_client *i2c,
		struct max77620_platform_data *pdata)
{
	pdata->irq_base = -1;
}

static int max77620_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max77620_chip *chip;
	int i = 0;
	int ret = 0;
	struct device_node *node = client->dev.of_node;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "Memory alloc for chip failed\n");
		return -ENOMEM;
	}

	if (node) {
		chip->pdata = devm_kzalloc(&client->dev,
					sizeof(*chip->pdata), GFP_KERNEL);
		if (!chip->pdata)
			return -ENOMEM;
		max77620_dt_to_pdata(client, chip->pdata);
	} else
		chip->pdata = client->dev.platform_data;

	if (!chip->pdata)
		return -ENODATA;

	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	chip->irq_base = chip->pdata->irq_base;
	chip->chip_irq = client->irq;

	for (i = 0; i < MAX77620_NUM_SLAVES; i++) {
		if (max77620_slave_address[i] == client->addr)
			chip->clients[i] = client;
		else
			chip->clients[i] = i2c_new_dummy(client->adapter,
						max77620_slave_address[i]);
		if (!chip->clients[i]) {
			dev_err(&client->dev, "can't attach client %d\n", i);
			ret = -ENOMEM;
			goto fail_client_reg;
		}

		i2c_set_clientdata(chip->clients[i], chip);
		chip->rmap[i] = devm_regmap_init_i2c(chip->clients[i],
					&max77620_regmap_config[i]);
		if (IS_ERR(chip->rmap[i])) {
			ret = PTR_ERR(chip->rmap[i]);
			dev_err(&client->dev,
				"regmap %d init failed, err %d\n", i, ret);
			goto fail_client_reg;
		}
	}

	ret = max77620_init_irqs(chip);
	if (ret < 0)
		goto fail_client_reg;

	if (node) {
		ret = of_platform_populate(node, NULL, NULL, &client->dev);
		if (ret < 0) {
			dev_err(&client->dev,
				"Couldn't populate max77620 devs, %d\n", ret);
			goto fail_free_irq;
		}

	} else {
		max77620_children[MAX77620_PMIC_ID].platform_data =
						chip->pdata->pmic_pdata;
		max77620_children[MAX77620_PMIC_ID].pdata_size =
					sizeof(*chip->pdata->pmic_pdata);

		max77620_children[MAX77620_RTC_ID].platform_data =
						chip->pdata->rtc_pdata;
		max77620_children[MAX77620_RTC_ID].pdata_size =
					sizeof(*chip->pdata->rtc_pdata);

		ret =  mfd_add_devices(&client->dev, -1, max77620_children,
					ARRAY_SIZE(max77620_children),
					NULL, chip->irq_base, NULL);
		if (ret < 0) {
			dev_err(&client->dev, "mfd add dev fail %d\n", ret);
			goto fail_free_irq;
		}
	}

	dev_info(&client->dev, "max77620 loaded successfully\n");
	return 0;

fail_free_irq:
	regmap_del_irq_chip(chip->chip_irq, chip->top_irq_data);
fail_client_reg:
	for (i = 1; i < MAX77620_NUM_SLAVES; i++) {
		if (chip->clients[i])
			i2c_unregister_device(chip->clients[i]);
	}
	return ret;
}

static int max77620_remove(struct i2c_client *client)
{

	struct max77620_chip *chip = i2c_get_clientdata(client);
	int i;

	mfd_remove_devices(chip->dev);
	regmap_del_irq_chip(chip->chip_irq, chip->top_irq_data);

	for (i = 0; i < MAX77620_NUM_SLAVES; i++) {
		if (chip->clients[i] != client)
			i2c_unregister_device(chip->clients[i]);
	}

	return 0;
}

static const struct i2c_device_id max77620_id[] = {
	{"max77620", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, max77620_id);

static struct i2c_driver max77620_driver = {
	.driver = {
		.name = "max77620",
		.owner = THIS_MODULE,
	},
	.probe = max77620_probe,
	.remove = max77620_remove,
	.id_table = max77620_id,
};

static int __init max77620_init(void)
{
	return i2c_add_driver(&max77620_driver);
}
subsys_initcall(max77620_init);

static void __exit max77620_exit(void)
{
	i2c_del_driver(&max77620_driver);
}
module_exit(max77620_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77620 Multi Function Device Core Driver");
MODULE_AUTHOR("Chaitanya Bandi <bandik@nvidia.com>");
MODULE_ALIAS("i2c:max77620-core");
