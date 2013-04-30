/*
 * bq2419x-i2c-charger.c -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger driver
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

static const struct regmap_config bq2419x_regmap_config = {
	.reg_bits               = 8,
	.val_bits               = 8,
	.max_register           = BQ2419X_MAX_REGS,
};


static int bq2419x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bq2419x_chip *bq2419x;
	struct bq2419x_platform_data *pdata;
	int ret = 0;

	pdata = client->dev.platform_data;
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
	bq2419x->irq = client->irq;
	i2c_set_clientdata(client, bq2419x);
	bq2419x->use_regmap = 1;

	ret = bq2419x_hw_init(bq2419x, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "hw init failed %d\n", ret);
		return ret;
	}
	return ret;
}

static int bq2419x_remove(struct i2c_client *client)
{
	struct bq2419x_chip *bq2419x = i2c_get_clientdata(client);

	bq2419x_resource_cleanup(bq2419x);
	return 0;
}

static const struct i2c_device_id bq2419x_id[] = {
	{.name = "bq2419x",},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2419x_id);

static struct i2c_driver bq2419x_i2c_driver = {
	.driver = {
		.name   = "bq2419x-i2c",
		.owner  = THIS_MODULE,
	},
	.probe		= bq2419x_probe,
	.remove		= bq2419x_remove,
	.id_table	= bq2419x_id,
};

static inline int bq2419x_charger_i2c_init(void)
{
	return i2c_add_driver(&bq2419x_i2c_driver);
}

static inline void bq2419x_charger_i2c_exit(void)
{
	i2c_del_driver(&bq2419x_i2c_driver);
}

static int __init bq2419x_module_init(void)
{
	bq2419x_charger_i2c_init();
	return 0;
}
subsys_initcall(bq2419x_module_init);

static void __exit bq2419x_cleanup(void)
{
	bq2419x_charger_i2c_exit();
}
module_exit(bq2419x_cleanup);

MODULE_DESCRIPTION("bq2419x battery charger driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
