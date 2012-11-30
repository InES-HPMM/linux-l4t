/*
 * bq2419x.c --  bq2419x
 *
 * MFD driver for BQ2419X charger.
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/mfd/bq2419x.h>
#include <linux/slab.h>

static struct mfd_cell bq2419x_children[] = {
	{.name = "bq2419x-pmic",},
	{.name = "bq2419x-battery-charger",},
};

static const struct regmap_config bq2419x_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= BQ2419X_MAX_REGS,
	.cache_type		= REGCACHE_RBTREE,
};

static int __devinit bq2419x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bq2419x_platform_data *pdata;
	struct bq2419x_chip *bq;
	int ret;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "No Platform data");
		return -EIO;
	}

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->regmap = devm_regmap_init_i2c(client, &bq2419x_regmap_config);
	if (IS_ERR(bq->regmap)) {
		ret = PTR_ERR(bq->regmap);
		dev_err(&client->dev, "regmap init failed with err %d\n", ret);
		return ret;
	}
	i2c_set_clientdata(client, bq);

	if (pdata->disable_watchdog) {
		ret = regmap_update_bits(bq->regmap, BQ2419X_WD,
					BQ2419X_WD_MASK, BQ2419X_WD_DISABLE);
		if (ret < 0) {
			dev_err(bq->dev, "register %d update failed with err %d",
				BQ2419X_WD, ret);
			return ret;
		}
	}

	ret = mfd_add_devices(bq->dev, -1, bq2419x_children,
			ARRAY_SIZE(bq2419x_children), NULL, 0);
	if (ret < 0) {
		dev_err(bq->dev,
			"registering mfd sub devices failed, err = %d\n", ret);
		return ret;
	}
	return 0;
}

static int __devexit bq2419x_remove(struct i2c_client *client)
{
	struct bq2419x_chip *bq = i2c_get_clientdata(client);

	mfd_remove_devices(bq->dev);
	return 0;
}

static const struct i2c_device_id bq2419x_id[] = {
	{.name = "bq2419x",},
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2419x_id);

static struct i2c_driver bq2419x_i2c_driver = {
	.driver = {
		.name = "bq2419x",
		.owner = THIS_MODULE,
	},
	.probe = bq2419x_probe,
	.remove = __devexit_p(bq2419x_remove),
	.id_table = bq2419x_id,
};

static int __init bq2419x_init(void)
{
	return i2c_add_driver(&bq2419x_i2c_driver);
}
subsys_initcall(bq2419x_init);

static void __exit bq2419x_cleanup(void)
{
	i2c_del_driver(&bq2419x_i2c_driver);
}
module_exit(bq2419x_cleanup);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("BQ2419X mfd core driver");
MODULE_LICENSE("GPL v2");
