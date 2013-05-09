/*
 * bq2419x-platform-charger.c -- BQ24190/BQ24192/BQ24192i/BQ24193 Charger driver
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

static int bq2419x_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bq2419x_chip *bq2419x;
	struct bq2419x_platform_data *pdata;
	struct palmas_platform_data *palmas_pdata;

	palmas_pdata = dev_get_platdata(pdev->dev.parent);
	if (!palmas_pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -ENODEV;
	}

	bq2419x = devm_kzalloc(&pdev->dev, sizeof(*bq2419x), GFP_KERNEL);
	if (!bq2419x) {
		dev_err(&pdev->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	pdata = palmas_pdata->charger_pdata;
	bq2419x->dev = &pdev->dev;
	bq2419x->irq = platform_get_irq(pdev, 0);
	bq2419x->palmas = dev_get_drvdata(pdev->dev.parent);
	bq2419x->use_regmap     = 0;

	if (!pdata->bcharger_pdata) {
		dev_err(&pdev->dev, "No battery charger platform data\n");
		return -ENODEV;
	}

	if (!pdata->bcharger_pdata->is_battery_present) {
		dev_err(&pdev->dev, "Battery not detected! Exiting driver...\n");
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, bq2419x);

	ret = bq2419x_hw_init(bq2419x, pdata);
	if (ret < 0) {
		dev_err(bq2419x->dev, "hw init failed %d\n", ret);
		return ret;
	}
	return ret;
}

static int bq2419x_remove(struct platform_device *pdev)
{
	struct bq2419x_chip *bq2419x = dev_get_drvdata(&pdev->dev);
	bq2419x_resource_cleanup(bq2419x);
	return 0;
}

static const struct platform_device_id bq2419x_platform_id[] = {
	{ "palmas-charger", PALMAS_CHARGER},
	{ /* end */ }
};

static struct platform_driver bq2419x_platform_driver = {
	.probe  = bq2419x_probe,
	.remove = bq2419x_remove,
	.id_table = bq2419x_platform_id,
	.driver = {
		.name = "bq2419x-platform",
		.owner = THIS_MODULE,
	},
};

static inline int bq2419x_charger_platform_init(void)
{
	return platform_driver_register(&bq2419x_platform_driver);
}

static inline void bq2419x_charger_platform_exit(void)
{
	return platform_driver_unregister(&bq2419x_platform_driver);
}

static int __init bq2419x_module_init(void)
{
	bq2419x_charger_platform_init();
	return 0;
}
subsys_initcall(bq2419x_module_init);

static void __exit bq2419x_cleanup(void)
{
	bq2419x_charger_platform_exit();
}
module_exit(bq2419x_cleanup);

MODULE_DESCRIPTION("bq2419x battery charger driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
