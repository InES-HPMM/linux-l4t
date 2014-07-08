/*
 * Watchdog timer for Max77620 PMIC.
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * Author: Chaitanya Bandi <bandik@nvidia.com>
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/max77620.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/watchdog.h>
#include <linux/workqueue.h>

static bool nowayout = WATCHDOG_NOWAYOUT;

struct max77620_wdt {
	struct watchdog_device		wdt_dev;
	struct device			*dev;
	struct max77620_chip		*chip;

	struct delayed_work		wdt_restart_wq;
	int				timeout;
	int				clear_time;
	bool				otp_wdtt;
	bool				otp_wdten;
};

static int max77620_wdt_start(struct watchdog_device *wdt_dev)
{
	struct max77620_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret;

	ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
			MAX77620_REG_CNFGGLBL2, MAX77620_WDTEN,
						MAX77620_WDTEN);
	if (ret < 0) {
		dev_err(wdt->dev, "clear wdten failed %d\n", ret);
		return ret;
	}

	schedule_delayed_work(&wdt->wdt_restart_wq, wdt->clear_time * HZ);
	return 0;
}

static int max77620_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct max77620_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret;

	if (wdt->otp_wdten == 0) {
		ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
			MAX77620_REG_CNFGGLBL2, MAX77620_WDTEN, 0);
		if (ret < 0) {
			dev_err(wdt->dev, "clear wdten failed %d\n", ret);
			return ret;
		}
		cancel_delayed_work(&wdt->wdt_restart_wq);
	} else {
		dev_err(wdt->dev, "Can't clear WDTEN as OTP_WDTEN=1\n");
		return -EPERM;
	}
	return 0;
}

static int max77620_wdt_set_timeout(struct watchdog_device *wdt_dev,
		unsigned int timeout)
{
	struct max77620_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret;
	u8 regval = 0;
	u8 val = 0;

	switch (timeout) {
	case 2:
		regval = MAX77620_TWD_2s;
		break;
	case 16:
		regval = MAX77620_TWD_16s;
		break;
	case 64:
		regval = MAX77620_TWD_64s;
		break;
	case 128:
		regval = MAX77620_TWD_128s;
		break;
	}

	if (wdt->otp_wdtt) {
		/* if OTP_WDTT = 1, TWD can be changed when WDTEN = 0*/
		if (wdt->otp_wdten == 0) {
			/* Set WDTEN = 0 and change TWD*/
			ret = max77620_reg_read(wdt->chip->dev,
				MAX77620_PWR_SLAVE,
				MAX77620_REG_CNFGGLBL2, &val);

			ret = max77620_reg_update(wdt->chip->dev,
				MAX77620_PWR_SLAVE,
				MAX77620_REG_CNFGGLBL2, MAX77620_WDTEN, 0);
			if (ret < 0) {
				dev_err(wdt->dev,
					"clear wdten failed: %d\n", ret);
				return ret;
			}

			ret = max77620_reg_update(wdt->chip->dev,
				MAX77620_PWR_SLAVE,
				MAX77620_REG_CNFGGLBL2,
				MAX77620_TWD_MASK, regval);
			if (ret < 0) {
				dev_err(wdt->dev,
					"set wdt timer failed: %d\n", ret);
				return ret;
			}

			if (val & MAX77620_WDTEN) {
				ret = max77620_reg_update(wdt->chip->dev,
					MAX77620_PWR_SLAVE,
					MAX77620_REG_CNFGGLBL2,
					MAX77620_WDTEN, MAX77620_WDTEN);
				if (ret < 0) {
					dev_err(wdt->dev,
						"set wdten failed: %d\n", ret);
					return ret;
				}
			}

		} else {
			ret = max77620_reg_read(wdt->chip->dev,
				MAX77620_PWR_SLAVE,
				MAX77620_REG_CNFGGLBL2, &val);
			if (val & MAX77620_WDTEN) {
				dev_err(wdt->dev,
					"WDTEN is 1. Cannot update timer\n");
				return -EPERM;
			} else {
				ret = max77620_reg_update(wdt->chip->dev,
					MAX77620_PWR_SLAVE,
					MAX77620_REG_CNFGGLBL2,
					MAX77620_TWD_MASK, regval);
				if (ret < 0) {
					dev_err(wdt->dev,
					"set wdt timer failed: %d\n", ret);
					return ret;
				}
			}
		}
	} else {
		/*OTP_WDTT = 0, TWD can be changed by clearing the WDT first*/
		ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
			MAX77620_REG_CNFGGLBL3, MAX77620_WDTC_MASK, 0x1);
		if (ret < 0) {
			dev_err(wdt->dev, "clear wdt failed: %d\n", ret);
			return ret;
		}
		ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
			MAX77620_REG_CNFGGLBL2, MAX77620_TWD_MASK, regval);
		if (ret < 0) {
			dev_err(wdt->dev, "set wdt timer failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static const struct watchdog_info max77620_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.identity = "max77620 Watchdog",
};

static const struct watchdog_ops max77620_wdt_ops = {
	.owner = THIS_MODULE,
	.start = max77620_wdt_start,
	.stop = max77620_wdt_stop,
	.set_timeout = max77620_wdt_set_timeout,
};

static void max77620_wdt_restart_wq(struct work_struct *work)
{
	struct max77620_wdt *wdt = container_of(work, struct max77620_wdt,
						wdt_restart_wq.work);
	int ret;

	ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
		MAX77620_REG_CNFGGLBL3, MAX77620_WDTC_MASK, 0x1);
	if (ret < 0)
		dev_err(wdt->dev, "clear wdt failed: %d\n", ret);

	dev_info(wdt->dev, "wdt cleared\n");

	schedule_delayed_work(&wdt->wdt_restart_wq, wdt->clear_time * HZ);
}

static int max77620_wdt_probe(struct platform_device *pdev)
{
	struct max77620_wdt *wdt;
	struct watchdog_device *wdt_dev;
	int ret;
	u32 prop;
	struct device_node *np = pdev->dev.parent->of_node;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		dev_err(&pdev->dev, "Failed to allocate mem\n");
		return -ENOMEM;
	}

	if (np) {
		ret =	of_property_read_u32(np, "maxim,wdt-timeout", &prop);
		if (!ret)
			wdt->timeout = prop;
		ret =	of_property_read_u32(np, "maxim,wdt-clear-time", &prop);
		if (!ret)
			wdt->clear_time = prop;

		wdt->otp_wdtt = of_property_read_bool(np, "maxim,otp-wdtt");
		wdt->otp_wdten = of_property_read_bool(np, "maxim,otp-wdten");
	} else {
		wdt->timeout = 64;
		wdt->clear_time = 60;
		wdt->otp_wdtt = 0;
		wdt->otp_wdten = 0;
	}

	wdt->dev = &pdev->dev;
	wdt_dev = &wdt->wdt_dev;
	wdt->chip = dev_get_drvdata(pdev->dev.parent);
	wdt_dev->info = &max77620_wdt_info;
	wdt_dev->ops = &max77620_wdt_ops;
	wdt_dev->min_timeout = 2;
	wdt_dev->max_timeout = 128;

	watchdog_set_nowayout(wdt_dev, nowayout);
	watchdog_set_drvdata(wdt_dev, wdt);
	platform_set_drvdata(pdev, wdt);

	ret = watchdog_register_device(wdt_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "watchdog registration failed: %d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&wdt->wdt_restart_wq, max77620_wdt_restart_wq);

	/*Enable WD_RST_WK - WDT expire results in a restart*/
	ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
		MAX77620_REG_ONOFFCNFG2, MAX77620_ONOFFCNFG2_WD_RST_WK,
					MAX77620_ONOFFCNFG2_WD_RST_WK);
	if (ret < 0)
		dev_err(wdt->dev, "onoffcnfg2 failed: %d\n", ret);

	/*Set WDT clear in OFF mode*/
	ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
			MAX77620_REG_CNFGGLBL2, MAX77620_WDTOFFC,
						MAX77620_WDTOFFC);
	if (ret < 0)
		dev_err(wdt->dev, "set wdtoffc failed %d\n", ret);

	/*Set WDT clear in Sleep mode*/
	ret = max77620_reg_update(wdt->chip->dev, MAX77620_PWR_SLAVE,
			MAX77620_REG_CNFGGLBL2, MAX77620_WDTSLPC,
						MAX77620_WDTSLPC);
	if (ret < 0)
		dev_err(wdt->dev, "set wdtslpc failed %d\n", ret);

	ret = max77620_wdt_set_timeout(wdt_dev, wdt->timeout);
	if (ret < 0) {
		dev_err(wdt->dev, "wdt set timeout failed: %d\n", ret);
		goto scrub;
	}

	ret = max77620_wdt_start(wdt_dev);
	if (ret < 0) {
		dev_err(wdt->dev, "wdt start failed: %d\n", ret);
		goto scrub;
	}

	return 0;
scrub:
	cancel_delayed_work(&wdt->wdt_restart_wq);
	watchdog_unregister_device(&wdt->wdt_dev);
	return ret;
}

static int max77620_wdt_remove(struct platform_device *pdev)
{
	struct max77620_wdt *wdt = platform_get_drvdata(pdev);

	max77620_wdt_stop(&wdt->wdt_dev);
	cancel_delayed_work(&wdt->wdt_restart_wq);
	watchdog_unregister_device(&wdt->wdt_dev);
	return 0;
}

static int max77620_wdt_suspend(struct device *dev)
{
	struct max77620_wdt *wdt = dev_get_drvdata(dev);
	int ret;

	ret = max77620_wdt_stop(&wdt->wdt_dev);
	if (ret < 0)
		dev_err(wdt->dev, "wdt stop failed: %d\n", ret);
	return 0;
}

static int max77620_wdt_resume(struct device *dev)
{
	struct max77620_wdt *wdt = dev_get_drvdata(dev);
	int ret;

	ret = max77620_wdt_start(&wdt->wdt_dev);
	if (ret < 0)
		dev_err(wdt->dev, "wdt start failed: %d\n", ret);
	return 0;
}

static const struct dev_pm_ops max77620_wdt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77620_wdt_suspend, max77620_wdt_resume)
};

static struct platform_driver max77620_wdt_driver = {
	.driver	= {
		.name	= "max77620-wdt",
		.owner	= THIS_MODULE,
		.pm = &max77620_wdt_pm_ops,
	},
	.probe	= max77620_wdt_probe,
	.remove	= max77620_wdt_remove,
};

static int __init max77620_wdt_init(void)
{
	return platform_driver_register(&max77620_wdt_driver);
}
subsys_initcall(max77620_wdt_init);

static void __exit max77620_wdt_exit(void)
{
	platform_driver_unregister(&max77620_wdt_driver);
}
module_exit(max77620_wdt_exit);

MODULE_ALIAS("platform:max77620-wdt");
MODULE_DESCRIPTION("Max77620 watchdog timer driver");
MODULE_AUTHOR("Chaitanya Bandi <bandik@nvidia.com>");
MODULE_LICENSE("GPL v2");
