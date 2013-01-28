/*
 * max77660_sys_wdt.c -- MAX77660 System WatchDog Timer.
 *
 * System watchdog timer for MAXIM MAX77660 PMIC.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/max77660/max77660-core.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/watchdog.h>

static bool nowayout = WATCHDOG_NOWAYOUT;

struct max77660_sys_wdt {
	struct watchdog_device wdt_dev;
	struct device *dev;
	struct device *parent;
	int timeout;
	int irq;
};

static int max77660_twd_sys[] = {16, 32, 64, 128};

static irqreturn_t max77660_sys_wdt_irq(int irq, void *data)
{
	struct max77660_sys_wdt *wdt = data;
	int ret;

	/* Reset timer before any debug prints */
	ret = max77660_reg_write(wdt->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG4,
			MAX77660_GLBLCNFG4_WDTC_SYS_CLR);
	if (ret < 0)
		dev_err(wdt->dev, "GLOBAL_CFG4 update failed: %d\n", ret);

	dev_info(wdt->dev, "System WDT interrupt occur\n");
	return IRQ_HANDLED;
}

static int max77660_sys_wdt_start(struct watchdog_device *wdt_dev)
{
	struct max77660_sys_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret;

	ret = max77660_reg_set_bits(wdt->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1, MAX77660_GLBLCNFG1_WDTEN_SYS);
	if (ret < 0) {
		dev_err(wdt->dev, "GLOBAL_CFG1 update failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static int max77660_sys_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct max77660_sys_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int ret;

	ret = max77660_reg_clr_bits(wdt->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1, MAX77660_GLBLCNFG1_WDTEN_SYS);
	if (ret < 0) {
		dev_err(wdt->dev, "GLOBAL_CFG1 update failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static int max77660_sys_wdt_set_timeout(struct watchdog_device *wdt_dev,
		unsigned int timeout)
{
	struct max77660_sys_wdt *wdt = watchdog_get_drvdata(wdt_dev);
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(max77660_twd_sys); ++i) {
		if (max77660_twd_sys[i] >= timeout)
			break;
	}

	if (i == ARRAY_SIZE(max77660_twd_sys)) {
		dev_err(wdt->dev, "Not a valid timeout: %u\n", timeout);
		return -EINVAL;
	}

	ret = max77660_reg_update(wdt->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG2,
			MAX77660_GLBLCNFG2_TWD_SYS_MASK,
			MAX77660_GLBLCNFG2_TWD_SYS(i));
	if (ret < 0) {
		dev_err(wdt->dev, "GLOBAL_CFG2 update failed: %d\n", ret);
		return ret;
	}
	wdt->timeout = timeout;
	return 0;
}

static const struct watchdog_info max77660_sys_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
	.identity = "MAX77660 System Watchdog",
};

static const struct watchdog_ops max77660_sys_wdt_ops = {
	.owner = THIS_MODULE,
	.start = max77660_sys_wdt_start,
	.stop = max77660_sys_wdt_stop,
	.set_timeout = max77660_sys_wdt_set_timeout,
};

static int __devinit max77660_sys_wdt_probe(struct platform_device *pdev)
{
	struct max77660_platform_data *pdata;
	struct max77660_sys_wdt *wdt;
	struct watchdog_device *wdt_dev;
	uint8_t regval;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	pdata = dev_get_platdata(pdev->dev.parent);

	wdt->dev = &pdev->dev;
	wdt->irq = platform_get_irq(pdev, 0);
	wdt->parent = pdev->dev.parent;
	wdt_dev = &wdt->wdt_dev;

	wdt_dev->info = &max77660_sys_wdt_info;
	wdt_dev->ops = &max77660_sys_wdt_ops;
	wdt_dev->timeout = 128;
	wdt_dev->min_timeout = 16;
	wdt_dev->max_timeout = 128;
	watchdog_set_nowayout(wdt_dev, nowayout);
	watchdog_set_drvdata(wdt_dev, wdt);
	platform_set_drvdata(pdev, wdt);

	ret = request_threaded_irq(wdt->irq, NULL, max77660_sys_wdt_irq,
			IRQF_ONESHOT | IRQF_EARLY_RESUME,
			dev_name(&pdev->dev), wdt);
	if (ret < 0) {
		dev_err(&pdev->dev, "request IRQ:%d failed, err = %d\n",
			 wdt->irq, ret);
		return ret;
	}

	ret = watchdog_register_device(wdt_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "watchdog registration failed: %d\n", ret);
		return ret;
	}

	ret = max77660_reg_read(wdt->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_GLOBAL_CFG1, &regval);
	if (ret < 0) {
		dev_err(wdt->dev, "GLOBAL_CFG1 read failed: %d\n", ret);
		goto scrub;
	}

	if (regval & MAX77660_GLBLCNFG1_WDTEN_SYS)
		dev_info(wdt->dev, "System watchdog timer enabled\n");
	else
		dev_info(wdt->dev, "System watchdog timer disabled\n");

	ret = max77660_sys_wdt_stop(wdt_dev);
	if (ret < 0) {
		dev_err(wdt->dev, "wdt stop failed: %d\n", ret);
		goto scrub;
	}

	if (pdata && (pdata->system_watchdog_timeout > 0)) {
		ret = max77660_sys_wdt_set_timeout(wdt_dev,
					pdata->system_watchdog_timeout);
		if (ret < 0) {
			dev_err(wdt->dev, "wdt set timeout failed: %d\n", ret);
			goto scrub;
		}
		ret = max77660_sys_wdt_start(wdt_dev);
		if (ret < 0) {
			dev_err(wdt->dev, "wdt start failed: %d\n", ret);
			goto scrub;
		}
	}

	return 0;
scrub:
	free_irq(wdt->irq, wdt);
	watchdog_unregister_device(&wdt->wdt_dev);
	return ret;
}

static int __devexit max77660_sys_wdt_remove(struct platform_device *pdev)
{
	struct max77660_sys_wdt *wdt = platform_get_drvdata(pdev);

	max77660_sys_wdt_stop(&wdt->wdt_dev);
	watchdog_unregister_device(&wdt->wdt_dev);
	free_irq(wdt->irq, wdt);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77660_sys_wdt_suspend(struct device *dev)
{
	struct max77660_sys_wdt *wdt = dev_get_drvdata(dev);
	int ret;

	if (device_may_wakeup(dev)) {
		enable_irq_wake(wdt->irq);
	} else if (wdt->timeout > 0) {
		ret = max77660_sys_wdt_stop(&wdt->wdt_dev);
		if (ret < 0)
			dev_err(wdt->dev, "wdt stop failed: %d\n", ret);
	}
	return 0;
}

static int max77660_sys_wdt_resume(struct device *dev)
{
	struct max77660_sys_wdt *wdt = dev_get_drvdata(dev);
	int ret;

	if (device_may_wakeup(dev)) {
		disable_irq_wake(wdt->irq);
	} else if (wdt->timeout > 0) {
		ret = max77660_sys_wdt_start(&wdt->wdt_dev);
		if (ret < 0)
			dev_err(wdt->dev, "wdt start failed: %d\n", ret);
	}
	return 0;
}
#endif

static const struct dev_pm_ops max77660_sys_wdt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(max77660_sys_wdt_suspend, max77660_sys_wdt_resume)
};

static struct platform_driver max77660_sys_wdt_driver = {
	.driver	= {
		.name	= "max77660-sys-wdt",
		.owner	= THIS_MODULE,
		.pm = &max77660_sys_wdt_pm_ops,
	},
	.probe	= max77660_sys_wdt_probe,
	.remove	= __devexit_p(max77660_sys_wdt_remove),
};

module_platform_driver(max77660_sys_wdt_driver);

MODULE_ALIAS("platform:max77660-sys-wdt");
MODULE_DESCRIPTION("Maxim Max77660 system watchdog timer driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
