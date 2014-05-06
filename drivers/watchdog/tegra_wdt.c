/*
 * drivers/watchdog/tegra_wdt.c
 *
 * watchdog driver for NVIDIA tegra internal watchdog
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * based on drivers/watchdog/softdog.c and drivers/watchdog/omap_wdt.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>

/* minimum and maximum watchdog trigger periods, in seconds */
#define MIN_WDT_PERIOD	5
#define MAX_WDT_PERIOD	1000

enum tegra_wdt_status {
	WDT_DISABLED = 1 << 0,
	WDT_ENABLED = 1 << 1,
};

struct tegra_wdt {
	struct watchdog_device	wdt;
	struct resource		*res_src;
	struct resource		*res_wdt;
	unsigned long		users;
	void __iomem		*wdt_source;
	void __iomem		*wdt_timer;
	int			tmrsrc;
	int			status;
};

/*
 * For spinlock lockup detection to work, the heartbeat should be 2*lockup
 * for cases where the spinlock disabled irqs.
 */
static int heartbeat = 120; /* must be greater than MIN_WDT_PERIOD and lower than MAX_WDT_PERIOD */
#ifdef CONFIG_TEGRA_WATCHDOG_ENABLE_ON_PROBE
static bool enable_on_probe = true;
#else
static bool enable_on_probe;
#endif

static inline struct tegra_wdt *to_tegra_wdt(struct watchdog_device *wdt)
{
	return container_of(wdt, struct tegra_wdt, wdt);
}

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)

#define TIMER_PTV		0x0
 #define TIMER_EN		(1 << 31)
 #define TIMER_PERIODIC		(1 << 30)
#define TIMER_PCR		0x4
 #define TIMER_PCR_INTR		(1 << 30)
#define WDT_EN			(1 << 5)
#define WDT_SEL_TMR1		(0 << 4)
#define WDT_SYS_RST		(1 << 2)

static int __tegra_wdt_enable(struct tegra_wdt *tegra_wdt)
{
	u32 val;

	/* since the watchdog reset occurs when a second interrupt
	 * is asserted before the first is processed, program the
	 * timer period to one-half of the watchdog period */
	val = tegra_wdt->timeout * 1000000ul / 2;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, tegra_wdt->wdt_timer + TIMER_PTV);

	val = WDT_EN | WDT_SEL_TMR1 | WDT_SYS_RST;
	writel(val, tegra_wdt->wdt_source);

	return 0;
}

static int __tegra_wdt_disable(struct tegra_wdt *tegra_wdt)
{
	writel(0, tegra_wdt->wdt_source);
	writel(0, tegra_wdt->wdt_timer + TIMER_PTV);

	return 0;
}

static int __tegra_wdt_ping(struct tegra_wdt *tegra_wdt)
{
	writel(TIMER_PCR_INTR, tegra_wdt->wdt_timer + TIMER_PCR);
	return 0;
}

#elif defined(CONFIG_ARCH_TEGRA_3x_SOC) || defined(CONFIG_ARCH_TEGRA_11x_SOC) \
	|| defined(CONFIG_ARCH_TEGRA_12x_SOC)

#define TIMER_PTV			0
 #define TIMER_EN			(1 << 31)
 #define TIMER_PERIODIC			(1 << 30)
#define TIMER_PCR			0x4
 #define TIMER_PCR_INTR			(1 << 30)
#define WDT_CFG				(0)
 #define WDT_CFG_PERIOD			(1 << 4)
 #define WDT_CFG_INT_EN			(1 << 12)
 #define WDT_CFG_SYS_RST_EN		(1 << 14)
 #define WDT_CFG_PMC2CAR_RST_EN		(1 << 15)
#define WDT_STATUS			(4)
 #define WDT_INTR_STAT			(1 << 1)
#define WDT_CMD				(8)
 #define WDT_CMD_START_COUNTER		(1 << 0)
 #define WDT_CMD_DISABLE_COUNTER	(1 << 1)
#define WDT_UNLOCK			(0xC)
 #define WDT_UNLOCK_PATTERN		(0xC45A << 0)
#define MAX_NR_CPU_WDT			0x4

static int __tegra_wdt_ping(struct tegra_wdt *tegra_wdt)
{
	writel(WDT_CMD_START_COUNTER, tegra_wdt->wdt_source + WDT_CMD);
	return 0;
}

static int __tegra_wdt_enable(struct tegra_wdt *tegra_wdt)
{
	u32 val;

	writel(TIMER_PCR_INTR, tegra_wdt->wdt_timer + TIMER_PCR);
	val = (tegra_wdt->wdt.timeout * USEC_PER_SEC) / 4;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, tegra_wdt->wdt_timer + TIMER_PTV);

	val = tegra_wdt->tmrsrc | WDT_CFG_PERIOD | WDT_CFG_PMC2CAR_RST_EN;
	writel(val, tegra_wdt->wdt_source + WDT_CFG);
	writel(WDT_CMD_START_COUNTER, tegra_wdt->wdt_source + WDT_CMD);

	return 0;
}

static int __tegra_wdt_disable(struct tegra_wdt *tegra_wdt)
{
	writel(WDT_UNLOCK_PATTERN, tegra_wdt->wdt_source + WDT_UNLOCK);
	writel(WDT_CMD_DISABLE_COUNTER, tegra_wdt->wdt_source + WDT_CMD);

	writel(0, tegra_wdt->wdt_timer + TIMER_PTV);

	tegra_wdt->status = WDT_DISABLED;

	return 0;
}

#endif

static int tegra_wdt_enable(struct watchdog_device *wdt)
{
	struct tegra_wdt *tegra_wdt = to_tegra_wdt(wdt);
	return __tegra_wdt_enable(tegra_wdt);
}

static int tegra_wdt_disable(struct watchdog_device *wdt)
{
	struct tegra_wdt *tegra_wdt = to_tegra_wdt(wdt);
	return __tegra_wdt_disable(tegra_wdt);
}

static int tegra_wdt_ping(struct watchdog_device *wdt)
{
	struct tegra_wdt *tegra_wdt = to_tegra_wdt(wdt);
	return __tegra_wdt_ping(tegra_wdt);
}


static int tegra_wdt_set_timeout(struct watchdog_device *wdt, unsigned int timeout)
{
	tegra_wdt_disable(wdt);
	wdt->timeout = timeout;
	tegra_wdt_enable(wdt);
	return 0;
}

static const struct watchdog_info tegra_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Tegra WDT",
	.firmware_version = 1,
};

static const struct watchdog_ops tegra_wdt_ops = {
	.owner = THIS_MODULE,
	.start = tegra_wdt_enable,
	.stop  = tegra_wdt_disable,
	.ping  = tegra_wdt_ping,
	.set_timeout = tegra_wdt_set_timeout,
};

static int tegra_wdt_probe(struct platform_device *pdev)
{
	struct resource *res_src, *res_wdt;
	struct tegra_wdt *tegra_wdt;
	int ret = 0;

	res_src = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res_wdt = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (!res_src || !res_wdt) {
		dev_err(&pdev->dev, "incorrect resources\n");
		return -ENOENT;
	}

	tegra_wdt = kzalloc(sizeof(*tegra_wdt), GFP_KERNEL);
	if (!tegra_wdt) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	tegra_wdt->wdt.info = &tegra_wdt_info;
	tegra_wdt->wdt.ops = &tegra_wdt_ops;
	tegra_wdt->wdt.min_timeout = MIN_WDT_PERIOD;
	tegra_wdt->wdt.max_timeout = MAX_WDT_PERIOD;
	tegra_wdt->wdt.timeout = 120;

	res_src = request_mem_region(res_src->start, resource_size(res_src),
				     pdev->name);
	res_wdt = request_mem_region(res_wdt->start, resource_size(res_wdt),
				     pdev->name);

	if (!res_src || !res_wdt) {
		dev_err(&pdev->dev, "unable to request memory resources\n");
		ret = -EBUSY;
		goto fail;
	}

	tegra_wdt->wdt_source = ioremap(res_src->start, resource_size(res_src));
	tegra_wdt->wdt_timer = ioremap(res_wdt->start, resource_size(res_wdt));
	/* tmrsrc will be used to set WDT_CFG */
	if ((res_wdt->start & 0xff) < 0x50)
		tegra_wdt->tmrsrc = 1 + (res_wdt->start & 0xf) / 8;
	else
		tegra_wdt->tmrsrc = (3 + ((res_wdt->start & 0xff) - 0x50) / 8) % 10;
	if (!tegra_wdt->wdt_source || !tegra_wdt->wdt_timer) {
		dev_err(&pdev->dev, "unable to map registers\n");
		ret = -ENOMEM;
		goto fail;
	}

	tegra_wdt_disable(&tegra_wdt->wdt);
	writel(TIMER_PCR_INTR, tegra_wdt->wdt_timer + TIMER_PCR);

	tegra_wdt->res_src = res_src;
	tegra_wdt->res_wdt = res_wdt;

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Init and enable watchdog on WDT0 with timer 8 during probe */
	if (enable_on_probe) {
		set_bit(WDOG_ACTIVE, &tegra_wdt->wdt.status);
		tegra_wdt_enable(&tegra_wdt->wdt);
		pr_info("WDT heartbeat enabled on probe\n");
	}
#endif

	watchdog_init_timeout(&tegra_wdt->wdt, heartbeat,  &pdev->dev);

	ret = watchdog_register_device(&tegra_wdt->wdt);
	if (ret) {
		dev_err(&pdev->dev, "failed to register watchdog device\n");
		goto fail;
	}

	platform_set_drvdata(pdev, tegra_wdt);

	dev_info(&pdev->dev, "%s done\n", __func__);
	return 0;
fail:
	if (tegra_wdt->wdt_source)
		iounmap(tegra_wdt->wdt_source);
	if (tegra_wdt->wdt_timer)
		iounmap(tegra_wdt->wdt_timer);
	if (res_src)
		release_mem_region(res_src->start, resource_size(res_src));
	if (res_wdt)
		release_mem_region(res_wdt->start, resource_size(res_wdt));
	kfree(tegra_wdt);
	return ret;
}

static int tegra_wdt_remove(struct platform_device *pdev)
{
	struct tegra_wdt *tegra_wdt = platform_get_drvdata(pdev);

	tegra_wdt_disable(&tegra_wdt->wdt);

	watchdog_unregister_device(&tegra_wdt->wdt);
	iounmap(tegra_wdt->wdt_source);
	iounmap(tegra_wdt->wdt_timer);
	release_mem_region(tegra_wdt->res_src->start, resource_size(tegra_wdt->res_src));
	release_mem_region(tegra_wdt->res_wdt->start, resource_size(tegra_wdt->res_wdt));
	kfree(tegra_wdt);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_wdt *tegra_wdt = platform_get_drvdata(pdev);

	__tegra_wdt_disable(tegra_wdt);
	return 0;
}

static int tegra_wdt_resume(struct platform_device *pdev)
{
	struct tegra_wdt *tegra_wdt = platform_get_drvdata(pdev);

	if (watchdog_active(&tegra_wdt->wdt))
		__tegra_wdt_enable(tegra_wdt);

	return 0;
}
#endif

static const struct of_device_id tegra_wdt_match[] = {
	{ .compatible = "nvidia,tegra-wdt", },
	{}
};

static struct platform_driver tegra_wdt_driver = {
	.probe		= tegra_wdt_probe,
	.remove		= tegra_wdt_remove,
#ifdef CONFIG_PM
	.suspend	= tegra_wdt_suspend,
	.resume		= tegra_wdt_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tegra_wdt",
		.of_match_table = of_match_ptr(tegra_wdt_match),
	},
};

static int __init tegra_wdt_init(void)
{
	return platform_driver_register(&tegra_wdt_driver);
}

static void __exit tegra_wdt_exit(void)
{
	platform_driver_unregister(&tegra_wdt_driver);
}

module_init(tegra_wdt_init);
module_exit(tegra_wdt_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra Watchdog Driver");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
		 "Watchdog heartbeat period in seconds");

module_param(enable_on_probe, bool, 0);
MODULE_PARM_DESC(enable_on_probe,
		 "Start watchdog during boot");

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tegra_wdt");
