/*
 * arch/arm/mach-tegra/tegra3_emc.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
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
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_emc.h>

#include <mach/iomap.h>

#include "clock.h"
#include "tegra3_emc.h"

#ifdef CONFIG_TEGRA_EMC_SCALING_ENABLE
static bool emc_enable = true;
#else
static bool emc_enable;
#endif
module_param(emc_enable, bool, 0644);

static struct platform_device *emc_pdev;
static void __iomem *emc_regbase;

static inline void emc_writel(u32 val, unsigned long addr)
{
	__raw_writel(val, emc_regbase + addr);
}

static inline u32 emc_readl(unsigned long addr)
{
	return __raw_readl(emc_regbase + addr);
}

/* Select the closest EMC rate that is higher than the requested rate */
long tegra_emc_round_rate(unsigned long rate)
{
	return -ENOSYS;
}

/* The EMC registers have shadow registers.  When the EMC clock is updated
 * in the clock controller, the shadow registers are copied to the active
 * registers, allowing glitchless memory bus frequency changes.
 * This function updates the shadow registers for a new clock frequency,
 * and relies on the clock lock on the emc clock to avoid races between
 * multiple frequency changes */
int tegra_emc_set_rate(unsigned long rate)
{
	return -ENOSYS;
}

static int tegra_emc_probe(struct platform_device *pdev)
{
	struct tegra_emc_chip *pdata = NULL;
	struct resource *res;

	if (!emc_enable) {
		dev_err(&pdev->dev, "disabled per module parameter\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing register base\n");
		return -ENOMEM;
	}

	emc_regbase = devm_request_and_ioremap(&pdev->dev, res);
	if (!emc_regbase) {
		dev_err(&pdev->dev, "failed to remap registers\n");
		return -ENOMEM;
	}

	if (pdev->dev.platform_data.num_chips > 0)
		pdata = pdev->dev.platform_data.chips[0];

	pdev->dev.platform_data = pdata;

	emc_pdev = pdev;

	return 0;
}

static struct platform_driver tegra_emc_driver = {
	.driver		= {
		.name	= "tegra30-emc",
		.owner	= THIS_MODULE,
	},
	.probe		= tegra_emc_probe,
};

static int __init tegra_init_emc(void)
{
	return platform_driver_register(&tegra_emc_driver);
}
device_initcall(tegra_emc_init);
