/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include "hier_ictlr.h"

#define HIER_GROUP_CPU_ENABLE                                 0x00000000
#define HIER_GROUP_CPU_STATUS                                 0x00000004
#define HIER_GROUP1_MSELECT_ERROR                                     15

#define MSELECT_CONFIG_0                                             0x0
#define MSELECT_CONFIG_0_READ_TIMEOUT_EN_SLAVE0_SHIFT                 16
#define MSELECT_CONFIG_0_WRITE_TIMEOUT_EN_SLAVE0_SHIFT                17
#define MSELECT_CONFIG_0_READ_TIMEOUT_EN_SLAVE1_SHIFT                 18
#define MSELECT_CONFIG_0_WRITE_TIMEOUT_EN_SLAVE1_SHIFT                19
#define MSELECT_CONFIG_0_READ_TIMEOUT_EN_SLAVE2_SHIFT                 20
#define MSELECT_CONFIG_0_WRITE_TIMEOUT_EN_SLAVE2_SHIFT                21
#define MSELECT_CONFIG_0_ERR_RESP_EN_SLAVE1_SHIFT                     24
#define MSELECT_CONFIG_0_ERR_RESP_EN_SLAVE2_SHIFT                     25

#define MSELECT_TIMEOUT_TIMER_0                                     0x5c
#define MSELECT_ERROR_STATUS_0                                      0x60
#define MSELECT_DEFAULT_TIMEOUT                                 0xFFFFFF

static irqreturn_t tegra_hier_ictlr_irq_handler(int irq, void *data)
{
	struct device *dev = data;
	struct tegra_hier_ictlr *ictlr = dev_get_drvdata(dev);
	unsigned long status;

	status = readl(ictlr->mselect_base + MSELECT_ERROR_STATUS_0);
	if (status != 0) {
		pr_err("MSELECT error detected! status=0x%x\n",
			(unsigned int)status);
		BUG();
	}

	return IRQ_HANDLED;
}

static int tegra_hier_ictlr_map_memory(struct platform_device *pdev,
				       struct tegra_hier_ictlr *ictlr)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to allocate resources (hier-ictlr).\n");
		return -EBUSY;
	}

	ictlr->hier_ictlr_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!ictlr->hier_ictlr_base) {
		dev_err(&pdev->dev, "Unable to map memory (hier-ictlr).\n");
		return -EBUSY;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev,
		    "Unable to allocate resources (mselect).\n");
		return -EBUSY;
	}

	ictlr->mselect_base = devm_request_and_ioremap(&pdev->dev, res);
	if (!ictlr->mselect_base) {
		dev_err(&pdev->dev, "Unable to map memory (mselect).\n");
		return -EBUSY;
	}

	return 0;
}

static int tegra_hier_ictlr_irq_init(struct platform_device *pdev,
				     struct tegra_hier_ictlr *ictlr)
{
	unsigned long reg;
	int ret;

	ictlr->irq = platform_get_irq(pdev, 0);
	if (ictlr->irq <= 0)
		return -EBUSY;

	ret = devm_request_irq(&pdev->dev, ictlr->irq,
		tegra_hier_ictlr_irq_handler, IRQF_TRIGGER_HIGH,
		"hier_ictlr_irq", &pdev->dev);

	if (ret) {
		dev_err(&pdev->dev,
			"Unable to request interrupt for device (err=%d).\n",
			ret);
		return ret;
	}

	reg = readl(ictlr->hier_ictlr_base + HIER_GROUP_CPU_ENABLE);
	writel(reg | (1 << HIER_GROUP1_MSELECT_ERROR),
		ictlr->hier_ictlr_base + HIER_GROUP_CPU_ENABLE);

	return 0;
}

static int tegra_hier_ictlr_mselect_init(struct platform_device *pdev,
					 struct tegra_hier_ictlr *ictlr)
{
	unsigned long reg;

	tegra_hier_ictlr_set_mselect_timeout(ictlr, MSELECT_DEFAULT_TIMEOUT);

	reg = readl(ictlr->mselect_base + MSELECT_CONFIG_0);
	writel(reg |
		((1 << MSELECT_CONFIG_0_READ_TIMEOUT_EN_SLAVE0_SHIFT)  |
		 (1 << MSELECT_CONFIG_0_WRITE_TIMEOUT_EN_SLAVE0_SHIFT) |
		 (1 << MSELECT_CONFIG_0_READ_TIMEOUT_EN_SLAVE1_SHIFT)  |
		 (1 << MSELECT_CONFIG_0_WRITE_TIMEOUT_EN_SLAVE1_SHIFT) |
		 (1 << MSELECT_CONFIG_0_READ_TIMEOUT_EN_SLAVE2_SHIFT)  |
		 (1 << MSELECT_CONFIG_0_WRITE_TIMEOUT_EN_SLAVE2_SHIFT) |
		 (1 << MSELECT_CONFIG_0_ERR_RESP_EN_SLAVE1_SHIFT)      |
		 (1 << MSELECT_CONFIG_0_ERR_RESP_EN_SLAVE2_SHIFT)),
		ictlr->mselect_base + MSELECT_CONFIG_0);

	return 0;
}

void tegra_hier_ictlr_set_mselect_timeout(struct tegra_hier_ictlr *ictlr,
					  u32 timeout_cycles)
{
	ictlr->mselect_timeout_cycles = timeout_cycles;

	writel(ictlr->mselect_timeout_cycles, ictlr->mselect_base +
		MSELECT_TIMEOUT_TIMER_0);
}

static int tegra_hier_ictlr_probe(struct platform_device *pdev)
{
	struct tegra_hier_ictlr *ictlr;
	int ret;

	ictlr = devm_kzalloc(&pdev->dev, sizeof(struct tegra_hier_ictlr),
		GFP_KERNEL);
	if (!ictlr)
		return -ENOMEM;

	ret = tegra_hier_ictlr_map_memory(pdev, ictlr);
	if (ret)
		return ret;

	ret = tegra_hier_ictlr_mselect_init(pdev, ictlr);
	if (ret)
		return ret;

	ret = tegra_hier_ictlr_irq_init(pdev, ictlr);
	if (ret)
		return ret;

	tegra_hier_ictlr_create_sysfs(pdev);

	dev_notice(&pdev->dev, "probed\n");

	dev_set_drvdata(&pdev->dev, ictlr);
	return 0;
}

static int __exit tegra_hier_ictlr_remove(struct platform_device *pdev)
{
	tegra_hier_ictlr_remove_sysfs(pdev);
	return 0;
}

static struct platform_driver tegra_hier_ictlr_driver = {
	.driver = {
		   .name = "tegra-hier-ictlr",
		   .owner = THIS_MODULE,
		   },
	.probe = tegra_hier_ictlr_probe,
	.remove = __exit_p(tegra_hier_ictrl_remove),
};

static int __init tegra_hier_ictlr_init(void)
{
	return platform_driver_register(&tegra_hier_ictlr_driver);
}

static void __exit tegra_hier_ictlr_exit(void)
{
	platform_driver_unregister(&tegra_hier_ictlr_driver);
}

module_init(tegra_hier_ictlr_init);
module_exit(tegra_hier_ictlr_exit);

MODULE_DESCRIPTION("Tegra Hierarchical Interrupt Controller Driver");
MODULE_LICENSE("GPL v2");
