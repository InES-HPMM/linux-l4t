/*
 * Power off driver for Maxim MAX77620 device.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Chaitanya Bandi <bandik@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power/reset/system-pmic.h>
#include <linux/slab.h>
#include <linux/mfd/max77620.h>

struct max77620_poweroff {
	struct device *dev;
	struct max77620_chip *max77620;
	struct system_pmic_dev *system_pmic_dev;
	bool use_power_off;
	bool use_power_reset;
};

static void max77620_pm_power_off(void *drv_data)
{
	struct max77620_poweroff *max77620_poweroff = drv_data;
	int ret;

	dev_info(max77620_poweroff->dev, "Powering off system\n");

	max77620_allow_atomic_xfer(max77620_poweroff->max77620);

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG1,
		MAX77620_ONOFFCNFG1_PWR_OFF, MAX77620_ONOFFCNFG1_PWR_OFF);
	if (ret < 0)
		dev_err(max77620_poweroff->dev,
			"REG_ONOFFCNFG1 update failed, %d\n", ret);
}

static void max77620_pm_power_reset(void *drv_data)
{
	struct max77620_poweroff *max77620_poweroff = drv_data;
	int ret;

	dev_info(max77620_poweroff->dev, "Power resetting system\n");

	max77620_allow_atomic_xfer(max77620_poweroff->max77620);

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG2,
		MAX77620_ONOFFCNFG2_SFT_RST_WK, MAX77620_ONOFFCNFG2_SFT_RST_WK);

	ret = max77620_reg_update(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_ONOFFCNFG1,
		MAX77620_ONOFFCNFG1_SFT_RST, MAX77620_ONOFFCNFG1_SFT_RST);
	if (ret < 0)
		dev_err(max77620_poweroff->dev,
			"REG_ONOFFCNFG1 update failed, %d\n", ret);
}

static struct system_pmic_ops max77620_pm_ops = {
	.power_off = max77620_pm_power_off,
	.power_reset = max77620_pm_power_reset,
};

static int max77620_poweroff_probe(struct platform_device *pdev)
{
	struct max77620_poweroff *max77620_poweroff;
	struct device_node *np = pdev->dev.parent->of_node;
	struct max77620_chip *max77620 = dev_get_drvdata(pdev->dev.parent);
	struct system_pmic_config config;
	bool use_power_off = false;
	bool use_power_reset = false;
	u8 poweroff_event_recorder;
	int ret;

	if (np) {
		bool system_pc;

		use_power_off = of_property_read_bool(np,
				"maxim,system-pmic-power-off");
		use_power_reset = of_property_read_bool(np,
				"maxim,system-pmic-power-reset");
		system_pc = of_property_read_bool(np,
				"maxim,system-power-controller");
		if (system_pc) {
			use_power_off = true;
			use_power_reset = true;
		}
	}

	if (!use_power_off && !use_power_reset) {
		dev_warn(&pdev->dev,
			"power off and reset functionality not selected\n");
		return 0;
	}

	max77620_poweroff = devm_kzalloc(&pdev->dev, sizeof(*max77620_poweroff),
				GFP_KERNEL);
	if (!max77620_poweroff)
		return -ENOMEM;

	max77620_poweroff->max77620 = max77620;
	max77620_poweroff->dev = &pdev->dev;
	max77620_poweroff->use_power_off = use_power_off;
	max77620_poweroff->use_power_reset = use_power_reset;

	config.allow_power_off = use_power_off;
	config.allow_power_reset = use_power_reset;

	max77620_poweroff->system_pmic_dev = system_pmic_register(&pdev->dev,
				&max77620_pm_ops, &config, max77620_poweroff);
	if (IS_ERR(max77620_poweroff->system_pmic_dev)) {
		ret = PTR_ERR(max77620_poweroff->system_pmic_dev);

		dev_err(&pdev->dev, "System PMIC registration failed: %d\n",
			ret);
		return ret;
	}

	ret = max77620_reg_read(max77620_poweroff->max77620->dev,
		MAX77620_PWR_SLAVE, MAX77620_REG_NVERC,
		&poweroff_event_recorder);
	if (ret < 0) {
		dev_err(max77620_poweroff->dev,
			"REG_NVERC read failed, %d\n", ret);
		return ret;
	} else
		dev_info(&pdev->dev, "Event recorder REG_NVERC : 0x%x\n",
				poweroff_event_recorder);

	return 0;
}

static int max77620_poweroff_remove(struct platform_device *pdev)
{
	struct max77620_poweroff *max77620_poweroff =
					platform_get_drvdata(pdev);
	system_pmic_unregister(max77620_poweroff->system_pmic_dev);
	return 0;
}

static struct platform_driver max77620_poweroff_driver = {
	.driver = {
		.name = "max77620-power-off",
		.owner = THIS_MODULE,
	},
	.probe = max77620_poweroff_probe,
	.remove = max77620_poweroff_remove,
};

module_platform_driver(max77620_poweroff_driver);

MODULE_DESCRIPTION("Power off driver for MAX77620 PMIC Device");
MODULE_ALIAS("platform:max77620-power-off");
MODULE_AUTHOR("Chaitanya Bandi <bandik@nvidia.com>");
MODULE_LICENSE("GPL v2");
