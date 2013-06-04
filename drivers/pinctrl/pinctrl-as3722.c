/*
 * pinctrl-as3722.c -- TI AS3722 series pin control driver.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author: Mallikarjun Kasoju <mkasoju@nvidia.com>
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
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>
#include <linux/module.h>
#include <linux/platform_device.h>

struct as3722_pinctrl {
	struct device	*dev;
	struct as3722	*as3722;
};

static int as3722_set_single_pin_config(struct as3722_pinctrl *pinctrl,
	struct as3722_pinctrl_init_data *pin_info)
{
	int ret;

	if ((pin_info->pin_id < 0) || (pin_info->pin_id >= AS3722_NUM_GPIO)) {
		dev_err(pinctrl->dev, "Pin id %d is out of range\n",
			pin_info->pin_id);
		return -EINVAL;
	}

	if (pin_info->usage == AS3722_GPIO_USAGE_IO_OPERATION)
		return 0;

	ret = as3722_set_bits(pinctrl->as3722,
			AS3722_GPIO0_CONTROL_REG + pin_info->pin_id,
			AS3722_GPIO_MODE_MASK, pin_info->mode);
	if (ret < 0) {
		dev_err(pinctrl->dev,
				"Failed to set pin mode config\n");
		return ret;
	}
	ret = as3722_set_bits(pinctrl->as3722,
			AS3722_GPIO0_CONTROL_REG + pin_info->pin_id,
			AS3722_GPIO_USAGE_MASK,
			pin_info->usage);
	if (ret < 0)
		dev_err(pinctrl->dev, "Failed to set pin usage\n");

	return ret;
}

static int as3722_pinctrl_probe(struct platform_device *pdev)
{
	struct as3722_platform_data *pdata;
	struct as3722_pinctrl_init_data *pctrl_pdata;
	struct as3722_pinctrl *as3722_pinctrl;
	struct as3722 *as3722 = dev_get_drvdata(pdev->dev.parent);
	int i;

	pdata = dev_get_platdata(pdev->dev.parent);
	if (!pdata || !pdata->pinctrl_pdata || !pdata->pinctrl_init_data_size) {
		dev_info(&pdev->dev, "No pinctrl platform data\n");
		return -EINVAL;
	}

	as3722_pinctrl =
		devm_kzalloc(&pdev->dev, sizeof(*as3722_pinctrl), GFP_KERNEL);
	if (!as3722_pinctrl) {
		dev_err(&pdev->dev, "Memory allocation for pinctrl failed\n");
		return -ENOMEM;
	}

	as3722_pinctrl->dev = &pdev->dev;
	as3722_pinctrl->as3722 = as3722;
	pctrl_pdata = pdata->pinctrl_pdata;

	for (i = 0; i < pdata->pinctrl_init_data_size; i++)
		as3722_set_single_pin_config(as3722_pinctrl, &pctrl_pdata[i]);

	return 0;
}

static struct platform_driver as3722_pinctrl_driver = {
	.probe = as3722_pinctrl_probe,
	.driver = {
		.name = "as3722-pinctrl",
		.owner = THIS_MODULE,
	},
};

static int __init as3722_pinctrl_init(void)
{
	return platform_driver_register(&as3722_pinctrl_driver);
}
subsys_initcall(as3722_pinctrl_init);

static void __exit as3722_pinctrl_exit(void)
{
	platform_driver_unregister(&as3722_pinctrl_driver);
}
module_exit(as3722_pinctrl_exit);

MODULE_DESCRIPTION("as3722 pin control driver");
MODULE_AUTHOR("Mallikarjun Kasoju<mkasoju@nvidia.com>");
MODULE_ALIAS("platform:as3722-pinctrl");
MODULE_LICENSE("GPL v2");
