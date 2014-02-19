/*
 * dev.c
 *
 * A device driver for ADSP and APE
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "dev.h"

static int nvadsp_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int nvadsp_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long nvadsp_ioctl(struct file *filp, unsigned int cmd,
			 unsigned long arg)
{
	return 0;
}

static long nvadsp_compat_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg)
{
	return 0;
}

static const struct file_operations nvadsp_fops = {
	.owner		= THIS_MODULE,
	.open		= nvadsp_open,
	.release	= nvadsp_release,
	.unlocked_ioctl	= nvadsp_ioctl,
	.compat_ioctl	= nvadsp_compat_ioctl,
};

#ifdef CONFIG_PM_SLEEP
static int nvadsp_suspend(struct device *dev)
{
	return 0;
}

static int nvadsp_resume(struct device *dev)
{
	return 0;
}
#endif	/* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int nvadsp_rt_suspend(struct device *dev)
{
	return 0;
}

static int nvadsp_rt_resume(struct device *dev)
{
	return 0;
}
static int nvadsp_rt_idle(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops nvadsp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nvadsp_suspend, nvadsp_resume)
	SET_RUNTIME_PM_OPS(nvadsp_rt_suspend, nvadsp_rt_resume,
			   nvadsp_rt_idle)
};

static int nvadsp_probe(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data;
	struct resource *res = NULL;
	void __iomem *base = NULL;
	int ret = 0;

	dev_info(&pdev->dev, "in probe()...\n");

	drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data),
				GFP_KERNEL);
	if (!drv_data) {
		dev_err(&pdev->dev, "Failed to allocate driver data");
		ret = -ENOMEM;
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get AMISC resource\n");
		ret = -EINVAL;
		goto err;
	}

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "Failed to remap AMISC resource\n");
		ret = PTR_ERR(base);
		goto err;
	}
	drv_data->amisc_base = base;

	platform_set_drvdata(pdev, drv_data);

	ret = nvadsp_hwmbox_init(pdev);

 err:
	return ret;
}

static int nvadsp_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nvadsp_of_match[] = {
	{ .compatible = "nvidia,tegra210-adsp", .data = NULL, },
	{},
};
#endif

static struct platform_driver nvadsp_driver = {
	.driver	= {
		.name	= "nvadsp",
		.owner	= THIS_MODULE,
		.pm	= &nvadsp_pm_ops,
		.of_match_table = of_match_ptr(nvadsp_of_match),
	},
	.probe		= nvadsp_probe,
	.remove		= nvadsp_remove,
};

static int __init nvadsp_init(void)
{
	return platform_driver_register(&nvadsp_driver);
}

static void __exit nvadsp_exit(void)
{
	platform_driver_unregister(&nvadsp_driver);
}

module_init(nvadsp_init);
module_exit(nvadsp_exit);

MODULE_AUTHOR("NVIDIA");
MODULE_DESCRIPTION("Tegra Host ADSP Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
