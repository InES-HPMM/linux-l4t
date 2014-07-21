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
#include <linux/io.h>
#include <linux/tegra_nvadsp.h>
#include <linux/miscdevice.h>

#include "dev.h"
#include "os.h"
#include "ape_actmon.h"
#include "aram_manager.h"

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
	int iter;
	int dram_iter;
	struct device *dev = &pdev->dev;

	dev_info(dev, "in probe()...\n");

	drv_data = devm_kzalloc(dev, sizeof(*drv_data),
				GFP_KERNEL);
	if (!drv_data) {
		dev_err(&pdev->dev, "Failed to allocate driver data");
		ret = -ENOMEM;
		goto err;
	}

	drv_data->base_regs =
		devm_kzalloc(dev, sizeof(void *) * APE_MAX_REG,
							GFP_KERNEL);
	if (!drv_data->base_regs) {
		dev_err(dev, "Failed to allocate regs");
		ret = -ENOMEM;
		goto err;
	}

	for (iter = 0; iter < APE_MAX_REG; iter++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, iter);
		if (!res) {
			dev_err(dev,
			"Failed to get resource with ID %d\n",
							iter);
			ret = -EINVAL;
			goto err;
		}

		base = devm_ioremap_resource(dev, res);
		if (IS_ERR(base)) {
			dev_err(dev,
				"Failed to remap AMISC resource\n");
			ret = PTR_ERR(base);
			goto err;
		}
		drv_data->base_regs[iter] = base;
		adsp_add_load_mappings(res->start, base,
						resource_size(res));
	}

	for (dram_iter = 0; dram_iter < ADSP_MAX_DRAM_MAP; dram_iter++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, iter++);
		if (!res) {
			dev_err(dev,
			"Failed to get DRAM map with ID %d\n", iter);
			ret = -EINVAL;
			goto err;
		}

		drv_data->dram_region[dram_iter] = res;
	}

	platform_set_drvdata(pdev, drv_data);
	ret = nvadsp_os_probe(pdev);
	if (ret)
		goto err;

	ret = nvadsp_aram_init(pdev);
	if (ret)
		goto err;

	ret = nvadsp_hwmbox_init(pdev);
	if (ret)
		goto err;

	ret = nvadsp_mbox_init(pdev);
	if (ret)
		goto err;

	ret = ape_actmon_init(pdev);
	if (ret)
		goto err;

#ifdef CONFIG_TEGRA_ADSP_DFS
	ret = adsp_dfs_core_init();
	if (ret)
		goto err;
#endif
	ret = nvadsp_app_module_probe(pdev);
	if (ret)
		goto err;

	ret = aram_init();
	if (ret)
		dev_err(dev, "Failed to init aram\n");
err:
	return ret;
}

static int nvadsp_remove(struct platform_device *pdev)
{
	aram_exit();

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
