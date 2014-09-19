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
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/io.h>
#include <linux/tegra_nvadsp.h>
#include <linux/tegra-soc.h>
#include <linux/pm_runtime.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>

#include "dev.h"
#include "os.h"
#include "amc.h"
#include "ape_actmon.h"
#include "aram_manager.h"

#ifdef CONFIG_DEBUG_FS
static int __init adsp_debug_init(struct nvadsp_drv_data *drv_data)
{
	drv_data->adsp_debugfs_root = debugfs_create_dir("tegra_ape", NULL);
	if (!drv_data->adsp_debugfs_root)
		return -ENOMEM;
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static inline bool nvadsp_amsic_skip_reg(u32 offset)
{
	if (offset == AMISC_ADSP_L2_REGFILEBASE ||
	    offset == AMISC_SHRD_SMP_STA ||
	    (offset >= AMISC_SEM_REG_START && offset <= AMISC_SEM_REG_END) ||
	    offset == AMISC_TSC ||
	    offset == AMISC_ACTMON_AVG_CNT) {
		return true;
	} else {
		return false;
	}
}

int nvadsp_amisc_save(struct platform_device *pdev)
{
	struct nvadsp_drv_data *d = platform_get_drvdata(pdev);
	u32 val, offset;
	int i = 0;

	offset = AMISC_REG_START_OFFSET;
	while (offset <= AMISC_REG_MBOX_OFFSET) {
		if (nvadsp_amsic_skip_reg(offset)) {
			offset += 4;
			continue;
		}
		val = readl(d->base_regs[AMISC] + offset);
		d->state.amisc_regs[i++] = val;
		offset += 4;
	}

	offset = ADSP_ACTMON_REG_START_OFFSET;
	while (offset <= ADSP_ACTMON_REG_END_OFFSET) {
		if (nvadsp_amsic_skip_reg(offset)) {
			offset += 4;
			continue;
		}
		val = readl(d->base_regs[AMISC] + offset);
		d->state.amisc_regs[i++] = val;
		offset += 4;
	}
	return 0;
}

int nvadsp_amisc_restore(struct platform_device *pdev)
{
	struct nvadsp_drv_data *d = platform_get_drvdata(pdev);
	u32 val, offset;
	int i = 0;

	offset = AMISC_REG_START_OFFSET;
	while (offset <= AMISC_REG_MBOX_OFFSET) {
		if (nvadsp_amsic_skip_reg(offset)) {
			offset += 4;
			continue;
		}
		val = d->state.amisc_regs[i++];
		writel(val, d->base_regs[AMISC] + offset);
		offset += 4;
	}

	offset = ADSP_ACTMON_REG_START_OFFSET;
	while (offset <= ADSP_ACTMON_REG_END_OFFSET) {
		if (nvadsp_amsic_skip_reg(offset)) {
			offset += 4;
			continue;
		}
		val = d->state.amisc_regs[i++];
		writel(val, d->base_regs[AMISC] + offset);
		offset += 4;
	}
	return 0;
}


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
int nvadsp_clocks_enable(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	uint32_t val;
	int ret;

	drv_data->ape_clk = clk_get_sys(NULL, "ape");
	if (IS_ERR_OR_NULL(drv_data->ape_clk)) {
		dev_info(dev, "unable to find ape clock\n");
		ret = PTR_ERR(drv_data->ape_clk);
		goto end;
	}
	clk_prepare_enable(drv_data->ape_clk);
	dev_dbg(dev, "ape clock enabled\n");

	drv_data->adsp_clk = clk_get_sys(NULL, "adsp");
	if (IS_ERR_OR_NULL(drv_data->adsp_clk)) {
		dev_err(dev, "unable to find adsp clock\n");
		ret = PTR_ERR(drv_data->adsp_clk);
		goto end;
	}
	clk_prepare_enable(drv_data->adsp_clk);
	tegra_periph_reset_assert(drv_data->adsp_clk);
	udelay(10);
	dev_dbg(dev, "adsp clock enabled and asserted\n");

	drv_data->ape_uart_clk = clk_get_sys("uartape", NULL);
	if (IS_ERR_OR_NULL(drv_data->ape_uart_clk)) {
		dev_err(dev, "unable to find uart ape clk\n");
		ret = PTR_ERR(drv_data->ape_uart_clk);
		goto end;
	}
	clk_prepare_enable(drv_data->ape_uart_clk);
	clk_set_rate(drv_data->ape_uart_clk, UART_BAUD_RATE * 16);
	dev_dbg(dev, "uartape clock enabled\n");

	/* Set MAXCLKLATENCY value before ADSP deasserting reset */
	val = readl(drv_data->base_regs[AMISC] + ADSP_CONFIG);
	writel(val | MAXCLKLATENCY, drv_data->base_regs[AMISC] + ADSP_CONFIG);

 end:
	return 0;
}

static int nvadsp_clocks_disable(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	clk_disable_unprepare(drv_data->adsp_clk);
	clk_disable_unprepare(drv_data->ape_uart_clk);
	dev_dbg(dev, "adsp & uartape clocks disabled\n");

	clk_disable_unprepare(drv_data->ape_clk);
	dev_dbg(dev, "ape clock disabled\n");
	return 0;
}

static int nvadsp_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = 0;

	if (!drv_data->adsp_os_loaded) {
		dev_err(dev, "adsp os is not loaded\n");
		goto clocks;
	}

	dev_dbg(dev, "saving amsic\n");
	nvadsp_amisc_save(pdev);

	dev_dbg(dev, "saving aram\n");
	nvadsp_aram_save(pdev);

	dev_dbg(dev, "saving amc\n");
	nvadsp_amc_save(pdev);
 clocks:
	dev_dbg(dev, "disabling clocks\n");
	nvadsp_clocks_disable(pdev);
	return ret;
}

static int nvadsp_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = 0;

	dev_dbg(dev, "enabling clocks\n");
	nvadsp_clocks_enable(pdev);

	if (!drv_data->adsp_os_loaded) {
		dev_info(dev, "adsp os is not loaded\n");
		goto skip;
	}

	dev_dbg(dev, "restoring ape state\n");
	nvadsp_amc_restore(pdev);
	nvadsp_aram_restore(pdev);
	nvadsp_amisc_restore(pdev);
 skip:
	return ret;
}

static int nvadsp_runtime_idle(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops nvadsp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nvadsp_suspend, nvadsp_resume)
	SET_RUNTIME_PM_OPS(nvadsp_runtime_suspend, nvadsp_runtime_resume,
			   nvadsp_runtime_idle)
};

static int __init nvadsp_probe(struct platform_device *pdev)
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

#if CONFIG_DEBUG_FS
	if (adsp_debug_init(drv_data))
		dev_err(dev,
			"unable to create tegra_ape debug fs directory\n");
#endif

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

	pm_runtime_enable(dev);

	ret = nvadsp_os_probe(pdev);
	if (ret)
		goto err;

	ret = nvadsp_amc_init(pdev);
	if (ret)
		goto err;

	ret = nvadsp_hwmbox_init(pdev);
	if (ret)
		goto err;

	ret = nvadsp_mbox_init(pdev);
	if (ret)
		goto err;

#ifdef CONFIG_TEGRA_EMC_APE_DFS
	ret = emc_dfs_init(pdev);
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
	emc_dfs_exit();
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
