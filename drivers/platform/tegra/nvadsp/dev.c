/*
 * dev.c
 *
 * A device driver for ADSP and APE
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
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
#include <linux/tegra_pm_domains.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>

#include "dev.h"
#include "os.h"
#include "amc.h"
#include "ape_actmon.h"
#include "aram_manager.h"

#define ADSP_TSC	0x48

static DEFINE_SPINLOCK(tsc_lock);
static struct nvadsp_drv_data *nvadsp_drv_data;

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

#ifdef CONFIG_PM_RUNTIME
static int nvadsp_amisc_save(struct platform_device *pdev)
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

static int nvadsp_amisc_restore(struct platform_device *pdev)
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
#endif /* CONFIG_PM_RUNTIME */

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
static void nvadsp_clocks_disable(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (drv_data->uartape_clk) {
		clk_disable_unprepare(drv_data->uartape_clk);
		dev_dbg(dev, "uartape clock disabled\n");
		drv_data->uartape_clk = NULL;
	}

	if (drv_data->adsp_cpu_clk) {
		clk_disable_unprepare(drv_data->adsp_cpu_clk);
		dev_dbg(dev, "adsp_cpu clock disabled\n");
		drv_data->adsp_cpu_clk = NULL;
	}

	if (drv_data->adsp_clk) {
		clk_disable_unprepare(drv_data->adsp_clk);
		dev_dbg(dev, "adsp clocks disabled\n");
		drv_data->adsp_clk = NULL;
	}

	if (drv_data->ape_clk) {
		clk_disable_unprepare(drv_data->ape_clk);
		dev_dbg(dev, "ape clock disabled\n");
		drv_data->ape_clk = NULL;
	}

	if (drv_data->ape_emc_clk) {
		clk_disable_unprepare(drv_data->ape_emc_clk);
		dev_dbg(dev, "ape.emc clock disabled\n");
		drv_data->ape_emc_clk = NULL;
	}


	if (drv_data->ahub_clk) {
		clk_disable_unprepare(drv_data->ahub_clk);
		dev_dbg(dev, "ahub clock disabled\n");
		drv_data->ahub_clk = NULL;
	}
}

static int nvadsp_clocks_enable(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	uint32_t val;
	int ret = 0;

	drv_data->ahub_clk = clk_get_sys("nvadsp", "ahub");
	if (IS_ERR_OR_NULL(drv_data->ahub_clk)) {
		dev_err(dev, "unable to find ahub clock\n");
		ret = PTR_ERR(drv_data->ahub_clk);
		goto end;
	}
	ret = clk_prepare_enable(drv_data->ahub_clk);
	if (ret) {
		dev_err(dev, "unable to enable ahub clock\n");
		goto end;
	}
	dev_dbg(dev, "ahub clock enabled\n");

	drv_data->ape_clk = clk_get_sys(NULL, "adsp.ape");
	if (IS_ERR_OR_NULL(drv_data->ape_clk)) {
		dev_err(dev, "unable to find ape clock\n");
		ret = PTR_ERR(drv_data->ape_clk);
		goto end;
	}
	ret = clk_prepare_enable(drv_data->ape_clk);
	if (ret) {
		dev_err(dev, "unable to enable ape clock\n");
		goto end;
	}
	dev_dbg(dev, "ape clock enabled\n");

	drv_data->adsp_clk = clk_get_sys(NULL, "adsp");
	if (IS_ERR_OR_NULL(drv_data->adsp_clk)) {
		dev_err(dev, "unable to find adsp clock\n");
		ret = PTR_ERR(drv_data->adsp_clk);
		goto end;
	}
	ret = clk_prepare_enable(drv_data->adsp_clk);
	if (ret) {
		dev_err(dev, "unable to enable adsp clock\n");
		goto end;
	}

	drv_data->adsp_cpu_clk = clk_get_sys(NULL, "adsp_cpu");
	if (IS_ERR_OR_NULL(drv_data->adsp_cpu_clk)) {
		dev_err(dev, "unable to find adsp cpu clock\n");
		ret = PTR_ERR(drv_data->adsp_cpu_clk);
		goto end;
	}
	ret = clk_prepare_enable(drv_data->adsp_cpu_clk);
	if (ret) {
		dev_err(dev, "unable to enable adsp cpu clock\n");
		goto end;
	}
	dev_dbg(dev, "adsp cpu clock enabled\n");

	drv_data->ape_emc_clk = clk_get_sys("ape", "emc");
	if (IS_ERR_OR_NULL(drv_data->ape_emc_clk)) {
		dev_err(dev, "unable to find ape.emc clock\n");
		ret = PTR_ERR(drv_data->ape_emc_clk);
		goto end;
	}

	ret = clk_prepare_enable(drv_data->ape_emc_clk);
	if (ret) {
		dev_err(dev, "unable to enable ape.emc clock\n");
		goto end;
	}
	dev_dbg(dev, "ape.emc is enabled\n");

	drv_data->uartape_clk = clk_get_sys("uartape", NULL);
	if (IS_ERR_OR_NULL(drv_data->uartape_clk)) {
		dev_err(dev, "unable to find uart ape clk\n");
		ret = PTR_ERR(drv_data->uartape_clk);
		goto end;
	}
	ret = clk_prepare_enable(drv_data->uartape_clk);
	if (ret) {
		dev_err(dev, "unable to enable uartape clock\n");
		goto end;
	}
	clk_set_rate(drv_data->uartape_clk, UART_BAUD_RATE * 16);
	dev_dbg(dev, "uartape clock enabled\n");

	/* Set MAXCLKLATENCY value before ADSP deasserting reset */
	val = readl(drv_data->base_regs[AMISC] + ADSP_CONFIG);
	writel(val | MAXCLKLATENCY, drv_data->base_regs[AMISC] + ADSP_CONFIG);
	dev_dbg(dev, "all clocks enabled\n");
	return 0;
 end:
	nvadsp_clocks_disable(pdev);
	return ret;
}

static int nvadsp_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = 0;

	if (!drv_data->adsp_os_suspended) {
		dev_dbg(dev, "%s: adsp os is not suspended\n", __func__);
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

	dev_dbg(dev, "locking out adsp base regs\n");
	drv_data->base_regs = NULL;

	return ret;
}

static int nvadsp_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	int ret = 0;

	dev_dbg(dev, "restoring adsp base regs\n");
	drv_data->base_regs = drv_data->base_regs_saved;

	dev_dbg(dev, "enabling clocks\n");
	ret = nvadsp_clocks_enable(pdev);
	if (ret) {
		dev_err(dev, "nvadsp_clocks_enable failed\n");
		goto skip;
	}

	if (!drv_data->adsp_os_suspended) {
		dev_dbg(dev, "%s: adsp os is not suspended\n", __func__);
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

uint64_t nvadsp_get_timestamp_counter(void)
{
	uint32_t count_low = 0;
	uint32_t count_high = 0;
	uint64_t tsc = 0;
	void __iomem *base = nvadsp_drv_data->base_regs[AMISC];
	unsigned long flags;

	spin_lock_irqsave(&tsc_lock, flags);
read_again:
	/* read MSB 32 bits */
	count_high = readl(base + ADSP_TSC);
	/* read LSB 32 bits */
	count_low = readl(base + ADSP_TSC);
	if (count_high != readl(base + ADSP_TSC))
		goto read_again;

	/*
	 * Additional read to make sure that the next read is the higher
	 * nibble
	 */
	readl(base + ADSP_TSC);

	tsc = count_high;
	tsc <<= 32;
	tsc |= count_low;
	spin_unlock_irqrestore(&tsc_lock, flags);
	return tsc;
}
EXPORT_SYMBOL(nvadsp_get_timestamp_counter);

static void __init nvadsp_parse_clk_entries(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 val32 = 0;


	/* Optional properties, should come from platform dt files */
	if (of_property_read_u32(dev->of_node, "nvidia,adsp_freq", &val32))
		dev_dbg(dev, "adsp_freq dt not found\n");
	else
		drv_data->adsp_freq = val32;

	if (of_property_read_u32(dev->of_node, "nvidia,ape_freq", &val32))
		dev_dbg(dev, "ape_freq dt not found\n");
	else
		drv_data->ape_freq = val32;

	if (of_property_read_u32(dev->of_node, "nvidia,ape_emc_freq", &val32))
		dev_dbg(dev, "ape_emc_freq dt not found\n");
	else
		drv_data->ape_emc_freq = val32;
}

static int __init nvadsp_parse_dt(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	u32 *adsp_reset;
	u32 *adsp_mem;
	int iter;

	adsp_reset = drv_data->unit_fpga_reset;
	adsp_mem = drv_data->adsp_mem;

	for (iter = 0; iter < ADSP_MEM_END; iter++) {
		if (of_property_read_u32_index(dev->of_node, "nvidia,adsp_mem",
			iter, &adsp_mem[iter])) {
			dev_err(dev, "adsp memory dt %d not found\n", iter);
			return -EINVAL;
		}
	}

	drv_data->adsp_unit_fpga = of_property_read_bool(dev->of_node,
				"nvidia,adsp_unit_fpga");

	if (drv_data->adsp_unit_fpga) {
		for (iter = 0; iter < ADSP_UNIT_FPGA_RESET_END; iter++) {
			if (of_property_read_u32_index(dev->of_node,
				"nvidia,adsp_unit_fpga_reset", iter,
				&adsp_reset[iter])) {
				dev_err(dev, "adsp reset dt %d not found\n",
					iter);
				return -EINVAL;
			}
		}
	}
	nvadsp_parse_clk_entries(pdev);

	return 0;
}

static int __init nvadsp_probe(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data;
	struct device *dev = &pdev->dev;
	struct resource *res = NULL;
	void __iomem *base = NULL;
	uint32_t aram_addr;
	uint32_t aram_size;
	int dram_iter;
	int ret = 0;
	int iter;

	dev_info(dev, "in probe()...\n");

	drv_data = devm_kzalloc(dev, sizeof(*drv_data),
				GFP_KERNEL);
	if (!drv_data) {
		dev_err(&pdev->dev, "Failed to allocate driver data");
		ret = -ENOMEM;
		goto out;
	}

	platform_set_drvdata(pdev, drv_data);
	ret = nvadsp_parse_dt(pdev);
	if (ret)
		goto out;

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
		goto out;
	}

	for (iter = 0; iter < APE_MAX_REG; iter++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, iter);
		if (!res) {
			dev_err(dev,
			"Failed to get resource with ID %d\n",
							iter);
			ret = -EINVAL;
			goto out;
		}

		if (!drv_data->adsp_unit_fpga && iter == UNIT_FPGA_RST)
			continue;

		base = devm_ioremap_resource(dev, res);
		if (IS_ERR(base)) {
			dev_err(dev, "Failed to iomap resource reg[%d]\n",
				iter);
			ret = PTR_ERR(base);
			goto out;
		}
		drv_data->base_regs[iter] = base;
		adsp_add_load_mappings(res->start, base,
						resource_size(res));
	}

	drv_data->base_regs_saved = drv_data->base_regs;

	for (dram_iter = 0; dram_iter < ADSP_MAX_DRAM_MAP; dram_iter++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, iter++);
		if (!res) {
			dev_err(dev,
			"Failed to get DRAM map with ID %d\n", iter);
			ret = -EINVAL;
			goto out;
		}

		drv_data->dram_region[dram_iter] = res;
	}

	nvadsp_drv_data = drv_data;

#ifdef CONFIG_PM_RUNTIME
	tegra_adsp_pd_add_device(dev);

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret)
		goto out;
#endif

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

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	ret = ape_actmon_probe(pdev);
	if (ret)
		goto err;
#endif
	ret = nvadsp_app_module_probe(pdev);
	if (ret)
		goto err;

	aram_addr = drv_data->adsp_mem[ARAM_ALIAS_0_ADDR];
	aram_size = drv_data->adsp_mem[ARAM_ALIAS_0_SIZE];
	ret = aram_init(aram_addr, aram_size);
	if (ret)
		dev_err(dev, "Failed to init aram\n");
err:
#ifdef CONFIG_PM_RUNTIME
	ret = pm_runtime_put_sync(dev);
	if (ret)
		dev_err(dev, "pm_runtime_put_sync failed\n");
#endif
out:
	return ret;
}

static int nvadsp_remove(struct platform_device *pdev)
{
#ifdef CONFIG_TEGRA_EMC_APE_DFS
	emc_dfs_exit();
#endif
	aram_exit();
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nvadsp_of_match[] = {
	{ .compatible = "nvidia,tegra210-adsp", .data = NULL, },
	{ .compatible = "nvidia,tegra18x-adsp", .data = NULL, },
	{},
};
#endif

static struct platform_driver nvadsp_driver __refdata = {
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
