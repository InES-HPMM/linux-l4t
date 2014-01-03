/*
 * tegra210_peq.c - Tegra PEQ driver.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 *
 * Copyright (C) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/*
#define VERBOSE_DEBUG 1
#define DEBUG 1
*/

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_peq.h"
#include "tegra210_axbar.h"

#define DRV_NAME "tegra210-peq"

/* TEGRA210 PEQ driver context */
struct tegra210_peq_ctx {
	struct device		*dev;
	void __iomem		*regs;
	struct regmap		*regmap;

	int			id;
};

static struct tegra210_peq_ctx *tegra210_peq[TEGRA210_PEQ_COUNT];

/* API to enable/disable PEQ clock */
void tegra210_peq_enable_clk(enum tegra210_ahub_cifs cif)
{
	pm_runtime_get_sync(tegra210_peq[OPE_ID(cif)]->dev);
}
EXPORT_SYMBOL_GPL(tegra210_peq_enable_clk);

void tegra210_peq_disable_clk(enum tegra210_ahub_cifs cif)
{
	pm_runtime_put_sync(tegra210_peq[OPE_ID(cif)]->dev);
}
EXPORT_SYMBOL_GPL(tegra210_peq_disable_clk);

int tegra210_peq_is_slcg_enable(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];
	u32 val = 0;

	regmap_read(peq->regmap, TEGRA210_PEQ_STATUS, &val);
	val &= TEGRA210_PEQ_STATUS_SLCG_CLKEN;

	dev_vdbg(peq->dev, "%s PEQ SLCG enable %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_peq_is_slcg_enable);

int tegra210_peq_is_config_err(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];
	u32 val = 0;

	regmap_read(peq->regmap, TEGRA210_PEQ_STATUS, &val);
	val &= TEGRA210_PEQ_STATUS_CONFIG_ERR;

	dev_vdbg(peq->dev, "%s PEQ Config error %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_peq_is_config_err);

/* Master soft reset for PEQ */
int tegra210_peq_reset(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];
	int cnt = AHUB_OP_MAX_RETRY;
	u32 val = 0;

	dev_vdbg(peq->dev, "%s", __func__);

	regmap_update_bits(peq->regmap,
			   TEGRA210_PEQ_SOFT_RST,
			   TEGRA210_PEQ_SOFT_RST_RESET,
			   TEGRA210_PEQ_SOFT_RST_RESET);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(peq->regmap, TEGRA210_PEQ_SOFT_RST, &val);
	} while ((val & TEGRA210_PEQ_SOFT_RST_RESET) && cnt--);

	if (!cnt) {
		dev_warn(peq->dev, "%s Failed to reset PEQ", __func__);
		return -ETIME;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_peq_reset);

/* Enable second level clock gating for PEQ */
int tegra210_peq_set_slcg(enum tegra210_ahub_cifs cif, int en)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];

	dev_vdbg(peq->dev, "%s PEQ slcg enable %d", __func__, en);

	regmap_update_bits(peq->regmap,
			   TEGRA210_PEQ_CG,
			   TEGRA210_PEQ_CG_SLCG_EN,
			   en ? TEGRA210_PEQ_CG_SLCG_EN : 0);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_peq_set_slcg);

int tegra210_peq_set_active(enum tegra210_ahub_cifs cif, int active)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];

	dev_vdbg(peq->dev, "%s PEQ set active %d", __func__, active);

	regmap_update_bits(peq->regmap,
		TEGRA210_PEQ_CONFIG,
		TEGRA210_PEQ_CONFIG_MODE_MASK,
		active ? TEGRA210_PEQ_CONFIG_MODE_ACTIVE :
			 TEGRA210_PEQ_CONFIG_MODE_BYPASS);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_peq_set_active);

/* APIs to set PEQ filter parameters */
int tegra210_peq_set_biquad_stages(enum tegra210_ahub_cifs cif, int biq_stages)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];

	dev_vdbg(peq->dev, "%s PEQ biquad stages %d", __func__, biq_stages);

	regmap_update_bits(peq->regmap,
		TEGRA210_PEQ_CONFIG,
		TEGRA210_PEQ_CONfIG_BIQUAD_STAGES_MASK,
		(biq_stages - 1) << TEGRA210_PEQ_CONFIG_BIQUAD_STAGES_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_peq_set_biquad_stages);

int tegra210_peq_write_coeff_params(enum tegra210_ahub_cifs cif,
				    struct peq_coeff_params *params,
				    int channel_id)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];
	u32 start_addr = 0;
	int i = 0;
	u32 biq_stages;

	regmap_read(peq->regmap,
		TEGRA210_PEQ_CONFIG, &biq_stages);
	biq_stages &= TEGRA210_PEQ_CONfIG_BIQUAD_STAGES_MASK;
	biq_stages = (biq_stages >> TEGRA210_PEQ_CONFIG_BIQUAD_STAGES_SHIFT);

	for (i = biq_stages + 1; i < TEGRA210_PEQ_MAX_BIQ_STAGES; i++) {
		params->biq[i].biq_b0 = 1;
		params->biq[i].biq_b1 = 0;
		params->biq[i].biq_b2 = 0;
		params->biq[i].biq_a1 = 0;
		params->biq[i].biq_a2 = 0;
	}
	start_addr = channel_id * TEGRA210_PEQ_COEFF_DATA_SIZE_PER_CH;

	dev_vdbg(peq->dev, "%s PEQ ch %d", __func__, channel_id);

	tegra210_ahubram_write(peq->dev,
			TEGRA210_PEQ_AHUBRAMCTL_PEQ_CTRL,
			TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA,
			start_addr,
			(u32 *)params,
			TEGRA210_PEQ_COEFF_DATA_SIZE_PER_CH);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_peq_write_coeff_params);

int tegra210_peq_write_shift_params(enum tegra210_ahub_cifs cif,
				    struct peq_shift_params *params,
				    int channel_id)
{
	int id = OPE_ID(cif);
	struct tegra210_peq_ctx *peq = tegra210_peq[id];
	u32 start_addr = 0;
	int i;
	u32 biq_stages;

	regmap_read(peq->regmap,
		TEGRA210_PEQ_CONFIG, &biq_stages);
	biq_stages &= TEGRA210_PEQ_CONfIG_BIQUAD_STAGES_MASK;
	biq_stages = (biq_stages >> TEGRA210_PEQ_CONFIG_BIQUAD_STAGES_SHIFT);
	for (i = biq_stages + 1; i < TEGRA210_PEQ_MAX_BIQ_STAGES; i++)
		params->biq_shift[i] = 0;

	start_addr = channel_id * TEGRA210_PEQ_SHIFT_DATA_SIZE_PER_CH;

	dev_vdbg(peq->dev, "%s PEQ ch %d", __func__, channel_id);

	tegra210_ahubram_write(peq->dev,
			TEGRA210_PEQ_AHUBRAMCTL_SHIFT_CTRL,
			TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA,
			start_addr,
			(u32 *)params,
			TEGRA210_PEQ_SHIFT_DATA_SIZE_PER_CH);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_peq_write_shift_params);

/* Regmap callback functions */
static bool tegra210_peq_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_PEQ_SOFT_RST:
	case TEGRA210_PEQ_CG:
	case TEGRA210_PEQ_CONFIG:
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_CTRL:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_CTRL:
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_peq_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_PEQ_SOFT_RST:
	case TEGRA210_PEQ_CG:
	case TEGRA210_PEQ_STATUS:
	case TEGRA210_PEQ_CONFIG:
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_CTRL:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_CTRL:
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_peq_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_PEQ_SOFT_RST:
	case TEGRA210_PEQ_STATUS:
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_CTRL:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_CTRL:
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_peq_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA:
	case TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_peq_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_PEQ_MAX_REGISTER,
	.writeable_reg = tegra210_peq_wr_reg,
	.readable_reg = tegra210_peq_rd_reg,
	.volatile_reg = tegra210_peq_volatile_reg,
	.precious_reg = tegra210_peq_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_peq_runtime_suspend(struct device *dev)
{
	struct tegra210_peq_ctx *peq = dev_get_drvdata(dev);

	dev_dbg(peq->dev, "%s PEQ %d", __func__, peq->id);
	regcache_cache_only(peq->regmap, true);
	tegra210_axbar_disable_clk();

	return 0;
}

static int tegra210_peq_runtime_resume(struct device *dev)
{
	struct tegra210_peq_ctx *peq = dev_get_drvdata(dev);

	dev_dbg(peq->dev, "%s PEQ %d", __func__, peq->id);

	tegra210_axbar_enable_clk();
	regcache_cache_only(peq->regmap, false);

	return 0;
}

/*
PEQ Driver probe and remove functions
*/
static int tegra210_peq_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_peq_ctx *peq;
	int ret = 0;
	int id = 0;

	if (!pdev->dev.of_node) {
		pr_err("%s Probe failed. DT node missing.", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-peq-id",
			     &pdev->id);

	dev_dbg(&pdev->dev, "%s PEQ %d : starting probe",
		__func__, pdev->id);

	if ((pdev->id < 0) ||
	    (pdev->id >= TEGRA210_PEQ_COUNT)) {
		dev_err(&pdev->dev, "PEQ id %d out of range\n", pdev->id);
		return -EINVAL;
	}
	id = pdev->id;

	tegra210_peq[id] = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_peq_ctx),
			   GFP_KERNEL);
	if (!tegra210_peq[id]) {
		dev_err(&pdev->dev, "Can't allocate peq context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_peq[id]->dev = &pdev->dev;

	peq = tegra210_peq[id];
	peq->id = id;
	dev_set_drvdata(&pdev->dev, peq);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory 0 resource\n");
		ret = -ENODEV;
		goto err_free;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region 0 already claimed\n");
		ret = -EBUSY;
		goto err_free;
	}

	peq->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!peq->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	peq->regmap = devm_regmap_init_mmio(&pdev->dev, peq->regs,
					    &tegra210_peq_regmap_config);
	if (IS_ERR(peq->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(peq->regmap);
		goto err_free;
	}
	regcache_cache_only(peq->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_peq_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(peq->dev, "%s PEQ %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_peq[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s PEQ %d probe failed with %d",
		__func__, pdev->id, ret);
	return ret;
}


static int tegra210_peq_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_peq_runtime_suspend(&pdev->dev);

	tegra210_peq[pdev->id] = NULL;

	dev_dbg(&pdev->dev, "%s PEQ %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_peq_of_match[] = {
	{ .compatible = "nvidia,tegra210-peq",},
	{},
};

static const struct dev_pm_ops tegra210_peq_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_peq_runtime_suspend,
			   tegra210_peq_runtime_resume, NULL)
};

static struct platform_driver tegra210_peq_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_peq_of_match,
		.pm = &tegra210_peq_pm_ops,
	},
	.probe = tegra210_peq_probe,
	.remove = tegra210_peq_remove,
};
module_platform_driver(tegra210_peq_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra APE PEQ driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
