/*
 * tegra210_dmic.c - Tegra210 DMIC driver.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
 *
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
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
#define VERBOSE_DEBUG
#define DEBUG
*/

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_dmic.h"
#include "tegra210_axbar.h"
#include "tegra210_ahub_utils.h"

#define DRV_NAME "tegra210-dmic"

/* TEGRA210 DMIC driver context */
struct tegra210_dmic_ctx {
	struct device		*dev;
	void __iomem		*regs;
	struct regmap		*regmap;
	int			id;
	bool			in_use;
};
static struct tegra210_dmic_ctx *tegra210_dmic[TEGRA210_DMIC_COUNT];

/* DMIC APIs definition */
/**
 *  tegra210_dmic_get - gets a dmic instance before use
 *  @cif : dmic txcif (pass CIF_NONE to get first available dmic)
 */
int tegra210_dmic_get(enum tegra210_ahub_cifs *cif)
{
	int i, id = DMIC_ID(*cif);
	struct tegra210_dmic_ctx *dmic;

	if ((id >= 0) && (id < TEGRA210_DMIC_COUNT)) {
		dmic = tegra210_dmic[id];
		if (!dmic->in_use) {
			dev_vdbg(dmic->dev, "%s allocated dmic %d",
					__func__, id);
			dmic->in_use = true;
			pm_runtime_get_sync(dmic->dev);
			return 0;
		} else {
			dev_err(dmic->dev, "%s dmic %d not free", __func__, id);
			return -EBUSY;
		}
	}

	for (i = 0; i < TEGRA210_DMIC_COUNT; i++) {
		dmic = tegra210_dmic[i];
		if (!dmic->in_use) {
			dev_vdbg(dmic->dev, "%s allocated dmic %d",
					__func__, i);
			dmic->in_use = true;
			*cif = DMIC1_TX0 + i;
			pm_runtime_get_sync(dmic->dev);
			return 0;
		}
	}

	pr_err("%s allocation failed, no dmic is free", __func__);
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_get);

/**
 *  tegra210_dmic_put - frees a dmic instance after use
 *  @cif : dmic txcif
 */
int tegra210_dmic_put(enum tegra210_ahub_cifs cif)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	if (dmic->in_use) {
		dev_vdbg(dmic->dev, "%s freed dmic %d", __func__, id);
		dmic->in_use = false;
		pm_runtime_put_sync(dmic->dev);
	} else
		dev_info(dmic->dev, "%s dmic %d already free", __func__, id);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_put);

/**
 *  tegra210_dmic_set_acif_param - sets acif params for a dmic
 *  @cif : dmic txcif
 *  @acif : acif param values
 */
int tegra210_dmic_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];
	u32 reg = TEGRA210_DMIC_TX_CIF_CTRL;

	dev_vdbg(dmic->dev, "%s dmic cif %d", __func__, cif);

	return tegra210_set_axbar_cif_param(dmic->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_dmic_set_acif_param);

/**
 *  tegra210_dmic_get_acif_param - gets acif params for a dmic
 *  @cif : dmic txcif
 *  @acif : acif ptr to be populated
 */
int tegra210_dmic_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];
	u32 reg = TEGRA210_DMIC_TX_CIF_CTRL;

	dev_vdbg(dmic->dev, "%s dmic cif %d", __func__, cif);

	return tegra210_get_axbar_cif_param(dmic->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_dmic_get_acif_param);

/**
 *  tegra210_dmic_enable - enables/disables a dmic
 *  @cif : dmic txcif
 *  @en : enable/disable flag
 */
int tegra210_dmic_enable(enum tegra210_ahub_cifs cif, bool en)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d enable %d", __func__, id, en);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_ENABLE,
				TEGRA210_DMIC_ENABLE_EN,
				en ? TEGRA210_DMIC_ENABLE_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_enable);

/**
 *  tegra210_dmic_reset - resets a dmic
 *  @cif : dmic txcif
 */
int tegra210_dmic_reset(enum tegra210_ahub_cifs cif)
{
	int cnt = AHUB_OP_MAX_RETRY, id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];
	u32 val = 0;

	dev_vdbg(dmic->dev, "%s reset dmic %d", __func__, id);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_SOFT_RESET,
				TEGRA210_DMIC_SOFT_RESET_EN,
				TEGRA210_DMIC_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(dmic->regmap, TEGRA210_DMIC_SOFT_RESET, &val);
	} while ((val & TEGRA210_DMIC_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(dmic->dev, "%s Failed to reset dmic %d", __func__, id);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_reset);

/**
 *  tegra210_dmic_set_slcg - enables/disables second level clock gating for a dmic
 *  @cif : dmic txcif
 *  @en : enable/disable flag
 */
int tegra210_dmic_set_slcg(enum tegra210_ahub_cifs cif, bool en)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d enable %d", __func__, id, en);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_CG,
				TEGRA210_DMIC_CG_SLCG_EN,
				en ? TEGRA210_DMIC_CG_SLCG_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_set_slcg);

/**
 *  tegra210_dmic_set_osr - sets oversampling ratio for a dmic
 *  @cif : dmic txcif
 *  @osr : oversampling ratio (64, 128 or 256)
 */
int tegra210_dmic_set_osr(enum tegra210_ahub_cifs cif, int osr)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];
	u32 val;

	dev_vdbg(dmic->dev, "%s dmic %d osr %d", __func__, id, osr);

	switch (osr) {
	case 64:
		val = TEGRA210_DMIC_OSR_64;
		break;
	case 128:
		val = TEGRA210_DMIC_OSR_128;
		break;
	case 256:
		val = TEGRA210_DMIC_OSR_256;
		break;
	default:
		dev_err(dmic->dev, "Invalid OSR value %d", osr);
		return -EINVAL;
	}

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_CTRL,
				TEGRA210_DMIC_CTRL_OSR_MASK,
				val << TEGRA210_DMIC_CTRL_OSR_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_set_osr);

/**
 *  tegra210_dmic_set_lr_polarity - sets channel polarity for a dmic (LR or RL)
 *  @cif : dmic txcif
 *  @pol : polarity (0 = LR, 1 = RL)
 */
int tegra210_dmic_set_lr_polarity(enum tegra210_ahub_cifs cif, int pol)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d polarity %d", __func__, id, pol);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_CTRL,
				TEGRA210_DMIC_CTRL_LRSEL_POLARITY_MASK,
				pol << TEGRA210_DMIC_CTRL_LRSEL_POLARITY_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_set_lr_polarity);

/**
 *  tegra210_dmic_set_cap_channels - sets capture channels for a dmic
 *  @cif : dmic txcif
 *  @ch : channel count (0 = none, 1 = left, 2 = right, 3 = stereo)
 */
int tegra210_dmic_set_cap_channels(enum tegra210_ahub_cifs cif, int ch)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d ch %d", __func__, id, ch);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_CTRL,
				TEGRA210_DMIC_CTRL_CHANNEL_SELECT_MASK,
				ch << TEGRA210_DMIC_CTRL_CHANNEL_SELECT_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_set_cap_channels);

/**
 *  tegra210_dmic_set_trimmer_sel - sets trim value for alignment for a dmic
 *  @cif : dmic txcif
 *  @trim : trim value
 */
int tegra210_dmic_set_trimmer_sel(enum tegra210_ahub_cifs cif, int trim)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d trim %d", __func__, id, trim);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_CTRL,
				TEGRA210_DMIC_CTRL_TRIMMER_SEL_MASK,
				trim << TEGRA210_DMIC_CTRL_TRIMMER_SEL_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_set_trimmer_sel);

/**
 *  tegra210_dmic_enable_sc - enables/disables sinc correction for a dmic
 *  @cif : dmic txcif
 *  @en : enable/disable flag
 */
int tegra210_dmic_enable_sc(enum tegra210_ahub_cifs cif, bool en)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d enable %d", __func__, id, en);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_DBG_CTRL,
				TEGRA210_DMIC_DBG_CTRL_SC_ENABLE,
				en ? TEGRA210_DMIC_DBG_CTRL_SC_ENABLE : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_enable_sc);

/**
 *  tegra210_dmic_enable_dcr - enables/disables DC removal filter for a dmic
 *  @cif : dmic txcif
 *  @en : enable/disable flag
 */
int tegra210_dmic_enable_dcr(enum tegra210_ahub_cifs cif, bool en)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d enable %d", __func__, id, en);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_DBG_CTRL,
				TEGRA210_DMIC_DBG_CTRL_DCR_ENABLE,
				en ? TEGRA210_DMIC_DBG_CTRL_DCR_ENABLE : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_enable_dcr);

/**
 *  tegra210_dmic_enable_lp - enables/disables low pass filter on dcr output for a dmic
 *  @cif : dmic txcif
 *  @en : enable/disable flag
 */
int tegra210_dmic_enable_lp(enum tegra210_ahub_cifs cif, bool en)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d enable %d", __func__, id, en);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_DBG_CTRL,
				TEGRA210_DMIC_DBG_CTRL_LP_ENABLE,
				en ? TEGRA210_DMIC_DBG_CTRL_LP_ENABLE : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_enable_lp);

/**
 *  tegra210_dmic_enable_bypass - enables/disables bypassing raw mic data for a dmic
 *  @cif : dmic txcif
 *  @en : enable/disable flag
 */
int tegra210_dmic_enable_bypass(enum tegra210_ahub_cifs cif, bool en)
{
	int id = DMIC_ID(cif);
	struct tegra210_dmic_ctx *dmic = tegra210_dmic[id];

	dev_vdbg(dmic->dev, "%s dmic %d enable %d", __func__, id, en);

	regmap_update_bits(dmic->regmap,
				TEGRA210_DMIC_DBG_CTRL,
				TEGRA210_DMIC_DBG_CTRL_BYPASS,
				en ? TEGRA210_DMIC_DBG_CTRL_BYPASS : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_dmic_enable_bypass);

/* Regmap callback functions */
static bool tegra210_dmic_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_DMIC_TX_INT_MASK:
	case TEGRA210_DMIC_TX_INT_SET:
	case TEGRA210_DMIC_TX_INT_CLEAR:
	case TEGRA210_DMIC_TX_CIF_CTRL:

	case TEGRA210_DMIC_ENABLE:
	case TEGRA210_DMIC_SOFT_RESET:
	case TEGRA210_DMIC_CG:
	case TEGRA210_DMIC_CTRL:
		return true;
	default:
		if (((reg % 4) == 0) && (reg >= TEGRA210_DMIC_DBG_CTRL) &&
		    (reg <= TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4))
			return true;
		else
			return false;
	};
}

static bool tegra210_dmic_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_DMIC_TX_STATUS:
	case TEGRA210_DMIC_TX_INT_STATUS:
	case TEGRA210_DMIC_TX_INT_MASK:
	case TEGRA210_DMIC_TX_INT_SET:
	case TEGRA210_DMIC_TX_INT_CLEAR:
	case TEGRA210_DMIC_TX_CIF_CTRL:

	case TEGRA210_DMIC_ENABLE:
	case TEGRA210_DMIC_SOFT_RESET:
	case TEGRA210_DMIC_CG:
	case TEGRA210_DMIC_STATUS:
	case TEGRA210_DMIC_INT_STATUS:
	case TEGRA210_DMIC_CTRL:
		return true;
	default:
		if (((reg % 4) == 0) && (reg >= TEGRA210_DMIC_DBG_CTRL) &&
		    (reg <= TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4))
			return true;
		else
			return false;
	};
}

static bool tegra210_dmic_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_DMIC_TX_STATUS:
	case TEGRA210_DMIC_TX_INT_STATUS:
	case TEGRA210_DMIC_TX_INT_SET:

	case TEGRA210_DMIC_SOFT_RESET:
	case TEGRA210_DMIC_STATUS:
	case TEGRA210_DMIC_INT_STATUS:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_dmic_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4,
	.writeable_reg = tegra210_dmic_wr_reg,
	.readable_reg = tegra210_dmic_rd_reg,
	.volatile_reg = tegra210_dmic_volatile_reg,
	.precious_reg = NULL,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_dmic_runtime_suspend(struct device *dev)
{
	struct tegra210_dmic_ctx *dmic = dev_get_drvdata(dev);

	dev_dbg(dmic->dev, "%s dmic %d", __func__, dmic->id);
	tegra210_axbar_disable_clk();
	regcache_cache_only(dmic->regmap, true);

	return 0;
}

static int tegra210_dmic_runtime_resume(struct device *dev)
{
	struct tegra210_dmic_ctx *dmic = dev_get_drvdata(dev);

	dev_dbg(dmic->dev, "%s dmic %d", __func__, dmic->id);
	tegra210_axbar_enable_clk();
	regcache_cache_only(dmic->regmap, false);

	return 0;
}

/* DMIC driver probe and remove functions */
static int tegra210_dmic_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_dmic_ctx *dmic;
	int ret = 0;
	int id = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s failed, no DT node for dmic", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-dmic-id",
						 &pdev->id);

	dev_dbg(&pdev->dev, "%s dmic %d : starting probe",
		__func__, pdev->id);

	if ((pdev->id < 0) ||
		(pdev->id >= TEGRA210_DMIC_COUNT)) {
		dev_err(&pdev->dev, "dmic id %d out of range\n", pdev->id);
		return -EINVAL;
	}
	id = pdev->id;

	tegra210_dmic[id] = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_dmic_ctx), GFP_KERNEL);
	if (!tegra210_dmic[id]) {
		dev_err(&pdev->dev, "Can't allocate dmic context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_dmic[id]->dev = &pdev->dev;

	dmic = tegra210_dmic[id];
	dmic->id = id;
	dev_set_drvdata(&pdev->dev, dmic);

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

	dmic->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!dmic->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	dmic->regmap = devm_regmap_init_mmio(&pdev->dev, dmic->regs,
					    &tegra210_dmic_regmap_config);
	if (IS_ERR(dmic->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(dmic->regmap);
		goto err_free;
	}
	regcache_cache_only(dmic->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_dmic_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(dmic->dev, "%s dmic %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_dmic[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s dmic %d probe failed with %d",
		__func__, pdev->id, ret);
	return ret;
}

static int tegra210_dmic_remove(struct platform_device *pdev)
{
	struct tegra210_dmic_ctx *dmic;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_dmic_runtime_suspend(&pdev->dev);

	dmic = platform_get_drvdata(pdev);
	tegra210_dmic[pdev->id] = NULL;

	dev_dbg(&pdev->dev, "%s dmic %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_dmic_of_match[] = {
	{ .compatible = "nvidia,tegra210-dmic",},
	{},
};

static const struct dev_pm_ops tegra210_dmic_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_dmic_runtime_suspend,
			   tegra210_dmic_runtime_resume, NULL)
};

static struct platform_driver tegra210_dmic_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_dmic_of_match,
		.pm = &tegra210_dmic_pm_ops,
	},
	.probe = tegra210_dmic_probe,
	.remove = tegra210_dmic_remove,
};
module_platform_driver(tegra210_dmic_driver);

MODULE_AUTHOR("Rahul Mittal <rmittal@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 DMIC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
