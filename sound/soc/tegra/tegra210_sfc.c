/*
 * tegra210_sfc.c - Tegra210 SFC driver.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
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

#include "tegra210_sfc.h"
#include "tegra210_axbar.h"
#include "tegra210_ahub_utils.h"

#define DRV_NAME "tegra210-sfc"

/* TEGRA210 SFC driver context */
struct tegra210_sfc_ctx {
	struct device		*dev;
	void __iomem		*regs;
	struct regmap		*regmap;
	int			id;
	bool			in_use;
};
static struct tegra210_sfc_ctx *tegra210_sfc[TEGRA210_SFC_COUNT];

/* SFC APIs definition */
/**
 *  tegra210_sfc_get - gets a sfc instance before use
 *  @cif : sfc cif (pass CIF_NONE to get first available sfc)
 */
int tegra210_sfc_get(enum tegra210_ahub_cifs *cif)
{
	enum tegra210_cif_dir dir = IS_SFC_RX(*cif) ? DIR_RX : DIR_TX;
	int i, id = SFC_ID(*cif);
	struct tegra210_sfc_ctx *sfc;

	if ((id >= 0) && (id < TEGRA210_SFC_COUNT)) {
		sfc = tegra210_sfc[id];
		if (!sfc->in_use) {
			dev_vdbg(sfc->dev, "%s allocated sfc %d", __func__, id);
			sfc->in_use = true;
			pm_runtime_get_sync(sfc->dev);
			return 0;
		} else {
			dev_err(sfc->dev, "%s sfc %d not free", __func__, id);
			return -EBUSY;
		}
	}

	for (i = 0; i < TEGRA210_SFC_COUNT; i++) {
		sfc = tegra210_sfc[i];
		if (!sfc->in_use) {
			dev_vdbg(sfc->dev, "%s allocated sfc %d", __func__, i);
			sfc->in_use = true;
			*cif = (dir == DIR_RX) ? (SFC1_RX0 + i) :
						 (SFC1_TX0 + i);
			pm_runtime_get_sync(sfc->dev);
			return 0;
		}
	}

	pr_err("%s allocation failed, no sfc is free", __func__);
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_get);

/**
 *  tegra210_sfc_put - frees a sfc instance after use
 *  @cif : sfc cif
 */
int tegra210_sfc_put(enum tegra210_ahub_cifs cif)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];

	if (sfc->in_use) {
		dev_vdbg(sfc->dev, "%s freed sfc %d", __func__, id);
		sfc->in_use = false;
		pm_runtime_put_sync(sfc->dev);
	} else
		dev_info(sfc->dev, "%s sfc %d already free", __func__, id);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_put);

/**
 *  tegra210_sfc_set_acif_param - sets acif params for a sfc
 *  @cif : sfc cif
 *  @acif : acif param values
 */
int tegra210_sfc_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];
	u32 reg;

	dev_vdbg(sfc->dev, "%s sfc cif %d", __func__, cif);

	reg = IS_SFC_RX(cif) ? TEGRA210_SFC_RX_CIF_CTRL :
				TEGRA210_SFC_TX_CIF_CTRL;

	return tegra210_set_axbar_cif_param(sfc->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_sfc_set_acif_param);

/**
 *  tegra210_sfc_get_acif_param - gets acif params for a sfc
 *  @cif : sfc cif
 *  @acif : acif ptr to be populated
 */
int tegra210_sfc_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];
	u32 reg;

	dev_vdbg(sfc->dev, "%s sfc cif %d", __func__, cif);

	reg = IS_SFC_RX(cif) ? TEGRA210_SFC_RX_CIF_CTRL :
				TEGRA210_SFC_TX_CIF_CTRL;

	return tegra210_get_axbar_cif_param(sfc->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_sfc_get_acif_param);

/**
 *  tegra210_sfc_set_rx_rate - sets input sample rate for a sfc
 *  @cif : sfc rxcif
 *  @fs_in : input sampling frequency
 */
int tegra210_sfc_set_rx_rate(enum tegra210_ahub_cifs cif, int fs_in)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];
	u32 val = 0;

	dev_vdbg(sfc->dev, "%s sfc %d fs_in %d", __func__, id, fs_in);

	switch (fs_in) {
	case 8000: val = TEGRA210_SFC_FS_8000HZ; break;
	case 11025: val = TEGRA210_SFC_FS_11025HZ; break;
	case 16000: val = TEGRA210_SFC_FS_16000HZ; break;
	case 22050: val = TEGRA210_SFC_FS_22050HZ; break;
	case 24000: val = TEGRA210_SFC_FS_24000HZ; break;
	case 32000: val = TEGRA210_SFC_FS_32000HZ; break;
	case 44100: val = TEGRA210_SFC_FS_44100HZ; break;
	case 48000: val = TEGRA210_SFC_FS_48000HZ; break;
	case 64000: val = TEGRA210_SFC_FS_64000HZ; break;
	case 88200: val = TEGRA210_SFC_FS_88200HZ; break;
	case 96000: val = TEGRA210_SFC_FS_96000HZ; break;
	case 176400: val = TEGRA210_SFC_FS_176400HZ; break;
	case 192000: val = TEGRA210_SFC_FS_192000HZ; break;
	default:
		dev_err(sfc->dev, "Invalid rx sample rate %d", fs_in);
		return -EINVAL;
	}

	regmap_update_bits(sfc->regmap,
				TEGRA210_SFC_RX_FREQ,
				TEGRA210_SFC_RX_FREQ_FS_IN_MASK,
				val << TEGRA210_SFC_RX_FREQ_FS_IN_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_set_rx_rate);

/**
 *  tegra210_sfc_set_tx_rate - set output sample rate for a sfc
 *  @cif : sfc txcif
 *  @fs_out : output sampling frequency
 */
int tegra210_sfc_set_tx_rate(enum tegra210_ahub_cifs cif, int fs_out)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];
	u32 val = 0;

	dev_vdbg(sfc->dev, "%s sfc %d fs_out %d", __func__, id, fs_out);

	switch (fs_out) {
	case 8000: val = TEGRA210_SFC_FS_8000HZ; break;
	case 11025: val = TEGRA210_SFC_FS_11025HZ; break;
	case 16000: val = TEGRA210_SFC_FS_16000HZ; break;
	case 22050: val = TEGRA210_SFC_FS_22050HZ; break;
	case 24000: val = TEGRA210_SFC_FS_24000HZ; break;
	case 32000: val = TEGRA210_SFC_FS_32000HZ; break;
	case 44100: val = TEGRA210_SFC_FS_44100HZ; break;
	case 48000: val = TEGRA210_SFC_FS_48000HZ; break;
	case 64000: val = TEGRA210_SFC_FS_64000HZ; break;
	case 88200: val = TEGRA210_SFC_FS_88200HZ; break;
	case 96000: val = TEGRA210_SFC_FS_96000HZ; break;
	case 176400: val = TEGRA210_SFC_FS_176400HZ; break;
	case 192000: val = TEGRA210_SFC_FS_192000HZ; break;
	default:
		dev_err(sfc->dev, "Invalid tx sample rate %d", fs_out);
		return -EINVAL;
	}

	regmap_update_bits(sfc->regmap,
				TEGRA210_SFC_TX_FREQ,
				TEGRA210_SFC_TX_FREQ_FS_OUT_MASK,
				val << TEGRA210_SFC_TX_FREQ_FS_OUT_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_set_tx_rate);

/**
 *  tegra210_sfc_enable - enables/disables a sfc
 *  @cif : sfc cif
 *  @en : enable/disable flag
 */
int tegra210_sfc_enable(enum tegra210_ahub_cifs cif, bool en)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];

	dev_vdbg(sfc->dev, "%s sfc %d enable %d", __func__, id, en);

	regmap_update_bits(sfc->regmap,
				TEGRA210_SFC_ENABLE,
				TEGRA210_SFC_ENABLE_EN,
				en ? TEGRA210_SFC_ENABLE_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_enable);

/**
 *  tegra210_sfc_reset - resets a sfc
 *  @cif : sfc cif
 */
int tegra210_sfc_reset(enum tegra210_ahub_cifs cif)
{
	int cnt = AHUB_OP_MAX_RETRY, id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];
	u32 val = 0;

	dev_vdbg(sfc->dev, "%s reset sfc %d", __func__, id);

	regmap_update_bits(sfc->regmap,
				TEGRA210_SFC_SOFT_RESET,
				TEGRA210_SFC_SOFT_RESET_EN,
				TEGRA210_SFC_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(sfc->regmap, TEGRA210_SFC_SOFT_RESET, &val);
	} while ((val & TEGRA210_SFC_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(sfc->dev, "%s Failed to reset sfc %d", __func__, id);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_reset);

/**
 *  tegra210_sfc_set_slcg - enables/disables second level clock gating for a sfc
 *  @cif : sfc cif
 *  @en : enable/disable flag
 */
int tegra210_sfc_set_slcg(enum tegra210_ahub_cifs cif, bool en)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];

	dev_vdbg(sfc->dev, "%s sfc %d enable %d", __func__, id, en);

	regmap_update_bits(sfc->regmap,
				TEGRA210_SFC_CG,
				TEGRA210_SFC_CG_SLCG_EN,
				en ? TEGRA210_SFC_CG_SLCG_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_set_slcg);

/**
 *  tegra210_sfc_enable_coef_ram - enables coefficient ram params for a sfc
 *  @cif : sfc cif
 *  @coef : ptr to coefficients (pass NULL while disabling, i.e. when en = 0)
 *  @en : enable/disable flag
 */
int tegra210_sfc_enable_coef_ram(enum tegra210_ahub_cifs cif,
					u32 *coef, bool en)
{
	int id = SFC_ID(cif);
	struct tegra210_sfc_ctx *sfc = tegra210_sfc[id];

	dev_vdbg(sfc->dev, "%s sfc %d enable %d", __func__, id, en);

	if (en)
		tegra210_ahubram_write(sfc->dev,
					TEGRA210_SFC_ARAMCTL_COEF_CTRL,
					TEGRA210_SFC_ARAMCTL_COEF_DATA,
					0, coef, TEGRA210_SFC_COEF_RAM_DEPTH);

	regmap_update_bits(sfc->regmap,
				TEGRA210_SFC_COEF_RAM,
				TEGRA210_SFC_COEF_RAM_COEF_RAM_EN,
				en ? TEGRA210_SFC_COEF_RAM_COEF_RAM_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_enable_coef_ram);

/* Regmap callback functions */
static bool tegra210_sfc_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_RX_INT_MASK:
	case TEGRA210_SFC_RX_INT_SET:
	case TEGRA210_SFC_RX_INT_CLEAR:
	case TEGRA210_SFC_RX_CIF_CTRL:
	case TEGRA210_SFC_RX_FREQ:

	case TEGRA210_SFC_TX_INT_MASK:
	case TEGRA210_SFC_TX_INT_SET:
	case TEGRA210_SFC_TX_INT_CLEAR:
	case TEGRA210_SFC_TX_CIF_CTRL:
	case TEGRA210_SFC_TX_FREQ:

	case TEGRA210_SFC_ENABLE:
	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_CG:
	case TEGRA210_SFC_COEF_RAM:
	case TEGRA210_SFC_ARAMCTL_COEF_CTRL:
	case TEGRA210_SFC_ARAMCTL_COEF_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_sfc_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_RX_STATUS:
	case TEGRA210_SFC_RX_INT_STATUS:
	case TEGRA210_SFC_RX_INT_MASK:
	case TEGRA210_SFC_RX_INT_SET:
	case TEGRA210_SFC_RX_INT_CLEAR:
	case TEGRA210_SFC_RX_CIF_CTRL:
	case TEGRA210_SFC_RX_FREQ:

	case TEGRA210_SFC_TX_STATUS:
	case TEGRA210_SFC_TX_INT_STATUS:
	case TEGRA210_SFC_TX_INT_MASK:
	case TEGRA210_SFC_TX_INT_SET:
	case TEGRA210_SFC_TX_INT_CLEAR:
	case TEGRA210_SFC_TX_CIF_CTRL:
	case TEGRA210_SFC_TX_FREQ:

	case TEGRA210_SFC_ENABLE:
	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_CG:
	case TEGRA210_SFC_STATUS:
	case TEGRA210_SFC_INT_STATUS:
	case TEGRA210_SFC_COEF_RAM:
	case TEGRA210_SFC_ARAMCTL_COEF_CTRL:
	case TEGRA210_SFC_ARAMCTL_COEF_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_sfc_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_RX_STATUS:
	case TEGRA210_SFC_RX_INT_STATUS:
	case TEGRA210_SFC_RX_INT_SET:

	case TEGRA210_SFC_TX_STATUS:
	case TEGRA210_SFC_TX_INT_STATUS:
	case TEGRA210_SFC_TX_INT_SET:

	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_STATUS:
	case TEGRA210_SFC_INT_STATUS:
	case TEGRA210_SFC_ARAMCTL_COEF_CTRL:
	case TEGRA210_SFC_ARAMCTL_COEF_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_sfc_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_ARAMCTL_COEF_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_sfc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_SFC_ARAMCTL_COEF_DATA,
	.writeable_reg = tegra210_sfc_wr_reg,
	.readable_reg = tegra210_sfc_rd_reg,
	.volatile_reg = tegra210_sfc_volatile_reg,
	.precious_reg = tegra210_sfc_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_sfc_runtime_suspend(struct device *dev)
{
	struct tegra210_sfc_ctx *sfc = dev_get_drvdata(dev);

	dev_dbg(sfc->dev, "%s sfc %d", __func__, sfc->id);
	tegra210_axbar_disable_clk();
	regcache_cache_only(sfc->regmap, true);

	return 0;
}

static int tegra210_sfc_runtime_resume(struct device *dev)
{
	struct tegra210_sfc_ctx *sfc = dev_get_drvdata(dev);

	dev_dbg(sfc->dev, "%s sfc %d", __func__, sfc->id);
	tegra210_axbar_enable_clk();
	regcache_cache_only(sfc->regmap, false);

	return 0;
}

/* SFC Driver probe and remove functions */
static int tegra210_sfc_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_sfc_ctx *sfc;
	int ret = 0;
	int id = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, " %s failed, no DT node for sfc", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-sfc-id",
						 &pdev->id);

	dev_dbg(&pdev->dev, "%s sfc %d : starting probe",
		__func__, pdev->id);

	if ((pdev->id < 0) ||
		(pdev->id >= TEGRA210_SFC_COUNT)) {
		dev_err(&pdev->dev, "sfc id %d out of range\n", pdev->id);
		return -EINVAL;
	}
	id = pdev->id;

	tegra210_sfc[id] = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_sfc_ctx), GFP_KERNEL);
	if (!tegra210_sfc[id]) {
		dev_err(&pdev->dev, "Can't allocate sfc context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_sfc[id]->dev = &pdev->dev;

	sfc = tegra210_sfc[id];
	sfc->id = id;
	dev_set_drvdata(&pdev->dev, sfc);

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

	sfc->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!sfc->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	sfc->regmap = devm_regmap_init_mmio(&pdev->dev, sfc->regs,
					    &tegra210_sfc_regmap_config);
	if (IS_ERR(sfc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(sfc->regmap);
		goto err_free;
	}
	regcache_cache_only(sfc->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_sfc_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(sfc->dev, "%s sfc %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_sfc[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s sfc %d probe failed with %d",
		__func__, pdev->id, ret);
	return ret;
}

static int tegra210_sfc_remove(struct platform_device *pdev)
{
	struct tegra210_sfc_ctx *sfc;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_sfc_runtime_suspend(&pdev->dev);

	sfc = platform_get_drvdata(pdev);
	tegra210_sfc[pdev->id] = NULL;

	dev_dbg(&pdev->dev, "%s sfc %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_sfc_of_match[] = {
	{ .compatible = "nvidia,tegra210-sfc",},
	{},
};

static const struct dev_pm_ops tegra210_sfc_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_sfc_runtime_suspend,
			   tegra210_sfc_runtime_resume, NULL)
};

static struct platform_driver tegra210_sfc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_sfc_of_match,
		.pm = &tegra210_sfc_pm_ops,
	},
	.probe = tegra210_sfc_probe,
	.remove = tegra210_sfc_remove,
};
module_platform_driver(tegra210_sfc_driver);

MODULE_AUTHOR("Rahul Mittal <rmittal@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 SFC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
