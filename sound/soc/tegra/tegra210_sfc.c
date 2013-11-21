/*
 * tegra210_sfc.c - Tegra210 SFC driver.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
 *
 * Copyright (C) 2013, NVIDIA CORPORATION. All rights reserved.
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

#define GET_ACIF_REG(cif) \
	(IS_SFC_RX(cif)) ? TEGRA210_SFC_RX_CIF_CTRL : \
			      TEGRA210_SFC_TX_CIF_CTRL

#define VALIDATE_SFC_ID(id) \
	if ((id < 0) || (id >= TEGRA210_SFC_COUNT)) { \
		pr_err("%s Invalid SFC id %d", __func__, id); \
		return -EINVAL; }

/* TEGRA210 SFC driver context */
struct tegra210_sfc_ctx {
	struct device	*dev;
	void __iomem	*regs;
	struct regmap	*regmap;
	int				id;
	bool			in_use;
};
static struct tegra210_sfc_ctx *tegra210_sfc[TEGRA210_SFC_COUNT];

/* Regmap callback functions */
static bool tegra210_sfc_rw_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_RX_STATUS:
	case TEGRA210_SFC_RX_INT_STATUS:
	case TEGRA210_SFC_RX_INT_MASK:
	case TEGRA210_SFC_RX_INT_CLEAR:
	case TEGRA210_SFC_RX_CIF_CTRL:
	case TEGRA210_SFC_RX_FREQ:
	case TEGRA210_SFC_TX_STATUS:
	case TEGRA210_SFC_TX_INT_STATUS:
	case TEGRA210_SFC_TX_INT_MASK:
	case TEGRA210_SFC_TX_INT_CLEAR:
	case TEGRA210_SFC_TX_CIF_CTRL:
	case TEGRA210_SFC_TX_FREQ:
	case TEGRA210_SFC_ENABLE:
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
	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_RX_INT_SET:
	case TEGRA210_SFC_TX_INT_SET:
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
	.writeable_reg = tegra210_sfc_rw_reg,
	.readable_reg = tegra210_sfc_rw_reg,
	.volatile_reg = tegra210_sfc_volatile_reg,
	.precious_reg = tegra210_sfc_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* SFC APIs definition*/

/**
 *  tegra210_sfc_allocate - allocate a sfc
 *  @id : sfc id (pass id = -1 to allocate first available sfc)
 */
int tegra210_sfc_allocate(int *id)
{
	struct tegra210_sfc_ctx *sfc;
	int i;

	if ((*id >= 0) && (*id < TEGRA210_SFC_COUNT)) {
		sfc = tegra210_sfc[*id];
		pm_runtime_get_sync(sfc->dev);
		if (!sfc->in_use) {
			dev_vdbg(sfc->dev, "%s allocated SFC %d", __func__, *id);
			sfc->in_use = true;
			return 0;
		} else {
			dev_err(sfc->dev, "%s SFC %d is in use, can't allocate",
				__func__, *id);
			return -ENOMEM;
		}
	}

	for (i = 0; i < TEGRA210_SFC_COUNT; i++) {
		sfc = tegra210_sfc[i];
		pm_runtime_get_sync(sfc->dev);
		if (!sfc->in_use) {
			dev_vdbg(sfc->dev, "%s allocated SFC %d",
				 __func__, *id);
			sfc->in_use = true;
			*id = i;
			return 0;
		}
	}

	pr_err("%s allocation failed, no SFC is free", __func__);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_allocate);

/**
 *  tegra210_sfc_free - free a sfc
 *  @id : sfc id
 */
int tegra210_sfc_free(int id)
{
	struct tegra210_sfc_ctx *sfc;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	pm_runtime_put_sync(sfc->dev);
	if (sfc->in_use) {
		dev_vdbg(sfc->dev, "%s freed SFC %d", __func__, id);
		sfc->in_use = false;
	} else
		dev_info(sfc->dev, "%s SFC %d is already free", __func__, id);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_free);

/**
 *  tegra210_sfc_set_acif_param - set acif param for a sfc
 *  @cif : cif to be set
 *  @acif : acif param values
 */
int tegra210_sfc_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_sfc_ctx *sfc;
	int id = SFC_ID(cif);

	VALIDATE_SFC_ID(id)
	sfc = tegra210_sfc[id];

	dev_vdbg(sfc->dev, "%s SFC %d cif %d", __func__, id, cif);
	return tegra210_set_axbar_cif_param(sfc->dev, GET_ACIF_REG(cif), acif);
}
EXPORT_SYMBOL_GPL(tegra210_sfc_set_acif_param);

/**
 *  tegra210_sfc_get_acif_param - get acif param for a sfc
 *  @cif : cif to get
 *  @acif : acif struct to be populated
 */
int tegra210_sfc_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_sfc_ctx *sfc;
	int id = SFC_ID(cif);

	VALIDATE_SFC_ID(id)
	sfc = tegra210_sfc[id];

	dev_vdbg(sfc->dev, "%s SFC %d cif %d", __func__, id, cif);
	return tegra210_get_axbar_cif_param(sfc->dev, GET_ACIF_REG(cif), acif);
}
EXPORT_SYMBOL_GPL(tegra210_sfc_get_acif_param);

/**
 *  tegra210_sfc_set_rx_rate - set input sample rate for a sfc
 *  @id : sfc id
 *  @fs_in : input sampling frequency
 */
int tegra210_sfc_set_rx_rate(int id, int fs_in)
{
	struct tegra210_sfc_ctx *sfc;
	u32 val = 0;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	dev_vdbg(sfc->dev, "%s SFC %d fs_in %d", __func__, id, fs_in);

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
		dev_err(sfc->dev, "Invalid RX sample rate %d", fs_in);
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
 *  @id : sfc id
 *  @fs_out : output sampling frequency
 */
int tegra210_sfc_set_tx_rate(int id, int fs_out)
{
	struct tegra210_sfc_ctx *sfc;
	u32 val = 0;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	dev_vdbg(sfc->dev, "%s SFC %d fs_out %d", __func__, id, fs_out);

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
		dev_err(sfc->dev, "Invalid TX sample rate %d", fs_out);
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
 *  tegra210_sfc_enable - enable or disable a sfc
 *  @id : sfc id
 *  @en : enable/disable flag
 */
int tegra210_sfc_enable(int id, bool en)
{
	struct tegra210_sfc_ctx *sfc;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	dev_vdbg(sfc->dev, "%s SFC %d enable %d", __func__, id, en);

	regmap_update_bits(sfc->regmap,
			   TEGRA210_SFC_ENABLE,
			   TEGRA210_SFC_ENABLE_EN,
			   en ? TEGRA210_SFC_ENABLE_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_enable);

/**
 *  tegra210_sfc_reset - reset a sfc
 *  @id : sfc id
 */
int tegra210_sfc_reset(int id)
{
	struct tegra210_sfc_ctx *sfc;
	u32 val = 0;
	int cnt = AHUB_OP_MAX_RETRY;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	dev_vdbg(sfc->dev, "%s reset SFC %d", __func__, id);

	regmap_update_bits(sfc->regmap,
			   TEGRA210_SFC_SOFT_RESET,
			   TEGRA210_SFC_SOFT_RESET_EN,
			   TEGRA210_SFC_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(sfc->regmap, TEGRA210_SFC_SOFT_RESET, &val);
	} while ((val & TEGRA210_SFC_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(sfc->dev, "%s Failed to reset SFC %d", __func__, id);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_reset);

/**
 *  tegra210_sfc_set_slcg - enable/disable second level clock gating for a sfc
 *  @id : sfc id
 *  @en : enable/disable flag
 */
int tegra210_sfc_set_slcg(int id, bool en)
{
	struct tegra210_sfc_ctx *sfc;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	dev_vdbg(sfc->dev, "%s SFC %d slcg enable %d", __func__, id, en);

	regmap_update_bits(sfc->regmap,
			   TEGRA210_SFC_CG,
			   TEGRA210_SFC_CG_SLCG_EN,
			   en ? TEGRA210_SFC_CG_SLCG_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_set_slcg);

/**
 *  tegra210_sfc_enable_coef_ram - enable coefficient ram params for a sfc
 *  @id : sfc id
 *  @en : enable/disable flag
 *  @fs_in : input sampling frequency
 *  @fs_out : output sampling frequency
 */
int tegra210_sfc_enable_coef_ram(int id, bool en, int fs_in, int fs_out)
{
	struct tegra210_sfc_ctx *sfc;
	u32 *coef;
	VALIDATE_SFC_ID(id)

	sfc = tegra210_sfc[id];
	if (en) {
		switch (fs_in) {
		case 8000:
			if (fs_out == 16000) coef = coef_8to16;
			else if (fs_out == 44100) coef = coef_8to44;
			else if (fs_out == 48000) coef = coef_8to48;
			else goto err_rate;
			break;
		case 16000:
			if (fs_out == 8000) coef = coef_16to8;
			else if (fs_out == 44100) coef = coef_16to44;
			else if (fs_out == 48000) coef = coef_16to48;
			else goto err_rate;
			break;
		case 44100:
			if (fs_out == 8000) coef = coef_44to8;
			else if (fs_out == 16000) coef = coef_44to16;
			else goto err_rate;
			break;
		case 48000:
			if (fs_out == 8000) coef = coef_48to8;
			else if (fs_out == 16000) coef = coef_48to16;
			else goto err_rate;
			break;
		default:
			goto err_rate;
		}

		tegra210_ahubram_write(sfc->dev, TEGRA210_SFC_ARAMCTL_COEF_CTRL,
					TEGRA210_SFC_ARAMCTL_COEF_DATA,
					0, coef, TEGRA210_SFC_COEF_RAM_DEPTH);
	}

	dev_vdbg(sfc->dev, "%s SFC %d enable_coef_ram %d", __func__, id, en);

	regmap_update_bits(sfc->regmap,
			   TEGRA210_SFC_COEF_RAM,
			   TEGRA210_SFC_COEF_RAM_COEF_RAM_EN,
			   en ? TEGRA210_SFC_COEF_RAM_COEF_RAM_EN : 0);

	return 0;
err_rate:
	dev_err(sfc->dev, "%s conversion from %d to %d not supported",
	                   __func__, fs_in, fs_out);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra210_sfc_enable_coef_ram);

/* PM runtime callback functions */
static int tegra210_sfc_runtime_suspend(struct device *dev)
{
	struct tegra210_sfc_ctx *sfc = dev_get_drvdata(dev);

	dev_dbg(sfc->dev, "%s SFC %d", __func__, sfc->id);
	tegra210_axbar_disable_clk();
	regcache_cache_only(sfc->regmap, true);

	return 0;
}

static int tegra210_sfc_runtime_resume(struct device *dev)
{
	struct tegra210_sfc_ctx *sfc = dev_get_drvdata(dev);

	dev_dbg(sfc->dev, "%s SFC %d", __func__, sfc->id);
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
		dev_err(&pdev->dev, " %s failed, no DT node for SFC", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-sfc-id",
						 &pdev->id);

	dev_dbg(&pdev->dev, "%s SFC %d : starting probe",
		__func__, pdev->id);

	if ((pdev->id < 0) ||
		(pdev->id >= TEGRA210_SFC_COUNT)) {
		dev_err(&pdev->dev, "SFC id %d out of range\n", pdev->id);
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

	dev_dbg(sfc->dev, "%s SFC %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_sfc[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s SFC %d probe failed with %d",
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

	dev_dbg(&pdev->dev, "%s SFC %d removed",
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
