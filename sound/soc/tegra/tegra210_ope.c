/*
 * tegra210_ope.c - Tegra OPE driver.
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
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_ope.h"
#include "tegra210_peq.h"
#include "tegra210_mbdrc.h"
#include "tegra210_axbar.h"

#define DRV_NAME "tegra210-ope"

/* TEGRA210 OPE driver context */
struct tegra210_ope_ctx {
	struct device		*dev;
	void __iomem		*regs;
	struct regmap		*regmap;

	int			id;
	bool			in_use;
	int			enable_count;
};

static struct tegra210_ope_ctx *tegra210_ope[TEGRA210_OPE_COUNT];

/* Allocate a OPE instance. If cif = CIF_NONE is passed ope driver will
   allocate any available free instance */
int tegra210_ope_get(enum tegra210_ahub_cifs *cif)
{
	enum tegra210_cif_dir dir = IS_OPE_RX(*cif) ? DIR_RX : DIR_TX;
	int id = OPE_ID(*cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	int i;

	if (id >= TEGRA210_OPE_COUNT) {
		pr_err("%s Invalid OPE id %d", __func__, id);
		return -EINVAL;
	}

	if (IS_OPE(*cif)) {
		ope = tegra210_ope[id];
		if (ope->in_use) {
			dev_warn(ope->dev, "%s OPE %d in use", __func__, id);
			return -EBUSY;
		} else {
			ope->in_use = 1;
			pm_runtime_get_sync(ope->dev);
			tegra210_peq_enable_clk(*cif);
			tegra210_mbdrc_enable_clk(*cif);
			dev_dbg(ope->dev, "%s Allocated OPE %d", __func__, id);
			return 0;
		}
	}

	for (i = 0; i < TEGRA210_OPE_COUNT; i++) {
		ope = tegra210_ope[i];
		if (!ope->in_use) {
			ope->in_use = 1;
			pm_runtime_get_sync(ope->dev);
			dev_dbg(ope->dev, "%s Allocated OPE %d", __func__, i);
			*cif = (dir == DIR_RX) ? (OPE1_RX0 + i) :
						 (OPE1_TX0 + i);
			tegra210_peq_enable_clk(*cif);
			tegra210_mbdrc_enable_clk(*cif);
			return 0;
		}
	}
	pr_err("%s Failed to allocate OPE id %d", __func__, id);
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_ope_get);

/* Free OPE */
int tegra210_ope_put(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];

	if (!IS_OPE(cif)) {
		pr_err("%s Invalid OPE id %d", __func__, id);
		return -EINVAL;
	}
	ope = tegra210_ope[id];

	if (ope->in_use) {
		ope->in_use = 0;
		pm_runtime_put_sync(ope->dev);
		tegra210_peq_disable_clk(cif);
		tegra210_mbdrc_disable_clk(cif);
	} else
		dev_warn(ope->dev, "%s OPE %d not in use", __func__, id);

	dev_dbg(ope->dev, "%s OPE %d freed", __func__, id);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ope_put);

/* OPE related APIs */
int tegra210_ope_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 reg;

	dev_vdbg(ope->dev, "%s OPE cif %d", __func__, cif);

	reg = (IS_OPE_TX(cif) ? TEGRA210_OPE_XBAR_TX_CIF_CTRL :
				TEGRA210_OPE_XBAR_RX_CIF_CTRL);

	return tegra210_set_axbar_cif_param(ope->dev,  reg , acif);
}
EXPORT_SYMBOL_GPL(tegra210_ope_set_acif_param);

int tegra210_ope_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 reg;

	dev_vdbg(ope->dev, "%s OPE cif %d", __func__, cif);

	reg = (IS_OPE_TX(cif) ? TEGRA210_OPE_XBAR_TX_CIF_CTRL :
				TEGRA210_OPE_XBAR_RX_CIF_CTRL);

	return tegra210_get_axbar_cif_param(ope->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_ope_get_acif_param);

int tegra210_ope_is_cif_fifo_full(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 val = 0;

	regmap_read(ope->regmap,
		    IS_OPE_RX(cif) ?
		    TEGRA210_OPE_XBAR_RX_STATUS :
		    TEGRA210_OPE_XBAR_TX_STATUS,
		    &val);
	val &= TEGRA210_OPE_XBAR_RX_STATUS_ACIF_FIFO_FULL;

	dev_vdbg(ope->dev, "%s OPE %s cif fifo full %d",
		 __func__, IS_OPE_RX(cif) ? "RX" : "TX", val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_ope_is_cif_fifo_full);

int tegra210_ope_is_cif_fifo_empty(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 val = 0;

	regmap_read(ope->regmap,
		    IS_OPE_RX(cif) ?
		    TEGRA210_OPE_XBAR_RX_STATUS :
		    TEGRA210_OPE_XBAR_TX_STATUS,
		    &val);
	val &= TEGRA210_OPE_XBAR_RX_STATUS_ACIF_FIFO_EMPTY;

	dev_vdbg(ope->dev, "%s OPE %s cif fifo empty %d",
		 __func__, IS_OPE_RX(cif) ? "RX" : "TX", val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_ope_is_cif_fifo_empty);

int tegra210_ope_is_enable(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 val = 0;

	regmap_read(ope->regmap, TEGRA210_OPE_STATUS, &val);
	val &= TEGRA210_OPE_STATUS_ENABLE_STATUS;

	dev_vdbg(ope->dev, "%s OPE enable %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_ope_is_enable);

int tegra210_ope_is_slcg_enable(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 val = 0;

	regmap_read(ope->regmap, TEGRA210_OPE_STATUS, &val);
	val &= TEGRA210_OPE_STATUS_SLCG_CLKEN;

	dev_vdbg(ope->dev, "%s OPE SLCG enable %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_ope_is_slcg_enable);

int tegra210_ope_is_config_err(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	u32 val = 0;

	regmap_read(ope->regmap, TEGRA210_OPE_STATUS, &val);
	val &= TEGRA210_OPE_STATUS_CONFIG_ERR;

	dev_vdbg(ope->dev, "%s OPE Config error %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_ope_is_config_err);

/* Master enable for OPE */
int tegra210_ope_enable(enum tegra210_ahub_cifs cif, bool en)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	int cnt = AHUB_OP_MAX_RETRY;

	dev_vdbg(ope->dev, "%s OPE enable %d count %d",
		 __func__, en, ope->enable_count);

	if (en) {
		ope->enable_count++;
		if (ope->enable_count > 1)
			return 0;
		/* reset OPE before enabling data flow */
		tegra210_ope_reset(cif);
	} else if (ope->enable_count > 0) {
		ope->enable_count--;
		if (ope->enable_count > 0)
			return 0;
	}

	regmap_update_bits(ope->regmap,
			   TEGRA210_OPE_ENABLE,
			   TEGRA210_OPE_ENABLE_ENABLE,
			   en ? TEGRA210_OPE_ENABLE_ENABLE : 0);

	/* ensure OPE is enabled */
	while ((tegra210_ope_is_enable(cif) != en) && cnt--)
		udelay(AHUB_OP_SLEEP_US);

	if (!cnt) {
		dev_warn(ope->dev, "%s Failed to %s OPE",
			 __func__, en ? "enable" : "disable");
		tegra210_ope_reset(cif);
		return -ETIME;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ope_enable);

/* Master soft reset for OPE */
int tegra210_ope_reset(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];
	int cnt = AHUB_OP_MAX_RETRY;
	u32 val = 0;

	dev_vdbg(ope->dev, "%s", __func__);

	regmap_update_bits(ope->regmap,
			   TEGRA210_OPE_SOFT_RST,
			   TEGRA210_OPE_SOFT_RST_RESET,
			   TEGRA210_OPE_SOFT_RST_RESET);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(ope->regmap, TEGRA210_OPE_SOFT_RST, &val);
	} while ((val & TEGRA210_OPE_SOFT_RST_RESET) && cnt--);

	if (!cnt) {
		dev_warn(ope->dev, "%s Failed to reset OPE", __func__);
		return -ETIME;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ope_reset);

/* Enable second level clock gating for OPE */
int tegra210_ope_set_slcg(enum tegra210_ahub_cifs cif, int en)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];

	dev_vdbg(ope->dev, "%s OPE slcg enable %d", __func__, en);

	regmap_update_bits(ope->regmap,
			   TEGRA210_OPE_CG,
			   TEGRA210_OPE_CG_SLCG_EN,
			   en ? TEGRA210_OPE_CG_SLCG_EN : 0);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ope_set_slcg);

/* Set order of MBDRC and PEQ in OPE */
int tegra210_ope_dir_peq_to_mbdrc(enum tegra210_ahub_cifs cif, int val)
{
	int id = OPE_ID(cif);
	struct tegra210_ope_ctx *ope = tegra210_ope[id];

	dev_vdbg(ope->dev, "%s OPE direction MBDRC to PEQ %d", __func__, val);

	regmap_update_bits(ope->regmap,
			   TEGRA210_OPE_DIRECTION,
			   TEGRA210_OPE_DIRECTION_DIR_MASK,
			   val ? TEGRA210_OPE_DIRECTION_DIR_PEQ_TO_MBDRC :
			   TEGRA210_OPE_DIRECTION_DIR_MBDRC_TO_PEQ);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ope_dir_peq_to_mbdrc);


/* Regmap callback functions */
static bool tegra210_ope_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_OPE_XBAR_RX_INT_MASK:
	case TEGRA210_OPE_XBAR_RX_INT_SET:
	case TEGRA210_OPE_XBAR_RX_INT_CLEAR:
	case TEGRA210_OPE_XBAR_RX_CIF_CTRL:
	case TEGRA210_OPE_XBAR_TX_INT_MASK:
	case TEGRA210_OPE_XBAR_TX_INT_SET:
	case TEGRA210_OPE_XBAR_TX_INT_CLEAR:
	case TEGRA210_OPE_XBAR_TX_CIF_CTRL:
	case TEGRA210_OPE_ENABLE:
	case TEGRA210_OPE_SOFT_RST:
	case TEGRA210_OPE_CG:
	case TEGRA210_OPE_DIRECTION:
		return true;
	default:
		return false;
	};
}

static bool tegra210_ope_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_OPE_XBAR_RX_STATUS:
	case TEGRA210_OPE_XBAR_RX_INT_STATUS:
	case TEGRA210_OPE_XBAR_RX_INT_MASK:
	case TEGRA210_OPE_XBAR_RX_INT_SET:
	case TEGRA210_OPE_XBAR_RX_INT_CLEAR:
	case TEGRA210_OPE_XBAR_RX_CIF_CTRL:
	case TEGRA210_OPE_XBAR_TX_STATUS:
	case TEGRA210_OPE_XBAR_TX_INT_STATUS:
	case TEGRA210_OPE_XBAR_TX_INT_MASK:
	case TEGRA210_OPE_XBAR_TX_INT_SET:
	case TEGRA210_OPE_XBAR_TX_INT_CLEAR:
	case TEGRA210_OPE_XBAR_TX_CIF_CTRL:
	case TEGRA210_OPE_ENABLE:
	case TEGRA210_OPE_SOFT_RST:
	case TEGRA210_OPE_CG:
	case TEGRA210_OPE_STATUS:
	case TEGRA210_OPE_INT_STATUS:
	case TEGRA210_OPE_DIRECTION:
		return true;
	default:
		return false;
	};
}

static bool tegra210_ope_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_OPE_XBAR_RX_STATUS:
	case TEGRA210_OPE_XBAR_RX_INT_SET:
	case TEGRA210_OPE_XBAR_RX_INT_STATUS:
	case TEGRA210_OPE_XBAR_TX_STATUS:
	case TEGRA210_OPE_XBAR_TX_INT_SET:
	case TEGRA210_OPE_XBAR_TX_INT_STATUS:
	case TEGRA210_OPE_SOFT_RST:
	case TEGRA210_OPE_STATUS:
	case TEGRA210_OPE_INT_STATUS:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_ope_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGEA210_OPE_MAX_REGISTER,
	.writeable_reg = tegra210_ope_wr_reg,
	.readable_reg = tegra210_ope_rd_reg,
	.volatile_reg = tegra210_ope_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_ope_runtime_suspend(struct device *dev)
{
	struct tegra210_ope_ctx *ope = dev_get_drvdata(dev);

	dev_dbg(ope->dev, "%s OPE %d", __func__, ope->id);
	regcache_cache_only(ope->regmap, true);
	tegra210_axbar_disable_clk();

	return 0;
}

static int tegra210_ope_runtime_resume(struct device *dev)
{
	struct tegra210_ope_ctx *ope = dev_get_drvdata(dev);

	dev_dbg(ope->dev, "%s OPE %d", __func__, ope->id);

	tegra210_axbar_enable_clk();
	regcache_cache_only(ope->regmap, false);

	return 0;
}

struct of_dev_auxdata ope_auxdata[] = {
	OF_DEV_AUXDATA("nvidia,tegra210-peq", 0x702D8100, "tegra210-peq.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-peq", 0x702D8500, "tegra210-peq.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-mbdrc", 0x702D8200, "tegra210-mbdrc.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-mbdrc", 0x702D8600, "tegra210-mbdrc.1",
				NULL),
	{}
};

static int tegra210_ope_remove_child(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	dev_err(dev, "%s", __func__);

	of_device_unregister(pdev);
	return 0;
}

/*
OPE Driver probe and remove functions
*/
static int tegra210_ope_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_ope_ctx *ope;
	int ret = 0;
	int id = 0;

	if (!pdev->dev.of_node) {
		pr_err("%s Probe failed. DT node missing.", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-ope-id",
			     &pdev->id);

	dev_dbg(&pdev->dev, "%s OPE %d : starting probe",
		__func__, pdev->id);

	if ((pdev->id < 0) ||
	    (pdev->id >= TEGRA210_OPE_COUNT)) {
		dev_err(&pdev->dev, "OPE id %d out of range\n", pdev->id);
		return -EINVAL;
	}
	id = pdev->id;

	tegra210_ope[id] = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_ope_ctx),
			   GFP_KERNEL);
	if (!tegra210_ope[id]) {
		dev_err(&pdev->dev, "Can't allocate ope context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_ope[id]->dev = &pdev->dev;

	ope = tegra210_ope[id];
	ope->id = id;
	dev_set_drvdata(&pdev->dev, ope);

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

	ope->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!ope->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	ope->regmap = devm_regmap_init_mmio(&pdev->dev, ope->regs,
					    &tegra210_ope_regmap_config);
	if (IS_ERR(ope->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(ope->regmap);
		goto err_free;
	}
	regcache_cache_only(ope->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_ope_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	if (pdev->dev.of_node)
		of_platform_populate(pdev->dev.of_node, NULL, ope_auxdata,
				     &pdev->dev);

	dev_dbg(ope->dev, "%s OPE %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_ope[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s OPE %d probe failed with %d",
		__func__, pdev->id, ret);
	return ret;
}

static int tegra210_ope_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_ope_runtime_suspend(&pdev->dev);

	tegra210_ope[pdev->id] = NULL;

	device_for_each_child(&pdev->dev, NULL, tegra210_ope_remove_child);
	dev_dbg(&pdev->dev, "%s OPE %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_ope_of_match[] = {
	{ .compatible = "nvidia,tegra210-ope",},
	{},
};

static const struct dev_pm_ops tegra210_ope_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_ope_runtime_suspend,
			   tegra210_ope_runtime_resume, NULL)
};

static struct platform_driver tegra210_ope_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_ope_of_match,
		.pm = &tegra210_ope_pm_ops,
	},
	.probe = tegra210_ope_probe,
	.remove = tegra210_ope_remove,
};
module_platform_driver(tegra210_ope_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra APE OPE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
