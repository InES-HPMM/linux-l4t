/*
 * tegra210_axbar.c - Tegra AXBAR driver.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Based on code by Stephen Warren <swarren@nvidia.com>
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
#define VERBOSE_DEBUG 1
#define DEBUG 1
*/

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "tegra210_axbar.h"

#define DRV_NAME "tegra210-axbar"

/* TEGRA210 AXBAR driver context */
struct tegra210_axbar_ctx {
	struct device		*dev;
	struct clk		*clk;
	struct clk		*clk_parent;
	struct clk		*clk_ape;
	void __iomem		*regs;
	struct regmap		*regmap;
};
static struct tegra210_axbar_ctx *tegra210_axbar;

/* API to enable/disable AHUB clock */
void tegra210_axbar_enable_clk()
{
	if (tegra210_axbar)
		pm_runtime_get_sync(tegra210_axbar->dev);
	else
		pr_err("AXBAR driver not initialized\n");
}
EXPORT_SYMBOL_GPL(tegra210_axbar_enable_clk);

void tegra210_axbar_disable_clk()
{
	if (tegra210_axbar)
		pm_runtime_put_sync(tegra210_axbar->dev);
	else
		pr_err("AHUB driver not initialized\n");
}
EXPORT_SYMBOL_GPL(tegra210_axbar_disable_clk);

int tegra210_axbar_set_rx_cif_source(enum tegra210_ahub_cifs rxcif,
				     enum tegra210_ahub_cifs txcif)
{
	int reg, shift;

	dev_vdbg(tegra210_axbar->dev, "%s Connect rx cif %d with tx cif %d",
		 __func__, rxcif, txcif);

	if (IS_ADMAIF_RX(rxcif)) {
		reg = TEGRA210_AXBAR_ADMAIF_RX1 +
		      (ADMAIF_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_I2S_RX(rxcif)) {
		reg = TEGRA210_AXBAR_I2S1_RX0 +
		      (I2S_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_SFC_RX(rxcif)) {
		reg = TEGRA210_AXBAR_SFC1_RX0 +
		      (SFC_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_MIXER_RX(rxcif)) {
		reg = TEGRA210_AXBAR_MIXER1_RX1 +
		      (MIXER_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_SPDIF_RX(rxcif)) {
		reg = TEGRA210_AXBAR_SPDIF1_RX0 +
		      (SPDIF_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_AFC_RX(rxcif)) {
		reg = TEGRA210_AXBAR_AFC1_RX0 +
		      (AFC_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_OPE_RX(rxcif)) {
		reg = TEGRA210_AXBAR_OPE1_RX0 +
		      (OPE_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_SPKPROT_RX(rxcif)) {
		reg = TEGRA210_AXBAR_SPKPROT1_RX0 +
		      (SPKPROT_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_MVC_RX(rxcif)) {
		reg = TEGRA210_AXBAR_MVC1_RX0 +
		      (MVC_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_AMX_RX(rxcif)) {
		reg = TEGRA210_AXBAR_AMX1_RX0 +
		      (AMX_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_ADX_RX(rxcif)) {
		reg = TEGRA210_AXBAR_ADX1_RX0 +
		      (ADX_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else {
		dev_err(tegra210_axbar->dev, "Invalid RX CIF %x", rxcif);
		return -EINVAL;
	}

	if (IS_ADMAIF_TX(txcif)) {
		shift = TEGRA210_AXBAR_ADMAIF_TX1_SHIFT +
			ADMAIF_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_0_BASE;
	} else if (IS_I2S_TX(txcif)) {
		shift = TEGRA210_AXBAR_I2S1_TX0_SHIFT + I2S_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_0_BASE;
	} else if (IS_SFC_TX(txcif)) {
		shift = TEGRA210_AXBAR_SFC1_TX0_SHIFT + SFC_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_0_BASE;
	} else if (IS_MIXER_TX(txcif)) {
		shift = TEGRA210_AXBAR_MIXER1_TX1_SHIFT + MIXER_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_1_BASE;
	} else if (IS_SPDIF_TX(txcif)) {
		shift = TEGRA210_AXBAR_SPDIF1_TX0_SHIFT + SPDIF_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_1_BASE;
	} else if (IS_AFC_TX(txcif)) {
		shift = TEGRA210_AXBAR_AFC1_TX0_SHIFT + AFC_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_1_BASE;
	} else if (IS_OPE_TX(txcif)) {
		shift = TEGRA210_AXBAR_OPE1_TX0_SHIFT + OPE_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_2_BASE;
	} else if (IS_SPKPROT_TX(txcif)) {
		shift = TEGRA210_AXBAR_SPKPROT1_TX0_SHIFT +
			SPKPROT_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_2_BASE;
	} else if (IS_MVC_TX(txcif)) {
		shift = TEGRA210_AXBAR_MVC1_TX0_SHIFT + MVC_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_2_BASE;
	} else if (IS_AMX_TX(txcif)) {
		shift = TEGRA210_AXBAR_AMX1_TX0_SHIFT + AMX_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_1_BASE;
	} else if (IS_ADX_TX(txcif)) {
		shift = TEGRA210_AXBAR_ADX1_TX0_SHIFT + ADX_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_2_BASE;
	} else if (IS_DMIC_TX(txcif)) {
		shift = TEGRA210_AXBAR_DMIC1_TX0_SHIFT + DMIC_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_2_BASE;
	} else if (IS_IQC_TX(txcif)) {
		shift = TEGRA210_AXBAR_IQC1_TX1_SHIFT + IQC_TX_SHIFT(txcif);
		reg += TEGRA210_AXBAR_PART_2_BASE;
	} else {
		dev_err(tegra210_axbar->dev, "Invalid TX CIF %x", txcif);
		return -EINVAL;
	}

	pm_runtime_get_sync(tegra210_axbar->dev);
	regmap_write(tegra210_axbar->regmap, reg, BIT(shift));
	pm_runtime_put_sync(tegra210_axbar->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_axbar_set_rx_cif_source);

int tegra210_axbar_unset_rx_cif_source(enum tegra210_ahub_cifs rxcif)
{
	int reg;

	dev_vdbg(tegra210_axbar->dev, "%s Disconnect rx cif %d",
		 __func__, rxcif);

	if (IS_ADMAIF_RX(rxcif)) {
		reg = TEGRA210_AXBAR_ADMAIF_RX1 +
		      (ADMAIF_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_I2S_RX(rxcif)) {
		reg = TEGRA210_AXBAR_I2S1_RX0 +
		      (I2S_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_SFC_RX(rxcif)) {
		reg = TEGRA210_AXBAR_SFC1_RX0 +
		      (SFC_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_MIXER_RX(rxcif)) {
		reg = TEGRA210_AXBAR_MIXER1_RX1 +
		      (MIXER_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_SPDIF_RX(rxcif)) {
		reg = TEGRA210_AXBAR_SPDIF1_RX0 +
		      (SPDIF_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_AFC_RX(rxcif)) {
		reg = TEGRA210_AXBAR_AFC1_RX0 +
		      (AFC_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_OPE_RX(rxcif)) {
		reg = TEGRA210_AXBAR_OPE1_RX0 +
		      (OPE_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_SPKPROT_RX(rxcif)) {
		reg = TEGRA210_AXBAR_SPKPROT1_RX0 +
		      (SPKPROT_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_MVC_RX(rxcif)) {
		reg = TEGRA210_AXBAR_MVC1_RX0 +
		      (MVC_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_AMX_RX(rxcif)) {
		reg = TEGRA210_AXBAR_AMX1_RX0 +
		      (AMX_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else if (IS_ADX_RX(rxcif)) {
		reg = TEGRA210_AXBAR_ADX1_RX0 +
		      (ADX_RX_SHIFT(rxcif) * TEGRA210_AXBAR_REG_STRIDE);
	} else {
		dev_err(tegra210_axbar->dev, "Invalid RX CIF %x", rxcif);
		return -EINVAL;
	}

	pm_runtime_get_sync(tegra210_axbar->dev);
	regmap_write(tegra210_axbar->regmap,
		     TEGRA210_AXBAR_PART_0_BASE + reg, 0);
	regmap_write(tegra210_axbar->regmap,
		     TEGRA210_AXBAR_PART_1_BASE + reg, 0);
	regmap_write(tegra210_axbar->regmap,
		     TEGRA210_AXBAR_PART_2_BASE + reg, 0);
	pm_runtime_put_sync(tegra210_axbar->dev);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_axbar_unset_rx_cif_source);

/* Regmap callback functions */
static bool tegra210_axbar_rw_reg(struct device *dev, unsigned int reg)
{
	if ((reg >= TEGRA210_AXBAR_PART_0_BASE + TEGRA210_AXBAR_ADMAIF_RX1) &&
	    (reg <= TEGRA210_AXBAR_PART_2_BASE + TEGRA210_AXBAR_ADX2_RX0))
		return true;
	else
		return false;
}

static const struct regmap_config tegra210_axbar_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_AXBAR_PART_2_BASE + TEGRA210_AXBAR_ADX2_RX0,
	.writeable_reg = tegra210_axbar_rw_reg,
	.readable_reg = tegra210_axbar_rw_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_axbar_runtime_suspend(struct device *dev)
{
	struct tegra210_axbar_ctx *axbar = dev_get_drvdata(dev);

	dev_dbg(axbar->dev, "%s", __func__);

	regcache_cache_only(axbar->regmap, true);
#ifndef CONFIG_MACH_GRENADA
	clk_disable_unprepare(axbar->clk);
#endif

	return 0;
}

static int tegra210_axbar_runtime_resume(struct device *dev)
{
	struct tegra210_axbar_ctx *axbar = dev_get_drvdata(dev);
#ifndef CONFIG_MACH_GRENADA
	int ret;

	dev_dbg(axbar->dev, "%s", __func__);

	ret = clk_prepare_enable(axbar->clk);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}
#endif
	regcache_cache_only(axbar->regmap, false);
	return 0;
}

struct of_dev_auxdata axbar_auxdata[] = {
	OF_DEV_AUXDATA("nvidia,tegra210-ope", 0x702D8000, "tegra210-ope.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-ope", 0x702D8400, "tegra210-ope.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", 0x702D2000, "tegra210-sfc.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", 0x702D2200, "tegra210-sfc.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", 0x702D2400, "tegra210-sfc.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", 0x702D2600, "tegra210-sfc.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-admaif", 0x702D0000,
			"tegra210-admaif", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2s", 0x702D1000, "tegra210-i2s.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2s", 0x702D1100, "tegra210-i2s.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2s", 0x702D1200, "tegra210-i2s.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2s", 0x702D1300, "tegra210-i2s.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2s", 0x702D1400, "tegra210-i2s.4",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spkprot", 0x702D8C00,
			"tegra210-spkprot.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-amixer", 0x702DBB00,
			"tegra210-amixer.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", 0x702D4000, "tegra210-dmic.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", 0x702D4100, "tegra210-dmic.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", 0x702D4200, "tegra210-dmic.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-mvc", 0x702DA000, "tegra210-mvc.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-mvc", 0x702DA200, "tegra210-mvc.1",
			NULL),
	{}
};

static int tegra210_axbar_remove_child(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	dev_dbg(dev, "%s", __func__);

	of_device_unregister(pdev);
	return 0;
}

/*
AXBAR Driver probe and remove functions
*/
static int tegra210_axbar_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_axbar_ctx *axbar;
	int ret = 0;

	dev_dbg(&pdev->dev, "AXBAR : starting probe");

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No device tree node for AXBAR driver");
		return -ENODEV;
	}

	tegra210_axbar = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_axbar_ctx),
			   GFP_KERNEL);
	if (!tegra210_axbar) {
		dev_err(&pdev->dev, "Can't allocate ahub context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_axbar->dev = &pdev->dev;

	axbar = tegra210_axbar;
	dev_set_drvdata(&pdev->dev, axbar);

	axbar->clk = clk_get(&pdev->dev, "ahub");
	if (IS_ERR(axbar->clk)) {
		dev_err(&pdev->dev, "Can't retrieve ahub clock\n");
		ret = PTR_ERR(axbar->clk);
		goto err_free;
	}

	axbar->clk_parent = clk_get_sys(NULL, "pll_a_out0");
	if (IS_ERR(axbar->clk)) {
		dev_err(&pdev->dev, "Can't retrieve pll_a_out0 clock\n");
		ret = PTR_ERR(axbar->clk_parent);
		goto err_clk_put_axbar;
	}

	axbar->clk_ape = clk_get_sys(NULL, "ape");
	if (IS_ERR(axbar->clk_ape)) {
		dev_err(&pdev->dev, "Can't retrieve ape clock\n");
		ret = PTR_ERR(axbar->clk_ape);
		goto err_clk_put_pll_a_out0;
	}

	ret = clk_set_rate(axbar->clk_parent, 24560000);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set clock rate of pll_a_out0\n");
		goto err_clk_put_ape;
	}

	ret = clk_set_parent(axbar->clk, axbar->clk_parent);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set parent clock with pll_a_out0\n");
		goto err_clk_put_ape;
	}

	ret = clk_enable(axbar->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable xbar clock: %d\n", ret);
		goto err_clk_put_ape;
	}

	ret = clk_enable(axbar->clk_ape);
	if (ret) {
		dev_err(&pdev->dev, "APE clk_enable failed: %d\n", ret);
		goto err_clk_put_ape;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory 0 resource\n");
		ret = -ENODEV;
		goto err_clk_put_ape;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region 0 already claimed\n");
		ret = -EBUSY;
		goto err_clk_put_ape;
	}

	axbar->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!axbar->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_clk_put_ape;
	}

	axbar->regmap = devm_regmap_init_mmio(&pdev->dev, axbar->regs,
					    &tegra210_axbar_regmap_config);
	if (IS_ERR(axbar->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(axbar->regmap);
		goto err_clk_put_ape;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_axbar_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	if (pdev->dev.of_node)
		of_platform_populate(pdev->dev.of_node, NULL, axbar_auxdata,
				     &pdev->dev);

	dev_dbg(axbar->dev, "ahub probe successful");

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_clk_put_ape:
	clk_put(axbar->clk_ape);
err_clk_put_pll_a_out0:
	clk_put(axbar->clk_parent);
err_clk_put_axbar:
	clk_put(axbar->clk);
err_free:
	tegra210_axbar = NULL;
exit:
	dev_dbg(&pdev->dev, "AXBAR probe failed with %d", ret);
	return ret;
}

static int tegra210_axbar_remove(struct platform_device *pdev)
{
	struct tegra210_axbar_ctx *axbar;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_axbar_runtime_suspend(&pdev->dev);

	axbar = platform_get_drvdata(pdev);
	/* clk_put(axbar->clk); */
	tegra210_axbar = NULL;
	device_for_each_child(&pdev->dev, NULL, tegra210_axbar_remove_child);

	dev_dbg(&pdev->dev, "%s AXBAR removed", __func__);

	return 0;
}

static const struct of_device_id tegra210_axbar_of_match[] = {
	{ .compatible = "nvidia,tegra210-axbar",},
	{},
};

static const struct dev_pm_ops tegra210_axbar_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_axbar_runtime_suspend,
			   tegra210_axbar_runtime_resume, NULL)
};

static struct platform_driver tegra210_axbar_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_axbar_of_match,
		.pm = &tegra210_axbar_pm_ops,
	},
	.probe = tegra210_axbar_probe,
	.remove = tegra210_axbar_remove,
};
module_platform_driver(tegra210_axbar_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 AXBAR driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
