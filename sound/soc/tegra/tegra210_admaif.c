/*
 * tegra210_admaif.c - Tegra ADMAIF driver.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Author: Dara Ramesh <dramesh@nvidia.com>
 * Based on code by Stephen Warren <swarren@nvidia.com>
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_admaif.h"
#include "tegra210_axbar.h"

#define DRV_NAME "tegra210-admaif"

/* TEGRA210 ADMIF driver context */
struct tegra210_admaif_ctx {
	struct device		*dev;
	struct clk		*clk;
	void __iomem		*regs;
	struct regmap		*regmap;
	int			dma_sel_base;
	u32			chan_in_use[DIR_NUM];
	u32			enable_count;
};

static struct tegra210_admaif_ctx *tegra210_admaif;

/* API to enable/disable ADMAIF clock */
void tegra210_admaif_enable_clk(void)
{
	if (tegra210_admaif)
		pm_runtime_get_sync(tegra210_admaif->dev);
	else
		pr_err("ADMAIF driver not initialized\n");
}
EXPORT_SYMBOL_GPL(tegra210_admaif_enable_clk);

void tegra210_admaif_disable_clk(void)
{
	if (tegra210_admaif)
		pm_runtime_put_sync(tegra210_admaif->dev);
	else
		pr_err("ADMAIF driver not initialized\n");
}
EXPORT_SYMBOL_GPL(tegra210_admaif_disable_clk);

int tegra210_admaif_soft_reset(enum tegra210_ahub_cifs cif)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	unsigned int val, dcnt = 10;

	u32 reg = IS_ADMAIF_TX(cif) ? TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET :
				    TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET;
	reg += (ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);

	regmap_write(admaif->regmap, reg, 1);

	do {
		udelay(100);
		regmap_read(admaif->regmap, reg, &val);
	} while (val && dcnt--);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_soft_reset);

/* Allocate a ADMIF channel. If cif = CIF_NONE is passed admaif driver
 * will allocate any available free channel */
int tegra210_admaif_get(enum tegra210_ahub_cifs *cif)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	enum tegra210_cif_dir dir = IS_ADMAIF_RX(*cif) ? DIR_RX : DIR_TX;
	int cif_id = ADMAIF_CIF_ID(*cif);
	int i;

	if (cif_id >= TEGRA210_ADMAIF_CHAN_COUNT) {
		dev_err(admaif->dev, "%s Invalid ADMAIF channel ID %d",
			__func__, cif_id);
		return -EINVAL;
	}

	if (IS_ADMAIF(*cif)) {
		if ((admaif->chan_in_use[dir] >> cif_id) & 0x1) {
			dev_dbg(admaif->dev, "%s ADMAIF chan %d in use",
				 __func__, cif_id);
			return -EBUSY;
		} else {
			admaif->chan_in_use[dir] |= BIT(cif_id);
			tegra210_admaif_enable_clk();
			dev_dbg(admaif->dev, "%s Allocated ADMAIF chan %d",
				__func__, cif_id);
			pm_runtime_get_sync(admaif->dev);
			return 0;
		}
	}

	for (i = 0; i < TEGRA210_ADMAIF_CHAN_COUNT; i++) {
		if (!(admaif->chan_in_use[dir] >> i) & 0x1) {
			admaif->chan_in_use[dir] |= BIT(i);
			tegra210_admaif_enable_clk();
			dev_dbg(admaif->dev, "%s Allocated ADMAIF chan %d",
				__func__, i);
			*cif = (dir == DIR_RX) ? (ADMAIF_RX1 + i) :
						 (ADMAIF_TX1 + i);
			pm_runtime_get_sync(admaif->dev);
			return 0;
		}
	}

	dev_err(admaif->dev, "%s Failed to allocate ADMAIF chan %d",
		__func__, cif_id);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_get);

/* Free a ADMAIF channel */
int tegra210_admaif_put(enum tegra210_ahub_cifs cif)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	enum tegra210_cif_dir dir = IS_ADMAIF_RX(cif) ? DIR_RX : DIR_TX;
	int cif_id = ADMAIF_CIF_ID(cif);

	if (!IS_ADMAIF(cif)) {
		dev_err(admaif->dev, "%s Invalid ADMAIF ID %d", __func__, cif);
		return -EINVAL;
	}

	if ((admaif->chan_in_use[dir] >> cif_id) & 0x1) {
		admaif->chan_in_use[dir] &= ~BIT(cif_id);
		tegra210_admaif_disable_clk();
		pm_runtime_put_sync(admaif->dev);
	} else
		dev_warn(admaif->dev, "%s ADMAIF %d not in use",
			 __func__, cif_id);

	dev_dbg(admaif->dev, "%s ADMAIF chan %d freed",
		__func__, cif_id);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_put);

/* ADMAIF ACIF related APIs */
int tegra210_admaif_set_acif_param(enum tegra210_ahub_cifs cif,
				   struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	u32 reg;

	dev_dbg(admaif->dev, "ADMAIF cif %d", cif);

	reg = ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;
	reg += (IS_ADMAIF_TX(cif) ? TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL :
				TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL);
	return tegra210_set_axbar_cif_param(admaif->dev,  reg , acif);
}
EXPORT_SYMBOL_GPL(tegra210_admaif_set_acif_param);

int tegra210_admaif_get_acif_param(enum tegra210_ahub_cifs cif,
				   struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	u32 reg;

	dev_vdbg(admaif->dev, "ADMAIF cif %d", cif);

	reg = ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;
	reg += (IS_ADMAIF_TX(cif) ? TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL :
				TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL);

	return tegra210_get_axbar_cif_param(admaif->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_admaif_get_acif_param);

int tegra210_admaif_enable(enum tegra210_ahub_cifs cif, int en)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	u32 reg = IS_ADMAIF_TX(cif) ? TEGRA210_ADMAIF_XBAR_TX_ENABLE :
				    TEGRA210_ADMAIF_XBAR_RX_ENABLE;
	reg += (ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);

	/* soft reset ADMAIF channel to discard/drop any stale data of
	 * previous transfer. */
	if (en)
		tegra210_admaif_soft_reset(cif);

	regmap_write(admaif->regmap, reg, en);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_enable);

int tegra210_admaif_global_enable(int en)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;

	dev_vdbg(admaif->dev, "admaif global enable = %d, ref.count = %d",
		en, admaif->enable_count);

	if (en) {
		admaif->enable_count++;
		if (admaif->enable_count > 1)
			return 0;
	} else {
		admaif->enable_count--;
		if (admaif->enable_count > 0)
			return 0;
	}
	regmap_write(admaif->regmap, TEGRA210_ADMAIF_GLOBAL_ENABLE, en);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_global_enable);

int tegra210_admaif_set_rx_dma_fifo(enum tegra210_ahub_cifs cif,
				 int start_addr, int size)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	u32 val, reg;
	int mask;

	dev_dbg(admaif->dev,
		"admaif rx cif %d, dma start.addr = %d, size = %d ",
		cif, start_addr, size);

	reg = ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;
	reg += TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL;

	regmap_read(admaif->regmap, reg, &val);

	val = (start_addr << TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT) |
		(size << TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT);

	mask = (TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT |
			TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_MASK);

	regmap_update_bits(admaif->regmap,
				reg, mask, val);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_set_rx_dma_fifo);

int tegra210_admaif_set_tx_dma_fifo(enum tegra210_ahub_cifs cif,
				 int start_addr, int size, int threshold)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	u32 val, reg;
	int mask;

	dev_dbg(admaif->dev,
		"admaif tx cif %d, dma start addr = %d, size = %d, thres.= %d",
		cif, start_addr, size, threshold);

	reg = ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;
	reg += TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL;

	regmap_read(admaif->regmap, reg, &val);

	val = (start_addr << TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT) |
		(size << TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT) |
		(threshold << TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT);

	mask = (TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT |
			TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_MASK |
			TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_MASK);

	regmap_update_bits(admaif->regmap,
				reg, mask, val);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_admaif_set_tx_dma_fifo);

int tegra210_admaif_xbar_transfer_status(enum tegra210_ahub_cifs cif)
{
	struct tegra210_admaif_ctx *admaif = tegra210_admaif;
	u32 reg, val;

	reg = ADMAIF_CIF_ID(cif) * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;
	reg += (IS_ADMAIF_TX(cif) ? TEGRA210_ADMAIF_XBAR_TX_STATUS :
				TEGRA210_ADMAIF_XBAR_RX_STATUS);

	regmap_read(admaif->regmap, reg, &val);
	val = (val & TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_MASK) >>
				TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_SHIFT;
	dev_dbg(admaif->dev, "admaif cif = %d, trans.enable status = %d",
		cif, val);
	return val;
}
EXPORT_SYMBOL(tegra210_admaif_xbar_transfer_status);


/* Regmap callback functions */
static bool tegra210_admaif_rw_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;

	switch (reg) {
	case TEGRA210_ADMAIF_XBAR_TX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL:
	case TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL:
	case TEGRA210_ADMAIF_XBAR_RX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL:
	case TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL:
	case TEGRA210_ADMAIF_GLOBAL_ENABLE:
	case TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET:
	case TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET:
		return true;
	default:
		return false;
	};
}

static bool tegra210_admaif_rd_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;

	switch (reg) {
	case TEGRA210_ADMAIF_XBAR_RX_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_STATUS:
	case TEGRA210_ADMAIF_XBAR_RX_INT_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_INT_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL:
	case TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL:
	case TEGRA210_ADMAIF_XBAR_RX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL:
	case TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL:
	case TEGRA210_ADMAIF_GLOBAL_ENABLE:
	case TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET:
	case TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET:
		return true;
	default:
		return false;
	};
}

static bool tegra210_admaif_volatile_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;

	switch (reg) {
	/* SOFT_RESET bit is auto cleared by HW */
	case TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET:
	case TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_admaif_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_ADMAIF_LAST_REG,
	.writeable_reg = tegra210_admaif_rw_reg,
	.readable_reg = tegra210_admaif_rd_reg,
	.volatile_reg = tegra210_admaif_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_admaif_runtime_suspend(struct device *dev)
{
	struct tegra210_admaif_ctx *admaif = dev_get_drvdata(dev);

	dev_dbg(admaif->dev, "%s", __func__);
#ifndef CONFIG_MACH_GRENADA
	tegra210_axbar_disable_clk();
#endif
	regcache_cache_only(admaif->regmap, true);
#ifndef CONFIG_MACH_GRENADA
	clk_disable_unprepare(admaif->clk);
#endif

	return 0;
}

static int tegra210_admaif_runtime_resume(struct device *dev)
{
	struct tegra210_admaif_ctx *admaif = dev_get_drvdata(dev);
#ifndef CONFIG_MACH_GRENADA
	int ret;

	dev_dbg(admaif->dev, "%s", __func__);

	tegra210_axbar_enable_clk();
	ret = clk_prepare_enable(admaif->clk);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		tegra210_axbar_disable_clk();
		return ret;
	}
#endif
	regcache_cache_only(admaif->regmap, false);
	return 0;
}
/*
ADMAIF Driver probe and remove functions
*/
static int tegra210_admaif_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	int ret = 0;

	dev_dbg(&pdev->dev, "ADMAIF : starting probe ");

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No device tree node for ADMAIF driver");
		return -ENODEV;
	}

	tegra210_admaif = devm_kzalloc(&pdev->dev,
			 sizeof(struct tegra210_admaif_ctx),
			 GFP_KERNEL);
	if (!tegra210_admaif) {
		dev_err(&pdev->dev, "Can't allocate admaif context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_admaif->dev = &pdev->dev;

	dev_set_drvdata(&pdev->dev, tegra210_admaif);

	/*tegra210_admaif->clk = clk_get(&pdev->dev, "admaif");
	if (IS_ERR(tegra210_admaif->clk)) {
		dev_err(&pdev->dev, "Can't retrieve admaif clock\n");
		ret = PTR_ERR(tegra210_admaif->clk);
		goto err_free;
	} */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory 0 resource\n");
		ret = -ENODEV;
		goto err_clk_put_admaif;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region 0 already claimed\n");
		ret = -EBUSY;
		goto err_clk_put_admaif;
	}

	tegra210_admaif->regs = devm_ioremap(&pdev->dev, res->start,
					    resource_size(res));
	if (!tegra210_admaif->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_clk_put_admaif;
	}

	tegra210_admaif->regmap = devm_regmap_init_mmio(&pdev->dev,
						tegra210_admaif->regs,
						&tegra210_admaif_regmap_config);
	if (IS_ERR(tegra210_admaif->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(tegra210_admaif->regmap);
		goto err_clk_put_admaif;
	}
	regcache_cache_only(tegra210_admaif->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_admaif_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(tegra210_admaif->dev, "ADMAIF probe successful");

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_clk_put_admaif:
/*	clk_put(tegra210_admaif->clk);
err_free: */
	tegra210_admaif = NULL;
exit:
	dev_dbg(&pdev->dev, "%s ADMAIF probe failed with %d",
		__func__, ret);
	return ret;
}

static int tegra210_admaif_remove(struct platform_device *pdev)
{
	struct tegra210_admaif_ctx *admaif;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_admaif_runtime_suspend(&pdev->dev);

	admaif = platform_get_drvdata(pdev);
#ifndef CONFIG_MACH_GRENADA
	clk_put(admaif->clk);
#endif
	tegra210_admaif = NULL;
	dev_dbg(&pdev->dev, "%s ADMAIF removed", __func__);
	return 0;
}

static const struct of_device_id tegra210_admaif_of_match[] = {
	{ .compatible = "nvidia,tegra210-admaif"},
	{},
};

static const struct dev_pm_ops tegra210_admaif_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_admaif_runtime_suspend,
			   tegra210_admaif_runtime_resume, NULL)
};

static struct platform_driver tegra210_admaif_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_admaif_of_match,
		.pm = &tegra210_admaif_pm_ops,
	},
	.probe = tegra210_admaif_probe,
	.remove = tegra210_admaif_remove,
};
module_platform_driver(tegra210_admaif_driver);

MODULE_AUTHOR("Dara Ramesh <dramesh@nvidia.com>");
MODULE_DESCRIPTION("Tegra APE ADMAIF driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
