/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Authors:
 *      VenkataJagadish.p	<vjagadish@nvidia.com>
 *      Naveen Kumar Arepalli	<naveenk@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/time.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>

#include "ufshcd.h"
#include "unipro.h"
#include "ufs-tegra.h"
#include "ufshci.h"

#define UFS_TEGRA_ENABLE_MPHY 0

#if UFS_TEGRA_ENABLE_MPHY

static int ufs_tegra_host_clk_get(struct device *dev,
		const char *name, struct clk **clk_out)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(dev, "%s: failed to get %s err %d",
				__func__, name, err);
	} else {
		*clk_out = clk;
	}

	return err;
}

static int ufs_tegra_host_clk_enable(struct device *dev,
		const char *name, struct clk *clk)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		dev_err(dev, "%s: %s enable failed %d\n", __func__, name, err);

	return err;
}

static void ufs_tegra_disable_mphylane_clks(struct ufs_tegra_host *host)
{
	if (!host->is_lane_clks_enabled)
		return;

	clk_disable_unprepare(host->mphy_core_pll_fixed);
	clk_disable_unprepare(host->mphy_l0_tx_symb)
	clk_disable_unprepare(host->mphy_tx_1mhz_ref);
	clk_disable_unprepare(host->mphy_l0_rx_ana);
	clk_disable_unprepare(host->mphy_l0_rx_symb);
	clk_disable_unprepare(host->mphy_l0_tx_ls_3xbit);
	clk_disable_unprepare(host->mphy_l0_rx_ls_bit);

	if (host->x2config)
		clk_disable_unprepare(host->mphy_l1_rx_ana);

	host->is_lane_clks_enabled = false;
}

static int ufs_tegra_enable_mphylane_clks(struct ufs_tegra_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	if (host->is_lane_clks_enabled)
		return 0;

	err = ufs_tegra_host_clk_enable(dev, "mphy_core_pll_fixed",
		host->mphy_core_pll_fixed);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_tx_symb",
		host->mphy_l0_tx_symb);
	if (err)
		goto disable_l0_rx_symb;

	err = ufs_tegra_host_clk_enable(dev, "mphy_tx_1mhz_ref",
		host->mphy_tx_1mhz_ref);
	if (err)
		goto disable_tx_1mhz_ref;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_ana",
		host->mphy_l0_rx_ana);
	if (err)
		goto disable_rx_l1;

	err = ufs_tegra_host_clk_enable(dev, "mphy_lo_rx_symb",
		host->mphy_l0_rx_symb);
	if (err)
		goto disable_rx_symb;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_tx_ls_3xbit",
		host->mphy_l0_tx_ls_3xbit);
	if (err)
		goto disable_l0_tx_ls_3xbit;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_ls_bit",
		host->mphy_l0_rx_ls_bit);
	if (err)
		goto disable_l0_rx_ls_bit;

	if (host->x2config) {
		err = ufs_tegra_host_clk_enable(dev, "mphy_l1_rx_ana",
			host->mphy_l1_rx_ana);
		if (err)
			goto disable_l1_rx_ana;
	}

	host->is_lane_clks_enabled = true;
	goto out;

disable_l1_rx_ana;
	if (host->x2config)
		clk_disable_unprepare(host->mphy_l0_rx_ls_bit);
disable_lo_rx_ls_bit:
	clk_disable_unprepare(host->mphy_l0_tx_ls_3xbit);
disable_l0_tx_ls_3xbit:
	clk_disable_unprepare(host->mphy_l0_rx_symb);
disable_l0_rx_symb:
	clk_disable_unprepare(host->mphy_l0_rx_ana);
disable_l0_rx_ana:
	clk_disable_unprepare(host->mphy_tx_1mhz_ref);
disable_tx_1mhz_ref:
	clk_disable_unprepare(host->mphy_l0_tx_symb)
disable_l0_tx_symb:
	clk_disable_unprepare(host->mphy_core_pll_fixed);
out:
	return err;
}

static int ufs_tegra_init_lane_clks(struct ufs_tegra_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	err = ufs_tegra_host_clk_get(dev,
			"mphy_core_pll_fixed", &host->mphy_core_pll_fixed);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev,
			"mphy_l0_tx_symb", &host->mphy_l0_tx_symb);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_tx_1mhz_ref",
		&host->mphy_tx_1mhz_ref);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_ana",
		&host->mphy_l0_rx_ana);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_symb",
		&host->mphy_l0_rx_symb);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_tx_ls_3xbit",
		&host->mphy_l0_tx_ls_3xbit);
	if (err)
		goto out;

	if (host->x2config)
		err = ufs_tegra_host_clk_get(dev, "mphy_l1_rx_ana",
			&host->mphy_l1_rx_ana);
out:
	return err;
}

static int ufs_tegra_power_up_sequence(struct ufs_hba *hba)
{
	int ret = 0;

	return ret;
}

void ufs_tegra_disable_mphy_slcg(struct ufs_tegra_host *ufs_tegra)
{
	u32 val;

	val = mphy_readl(ufs_tegra->mphy_l0_base, MPHY_TX_APB_TX_CG_OVR0_0);
	val |= (MPHY_TX_CLK_EN_SYMB | MPHY_TX_CLK_EN_SLOW |
			MPHY_TX_CLK_EN_FIXED | MPHY_TX_CLK_EN_3X);
	mphy_writel(ufs_tegra->mphy_l0_base, val, MPHY_TX_APB_TX_CG_OVR0_0);

	val = mphy_readl(ufs_tegra->mphy_l1_base, MPHY_TX_APB_TX_CG_OVR0_0);
	val |= (MPHY_TX_CLK_EN_SYMB | MPHY_TX_CLK_EN_SLOW |
			MPHY_TX_CLK_EN_FIXED | MPHY_TX_CLK_EN_3X);
	mphy_writel(ufs_tegra->mphy_l1_base, val, MPHY_TX_APB_TX_CG_OVR0_0);

}

void ufs_tegra_mphy_advgran(struct ufs_tegra_host *ufs_tegra)
{
	u32 val;

	val = mphy_readl(ufs_tegra->mphy_l0_base,
				MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
	val |= (TX_ADVANCED_GRANULARITY | TX_ADVANCED_GRANULARITY_SETTINGS);
	mphy_writel(ufs_tegra->mphy_l0_base, val,
				MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
	mphy_writel(ufs_tegra->mphy_l0_base, MPHY_GO_BIT, MPHY_TX_APB_TX_VENDOR0_0);

	val = mphy_readl(ufs_tegra->mphy_l1_base,
				MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
	val |= (TX_ADVANCED_GRANULARITY | TX_ADVANCED_GRANULARITY_SETTINGS);
	mphy_writel(ufs_tegra->mphy_l1_base, val,
			MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
	mphy_writel(ufs_tegra->mphy_l1_base, MPHY_GO_BIT, MPHY_TX_APB_TX_VENDOR0_0);

}

#endif

static int ufs_tegra_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	int ret = 0;

	/*
	 * Disable mphy tx/rx lane clocks if they are on
	 * and powerdown UPHY
	 */

out:
	return ret;
}

static int ufs_tegra_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	int ret = 0;

	/*
	 * Power on UPHY
	 */

out:
	return ret;
}

/**
 * ufs_tegra_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up UPHY enabling clocks
 * and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_tegra_init(struct ufs_hba *hba)
{
#if UFS_TEGRA_ENABLE_MPHY
	struct ufs_tegra_host *ufs_tegra;
	struct device *dev = hba->dev;
#endif
	int err = 0;
#if UFS_TEGRA_ENABLE_MPHY
	ufs_tegra->mphy_l0_base = devm_ioremap(dev,
				NV_ADDRESS_MAP_MPHY_L0_BASE, MPHY_ADDR_RANGE);
	ufs_tegra->mphy_l1_base = devm_ioremap(dev,
				NV_ADDRESS_MAP_MPHY_L1_BASE, MPHY_ADDR_RANGE);
#endif

	return err;
}

static void ufs_tegra_exit(struct ufs_hba *hba)
{

}

/**
 * struct ufs_hba_tegra_vops - UFS TEGRA specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initialization.
 */
struct ufs_hba_variant_ops ufs_hba_tegra_vops = {
	.name                   = "ufs-tegra",
	.init                   = ufs_tegra_init,
	.exit                   = ufs_tegra_exit,
	.suspend		= ufs_tegra_suspend,
	.resume			= ufs_tegra_resume,
};
