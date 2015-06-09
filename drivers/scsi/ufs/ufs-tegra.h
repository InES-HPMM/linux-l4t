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


#ifndef _UFS_TEGRA_H
#define _UFS_TEGRA_H

#include <linux/io.h>

#define NV_ADDRESS_MAP_MPHY_L0_BASE	0x02470000
#define NV_ADDRESS_MAP_MPHY_L1_BASE	0x02480000
#define NV_ADDRESS_MAP_UFSHC_AUX_BASE		0x2460000

/*
 * M-PHY Registers
 */

#define MPHY_TX_APB_TX_CG_OVR0_0	0x170
#define MPHY_TX_CLK_EN_SYMB	(1 << 1)
#define MPHY_TX_CLK_EN_SLOW	(1 << 3)
#define MPHY_TX_CLK_EN_FIXED	(1 << 4)
#define MPHY_TX_CLK_EN_3X	(1 << 5)

#define MPHY_TX_APB_TX_ATTRIBUTE_34_37_0	0x34
#define TX_ADVANCED_GRANULARITY		(0x8 << 16)
#define TX_ADVANCED_GRANULARITY_SETTINGS	(0x1 << 8)
#define MPHY_GO_BIT	1

#define MPHY_TX_APB_TX_VENDOR0_0	0x100
#define MPHY_ADDR_RANGE		400

/* vendor specific pre-defined parameters */

/*
 * HCLKFrequency in MHz.
 * HCLKDIV is used to generate 1usec tick signal used by Unipro.
 */
#define UFS_VNDR_HCLKDIV_1US_TICK	0x33


/*UFS host controller vendor specific registers */
enum {
	REG_UFS_VNDR_HCLKDIV	= 0xFC,
};

/*
 * UFS AUX Registers
 */

#define UFSHC_AUX_UFSHC_DEV_CTRL_0	0x14
#define UFSHC_DEV_CLK_EN		(1 << 0)
#define UFSHC_DEV_RESET			(1 << 1)
#define UFSHC_AUX_UFSHC_SW_EN_CLK_SLCG_0	0x08
#define UFSHC_CLK_OVR_ON	(1 << 0)
#define UFSHC_HCLK_OVR_ON	(1 << 1)
#define UFSHC_LP_CLK_T_CLK_OVR_ON	(1 << 2)
#define UFSHC_CLK_T_CLK_OVR_ON		(1 << 3)
#define UFSHC_CG_SYS_CLK_OVR_ON		(1 << 4)
#define UFSHC_TX_SYMBOL_CLK_OVR_ON	(1 << 5)
#define UFSHC_RX_SYMBOLCLKSELECTED_CLK_OVR_ON		(1 << 6)
#define UFSHC_PCLK_OVR_ON		(1 << 7)

struct ufs_tegra_host {
	struct ufs_hba *hba;
	struct phy *u_phy;
	bool is_lane_clks_enabled;
	bool x2config;
	void __iomem *mphy_l0_base;
	void __iomem *mphy_l1_base;
	void __iomem *ufs_aux_base;
	struct clk *mphy_core_pll_fixed;
	struct clk *mphy_l0_tx_symb;
	struct clk *mphy_tx_1mhz_ref;
	struct clk *mphy_l0_rx_ana;
	struct clk *mphy_l0_rx_symb;
	struct clk *mphy_l0_tx_ls_3xbit;
	struct clk *mphy_l0_rx_ls_bit;
	struct clk *mphy_l1_rx_ana;
};

extern struct ufs_hba_variant_ops ufs_hba_tegra_vops;

static inline u32 mphy_readl(void __iomem *mphy_base, u32 offset)
{
	u32 val;

	val = readl(mphy_base + offset);
	return val;
}

static inline void mphy_writel(void __iomem *mphy_base, u32 val, u32 offset)
{
	writel(val, mphy_base + offset);
}

static inline void mphy_update(void __iomem *mphy_base, u32 val,
								u32 offset)
{
	u32 update_val;

	update_val = mphy_readl(mphy_base, offset);
	update_val |= val;
	mphy_writel(mphy_base, update_val, offset);
}

static inline u32 ufs_aux_readl(void __iomem *ufs_aux_base, u32 offset)
{
	u32 val;

	val = readl(ufs_aux_base + offset);
	return val;
}

static inline void ufs_aux_writel(void __iomem *ufs_aux_base, u32 val,
								u32 offset)
{
	writel(val, ufs_aux_base + offset);
}

static inline void ufs_aux_update(void __iomem *ufs_aux_base, u32 val,
								u32 offset)
{
	u32 update_val;

	update_val = ufs_aux_readl(ufs_aux_base, offset);
	update_val |= val;
	ufs_aux_writel(ufs_aux_base, update_val, offset);
}

#endif
