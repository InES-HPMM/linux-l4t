/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-soc.h>
#include <linux/clk/tegra.h>

#define TEGRA_XUSB_PCIE_PHYS 1
#define TEGRA_XUSB_SATA_PHYS 1
#define TEGRA_XUSB_UFS_PHYS 1
#define TEGRA_XUSB_USB3_PHYS 3
#define TEGRA_XUSB_UTMI_PHYS 3
#define TEGRA_XUSB_HSIC_PHYS 1
#define TEGRA_XUSB_NUM_USB_PHYS (TEGRA_XUSB_USB3_PHYS + TEGRA_XUSB_UTMI_PHYS + \
				 TEGRA_XUSB_HSIC_PHYS)
#define TEGRA_XUSB_NUM_PHYS (TEGRA_XUSB_NUM_USB_PHYS + TEGRA_XUSB_PCIE_PHYS + \
				TEGRA_XUSB_SATA_PHYS + TEGRA_XUSB_UFS_PHYS)

#include <dt-bindings/pinctrl/pinctrl-tegra-xusb.h>
#include "../../../arch/arm/mach-tegra/iomap.h" /* FIXME */

#ifdef TRACE
#undef TRACE
#endif
#define TRACE(fmt, args...) pr_info("%s %d " fmt "\n", __func__, __LINE__, ## args)

#ifdef TRACE_DEV
#undef TRACE_DEV
#endif
#define TRACE_DEV(dev, fmt, args...) dev_info(dev, "%s %d " fmt "\n", __func__, __LINE__, ## args)

#include "core.h"
#include "pinctrl-utils.h"

#define T186_UPHY_LANES				(6)

/* UPHY PLL registers */
#define UPHY_PLL_CTL_1				(0x0)
#define   PLL_IDDQ				(1 << 0)
#define   PLL_SLEEP(x)				(((x) & 0x3) << 1)
#define   PLL_ENABLE				(1 << 3)
#define   PWR_OVRD				(1 << 4)
#define   RATE_ID(x)				(((x) & 0x3) << 8)
#define   RATE_ID_OVRD				(1 << 11)
#define   LOCKDET_STATUS			(1 << 15)

#define UPHY_PLL_CTL_2				(0x4)
#define   CAL_EN				(1 << 0)
#define   CAL_DONE				(1 << 1)
#define   CAL_OVRD				(1 << 2)
#define   CAL_RESET				(1 << 3)
#define   RCAL_EN				(1 << 12)
#define   RCAL_CLK_EN				(1 << 13)
#define   RCAL_OVRD				(1 << 15)
#define   RCAL_DONE				(1 << 31)

#define UPHY_PLL_CTL_4				(0xc)

/* UPHY Lane registers */
#define UPHY_LANE_AUX_CTL_1			(0x0)
#define   AUX_RX_IDLE_TH(x)			(((x) & 0x3) << 24)

#define UPHY_LANE_DIRECT_CTL_1			(0x10)
#define   MISC_CTRL(x)				(((x) & 0xff) << 0)
#define   MISC_OUT(x)				(((x) & 0xff) << 16)

#define UPHY_LANE_DIRECT_CTL_2			(0x14)
#define   CFG_WDATA(x)				(((x) & 0xffff) << 0)
#define   CFG_ADDR(x)				(((x) & 0xff) << 16)
#define   CFG_WDS(x)                            (((x) & 0x1) << 24)
#define   CFG_RDS(x)                            (((x) & 0x1) << 25)
#define   CFG_RESET(x)                          (((x) & 0x1) << 27)

#define UPHY_LANE_MUX				(0x284)
#define   SEL(x)				(((x) & 0x7) << 0)
#define     SEL_XUSB				SEL(0)
#define     SEL_PCIE				SEL(1)
#define     SEL_SATA				SEL(2)
#define     SEL_MPHY				SEL(3)
#define   CLAMP_EN_EARLY			(1 << 8)
#define   FORCE_IDDQ_DISABLE			(1 << 9)

/* UPHY APB Dynamic DYN_CTL registers */
#define UPHY_LANE_DYN_CTL_1			(0x80)
#define   TX_DRV_AMP_SEL0(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_AMP_SEL1(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_AMP_SEL2(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_AMP_SEL3(x)			(((x) & 0x3f) << 24)

#define UPHY_LANE_DYN_CTL_2			(0x84)
#define   TX_DRV_AMP_SEL4(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_AMP_SEL5(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_AMP_SEL6(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_AMP_SEL7(x)			(((x) & 0x3f) << 24)

#define UPHY_LANE_DYN_CTL_3			(0x88)
#define   TX_DRV_AMP_SEL8(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_AMP_SEL9(x)			(((x) & 0x3f) << 8)

#define UPHY_LANE_DYN_CTL_4			(0x8c)
#define   TX_DRV_POST_SEL0(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_PRE_SEL0(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_POST_SEL1(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_PRE_SEL1(x)			(((x) & 0x3f) << 24)

#define UPHY_LANE_DYN_CTL_5			(0x90)
#define   TX_DRV_POST_SEL2(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_PRE_SEL2(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_POST_SEL3(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_PRE_SEL3(x)			(((x) & 0x3f) << 24)

/* FUSE USB_CALIB registers */ /* TODO: check spec */
#define USB_CALIB_HS_CURR_LEVEL_PADX_SHIFT(x)	((x) ? (11 + (x - 1) * 6) : 0)
#define USB_CALIB_HS_CURR_LEVEL_PAD_MASK	(0x3f)
#define USB_CALIB_HS_TERM_RANGE_ADJ_SHIFT	(7)
#define USB_CALIB_HS_TERM_RANGE_ADJ_MASK	(0xf)

#define USB_CALIB_EXT_RPD_CTRL_SHIFT		(7)
#define USB_CALIB_EXT_RPD_CTRL_MASK		(0xf)

/* FUSE SATA MPHY registers */
#define FUSE_SATA_MPHY_ODM_CALIB_0	(0x224)
#define    SATA_MPHY_ODM_CALIB_0_1(x)		(((x) & 0x3) << 0)

#define FUSE_SATA_NV_CALIB_0		(0x49c)
#define    SATA_NV_CALIB_0_1(x)			(((x) & (0x3 << 0)) >> 0)
#define    SATA_NV_CALIB_2_3(x)			(((x) & (0x3 << 2)) >> 2)

#define FUSE_MPHY_NV_CALIB_0		(0x4a0)
#define    MPHY_NV_CALIB_0_1(x)			(((x) & (0x3 << 0)) >> 0)
#define    MPHY_NV_CALIB_2_3(x)			(((x) & (0x3 << 2)) >> 2)
#define    MPHY_NV_CALIB_4_5(x)			(((x) & (0x3 << 4)) >> 4)

/* XUSB PADCTL registers */
#define XUSB_PADCTL_USB2_PAD_MUX		(0x4)

#define XUSB_PADCTL_USB2_PORT_CAP		(0x8)
#define XUSB_PADCTL_SS_PORT_CAP			(0xc)
#define   PORTX_CAP_SHIFT(x)			((x) * 4)
#define   PORT_CAP_MASK				(0x3)
#define     PORT_CAP_DISABLED			(0x0)
#define     PORT_CAP_HOST			(0x1)
#define     PORT_CAP_DEVICE			(0x2)
#define     PORT_CAP_OTG			(0x3)

#define XUSB_PADCTL_ELPG_PROGRAM 		(0x20)

#define XUSB_PADCTL_ELPG_PROGRAM_1		(0x24)
#define   SSPX_ELPG_CLAMP_EN(x)			(1 << (0 + (x) * 3))
#define   SSPX_ELPG_CLAMP_EN_EARLY(x)		(1 << (1 + (x) * 3))
#define   SSPX_ELPG_VCORE_DOWN(x)		(1 << (2 + (x) * 3))

#define USB2_BATTERY_CHRG_OTGPADX_CTL1(x)	(0x84 + (x) * 0x40)
#define   VREG_LEV(x)				(((x) & 0x3) << 7)
#define   VREG_FIX18				(1 << 6)

#define XUSB_PADCTL_USB2_OTG_PADX_CTL0(x)	(0x88 + (x) * 0x40)
#define   HS_CURR_LEVEL(x)			((x) & 0x3f)
#define   USB2_OTG_PD				(1 << 26)
#define   USB2_OTG_PD2				(1 << 27)
#define   USB2_OTG_PD_ZI			(1 << 29)

#define XUSB_PADCTL_USB2_OTG_PADX_CTL1(x)	(0x8c + (x) * 0x40)
#define   USB2_OTG_PD_DR			(1 << 2)
#define   TERM_RANGE_ADJ(x)			(((x) & 0xf) << 3)
#define   RPD_CTRL(x)				(((x) & 0x1f) << 26)

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0		(0x284)
#define   BIAS_PAD_PD				(1 << 11)

#define USB2_VBUS_ID				(0x360)
#define   VBUS_OVERRIDE				(1 << 14)

/* XUSB AO registers */
#define XUSB_AO_UTMIP_TRIGGERS(x)		(0x40 + (x) * 4)
#define   CLR_WALK_PTR				(1 << 0)
#define   CAP_CFG				(1 << 1)
#define   CLR_WAKE_ALARM			(1 << 3)

#define XUSB_AO_UTMIP_SLEEPWALK_CFG(x)		(0xd0 + (x) * 4)
#define XUSB_AO_UHSIC_SLEEPWALK_CFG(x)		(0xf0 + (x) * 4)
#define   MASTER_ENABLE				(1 << 15)
#define   MASTER_CFG_SEL			(1 << 22)

#define XUSB_AO_UTMIP_SLEEPWALK(x)		(0x120 + (x) * 4)
#if 0 /* TODO: implement */
#define padctl_ao_readl		padctl_readl /* FIXME */
#define padctl_ao_writel	padctl_writel /* FIXME */
static int utmip_setup_sleepwalk(struct tegra_padctl_uphy *ctx, int pad)
{
	u32 reg;

	/* ensure sleepwalk logic is disabled */
	reg = padctl_ao_readl(ctx, XUSB_AO_UTMIP_SLEEPWALK_CFG(pad));
	reg &= ~MASTER_ENABLE;
	padctl_ao_writel(ctx, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(pad));


	return 0;
}
#endif
/* UPHY PLL config space registers */
#define MGMT_FREQ_CTRL_ID0			(0)
#define MGMT_FREQ_CTRL_ID1			(1)
#define   CFG_FREQ_NDIV(x)			(((x) & 0xff) << 0)
#define   CFG_FREQ_MDIV(x)			(((x) & 0x3) << 8)
#define   CFG_FREQ_PSDIV(x)			(((x) & 0x3) << 10)
#define   CFG_MODE(x)				(((x) & 0x3) << 12)

#define MGMT_REFCLK_CTRL			(2)

#define MGMT_CORECLK_CTRL_ID0			(3)
#define MGMT_CORECLK_CTRL_ID1			(4)
#define   CFG_TXCLKREF_EN(x)			(((x) & 0x1) << 0)
#define   CFG_TXCLKREF_SEL(x)			(((x) & 0x3) << 4)
#define     DIVIDE_TX_BY_10			(0)
#define     DIVIDE_TX_BY_8			(1)
#define     DIVIDE_TX_BY_5			(2)
#define     DIVIDE_TX_BY_4			(3)
#define   CFG_XDIGCLK_EN(x)			(((x) & 0x1) << 8)
#define   CFG_XDIGCLK_SEL(x)			(((x) & 0x3) << 12)

#define PLLC_CRSWRD_OVRD_ID0			(5)

#define PLLC_CRSWRD_OVRD_ID1			(6)

#define MGMT_CYA_CTRL				(29)
#define   CFG_MGMT_CLK_SEL(x)			(((x) & 0x1) << 4)

/* UPHY Lane config space registers */
#define MGMT_TX_RATE_CTRL_ID0			(0)
#define MGMT_TX_RATE_CTRL_ID1			(1)
#define MGMT_TX_RATE_CTRL_ID2			(2)
#define MGMT_TX_RATE_CTRL_ID3			(3)
#define   TX_RATE_SDIV(x)			(((x) & 0x3) << 0)
#define     SDIV4				(0)
#define     SDIV2				(1)
#define     SDIV1				(2)
#define     SDIVX				(3)

#define MGMT_RX_RATE_CTRL_ID0			(4)
#define MGMT_RX_RATE_CTRL_ID1			(5)
#define MGMT_RX_RATE_CTRL_ID2			(6)
#define MGMT_RX_RATE_CTRL_ID3			(7)
#define   RX_RATE_SDIV(x)			(((x) & 0x3) << 0)
#define   RX_RATE_CDIV(x)			(((x) & 0x3) << 4)
#define     CDIV1				(0)
#define     CDIV2				(1)
#define     CDIV4				(2)
#define     CDIV8				(3)

#define MGMT_TX_CTRL				(8)
#define   TX_TERM_MODE(x)			(((x) & 0x1) << 0)
#define   SYNC_DLY(x)				(((x) & 0xf) << 4)

#define AE_CTLE_CTRL_ID0			(10)
#define AE_CTLE_CTRL_ID1			(11)
#define AE_CTLE_CTRL_ID2			(12)
#define AE_CTLE_CTRL_ID3			(13)
#define   LF_UGRAY(x)				(((x) & 0xf) << 0)
#define   HF_UBIN(x)				(((x) & 0xf) << 4)

#define AE_DFE0_CTRL_ID0			(14)
#define AE_DFE0_CTRL_ID1			(15)
#define AE_DFE0_CTRL_ID2			(16)
#define AE_DFE1_CTRL_ID2			(20)
#define   H1_SBIN(x)				(((x) & 0x1f) << 0)
#define   H2_SBIN(x)				(((x) & 0xf) << 8)
#define   H3_SBIN(x)				(((x) & 0x7) << 12)

#define AE_DFE1_CTRL_ID0			(18)
#define   H4_SBIN(x)				(((x) & 0x7) << 0)
#define   H0_SBIN(x)				(((x) & 0x3f) << 8)

#define AE_CDR_CTRL_ID0				(22)
#define AE_CDR_CTRL_ID1				(23)
#define AE_CDR_CTRL_ID2				(24)
#define AE_CDR_CTRL_ID3				(25)
#define   PHGAIN(x)				(((x) & 0xf) << 0)
#define   FRGAIN(x)				(((x) & 0xf) << 4)
#define   FRLOOP_EN(x)				(((x) & 0x1) << 8)

#define AE_EQ0_CTRL_ID0				(26)
#define AE_EQ0_CTRL_ID1				(27)
#define AE_EQ0_CTRL_ID2				(28)
#define   EQ0_SEQ_MODE(x)			(((x) & 0x3) << 0)
#define     SEQ_MODE_HF_Z_H0_HN_Z		(0)
#define     SEQ_MODE_H0_HN_Z_HF_Z		(1)
#define     SEQ_MODE_HF_H0_HN_Z			(2)
#define     SEQ_MODE_HF_H0_HN_Z_HF		(3)
#define   H0INIT_ITERS(x)			(((x) & 0xf) << 4)
#define   H4_GRAD_INV(x)			(((x) & 0x1) << 8)
#define   H3_GRAD_INV(x)			(((x) & 0x1) << 9)
#define   H2_GRAD_INV(x)			(((x) & 0x1) << 10)
#define   H1_GRAD_INV(x)			(((x) & 0x1) << 11)
#define   H0_GRAD_INV(x)			(((x) & 0x1) << 12)
#define   CTLE_HF_GRAD_INV(x)			(((x) & 0x1) << 13)
#define   SAMP_VOS_GRAD_INV(x)			(((x) & 0x1) << 14)
#define   CTLE_VOS_GRAD_INV(x)			(((x) & 0x1) << 15)

#define AE_EQ1_CTRL_ID0				(30)
#define AE_EQ1_CTRL_ID1				(31)
#define AE_EQ1_CTRL_ID2				(32)
#define AE_EQ1_CTRL_ID3				(33)
#define   CTLE_HF_MODE(x)			(((x) & 0x3) << 0)
#define     VAR_MODE_AUTO			(0)
#define     VAR_MODE_OFF			(1)
#define     VAR_MODE_HOLD			(2)
#define     VAR_MODE_LOAD			(3)
#define   CTLE_HF_GRAD(x)			(((x) & 0x3) << 2)
#define      CTLE_HF_GRAD_1P5			(0)
#define      CTLE_HF_GRAD_2P5			(1)
#define      CTLE_HF_GRAD_3P5			(2)
#define      CTLE_HF_GRAD_1TP			(3)
#define   H1_MODE(x)				(((x) & 0x3) << 8)
#define   H2_MODE(x)				(((x) & 0x3) << 10)
#define   H3_MODE(x)				(((x) & 0x3) << 12)
#define   H4_MODE(x)				(((x) & 0x3) << 14)

#define AE_EQ2_CTRL_ID0				(34)
#define AE_EQ2_CTRL_ID1				(35)
#define AE_EQ2_CTRL_ID2				(36)
#define AE_EQ2_CTRL_ID3				(37)
#define   H0_MODE(x)				(((x) & 0x3) << 0)
#define   H0_DAC_TIME(x)			(((x) & 0x3) << 2)
#define   H0_TIME(x)				(((x) & 0xf) << 4)
#define   H0_ITERS(x)				(((x) & 0xf) << 8)
#define   HN_ITERS(x)				(((x) & 0xf) << 12)

#define AE_EQ3_CTRL_ID0				(38)
#define AE_EQ3_CTRL_ID2				(40)
#define   H0_GAIN(x)				(((x) & 0xf) << 0)
#define   HN_GAIN(x)				(((x) & 0xf) << 4)
#define   CTLE_HF_TIME(x)			(((x) & 0xf) << 8)
#define   CTLE_HF_GAIN(x)			(((x) & 0xf) << 12)

#define MGMT_RX_PI_CTRL_ID2			(48)
#define   TX_SLEW(x)				(((x) & 0x3) << 0)
#define   RX_SLEW(x)				(((x) & 0x3) << 2)

#define CLK_RST_CONTROLLER_RST_DEV_UPHY_0		(0x4000c)
#define   SWR_UPHY_RST					(1 << 0)

#define CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0	(0x40000)
#define   SWR_PEX_USB_UPHY_RST				(1 << 0)
#define   SWR_PEX_USB_UPHY_PLL0_RST			(1 << 7)
#define   SWR_PEX_USB_UPHY_PLL1_RST			(1 << 15)
#define   SWR_PEX_USB_UPHY_LANE_RST(lane)		(1 << (16+lane))

enum tegra186_function {
	TEGRA186_FUNC_HSIC,
	TEGRA186_FUNC_HSIC_PLUS,
	TEGRA186_FUNC_XUSB,
	TEGRA186_FUNC_UART,
	TEGRA186_FUNC_PCIE,
	TEGRA186_FUNC_USB3,
	TEGRA186_FUNC_SATA,
	TEGRA186_FUNC_MPHY,
	TEGRA186_FUNC_RSVD_0,
	TEGRA186_FUNC_RSVD_3,
};

struct tegra_padctl_uphy_function {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct tegra_padctl_uphy_group {
	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_padctl_uphy_soc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;

	const struct tegra_padctl_uphy_function *functions;
	unsigned int num_functions;

	const struct tegra_padctl_uphy_lane *lanes;
	unsigned int num_lanes;

	unsigned int hsic_port_offset;
};

struct tegra_padctl_uphy_lane {
	const char *name;

	unsigned int offset;
	unsigned int shift;
	unsigned int mask;
	unsigned int iddq;

	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_xusb_fuse_calibration {
	u32 hs_curr_level[TEGRA_XUSB_UTMI_PHYS];
	u32 hs_term_range_adj;
	u32 rpd_ctrl;
};

struct tegra_fuse_calibration {
	u32 sata_mphy_odm;
	u32 sata_nv;
	u32 mphy_nv;
};

struct tegra_xusb_usb3_port {
	unsigned int lane;
};

struct tegra_padctl_uphy {
	struct device *dev;
	void __iomem *padctl_regs;
	void __iomem *uphy_regs;
	void __iomem *uphy_pll_regs[2];
	void __iomem *uphy_lane_regs[6];

	struct clk *clk; /* uphy pad macro clock */

	struct mutex lock;

	const struct tegra_padctl_uphy_soc *soc;
	struct tegra_xusb_fuse_calibration calib;
	struct tegra_fuse_calibration fuse_calib;
	struct pinctrl_dev *pinctrl;
	struct pinctrl_desc desc;

	struct phy_provider *provider;
//	struct phy *phys[TEGRA_XUSB_NUM_PHYS];
	struct phy *usb3_phys[TEGRA_XUSB_USB3_PHYS];
	struct phy *utmi_phys[TEGRA_XUSB_UTMI_PHYS];
	struct phy *hsic_phys[TEGRA_XUSB_HSIC_PHYS];
	struct phy *pcie_phys[TEGRA_XUSB_PCIE_PHYS];
	struct phy *sata_phys[TEGRA_XUSB_SATA_PHYS];
	struct phy *ufs_phys[TEGRA_XUSB_UFS_PHYS];
	unsigned long usb3_lanes;
	unsigned long pcie_lanes;
	unsigned long sata_lanes;
	unsigned long ufs_lanes;

	unsigned int padctl_clients;

	struct tegra_xusb_usb3_port usb3_ports[TEGRA_XUSB_USB3_PHYS];
	unsigned int utmi_enable;
	unsigned int hs_curr_level_offset[TEGRA_XUSB_UTMI_PHYS];
	struct regulator *vbus[TEGRA_XUSB_UTMI_PHYS];
	struct regulator *vddio_hsic;

	int uphy_pll_clients[2];
	struct clk *uphy_pll_clk[2];
	struct clk *uphy_lane_clk[6];

};

#ifdef SIM
#undef writel
#define writel(v,c) {*(u32 *)(c) = v;}
#undef readl
#define readl(c) ({ *(u32 *)(c);})
#endif

#if 0
static inline void padctl_writel(struct tegra_padctl_uphy *padctl, u32 value,
				 unsigned long offset)
{
	writel(value, padctl->padctl_regs + offset);
}

static inline u32 padctl_readl(struct tegra_padctl_uphy *padctl,
			       unsigned long offset)
{
	return readl(padctl->padctl_regs + offset);
}
#else
#define padctl_writel(_padctl, _value, _offset)				\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_info("%s padctl_write %s(@0x%lx) with 0x%lx\n", __func__,	\
		#_offset, o, v);					\
	writel(v, _padctl->padctl_regs + o);					\
}

#define padctl_readl(_padctl, _offset)					\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_padctl->padctl_regs + o);				\
	pr_info("%s padctl_read %s(@0x%lx) = 0x%lx\n", __func__,	\
		#_offset, o, v);					\
	v;								\
})
#endif

#if 0
static inline void uphy_pll_writel(struct tegra_padctl_uphy *ctx, int pll,
				   u32 value, unsigned long offset)
{
	writel(value, ctx->uphy_pll_regs[pll] + offset);
}

static inline u32 uphy_pll_readl(struct tegra_padctl_uphy *ctx, int pll,
				 unsigned long offset)
{
	return readl(ctx->uphy_pll_regs[pll] + offset);
}
#else
#define uphy_pll_writel(_ctx, _pll, _value, _offset)			\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_info("%s uphy_pll_writel pll %d %s(@0x%lx) with 0x%lx\n", __func__, \
		_pll, #_offset, o, v);					\
	writel(v, _ctx->uphy_pll_regs[_pll] + o);			\
}

#define uphy_pll_readl(_ctx, _pll, _offset)				\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_ctx->uphy_pll_regs[_pll] + o);			\
	pr_info("%s uphy_pll_readl pll %d %s(@0x%lx) = 0x%lx\n", __func__, \
		_pll, #_offset, o, v);					\
	v;								\
})
#endif

#if 0
static inline void uphy_lane_writel(struct tegra_padctl_uphy *ctx, int lane,
				   u32 value, unsigned long offset)
{
	writel(value, ctx->uphy_lane_regs[lane] + offset);
}

static inline u32 uphy_pll_readl(struct tegra_padctl_uphy *ctx, int lane,
				 unsigned long offset)
{
	return readl(ctx->uphy_lane_regs[lane] + offset);
}
#else
#define uphy_lane_writel(_ctx, _lane, _value, _offset)			\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_info("%s uphy_lane_writel lane %d %s(@0x%lx) with 0x%lx\n", __func__, \
		_lane, #_offset, o, v);					\
	writel(v, _ctx->uphy_lane_regs[_lane] + o);			\
}

#define uphy_lane_readl(_ctx, _lane, _offset)				\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_ctx->uphy_lane_regs[_lane] + o);			\
	pr_info("%s uphy_lane_readl lane %d %s(@0x%lx) = 0x%lx\n", __func__, \
		_lane, #_offset, o, v);					\
	v;								\
})
#endif

struct tegra_mphy_sata_calib {
	u8 aux_rx_idle_th;
	u8 tx_drv_amp_sel0;
	u8 tx_drv_amp_sel1;
	u8 tx_drv_amp_sel2;
	u8 tx_drv_amp_sel3;
	u8 tx_drv_amp_sel4;
	u8 tx_drv_amp_sel5;
	u8 tx_drv_amp_sel6;
	u8 tx_drv_amp_sel7;
	u8 tx_drv_amp_sel8;
	u8 tx_drv_amp_sel9;
	u8 tx_drv_post_sel0;
	u8 tx_drv_post_sel1;
	u8 tx_drv_post_sel2;
	u8 tx_drv_post_sel3;
	u8 tx_drv_pre_sel3;
	u8 ae_ctle_ctrl_id0;
	u8 ae_ctle_ctrl_id1;
};

static struct tegra_mphy_sata_calib mphy_data[] = {
	{
		.aux_rx_idle_th = 0x0,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
	{
		.aux_rx_idle_th = 0x1,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
	{
		.aux_rx_idle_th = 0x2,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
	{
		.aux_rx_idle_th = 0x3,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
};

static struct tegra_mphy_sata_calib sata_data[] = {
	{
		.aux_rx_idle_th = 0x0,
		.tx_drv_amp_sel0 = 0x1b,
		.tx_drv_amp_sel1 = 0x1f,
		.tx_drv_post_sel0 = 0x7,
		.tx_drv_post_sel1 = 0xa,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0x8f,
	},
	{
		.aux_rx_idle_th = 0x1,
		.tx_drv_amp_sel0 = 0x17,
		.tx_drv_amp_sel1 = 0x1b,
		.tx_drv_post_sel0 = 0x5,
		.tx_drv_post_sel1 = 0xa,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0x4f,
	},
	{
		.aux_rx_idle_th = 0x2,
		.tx_drv_amp_sel0 = 0x13,
		.tx_drv_amp_sel1 = 0x17,
		.tx_drv_post_sel0 = 0x4,
		.tx_drv_post_sel1 = 0xa,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0xf,
	},
	{
		.aux_rx_idle_th = 0x3,
		.tx_drv_amp_sel0 = 0x1f,
		.tx_drv_amp_sel1 = 0x23,
		.tx_drv_post_sel0 = 0xa,
		.tx_drv_post_sel1 = 0xe,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0xcd,
	},
};

struct init_data {
	u8 cfg_addr;
	u16 cfg_wdata;
	bool cfg_wds;
	bool cfg_rds;
	bool cfg_rst;
};

static struct init_data usb3_pll_g1_init_data[] = {
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_5),
	},
};

static void pcie_usb3_pll_defaults(struct tegra_padctl_uphy *ctx)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(usb3_pll_g1_init_data); i++) {
		reg = CFG_ADDR(usb3_pll_g1_init_data[i].cfg_addr);
		reg |= CFG_WDATA(usb3_pll_g1_init_data[i].cfg_wdata);
		uphy_pll_writel(ctx, 0, reg, UPHY_PLL_CTL_4);
	}
}

#define pcie_pll_init pcie_usb3_pll_defaults

static struct init_data sata_pll_g1_g2_g3_init_data[] = {
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID0,
		.cfg_wdata = CFG_FREQ_NDIV(30),
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_10),
	},
};

static void sata_pll_init(struct tegra_padctl_uphy *ctx)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_pll_g1_g2_g3_init_data); i++) {
		reg = CFG_ADDR(sata_pll_g1_g2_g3_init_data[i].cfg_addr);
		reg |= CFG_WDATA(sata_pll_g1_g2_g3_init_data[i].cfg_wdata);
		uphy_pll_writel(ctx, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_pll_g1_g2_g3_A_B_init_data[] = {
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID0,
		.cfg_wdata = CFG_FREQ_NDIV(24) | CFG_FREQ_MDIV(1),
	},
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID1,
		.cfg_wdata = CFG_FREQ_NDIV(28) | CFG_FREQ_MDIV(1),
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_10),
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID1,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_10),
	},
	{
		.cfg_addr = PLLC_CRSWRD_OVRD_ID1,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = MGMT_CYA_CTRL,
		.cfg_wdata = CFG_MGMT_CLK_SEL(1),
	},
};
static void ufs_pll_init(struct tegra_padctl_uphy *ctx)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_pll_g1_g2_g3_A_B_init_data); i++) {
		reg = CFG_ADDR(ufs_pll_g1_g2_g3_A_B_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_pll_g1_g2_g3_A_B_init_data[i].cfg_wdata);
		uphy_pll_writel(ctx, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_pll_rateid_init_data[] = {
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID0,
		.cfg_wdata = CFG_FREQ_NDIV(0x18) | CFG_FREQ_MDIV(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID1,
		.cfg_wdata = CFG_FREQ_NDIV(0x1c) | CFG_FREQ_MDIV(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_REFCLK_CTRL,
		.cfg_wdata = 0x0,
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_XDIGCLK_SEL(0x7) | CFG_XDIGCLK_EN(0x1)
			| CFG_TXCLKREF_SEL(0x2) | CFG_TXCLKREF_EN(1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = CFG_XDIGCLK_SEL(0x7) | CFG_XDIGCLK_EN(0x1)
			| CFG_TXCLKREF_SEL(0x2) | CFG_TXCLKREF_EN(1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = PLLC_CRSWRD_OVRD_ID0,
		.cfg_wdata = 0x3e,
		.cfg_wds = true,
		.cfg_rst = true,
	}
};

static void ufs_pll_rateid_init(struct tegra_padctl_uphy *ctx)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_pll_rateid_init_data); i++) {
		reg = CFG_ADDR(ufs_pll_rateid_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_pll_rateid_init_data[i].cfg_wdata);
		reg |= CFG_WDS(ufs_pll_rateid_init_data[i].cfg_wds);
		reg |= CFG_RDS(ufs_pll_rateid_init_data[i].cfg_rds);
		reg |= CFG_RESET(ufs_pll_rateid_init_data[i].cfg_rst);
		uphy_pll_writel(ctx, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_lane_rateid_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV4),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = TX_RATE_SDIV(SDIV2),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID2,
		.cfg_wdata = TX_RATE_SDIV(SDIV1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID3,
		.cfg_wdata = TX_RATE_SDIV(SDIV1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(CDIV1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID1,
		.cfg_wdata = RX_RATE_SDIV(CDIV2),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID2,
		.cfg_wdata = RX_RATE_SDIV(CDIV4),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(CDIV4),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0x3) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID1,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0x6) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID2,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0xc) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID3,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0xc) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = HF_UBIN(0x0) | LF_UGRAY(0xf),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID1,
		.cfg_wdata = HF_UBIN(0x0) | LF_UGRAY(0xf),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID2,
		.cfg_wdata = HF_UBIN(0x8) | LF_UGRAY(0xf),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID3,
		.cfg_wdata = HF_UBIN(0xf) | LF_UGRAY(0xd),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID0,
		.cfg_wdata = CTLE_HF_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID1,
		.cfg_wdata = CTLE_HF_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID2,
		.cfg_wdata = CTLE_HF_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID3,
		.cfg_wdata = CTLE_HF_MODE(0x0),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x0),
		.cfg_wds = true,
		.cfg_rst = true,
	},
};

static void ufs_lane_rateid_init(struct tegra_padctl_uphy *ctx, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_lane_rateid_init_data); i++) {
		reg = CFG_ADDR(ufs_lane_rateid_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_lane_rateid_init_data[i].cfg_wdata);
		reg |= CFG_WDS(ufs_lane_rateid_init_data[i].cfg_wds);
		reg |= CFG_RDS(ufs_lane_rateid_init_data[i].cfg_rds);
		reg |= CFG_RESET(ufs_lane_rateid_init_data[i].cfg_rst);
		uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}
}


static struct init_data pcie_lane_g1_g2_init_data[] = {
};

static void pcie_lane_defaults(struct tegra_padctl_uphy *ctx, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(pcie_lane_g1_g2_init_data); i++) {
		reg = CFG_ADDR(pcie_lane_g1_g2_init_data[i].cfg_addr);
		reg |= CFG_WDATA(pcie_lane_g1_g2_init_data[i].cfg_wdata);
		uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}
}

static struct init_data usb3_lane_g1_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV1),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(SDIV1) | RX_RATE_CDIV(CDIV4),
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(0),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0xd) | HF_UBIN(0xf),
	},
	{
		.cfg_addr = AE_DFE0_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0xd) | HF_UBIN(0xf),
	},
	{
		.cfg_addr = AE_DFE1_CTRL_ID0,
		.cfg_wdata = H0_SBIN(48) | H4_SBIN(-1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0xc) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_EQ0_CTRL_ID0,
		.cfg_wdata = EQ0_SEQ_MODE(SEQ_MODE_HF_Z_H0_HN_Z) |
			     H0INIT_ITERS(0x3),
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID0,
		.cfg_wdata = CTLE_HF_MODE(VAR_MODE_AUTO) |
			     CTLE_HF_GRAD(CTLE_HF_GRAD_1P5) |
			     H1_MODE(VAR_MODE_AUTO) |
			     H2_MODE(VAR_MODE_AUTO) |
			     H3_MODE(VAR_MODE_AUTO) |
			     H4_MODE(VAR_MODE_AUTO),
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H1_MODE(VAR_MODE_AUTO) | H0_DAC_TIME(0x2) |
			     H0_TIME(0x6) | H0_ITERS(0x3) | HN_ITERS(0x1),
	},
	{
		.cfg_addr = AE_EQ3_CTRL_ID0,
		.cfg_wdata = HN_GAIN(0Xf) | CTLE_HF_TIME(0xc) |
			     CTLE_HF_GAIN(0xf),
	},
};

static void usb3_lane_defaults(struct tegra_padctl_uphy *ctx, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(usb3_lane_g1_init_data); i++) {
		reg = CFG_ADDR(usb3_lane_g1_init_data[i].cfg_addr);
		reg |= CFG_WDATA(usb3_lane_g1_init_data[i].cfg_wdata);
		uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static struct init_data sata_lane_g1_g2_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV4),
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = TX_RATE_SDIV(SDIV2),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(SDIV4) | RX_RATE_CDIV(CDIV1),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID1,
		.cfg_wdata = RX_RATE_SDIV(SDIV2) | RX_RATE_CDIV(CDIV2),
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(0),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0x8),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x3) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID1,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x6) | FRLOOP_EN(1),
	},
};

static void sata_lane_init(struct tegra_padctl_uphy *ctx, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_lane_g1_g2_init_data); i++) {
		reg = CFG_ADDR(sata_lane_g1_g2_init_data[i].cfg_addr);
		reg |= CFG_WDATA(sata_lane_g1_g2_init_data[i].cfg_wdata);
		uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static struct init_data ufs_lane_g1_g2_g3_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV4),
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = TX_RATE_SDIV(SDIV2),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(SDIV4) | RX_RATE_CDIV(CDIV1),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID1,
		.cfg_wdata = RX_RATE_SDIV(SDIV2) | RX_RATE_CDIV(CDIV2),
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(1),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0xf),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID1,
		.cfg_wdata = LF_UGRAY(0xf),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID2,
		.cfg_wdata = LF_UGRAY(0xf) | HF_UBIN(8),
	},

	{
		.cfg_addr = AE_DFE0_CTRL_ID2,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = AE_DFE1_CTRL_ID2,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x3) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID1,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x6) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID2,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0xc) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_EQ0_CTRL_ID2,
		.cfg_wdata = EQ0_SEQ_MODE(SEQ_MODE_HF_Z_H0_HN_Z),
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID2,
		.cfg_wdata = CTLE_HF_MODE(VAR_MODE_OFF) |
			     CTLE_HF_GRAD(CTLE_HF_GRAD_1P5) |
			     H1_MODE(VAR_MODE_OFF) |
			     H2_MODE(VAR_MODE_OFF) |
			     H3_MODE(VAR_MODE_OFF) |
			     H4_MODE(VAR_MODE_OFF),
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID2,
		.cfg_wdata = H1_MODE(VAR_MODE_OFF),
	},
	{
		.cfg_addr = AE_EQ3_CTRL_ID2,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = MGMT_RX_PI_CTRL_ID2,
		.cfg_wdata = 0,
	},
};

static void ufs_lane_init(struct tegra_padctl_uphy *ctx, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_lane_g1_g2_g3_init_data); i++) {
		reg = CFG_ADDR(ufs_lane_g1_g2_g3_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_lane_g1_g2_g3_init_data[i].cfg_wdata);
		uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static int __uphy_pll_init(struct tegra_padctl_uphy *ctx, int pll)
{
	struct device *dev = ctx->dev;
	u32 reg;
	int i;

	/* FIXME: Need to see needed ??? */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
	reg |= PWR_OVRD;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);

	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg |= (CAL_OVRD | RCAL_OVRD | CAL_RESET);
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);

	/* power up PLL */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_IDDQ;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);

	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_SLEEP(~0);
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);

	ndelay(100);

	/* perform PLL calibration */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg |= CAL_EN;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
	for (i = 0; i < 20; i++) {
		reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
		if (reg & CAL_DONE)
			break;
		usleep_range(10, 15);
	}
	if (!(reg & CAL_DONE)) {
		dev_err(dev, "start PLL %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* stop PLL calibration */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg &= ~CAL_EN;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
	for (i = 0; i < 20; i++) {
		reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
		if (!(reg & CAL_DONE))
			break;
		usleep_range(10, 15);
	}
	if (reg & CAL_DONE) {
		dev_err(dev, "stop PLL %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	if (pll == 1) {
		/* perform PLL rate change for UPHY_PLL_1, used by UFS */
		reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
		reg &= ~RATE_ID(~0);
		reg |= (RATE_ID(1) | RATE_ID_OVRD);
		uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);

		ndelay(100);

		/* perform PLL calibration for rate B */
		reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
		reg |= CAL_EN;
		uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
		for (i = 0; i < 20; i++) {
			reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
			if (reg & CAL_DONE)
				break;
			usleep_range(10, 15);
		}
		if (!(reg & CAL_DONE)) {
			dev_err(dev, "start PLL %d calibration timeout\n", pll);
			return -ETIMEDOUT;
		}

		/* stop PLL calibration */
		reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
		reg &= ~CAL_EN;
		uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
		for (i = 0; i < 20; i++) {
			reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
			if (!(reg & CAL_DONE))
				break;
			usleep_range(10, 15);
		}
		if (reg & CAL_DONE) {
			dev_err(dev, "stop PLL %d calibration timeout\n", pll);
			return -ETIMEDOUT;
		}

		reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
		reg &= ~RATE_ID(~0);
		reg |= RATE_ID_OVRD;
		uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);
		ndelay(100);
	}

	/* enable PLL and wait for lock */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
	reg |= PLL_ENABLE;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);
	usleep_range(20, 25);
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
	if (!(reg & LOCKDET_STATUS)) {
		dev_err(dev, "enable PLL %d timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* perform resistor calibration */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg |= RCAL_EN;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg |= RCAL_CLK_EN;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
	usleep_range(5, 10);
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	if (!(reg & RCAL_DONE)) {
		dev_err(dev, "start resistor %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* stop resistor calibration */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg &= ~RCAL_EN;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);
	usleep_range(5, 10);
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	if (reg & RCAL_DONE) {
		dev_err(dev, "stop resistor %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg &= ~RCAL_CLK_EN;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);

	/* remove SW overrides to allow HW sequencer to run */
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_1);
	reg &= ~PWR_OVRD;
	if (pll == 1)
		reg &= ~RATE_ID_OVRD;
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_1);
	reg = uphy_pll_readl(ctx, pll, UPHY_PLL_CTL_2);
	reg &= ~(CAL_OVRD | RCAL_OVRD);
	uphy_pll_writel(ctx, pll, reg, UPHY_PLL_CTL_2);

	return 0;
}

static int uphy_pll_init(struct tegra_padctl_uphy *ctx, int pll)
{
	struct device *dev = ctx->dev;
	char clk_name[] = "uphy_pllX";
	int rc = 0;

	if (pll < 0 || pll > 1)
		return -EINVAL;

	if (!ctx->uphy_pll_clk[pll]) {
		snprintf(clk_name, sizeof(clk_name), "uphy_pll%d", pll);
		ctx->uphy_pll_clk[pll] = clk_get_sys(NULL, clk_name);

		if (IS_ERR(ctx->uphy_pll_clk[pll])) {
			rc = PTR_ERR(ctx->uphy_pll_clk[pll]);
			dev_err(dev, "fail get uphy_pll%d clk %d\n", pll, rc);
			ctx->uphy_pll_clk[pll] = NULL;
			return rc;
		}
	}

	/* FIXME: lock */
	TRACE_DEV(dev, "clients %d", ctx->uphy_pll_clients[pll]);
	if (ctx->uphy_pll_clients[pll]++ > 0)
		goto done;

	tegra_periph_reset_deassert(ctx->uphy_pll_clk[pll]);
	rc = __uphy_pll_init(ctx, pll);

done:
	return rc;
}

static int uphy_pll_deinit(struct tegra_padctl_uphy *ctx, int pll)
{
	struct device *dev = ctx->dev;

	TRACE_DEV(dev, "clients %d", ctx->uphy_pll_clients[pll]);
	ctx->uphy_pll_clients[pll]--;

	TRACE("FIXME: implement!\n"); /* TODO */
	return 0;
}

static int uphy_lane_init(struct tegra_padctl_uphy *ctx, int lane)
{
	struct device *dev = ctx->dev;
	char clk_name[] = "uphy_laneX";
	int rc;

	if (lane < 0 || lane > 5)
		return -EINVAL;

	if (!ctx->uphy_lane_clk[lane]) {
		snprintf(clk_name, sizeof(clk_name), "uphy_lane%d", lane);
		ctx->uphy_lane_clk[lane] = clk_get_sys(NULL, clk_name);

		if (IS_ERR(ctx->uphy_lane_clk[lane])) {
			rc = PTR_ERR(ctx->uphy_lane_clk[lane]);
			dev_err(dev, "fail get uphy_lane%d clk %d\n", lane, rc);
			ctx->uphy_lane_clk[lane] = NULL;
			return rc;
		}
	}

	tegra_periph_reset_deassert(ctx->uphy_lane_clk[lane]);

	return 0;
}

static int uphy_lane_deinit(struct tegra_padctl_uphy *ctx, int lane)
{
	if (lane < 0 || lane > 5)
		return -EINVAL;

	if (ctx->uphy_lane_clk[lane])
		tegra_periph_reset_assert(ctx->uphy_lane_clk[lane]);

	return 0;
}


#define PIN_OTG_0	0
#define PIN_OTG_1	1
#define PIN_OTG_2	2
#define PIN_OTG_3	3
#define PIN_HSIC_0	4
#define PIN_HSIC_1	5
#define PIN_UPHY_0	6
#define PIN_UPHY_1	7
#define PIN_UPHY_2	8
#define PIN_UPHY_3	9
#define PIN_UPHY_4	10
#define PIN_UPHY_5	11
#define PIN_UPHY_6	12
#define PIN_SATA_0	13

static inline bool lane_is_otg(unsigned int lane)
{
	return lane >= PIN_OTG_0 && lane <= PIN_OTG_3;
}

static inline bool lane_is_hsic(unsigned int lane)
{
	return lane >= PIN_HSIC_0 && lane <= PIN_HSIC_1;
}

static inline bool lane_is_uphy(unsigned int lane)
{
	return lane >= PIN_UPHY_0 && lane <= PIN_UPHY_6;
}

static inline bool lane_is_pcie_or_sata(unsigned int lane)
{
	return lane >= PIN_UPHY_0 && lane <= PIN_SATA_0;
}

static int lane_to_usb3_port(struct tegra_padctl_uphy *padctl,
			     unsigned int lane)
{
	unsigned int i;

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		if (padctl->usb3_ports[i].lane == lane)
			return i;
	}

	return -EINVAL;
}

static int tegra_xusb_padctl_get_groups_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);

	TRACE("num_pins %u", padctl->soc->num_pins);
	return padctl->soc->num_pins;
}

static const char *tegra_xusb_padctl_get_group_name(struct pinctrl_dev *pinctrl,
						    unsigned int group)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);

	TRACE("group %u name %s", group, padctl->soc->pins[group].name);
	return padctl->soc->pins[group].name;
}

static int tegra_xusb_padctl_get_group_pins(struct pinctrl_dev *pinctrl,
				 unsigned group,
				 const unsigned **pins,
				 unsigned *num_pins)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);

	*pins = &padctl->soc->pins[group].number;
	*num_pins = 1; /* one pin per group */

	TRACE("group %u num_pins %u pins[0] %u", group, *num_pins, *pins[0]);

	return 0;
}

enum tegra_xusb_padctl_param {
	TEGRA_XUSB_PADCTL_USB3_PORT,
	TEGRA_XUSB_PADCTL_USB2_PORT,
};

static const struct tegra_xusb_padctl_property {
	const char *name;
	enum tegra_xusb_padctl_param param;
} properties[] = {
	{"nvidia,usb3-port", TEGRA_XUSB_PADCTL_USB3_PORT},
	{"nvidia,usb2-port", TEGRA_XUSB_PADCTL_USB2_PORT},
};

#define TEGRA_XUSB_PADCTL_PACK(param, value) ((param) << 16 | (value))
#define TEGRA_XUSB_PADCTL_UNPACK_PARAM(config) ((config) >> 16)
#define TEGRA_XUSB_PADCTL_UNPACK_VALUE(config) ((config) & 0xffff)

static int tegra186_padctl_uphy_parse_subnode(struct tegra_padctl_uphy *padctl,
					   struct device_node *np,
					   struct pinctrl_map **maps,
					   unsigned int *reserved_maps,
					   unsigned int *num_maps)
{
	unsigned int i, reserve = 0, num_configs = 0;
	unsigned long config, *configs = NULL;
	const char *function, *group;
	struct property *prop;
	int err = 0;
	u32 value;

	TRACE();
	err = of_property_read_string(np, "nvidia,function", &function);
	TRACE("err %d function %s", err, function ? function : "(null)");
	if (err < 0) {
		if (err != -EINVAL)
			return err;

		function = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		err = of_property_read_u32(np, properties[i].name, &value);
		TRACE("err %d property %s (%d) value %d",
			err, properties[i].name, i, value);
		if (err < 0) {
			if (err == -EINVAL)
				continue;

			return err;
		}

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, value);

		err = pinctrl_utils_add_config(padctl->pinctrl, &configs,
					       &num_configs, config);
		TRACE("err %d num_configs %d", err, num_configs);
		if (err < 0)
			return err;
	}

	if (function)
		reserve++;

	if (num_configs)
		reserve++;

	err = of_property_count_strings(np, "nvidia,lanes");
	TRACE("of_property_count_strings(\"nvidia,lanes\") %d", err);
	if (err < 0)
		return err;

	reserve *= err;

	err = pinctrl_utils_reserve_map(padctl->pinctrl, maps, reserved_maps,
					num_maps, reserve);
	TRACE("pinctrl_utils_reserve_map reserve %d num_maps %d err %d",
		reserve, *num_maps, err);
	if (err < 0)
		return err;

	of_property_for_each_string(np, "nvidia,lanes", prop, group) {
		if (function) {
			err = pinctrl_utils_add_map_mux(padctl->pinctrl, maps,
					reserved_maps, num_maps, group,
					function);
			TRACE("pinctrl_utils_add_map_mux() err %d num_maps %u group %s function %s",
				err, *num_maps, group, function);
			if (err < 0)
				return err;
		}

		if (num_configs) {
			err = pinctrl_utils_add_map_configs(padctl->pinctrl,
					maps, reserved_maps, num_maps, group,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_GROUP);
			TRACE("pinctrl_utils_add_map_configs() err %d", err);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int tegra_xusb_padctl_dt_node_to_map(struct pinctrl_dev *pinctrl,
					    struct device_node *parent,
					    struct pinctrl_map **maps,
					    unsigned int *num_maps)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);
	unsigned int reserved_maps = 0;
	struct device_node *np;
	int err;

	*num_maps = 0;
	*maps = NULL;

	pr_info("%s %d\n", __func__, __LINE__);
	for_each_child_of_node(parent, np) {
		err = tegra186_padctl_uphy_parse_subnode(padctl, np, maps,
						      &reserved_maps,
						      num_maps);
		if (err < 0) {
			pr_info("%s %d err %d\n", __func__, __LINE__, err);
			return err;
		}
	}

	return 0;
}

static const struct pinctrl_ops tegra_xusb_padctl_pinctrl_ops = {
	.get_groups_count = tegra_xusb_padctl_get_groups_count,
	.get_group_name = tegra_xusb_padctl_get_group_name,
	.get_group_pins = tegra_xusb_padctl_get_group_pins,
	.dt_node_to_map = tegra_xusb_padctl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_dt_free_map,
};

static int tegra186_padctl_uphy_get_functions_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);

	TRACE("num_functions %u", padctl->soc->num_functions);
	return padctl->soc->num_functions;
}

static const char *
tegra186_padctl_uphy_get_function_name(struct pinctrl_dev *pinctrl,
				    unsigned int function)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);

	TRACE("function %u name %s", function, padctl->soc->functions[function].name);
	return padctl->soc->functions[function].name;
}

static int tegra186_padctl_uphy_get_function_groups(struct pinctrl_dev *pinctrl,
						 unsigned int function,
						 const char * const **groups,
						 unsigned * const num_groups)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);

	*num_groups = padctl->soc->functions[function].num_groups;
	*groups = padctl->soc->functions[function].groups;

	TRACE("function %u *num_groups %u groups %s",
				function, *num_groups, *groups[0]);
	return 0;
}

static int tegra186_padctl_uphy_pinmux_enable(struct pinctrl_dev *pinctrl,
					   unsigned int function,
					   unsigned int group)
{
	struct tegra_padctl_uphy *ctx = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_padctl_uphy_lane *lane;
	unsigned int i;
	u32 value;

	lane = &ctx->soc->lanes[group];

	TRACE_DEV(pinctrl->dev, "group %u (%s) function %u num_funcs %d",
			group, lane->name, function, lane->num_funcs);

	for (i = 0; i < lane->num_funcs; i++) {
		TRACE_DEV(pinctrl->dev, "funcs[%d] %d", i, lane->funcs[i]);
		if (lane->funcs[i] == function)
			break;
	}

	if (i >= lane->num_funcs)
		return -EINVAL;

	TRACE_DEV(pinctrl->dev, "group %s set to function %d", lane->name, i);

	if (lane_is_otg(group) || lane_is_hsic(group)) {
		value = padctl_readl(ctx, lane->offset);
		value &= ~(lane->mask << lane->shift);
		value |= i << lane->shift;
		padctl_writel(ctx, value, lane->offset);
	} else if (lane_is_uphy(group)) {
		int uphy_lane = group - PIN_UPHY_0;
		if (function == TEGRA186_FUNC_USB3)
			set_bit(uphy_lane, &ctx->usb3_lanes);
		else if (function == TEGRA186_FUNC_PCIE)
			set_bit(uphy_lane, &ctx->pcie_lanes);
		else if (function == TEGRA186_FUNC_SATA)
			set_bit(uphy_lane, &ctx->sata_lanes);
		else if (function == TEGRA186_FUNC_MPHY)
			set_bit(uphy_lane, &ctx->ufs_lanes);
#if 0
		value = uphy_lane_readl(ctx, uphy_lane, lane->offset);
		value &= ~(lane->mask << lane->shift);
		value |= i << lane->shift;
		uphy_lane_writel(ctx, uphy_lane, value, lane->offset);
#endif
	} else
		return -EINVAL;

	return 0;
}

static const struct pinmux_ops tegra186_padctl_uphy_pinmux_ops = {
	.get_functions_count = tegra186_padctl_uphy_get_functions_count,
	.get_function_name = tegra186_padctl_uphy_get_function_name,
	.get_function_groups = tegra186_padctl_uphy_get_function_groups,
	.enable = tegra186_padctl_uphy_pinmux_enable,
};

static int tegra_xusb_padctl_pinconf_group_get(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *config)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_padctl_uphy_lane *lane;
	enum tegra_xusb_padctl_param param;
	u32 value = 0;
	int port;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(*config);
	lane = &padctl->soc->lanes[group];

	TRACE("group %u param 0x%x\n", group, param);

	switch (param) {
	case TEGRA_XUSB_PADCTL_USB3_PORT:
		value = lane_to_usb3_port(padctl, group);
		if (value < 0) {
			dev_err(padctl->dev,
				"Pin %d not mapped to USB3 port\n", group);
			return -EINVAL;
		}
		break;

	case TEGRA_XUSB_PADCTL_USB2_PORT:
		port = lane_to_usb3_port(padctl, group);
		if (port < 0) {
			dev_err(padctl->dev,
				"Pin %d not mapped to USB3 port\n", group);
			return -EINVAL;
		}

		break;

	default:
		dev_err(padctl->dev, "invalid configuration parameter: %04x\n",
			param);
		return -ENOTSUPP;
	}

	*config = TEGRA_XUSB_PADCTL_PACK(param, value);
	return 0;
}

#if 0
static int tegra_xusb_padctl_pinconf_group_set(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *configs,
					       unsigned int num_configs)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_padctl_uphy_lane *lane;
	enum tegra_xusb_padctl_param param;
	unsigned long value;
	unsigned int i;
	int port;

	lane = &padctl->soc->lanes[group];

	for (i = 0; i < num_configs; i++) {
		param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(configs[i]);
		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(configs[i]);

		TRACE("group %u config %d param 0x%x value 0x%lx",
			group, i, param, value);

		switch (param) {
		case TEGRA_XUSB_PADCTL_USB3_PORT:
			if (value >= TEGRA_XUSB_USB3_PHYS) {
				dev_err(padctl->dev, "Invalid USB3 port: %lu\n",
					value);
				return -EINVAL;
			}
			if (!lane_is_uphy(group)) {
				dev_err(padctl->dev,
					"USB3 port not applicable for pin %d\n",
					group);
				return -EINVAL;
			}

			padctl->usb3_ports[value].lane = group;
			break;

		case TEGRA_XUSB_PADCTL_USB2_PORT:
			if (value >= TEGRA_XUSB_UTMI_PHYS) {
				dev_err(padctl->dev, "Invalid USB2 port: %lu\n",
					value);
				return -EINVAL;
			}
			if (!lane_is_pcie_or_sata(group)) {
				dev_err(padctl->dev,
					"USB2 port not applicable for pin %d\n",
					group);
				return -EINVAL;
			}
			port = lane_to_usb3_port(padctl, group);
			if (port < 0) {
				dev_err(padctl->dev,
					"Pin %d not mapped to USB3 port\n",
					group);
				return -EINVAL;
			}

			break;

		default:
			dev_err(padctl->dev,
				"invalid configuration parameter: %04x\n",
				param);
			return -ENOTSUPP;
		}
	}

	return 0;
}
#else
static int tegra_xusb_padctl_pinconf_group_set(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long config)
{
	struct tegra_padctl_uphy *padctl = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_padctl_uphy_lane *lane;
	enum tegra_xusb_padctl_param param;
	unsigned long value;
	int port;

	lane = &padctl->soc->lanes[group];

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(config);
	value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

	TRACE("group %u config 0x%lx param 0x%x value 0x%lx",
		group, config, param, value);

	switch (param) {
	case TEGRA_XUSB_PADCTL_USB3_PORT:
		if (value >= TEGRA_XUSB_USB3_PHYS) {
			dev_err(padctl->dev, "Invalid USB3 port: %lu\n",
				value);
			return -EINVAL;
		}
		if (!lane_is_uphy(group)) {
			dev_err(padctl->dev,
				"USB3 port not applicable for pin %d\n",
				group);
			return -EINVAL;
		}

		padctl->usb3_ports[value].lane = group;
		break;

	case TEGRA_XUSB_PADCTL_USB2_PORT:
		if (value >= TEGRA_XUSB_UTMI_PHYS) {
			dev_err(padctl->dev, "Invalid USB2 port: %lu\n",
				value);
			return -EINVAL;
		}
		if (!lane_is_pcie_or_sata(group)) {
			dev_err(padctl->dev,
				"USB2 port not applicable for pin %d\n",
				group);
			return -EINVAL;
		}
		port = lane_to_usb3_port(padctl, group);
		if (port < 0) {
			dev_err(padctl->dev,
				"Pin %d not mapped to USB3 port\n",
				group);
			return -EINVAL;
		}

		break;

	default:
		dev_err(padctl->dev,
			"invalid configuration parameter: %04x\n",
			param);
		return -ENOTSUPP;
	}

	return 0;
}
#endif

#ifdef CONFIG_DEBUG_FS
static const char *strip_prefix(const char *s)
{
	const char *comma = strchr(s, ',');
	if (!comma)
		return s;

	return comma + 1;
}

static void
tegra_xusb_padctl_pinconf_group_dbg_show(struct pinctrl_dev *pinctrl,
					 struct seq_file *s,
					 unsigned int group)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		unsigned long config, value;
		int err;

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, 0);

		err = tegra_xusb_padctl_pinconf_group_get(pinctrl, group,
							  &config);
		if (err < 0)
			continue;

		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

		seq_printf(s, "\n\t%s=%lu\n", strip_prefix(properties[i].name),
			   value);
	}
}

static void
tegra_xusb_padctl_pinconf_config_dbg_show(struct pinctrl_dev *pinctrl,
					  struct seq_file *s,
					  unsigned long config)
{
	enum tegra_xusb_padctl_param param;
	const char *name = "unknown";
	unsigned long value;
	unsigned int i;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(config);
	value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		if (properties[i].param == param) {
			name = properties[i].name;
			break;
		}
	}

	seq_printf(s, "%s=%lu", strip_prefix(name), value);
}
#endif

static const struct pinconf_ops tegra_xusb_padctl_pinconf_ops = {
	.pin_config_group_get = tegra_xusb_padctl_pinconf_group_get,
	.pin_config_group_set = tegra_xusb_padctl_pinconf_group_set,
#ifdef CONFIG_DEBUG_FS
	.pin_config_group_dbg_show = tegra_xusb_padctl_pinconf_group_dbg_show,
	.pin_config_config_dbg_show = tegra_xusb_padctl_pinconf_config_dbg_show,
#endif
};

static int tegra_xusb_padctl_enable(struct tegra_padctl_uphy *ctx)
{

	mutex_lock(&ctx->lock);
	TRACE_DEV(ctx->dev, "padctl_clients %d", ctx->padctl_clients);

	if (ctx->padctl_clients++ > 0)
		goto out;

out:
	mutex_unlock(&ctx->lock);
	return 0;
}

static int tegra_xusb_padctl_disable(struct tegra_padctl_uphy *ctx)
{

	mutex_lock(&ctx->lock);

	TRACE_DEV(ctx->dev, "padctl_clients %d", ctx->padctl_clients);

	if (WARN_ON(ctx->padctl_clients == 0))
		goto out;

	if (--ctx->padctl_clients > 0)
		goto out;

out:
	mutex_unlock(&ctx->lock);
	return 0;
}

static int tegra_xusb_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);

	return tegra_xusb_padctl_enable(padctl);
}

static int tegra_xusb_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);

	return tegra_xusb_padctl_disable(padctl);
}

static int tegra186_pcie_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	for_each_set_bit(uphy_lane, &ctx->pcie_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);

		reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
		reg &= SEL(~0);
		reg |= SEL_PCIE;
		reg |= FORCE_IDDQ_DISABLE;
		reg &= ~CLAMP_EN_EARLY;
		uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);
	}

	TRACE_DEV(&phy->dev, "FIXME: implement!"); /* TODO: more? */
	return 0;
}

static int tegra186_pcie_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	for_each_set_bit(uphy_lane, &ctx->pcie_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);

		reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
		reg &= ~FORCE_IDDQ_DISABLE;
		reg |= CLAMP_EN_EARLY;
		uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);
	}

	TRACE_DEV(&phy->dev, "FIXME: implement!"); /* TODO */
	return 0;
}

static int tegra186_pcie_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned uphy_lane;

	for_each_set_bit(uphy_lane, &ctx->pcie_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);

		uphy_pll_init(ctx, 0);
		uphy_lane_init(ctx, uphy_lane);
		pcie_lane_defaults(ctx, uphy_lane);
		pcie_usb3_pll_defaults(ctx);
	}

	return tegra_xusb_phy_init(phy);
}

static int tegra186_pcie_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned uphy_lane;

	for_each_set_bit(uphy_lane, &ctx->pcie_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);

		uphy_lane_deinit(ctx, uphy_lane);
		uphy_pll_deinit(ctx, 0);
	}

	return tegra_xusb_phy_exit(phy);
}

static const struct phy_ops pcie_phy_ops = {
	.init = tegra186_pcie_phy_init,
	.exit = tegra186_pcie_phy_exit,
	.power_on = tegra186_pcie_phy_power_on,
	.power_off = tegra186_pcie_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra186_sata_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	for_each_set_bit(uphy_lane, &ctx->sata_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);

		reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
		reg &= SEL(~0);
		reg |= SEL_SATA;
		reg |= FORCE_IDDQ_DISABLE;
		reg &= ~CLAMP_EN_EARLY;
		uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);
	}

	TRACE_DEV(&phy->dev, "FIXME: implement!"); /* TODO: more? */
	return 0;
}

static int tegra186_sata_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	for_each_set_bit(uphy_lane, &ctx->sata_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);

		reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
		reg &= ~FORCE_IDDQ_DISABLE;
		reg |= CLAMP_EN_EARLY;
		uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);
	}

	TRACE_DEV(&phy->dev, "FIXME: implement!"); /* TODO */
	return 0;
}

static int tegra186_sata_fuse_calibration(struct tegra_padctl_uphy *ctx,
						int lane)
{
	u32 reg;
	int idx;

	/* Update based on fuse_sata_nv_calib[1:0] value */
	idx = SATA_NV_CALIB_0_1(ctx->fuse_calib.sata_nv);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_AUX_CTL_1);
	reg |= AUX_RX_IDLE_TH(sata_data[idx].aux_rx_idle_th);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_AUX_CTL_1);

	/* Update based on fuse_sata_nv_calib[3:2] value TBD */
	idx = SATA_NV_CALIB_2_3(ctx->fuse_calib.sata_nv);

	/* Update based on fuse_sata_mphy_odm_calib[1:0] value */
	idx = SATA_MPHY_ODM_CALIB_0_1(ctx->fuse_calib.sata_mphy_odm);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_1);
	reg |= TX_DRV_AMP_SEL0(sata_data[idx].tx_drv_amp_sel0);
	reg |= TX_DRV_AMP_SEL1(sata_data[idx].tx_drv_amp_sel1);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_1);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_4);
	reg |= TX_DRV_POST_SEL0(sata_data[idx].tx_drv_post_sel0);
	reg |= TX_DRV_POST_SEL1(sata_data[idx].tx_drv_post_sel1);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_4);

	reg = CFG_ADDR(AE_CTLE_CTRL_ID0);
	reg |= CFG_WDATA(sata_data[idx].ae_ctle_ctrl_id0);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);

	reg = CFG_ADDR(AE_CTLE_CTRL_ID1);
	reg |= CFG_WDATA(sata_data[idx].ae_ctle_ctrl_id1);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DIRECT_CTL_2);

	return 0;
}

static int tegra186_sata_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned uphy_lane;

	sata_pll_init(ctx);

	for_each_set_bit(uphy_lane, &ctx->sata_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "uphy_lane %u\n", uphy_lane);
		sata_lane_init(ctx, uphy_lane);
	}

	tegra186_sata_fuse_calibration(ctx, ctx->sata_lanes);

	return tegra_xusb_phy_init(phy);
}

static int tegra186_sata_phy_exit(struct phy *phy)
{
	TRACE_DEV(&phy->dev, "");
	return tegra_xusb_phy_exit(phy);
}

static const struct phy_ops sata_phy_ops = {
	.init = tegra186_sata_phy_init,
	.exit = tegra186_sata_phy_exit,
	.power_on = tegra186_sata_phy_power_on,
	.power_off = tegra186_sata_phy_power_off,
	.owner = THIS_MODULE,
};


static int tegra186_ufs_phy_power_on(struct phy *phy)
{

	return 0;
}

static int tegra186_ufs_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	TRACE();
	for_each_set_bit(uphy_lane, &ctx->ufs_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "lane %d", uphy_lane);
		reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
		reg &= ~FORCE_IDDQ_DISABLE;
		reg |= CLAMP_EN_EARLY;
		uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);
	}

	return 0;
}

static void tegra186_uphy_lane_pad_macro(bool on)
{
	void __iomem *car_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	u32 reg;

	reg = ioread32(car_base + CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
	if (on)
		reg |= SWR_UPHY_RST;
	else
		reg &= ~SWR_UPHY_RST;
	iowrite32(reg, car_base + CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
}

static void tegra186_uphy_lane_and_pll_reset(int function, unsigned long lanes)
{
	void __iomem *car_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
	unsigned uphy_lane;
	u32 reg;

	reg = ioread32(car_base + CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0);

	/* De-assert global reset */
	reg |= SWR_PEX_USB_UPHY_RST;

	/* De-assert corresponding pll reset */
	if (function == TEGRA186_FUNC_MPHY)
		reg &= ~SWR_PEX_USB_UPHY_PLL1_RST;
	else
		reg &= ~SWR_PEX_USB_UPHY_PLL0_RST;

	/* De-assert corresponding lane reset */
	for_each_set_bit(uphy_lane, &lanes, T186_UPHY_LANES) {
		reg &= ~SWR_PEX_USB_UPHY_LANE_RST(uphy_lane);
	}

	iowrite32(reg, car_base + CLK_RST_CONTROLLER_RST_DEV_PEX_USB_UPHY_0);
}

static int tegra186_ufs_fuse_calibration(struct tegra_padctl_uphy *ctx,
						int lane)
{
	u32 reg;
	int idx;

	/* Update based on fuse_mphy_nv_calib[1:0] value */
	idx = MPHY_NV_CALIB_0_1(ctx->fuse_calib.mphy_nv);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_AUX_CTL_1);
	reg |= AUX_RX_IDLE_TH(mphy_data[idx].aux_rx_idle_th);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_AUX_CTL_1);

	/* Update based on fuse_mphy_nv_calib[3:2] value TBD */
	idx = MPHY_NV_CALIB_2_3(ctx->fuse_calib.mphy_nv);


	/* Update based on fuse_mphy_nv_calib[5:4] value TBD */
	idx = MPHY_NV_CALIB_4_5(ctx->fuse_calib.mphy_nv);


	/* Update based on fuse_sata_mphy_odm_calib[1:0] value */
	idx = SATA_MPHY_ODM_CALIB_0_1(ctx->fuse_calib.sata_mphy_odm);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_1);
	reg |= TX_DRV_AMP_SEL0(mphy_data[idx].tx_drv_amp_sel0);
	reg |= TX_DRV_AMP_SEL1(mphy_data[idx].tx_drv_amp_sel1);
	reg |= TX_DRV_AMP_SEL2(mphy_data[idx].tx_drv_amp_sel2);
	reg |= TX_DRV_AMP_SEL3(mphy_data[idx].tx_drv_amp_sel3);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_1);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_2);
	reg |= TX_DRV_AMP_SEL4(mphy_data[idx].tx_drv_amp_sel4);
	reg |= TX_DRV_AMP_SEL5(mphy_data[idx].tx_drv_amp_sel5);
	reg |= TX_DRV_AMP_SEL6(mphy_data[idx].tx_drv_amp_sel6);
	reg |= TX_DRV_AMP_SEL7(mphy_data[idx].tx_drv_amp_sel7);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_2);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_3);
	reg |= TX_DRV_AMP_SEL8(mphy_data[idx].tx_drv_amp_sel8);
	reg |= TX_DRV_AMP_SEL9(mphy_data[idx].tx_drv_amp_sel9);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_3);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_4);
	reg |= TX_DRV_POST_SEL0(mphy_data[idx].tx_drv_post_sel0);
	reg |= TX_DRV_POST_SEL1(mphy_data[idx].tx_drv_post_sel1);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_4);

	reg = uphy_lane_readl(ctx, lane, UPHY_LANE_DYN_CTL_5);
	reg |= TX_DRV_POST_SEL2(mphy_data[idx].tx_drv_post_sel2);
	reg |= TX_DRV_POST_SEL3(mphy_data[idx].tx_drv_post_sel3);
	reg |= TX_DRV_PRE_SEL3(mphy_data[idx].tx_drv_pre_sel3);
	uphy_lane_writel(ctx, lane, reg, UPHY_LANE_DYN_CTL_5);

	return 0;
}

static int tegra186_ufs_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned uphy_lane;
	u32 reg;

	TRACE();
	/* FIXME: step 1: Enable refplle to 208M and pllp 102M */

	/* step 2: De-assert UPHY LANE PAD Macro */
	/* FIXME: Should be part of probe ? */
	tegra186_uphy_lane_pad_macro(false);

	/* Bring the lanes out of IDDQ, remove clamps, select MPHY for mux */
	for_each_set_bit(uphy_lane, &ctx->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
		reg &= SEL(~0);
		reg |= SEL_MPHY;
		reg |= FORCE_IDDQ_DISABLE;
		reg &= ~CLAMP_EN_EARLY;
		uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* FIXME: bring refPLLE to under HW control. Needed ? */


	/* step 3: Enable PLL1 and lane by releasing resets */
	tegra186_uphy_lane_and_pll_reset(TEGRA186_FUNC_MPHY, ctx->ufs_lanes);

	/* pll parameters init */
	ufs_pll_init(ctx);

	/* lane parameters init */
	for_each_set_bit(uphy_lane, &ctx->ufs_lanes, T186_UPHY_LANES) {
		TRACE_DEV(&phy->dev, "uphy_lane %u\n", uphy_lane);
		ufs_lane_init(ctx, uphy_lane);
	}

	/* step 4: electrical parameters programming based on fuses */
	for_each_set_bit(uphy_lane, &ctx->ufs_lanes, T186_UPHY_LANES)
		tegra186_ufs_fuse_calibration(ctx, uphy_lane);

	/* step 5: rate id programming */
	ufs_pll_rateid_init(ctx);
	ufs_lane_rateid_init(ctx, ctx->ufs_lanes);

	/* step 6: uphy pll1 calibration */
	uphy_pll_init(ctx, 0);

	/* FIXME: Need to add MPHY programming in above step ? */

	return tegra_xusb_phy_init(phy);
}

static int tegra186_ufs_phy_exit(struct phy *phy)
{
	TRACE_DEV(&phy->dev, "");
	return tegra_xusb_phy_exit(phy);
}

static const struct phy_ops ufs_phy_ops = {
	.init = tegra186_ufs_phy_init,
	.exit = tegra186_ufs_phy_exit,
	.power_on = tegra186_ufs_phy_power_on,
	.power_off = tegra186_ufs_phy_power_off,
	.owner = THIS_MODULE,
};

static int usb3_phy_to_port(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		if (phy == ctx->usb3_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_usb3_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	unsigned int uphy_lane;
	unsigned int lane;
	u32 reg;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

	lane = ctx->usb3_ports[port].lane;
	if (!lane_is_uphy(lane)) {
		dev_err(ctx->dev, "USB3 PHY %d mapped to invalid lane: %d\n",
			port, lane);
		return -EINVAL;
	}

	reg = padctl_readl(ctx, XUSB_PADCTL_SS_PORT_CAP);
	reg &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(port));
	reg |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(port));
	padctl_writel(ctx, reg, XUSB_PADCTL_SS_PORT_CAP);

	uphy_lane = lane - PIN_UPHY_0;
	reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
	reg &= SEL(~0);
	reg |= SEL_XUSB;
	reg |= FORCE_IDDQ_DISABLE;
	reg &= ~CLAMP_EN_EARLY;
	uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);

	reg = padctl_readl(ctx, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(ctx, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(ctx, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(ctx, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	reg = padctl_readl(ctx, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(ctx, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	return 0;
}

static int tegra186_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	unsigned int uphy_lane;
	unsigned int lane;
	u32 reg;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

	lane = ctx->usb3_ports[port].lane;
	if (!lane_is_uphy(lane)) {
		dev_err(ctx->dev, "USB3 PHY %d mapped to invalid lane: %d\n",
			port, lane);
		return -EINVAL;
	}

	uphy_lane = lane - PIN_UPHY_0;
	reg = uphy_lane_readl(ctx, uphy_lane, UPHY_LANE_MUX);
	reg &= ~FORCE_IDDQ_DISABLE;
	reg |= CLAMP_EN_EARLY;
	uphy_lane_writel(ctx, uphy_lane, reg, UPHY_LANE_MUX);

	reg = padctl_readl(ctx, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(ctx, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(ctx, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(ctx, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(250, 350);

	reg = padctl_readl(ctx, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(ctx, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	return 0;
}

static int tegra186_usb3_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	unsigned uphy_lane;
	unsigned int lane;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

	lane = ctx->usb3_ports[port].lane;
	if (!lane_is_uphy(lane)) {
		dev_err(ctx->dev, "USB3 PHY %d mapped to invalid lane: %d\n",
			port, lane);
		return -EINVAL;
	}

	uphy_pll_init(ctx, 0);

	uphy_lane = lane - PIN_UPHY_0;
	uphy_lane_init(ctx, uphy_lane);
	usb3_lane_defaults(ctx, uphy_lane);
	pcie_usb3_pll_defaults(ctx);

	return tegra_xusb_phy_init(phy);
}

static int tegra186_usb3_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *ctx = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	unsigned uphy_lane;
	unsigned int lane;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

	lane = ctx->usb3_ports[port].lane;
	if (!lane_is_uphy(lane)) {
		dev_err(ctx->dev, "USB3 PHY %d mapped to invalid lane: %d\n",
			port, lane);
		return -EINVAL;
	}

	uphy_lane = lane - PIN_UPHY_0;
	uphy_lane_deinit(ctx, lane);
	uphy_pll_deinit(ctx, 0);

	return tegra_xusb_phy_exit(phy);
}

static const struct phy_ops usb3_phy_ops = {
	.init = tegra186_usb3_phy_init,
	.exit = tegra186_usb3_phy_exit,
	.power_on = tegra186_usb3_phy_power_on,
	.power_off = tegra186_usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static int utmi_phy_to_port(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_XUSB_UTMI_PHYS; i++) {
		if (phy == padctl->utmi_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	int err;
	u32 value;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

#if 0
	value = padctl_readl(padctl, XUSB_PADCTL_USB2_PORT_CAP);
	value &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(port));
#if 1
	if (port == 0) /* TODO: support OTG */
		value |= (PORT_CAP_DEVICE << PORTX_CAP_SHIFT(port));
	else
#endif
		value |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(port));
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_PORT_CAP);
#endif

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	value &= ~(USB2_OTG_PD | USB2_OTG_PD2 | USB2_OTG_PD_ZI);
	value &= ~HS_CURR_LEVEL(~0);
	value |= HS_CURR_LEVEL(padctl->calib.hs_curr_level[port]);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	value &= ~USB2_OTG_PD_DR;
	value &= ~TERM_RANGE_ADJ(~0);
	value |= TERM_RANGE_ADJ(padctl->calib.hs_term_range_adj);
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));

	err = regulator_enable(padctl->vbus[port]);
	if (err)
		return err;

	mutex_lock(&padctl->lock);

	if (padctl->utmi_enable++ > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value &= ~BIAS_PAD_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra186_utmi_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	u32 value;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

	regulator_disable(padctl->vbus[port]);

	mutex_lock(&padctl->lock);

	if (WARN_ON(padctl->utmi_enable == 0))
		goto out;

	if (--padctl->utmi_enable > 0)
		goto out;

	value = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	value |= BIAS_PAD_PD;
	padctl_writel(padctl, value, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

out:
	mutex_unlock(&padctl->lock);
	return 0;
}

static int tegra186_utmi_phy_init(struct phy *phy)
{
	TRACE_DEV(&phy->dev, "");
	return tegra_xusb_phy_init(phy);
}

static int tegra186_utmi_phy_exit(struct phy *phy)
{
	TRACE_DEV(&phy->dev, "");
	return tegra_xusb_phy_exit(phy);
}

static const struct phy_ops utmi_phy_ops = {
	.init = tegra186_utmi_phy_init,
	.exit = tegra186_utmi_phy_exit,
	.power_on = tegra186_utmi_phy_power_on,
	.power_off = tegra186_utmi_phy_power_off,
	.owner = THIS_MODULE,
};

#if 0
static int utmi_phy_set_role(struct phy *phy, bool host)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	int pad = utmi_phy_to_port(phy);
	u32 reg;

	dev_info(&phy->dev, "%s %d port %d\n", __func__, __LINE__, pad);
	if (pad < 0)
		return pad;

	reg = padctl_readl(padctl, USB2_BATTERY_CHRG_OTGPADX_CTL1(pad));
	reg &= ~(VREG_FIX18 | VREG_LEV(~0));
	if (host)
		reg |= VREG_FIX18;
	else
		reg |= VREG_LEV(3);
	padctl_writel(padctl, reg, USB2_BATTERY_CHRG_OTGPADX_CTL1(pad));

	reg = padctl_readl(padctl, USB2_VBUS_ID);
	if (host)
		reg &= ~VBUS_OVERRIDE;
	else
		reg |= VBUS_OVERRIDE;
	padctl_writel(padctl, reg, USB2_VBUS_ID);

	return 0;
}

static int tegra_phy_otg_set_host(struct phy *phy)
{
	return utmi_phy_set_role(phy, true);
}

static int tegra_phy_otg_set_device(struct phy *phy)
{
	return utmi_phy_set_role(phy, false);
}
#endif
static int hsic_phy_to_port(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_XUSB_HSIC_PHYS; i++) {
		if (phy == padctl->hsic_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_hsic_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);
	int err;
	u32 value;

	TRACE_DEV(&phy->dev, "port %d", port);
	if (port < 0)
		return port;

	err = regulator_enable(padctl->vddio_hsic);
	if (err)
		return err;
#if 0
	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value &= ~(XUSB_PADCTL_HSIC_PAD_CTL1_RPD_STROBE |
		   XUSB_PADCTL_HSIC_PAD_CTL1_RPU_DATA |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX |
		   XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX);
	value |= XUSB_PADCTL_HSIC_PAD_CTL1_RPD_DATA |
		 XUSB_PADCTL_HSIC_PAD_CTL1_RPU_STROBE;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));
#else
	value = 0;
	TRACE_DEV(&phy->dev, "FIXME: implement!"); /* TODO */
#endif
	return 0;
}

static int tegra186_hsic_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *padctl = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);
	u32 value;

	TRACE_DEV(&phy->dev, "port %d\n", port);
	if (port < 0)
		return port;
#if 0
	value = padctl_readl(padctl, XUSB_PADCTL_HSIC_PADX_CTL1(port));
	value |= XUSB_PADCTL_HSIC_PAD_CTL1_PD_RX |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_ZI |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_TRX |
		 XUSB_PADCTL_HSIC_PAD_CTL1_PD_TX;
	padctl_writel(padctl, value, XUSB_PADCTL_HSIC_PADX_CTL1(port));
#else
	value = 0;
	TRACE_DEV(&phy->dev, "FIXME: implement!"); /* TODO */
#endif
	regulator_disable(padctl->vddio_hsic);

	return 0;
}

static int tegra186_hsic_phy_init(struct phy *phy)
{
	TRACE_DEV(&phy->dev, "");
	return tegra_xusb_phy_init(phy);
}

static int tegra186_hsic_phy_exit(struct phy *phy)
{
	TRACE_DEV(&phy->dev, "");
	return tegra_xusb_phy_exit(phy);
}

static const struct phy_ops hsic_phy_ops = {
	.init = tegra186_hsic_phy_init,
	.exit = tegra186_hsic_phy_exit,
	.power_on = tegra186_hsic_phy_power_on,
	.power_off = tegra186_hsic_phy_power_off,
	.owner = THIS_MODULE,
};

static struct phy *tegra186_padctl_uphy_xlate(struct device *dev,
					   struct of_phandle_args *args)
{
	struct tegra_padctl_uphy *padctl = dev_get_drvdata(dev);
	unsigned int index = args->args[0];
	unsigned int phy_index;
	struct phy *phy = NULL;

	if (args->args_count <= 0)
		return ERR_PTR(-EINVAL);

	TRACE_DEV(dev, "index %d", index);

	if ((index >= TEGRA_USB3_PADCTL_USB3_BASE) &&
		(index < TEGRA_USB3_PADCTL_USB3_BASE + 16)) {

		phy_index = index - TEGRA_USB3_PADCTL_USB3_BASE;
		if (phy_index < TEGRA_XUSB_USB3_PHYS)
			phy = padctl->usb3_phys[phy_index];

	} else if ((index >= TEGRA_USB3_PADCTL_UTMI_BASE) &&
		(index < TEGRA_USB3_PADCTL_UTMI_BASE + 16)) {

		phy_index = index - TEGRA_USB3_PADCTL_UTMI_BASE;
		if (phy_index < TEGRA_XUSB_UTMI_PHYS)
			phy = padctl->utmi_phys[phy_index];

	} else if ((index >= TEGRA_USB3_PADCTL_HSIC_BASE) &&
		(index < TEGRA_USB3_PADCTL_HSIC_BASE + 16)) {

		phy_index = index - TEGRA_USB3_PADCTL_HSIC_BASE;
		if (phy_index < TEGRA_XUSB_HSIC_PHYS)
			phy = padctl->hsic_phys[phy_index];

	} else if ((index >= TEGRA_USB3_PADCTL_PCIE_BASE) &&
		(index < TEGRA_USB3_PADCTL_PCIE_BASE + 16)) {

		phy_index = index - TEGRA_USB3_PADCTL_PCIE_BASE;
		if (phy_index < TEGRA_XUSB_PCIE_PHYS)
			phy = padctl->pcie_phys[phy_index];

	} else if ((index >= TEGRA_USB3_PADCTL_SATA_BASE) &&
		(index < TEGRA_USB3_PADCTL_SATA_BASE + 16)) {

		phy_index = index - TEGRA_USB3_PADCTL_SATA_BASE;
		if (phy_index < TEGRA_XUSB_SATA_PHYS)
			phy = padctl->sata_phys[phy_index];

	} else if ((index >= TEGRA_USB3_PADCTL_UFS_BASE) &&
		(index < TEGRA_USB3_PADCTL_UFS_BASE + 16)) {

		phy_index = index - TEGRA_USB3_PADCTL_UFS_BASE;
		if (phy_index < TEGRA_XUSB_UFS_PHYS)
			phy = padctl->ufs_phys[phy_index];

	}

	return (phy) ? phy : ERR_PTR(-EINVAL);
}

static const struct pinctrl_pin_desc tegra186_pins[] = {
	PINCTRL_PIN(PIN_OTG_0,  "otg-0"),
	PINCTRL_PIN(PIN_OTG_1,  "otg-1"),
	PINCTRL_PIN(PIN_OTG_2,  "otg-2"),
	PINCTRL_PIN(PIN_OTG_3,  "otg-3"),
	PINCTRL_PIN(PIN_HSIC_0, "hsic-0"),
	PINCTRL_PIN(PIN_HSIC_1, "hsic-1"),
	PINCTRL_PIN(PIN_UPHY_0, "uphy-lane-0"),
	PINCTRL_PIN(PIN_UPHY_1, "uphy-lane-1"),
	PINCTRL_PIN(PIN_UPHY_2, "uphy-lane-2"),
	PINCTRL_PIN(PIN_UPHY_3, "uphy-lane-3"),
	PINCTRL_PIN(PIN_UPHY_4, "uphy-lane-4"),
	PINCTRL_PIN(PIN_UPHY_5, "uphy-lane-5"),
};


static const char * const tegra186_hsic_groups[] = {
	"hsic-0",
	"hsic-1",
};

static const char * const tegra186_hsic_plus_groups[] = {
	"hsic-0",
	"hsic-1",
};

static const char * const tegra186_xusb_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"otg-3",
};

static const char * const tegra186_uart_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
	"otg-3"
};

static const char * const tegra186_pcie_groups[] = {
	"uphy-lane-0",
	"uphy-lane-1",
	"uphy-lane-2",
	"uphy-lane-3",
	"uphy-lane-4",
	"uphy-lane-5",
};

static const char * const tegra186_usb3_groups[] = {
	"uphy-lane-0",
	"uphy-lane-1",
	"uphy-lane-2",
	"uphy-lane-3",
	"uphy-lane-4",
	"uphy-lane-5",
};

static const char * const tegra186_sata_groups[] = {
	"uphy-lane-0",
	"uphy-lane-1",
	"uphy-lane-2",
	"uphy-lane-3",
	"uphy-lane-4",
	"uphy-lane-5",
};

static const char * const tegra186_mphy_groups[] = {
	"uphy-lane-0",
	"uphy-lane-1",
	"uphy-lane-2",
	"uphy-lane-3",
	"uphy-lane-4",
	"uphy-lane-5",
};

#define TEGRA186_FUNCTION(_name)					\
	{								\
		.name = #_name,						\
		.num_groups = ARRAY_SIZE(tegra186_##_name##_groups),	\
		.groups = tegra186_##_name##_groups,			\
	}

static struct tegra_padctl_uphy_function tegra186_functions[] = {
	TEGRA186_FUNCTION(hsic),
	TEGRA186_FUNCTION(hsic_plus),
	TEGRA186_FUNCTION(xusb),
	TEGRA186_FUNCTION(uart),
	TEGRA186_FUNCTION(pcie),
	TEGRA186_FUNCTION(usb3),
	TEGRA186_FUNCTION(sata),
	TEGRA186_FUNCTION(mphy),
};

static const unsigned int tegra186_otg_functions[] = {
	TEGRA186_FUNC_RSVD_0,
	TEGRA186_FUNC_XUSB,
	TEGRA186_FUNC_UART,
	TEGRA186_FUNC_RSVD_3
};

static const unsigned int tegra186_hsic_functions[] = {
	TEGRA186_FUNC_HSIC,
	TEGRA186_FUNC_HSIC_PLUS,
};

static const unsigned int tegra186_uphy_functions[] = {
	TEGRA186_FUNC_USB3,
	TEGRA186_FUNC_PCIE,
	TEGRA186_FUNC_SATA,
	TEGRA186_FUNC_MPHY,
};

#define TEGRA186_LANE(_name, _offset, _shift, _mask, _iddq, _funcs)	\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.iddq = _iddq,						\
		.num_funcs = ARRAY_SIZE(tegra186_##_funcs##_functions),	\
		.funcs = tegra186_##_funcs##_functions,			\
	}

static const struct tegra_padctl_uphy_lane tegra186_lanes[] = {
	TEGRA186_LANE("otg-0",  0x004,  0, 0x3, 0, otg),
	TEGRA186_LANE("otg-1",  0x004,  2, 0x3, 0, otg),
	TEGRA186_LANE("otg-2",  0x004,  4, 0x3, 0, otg),
	TEGRA186_LANE("otg-3",  0x004,  6, 0x3, 0, otg),
	TEGRA186_LANE("hsic-0", 0x004, 20, 0x1, 0, hsic),
	TEGRA186_LANE("hsic-1", 0x004, 21, 0x1, 0, hsic),
	TEGRA186_LANE("uphy-lane-0", 0x284, 0, 0x3, 9, uphy),
	TEGRA186_LANE("uphy-lane-1", 0x284, 0, 0x3, 9, uphy),
	TEGRA186_LANE("uphy-lane-2", 0x284, 0, 0x3, 9, uphy),
	TEGRA186_LANE("uphy-lane-3", 0x284, 0, 0x3, 9, uphy),
	TEGRA186_LANE("uphy-lane-4", 0x284, 0, 0x3, 9, uphy),
	TEGRA186_LANE("uphy-lane-5", 0x284, 0, 0x3, 9, uphy),
};

static const struct tegra_padctl_uphy_soc tegra186_soc = {
	.num_pins = ARRAY_SIZE(tegra186_pins),
	.pins = tegra186_pins,
	.num_functions = ARRAY_SIZE(tegra186_functions),
	.functions = tegra186_functions,
	.num_lanes = ARRAY_SIZE(tegra186_lanes),
	.lanes = tegra186_lanes,
	.hsic_port_offset = 6,
};

static const struct of_device_id tegra_padctl_uphy_of_match[] = {
	{ .compatible = "nvidia,tegra186-padctl-uphy", .data = &tegra186_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_padctl_uphy_of_match);

static int tegra_xusb_read_fuse_calibration(struct tegra_padctl_uphy *padctl)
{
	unsigned int i;
	u32 value;

	value = tegra_fuse_readl(FUSE_SKU_USB_CALIB_0);
	dev_info(padctl->dev, "FUSE_SKU_USB_CALIB_0 0x%x\n", value);
	for (i = 0; i < TEGRA_XUSB_UTMI_PHYS; i++) {
		padctl->calib.hs_curr_level[i] =
			(value >> USB_CALIB_HS_CURR_LEVEL_PADX_SHIFT(i)) &
			USB_CALIB_HS_CURR_LEVEL_PAD_MASK;
	}
	padctl->calib.hs_term_range_adj =
		(value >> USB_CALIB_HS_TERM_RANGE_ADJ_SHIFT) &
		USB_CALIB_HS_TERM_RANGE_ADJ_MASK;

	value = tegra_fuse_readl(FUSE_USB_CALIB_EXT_0);
	dev_info(padctl->dev, "FUSE_USB_CALIB_EXT_0 0x%x\n", value);
	padctl->calib.rpd_ctrl =
		(value >> USB_CALIB_EXT_RPD_CTRL_SHIFT) &
		USB_CALIB_EXT_RPD_CTRL_MASK;

	return 0;
}

static int tegra_mphy_stata_fuse_calibration(struct tegra_padctl_uphy *padctl)
{
	u32 value;

	value = tegra_fuse_readl(FUSE_SATA_MPHY_ODM_CALIB_0);
	padctl->fuse_calib.sata_mphy_odm = value;

	value = tegra_fuse_readl(FUSE_SATA_NV_CALIB_0);
	padctl->fuse_calib.sata_nv = value;

	value = tegra_fuse_readl(FUSE_MPHY_NV_CALIB_0);
	padctl->fuse_calib.mphy_nv = value;

	return 0;
}

static int tegra_xusb_setup_usb(struct tegra_padctl_uphy *ctx)
{
	struct phy *phy;
	unsigned int i;

	for (i = 0; i < TEGRA_XUSB_USB3_PHYS; i++) {
		phy = devm_phy_create(ctx->dev, &usb3_phy_ops, NULL);
		TRACE("SS P%d phy %p\n", i, phy);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		ctx->usb3_phys[i] = phy;
		phy_set_drvdata(phy, ctx);
	}

	for (i = 0; i < TEGRA_XUSB_UTMI_PHYS; i++) {
		char reg_name[sizeof("vbus-N")];

		sprintf(reg_name, "vbus-%d", i);
		ctx->vbus[i] = devm_regulator_get(ctx->dev, reg_name);
		TRACE("UTMI P%d vbus %p IS_ERR %ld PTR_ERR %ld\n", i, ctx->vbus[i], IS_ERR(ctx->vbus[i]), PTR_ERR(ctx->vbus[i]));
		if (IS_ERR(ctx->vbus[i]))
			return PTR_ERR(ctx->vbus[i]);

		phy = devm_phy_create(ctx->dev, &utmi_phy_ops, NULL);
		TRACE("UTMI P%d phy %p IS_ERR %ld PTR_ERR %ld\n", i, phy, IS_ERR(phy), PTR_ERR(phy));
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		ctx->utmi_phys[i] = phy;
		phy_set_drvdata(phy, ctx);
	}

	ctx->vddio_hsic = devm_regulator_get(ctx->dev, "vddio_hsic");
	if (IS_ERR(ctx->vddio_hsic))
		return PTR_ERR(ctx->vddio_hsic);

	for (i = 0; i < TEGRA_XUSB_HSIC_PHYS; i++) {
		phy = devm_phy_create(ctx->dev, &hsic_phy_ops, NULL);
		TRACE("HSIC P%d phy %p\n", i, phy);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		ctx->hsic_phys[i] = phy;
		phy_set_drvdata(phy, ctx);
	}

	return 0;
}

#define DEBUG
#ifdef DEBUG
#define reg_dump(_dev, _base, _reg) \
	dev_info(_dev, "%s @%x = 0x%x\n", #_reg, _reg, ioread32(_base + _reg));
#else
#define reg_dump(_dev, _base, _reg)	do {} while (0)
#endif

static void __iomem *car_base;
static void tegra186_fpga_hacks_init(struct platform_device *pdev)
{
#if defined(CONFIG_ARCH_TEGRA_18x_SOC)
	car_base = devm_ioremap(&pdev->dev, 0x05000000, 0x1000000);
#endif
	if (!car_base)
		dev_err(&pdev->dev, "failed to map CAR mmio\n");
}

static
void tegra186_fpga_hacks_uphy_reset(struct platform_device *pdev, bool on)
{
	int reg;
	if (!car_base) {
		dev_err(&pdev->dev, "CAR mmio is not mapped\n");
		return;
	}

	reg_dump(&pdev->dev, car_base, CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
	reg = ioread32(car_base + CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
	if (on)
		reg |= SWR_UPHY_RST;
	else
		reg &= ~SWR_UPHY_RST;
	iowrite32(reg, car_base + CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
	reg_dump(&pdev->dev, car_base, CLK_RST_CONTROLLER_RST_DEV_UPHY_0);
}

static int tegra186_padctl_uphy_probe(struct platform_device *pdev)
{
	struct tegra_padctl_uphy *ctx;
	const struct of_device_id *match;
	struct resource *res;
	struct phy *phy;
	int err;

	TRACE();
	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	TRACE();
	platform_set_drvdata(pdev, ctx);
	mutex_init(&ctx->lock);
	ctx->dev = &pdev->dev;

	TRACE();
	match = of_match_node(tegra_padctl_uphy_of_match, pdev->dev.of_node);
	ctx->soc = match->data;

	TRACE();


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->padctl_regs = devm_ioremap_resource(&pdev->dev, res);
	TRACE_DEV(&pdev->dev, "padctl mmio start %pa end %pa\n", &res->start, &res->end);
	if (IS_ERR(ctx->padctl_regs))
		return PTR_ERR(ctx->padctl_regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ctx->uphy_regs = devm_ioremap_resource(&pdev->dev, res);
	TRACE_DEV(&pdev->dev, "uphy mmio start %pa end %pa\n", &res->start, &res->end);
	if (IS_ERR(ctx->uphy_regs))
		return PTR_ERR(ctx->uphy_regs);

	ctx->uphy_pll_regs[0] = ctx->uphy_regs;
	ctx->uphy_lane_regs[0] = ctx->uphy_regs + 0x20000;
	ctx->uphy_lane_regs[1] = ctx->uphy_regs + 0x30000;
	ctx->uphy_lane_regs[2] = ctx->uphy_regs + 0x40000;
	ctx->uphy_lane_regs[3] = ctx->uphy_regs + 0x50000;
	ctx->uphy_lane_regs[4] = ctx->uphy_regs + 0x60000;
	ctx->uphy_lane_regs[5] = ctx->uphy_regs + 0x70000;
	ctx->uphy_pll_regs[1] = ctx->uphy_regs + 0x80000;

	TRACE();
	if (tegra_platform_is_silicon()) {
		err = tegra_xusb_read_fuse_calibration(ctx);
		if (err < 0)
			return err;

		err = tegra_mphy_stata_fuse_calibration(ctx);
		if (err < 0)
			return err;
	}

	TRACE();
	if (tegra_platform_is_silicon()) {
		/* TODO: check clock framework */
		ctx->clk = devm_clk_get(ctx->dev, "uphy");
		if (IS_ERR(ctx->clk)) {
			dev_err(ctx->dev, "failed to get uphy pad clock\n");
			return PTR_ERR(ctx->clk);
		}
		tegra_periph_reset_deassert(ctx->clk);
	}

	if (tegra_platform_is_fpga()) {
		tegra186_fpga_hacks_init(pdev);
		tegra186_fpga_hacks_uphy_reset(pdev, false);
	}

	TRACE();
	memset(&ctx->desc, 0, sizeof(ctx->desc));
	ctx->desc.name = dev_name(ctx->dev);
	ctx->desc.pins = ctx->soc->pins;
	ctx->desc.npins = ctx->soc->num_pins;
	ctx->desc.pctlops = &tegra_xusb_padctl_pinctrl_ops;
	ctx->desc.pmxops = &tegra186_padctl_uphy_pinmux_ops;
	ctx->desc.confops = &tegra_xusb_padctl_pinconf_ops;
	ctx->desc.owner = THIS_MODULE;

	ctx->pinctrl = pinctrl_register(&ctx->desc, &pdev->dev, ctx);
	if (!ctx->pinctrl) {
		dev_err(&pdev->dev, "failed to register pincontrol\n");
		err = -ENODEV;
		goto assert_clk_reset;
	}

	TRACE();
	phy = devm_phy_create(&pdev->dev, &pcie_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	ctx->pcie_phys[0] = phy;
	phy_set_drvdata(phy, ctx);

	TRACE();
	phy = devm_phy_create(&pdev->dev, &sata_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	ctx->sata_phys[0] = phy;
	phy_set_drvdata(phy, ctx);

	TRACE();
	phy = devm_phy_create(&pdev->dev, &ufs_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}

	ctx->ufs_phys[0] = phy;
	phy_set_drvdata(phy, ctx);

		TRACE();
		err = tegra_xusb_setup_usb(ctx);
		TRACE("err %d\n", err);
		if (err)
			goto unregister;

	TRACE();
	ctx->provider = devm_of_phy_provider_register(&pdev->dev,
					tegra186_padctl_uphy_xlate);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register PHYs: %d\n", err);
		goto unregister;
	}

	TRACE();
	return 0;

unregister:
	TRACE();
	pinctrl_unregister(ctx->pinctrl);
assert_clk_reset:
	if (tegra_platform_is_silicon())
		tegra_periph_reset_assert(ctx->clk);
	return err;
}

static int tegra186_padctl_uphy_remove(struct platform_device *pdev)
{
	struct tegra_padctl_uphy *ctx = platform_get_drvdata(pdev);

	pinctrl_unregister(ctx->pinctrl);
	tegra_periph_reset_assert(ctx->clk);
	return 0;
}

static struct platform_driver tegra186_padctl_uphy_driver = {
	.driver = {
		.name = "tegra186-padctl-uphy",
		.of_match_table = tegra_padctl_uphy_of_match,
	},
	.probe = tegra186_padctl_uphy_probe,
	.remove = tegra186_padctl_uphy_remove,
};
module_platform_driver(tegra186_padctl_uphy_driver);

MODULE_AUTHOR("JC Kuo <jckuo@nvidia.com>");
MODULE_DESCRIPTION("Tegra 186 XUSB Pad Control and UPHY driver");
MODULE_LICENSE("GPL v2");
