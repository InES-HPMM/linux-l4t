/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _TEGRA_USB_PAD_CTRL_INTERFACE_H_
#define _TEGRA_USB_PAD_CTRL_INTERFACE_H_

#define UTMIPLL_HW_PWRDN_CFG0			0x52c
#define   UTMIPLL_HW_PWRDN_CFG0_IDDQ_OVERRIDE  (1<<1)
#define   UTMIPLL_HW_PWRDN_CFG0_IDDQ_SWCTL     (1<<0)

#define UTMIP_BIAS_CFG0		0x80c
#define UTMIP_OTGPD			(1 << 11)
#define UTMIP_BIASPD			(1 << 10)
#define UTMIP_HSSQUELCH_LEVEL(x)	(((x) & 0x3) << 0)
#define UTMIP_HSDISCON_LEVEL(x)	(((x) & 0x3) << 2)
#define UTMIP_HSDISCON_LEVEL_MSB	(1 << 24)

#define PCIE_LANES_X4_X1		0
#define PCIE_LANES_X4_X0		2
#define PCIE_LANES_X2_X1		3

/* xusb padctl regs for pad programming of t124 pcie */
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0	0x40
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL_MASK	(0xF << 12)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL	(0x0 << 12)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST_		(1 << 1)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET		(1 << 19)

#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0	0x44
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_SEL	(1 << 4)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_EN	(1 << 5)
#define XUSB_PADCTL_IOPHY_PLL_P0_CTL2_REFCLKBUF_EN	(1 << 6)

#define XUSB_PADCTL_ELPG_PROGRAM_0		0x1c
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN	(1 << 24)
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY	(1 << 25)
#define XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN	(1 << 26)

#define XUSB_PADCTL_USB3_PAD_MUX_0		0x134
#define XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK0	(1 << 1)
#define XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK1	(1 << 2)
#define XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK2	(1 << 3)
#define XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK3	(1 << 4)
#define XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK4	(1 << 5)
#define XUSB_PADCTL_USB3_PAD_MUX_FORCE_SATA_PAD_IDDQ_DISABLE_MASK0	(1 << 6)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE0	(0x3 << 16)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE1	(0x3 << 18)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE2	(0x3 << 20)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE3	(0x3 << 22)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE4	(0x3 << 24)
#define XUSB_PADCTL_USB3_PAD_MUX_SATA_PAD_LANE0	(0x3 << 26)
#define XUSB_PADCTL_USB3_PAD_MUX_SATA_PAD_LANE0_OWNER_USB3_SS	(0x1 << 26)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE0_OWNER_USB3_SS	(0x1 << 16)
#define XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE1_OWNER_USB3_SS	(0x1 << 18)

int utmi_phy_pad_disable(void);
int utmi_phy_pad_enable(void);
int usb3_phy_pad_enable(u8 lane_owner);
#ifdef CONFIG_ARCH_TEGRA_12x_SOC
int pcie_phy_pad_enable(int lane_owner);
#endif

int utmi_phy_iddq_override(bool set);
#endif
