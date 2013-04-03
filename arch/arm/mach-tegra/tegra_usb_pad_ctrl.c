/*
 * Copyright (C) 2013 NVIDIA Corporation
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

#include <linux/export.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/tegra_usb_pad_ctrl.h>

#include "iomap.h"

static DEFINE_SPINLOCK(utmip_pad_lock);
static DEFINE_SPINLOCK(xusb_padctl_lock);
static int utmip_pad_count;
static struct clk *utmi_pad_clk;
#define PLL_LOCK_MIN_TIME 100
#define PLL_LOCK_MAX_TIME 1000

int utmi_phy_iddq_override(bool set)
{
	unsigned long val, flags;
	void __iomem *clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

	spin_lock_irqsave(&utmip_pad_lock, flags);
	val = readl(clk_base + UTMIPLL_HW_PWRDN_CFG0);
	if (set && !utmip_pad_count)
		val |= UTMIPLL_HW_PWRDN_CFG0_IDDQ_OVERRIDE;
	else if (!set && utmip_pad_count)
		val &= ~UTMIPLL_HW_PWRDN_CFG0_IDDQ_OVERRIDE;
	else
		goto out1;
	val |= UTMIPLL_HW_PWRDN_CFG0_IDDQ_SWCTL;
	writel(val, clk_base + UTMIPLL_HW_PWRDN_CFG0);

out1:
	spin_unlock_irqrestore(&utmip_pad_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(utmi_phy_iddq_override);

int utmi_phy_pad_enable(void)
{
	unsigned long val, flags;
	void __iomem *pad_base =  IO_ADDRESS(TEGRA_USB_BASE);

	if (!utmi_pad_clk)
		utmi_pad_clk = clk_get_sys("utmip-pad", NULL);

	clk_enable(utmi_pad_clk);

	spin_lock_irqsave(&utmip_pad_lock, flags);
	utmip_pad_count++;

	val = readl(pad_base + UTMIP_BIAS_CFG0);
	val &= ~(UTMIP_OTGPD | UTMIP_BIASPD);
	val |= UTMIP_HSSQUELCH_LEVEL(0x2) | UTMIP_HSDISCON_LEVEL(0x1) |
		UTMIP_HSDISCON_LEVEL_MSB;
	writel(val, pad_base + UTMIP_BIAS_CFG0);

	spin_unlock_irqrestore(&utmip_pad_lock, flags);

	clk_disable(utmi_pad_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(utmi_phy_pad_enable);

int utmi_phy_pad_disable(void)
{
	unsigned long val, flags;
	void __iomem *pad_base =  IO_ADDRESS(TEGRA_USB_BASE);

	if (!utmi_pad_clk)
		utmi_pad_clk = clk_get_sys("utmip-pad", NULL);

	clk_enable(utmi_pad_clk);
	spin_lock_irqsave(&utmip_pad_lock, flags);

	if (!utmip_pad_count) {
		pr_err("%s: utmip pad already powered off\n", __func__);
		goto out;
	}
	if (--utmip_pad_count == 0) {
		val = readl(pad_base + UTMIP_BIAS_CFG0);
		val |= UTMIP_OTGPD | UTMIP_BIASPD;
		val &= ~(UTMIP_HSSQUELCH_LEVEL(~0) | UTMIP_HSDISCON_LEVEL(~0) |
			UTMIP_HSDISCON_LEVEL_MSB);
		writel(val, pad_base + UTMIP_BIAS_CFG0);
	}
out:
	spin_unlock_irqrestore(&utmip_pad_lock, flags);
	clk_disable(utmi_pad_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(utmi_phy_pad_disable);

int pcie_phy_pad_enable(void)
{
	unsigned long val, flags, timeout;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	spin_lock_irqsave(&xusb_padctl_lock, flags);

	/* set up PLL inputs in PLL_CTL1 */
	val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);
	val &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL_MASK;
	val |= XUSB_PADCTL_IOPHY_PLL_P0_CTL1_REFCLK_SEL;
	writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);

	/* set TX ref sel to div5 in PLL_CTL2 */
	/* T124 is div5 while T30 is div10,beacuse CAR will divide 2 */
	/* in GEN1 mode in T124,and if div10,it will be 125MHZ */
	val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0);
	val |= (XUSB_PADCTL_IOPHY_PLL_P0_CTL2_REFCLKBUF_EN |
		XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_EN |
		XUSB_PADCTL_IOPHY_PLL_P0_CTL2_TXCLKREF_SEL);
	writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0);

	/* take PLL out of reset */
	val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);
	val |= XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST_;
	writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);

	/* Wait for the PLL to lock */
	timeout = 300;
	do {
		val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);
		usleep_range(PLL_LOCK_MIN_TIME, PLL_LOCK_MAX_TIME);
		if (--timeout == 0) {
			pr_err("Tegra PCIe error: timeout waiting for PLL\n");
			return -EBUSY;
		}
	} while (!(val & XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET));

	/* disable IDDQ for all lanes for pcie tests */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	val |= XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK0 |
		XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK1 |
		XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK2 |
		XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK3 |
		XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK4;
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);

	/* clear AUX_MUX_LP0 related bits in ELPG_PROGRAM */
	val = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	udelay(1);
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	udelay(100);
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	udelay(100);
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);

	/* program ownership of all lanes to PCIe */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	val &= ~(XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE0 |
		XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE1 |
		XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE2 |
		XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE3 |
		XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE4);
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);

	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(pcie_phy_pad_enable);
