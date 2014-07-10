/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */


#include <linux/export.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/tegra-fuse.h>

#include <mach/tegra_usb_pad_ctrl.h>

#include "../../../arch/arm/mach-tegra/iomap.h"

static DEFINE_SPINLOCK(utmip_pad_lock);
static DEFINE_SPINLOCK(hsic_pad_lock);
static DEFINE_SPINLOCK(xusb_padctl_lock);
static int utmip_pad_count;
static int hsic_pad_count;
static struct clk *utmi_pad_clk;

void tegra_xhci_release_otg_port(bool release)
{
	void __iomem *padctl_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	u32 reg;
	unsigned long flags;

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	reg = readl(padctl_base + XUSB_PADCTL_USB2_PAD_MUX_0);

	reg &= ~PAD_PORT_MASK(0);
	if (release)
		reg |= PAD_PORT_SNPS(0);
	else
		reg |= PAD_PORT_XUSB(0);

	writel(reg, padctl_base + XUSB_PADCTL_USB2_PAD_MUX_0);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_xhci_release_otg_port);

void tegra_xhci_release_dev_port(bool release)
{
	void __iomem *padctl_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	u32 reg;
	unsigned long flags;

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	reg = readl(padctl_base + XUSB_PADCTL_USB2_PAD_MUX_0);

	reg &= ~USB2_OTG_PORT_CAP(0, ~0);
	if (release)
		reg |= USB2_OTG_PORT_CAP(0, XUSB_DEVICE_MODE);
	else
		reg |= USB2_OTG_PORT_CAP(0, XUSB_HOST_MODE);

	writel(reg, padctl_base + XUSB_PADCTL_USB2_PAD_MUX_0);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_xhci_release_dev_port);

void tegra_xhci_ss_wake_on_interrupts(u32 enabled_port, bool enable)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	u32 elpg_program0;
	unsigned long flags;

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	/* clear any event */
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	elpg_program0 |= (SS_PORT0_WAKEUP_EVENT | SS_PORT1_WAKEUP_EVENT
			| SS_PORT2_WAKEUP_EVENT | SS_PORT3_WAKEUP_EVENT);
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);

	/* enable ss wake interrupts */
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);

	if (enable) {
		/* enable interrupts */
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 |= SS_PORT0_WAKE_INTERRUPT_ENABLE;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 |= SS_PORT1_WAKE_INTERRUPT_ENABLE;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 |= SS_PORT2_WAKE_INTERRUPT_ENABLE;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 |= SS_PORT3_WAKE_INTERRUPT_ENABLE;
	} else {
		/* disable interrupts */
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 &= ~SS_PORT0_WAKE_INTERRUPT_ENABLE;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 &= ~SS_PORT1_WAKE_INTERRUPT_ENABLE;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 &= ~SS_PORT2_WAKE_INTERRUPT_ENABLE;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 &= ~SS_PORT3_WAKE_INTERRUPT_ENABLE;
	}
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_xhci_ss_wake_on_interrupts);

void tegra_xhci_hs_wake_on_interrupts(u32 portmap, bool enable)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	u32 elpg_program0;
	unsigned long flags;

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	elpg_program0 |= (USB2_PORT0_WAKEUP_EVENT |
			USB2_PORT1_WAKEUP_EVENT |
			USB2_PORT2_WAKEUP_EVENT |
			USB2_PORT3_WAKEUP_EVENT |
			USB2_HSIC_PORT0_WAKEUP_EVENT |
			USB2_HSIC_PORT1_WAKEUP_EVENT);

	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);

	/* Enable the wake interrupts */
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	if (enable) {
		/* enable interrupts */
		if (portmap & TEGRA_XUSB_USB2_P0)
			elpg_program0 |= USB2_PORT0_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_USB2_P1)
			elpg_program0 |= USB2_PORT1_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_USB2_P2)
			elpg_program0 |= USB2_PORT2_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_USB2_P3)
			elpg_program0 |= USB2_PORT3_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_HSIC_P0)
			elpg_program0 |= USB2_HSIC_PORT0_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_HSIC_P1)
			elpg_program0 |= USB2_HSIC_PORT1_WAKE_INTERRUPT_ENABLE;
	} else {
		if (portmap & TEGRA_XUSB_USB2_P0)
			elpg_program0 &= ~USB2_PORT0_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_USB2_P1)
			elpg_program0 &= ~USB2_PORT1_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_USB2_P2)
			elpg_program0 &= ~USB2_PORT2_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_USB2_P3)
			elpg_program0 &= ~USB2_PORT3_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_HSIC_P0)
			elpg_program0 &= ~USB2_HSIC_PORT0_WAKE_INTERRUPT_ENABLE;
		if (portmap & TEGRA_XUSB_HSIC_P1)
			elpg_program0 &= ~USB2_HSIC_PORT1_WAKE_INTERRUPT_ENABLE;
	}
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_xhci_hs_wake_on_interrupts);

void tegra_xhci_ss_wake_signal(u32 enabled_port, bool enable)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	u32 elpg_program0;
	unsigned long flags;

	/* DO NOT COMBINE BELOW 2 WRITES */

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	/* Assert/Deassert clamp_en_early signals to SSP0/1 */
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
#else
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
#endif
	if (enable) {
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 |= SSP0_ELPG_CLAMP_EN_EARLY;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 |= SSP1_ELPG_CLAMP_EN_EARLY;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 |= SSP2_ELPG_CLAMP_EN_EARLY;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 |= SSP3_ELPG_CLAMP_EN_EARLY;
	} else {
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 &= ~SSP0_ELPG_CLAMP_EN_EARLY;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 &= ~SSP1_ELPG_CLAMP_EN_EARLY;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 &= ~SSP2_ELPG_CLAMP_EN_EARLY;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 &= ~SSP3_ELPG_CLAMP_EN_EARLY;
	}
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
#else
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
#endif
	/*
	 * Check the LP0 figure and leave gap bw writes to
	 * clamp_en_early and clamp_en
	 */
	udelay(100);

	/* Assert/Deassert clam_en signal */
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
#else
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
#endif

	if (enable) {
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 |= SSP0_ELPG_CLAMP_EN;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 |= SSP1_ELPG_CLAMP_EN;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 |= SSP2_ELPG_CLAMP_EN;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 |= SSP3_ELPG_CLAMP_EN;
	} else {
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 &= ~SSP0_ELPG_CLAMP_EN;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 &= ~SSP1_ELPG_CLAMP_EN;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 &= ~SSP2_ELPG_CLAMP_EN;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 &= ~SSP3_ELPG_CLAMP_EN;
	}

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
#else
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
#endif
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);

	/* wait for 250us for the writes to propogate */
	if (enable)
		udelay(250);
}
EXPORT_SYMBOL_GPL(tegra_xhci_ss_wake_signal);

void tegra_xhci_ss_vcore(u32 enabled_port, bool enable)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	u32 elpg_program0;
	unsigned long flags;

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	/* Assert vcore_off signal */
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
#else
	elpg_program0 = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
#endif

	if (enable) {
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 |= SSP0_ELPG_VCORE_DOWN;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 |= SSP1_ELPG_VCORE_DOWN;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 |= SSP2_ELPG_VCORE_DOWN;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 |= SSP3_ELPG_VCORE_DOWN;
	} else {
		if (enabled_port & TEGRA_XUSB_SS_P0)
			elpg_program0 &= ~SSP0_ELPG_VCORE_DOWN;
		if (enabled_port & TEGRA_XUSB_SS_P1)
			elpg_program0 &= ~SSP1_ELPG_VCORE_DOWN;
		if (enabled_port & TEGRA_XUSB_SS_P2)
			elpg_program0 &= ~SSP2_ELPG_VCORE_DOWN;
		if (enabled_port & TEGRA_XUSB_SS_P3)
			elpg_program0 &= ~SSP3_ELPG_VCORE_DOWN;
	}
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
#else
	writel(elpg_program0, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
#endif
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_xhci_ss_vcore);

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

static void utmi_phy_pad(bool enable)
{
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	if (enable) {
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_PAD_MUX_0
			, BIAS_PAD_MASK , BIAS_PAD_XUSB);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_1
			, TRK_START_TIMER_MASK , TRK_START_TIMER);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_1
			, TRK_DONE_RESET_TIMER_MASK , TRK_DONE_RESET_TIMER);

		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_0
			, PD_MASK , PD);

		udelay(1);

		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_1
			, PD_TRK_MASK , PD_TRK);
	} else {

		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_PAD_MUX_0
			, BIAS_PAD_MASK , BIAS_PAD_XUSB);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_0
			, PD_MASK , PD_MASK);
	}
#else
	unsigned long val;
	void __iomem *pad_base =  IO_ADDRESS(TEGRA_USB_BASE);

	if (enable) {
		val = readl(pad_base + UTMIP_BIAS_CFG0);
		val &= ~(UTMIP_OTGPD | UTMIP_BIASPD);
		val |= UTMIP_HSSQUELCH_LEVEL(0x2) | UTMIP_HSDISCON_LEVEL(0x3) |
			UTMIP_HSDISCON_LEVEL_MSB;
		writel(val, pad_base + UTMIP_BIAS_CFG0);

#if defined(CONFIG_USB_XHCI_HCD)
	tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_0
			, PD_MASK , 0);
#endif
	} else {
		val = readl(pad_base + UTMIP_BIAS_CFG0);
		val |= UTMIP_OTGPD | UTMIP_BIASPD;
		val &= ~(UTMIP_HSSQUELCH_LEVEL(~0) | UTMIP_HSDISCON_LEVEL(~0) |
			UTMIP_HSDISCON_LEVEL_MSB);
		writel(val, pad_base + UTMIP_BIAS_CFG0);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_BIAS_PAD_CTL_0
			, PD_MASK , PD_MASK);
	}
#endif
}

int utmi_phy_pad_enable(void)
{
	unsigned long flags;

	if (!utmi_pad_clk)
		utmi_pad_clk = clk_get_sys("utmip-pad", NULL);

	clk_enable(utmi_pad_clk);

	spin_lock_irqsave(&utmip_pad_lock, flags);
	utmip_pad_count++;

	utmi_phy_pad(true);

	spin_unlock_irqrestore(&utmip_pad_lock, flags);

	clk_disable(utmi_pad_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(utmi_phy_pad_enable);

int utmi_phy_pad_disable(void)
{
	unsigned long flags;

	if (!utmi_pad_clk)
		utmi_pad_clk = clk_get_sys("utmip-pad", NULL);

	clk_enable(utmi_pad_clk);
	spin_lock_irqsave(&utmip_pad_lock, flags);

	if (!utmip_pad_count) {
		pr_err("%s: utmip pad already powered off\n", __func__);
		goto out;
	}
	if (--utmip_pad_count == 0)
		utmi_phy_pad(false);
out:
	spin_unlock_irqrestore(&utmip_pad_lock, flags);
	clk_disable(utmi_pad_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(utmi_phy_pad_disable);

int hsic_trk_enable(void)
{
	unsigned long flags;

	/* TODO : need to enable HSIC_TRK clk */
	spin_lock_irqsave(&hsic_pad_lock, flags);
	hsic_pad_count++;
	tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_PAD_MUX_0
			, HSIC_PAD_TRK , (0x1 << 16));
	tegra_usb_pad_reg_update(XUSB_PADCTL_HSIC_PAD_TRK_CTL_0
			, HSIC_TRK_START_TIMER_MASK , (0x1e << 5));
	tegra_usb_pad_reg_update(XUSB_PADCTL_HSIC_PAD_TRK_CTL_0
			, HSIC_TRK_DONE_RESET_TIMER_MASK , (0xa << 12));
	tegra_usb_pad_reg_update(XUSB_PADCTL_HSIC_PAD1_CTL_0_0
				, PAD1_PD_TX_MASK , ~(PAD1_PD_TX_MASK));

	udelay(1);
	tegra_usb_pad_reg_update(XUSB_PADCTL_HSIC_PAD_TRK_CTL_0
				, HSIC_PD_TRK_MASK , (0 << 19));

	spin_unlock_irqrestore(&hsic_pad_lock, flags);
	/* TODO : need to disable HSIC_TRK clk */
	return 0;
}
EXPORT_SYMBOL_GPL(hsic_trk_enable);


static void get_usb_calib_data(int pad, u32 *hs_curr_level_pad,
		u32 *term_range_adj, u32 *rpd_ctl, u32 *hs_iref_cap)
{
	u32 usb_calib0 = tegra_fuse_readl(FUSE_SKU_USB_CALIB_0);
	/*
	 * read from usb_calib0 and pass to driver
	 * set HS_CURR_LEVEL (PAD0)	= usb_calib0[5:0]
	 * set TERM_RANGE_ADJ		= usb_calib0[10:7]
	 * set HS_SQUELCH_LEVEL		= usb_calib0[12:11]
	 * set HS_IREF_CAP		= usb_calib0[14:13]
	 * set HS_CURR_LEVEL (PAD1)	= usb_calib0[20:15]
	 */
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	u32 usb_calib_exit = tegra_fuse_readl(FUSE_USB_CALIB_EXT_0);

	pr_info("usb_calib0 = 0x%08x\n", usb_calib0);
	pr_info("usb_calib_exit = 0x%08x\n", usb_calib_exit);

	*hs_curr_level_pad = (usb_calib0 >>
		((!pad) ? 0 : ((6 * (pad + 1)) - 1))) & 0x3f;
	*term_range_adj = (usb_calib0 >> 7) & 0xf;
	*rpd_ctl = (usb_calib_exit >> 7) & 0xf;
	*hs_iref_cap = 0;
#else
	pr_info("usb_calib0 = 0x%08x\n", usb_calib0);
	*hs_curr_level_pad = (usb_calib0 >>
			((!pad) ? 0 : 15)) & 0x3f;
	*term_range_adj = (usb_calib0 >> 7) & 0xf;
	*hs_iref_cap = (usb_calib0 >> 13) & 0x3;
	*rpd_ctl = 0;
#endif
}

void xusb_utmi_pad_init(int pad, u32 cap, bool external_pmic)
{
	unsigned long val, flags;
	u32 ctl0_offset, ctl1_offset, batry_chg_1;
	static u32 hs_curr_level_pad, term_range_adj;
	static u32 rpd_ctl, hs_iref_cap;
	static u8 utmi_pad_inited = 0x0;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	/* Check if programmed */
	if (utmi_pad_inited & (1 << pad)) {
		pr_warn("Already init for utmi pad %d\n", pad);
		spin_unlock_irqrestore(&xusb_padctl_lock, flags);
		return;
	}

	/* program utmi pad */
	val = readl(pad_base + XUSB_PADCTL_USB2_PAD_MUX_0);
	val &= ~PAD_PORT_MASK(pad);
	val |= PAD_PORT_XUSB(pad);
	writel(val, pad_base + XUSB_PADCTL_USB2_PAD_MUX_0);

	val = readl(pad_base + XUSB_PADCTL_USB2_PORT_CAP_0);
	val &= ~USB2_OTG_PORT_CAP(pad, ~0);
	val |= cap;
	writel(val, pad_base + XUSB_PADCTL_USB2_PORT_CAP_0);

	val = readl(pad_base + XUSB_PADCTL_USB2_OC_MAP_0);
	val &= ~PORT_OC_PIN(pad, ~0);
	val |= PORT_OC_PIN(pad, OC_DISABLED);
	writel(val, pad_base + XUSB_PADCTL_USB2_OC_MAP_0);

	val = readl(pad_base + XUSB_PADCTL_VBUS_OC_MAP_0);
	val &= ~VBUS_OC_MAP(pad, ~0);
	val |= VBUS_OC_MAP(pad, OC_DISABLED);
	writel(val, pad_base + XUSB_PADCTL_VBUS_OC_MAP_0);
	/*
	* Modify only the bits which belongs to the port
	* and enable respective VBUS_PAD for the port
	*/
	if (external_pmic) {
		val = readl(pad_base + XUSB_PADCTL_OC_DET_0);
		val &= ~VBUS_EN_OC_MAP(pad, ~0);
		val |= VBUS_EN_OC_MAP(pad, OC_VBUS_PAD(pad));
		writel(val, pad_base + XUSB_PADCTL_OC_DET_0);
	}

	val = readl(pad_base + XUSB_PADCTL_USB2_OC_MAP_0);
	val &= ~PORT_OC_PIN(pad, ~0);
	val |= PORT_OC_PIN(pad, OC_VBUS_PAD(pad));
	writel(val, pad_base + XUSB_PADCTL_USB2_OC_MAP_0);

	ctl0_offset = XUSB_PADCTL_USB2_OTG_PAD_CTL_0(pad);
	ctl1_offset = XUSB_PADCTL_USB2_OTG_PAD_CTL_1(pad);

	get_usb_calib_data(pad, &hs_curr_level_pad,
			&term_range_adj, &rpd_ctl, &hs_iref_cap);

	val = readl(pad_base + ctl0_offset);
	val &= ~(USB2_OTG_HS_CURR_LVL |	USB2_OTG_PD
			| USB2_OTG_PD2 | USB2_OTG_PD_ZI);
	val |= hs_curr_level_pad;
	writel(val, pad_base + ctl0_offset);

	val = readl(pad_base + ctl1_offset);
	val &= ~(USB2_OTG_TERM_RANGE_ADJ | RPD_CTRL
			| USB2_OTG_PD_DR | USB2_OTG_HS_IREF_CAP
			| USB2_OTG_PD_CHRP_FORCE_POWERUP
			| USB2_OTG_PD_DISC_FORCE_POWERUP);
	val |= (rpd_ctl << 26) |
		(term_range_adj << 3) |
		(hs_iref_cap << 9);
	writel(val, pad_base + ctl1_offset);

	batry_chg_1 = XUSB_PADCTL_USB2_BATTERY_CHRG_OTGPAD_CTL1(pad);

	val = readl(pad_base + batry_chg_1);
	val &= ~(VREG_FIX18 | VREG_LEV);
	if ((cap >> (4 * pad)) == XUSB_DEVICE_MODE)
		val |= VREG_LEV_EN;
	if ((cap >> (4 * pad)) == XUSB_HOST_MODE)
		val |= VREG_FIX18;
	writel(val, pad_base + batry_chg_1);

	utmi_pad_inited |= (1 << pad);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);

	/* Dump register value */
	pr_debug("[%s] UTMI pad %d, cap %x\n", __func__ , pad, cap);
	pr_debug("XUSB_PADCTL_USB2_PAD_MUX_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_USB2_PAD_MUX_0));
	pr_debug("XUSB_PADCTL_USB2_PORT_CAP_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_USB2_PORT_CAP_0));
	pr_debug("XUSB_PADCTL_USB2_OC_MAP_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_USB2_OC_MAP_0));
	pr_debug("XUSB_PADCTL_VBUS_OC_MAP_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_VBUS_OC_MAP_0));
	pr_debug("XUSB_PADCTL_USB2_OTG_PAD_CTL_0 (0x%x) = 0x%x\n"
			, ctl0_offset , readl(pad_base + ctl0_offset));
	pr_debug("XUSB_PADCTL_USB2_OTG_PAD_CTL_1 (0x%x) = 0x%x\n"
			, ctl1_offset, readl(pad_base + ctl1_offset));
	pr_debug("XUSB_PADCTL_USB2_BATTERY_CHRG_CTL1 (0x%x) = 0x%x\n"
			, batry_chg_1, readl(pad_base + batry_chg_1));
}
EXPORT_SYMBOL_GPL(xusb_utmi_pad_init);

void xusb_ss_pad_init(int pad, int port_map, u32 cap)
{
	unsigned long val, flags;
	static u8 ss_pad_inited = 0x0;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	/* Check if programmed */
	if (ss_pad_inited & (1 << pad)) {
		pr_warn("Already init for ss pad %d\n", pad);
		spin_unlock_irqrestore(&xusb_padctl_lock, flags);
		return;
	}

	/* Program ss pad */
	val = readl(pad_base + XUSB_PADCTL_SS_PORT_MAP);
	val &= ~SS_PORT_MAP(pad, ~0);
	val |= SS_PORT_MAP(pad, port_map);
	writel(val, pad_base + XUSB_PADCTL_SS_PORT_MAP);

	val = readl(pad_base + XUSB_PADCTL_USB2_PORT_CAP_0);
	val &= ~USB2_OTG_PORT_CAP(port_map, ~0);
	val |= USB2_OTG_PORT_CAP(port_map, cap);
	writel(val, pad_base + XUSB_PADCTL_USB2_PORT_CAP_0);

	ss_pad_inited |= (1 << pad);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
	pr_debug("[%s] ss pad %d\n", __func__ , pad);
	pr_debug("XUSB_PADCTL_SS_PORT_MAP = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_SS_PORT_MAP));
}
EXPORT_SYMBOL_GPL(xusb_ss_pad_init);

void usb2_vbus_id_init()
{
	unsigned long val, flags;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	val = readl(pad_base + XUSB_PADCTL_USB2_VBUS_ID_0);
	val &= ~(VBUS_SOURCE_SELECT(~0) | ID_SOURCE_SELECT(~0));
	val |= (VBUS_SOURCE_SELECT(1) | ID_SOURCE_SELECT(1));
	writel(val, pad_base + XUSB_PADCTL_USB2_VBUS_ID_0);

	/* clear wrong status change */
	val = readl(pad_base + XUSB_PADCTL_USB2_VBUS_ID_0);
	writel(val, pad_base + XUSB_PADCTL_USB2_VBUS_ID_0);

	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
	pr_debug("[%s] VBUS_ID\n", __func__);
	pr_debug("XUSB_PADCTL_USB2_VBUS_ID_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_USB2_VBUS_ID_0));
}
EXPORT_SYMBOL_GPL(usb2_vbus_id_init);

static void tegra_xusb_uphy_misc(bool ovrd)
{
#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	unsigned long val;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	/* WAR: Override pad controls and keep UPHy under IDDQ and */
	/* SLEEP 3 state during lane ownership changes and powergating */
	val = readl(pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2);
	if (ovrd) {
		val |= XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_TX_IDDQ |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_RX_IDDQ |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_TX_IDDQ_OVRD |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_RX_IDDQ_OVRD |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_TX_SLEEP |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_RX_SLEEP |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_TX_PWR_OVRD |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_RX_PWR_OVRD;
		udelay(1);
	} else {
		val &= ~(XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_TX_PWR_OVRD |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_RX_PWR_OVRD |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_TX_IDDQ_OVRD |
			XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2_RX_IDDQ_OVRD);
	}
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P0_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P1_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P2_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P3_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P4_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P5_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_P6_CTL2);
	writel(val, pad_base + XUSB_PADCTL_UPHY_MISC_PAD_S0_CTL2);
#endif
}

#ifdef CONFIG_ARCH_TEGRA_21x_SOC

static void usb3_pad_mux_set(u8 lane)
{
	unsigned long val;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	if (lane == USB3_LANE_NOT_ENABLED)
		return;
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	val &= ~PAD_MUX_PAD_LANE(lane, ~0);
	val |= PAD_MUX_PAD_LANE(lane, 1);
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);

}

static void usb3_lane_out_of_iddq(u8 lane)
{
	unsigned long val;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	if (lane == USB3_LANE_NOT_ENABLED)
		return;
	/* Bring enabled lane out of IDDQ */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	val |= PAD_MUX_PAD_LANE_IDDQ(lane, 1);
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
}

static void usb3_release_padmux_state_latch(void)
{
	unsigned long val;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	/* clear AUX_MUX_LP0 related bits in ELPG_PROGRAM_1 */
	udelay(1);

	val = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);

	udelay(100);

	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);

	udelay(100);

	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_1);

}
/*
 * Have below lane define used for each port
 * PCIE_LANEP6 0x6
 * PCIE_LANEP5 0x5
 * PCIE_LANEP4 0x4
 * PCIE_LANEP3 0x3
 * PCIE_LANEP2 0x2
 * PCIE_LANEP1 0x1
 * PCIE_LANEP0 0x0
 * SATA_LANE   0x8
 * NOT_SUPPORTED	0xF
 */
int usb3_phy_pad_enable(u32 lane_owner)
{
	unsigned long val, flags;
	int pad;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	static bool ss_pad_phy_inited;

	spin_lock_irqsave(&xusb_padctl_lock, flags);

	if (ss_pad_phy_inited) {
		pr_warn("Already init for ss pad phy\n");
		spin_unlock_irqrestore(&xusb_padctl_lock, flags);
		return 0;
	}
	/* Program SATA pad phy */
	if ((lane_owner & 0xf000) == SATA_LANE) {
		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0);
		val &= ~(XUSB_PADCTL_UPHY_PLL_S0_CTL1_MDIV_MASK |
				XUSB_PADCTL_UPHY_PLL_S0_CTL1_NDIV_MASK);
		val |= (XUSB_PADCTL_UPHY_PLL_S0_CTL1_PLL0_FREQ_MDIV |
				XUSB_PADCTL_UPHY_PLL_S0_CTL1_PLL0_FREQ_NDIV);
		writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0);

		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0);
		val &= ~XUSB_PADCTL_UPHY_PLL_S0_CTL2_CAL_CTRL_MASK;
		val |= XUSB_PADCTL_UPHY_PLL_S0_CTL2_PLL0_CAL_CTRL;
		writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0);

		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_4_0);
		val &= ~(XUSB_PADCTL_UPHY_PLL_S0_CTL4_TXCLKREF_SEL_MASK |
				XUSB_PADCTL_UPHY_PLL_S0_CTL4_TXCLKREF_EN_MASK);
		val |= (XUSB_PADCTL_UPHY_PLL_S0_CTL4_PLL0_TXCLKREF_SEL |
				XUSB_PADCTL_UPHY_PLL_S0_CTL4_PLL0_TXCLKREF_EN);
		writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_4_0);

		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_5_0);
		val &= ~XUSB_PADCTL_UPHY_PLL_S0_CTL5_DCO_CTRL_MASK;
		val |= XUSB_PADCTL_UPHY_PLL_S0_CTL5_PLL0_DCO_CTRL;
		writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_5_0);
	}

	/* Program PCIe pad phy */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL2_CAL_CTRL_MASK;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_CTRL;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL5_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL5_DCO_CTRL_MASK;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL5_PLL0_DCO_CTRL;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL5_0);

	/* Check PCIe/SATA pad phy programmed */
	pr_debug("[%s] PCIe pad/ SATA pad phy parameter\n", __func__);
	pr_debug("XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_1_0));
	pr_debug("XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_2_0));
	pr_debug("XUSB_PADCTL_UPHY_PLL_S0_CTL_4_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_4_0));
	pr_debug("XUSB_PADCTL_UPHY_PLL_S0_CTL_5_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_UPHY_PLL_S0_CTL_5_0));
	pr_debug("XUSB_PADCTL_UPHY_PLL_P0_CTL2_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0));
	pr_debug("XUSB_PADCTL_UPHY_PLL_P0_CTL5_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL5_0));


	/* Pad mux assign */
	tegra_xusb_uphy_misc(true);

	for (pad = 0; pad < SS_PAD_COUNT; pad++)
		usb3_pad_mux_set((lane_owner >> (pad * 4)) & 0xf);

	tegra_xusb_uphy_misc(false);

	pr_debug("[%s] ss pad mux\n", __func__);
	pr_debug("XUSB_PADCTL_USB3_PAD_MUX_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0));

	/* Bring out of IDDQ */
	for (pad = 0; pad < SS_PAD_COUNT; pad++)
		usb3_lane_out_of_iddq((lane_owner >> (pad * 4)) & 0xf);

	pr_debug("ss lane exit IDDQ\n");
	pr_debug("XUSB_PADCTL_USB3_PAD_MUX_0 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0));

	/* Release state latching */
	usb3_release_padmux_state_latch();

	pr_debug("ss release state latching\n");
	pr_debug("XUSB_PADCTL_ELPG_PROGRAM_1 = 0x%x\n"
			, readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_1));

	ss_pad_phy_inited = true;
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
	return 0;
}

#else
int usb3_phy_pad_enable(u32 lane_owner)
{
	unsigned long val, flags;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	spin_lock_irqsave(&xusb_padctl_lock, flags);

	/* Program SATA pad phy */
	if (lane_owner & BIT(0)) {
		void __iomem *clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

		val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_S0_CTL1_0);
		val &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL1_0_PLL0_REFCLK_NDIV_MASK;
		val |= XUSB_PADCTL_IOPHY_PLL_S0_CTL1_0_PLL0_REFCLK_NDIV;
		writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_S0_CTL1_0);

		val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0);
		val &= ~(XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_XDIGCLK_SEL_MASK |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_TXCLKREF_SEL |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_TCLKOUT_EN |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_PLL0_CP_CNTL_MASK |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_PLL1_CP_CNTL_MASK);
		val |= XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_XDIGCLK_SEL |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_TXCLKREF_SEL |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_PLL0_CP_CNTL |
			XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0_PLL1_CP_CNTL;

		writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_S0_CTL2_0);

		val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_S0_CTL3_0);
		val &= ~XUSB_PADCTL_IOPHY_PLL_S0_CTL3_0_RCAL_BYPASS;
		writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_S0_CTL3_0);

		/* Enable SATA PADPLL clocks */
		val = readl(clk_base + CLK_RST_CONTROLLER_SATA_PLL_CFG0_0);
		val &= ~SATA_PADPLL_RESET_SWCTL;
		val |= SATA_PADPLL_USE_LOCKDET | SATA_SEQ_START_STATE;
		writel(val, clk_base + CLK_RST_CONTROLLER_SATA_PLL_CFG0_0);

		udelay(1);

		val = readl(clk_base + CLK_RST_CONTROLLER_SATA_PLL_CFG0_0);
		val |= SATA_SEQ_ENABLE;
		writel(val, clk_base + CLK_RST_CONTROLLER_SATA_PLL_CFG0_0);
	}

	if ((lane_owner & BIT(1)) || (lane_owner & BIT(2))) {
		val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0);
		val &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL2_PLL0_CP_CNTL_MASK;
		val |= XUSB_PADCTL_IOPHY_PLL_P0_CTL2_PLL0_CP_CNTL_VAL;
		writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0);
	}

	/*
	 * program ownership of lanes owned by USB3 based on odmdata[28:30]
	 * odmdata[28] = 0 (SATA lane owner = SATA),
	 * odmdata[28] = 1 (SATA lane owner = USB3_SS port1)
	 * odmdata[29] = 0 (PCIe lane1 owner = PCIe),
	 * odmdata[29] = 1 (PCIe lane1 owner = USB3_SS port1)
	 * odmdata[30] = 0 (PCIe lane0 owner = PCIe),
	 * odmdata[30] = 1 (PCIe lane0 owner = USB3_SS port0)
	 * FIXME: Check any GPIO settings needed ?
	 */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	/* USB3_SS port1 can either be mapped to SATA lane or PCIe lane1 */
	if (lane_owner & BIT(0)) {
		val &= ~XUSB_PADCTL_USB3_PAD_MUX_SATA_PAD_LANE0;
		val |= XUSB_PADCTL_USB3_PAD_MUX_SATA_PAD_LANE0_OWNER_USB3_SS;
	} else if (lane_owner & BIT(1)) {
		val &= ~XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE1;
		val |= XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE1_OWNER_USB3_SS;
	}
	/* USB_SS port0 is alwasy mapped to PCIe lane0 */
	if (lane_owner & BIT(2)) {
		val &= ~XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE0;
		val |= XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE0_OWNER_USB3_SS;
	}
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);

	/* Bring enabled lane out of IDDQ */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	if (lane_owner & BIT(0))
		val |= XUSB_PADCTL_USB3_PAD_MUX_FORCE_SATA_PAD_IDDQ_DISABLE_MASK0;
	else if (lane_owner & BIT(1))
		val |= XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK1;
	if (lane_owner & BIT(2))
		val |= XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK0;
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);

	udelay(1);

	/* clear AUX_MUX_LP0 related bits in ELPG_PROGRAM */
	val = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);

	udelay(100);

	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);

	udelay(100);

	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);


	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
	return 0;
}

#endif
EXPORT_SYMBOL_GPL(usb3_phy_pad_enable);

void tegra_usb_pad_reg_update(u32 reg_offset, u32 mask, u32 val)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&xusb_padctl_lock, flags);

	reg = readl(pad_base + reg_offset);
	reg &= ~mask;
	reg |= val;
	writel(reg, pad_base + reg_offset);

	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_usb_pad_reg_update);

u32 tegra_usb_pad_reg_read(u32 reg_offset)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&xusb_padctl_lock, flags);
	reg = readl(pad_base + reg_offset);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);

	return reg;
}
EXPORT_SYMBOL_GPL(tegra_usb_pad_reg_read);

void tegra_usb_pad_reg_write(u32 reg_offset, u32 val)
{
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	unsigned long flags;
	spin_lock_irqsave(&xusb_padctl_lock, flags);
	writel(val, pad_base + reg_offset);
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
}
EXPORT_SYMBOL_GPL(tegra_usb_pad_reg_write);

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
static int tegra_xusb_padctl_phy_enable(void)
{
	unsigned long val, timeout;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	void __iomem *clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

	/* Enable overrides to enable SW control over PLL */
	/* init UPHY, Set PWR/CAL/RCAL OVRD */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_PWR_OVRD;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_OVRD;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_OVRD;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);

	/* Select REFCLK, TXCLKREF and Enable */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL4_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL4_PLL0_REFCLK_SEL_MASK;
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL4_PLL0_TXCLKREF_SEL_MASK;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL4_PLL0_TXCLKREF_SEL;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL4_PLL0_TXCLKREF_EN;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL4_0);

	/* FREQ_MDIV & FREQ_NDIV programming for 500 MHz */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_FREQ_MDIV_MASK;
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_FREQ_NDIV_MASK;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_FREQ_NDIV;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);

	/* Clear PLL0 iddq and sleep */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_IDDQ;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_SLEEP;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	udelay(1);

	/* Perform PLL calibration */
	/* Set PLL0_CAL_EN and wait for PLL0_CAL_DONE being set */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_EN;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	timeout = 20;
	do {
		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
		udelay(10);
		if (--timeout == 0) {
			pr_err("PCIe error: timeout for PLL0_CAL_DONE set\n");
			return -EBUSY;
		}
	} while (!(val & XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_DONE));

	/* Clear PLL0_CAL_EN and wait for PLL0_CAL_DONE being cleared */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_EN;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	timeout = 20;
	do {
		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
		udelay(10);
		if (--timeout == 0) {
			pr_err("PCIe error: timeout for PLL0_CAL_DONE cleared\n");
			return -EBUSY;
		}
	} while (val & XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_DONE);

	/* Set PLL0_ENABLE */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_ENABLE;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);

	/* Wait for the PLL to lock */
	timeout = 20;
	do {
		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
		udelay(10);
		if (--timeout == 0) {
			pr_err("Tegra PCIe error: timeout waiting for PLL\n");
			return -EBUSY;
		}
	} while (!(val & XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_LOCKDET_STATUS));

	/* Perform register calibration */
	/* Set PLL0_RCAL_EN and wait for PLL0_RCAL_DONE being set */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_OVRD;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_EN;
	val |= XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_CLK_EN;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	timeout = 10;
	do {
		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
		udelay(1);
		if (--timeout == 0) {
			pr_err("PCIe error: timeout for PLL0_RCAL_DONE set\n");
			return -EBUSY;
		}
	} while (!(val & XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_DONE));

	/* Clear PLL0_RCAL_EN and wait for PLL0_RCAL_DONE being cleared */
	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_EN;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	timeout = 10;
	do {
		val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
		udelay(1);
		if (--timeout == 0) {
			pr_err("PCIe error: timeout for PLL0_RCAL_DONE cleared\n");
			return -EBUSY;
		}
	} while (val & XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_DONE);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_CLK_EN;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);

	/* Enable PEX PADPLL clocks */
	val = readl(clk_base + CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0);
	val &= ~XUSBIO_CLK_ENABLE_SWCTL;
	val &= ~XUSBIO_PADPLL_RESET_SWCTL;
	val |= XUSBIO_PADPLL_USE_LOCKDET | XUSBIO_PADPLL_SLEEP_IDDQ;
	writel(val, clk_base + CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL1_PLL0_PWR_OVRD;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL1_0);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL2_PLL0_CAL_OVRD;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL2_0);

	val = readl(pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);
	val &= ~XUSB_PADCTL_UPHY_PLL_P0_CTL8_PLL0_RCAL_OVRD;
	writel(val, pad_base + XUSB_PADCTL_UPHY_PLL_P0_CTL8_0);

	udelay(1);
	val = readl(clk_base + CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0);
	val |= XUSBIO_SEQ_ENABLE;
	writel(val, clk_base + CLK_RST_CONTROLLER_XUSBIO_PLL_CFG0_0);

	return 0;
}
#else
static int tegra_xusb_padctl_phy_enable(void)
{
	unsigned long val, timeout;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

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
#ifdef CONFIG_ARCH_TEGRA_13x_SOC
	/* recommended prod setting */
	val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0);
	val &= ~XUSB_PADCTL_IOPHY_PLL_P0_CTL2_PLL0_CP_CNTL_MASK;
	val |= XUSB_PADCTL_IOPHY_PLL_P0_CTL2_PLL0_CP_CNTL_VAL;
	writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL2_0);
#endif
	/* take PLL out of reset */
	val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);
	val |= XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL_RST_;
	writel(val, pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);

	/* Wait for the PLL to lock */
	timeout = 300;
	do {
		val = readl(pad_base + XUSB_PADCTL_IOPHY_PLL_P0_CTL1_0);
		udelay(100);
		if (--timeout == 0) {
			pr_err("Tegra PCIe error: timeout waiting for PLL\n");
			return -EBUSY;
		}
	} while (!(val & XUSB_PADCTL_IOPHY_PLL_P0_CTL1_PLL0_LOCKDET));

	return 0;
}
#endif

static int tegra_pcie_lane_iddq(bool enable, int lane_owner)
{
	unsigned long val;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);

	/* disable IDDQ for all lanes based on odmdata */
	/* in same way as for lane ownership done below */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	switch (lane_owner) {
		case PCIE_LANES_X4_X1:
		if (enable)
			val |=
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK0;
		else
			val &=
			~XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK0;
		case PCIE_LANES_X4_X0:
		if (enable)
			val |=
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK1;
		else
			val &=
			~XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK1;
		case PCIE_LANES_X2_X1:
		if (enable)
			val |=
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK2 |
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK3 |
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK4;
		else
			val &=
			~(XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK2 |
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK3 |
			XUSB_PADCTL_USB3_PAD_MUX_FORCE_PCIE_PAD_IDDQ_DISABLE_MASK4);
		break;
		default:
			pr_err("Tegra PCIe error: wrong lane config\n");
			return -ENXIO;
	}
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	return 0;
}

int pcie_phy_pad_enable(bool enable, int lane_owner)
{
	unsigned long val, flags;
	void __iomem *pad_base = IO_ADDRESS(TEGRA_XUSB_PADCTL_BASE);
	int ret = 0;

	spin_lock_irqsave(&xusb_padctl_lock, flags);

	if (enable) {
		ret = tegra_xusb_padctl_phy_enable();
		if (ret)
			goto exit;
	}
	ret = tegra_pcie_lane_iddq(enable, lane_owner);
	if (ret || !enable) {
		if (!enable)
			tegra_xusb_uphy_misc(true);
		goto exit;
	}

	/* clear AUX_MUX_LP0 related bits in ELPG_PROGRAM */
	val = readl(pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	udelay(1);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_CLAMP_EN_EARLY;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	udelay(100);
	val &= ~XUSB_PADCTL_ELPG_PROGRAM_AUX_MUX_LP0_VCORE_DOWN;
	writel(val, pad_base + XUSB_PADCTL_ELPG_PROGRAM_0);
	udelay(100);

	tegra_xusb_uphy_misc(true);
	/* program ownership of all lanes based on odmdata as below */
	/* odm = 0x0, all 5 lanes owner = PCIe */
	/* odm = 0x2, 1st lane owner = USB3 else all lanes owner = PCIe */
	/* odm = 0x3, 1st 2 lane owner = USB3 else all lanes owner = PCIe */
	val = readl(pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	switch (lane_owner) {
		case PCIE_LANES_X4_X1:
			val &= ~XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE0;
		case PCIE_LANES_X4_X0:
			val &= ~XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE1;
		case PCIE_LANES_X2_X1:
			val &= ~(XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE2 |
				XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE3 |
				XUSB_PADCTL_USB3_PAD_MUX_PCIE_PAD_LANE4);
			break;
		default:
			pr_err("Tegra PCIe error: wrong lane config\n");
			ret = -ENXIO;
			goto exit;
	}
	writel(val, pad_base + XUSB_PADCTL_USB3_PAD_MUX_0);
	tegra_xusb_uphy_misc(false);
exit:
	spin_unlock_irqrestore(&xusb_padctl_lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(pcie_phy_pad_enable);
