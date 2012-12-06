/*
 * drivers/powergate/tegra-powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/clk/tegra.h>
#include <trace/events/power.h>
#include <asm/atomic.h>

#include <mach/powergate.h>

#include "clock.h"
#include "fuse.h"
#include "iomap.h"

#if defined(DEBUG_T11x_POWERGATE)
static void test_powergate_parts(void);
#endif
#if defined(DEBUG_T11x_POWERUNGATE)
static void test_powerungate_parts(void);
#endif
#if defined(DEBUG_T11x_POWERGATE_CLK_OFF)
static void test_powergate_clk_off_parts(void);
#endif
#if defined(DEBUG_T11x_POWERUNGATE_CLK_OFF)
static void test_unpowergate_clk_on_parts(void);
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static int tegra11x_check_partition_pug_seq(int id);
static int tegra11x_unpowergate(int id);
#endif

#define PWRGATE_TOGGLE		0x30
#define PWRGATE_TOGGLE_START	(1 << 8)

#define REMOVE_CLAMPING		0x34

#define PWRGATE_STATUS		0x38

#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
enum mc_client {
	MC_CLIENT_AFI		= 0,
	MC_CLIENT_AVPC		= 1,
	MC_CLIENT_DC		= 2,
	MC_CLIENT_DCB		= 3,
	MC_CLIENT_EPP		= 4,
	MC_CLIENT_G2		= 5,
	MC_CLIENT_HC		= 6,
	MC_CLIENT_HDA		= 7,
	MC_CLIENT_ISP		= 8,
	MC_CLIENT_MPCORE	= 9,
	MC_CLIENT_MPCORELP	= 10,
	MC_CLIENT_MPE		= 11,
	MC_CLIENT_NV		= 12,
	MC_CLIENT_NV2		= 13,
	MC_CLIENT_PPCS		= 14,
	MC_CLIENT_SATA		= 15,
	MC_CLIENT_VDE		= 16,
	MC_CLIENT_VI		= 17,
	MC_CLIENT_LAST		= -1,
};
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
enum mc_client {
	MC_CLIENT_AVPC		= 0,
	MC_CLIENT_DC		= 1,
	MC_CLIENT_DCB		= 2,
	MC_CLIENT_EPP		= 3,
	MC_CLIENT_G2		= 4,
	MC_CLIENT_HC		= 5,
	MC_CLIENT_ISP		= 6,
	MC_CLIENT_MPCORE	= 7,
	MC_CLIENT_MPEA		= 8,
	MC_CLIENT_MPEB		= 9,
	MC_CLIENT_MPEC		= 10,
	MC_CLIENT_NV		= 11,
	MC_CLIENT_PPCS		= 12,
	MC_CLIENT_VDE		= 13,
	MC_CLIENT_VI		= 14,
	MC_CLIENT_LAST		= -1,
	MC_CLIENT_AFI		= MC_CLIENT_LAST,
};
#else
/* bit positions are specific to chip */
enum mc_client {
	MC_CLIENT_AVPC		= 1,
	MC_CLIENT_DC		= 2,
	MC_CLIENT_DCB		= 3,
	MC_CLIENT_EPP		= 4,
	MC_CLIENT_G2		= 5,
	MC_CLIENT_HC		= 6,
	MC_CLIENT_HDA		= 7,
	MC_CLIENT_ISP		= 8,
	MC_CLIENT_MPCORE	= 9,
	MC_CLIENT_MPCORELP	= 10,
	MC_CLIENT_MSENC		= 11,
	MC_CLIENT_NV		= 12,
	MC_CLIENT_PPCS		= 14,
	MC_CLIENT_VDE		= 16,
	MC_CLIENT_VI		= 17,
	MC_CLIENT_XUSB_HOST	= 19,
	MC_CLIENT_XUSB_DEV	= 20,
	MC_CLIENT_EMUCIF	= 21,
	MC_CLIENT_TSEC		= 22,
	MC_CLIENT_LAST		= -1,
	MC_CLIENT_AFI		= MC_CLIENT_LAST,
	MC_CLIENT_MPE		= MC_CLIENT_LAST,
	MC_CLIENT_NV2		= MC_CLIENT_LAST,
	MC_CLIENT_SATA		= MC_CLIENT_LAST,
};
#endif

#define MAX_CLK_EN_NUM			9

static int tegra_num_powerdomains;
static int tegra_num_cpu_domains;
static u8 *tegra_cpu_domains;
static u8 tegra_quad_cpu_domains[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static DEFINE_SPINLOCK(tegra_powergate_lock);

#define MAX_HOTRESET_CLIENT_NUM		4

enum clk_type {
	CLK_AND_RST,
	RST_ONLY,
	CLK_ONLY,
};

struct partition_clk_info {
	const char *clk_name;
	enum clk_type clk_type;
	/* true if clk is only used in assert/deassert reset and not while enable-den*/
	struct clk *clk_ptr;
};

struct powergate_partition {
	const char *name;
	enum mc_client hot_reset_clients[MAX_HOTRESET_CLIENT_NUM];
	struct partition_clk_info clk_info[MAX_CLK_EN_NUM];
};

static struct powergate_partition powergate_partition_info[TEGRA_NUM_POWERGATE] = {
	[TEGRA_POWERGATE_CPU]	= { "cpu0",	{MC_CLIENT_LAST}, },
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
	[TEGRA_POWERGATE_L2]	= { "l2",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_3D]	= { "3d0",
#else
	[TEGRA_POWERGATE_3D]	= { "3d",
#endif
						{MC_CLIENT_NV, MC_CLIENT_LAST},
						{{"3d", CLK_AND_RST} }, },
/* T11x does not have pcie */
#if !defined(CONFIG_ARCH_TEGRA_11x_SOC)
#ifdef CONFIG_ARCH_TEGRA_HAS_PCIE
	[TEGRA_POWERGATE_PCIE]	= { "pcie",
						{MC_CLIENT_AFI, MC_CLIENT_LAST},
						{{"afi", CLK_AND_RST},
						{"pcie", CLK_AND_RST},
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
						{"cml0", CLK_ONLY},
#endif
						{"pciex", RST_ONLY} }, },
#endif
#endif
	[TEGRA_POWERGATE_VDEC]	= { "vde",
						{MC_CLIENT_VDE, MC_CLIENT_LAST},
						{{"vde", CLK_AND_RST} }, },
	[TEGRA_POWERGATE_MPE]	= { "mpe",
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
						{MC_CLIENT_MPE, MC_CLIENT_LAST},
						{{"mpe.cbus", CLK_AND_RST}, },
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
						{MC_CLIENT_MPEA, MC_CLIENT_MPEB,
						 MC_CLIENT_MPEC, MC_CLIENT_LAST},
						{{"mpe", CLK_AND_RST}, },
#else
						{MC_CLIENT_MSENC,
						 MC_CLIENT_LAST},
						{{"msenc.cbus", CLK_AND_RST}, },
#endif
				},
	[TEGRA_POWERGATE_VENC]	= { "ve",
					{
						MC_CLIENT_ISP,
						MC_CLIENT_VI,
						MC_CLIENT_LAST
					},
					{
						{"isp", CLK_AND_RST},
						{"vi", CLK_AND_RST},
						{"csi", CLK_AND_RST}
					},
				},
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC)
	[TEGRA_POWERGATE_CPU1]	= { "cpu1",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_CPU2]	= { "cpu2",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_CPU3]	= { "cpu3",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_CELP]	= { "celp",	{MC_CLIENT_LAST}, },
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
#ifdef CONFIG_ARCH_TEGRA_HAS_SATA
	[TEGRA_POWERGATE_SATA]	= { "sata",     {MC_CLIENT_SATA, MC_CLIENT_LAST},
						{{"sata", CLK_AND_RST},
						{"sata_oob", CLK_AND_RST},
						{"cml1", CLK_ONLY},
						{"sata_cold", RST_ONLY} }, },
#endif
#ifdef CONFIG_ARCH_TEGRA_HAS_DUAL_3D
	[TEGRA_POWERGATE_3D1]	= { "3d1",
						{MC_CLIENT_NV2, MC_CLIENT_LAST},
						{{"3d2", CLK_AND_RST} }, },
#endif
#endif
	[TEGRA_POWERGATE_HEG]	= { "heg",
					{
						MC_CLIENT_G2,
						MC_CLIENT_EPP,
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
						MC_CLIENT_HC,
#endif
						MC_CLIENT_LAST
					},
					{
#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
						{"2d", CLK_AND_RST},
						{"epp", CLK_AND_RST},
						{"host1x", CLK_AND_RST},
#elif defined(CONFIG_ARCH_TEGRA_3x_SOC)
						{"2d.cbus", CLK_AND_RST},
						{"epp.cbus", CLK_AND_RST},
						{"host1x.cbus", CLK_AND_RST},
#else
						{"2d.cbus", CLK_AND_RST},
						{"epp.cbus", CLK_AND_RST},
#endif
					},
				},
#endif
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	[TEGRA_POWERGATE_CRAIL]	= { "crail",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_C0NC]	= { "c0nc",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_C1NC]	= { "c1nc",	{MC_CLIENT_LAST}, },
	[TEGRA_POWERGATE_DISA]	= { "disa",
					{
						MC_CLIENT_DC,
						MC_CLIENT_LAST
					},
					{
						{"disp1", CLK_AND_RST},
						{"dsia", CLK_AND_RST},
						{"dsib", CLK_AND_RST},
						{"csi", CLK_AND_RST},
						{"mipi-cal", CLK_AND_RST}
					},
				},
	[TEGRA_POWERGATE_DISB]	= { "disb",
					{
						MC_CLIENT_DCB,
						MC_CLIENT_LAST
					},
					{
						{"disp2", CLK_AND_RST},
						{"hdmi", CLK_AND_RST}
					},
				},
	[TEGRA_POWERGATE_XUSBA]	= { "xusba",
					{ MC_CLIENT_LAST },
					{
						{"xusb_ss", CLK_AND_RST}
					},
				},
	[TEGRA_POWERGATE_XUSBB]	= { "xusbb",
					{
						MC_CLIENT_XUSB_DEV,
						MC_CLIENT_LAST
					},
					{
						{"xusb_dev", CLK_AND_RST},
					},
				},
	[TEGRA_POWERGATE_XUSBC]	= { "xusbc",
					{
						MC_CLIENT_XUSB_HOST,
						MC_CLIENT_LAST
					},
					{
						{"xusb_host", CLK_AND_RST},
					},
				},
#endif

};

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static void __iomem *mipi_cal = IO_ADDRESS(TEGRA_MIPI_CAL_BASE);

static u32 mipi_cal_read(unsigned long reg)
{
	return readl(mipi_cal + reg);
}

static void mipi_cal_write(u32 val, unsigned long reg)
{
	writel(val, mipi_cal + reg);
}

static void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

static u32 clk_rst_read(unsigned long reg)
{
	return readl(clk_rst + reg);
}
#endif

static u32 pmc_read(unsigned long reg)
{
	return readl(pmc + reg);
}

static void pmc_write(u32 val, unsigned long reg)
{
	writel(val, pmc + reg);
}

static void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

static u32 mc_read(unsigned long reg)
{
	return readl(mc + reg);
}

static void mc_write(u32 val, unsigned long reg)
{
	writel(val, mc + reg);
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_TEGRA_SIMULATION_PLATFORM)

#define MC_CLIENT_HOTRESET_CTRL	0x200
#define MC_CLIENT_HOTRESET_STAT	0x204

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC)
/* FIXME: this is sw workaround for unstable hotreset status
 * for T11x.
 */
#define HOTRESET_READ_COUNT 5
static bool tegra11x_stable_hotreset_check(u32 *stat)
{
	int i;
	u32 cur_stat;
	u32 prv_stat;
	unsigned long flags;

	spin_lock_irqsave(&tegra_powergate_lock, flags);
	prv_stat = mc_read(MC_CLIENT_HOTRESET_STAT);
	for (i = 0; i < HOTRESET_READ_COUNT; i++) {
		cur_stat = mc_read(MC_CLIENT_HOTRESET_STAT);
		if (cur_stat != prv_stat) {
			spin_unlock_irqrestore(&tegra_powergate_lock, flags);
			return false;
		}
	}
	*stat = cur_stat;
	spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	return true;
}
#endif

static void mc_flush(int id)
{
	u32 idx, rst_ctrl, rst_stat;
	enum mc_client mcClientBit;
	unsigned long flags;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && \
	!defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool ret;
#endif

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);
		rst_ctrl = mc_read(MC_CLIENT_HOTRESET_CTRL);
		rst_ctrl |= (1 << mcClientBit);
		mc_write(rst_ctrl, MC_CLIENT_HOTRESET_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);

		do {
			udelay(10);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
			rst_stat = mc_read(MC_CLIENT_HOTRESET_STAT);
#else
			rst_stat = 0;
			ret = tegra11x_stable_hotreset_check(&rst_stat);
			if (!ret)
				continue;
#endif
		} while (!(rst_stat & (1 << mcClientBit)));
	}
}

static void mc_flush_done(int id)
{
	u32 idx, rst_ctrl;
	enum mc_client mcClientBit;
	unsigned long flags;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		rst_ctrl = mc_read(MC_CLIENT_HOTRESET_CTRL);
		rst_ctrl &= ~(1 << mcClientBit);
		mc_write(rst_ctrl, MC_CLIENT_HOTRESET_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}

	wmb();
}

int tegra_powergate_mc_flush(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;
	mc_flush(id);
	return 0;
}

int tegra_powergate_mc_flush_done(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return -EINVAL;
	mc_flush_done(id);
	return 0;
}

int tegra_powergate_mc_disable(int id)
{
	return 0;
}

int tegra_powergate_mc_enable(int id)
{
	return 0;
}

#else

#define MC_CLIENT_CTRL		0x100
#define MC_CLIENT_HOTRESETN	0x104
#define MC_CLIENT_ORRC_BASE	0x140

int tegra_powergate_mc_disable(int id)
{
	u32 idx, clt_ctrl, orrc_reg;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* clear client enable bit */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl &= ~(1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);

		/* wait for outstanding requests to reach 0 */
		orrc_reg = MC_CLIENT_ORRC_BASE + (mcClientBit * 4);
		while (mc_read(orrc_reg) != 0)
			udelay(10);
	}
	return 0;
}

int tegra_powergate_mc_flush(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* assert hotreset (client module is currently in reset) */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn &= ~(1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}
	return 0;
}

int tegra_powergate_mc_flush_done(int id)
{
	u32 idx, hot_rstn;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* deassert hotreset */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);
		hot_rstn |= (1 << mcClientBit);
		mc_write(hot_rstn, MC_CLIENT_HOTRESETN);

		/* read back to flush write */
		hot_rstn = mc_read(MC_CLIENT_HOTRESETN);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}
	return 0;
}

int tegra_powergate_mc_enable(int id)
{
	u32 idx, clt_ctrl;
	enum mc_client mcClientBit;
	unsigned long flags;

	if (id < 0 || id >= TEGRA_NUM_POWERGATE) {
		WARN_ON(1);
		return -EINVAL;
	}

	for (idx = 0; idx < MAX_HOTRESET_CLIENT_NUM; idx++) {
		mcClientBit =
			powergate_partition_info[id].hot_reset_clients[idx];
		if (mcClientBit == MC_CLIENT_LAST)
			break;

		spin_lock_irqsave(&tegra_powergate_lock, flags);

		/* enable client */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);
		clt_ctrl |= (1 << mcClientBit);
		mc_write(clt_ctrl, MC_CLIENT_CTRL);

		/* read back to flush write */
		clt_ctrl = mc_read(MC_CLIENT_CTRL);

		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
	}
	return 0;
}

static void mc_flush(int id) {}
static void mc_flush_done(int id) {}
#endif

static int tegra_powergate_set(int id, bool new_state)
{
#ifndef CONFIG_TEGRA_SIMULATION_PLATFORM
	bool status;
	unsigned long flags;
	/* 10us timeout for toggle operation if it takes affect*/
	int toggle_timeout = 10;
	/* 100 * 10 = 1000us timeout for toggle command to take affect in case
	   of contention with h/w initiated CPU power gating */
	int contention_timeout = 100;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	if (TEGRA_IS_CPU_POWERGATE_ID(id)) {
		/* CPU ungated in s/w only during boot/resume with outer
		   waiting loop and no contention from other CPUs */
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	do {
		pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
		do {
			udelay(1);
			status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

			toggle_timeout--;
		} while ((status != new_state) && (toggle_timeout > 0));

		contention_timeout--;
	} while ((status != new_state) && (contention_timeout > 0));

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	if (status != new_state) {
		WARN(1, "Could not set powergate %d to %d", id, new_state);
		return -EBUSY;
	}

	trace_power_domain_target(powergate_partition_info[id].name, new_state,
			smp_processor_id());
#endif

	return 0;
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static bool tegra11x_check_plld_plld2_disable(void)
{
	/* FIXME:
	 * add check for plld and plld2 disable
	 */
#define CLK_RST_CONTROLLER_PLLD_BASE_0 0xd0
#define CLK_RST_CONTROLLER_PLLD_BASE_0_PLLD_ENABLE_LSB 30
#define CLK_RST_CONTROLLER_PLLD2_BASE_0 0x4b8
#define CLK_RST_CONTROLLER_PLLD2_BASE_0_PLLD2_ENABLE_LSB 30
	u32 status;
	status = clk_rst_read(CLK_RST_CONTROLLER_PLLD_BASE_0);
	if (status & (1 << CLK_RST_CONTROLLER_PLLD_BASE_0_PLLD_ENABLE_LSB))
		return false;
	status = clk_rst_read(CLK_RST_CONTROLLER_PLLD2_BASE_0);
	if (status & (1 << CLK_RST_CONTROLLER_PLLD2_BASE_0_PLLD2_ENABLE_LSB))
		return false;
	return true;
}

static bool tegra11x_pg_sw_war_missing(int id)
{
	bool ret;

	switch (id) {
	case TEGRA_POWERGATE_DISA:
		/* FIXME:
		 * [SW WAR bug 954988]:
		 * Disable PLLD and PLLD2 by clearing bits:
a.	CLK_RST_CONTROLLER_PLLD_BASE_0_PLLD_ENABLE
b.	CLK_RST_CONTROLLER_PLLD2_BASE_0_PLLD2_ENABLE
		 * We should not need to disable PLLD and PLLD2
		 * for linux/android implementation
		 * adding check in case PLLD or PLLD2 is/are ON
		 */
		ret = tegra11x_check_plld_plld2_disable();
		if (!ret)
			return true;

		break;
	}
	return false;
}
#endif

#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
static int unpowergate_module(int id)
{
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;
	return tegra_powergate_set(id, true);
}
#endif

static int powergate_module(int id)
{
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool need_sw_war;
#endif
	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	mc_flush(id);
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	need_sw_war = tegra11x_pg_sw_war_missing(id);
	if (need_sw_war) {
		pr_err("Error: missing powergate sw war in file: %s, func: %s, line=%d\n",
		__FILE__, __func__, __LINE__);
		return -1;
	}
#endif
	return tegra_powergate_set(id, false);
}

bool tegra_powergate_is_powered(int id)
{
	u32 status;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	status = pmc_read(PWRGATE_STATUS) & (1 << id);
	return !!status;
}
EXPORT_SYMBOL(tegra_powergate_is_powered);

int tegra_powergate_remove_clamping(int id)
{
	u32 mask;
	int contention_timeout = 100;

	if (id < 0 || id >= tegra_num_powerdomains)
		return -EINVAL;

	/*
	 * PCIE and VDE clamping masks are swapped with respect to their
	 * partition ids
	 */
	if (id ==  TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if (id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	pmc_write(mask, REMOVE_CLAMPING);
	/* Wait until clamp is removed */
	do {
		udelay(1);
		contention_timeout--;
	} while ((contention_timeout > 0)
			&& (pmc_read(REMOVE_CLAMPING) & mask));

	WARN(contention_timeout <= 0, "Couldn't remove clamping");

	return 0;
}

static void get_clk_info(int id)
{
	int idx;

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		if (!powergate_partition_info[id].clk_info[idx].clk_name)
			break;
		powergate_partition_info[id].
				clk_info[idx].clk_ptr =
					tegra_get_clock_by_name(
			powergate_partition_info[id].clk_info[idx].clk_name);
	}
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static bool tegra11x_pug_clk_n_rst_skip(int id, u32 idx)
{
	switch (id) {
	case TEGRA_POWERGATE_VENC:
		if ((powergate_partition_info[id].clk_info[idx].clk_name) &&
			(!(strncmp("csi",
			powergate_partition_info[id].clk_info[idx].clk_name,
			3)))) {
				/* DIS powered ON then do clk enable CSI */
				if (!tegra_powergate_is_powered(
						TEGRA_POWERGATE_DISA))
					return true;
		}
		break;
	case TEGRA_POWERGATE_DISA:
		if ((powergate_partition_info[id].clk_info[idx].clk_name) &&
			(!(strncmp("csi",
			powergate_partition_info[id].clk_info[idx].clk_name,
			3)))) {
				/* DIS powered ON then do clk enable CSI */
				if (!tegra_powergate_is_powered(
						TEGRA_POWERGATE_VENC))
					return true;
		}
		break;
	}
	return false;
}
#endif

static int partition_clk_enable(int id)
{
	int ret;
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool skip_enable;
#endif

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
		skip_enable = tegra11x_pug_clk_n_rst_skip(id, idx);
		if (skip_enable)
			continue;
#endif
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (clk_info->clk_type != RST_ONLY) {
			ret = tegra_clk_prepare_enable(clk);
			if (ret)
				goto err_clk_en;
		}
	}

	return 0;

err_clk_en:
	WARN(1, "Could not enable clk %s, error %d", clk->name, ret);
	while (idx--) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		if (clk_info->clk_type != RST_ONLY)
			tegra_clk_disable_unprepare(clk_info->clk_ptr);
	}

	return ret;
}

static int is_partition_clk_disabled(int id)
{
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;
	int ret = 0;

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (clk_info->clk_type != RST_ONLY) {
			if (tegra_is_clk_enabled(clk)) {
				ret = -1;
				break;
			}
		}
	}

	return ret;
}

static void partition_clk_disable(int id)
{
	u32 idx;
	struct clk *clk;
	struct partition_clk_info *clk_info;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool skip_disable;
#endif

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
		if (id == TEGRA_POWERGATE_DISA) {
			skip_disable = tegra11x_pug_clk_n_rst_skip(id, idx);
			if (skip_disable)
				continue;
		}
#endif
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk = clk_info->clk_ptr;
		if (!clk)
			break;

		if (clk_info->clk_type != RST_ONLY)
			tegra_clk_disable_unprepare(clk);
	}
}

static void powergate_partition_assert_reset(int id)
{
	u32 idx;
	struct clk *clk_ptr;
	struct partition_clk_info *clk_info;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool skip_reset;
#endif

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
		if (id == TEGRA_POWERGATE_DISA) {
			skip_reset = tegra11x_pug_clk_n_rst_skip(id, idx);
			if (skip_reset)
				continue;
		}
#endif
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk_ptr = clk_info->clk_ptr;
		if (!clk_ptr)
			break;
		if (clk_info->clk_type != CLK_ONLY)
			tegra_periph_reset_assert(clk_ptr);
	}
}

static void powergate_partition_deassert_reset(int id)
{
	u32 idx;
	struct clk *clk_ptr;
	struct partition_clk_info *clk_info;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool skip_reset;
#endif

	BUG_ON(id < 0 || id >= TEGRA_NUM_POWERGATE);

	for (idx = 0; idx < MAX_CLK_EN_NUM; idx++) {
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
		skip_reset = tegra11x_pug_clk_n_rst_skip(id, idx);
		if (skip_reset)
			continue;
#endif
		clk_info = &powergate_partition_info[id].clk_info[idx];
		clk_ptr = clk_info->clk_ptr;
		if (!clk_ptr)
			break;
		if (clk_info->clk_type != CLK_ONLY)
			tegra_periph_reset_deassert(clk_ptr);
	}
}

/* Must be called with clk disabled, and returns with clk disabled */
static int tegra_powergate_reset_module(int id)
{
	int ret;

	powergate_partition_assert_reset(id);

	udelay(10);

	ret = partition_clk_enable(id);
	if (ret)
		return ret;

	udelay(10);

	powergate_partition_deassert_reset(id);

	partition_clk_disable(id);

	return 0;
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
/*
 * FIXME: sw war for mipi-cal calibration when unpowergating DISA partition
 */
static void tegra11x_mipical_calibrate(int id)
{
	struct reg_offset_val {
		u32 offset;
		u32 por_value;
	};
	u32 status;
	unsigned long flags;
#define MIPI_CAL_MIPI_CAL_CTRL_0 0x0
#define MIPI_CAL_CIL_MIPI_CAL_STATUS_0 0x8
#define MIPI_CAL_CILA_MIPI_CAL_CONFIG_0 0x14
#define MIPI_CAL_CILB_MIPI_CAL_CONFIG_0 0x18
#define MIPI_CAL_CILC_MIPI_CAL_CONFIG_0 0x1c
#define MIPI_CAL_CILD_MIPI_CAL_CONFIG_0 0x20
#define MIPI_CAL_CILE_MIPI_CAL_CONFIG_0 0x24
#define MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0 0x38
#define MIPI_CAL_DSIB_MIPI_CAL_CONFIG_0 0x3c
#define MIPI_CAL_DSIC_MIPI_CAL_CONFIG_0 0x40
#define MIPI_CAL_DSID_MIPI_CAL_CONFIG_0 0x44
	static struct reg_offset_val mipi_cal_por_values[] = {
		{ MIPI_CAL_MIPI_CAL_CTRL_0, 0x2a000000 },
		{ MIPI_CAL_CILA_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_CILB_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_CILC_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_CILD_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_CILE_MIPI_CAL_CONFIG_0, 0x00000000 },
		{ MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_DSIB_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_DSIC_MIPI_CAL_CONFIG_0, 0x00200000 },
		{ MIPI_CAL_DSID_MIPI_CAL_CONFIG_0, 0x00200000 },
	};
	int i;

	if (id != TEGRA_POWERGATE_DISA)
		return;
	spin_lock_irqsave(&tegra_powergate_lock, flags);
	/* mipi cal por restore */
	for (i = 0; i < ARRAY_SIZE(mipi_cal_por_values); i++) {
		mipi_cal_write(mipi_cal_por_values[i].por_value,
			mipi_cal_por_values[i].offset);
	}
	/* mipi cal status clear */
	status = mipi_cal_read(MIPI_CAL_CIL_MIPI_CAL_STATUS_0);
	mipi_cal_write(status, MIPI_CAL_CIL_MIPI_CAL_STATUS_0);
	/* mipi cal status read - to flush writes */
	status = mipi_cal_read(MIPI_CAL_CIL_MIPI_CAL_STATUS_0);
	spin_unlock_irqrestore(&tegra_powergate_lock, flags);
}
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static bool skip_pg_check(int id, bool is_unpowergate)
{
	/*
	 * FIXME: need to stress test partition power gating before
	 * enabling power gating for T11x
	 * List of T11x partition id which skip power gating
	 */
	static int skip_pg_t11x_list[] = {
		/*
		 * CPU and 3D partitions enable/disable
		 * is managed by respective modules
		 */
	};
	int i;

	/*
	 * skip unnecessary multiple calls e.g. powergate call when
	 * partition is already powered-off or vice-versa
	 */
	if ((tegra_powergate_is_powered(id) &&
		is_unpowergate) ||
		(!(tegra_powergate_is_powered(id)) &&
		(!is_unpowergate))) {
		pr_err("Partition %s already powered-%s and %spowergate skipped\n",
			tegra_powergate_get_name(id),
			(tegra_powergate_is_powered(id)) ?
			"on" : "off",
			(is_unpowergate) ? "un" : "");
		return true;
	}
	/* unpowergate is allowed for all partitions */
	if (!tegra_powergate_is_powered(id) &&
		is_unpowergate)
		return false;
	for (i = 0; i < ARRAY_SIZE(skip_pg_t11x_list); i++) {
		if (id == skip_pg_t11x_list[i]) {
			pr_err("Partition %s %spowergate skipped\n",
				tegra_powergate_get_name(id),
				(is_unpowergate) ? "un" : "");
			return true;
		}
	}

	return false;
}
#endif

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static atomic_t ref_count_a = ATOMIC_INIT(1); /* for TEGRA_POWERGATE_DISA */
static atomic_t ref_count_b = ATOMIC_INIT(1); /* for TEGRA_POWERGATE_DISB */
#endif

/*
 * Must be called with clk disabled, and returns with clk disabled
 * Drivers should enable clks for partition. Unpowergates only the
 * partition.
 */
int tegra_unpowergate_partition(int id)
{
	int ret;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool is_pg_skip;

	WARN_ONCE(atomic_read(&ref_count_a) < 0, "ref count A underflow");
	WARN_ONCE(atomic_read(&ref_count_b) < 0, "ref count B underflow");
	if (id == TEGRA_POWERGATE_DISA && atomic_inc_return(&ref_count_a) != 1)
		return 0;
	else if (id == TEGRA_POWERGATE_DISB &&
		atomic_inc_return(&ref_count_b) != 1)
		return 0;

	is_pg_skip = skip_pg_check(id, true);
	if (is_pg_skip)
		return 0;
	ret = tegra11x_check_partition_pug_seq(id);
	if (ret)
		return ret;

	ret = tegra11x_unpowergate(id);
	return ret;
#else
	/* FIXME: not changing previous chip's power-ungate implementation */

	/* If first clk_ptr is null, fill clk info for the partition */
	if (!powergate_partition_info[id].clk_info[0].clk_ptr)
		get_clk_info(id);

	if (tegra_powergate_is_powered(id))
		return tegra_powergate_reset_module(id);

	ret = unpowergate_module(id);
	if (ret)
		goto err_power;

	powergate_partition_assert_reset(id);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_enable(id);
	if (ret)
		goto err_clk_on;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);

	powergate_partition_deassert_reset(id);

	mc_flush_done(id);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	partition_clk_disable(id);

	return 0;

err_clamp:
	partition_clk_disable(id);
err_clk_on:
	powergate_module(id);
err_power:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
#endif
}

int tegra_cpu_powergate_id(int cpuid)
{
	if (cpuid > 0 && cpuid < tegra_num_cpu_domains)
		return tegra_cpu_domains[cpuid];

	return -EINVAL;
}

int __init tegra_powergate_init(void)
{
	switch (tegra_chip_id) {
	case TEGRA20:
		tegra_num_powerdomains = 7;
		break;
	case TEGRA30:
		tegra_num_powerdomains = 14;
		tegra_num_cpu_domains = 4;
		tegra_cpu_domains = tegra_quad_cpu_domains;
		break;
	case TEGRA11X:
		tegra_num_powerdomains = 23;
		tegra_num_cpu_domains = 4;
		tegra_cpu_domains = tegra_quad_cpu_domains;
		break;
	default:
		/* Unknown Tegra variant. Disable powergating */
		tegra_num_powerdomains = 0;
		break;
	}

#if defined(DEBUG_T11x_POWERGATE)
	test_powergate_parts();
#endif
#if defined(DEBUG_T11x_POWERUNGATE)
	test_unpowergate_parts();
#endif
#if defined(DEBUG_T11x_POWERGATE_CLK_OFF)
	test_powergate_clk_off_parts();
#endif
#if defined(DEBUG_T11x_POWERUNGATE_CLK_ON)
	test_unpowergate_clk_on_parts();
#endif
	return 0;
}

/*
 * Must be called with clk disabled, and returns with clk enabled
 * Unpowergates the partition and enables all required clks.
 */
int tegra_unpowergate_partition_with_clk_on(int id)
{
	int ret = 0;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool is_pg_skip;

	is_pg_skip = skip_pg_check(id, true);
	if (is_pg_skip)
		return 0;
#endif
#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
	/* Restrict this functions use to few partitions */
	BUG_ON(id != TEGRA_POWERGATE_SATA && id != TEGRA_POWERGATE_PCIE);
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
	/* Restrict this functions use to few partitions */
	BUG_ON(id != TEGRA_POWERGATE_PCIE);
#endif

	ret = tegra_unpowergate_partition(id);
	if (ret)
		goto err_unpowergating;

	/* Enable clks for the partition */
	ret = partition_clk_enable(id);
	if (ret)
		goto err_unpowergate_clk;

	return ret;

err_unpowergate_clk:
	tegra_powergate_partition(id);
	WARN(1, "Could not Un-Powergate %d, err in enabling clk", id);
err_unpowergating:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static int tegra11x_powergate_set(int id, bool new_state)
{
#ifndef CONFIG_TEGRA_SIMULATION_PLATFORM
	bool status;
	unsigned long flags;
	/* 10us timeout for toggle operation if it takes affect*/
	int toggle_timeout = 10;
	/* 100 * 10 = 1000us timeout for toggle command to take affect in case
	   of contention with h/w initiated CPU power gating */
	int contention_timeout = 100;

	spin_lock_irqsave(&tegra_powergate_lock, flags);

	status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

	if (status == new_state) {
		spin_unlock_irqrestore(&tegra_powergate_lock, flags);
		return 0;
	}

	pmc_write(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);
	do {
		do {
			udelay(1);
			status = !!(pmc_read(PWRGATE_STATUS) & (1 << id));

			toggle_timeout--;
		} while ((status != new_state) && (toggle_timeout > 0));

		contention_timeout--;
	} while ((status != new_state) && (contention_timeout > 0));

	spin_unlock_irqrestore(&tegra_powergate_lock, flags);

	if (status != new_state) {
		WARN(1, "Could not set powergate %d to %d", id, new_state);
		return -EBUSY;
	}

	trace_power_domain_target(powergate_partition_info[id].name, new_state,
			smp_processor_id());
#endif

	return 0;
}

static int tegra11x_powergate(int id)
{
	int ret;

	/* If first clk_ptr is null, fill clk info for the partition */
	if (powergate_partition_info[id].clk_info[0].clk_ptr)
		get_clk_info(id);

	ret = partition_clk_enable(id);
	if (ret)
		WARN(1, "Couldn't enable clock");

	udelay(10);

	mc_flush(id);

	udelay(10);

	powergate_partition_assert_reset(id);

	udelay(10);

	/* Powergating is done only if refcnt of all clks is 0 */
	partition_clk_disable(id);

	udelay(10);

	ret = tegra11x_powergate_set(id, false);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	WARN(1, "Could not Powergate Partition %d", id);
	return ret;
}

static int tegra11x_unpowergate(int id)
{
	int ret;
	/* If first clk_ptr is null, fill clk info for the partition */
	if (!powergate_partition_info[id].clk_info[0].clk_ptr)
		get_clk_info(id);

	if (tegra_powergate_is_powered(id))
		return tegra_powergate_reset_module(id);

	ret = tegra11x_powergate_set(id, true);
	if (ret)
		goto err_power;

	udelay(10);

	/* Un-Powergating fails if all clks are not enabled */
	ret = partition_clk_enable(id);
	if (ret)
		goto err_clk_on;

	udelay(10);

	ret = tegra_powergate_remove_clamping(id);
	if (ret)
		goto err_clamp;

	udelay(10);

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	tegra11x_mipical_calibrate(id);
#endif
	powergate_partition_deassert_reset(id);

	udelay(10);

	mc_flush_done(id);

	udelay(10);

	/* Disable all clks enabled earlier. Drivers should enable clks */
	partition_clk_disable(id);

	return 0;

err_clamp:
	partition_clk_disable(id);
err_clk_on:
	powergate_module(id);
err_power:
	WARN(1, "Could not Un-Powergate %d", id);
	return ret;
}

static int tegra11x_powergate_partition(int id)
{
	int ret;

	if (tegra_powergate_is_powered(id)) {
		ret = is_partition_clk_disabled(id);
		if (ret < 0) {
			/* clock enabled */
			ret = tegra_powergate_partition_with_clk_off(id);
			if (ret < 0)
				return ret;
		} else {
			ret = tegra_powergate_partition(id);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

static int tegra11x_unpowergate_partition(int id)
{
	int ret;

	if (!tegra_powergate_is_powered(id)) {
		ret = is_partition_clk_disabled(id);
		if (ret) {
			/* clock disabled */
			ret = tegra_unpowergate_partition_with_clk_on(id);
			if (ret < 0)
				return ret;
		} else {
			ret = tegra_unpowergate_partition(id);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

/*
 * Tegra11x has powergate dependencies between partitions.
 * This function captures the dependencies.
 */
static int tegra11x_check_partition_pg_seq(int id)
{
	int ret;

	switch (id) {
	case TEGRA_POWERGATE_DISA:
		ret = tegra11x_powergate_partition(TEGRA_POWERGATE_VENC);
		if (ret < 0)
			return ret;
		ret = tegra11x_powergate_partition(TEGRA_POWERGATE_DISB);
		if (ret < 0)
			return ret;
		break;
	}
	return 0;
}

/*
 * This function captures power-ungate dependencies between tegra11x partitions
 */
static int tegra11x_check_partition_pug_seq(int id)
{
	int ret;

	switch (id) {
	case TEGRA_POWERGATE_DISB:
	case TEGRA_POWERGATE_VENC:
		ret = tegra11x_unpowergate_partition(TEGRA_POWERGATE_DISA);
		if (ret < 0)
			return ret;

		break;
	}
	return 0;
}
#endif

/*
 * Must be called with clk disabled. Powergates the partition only
 */
int tegra_powergate_partition(int id)
{
	int ret;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool is_pg_skip;

	WARN_ONCE(atomic_read(&ref_count_a) < 0, "ref count A underflow");
	WARN_ONCE(atomic_read(&ref_count_b) < 0, "ref count B underflow");
	if (id == TEGRA_POWERGATE_DISA && atomic_dec_return(&ref_count_a) != 0)
		return 0;
	else if (id == TEGRA_POWERGATE_DISB &&
		atomic_dec_return(&ref_count_b) != 0)
		return 0;

	is_pg_skip = skip_pg_check(id, false);
	if (is_pg_skip)
		return 0;

	ret = tegra11x_check_partition_pg_seq(id);
	if (ret)
		return ret;

	/* All Tegra11x partition powergate */
	ret = tegra11x_powergate(id);
	return ret;
#else
	/* FIXME: not changing previous chip's powergate implementation */

	/* If first clk_ptr is null, fill clk info for the partition */
	if (powergate_partition_info[id].clk_info[0].clk_ptr)
		get_clk_info(id);

	powergate_partition_assert_reset(id);

	/* Powergating is done only if refcnt of all clks is 0 */
	ret = is_partition_clk_disabled(id);
	if (ret)
		goto err_clk_off;

	ret = powergate_module(id);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	WARN(1, "Could not Powergate Partition %d", id);
err_clk_off:
	WARN(1, "Could not Powergate Partition %d, all clks not disabled", id);
	return ret;
#endif
}

int tegra_powergate_partition_with_clk_off(int id)
{
	int ret = 0;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	bool is_pg_skip;

	is_pg_skip = skip_pg_check(id, false);
	if (is_pg_skip)
		return 0;
#endif
#if defined(CONFIG_ARCH_TEGRA_3x_SOC)
	/* Restrict functions use to selected partitions */
	BUG_ON(id != TEGRA_POWERGATE_PCIE && id != TEGRA_POWERGATE_SATA);
#elif defined(CONFIG_ARCH_TEGRA_2x_SOC)
	/* Restrict functions use to selected partitions */
	BUG_ON(id != TEGRA_POWERGATE_PCIE);
#endif
	/* Disable clks for the partition */
	partition_clk_disable(id);

	ret = is_partition_clk_disabled(id);
	if (ret)
		goto err_powergate_clk;

	ret = tegra_powergate_partition(id);
	if (ret)
		goto err_powergating;

	return ret;

err_powergate_clk:
	WARN(1, "Could not Powergate Partition %d, all clks not disabled", id);
err_powergating:
	partition_clk_enable(id);
	WARN(1, "Could not Powergate Partition %d", id);
	return ret;
}

#if defined(DEBUG_T11x_POWERGATE)
static void test_powergate_parts(void)
{
	int i;

	for (i = 0; i < TEGRA_NUM_POWERGATE; i++)
		tegra_powergate_partition(i);
}
#endif

#if defined(DEBUG_T11x_POWERUNGATE)
static void test_powerungate_parts(void)
{
	int i;

	for (i = 0; i < TEGRA_NUM_POWERGATE; i++)
		tegra_unpowergate_partition(i);
}
#endif

#if defined(DEBUG_T11x_POWERGATE_CLK_OFF)
static void test_powergate_clk_off_parts(void)
{
	int i;

	for (i = 0; i < TEGRA_NUM_POWERGATE; i++)
		tegra_powergate_partition_with_clk_off(i);
}
#endif

#if defined(DEBUG_T11x_POWERUNGATE_CLK_OFF)
static void test_unpowergate_clk_on_parts(void)
{
	int i;

	for (i = 0; i < TEGRA_NUM_POWERGATE; i++)
		tegra_unpowergate_partition_with_clk_on(i);
}
#endif

const char *tegra_powergate_get_name(int id)
{
	if (id < 0 || id >= TEGRA_NUM_POWERGATE)
		return "invalid";

	return powergate_partition_info[id].name;
}

#ifdef CONFIG_DEBUG_FS

static int powergate_show(struct seq_file *s, void *data)
{
	int i;
	const char *name;

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < tegra_num_powerdomains; i++) {
		name = tegra_powergate_get_name(i);
		if (name)
			seq_printf(s, " %9s %7s\n", name,
				tegra_powergate_is_powered(i) ? "yes" : "no");
	}

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int __init tegra_powergate_debugfs_init(void)
{
	struct dentry *d;

	if (powergate_name) {
		d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
			&powergate_fops);
		if (!d)
			return -ENOMEM;
	}

	return 0;
}

#endif
