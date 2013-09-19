/*
 * arch/arm/mach-tegra/mc.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011-2013, NVIDIA Corporation.  All rights reserved.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include <mach/mc.h>

#include "iomap.h"

static DEFINE_SPINLOCK(tegra_mc_lock);
static void __iomem *mc_base = (void __iomem *)IO_TO_VIRT(TEGRA_MC_BASE);
#define MC_CLIENT_HOTRESET_CTRL		0x200
#define MC_CLIENT_HOTRESET_STAT		0x204
#define MC_CLIENT_HOTRESET_CTRL_1	0x970
#define MC_CLIENT_HOTRESET_STAT_1	0x974

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)

void tegra_mc_set_priority(unsigned long client, unsigned long prio)
{
	unsigned long reg = client >> 8;
	int field = client & 0xff;
	unsigned long val;
	unsigned long flags;

	spin_lock_irqsave(&tegra_mc_lock, flags);
	val = readl(mc_base + reg);
	val &= ~(TEGRA_MC_PRIO_MASK << field);
	val |= prio << field;
	writel(val, mc_base + reg);
	spin_unlock_irqrestore(&tegra_mc_lock, flags);

}

int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	return 1;
}

#else
	/* !!!FIXME!!! IMPLEMENT tegra_mc_set_priority() */

#include "tegra3_emc.h"

/*
 * If using T30/DDR3, the 2nd 16 bytes part of DDR3 atom is 2nd line and is
 * discarded in tiling mode.
 */
int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	int type;

	type = tegra_emc_get_dram_type();
	/*WARN_ONCE(type == -1, "unknown type DRAM because DVFS is disabled\n");*/

	if (type == DRAM_TYPE_DDR3)
		return 2;
	else
		return 1;
}
#endif

/* API to get EMC freq to be requested, for Bandwidth.
 * bw_kbps: BandWidth passed is in KBps.
 * returns freq in KHz
 */
unsigned int tegra_emc_bw_to_freq_req(unsigned int bw_kbps)
{
	unsigned int freq;
	unsigned int bytes_per_emc_clk;

	bytes_per_emc_clk = tegra_mc_get_effective_bytes_width() * 2;
	freq = (bw_kbps + bytes_per_emc_clk - 1) / bytes_per_emc_clk *
		CONFIG_TEGRA_EMC_TO_DDR_CLOCK;
	return freq;
}
EXPORT_SYMBOL_GPL(tegra_emc_bw_to_freq_req);

/* API to get EMC bandwidth, for freq that can be requested.
 * freq_khz: Frequency passed is in KHz.
 * returns bandwidth in KBps
 */
unsigned int tegra_emc_freq_req_to_bw(unsigned int freq_khz)
{
	unsigned int bw;
	unsigned int bytes_per_emc_clk;

	bytes_per_emc_clk = tegra_mc_get_effective_bytes_width() * 2;
	bw = freq_khz * bytes_per_emc_clk / CONFIG_TEGRA_EMC_TO_DDR_CLOCK;
	return bw;
}
EXPORT_SYMBOL_GPL(tegra_emc_freq_req_to_bw);

#define HOTRESET_READ_COUNT	5
static bool tegra_stable_hotreset_check(u32 stat_reg, u32 *stat)
{
	int i;
	u32 cur_stat;
	u32 prv_stat;
	unsigned long flags;

	spin_lock_irqsave(&tegra_mc_lock, flags);
	prv_stat = readl(mc_base + stat_reg);
	for (i = 0; i < HOTRESET_READ_COUNT; i++) {
		cur_stat = readl(mc_base + stat_reg);
		if (cur_stat != prv_stat) {
			spin_unlock_irqrestore(&tegra_mc_lock, flags);
			return false;
		}
	}
	*stat = cur_stat;
	spin_unlock_irqrestore(&tegra_mc_lock, flags);
	return true;
}

int tegra_mc_flush(int id)
{
	u32 rst_ctrl, rst_stat;
	u32 rst_ctrl_reg, rst_stat_reg;
	unsigned long flags;
	bool ret;

	if (id < 32) {
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
	} else {
		id %= 32;
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
	}

	spin_lock_irqsave(&tegra_mc_lock, flags);

	rst_ctrl = readl(mc_base + rst_ctrl_reg);
	rst_ctrl |= (1 << id);
	writel(rst_ctrl, mc_base + rst_ctrl_reg);

	spin_unlock_irqrestore(&tegra_mc_lock, flags);

	do {
		udelay(10);
		rst_stat = 0;
		ret = tegra_stable_hotreset_check(rst_stat_reg, &rst_stat);
		if (!ret)
			continue;
	} while (!(rst_stat & (1 << id)));

	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush);

int tegra_mc_flush_done(int id)
{
	u32 rst_ctrl;
	u32 rst_ctrl_reg, rst_stat_reg;
	unsigned long flags;

	if (id < 32) {
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
	} else {
		id %= 32;
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
	}

	spin_lock_irqsave(&tegra_mc_lock, flags);

	rst_ctrl = readl(mc_base + rst_ctrl_reg);
	rst_ctrl &= ~(1 << id);
	writel(rst_ctrl, mc_base + rst_ctrl_reg);

	spin_unlock_irqrestore(&tegra_mc_lock, flags);

	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush_done);
