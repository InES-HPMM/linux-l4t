/*
 * arch/arm/mach-tegra/tegra12_emc.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include "clock.h"
#include "iomap.h"
#include "tegra12_emc.h"

static u32 dram_type = -1;

static struct clk *emc;

struct tegra_emc_table;

u8 tegra_emc_bw_efficiency = 80;

static void __iomem *emc_base = IO_ADDRESS(TEGRA_EMC_BASE);
static void __iomem *mc_base = IO_ADDRESS(TEGRA_MC_BASE);

static inline void emc_writel(u32 val, unsigned long addr)
{
	writel(val, emc_base + addr);
	barrier();
}
static inline u32 emc_readl(unsigned long addr)
{
	return readl(emc_base + addr);
}
static inline void mc_writel(u32 val, unsigned long addr)
{
	writel(val, mc_base + addr);
	barrier();
}
static inline u32 mc_readl(unsigned long addr)
{
	return readl(mc_base + addr);
}

int tegra_emc_set_rate(unsigned long rate)
{
	/* FIXME: This is just a stub */
	return 0;
}

long tegra_emc_round_rate(unsigned long rate)
{
	/* FIXME: This is just a stub */
	return -EINVAL;
}

struct clk *tegra_emc_predict_parent(unsigned long rate, u32 *div_value)
{
	/* FIXME: This is just a stub */
	return NULL;
}

bool tegra_emc_is_parent_ready(unsigned long rate, struct clk **parent,
		unsigned long *parent_rate, unsigned long *backup_rate)
{
	return true;  /* !!!FIXME!!! t124 needs major MC updates */
}

void tegra_init_emc(const struct tegra_emc_table *table, int table_size)
{
	/* FIXME: This is just a stub */
}

void tegra_emc_timing_invalidate(void)
{
	/* FIXME: This is just a stub */
}

void tegra_emc_dram_type_init(struct clk *c)
{
	emc = c;

	dram_type = (emc_readl(EMC_FBIO_CFG5) &
		     EMC_CFG5_TYPE_MASK) >> EMC_CFG5_TYPE_SHIFT;
}

int tegra_emc_get_dram_type(void)
{
	return dram_type;
}
