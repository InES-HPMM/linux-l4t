/*
 * arch/arm/mach-tegra/tegra12_emc.h
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#ifndef _MACH_TEGRA_TEGRA11_EMC_H
#define _MACH_TEGRA_TEGRA11_EMC_H

#include "tegra_emc.h"

/* !!!FIXME!!! Need actual Tegra11x values */
#define TEGRA_EMC_NUM_REGS		110

#define TEGRA_EMC_BRIDGE_RATE_MIN	300000000
#define TEGRA_EMC_BRIDGE_MVOLTS_MIN	1200

struct tegra_emc_table {
	u8 rev;
	unsigned long rate;

	/* unconditionally updated in one burst shot */
	u32 burst_regs[TEGRA_EMC_NUM_REGS];

	/* updated separately under some conditions */
	u32 emc_zcal_cnt_long;
	u32 emc_acal_interval;
	u32 emc_periodic_qrst;
	u32 emc_mode_reset;
	u32 emc_mode_1;
	u32 emc_mode_2;
	u32 emc_dsr;
	int emc_min_mv;
};

struct clk;

void tegra_init_emc(const struct tegra_emc_table *table, int table_size);

void tegra_emc_dram_type_init(struct clk *c);
int tegra_emc_get_dram_type(void);

#ifdef CONFIG_PM_SLEEP
void tegra_mc_timing_restore(void);
#else
static inline void tegra_mc_timing_restore(void)
{ }
#endif

#define EMC_FBIO_CFG5				0x104
#define EMC_CFG5_TYPE_SHIFT			0x0
#define EMC_CFG5_TYPE_MASK			(0x3 << EMC_CFG5_TYPE_SHIFT)
enum {
	DRAM_TYPE_DDR3   = 0,
	DRAM_TYPE_LPDDR2 = 2,
};

#endif
