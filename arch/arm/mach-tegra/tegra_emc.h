/*
 * arch/arm/mach-tegra/tegra_emc.h
 *
 * Copyright (C) 2013 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
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

#ifndef _MACH_TEGRA_TEGRA_EMC_H
#define _MACH_TEGRA_TEGRA_EMC_H

extern u8 tegra_emc_bw_efficiency;

enum {
	DRAM_OVER_TEMP_NONE = 0,
	DRAM_OVER_TEMP_REFRESH,
};

struct clk;

void tegra_emc_dram_type_init(struct clk *c);
int tegra_emc_get_dram_type(void);
int tegra_emc_get_dram_temperature(void);
int tegra_emc_set_over_temp_state(unsigned long state);

int tegra_emc_set_rate(unsigned long rate);
long tegra_emc_round_rate(unsigned long rate);
long tegra_emc_round_rate_updown(unsigned long rate, bool up);
struct clk *tegra_emc_predict_parent(unsigned long rate, u32 *div_value);
bool tegra_emc_is_parent_ready(unsigned long rate, struct clk **parent,
		unsigned long *parent_rate, unsigned long *backup_rate);
void tegra_emc_timing_invalidate(void);

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
int tegra_emc_backup(unsigned long rate);
void tegra_init_dram_bit_map(const u32 *bit_map, int map_size);
#endif

#ifdef CONFIG_PM_SLEEP
void tegra_mc_timing_restore(void);
#else
static inline void tegra_mc_timing_restore(void)
{ }
#endif

#endif
