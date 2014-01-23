/*
 * arch/arm64/mach-tegra/pm-tegra132.h
 *
 * Copyright (C) 2013 NVIDIA Corporation. All rights reserved.
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

#ifndef __ASM_ARM_T132_H
#define __ASM_ARM_T132_H

#ifndef __ASSEMBLY__

enum tegra132_power_states {
	T132_CORE_C1 = 0,
	T132_CORE_C4 = 1,
	T132_CORE_C6 = 2,
	T132_CORE_C7 = 3,
	T132_CLUSTER_C4_SLEEP_EN = 8,
	T132_CLUSTER_C4 = 9,
	T132_CLUSTER_C6 = 0xA,
	T132_CLUSTER_C7 = 0xB,
	T132_SYSTEM_LP1 = 0xC,
	T132_SYSTEM_LP0 = 0xD,
};

#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
void tegra132_lp1_iram_hook(void);
void tegra132_sleep_core_init(void);
#else
static inline void tegra132_lp1_iram_hook(void) {}
static inline void tegra132_sleep_core_init(void) {}
#endif

/* Define in sleep-tegra132.S */
extern u32 tegra132_iram_start;
extern u32 tegra132_iram_end;

void tegra132_do_idle(ulong pmstate);

#endif /* __ASSEMBLY__ */

#endif
