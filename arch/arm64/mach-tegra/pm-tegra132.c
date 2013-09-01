/*
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/io.h>

#include "pmc.h"
#include "pm.h"
#include "sleep.h"
#include "pm-tegra132.h"

extern enum tegra_suspend_mode current_suspend_mode;

void tegra132_do_idle(ulong pmstate)
{
	do {
		asm volatile(
		"	msr actlr_el1, %0\n"
		"	wfi\n"
		:
		: "r" (pmstate));
	} while (1);
}

/*
 * tegra132_tear_down_cpu
 *  - core power gating entry finisher
 */
static void tegra132_tear_down_cpu(void)
{
	tegra132_do_idle(T132_CORE_C7);
}

/*
 * tegra132_sleep_core_finish(unsigned long v2p)
 *  - cluster power state entry finisher
 *  - v2p: ignored
 *
 * At this point, we have already:
 *  - switched to identity mapping
 *  - flushed and disabled dcache
 */
static void tegra132_sleep_core_finish(unsigned long v2p)
{
	int pmstate = T132_SYSTEM_LP1;

	if (current_suspend_mode == TEGRA_SUSPEND_LP0)
		pmstate = T132_SYSTEM_LP0;

	tegra132_do_idle(pmstate);
}

#ifdef CONFIG_PM_SLEEP
static struct tegra_lp1_iram tegra132_lp1_iram;

void tegra132_lp1_iram_hook(void)
{
	tegra132_lp1_iram.start_addr = &tegra132_iram_start;
	tegra132_lp1_iram.end_addr = &tegra132_iram_end;

	tegra_lp1_iram = &tegra132_lp1_iram;
}

void tegra132_sleep_core_init(void)
{
	tegra_tear_down_cpu = tegra132_tear_down_cpu;
	tegra_sleep_core_finish = tegra132_sleep_core_finish;
}
#endif
