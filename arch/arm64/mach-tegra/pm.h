/*
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
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

#ifndef _MACH_TEGRA_PM_H_
#define _MACH_TEGRA_PM_H_

#define PMC_SCRATCH0		0x50
#define PMC_SCRATCH1		0x54
#define PMC_SCRATCH4		0x60

struct tegra_lp1_iram {
	void	*start_addr;
	void	*end_addr;
};
extern struct tegra_lp1_iram *tegra_lp1_iram;
extern void (*tegra_sleep_core_finish)(unsigned long v2p);

void save_cpu_arch_register(void);
void restore_cpu_arch_register(void);

void tegra_clear_cpu_in_lp2(int phy_cpu_id);
bool tegra_set_cpu_in_lp2(int phy_cpu_id);

void tegra_idle_lp2_last(u32 cpu_on_time, u32 cpu_off_time);
extern void (*tegra_tear_down_cpu)(void);

#ifdef CONFIG_PM_SLEEP
void tegra_smp_clear_cpu_init_mask(void);
void tegra_init_suspend(struct pmc_pm_data *);
#else
static void tegra_init_suspend(struct pmc_pm_data *) {}
#endif

#endif /* _MACH_TEGRA_PM_H_ */
