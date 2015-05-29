/*
 * drivers/platform/tegra/cpu-tegra21.c
 *
 * Copyright (C) 2015 NVIDIA Corporation. All rights reserved.
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
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpufreq.h>

#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/cpu-tegra.h>

/*
 * Frequency table index must be sequential starting at 0 and frequencies
 * must be ascending.
 */
#define CPU_FREQ_STEP 102000 /* 102MHz cpu_g table step */
#define CPU_FREQ_TABLE_MAX_SIZE (2 * MAX_DVFS_FREQS + 1)

static struct cpufreq_frequency_table freq_table[CPU_FREQ_TABLE_MAX_SIZE];
static struct tegra_cpufreq_table_data freq_table_data;

struct tegra_cpufreq_table_data *tegra_cpufreq_table_get(void)
{
	int i, j, n;
	unsigned int virt_freq;
	unsigned int freq, lp_backup_freq, g_vmin_freq, g_start_freq, max_freq;
	struct clk *cpu_clk_g = tegra_get_clock_by_name("cpu_g");
	struct clk *cpu_clk_lp = tegra_get_clock_by_name("cpu_lp");

	/* Initialize once */
	if (freq_table_data.freq_table)
		return &freq_table_data;

	/* Clean table */
	for (i = 0; i < CPU_FREQ_TABLE_MAX_SIZE; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = CPUFREQ_TABLE_END;
	}

	lp_backup_freq = cpu_clk_lp->u.cpu.backup_rate / 1000;
	if (!lp_backup_freq) {
		WARN(1, "%s: cannot make cpufreq table: no LP CPU backup rate\n",
		     __func__);
		return NULL;
	}
	if (!cpu_clk_lp->dvfs) {
		WARN(1, "%s: cannot make cpufreq table: no LP CPU dvfs\n",
		     __func__);
		return NULL;
	}
	if (!cpu_clk_g->dvfs) {
		WARN(1, "%s: cannot make cpufreq table: no G CPU dvfs\n",
		     __func__);
		return NULL;
	}
	g_vmin_freq = cpu_clk_g->dvfs->freqs[0] / 1000;
	if (g_vmin_freq < lp_backup_freq) {
		WARN(1, "%s: cannot make cpufreq table: LP CPU backup rate"
			" exceeds G CPU rate at Vmin\n", __func__);
		return NULL;
	}

	freq = lp_backup_freq;
	virt_freq = lp_to_virtual_gfreq(freq);

	/* Start with backup frequencies */
	i = 0;
	freq_table[i++].frequency = virt_freq / 2;
	freq_table[i++].frequency = virt_freq;

	/* Throttle low index at backup level*/
	freq_table_data.throttle_lowest_index = i - 1;

	/*
	 * Next, set table steps along LP CPU dvfs ladder, but make sure G CPU
	 * dvfs rate at minimum voltage is not missed (if it happens to be below
	 * LP maximum rate)
	 */
	max_freq = cpu_clk_lp->max_rate / 1000;
	for (j = 0; j < cpu_clk_lp->dvfs->num_freqs; j++) {
		freq = cpu_clk_lp->dvfs->freqs[j] / 1000;
		virt_freq = lp_to_virtual_gfreq(freq);
		if (freq <= lp_backup_freq)
			continue;

		freq_table[i++].frequency = virt_freq;

		if (freq == max_freq)
			break;
	}

	/* Set G CPU min rate to lowest freq in the table */
	cpu_clk_g->min_rate = min(freq_table[0].frequency,
					g_vmin_freq) * 1000;

	/* Suspend index at max LP CPU */
	freq_table_data.suspend_index = i - 1;

	/*
	 * Fill in "hole" (if any) between LP CPU maximum rate and G CPU dvfs
	 * ladder rate at minimum voltage
	 */
	if (virt_freq < g_vmin_freq) {
		int n = (g_vmin_freq - virt_freq) / CPU_FREQ_STEP;
		for (j = 0; j <= n; j++) {
			virt_freq = g_vmin_freq - CPU_FREQ_STEP * (n - j);
			freq_table[i++].frequency = virt_freq;
		}
	}

	/* Now, step along the rest of G CPU dvfs ladder */
	g_start_freq = virt_freq;
	max_freq = cpu_clk_g->max_rate / 1000;
	n = cpu_clk_g->dvfs->num_freqs - (cpu_clk_g->dvfs->boost_table ? 1 : 0);
	for (j = 0; j < n; j++) {
		freq = cpu_clk_g->dvfs->freqs[j] / 1000;
		if (freq > g_start_freq)
			freq_table[i++].frequency = freq;
		if (freq == max_freq)
			break;
	}

	/* Throttle high index one step below maximum */
	BUG_ON(i >= CPU_FREQ_TABLE_MAX_SIZE);
	freq_table_data.throttle_highest_index = i - 2;
	freq_table_data.freq_table = freq_table;
	return &freq_table_data;
}

unsigned long tegra_emc_cpu_limit(unsigned long cpu_rate)
{
	static unsigned long emc_max_rate;

	if (emc_max_rate == 0)
		emc_max_rate = clk_round_rate(
			tegra_get_clock_by_name("emc"), ULONG_MAX);

	/* Vote on memory bus frequency based on cpu frequency;
	   cpu rate is in kHz, emc rate is in Hz */
	if (cpu_rate >= 1300000)
		return emc_max_rate;	/* cpu >= 1.3GHz, emc max */
	else if (cpu_rate >= 975000)
		return 400000000;	/* cpu >= 975 MHz, emc 400 MHz */
	else if (cpu_rate >= 725000)
		return  200000000;	/* cpu >= 725 MHz, emc 200 MHz */
	else if (cpu_rate >= 500000)
		return  100000000;	/* cpu >= 500 MHz, emc 100 MHz */
	else if (cpu_rate >= 275000)
		return  50000000;	/* cpu >= 275 MHz, emc 50 MHz */
	else
		return 0;		/* emc min */
}

unsigned long tegra_emc_to_cpu_ratio(unsigned long cpu_rate)
{
	return tegra_emc_cpu_limit(cpu_rate);
}

int tegra_update_mselect_rate(unsigned long cpu_rate)
{
	static struct clk *mselect; /* statics init to 0 */

	unsigned long mselect_rate;

	if (!mselect) {
		mselect = tegra_get_clock_by_name("cpu.mselect");
		if (!mselect)
			return -ENODEV;
	}

	/* Vote on mselect frequency based on cpu frequency:
	   keep mselect at half of cpu rate up to 102 MHz;
	   cpu rate is in kHz, mselect rate is in Hz */
	mselect_rate = DIV_ROUND_UP(cpu_rate, 2) * 1000;
	mselect_rate = min(mselect_rate, 102000000UL);
	return clk_set_rate(mselect, mselect_rate);
}
