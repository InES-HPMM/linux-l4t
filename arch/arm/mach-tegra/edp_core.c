/*
 * arch/arm/mach-tegra/edp_core.c
 *
 * Copyright (C) 2012 NVIDIA Corporation.
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
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/err.h>

#include <mach/edp.h>

#include "clock.h"
#include "fuse.h"

static DEFINE_MUTEX(core_edp_lock);

static struct tegra_core_edp_limits core_edp_limits;
static const struct tegra_core_edp_limits *limits;

static bool core_edp_scpu_state;
static int core_edp_profile;
static int core_edp_modules_state;
static int core_edp_thermal_idx;

static const char *profile_names[CORE_EDP_PROFILES_NUM] = {
	[CORE_EDP_PROFILE_BALANCED]  = "profile_balanced",
	[CORE_EDP_PROFILE_FAVOR_GPU] = "profile_favor_gpu",
	[CORE_EDP_PROFILE_FAVOR_EMC] = "profile_favor_emc",
};

static unsigned long *get_cap_rates(bool scpu_state, int profile,
				    int m_state, int t_idx)
{
	unsigned long *cap_rates = scpu_state ?
		limits->cap_rates_scpu_on : limits->cap_rates_scpu_off;

	cap_rates += ((profile * limits->core_modules_states + m_state) *
		      limits->temperature_ranges + t_idx) *
		      limits->cap_clocks_num;

	return cap_rates;
}

static unsigned long *get_current_cap_rates(void)
{
	return get_cap_rates(core_edp_scpu_state, core_edp_profile,
			     core_edp_modules_state, core_edp_thermal_idx);
}

static int set_cap_rates(unsigned long *new_rates)
{
	int i, ret;

	for (i = 0; i < limits->cap_clocks_num; i++) {
		struct clk *c = limits->cap_clocks[i];
		ret = clk_set_rate(c, new_rates[i]);
		if (ret) {
			pr_err("%s: Failed to set %s rate %lu\n",
			       __func__, c->name, new_rates[i]);
			return ret;
		}
	}
	return 0;
}

#if 0
static int update_cap_rates(unsigned long *new_rates, unsigned long *old_rates)
{
	int i, ret;

	/* 1st lower caps */
	for (i = 0; i < limits->cap_clocks_num; i++) {
		if (new_rates[i] < old_rates[i]) {
			struct clk *c = limits->cap_clocks[i];
			ret = clk_set_rate(c, new_rates[i]);
			if (ret) {
				pr_err("%s: Failed to set %s rate %lu\n",
				       __func__, c->name, new_rates[i]);
				return ret;
			}

		}
	}

	/* then increase caps */
	for (i = 0; i < limits->cap_clocks_num; i++) {
		if (new_rates[i] > old_rates[i]) {
			struct clk *c = limits->cap_clocks[i];
			ret = clk_set_rate(c, new_rates[i]);
			if (ret) {
				pr_err("%s: Failed to set %s rate %lu\n",
				       __func__, c->name, new_rates[i]);
				return ret;
			}

		}
	}
	return 0;
}
#endif

/* FIXME: resume sync ? */

static int __init start_core_edp(void)
{
	int ret;

	/*
	 * Default state:
	 * always boot G-cluster (no cpu on core rail),
	 * non-throttled EMC profile
	 * all core modules that affect EDP are On
	 * unknown temperature - assume maximum (WC)
	 */
	core_edp_scpu_state = false;
	core_edp_profile = CORE_EDP_PROFILE_FAVOR_EMC;
	core_edp_modules_state = 0;
	core_edp_thermal_idx = limits->temperature_ranges - 1;

	ret = set_cap_rates(get_current_cap_rates());
	if (ret)
		return ret;

	return 0;
}

void __init tegra_init_core_edp_limits(unsigned int regulator_mA)
{
	int i;
	unsigned long *cap_rates;

	switch (tegra_chip_id) {
	case TEGRA11X:
		if (tegra11x_select_core_edp_table(
			regulator_mA, &core_edp_limits))
			return;
		break;
	default:
		pr_err("%s: core edp is not supported on chip ID %d\n",
		       __func__, tegra_chip_id);
		return;
	}

	limits = &core_edp_limits;

	if (start_core_edp()) {
		limits = NULL;
		return;
	}

	cap_rates = get_current_cap_rates();
	pr_info("Core EDP limits are initialized at:\n");
	for (i = 0; i < limits->cap_clocks_num; i++)
		pr_info("    %10s: %lu\n",
			limits->cap_clocks[i]->name, cap_rates[i]);
}

#ifdef CONFIG_DEBUG_FS

static int edp_table_show(struct seq_file *s, void *data)
{
	int i, j, k, l;
	unsigned long *cap_rates;

	seq_printf(s, "VDD_CORE EDP TABLE (cap rates in kHz)\n");

	seq_printf(s, "%10s", " Temp.");
	for (l = 0; l < limits->cap_clocks_num; l++)
		seq_printf(s, "%10s", limits->cap_clocks[l]->name);
	seq_printf(s, "\n");
	for (l = 0; l < 10+10*limits->cap_clocks_num; l++)
		seq_printf(s, "-");
	seq_printf(s, "\n");

	seq_printf(s, "SCPU ON\n");
	for (i = 0; i < CORE_EDP_PROFILES_NUM; i++) {
		seq_printf(s, "%-19s%d\n", profile_names[i], i);
		for (j = 0; j < limits->core_modules_states; j++) {
			seq_printf(s, "%-19s%d\n", "modules_state", j);
			for (k = 0; k < limits->temperature_ranges; k++) {
				seq_printf(s, "%8dC:", limits->temperatures[k]);
				cap_rates = get_cap_rates(true, i, j, k);
				for (l = 0; l < limits->cap_clocks_num; l++)
					seq_printf(s, "%10lu",
						   cap_rates[l]/1000);
				seq_printf(s, "\n");
			}
		}
	}

	seq_printf(s, "SCPU OFF\n");
	for (i = 0; i < CORE_EDP_PROFILES_NUM; i++) {
		seq_printf(s, "%-19s%d\n", profile_names[i], i);
		for (j = 0; j < limits->core_modules_states; j++) {
			seq_printf(s, "%-19s%d\n", "modules_state", j);
			for (k = 0; k < limits->temperature_ranges; k++) {
				seq_printf(s, "%8dC:", limits->temperatures[k]);
				cap_rates = get_cap_rates(false, i, j, k);
				for (l = 0; l < limits->cap_clocks_num; l++)
					seq_printf(s, "%10lu",
						   cap_rates[l]/1000);
				seq_printf(s, "\n");
			}
		}
	}

	return 0;
}
static int edp_table_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_table_show, inode->i_private);
}
static const struct file_operations edp_table_fops = {
	.open		= edp_table_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int profile_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%s\n", profile_names[core_edp_profile]);
	return 0;
}
static int profile_open(struct inode *inode, struct file *file)
{
	return single_open(file, profile_show, inode->i_private);
}
static const struct file_operations profile_fops = {
	.open		= profile_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int range_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%d\n", limits->temperatures[core_edp_thermal_idx]);
	return 0;
}
static int range_open(struct inode *inode, struct file *file)
{
	return single_open(file, range_show, inode->i_private);
}
static const struct file_operations range_fops = {
	.open		= range_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int rates_show(struct seq_file *s, void *data)
{
	int i;
	unsigned long *cap_rates;

	mutex_lock(&core_edp_lock);
	cap_rates = get_current_cap_rates();
	mutex_unlock(&core_edp_lock);

	for (i = 0; i < limits->cap_clocks_num; i++)
		seq_printf(s, "%-10srate (kHz): %lu\n",
			   limits->cap_clocks[i]->name, cap_rates[i] / 1000);
	return 0;
}
static int rates_open(struct inode *inode, struct file *file)
{
	return single_open(file, rates_show, inode->i_private);
}
static const struct file_operations rates_fops = {
	.open		= rates_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int __init tegra_core_edp_debugfs_init(struct dentry *edp_dir)
{
	struct dentry *dir, *d;

	if (!limits)
		return 0;

	dir = debugfs_create_dir("vdd_core", edp_dir);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file("edp", S_IRUGO, dir, NULL, &edp_table_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_bool("scpu_state", S_IRUGO, dir,
				(u32 *)&core_edp_scpu_state);
	if (!d)
		goto err_out;

	d = debugfs_create_file("profile", S_IRUGO, dir, NULL, &profile_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_u32("modules_state", S_IRUGO, dir,
			       (u32 *)&core_edp_modules_state);
	if (!d)
		goto err_out;

	d = debugfs_create_file("therm_range", S_IRUGO, dir, NULL, &range_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("rates", S_IRUGO, dir, NULL, &rates_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(dir);
	return -ENOMEM;


}

#endif
