/*
 * arch/arm/mach-tegra/tegra_core_volt_cap.c
 *
 * Copyright (C) 2013 NVIDIA Corporation.
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
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/err.h>

#include "clock.h"
#include "dvfs.h"

/*
 * sysfs and kernel interfaces to limit tegra core shared bus frequencies based
 * on the required core voltage (cap level)
 *
 * Cap level is specified in millivolts, and rate limits from the respective
 * dvfs tables are applied to all bus clocks. Note that cap level affects only
 * scalable bus frequencies (graphics bus, emc, system clock). Core voltage is
 * not necessarily set at the cap level, since CPU and/or fixed peripheral
 * clocks outside the buses may require higher voltage level.
 */
static DEFINE_MUTEX(core_cap_lock);

static struct core_cap core_buses_cap;
static struct core_cap kdvfs_core_cap;
static struct core_cap user_core_cap;

static struct core_dvfs_cap_table *core_cap_table;
static int core_cap_table_size;

static const int *cap_millivolts;
static int cap_millivolts_num;

static int core_nominal_mv;

static void core_cap_level_set(int level)
{
	int i, j;

	if (!core_cap_table)
		return;

	for (j = 0; j < cap_millivolts_num; j++) {
		int v = cap_millivolts[j];
		if ((v == 0) || (level < v))
			break;
	}
	j = (j == 0) ? 0 : j - 1;
	level = cap_millivolts[j];

	if (level < core_buses_cap.level) {
		for (i = 0; i < core_cap_table_size; i++)
			if (core_cap_table[i].cap_clk)
				clk_set_rate(core_cap_table[i].cap_clk,
					     core_cap_table[i].freqs[j]);
	} else if (level > core_buses_cap.level) {
		for (i = core_cap_table_size - 1; i >= 0; i--)
			if (core_cap_table[i].cap_clk)
				clk_set_rate(core_cap_table[i].cap_clk,
					     core_cap_table[i].freqs[j]);
	}
	core_buses_cap.level = level;
}

static void core_cap_update(void)
{
	int new_level = core_nominal_mv;

	if (kdvfs_core_cap.refcnt)
		new_level = min(new_level, kdvfs_core_cap.level);
	if (user_core_cap.refcnt)
		new_level = min(new_level, user_core_cap.level);

	if (core_buses_cap.level != new_level)
		core_cap_level_set(new_level);
}

static void core_cap_enable(bool enable)
{
	if (enable)
		core_buses_cap.refcnt++;
	else if (core_buses_cap.refcnt)
		core_buses_cap.refcnt--;

	core_cap_update();
}

static ssize_t
core_cap_state_show(struct kobject *kobj, struct kobj_attribute *attr,
		    char *buf)
{
	return sprintf(buf, "%d (%d)\n", core_buses_cap.refcnt ? 1 : 0,
			user_core_cap.refcnt ? 1 : 0);
}
static ssize_t
core_cap_state_store(struct kobject *kobj, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	int state;

	if (sscanf(buf, "%d", &state) != 1)
		return -1;

	mutex_lock(&core_cap_lock);

	if (state) {
		user_core_cap.refcnt++;
		if (user_core_cap.refcnt == 1)
			core_cap_enable(true);
	} else if (user_core_cap.refcnt) {
		user_core_cap.refcnt--;
		if (user_core_cap.refcnt == 0)
			core_cap_enable(false);
	}

	mutex_unlock(&core_cap_lock);
	return count;
}

static ssize_t
core_cap_level_show(struct kobject *kobj, struct kobj_attribute *attr,
		    char *buf)
{
	return sprintf(buf, "%d (%d)\n", core_buses_cap.level,
			user_core_cap.level);
}
static ssize_t
core_cap_level_store(struct kobject *kobj, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	int level;

	if (sscanf(buf, "%d", &level) != 1)
		return -1;

	mutex_lock(&core_cap_lock);
	user_core_cap.level = level;
	core_cap_update();
	mutex_unlock(&core_cap_lock);
	return count;
}

static struct kobj_attribute cap_state_attribute =
	__ATTR(core_cap_state, 0644, core_cap_state_show, core_cap_state_store);
static struct kobj_attribute cap_level_attribute =
	__ATTR(core_cap_level, 0644, core_cap_level_show, core_cap_level_store);

const struct attribute *cap_attributes[] = {
	&cap_state_attribute.attr,
	&cap_level_attribute.attr,
	NULL,
};

void tegra_dvfs_core_cap_enable(bool enable)
{
	mutex_lock(&core_cap_lock);

	if (enable) {
		kdvfs_core_cap.refcnt++;
		if (kdvfs_core_cap.refcnt == 1)
			core_cap_enable(true);
	} else if (kdvfs_core_cap.refcnt) {
		kdvfs_core_cap.refcnt--;
		if (kdvfs_core_cap.refcnt == 0)
			core_cap_enable(false);
	}
	mutex_unlock(&core_cap_lock);
}

void tegra_dvfs_core_cap_level_set(int level)
{
	mutex_lock(&core_cap_lock);
	kdvfs_core_cap.level = level;
	core_cap_update();
	mutex_unlock(&core_cap_lock);
}

static int __init init_core_cap_one(struct clk *c, unsigned long *freqs)
{
	int i, v, next_v = 0;
	unsigned long rate, next_rate = 0;

	for (i = 0; i < cap_millivolts_num; i++) {
		v = cap_millivolts[i];
		if (v == 0)
			break;

		for (;;) {
			rate = next_rate;
			next_rate = clk_round_rate(c->parent, rate + 1000);
			if (IS_ERR_VALUE(next_rate)) {
				pr_debug("tegra11_dvfs: failed to round %s rate %lu\n",
					 c->name, rate);
				return -EINVAL;
			}
			if (rate == next_rate)
				break;

			next_v = tegra_dvfs_predict_millivolts(
				c->parent, next_rate);
			if (IS_ERR_VALUE(next_v)) {
				pr_debug("tegra11_dvfs: failed to predict %s mV for rate %lu\n",
					 c->name, next_rate);
				return -EINVAL;
			}
			if (next_v > v)
				break;
		}

		if (rate == 0) {
			rate = next_rate;
			pr_warn("tegra11_dvfs: minimum %s rate %lu requires %d mV\n",
				c->name, rate, next_v);
		}
		freqs[i] = rate;
		next_rate = rate;
	}
	return 0;
}

int __init tegra_init_core_cap(
	struct core_dvfs_cap_table *table, int table_size,
	const int *millivolts, int millivolts_num,
	struct kobject *cap_kobj)
{
	int i;
	struct clk *c = NULL;

	if (!table || !table_size || !millivolts || !millivolts_num)
		return -EINVAL;

	core_nominal_mv =
		tegra_dvfs_rail_get_nominal_millivolts(tegra_core_rail);
	if (core_nominal_mv <= 0)
		return -ENODATA;

	cap_millivolts = millivolts;
	cap_millivolts_num = millivolts_num;
	core_buses_cap.level = kdvfs_core_cap.level = user_core_cap.level =
		core_nominal_mv;

	for (i = 0; i < table_size; i++) {
		c = tegra_get_clock_by_name(table[i].cap_name);
		if (!c || !c->parent ||
		    init_core_cap_one(c, table[i].freqs)) {
			pr_err("%s: failed to initialize %s table\n",
			       __func__, table[i].cap_name);
			continue;
		}
		table[i].cap_clk = c;
	}

	if (!cap_kobj || sysfs_create_files(cap_kobj, cap_attributes))
		return -ENOMEM;

	core_cap_table = table;
	core_cap_table_size = table_size;
	return 0;
}

