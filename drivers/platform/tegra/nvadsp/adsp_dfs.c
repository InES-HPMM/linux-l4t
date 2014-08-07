/*
 * adsp_dfs.c
 *
 * adsp dynamic frequency scaling
 *
 * Copyright (C) 2014, NVIDIA Corporation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>

#include "ape_actmon.h"

#define POLICY_MAX 600000 /* 600 MHz: max adsp freq */

struct adsp_dfs_policy {
	bool enable;

	const char *clk_name;
	unsigned long min;    /* in kHz */
	unsigned long max;    /* in kHz */
	unsigned long cur;    /* in kHz */
	spinlock_t lock;

	struct clk *adsp_clk;
	struct notifier_block rate_change_nb;

#ifdef CONFIG_DEBUG_FS
	struct dentry *root;
#endif
};

struct adsp_dfs_policy *policy;

/* adsp clock rate change notifier callback */
static int adsp_dfs_rc_callback(
	struct notifier_block *nb, unsigned long rate, void *v)
{
	unsigned long flags;
	unsigned long freq = rate / 1000;
	struct adsp_dfs_policy *dfs_pol = container_of(
				nb, struct adsp_dfs_policy, rate_change_nb);

	spin_lock_irqsave(&dfs_pol->lock, flags);

	actmon_rate_change(freq);
	policy->cur = freq;

	spin_unlock_irqrestore(&policy->lock, flags);

	pr_info("policy->cur:%lu\n", policy->cur);
	/* TBD: Communicate ADSP about new freq */

	return NOTIFY_OK;
};

static struct adsp_dfs_policy dfs_policy =  {
	.enable = 0,
	.clk_name = "adsp_cpu",
	.max = POLICY_MAX, /* Check for 600MHz for max clock freq */
	.rate_change_nb = {
		.notifier_call = adsp_dfs_rc_callback,
	},
};

/* @parameter freq(KHz): can be any freq value */
unsigned long verify_policy(unsigned long freq)
{
	/* TBD: set freq  from adsp freq table. */
	 if (freq < policy->min)
		freq = policy->min;
	 else if (freq > policy->max)
		freq = policy->max;

	return freq;
}

/**
 * @parameter freq(KHz): one of the freq adsp freq table
 */
void update_policy(unsigned long freq)
{
	if (freq)
		clk_set_rate(policy->adsp_clk, freq * 1000);
}

#ifdef CONFIG_DEBUG_FS

#define RW_MODE (S_IWUSR | S_IRUGO)
#define RO_MODE S_IRUGO

/* Get adsp dfs staus: 0: disabled, 1: enabled */
static int dfs_enable_get(void *data, u64 *val)
{
	*val = policy->enable;
	return 0;
}

/* Enable/disable adsp dfs */
static int dfs_enable_set(void *data, u64 val)
{
	policy->enable = (bool) val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, dfs_enable_get,
	dfs_enable_set, "%llu\n");

/* Get adsp dfs policy min freq */
static int policy_min_get(void *data, u64 *val)
{
	*val = policy->min;
	return 0;
}

/* Set adsp dfs policy min freq */
static int policy_min_set(void *data, u64 val)
{
	if (policy->enable) {
		policy->min = verify_policy((unsigned long)val);
		update_policy(policy->min);
		return 0;
	} else
		return -EINVAL;
}
DEFINE_SIMPLE_ATTRIBUTE(min_fops, policy_min_get,
	policy_min_set, "%llu\n");

/* Get adsp dfs policy max freq */
static int policy_max_get(void *data, u64 *val)
{
	*val = policy->max;
	return 0;
}

/* Set adsp dfs policy max freq */
static int policy_max_set(void *data, u64 val)
{
	if (policy->enable) {
		policy->max = verify_policy((unsigned long)val);
		update_policy(policy->max);
		return 0;
	} else
		return -EINVAL;
}
DEFINE_SIMPLE_ATTRIBUTE(max_fops, policy_max_get,
	policy_max_set, "%llu\n");

/* Get adsp dfs policy's current freq */
static int policy_cur_get(void *data, u64 *val)
{
	*val = policy->cur;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cur_fops, policy_cur_get,
	NULL, "%llu\n");

static int __init adsp_dfs_debugfs_init(void)
{
	int ret = -ENOMEM;
	struct dentry *d, *root;

	root = debugfs_create_dir("adsp_dfs", NULL);
	if (!root)
		return ret;

	policy->root = root;

	d = debugfs_create_file("enable", RW_MODE, root, NULL,
		&enable_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("min_freq", RW_MODE, root, NULL,
		&min_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("max_freq", RW_MODE, root,
		NULL, &max_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("cur_freq", RO_MODE, root, NULL,
		&cur_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(root);
	policy->root = NULL;
	return ret;
}
#endif

void adsp_cpu_set_rate(unsigned long freq)
{
	if (policy->enable) {
		freq = verify_policy(freq);
		update_policy(freq);
	}
}

int adsp_dfs_core_init(void)
{
	int ret = 0;
	policy = &dfs_policy;
	policy->adsp_clk = clk_get_sys(NULL, policy->clk_name);

	spin_lock_init(&policy->lock);

	if (policy->rate_change_nb.notifier_call) {
		ret = tegra_register_clk_rate_notifier(policy->adsp_clk,
			&policy->rate_change_nb);
		if (ret) {
			pr_err("Failed to register rate change notifier for %s\n",
			policy->clk_name);
			return ret;
		}
	}

#ifdef CONFIG_DEBUG_FS
	adsp_dfs_debugfs_init();
#endif
	pr_info("*******adsp dfs is initialised*********\n");

	return ret;
}
