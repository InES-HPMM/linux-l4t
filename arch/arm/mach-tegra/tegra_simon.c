/*
 * arch/arm/mach-tegra/tegra_simon.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "tegra_simon.h"

static DEFINE_MUTEX(simon_lock);
static RAW_NOTIFIER_HEAD(simon_nh);
static int simon_grades[TEGRA_SIMON_DOMAIN_NUM];

static const char *simon_domain_names[TEGRA_SIMON_DOMAIN_NUM] = {
	[TEGRA_SIMON_DOMAIN_CPU] = "cpu",
	[TEGRA_SIMON_DOMAIN_GPU] = "gpu",
	[TEGRA_SIMON_DOMAIN_CORE] = "core",
};


int tegra_register_simon_notifier(struct notifier_block *nb)
{
	int ret;

	mutex_lock(&simon_lock);
	ret = raw_notifier_chain_register(&simon_nh, nb);
	mutex_unlock(&simon_lock);
	return ret;
}

void tegra_unregister_simon_notifier(struct notifier_block *nb)
{
	mutex_lock(&simon_lock);
	raw_notifier_chain_unregister(&simon_nh, nb);
	mutex_unlock(&simon_lock);
}

static int simon_grade_change_notify(int grade, enum tegra_simon_domain domain)
{
	int ret = 0;

	mutex_lock(&simon_lock);
	if (simon_grades[domain] != grade) {
		simon_grades[domain] = grade;
		ret = raw_notifier_call_chain(&simon_nh, grade, (void *)domain);
	}
	mutex_unlock(&simon_lock);
	return ret;
}

#ifdef CONFIG_DEBUG_FS

static int grade_get(void *data, u64 *val)
{
	int domain = (int)data;

	if (domain >= TEGRA_SIMON_DOMAIN_NUM) {
		*val = -EINVAL;
		return -EINVAL;
	}

	*val = simon_grades[domain];
	return 0;
}
static int grade_set(void *data, u64 val)
{
	int domain = (int)data;

	if (domain >= TEGRA_SIMON_DOMAIN_NUM)
		return -EINVAL;

	simon_grade_change_notify(val, domain);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(grade_fops, grade_get, grade_set, "%llu\n");

static int __init simon_debugfs_init(void)
{
	int domain;
	struct dentry *d, *dir;

	dir = debugfs_create_dir("tegra_simon", NULL);
	if (!dir)
		return -ENOMEM;

	for (domain = 0; domain < TEGRA_SIMON_DOMAIN_NUM; domain++) {
		if (!simon_domain_names[domain])
			continue;

		d = debugfs_create_file(simon_domain_names[domain],
			S_IWUSR | S_IRUGO, dir, (void *)domain, &grade_fops);
		if (!d)
			goto err_out;
	}

	return 0;

err_out:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}
#endif

static int __init tegra_simon_init(void)
{
#ifdef CONFIG_DEBUG_FS
	simon_debugfs_init();
#endif
	return 0;

}
late_initcall_sync(tegra_simon_init);

