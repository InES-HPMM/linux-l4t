/*
 * arch/arm/mach-tegra/tegra11_cpu_volt_cap.c
 *
 * Copyright (C) 2012 NVIDIA Corporation
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
#include <linux/suspend.h>
#include <linux/seq_file.h>
#include <mach/edp.h>

#include "cpu-tegra.h"

static struct volt_cap_data {
	int capped_voltage;
	bool voltage_capping_enabled;
} capping_data;

static DEFINE_MUTEX(capping_lock);
static struct kobject *volt_cap_kobj;

static ssize_t tegra_cpu_volt_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", capping_data.capped_voltage);
}

static ssize_t tegra_cpu_volt_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtou32(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&capping_lock);
	capping_data.capped_voltage = val;
	mutex_unlock(&capping_lock);
	return count;
}

static struct kobj_attribute tegra_cpu_volt =
	__ATTR(volt, 0644, tegra_cpu_volt_show, tegra_cpu_volt_store);

static ssize_t capping_enable_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", capping_data.voltage_capping_enabled);
}

static ssize_t capping_enable_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	int voltage;

	if (kstrtou32(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&capping_lock);
	capping_data.voltage_capping_enabled = (bool)val;
	voltage = capping_data.capped_voltage;
	mutex_unlock(&capping_lock);
	if (val && voltage)
		tegra_cpu_set_volt_cap(tegra_edp_find_maxf(voltage));
	else
		tegra_cpu_set_volt_cap(0);

	return count;
}

static struct kobj_attribute capping_enable =
	__ATTR(capping_state, 0644, capping_enable_show, capping_enable_store);

const struct attribute *tegra_volt_cap_attrs[] = {
	&capping_enable.attr,
	&tegra_cpu_volt.attr,
	NULL,
};

static int volt_cap_sysfs_init(void)
{
	volt_cap_kobj = kobject_create_and_add("tegra_cpu_volt_cap",
		kernel_kobj);

	if (!volt_cap_kobj) {
		pr_info("CPU volt_cap failed\n");
		return -1;
	}

	if (sysfs_create_files(volt_cap_kobj, tegra_volt_cap_attrs)) {
		pr_err("tegra:failed to create sysfs cap interface\n");
		return -1;
	}

	return 0;
}

static int __init tegra_volt_cap_init(void)
{
	volt_cap_sysfs_init();

	return 0;
}

MODULE_DESCRIPTION("Tegra11 CPU voltage capping driver");
MODULE_LICENSE("GPL");
module_init(tegra_volt_cap_init);
