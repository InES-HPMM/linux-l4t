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

#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>

#include <mach/hardware.h>

#include "denver-knobs.h"

#define BG_CLR_SHIFT	16
#define BG_STATUS_SHIFT	32

bool denver_get_bg_allowed(int cpu)
{
	unsigned long regval;

	asm volatile("mrs %0, s3_0_c15_c0_2" : "=r" (regval) : );
	regval = (regval >> BG_STATUS_SHIFT) & 0xffff;

	return !!(regval & (1 << cpu));
}

void denver_set_bg_allowed(int cpu, bool enable)
{
	unsigned long regval;

	BUG_ON(cpu >= num_present_cpus());

	regval = 1 << cpu;
	if (!enable)
		regval <<= BG_CLR_SHIFT;

	asm volatile("msr s3_0_c15_c0_2, %0" : : "r" (regval));
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *denver_debugfs_root;

static int bgallowed_get(void *data, u64 *val)
{
	int cpu = (int)((s64)data);
	*val = denver_get_bg_allowed(cpu);
	return 0;
}

static int bgallowed_set(void *data, u64 val)
{
	int cpu = (int)((s64)data);
	denver_set_bg_allowed(cpu, (bool)val);
	pr_debug("CPU%d: BGALLOWED is %s.\n",
			cpu, val ? "enabled" : "disabled");
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bgallowed_fops, bgallowed_get,
			bgallowed_set, "%llu\n");

static int __init denver_knobs_init(void)
{
	int cpu;
	int error;
	char name[30];

	if (tegra_cpu_is_asim())
		return 0;

	denver_debugfs_root = debugfs_create_dir("tegra_denver", NULL);

	for_each_present_cpu(cpu) {
		snprintf(name, 30, "bgallowed_cpu%d", cpu);
		if (!debugfs_create_file(
			name, S_IRUGO, denver_debugfs_root,
			(void *)((s64)cpu), &bgallowed_fops))
			return error;
	}

	return 0;

}
device_initcall(denver_knobs_init);

#endif
