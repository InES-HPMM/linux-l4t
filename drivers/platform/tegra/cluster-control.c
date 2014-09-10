/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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

#include <linux/compiler.h>
#include <linux/cpu_pm.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_data/tegra_bpmp.h>
#include <asm/psci.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#define PSCI_NV_CPU_FORCE_SUSPEND 0x84001000

#define PSCI_RET_SUCCESS                0
#define PSCI_RET_EOPNOTSUPP             -1
#define PSCI_RET_EINVAL                 -2
#define PSCI_RET_EPERM                  -3

static DEFINE_MUTEX(cluster_switch_lock);
static unsigned long pg_core_arg, pg_cluster_arg;

static struct psci_power_state core_pg __initdata = {
	.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
	.id = 31,
	.affinity_level = 0,
};

static struct psci_power_state cluster_pg __initdata = {
	.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
	.id = 31,
	.affinity_level = 1,
};

static inline unsigned int is_slow_cluster(void)
{
	int reg;

	asm("mrs        %0, mpidr_el1\n"
	    "ubfx       %0, %0, #8, #4"
	    : "=r" (reg)
	    :
	    : "cc", "memory");

	return reg & 1;
}

static void shutdown_core(void *info)
{
	uintptr_t target_cluster = (uintptr_t)info;

	if (target_cluster != is_slow_cluster()) {
		cpu_pm_enter();
		cpu_suspend(pg_core_arg, NULL);
		cpu_pm_exit();
	}
}

static void shutdown_cluster(void)
{
	cpu_pm_enter();

	cpu_suspend(pg_cluster_arg, NULL);

	cpu_pm_exit();
}

static void switch_cluster(int val)
{
	struct cpumask mask;
	int bpmp_cpu_mask;
	int phys_cpu_id;
	uintptr_t target_cluster;

	mutex_lock(&cluster_switch_lock);
	target_cluster = val;

	preempt_disable();

	phys_cpu_id = cpu_logical_map(smp_processor_id());
	bpmp_cpu_mask = tegra_bpmp_switch_cluster(phys_cpu_id);

	if (bpmp_cpu_mask & 1)
		cpumask_set_cpu(0, &mask);
	if (bpmp_cpu_mask & 2)
		cpumask_set_cpu(1, &mask);
	if (bpmp_cpu_mask & 4)
		cpumask_set_cpu(2, &mask);
	if (bpmp_cpu_mask & 8)
		cpumask_set_cpu(3, &mask);

	cpumask_clear_cpu(phys_cpu_id, &mask);
	smp_call_function_many(&mask, shutdown_core, (void *)target_cluster,
			       false);

	local_irq_disable();
	shutdown_cluster();
	local_irq_enable();

	preempt_enable();

	mutex_unlock(&cluster_switch_lock);
}

#ifdef CONFIG_DEBUG_FS
static int cluster_set(void *data, u64 val)
{
	if (val && val != 1)
		return -EINVAL;

	if (val != is_slow_cluster())
		switch_cluster(val);

	return 0;
}

static int cluster_get(void *data, u64 *val)
{
	*val = (u64) is_slow_cluster();

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cluster_ops, cluster_get, cluster_set, "%llu\n");

static int cluster_name_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%s\n", is_slow_cluster() ? "slow" : "fast");

	return 0;
}

static int cluster_name_open(struct inode *inode, struct file *file)
{
	return single_open(file, cluster_name_show, NULL);
}

static const struct file_operations cluster_name_ops = {
	.open		= cluster_name_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif

static void _setup_debugfs(void)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *rootdir;

	rootdir = debugfs_create_dir("tegra_cluster", NULL);
	if (rootdir) {
		debugfs_create_file("current_cluster", S_IRUGO | S_IWUSR,
					rootdir, NULL, &cluster_ops);
		debugfs_create_file("current_cluster_name", S_IRUGO,
					rootdir, NULL, &cluster_name_ops);
	}
#endif
}

int __init tegra210_cluster_control_init(void)
{
	pg_core_arg = psci_power_state_pack(core_pg);
	pg_cluster_arg = psci_power_state_pack(cluster_pg);

	_setup_debugfs();

	pr_info("Tegra210 cluster control initialized\n");

	return 0;
}

late_initcall(tegra210_cluster_control_init);
