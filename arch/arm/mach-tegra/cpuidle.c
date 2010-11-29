/*
 * arch/arm/mach-tegra/cpuidle.c
 *
 * CPU idle driver for Tegra CPUs
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
 * Copyright (c) 2011 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
 *         Gary King <gking@nvidia.com>
 *
 * Rework for 3.3 by Peter De Schrijver <pdeschrijver@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/tick.h>

#include <asm/proc-fns.h>

#include <mach/irqs.h>

#include "pm.h"
#include "sleep.h"

static int tegra_idle_enter_lp3(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index);
static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index);

#define TEGRA_CPUIDLE_BOTH_IDLE		INT_QUAD_RES_24
#define TEGRA_CPUIDLE_TEAR_DOWN		INT_QUAD_RES_25

static bool lp2_in_idle __read_mostly = true;
module_param(lp2_in_idle, bool, 0644);

static struct {
	unsigned int cpu_ready_count[2];
	unsigned long long cpu_wants_lp2_time[2];
	unsigned long long in_lp2_time;
	unsigned int both_idle_count;
	unsigned int tear_down_count;
	unsigned int lp2_count;
	unsigned int lp2_count_bin[32];
	unsigned int lp2_int_count[NR_IRQS];
	unsigned int last_lp2_int_count[NR_IRQS];
} idle_stats;

struct cpuidle_driver tegra_idle_driver = {
	.name = "tegra_idle",
	.owner = THIS_MODULE,
	.state_count = 1,
	.states = {
		[0] = {
			.enter			= tegra_idle_enter_lp3,
			.exit_latency		= 10,
			.target_residency	= 10,
			.power_usage		= 600,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.name			= "LP3",
			.desc			= "CPU flow-controlled",
		},
		[1] = {
			.enter			= tegra_idle_enter_lp2,
			.power_usage		= 0,
			.flags			= CPUIDLE_FLAG_TIME_VALID,
			.name			= "LP2",
			.desc			= "CPU power-gate",
		},
	},
};

static DEFINE_PER_CPU(struct cpuidle_device, tegra_idle_device);

static inline unsigned int time_to_bin(unsigned int time)
{
	return fls(time);
}

static int tegra_idle_enter_lp3(struct cpuidle_device *dev,
	struct cpuidle_driver *drv, int index)
{
	ktime_t enter, exit;
	s64 us;

	local_irq_disable();
	local_fiq_disable();

	enter = ktime_get();

	cpu_do_idle();

	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_fiq_enable();
	local_irq_enable();

	dev->last_residency = us;

	return index;
}

static int tegra_idle_enter_lp2(struct cpuidle_device *dev,
	struct cpuidle_driver *drv, int index)
{
	ktime_t enter, exit;
	s64 us;

	local_irq_disable();
	local_fiq_disable();
	enter = ktime_get();

	idle_stats.cpu_ready_count[dev->cpu]++;

	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &dev->cpu);
	tegra_idle_lp2();
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &dev->cpu);

	exit = ktime_sub(ktime_get(), enter);
	us = ktime_to_us(exit);

	local_fiq_enable();
	local_irq_enable();

	smp_rmb();

	idle_stats.cpu_wants_lp2_time[dev->cpu] += us;

	dev->last_residency = us;

	return index;
}

static int __init tegra_cpuidle_init(void)
{
	int ret;
	unsigned int cpu;
	struct cpuidle_device *dev;
	struct cpuidle_driver *drv = &tegra_idle_driver;

	tegra_idle_driver.states[1].exit_latency = tegra_cpu_power_good_time();
	tegra_idle_driver.states[1].target_residency = tegra_cpu_power_off_time() +
		tegra_cpu_power_good_time();

	ret = cpuidle_register_driver(&tegra_idle_driver);
	if (ret) {
		pr_err("CPUidle driver registration failed\n");
		return ret;
	}

	for_each_possible_cpu(cpu) {
		dev = &per_cpu(tegra_idle_device, cpu);
		dev->cpu = cpu;

		dev->state_count = drv->state_count;
		ret = cpuidle_register_device(dev);
		if (ret) {
			pr_err("CPU%u: CPUidle device registration failed\n",
				cpu);
			return ret;
		}
	}
	return 0;
}
device_initcall(tegra_cpuidle_init);

#ifdef CONFIG_DEBUG_FS
static int tegra_lp2_debug_show(struct seq_file *s, void *data)
{
	int bin;
	int i;
	seq_printf(s, "                                    cpu0     cpu1\n");
	seq_printf(s, "-------------------------------------------------\n");
	seq_printf(s, "cpu ready:                      %8u %8u\n",
		idle_stats.cpu_ready_count[0],
		idle_stats.cpu_ready_count[1]);
	seq_printf(s, "both idle:      %8u        %7u%% %7u%%\n",
		idle_stats.both_idle_count,
		idle_stats.both_idle_count * 100 /
			(idle_stats.cpu_ready_count[0] ?: 1),
		idle_stats.both_idle_count * 100 /
			(idle_stats.cpu_ready_count[1] ?: 1));
	seq_printf(s, "tear down:      %8u %7u%%\n", idle_stats.tear_down_count,
		idle_stats.tear_down_count * 100 /
			(idle_stats.both_idle_count ?: 1));
	seq_printf(s, "lp2:            %8u %7u%%\n", idle_stats.lp2_count,
		idle_stats.lp2_count * 100 /
			(idle_stats.both_idle_count ?: 1));

	seq_printf(s, "\n");
	seq_printf(s, "cpu ready time:                 %8llu %8llu ms\n",
		div64_u64(idle_stats.cpu_wants_lp2_time[0], 1000),
		div64_u64(idle_stats.cpu_wants_lp2_time[1], 1000));
	seq_printf(s, "lp2 time:       %8llu ms     %7d%% %7d%%\n",
		div64_u64(idle_stats.in_lp2_time, 1000),
		(int)div64_u64(idle_stats.in_lp2_time * 100,
			idle_stats.cpu_wants_lp2_time[0] ?: 1),
		(int)div64_u64(idle_stats.in_lp2_time * 100,
			idle_stats.cpu_wants_lp2_time[1] ?: 1));

	seq_printf(s, "\n");
	seq_printf(s, "%19s %8s\n", "", "lp2");
	seq_printf(s, "-------------------------------------------------\n");
	for (bin = 0; bin < 32; bin++) {
		if (idle_stats.lp2_count_bin[bin] == 0)
			continue;
		seq_printf(s, "%6u - %6u ms: %8u\n",
			1 << (bin - 1), 1 << bin,
			idle_stats.lp2_count_bin[bin]);
	}

	seq_printf(s, "\n");
	seq_printf(s, "%3s %20s %6s %10s\n",
		"int", "name", "count", "last count");
	seq_printf(s, "--------------------------------------------\n");
	for (i = 0; i < NR_IRQS; i++) {
		if (idle_stats.lp2_int_count[i] == 0)
			continue;
		seq_printf(s, "%3d %20s %6d %10d\n",
			i, irq_to_desc(i)->action ?
				irq_to_desc(i)->action->name ?: "???" : "???",
			idle_stats.lp2_int_count[i],
			idle_stats.lp2_int_count[i] -
				idle_stats.last_lp2_int_count[i]);
		idle_stats.last_lp2_int_count[i] = idle_stats.lp2_int_count[i];
	};
	return 0;
}

static int tegra_lp2_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra_lp2_debug_show, inode->i_private);
}

static const struct file_operations tegra_lp2_debug_ops = {
	.open		= tegra_lp2_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_cpuidle_debug_init(void)
{
	struct dentry *dir;
	struct dentry *d;

	dir = debugfs_create_dir("cpuidle", NULL);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file("lp2", S_IRUGO, dir, NULL,
		&tegra_lp2_debug_ops);
	if (!d)
		return -ENOMEM;

	return 0;
}

late_initcall(tegra_cpuidle_debug_init);
#endif
