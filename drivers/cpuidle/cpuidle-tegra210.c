/*
 * drivers/cpuidle/cpuidle-t210.c
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/cpu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpuidle.h>
#include <linux/of_platform.h>
#include <soc/tegra/tegra_bpmp.h>
#include <linux/tegra-soc.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/tegra-pm.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-pmc.h>
#include <linux/tegra_smmu.h>
#include <linux/tegra-timer.h>
#include <linux/rtc-tegra.h>
#include <linux/syscore_ops.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/tick.h>
#include <linux/irq.h>
#include <linux/tegra_cluster_control.h>
#include <linux/regulator/consumer.h>
#include "../../kernel/irq/internals.h"

#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <asm/psci.h>

#include <linux/platform/tegra/flowctrl.h>
#include "cpuidle-tegra210.h"

#define MRQ_CPU_FROZEN	13

static const char *driver_name = "tegra210_idle";
static struct module *owner = THIS_MODULE;

static DEFINE_PER_CPU(struct cpumask, idle_mask);
static DEFINE_PER_CPU(struct cpuidle_driver, cpuidle_drv);

struct t210_idle_state_params {
	unsigned int residency;
	unsigned int latency;
	bool enabled;
};

struct t210_idle_state {
	struct t210_idle_state_params cluster[2];
	int (*enter) (struct cpuidle_device *dev, struct cpuidle_driver *drv,
			int index);
};

static int tegra_bpmp_do_idle(int cpu, int ccxtl, int scxtl)
{
	int32_t tl;
	int32_t data[3];

	data[0] = cpu_to_le32(cpu);
	data[1] = cpu_to_le32(ccxtl);
	data[2] = cpu_to_le32(scxtl);

	return tegra_bpmp_send_receive_atomic(MRQ_DO_IDLE, data, sizeof(data),
			&tl, sizeof(tl)) ?: tl;
}

static int tegra_bpmp_tolerate_idle(int cpu, int ccxtl, int scxtl)
{
	int32_t data[3];

	data[0] = cpu_to_le32(cpu);
	data[1] = cpu_to_le32(ccxtl);
	data[2] = cpu_to_le32(scxtl);

	return tegra_bpmp_send(MRQ_TOLERATE_IDLE, data, sizeof(data));
}

static int csite_dbg_nopwrdown(void)
{
	u64 csite_dbg;

	asm volatile("mrs %0, dbgprcr_el1" : "=r" (csite_dbg));

	return csite_dbg & 0x1;
}

static int tegra210_enter_hvc(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	/* TODO: fix the counter */
	flowctrl_write_cc4_ctrl(dev->cpu, 0xfffffffd);
	cpu_do_idle();
	flowctrl_write_cc4_ctrl(dev->cpu, 0);

	return index;
}

static int tegra210_enter_retention(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	/* TODO: fix the counter */
	flowctrl_write_cc4_ctrl(dev->cpu, 0xffffffff);
	cpu_do_idle();
	flowctrl_write_cc4_ctrl(dev->cpu, 0);

	return index;
}

/* TODO: fix the thresholds */
static void do_cc4_init(void)
{
	flowctrl_update(FLOW_CTLR_CC4_HVC_CONTROL,
			2 << 3 | FLOW_CTRL_CC4_HVC_ENABLE);
	flowctrl_update(FLOW_CTRL_CC4_RETENTION_CONTROL, 2 << 3);
	flowctrl_update(FLOW_CTRL_CC4_HVC_RETRY, 2);
}

static struct syscore_ops tegra210_syscore_ops = {
	.restore = do_cc4_init,
	.resume = do_cc4_init
};

static void __tegra210_enter_c7(int cpu)
{
	struct psci_power_state ps = {
		.id = TEGRA210_CPUIDLE_C7,
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 1,
	};
	unsigned long arg = psci_power_state_pack(ps);

	cpu_pm_enter();

	if (!is_lp_cluster()) {
		flowctrl_write_cc4_ctrl(cpu, 0xffffffff);
	}

	cpu_suspend(arg, NULL);
	flowctrl_write_cc4_ctrl(cpu, 0);
	cpu_pm_exit();
}

/*
 * tegra210_enter_c7 - Programs CPU to enter power gate state
 *
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @idx: state index
 */
static int tegra210_enter_c7(struct cpuidle_device *dev,
		struct cpuidle_driver *drv,
		int idx)
{
	__tegra210_enter_c7(dev->cpu);
	return idx;
}

static int c7_index;

static int tegra210_enter_cc_state(struct cpuidle_device *dev,
		int cc_state_tolerance, int sc_state_tolerance,
		int state_id, int idx)
{
	struct psci_power_state ps = {
		.id = state_id,
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 1,
	};
	unsigned long arg = psci_power_state_pack(ps);

	if (tegra_bpmp_do_idle(dev->cpu, cc_state_tolerance,
			sc_state_tolerance)) {
		__tegra210_enter_c7(dev->cpu);
		return c7_index;
	}

	cpu_pm_enter();
	cpu_suspend(arg, NULL);
	cpu_pm_exit();

	return idx;
}

static int tegra210_enter_cc6(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	return tegra210_enter_cc_state(dev, TEGRA_PM_CC6, TEGRA_PM_SC1,
					TEGRA210_CPUIDLE_CC6, idx);
}

static int tegra210_enter_cc7(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	return tegra210_enter_cc_state(dev, TEGRA_PM_CC7, TEGRA_PM_SC1,
					TEGRA210_CPUIDLE_CC7, idx);
}

static int tegra210_enter_sc2(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int idx)
{
	tegra_bpmp_tolerate_idle(dev->cpu, TEGRA_PM_CC4, TEGRA_PM_SC2);
	__tegra210_enter_c7(dev->cpu);
	tegra_bpmp_tolerate_idle(dev->cpu, TEGRA_PM_CC1, TEGRA_PM_SC1);
	return idx;
}

static int tegra210_enter_sc3(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int idx)
{
	tegra_bpmp_tolerate_idle(dev->cpu, TEGRA_PM_CC4, TEGRA_PM_SC3);
	__tegra210_enter_c7(dev->cpu);
	tegra_bpmp_tolerate_idle(dev->cpu, TEGRA_PM_CC1, TEGRA_PM_SC1);
	return idx;
}

static int tegra210_enter_sc4(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int idx)
{
	if (csite_dbg_nopwrdown())
		return 0;
	tegra_bpmp_tolerate_idle(dev->cpu, TEGRA_PM_CC4, TEGRA_PM_SC4);
	__tegra210_enter_c7(dev->cpu);
	tegra_bpmp_tolerate_idle(dev->cpu, TEGRA_PM_CC1, TEGRA_PM_SC1);
	return idx;
}

static bool restore_sc7 = false;

static int tegra210_enter_sc7(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	int err = idx;
	int cpu;
	unsigned long arg;
	u64 sleep_time = ULONG_MAX;
	u64 next_event;

	/* Assume C7 config by default */
	struct psci_power_state ps = {
		.id = 0,
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 0,
	};

	if (csite_dbg_nopwrdown())
		return 0;

	if (!tegra_bpmp_do_idle(dev->cpu, TEGRA_PM_CC7,
					TEGRA_PM_SC7)) {

		for_each_online_cpu(cpu) {
			err = tegra210_timer_get_remain(dev->cpu, &next_event);
			if (err)
				continue;

			if (next_event < sleep_time)
				sleep_time = next_event;
		}

		/*
		 * We are the last core standing and bpmp says GO.
		 * Change to CCx config.
		 */
		ps.id = TEGRA210_CPUIDLE_SC7;
		ps.affinity_level = 2;

		tegra_rtc_set_trigger(sleep_time);

		/* This state is managed by power domains, hence no voice call expected if
		 * we are entering this state */
		tegra_pm_notifier_call_chain(TEGRA_PM_SUSPEND);

		tegra_actmon_save();

		tegra_dma_save();

		tegra_smmu_save();

		err = syscore_save();

		err = tegra_pm_prepare_sc7();

		restore_sc7 = true;
	}

	if (ps.id == 0)
		cpu_pm_enter();

	arg = psci_power_state_pack(ps);
	cpu_suspend(arg, NULL);

	if (!restore_sc7) {
		cpu_pm_exit();
		return err;
	}

	restore_sc7 = false;

	tegra_pm_post_sc7();

	syscore_restore();

	tegra_smmu_restore();

	tegra_dma_restore();

	tegra_actmon_restore();

	tegra_rtc_set_trigger(0);

	tegra_pm_notifier_call_chain(TEGRA_PM_RESUME);

	return err;
}

static int tegra210_enter_cg(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	cpu_do_idle();

	return idx;
}

/* Assuming this array and the states array in the driver are kept in sync */
static struct t210_idle_state t210_idle_states[CPUIDLE_STATE_MAX] = {
	[0] = { /* C1 */
		.cluster = {
			[0] = {
				.residency = 1,
				.latency = 1,
				.enabled = true,
			},
			[1] = {
				.residency = 1,
				.latency = 1,
				.enabled = true,
			},
		},
		.enter = tegra210_enter_cg,
	},
	[1] = { /* C3 */
		.cluster = {
			[0] = {
				.residency = 1,
				.latency = 10,
			},
			[1] = {
				.residency = 1,
				.latency = 10,
			},
		},
		.enter = tegra210_enter_hvc,
	},
	[2] = { /* C4 */
		.cluster = {
			[0] = {
				.residency = 500,
				.latency = 140,
			},
			[1] = {
				.residency = 500,
				.latency = 140,
			},
		},
		.enter = tegra210_enter_retention,
	},
	[3] = { /* C7 */
		.cluster = {
			[0] = {
				.residency = 1000,
				.latency = 130,
			},
			[1] = {
				.residency = 1000,
				.latency = 130,
			},
		},
		.enter = tegra210_enter_c7,
	},
	[4] = { /* CC6 */
		.cluster = {
			[0] = {
				.residency = 10000,
				.latency = 230,
			},
			[1] = {
				.residency = 10000,
				.latency = 230,
			},
		},
		.enter = tegra210_enter_cc6,
	},
	[5] = { /* CC7 */
		.cluster = {
			[0] = {
				.residency = 1000,
				.latency = 500,
			},
			[1] = {
				.residency = 1000,
				.latency = 500,
			},
		},
		.enter = tegra210_enter_cc7,
	},
	[6] = { /* SC2 */
		.cluster = {
			[0] = {
				.residency = 1500,
				.latency = 600,
			},
			[1] = {
				.residency = 1500,
				.latency = 600,
			},
		},
		.enter = tegra210_enter_sc2,
	},
	[7] = { /* SC3 */
		.cluster = {
			[0] = {
				.residency = 1600,
				.latency = 700,
			},
			[1] = {
				.residency = 1600,
				.latency = 700,
			},
		},
		.enter = tegra210_enter_sc3,
	},
	[8] = { /* SC4 */
		.cluster = {
			[0] = {
				.residency = 5000,
				.latency = 3000,
			},
			[1] = {
				.residency = 5000,
				.latency = 3000,
			},
		},
		.enter = tegra210_enter_sc4,
	},
	[9] = { /* SC7 */
		.cluster = {
			[0] = {
				.residency = 10000,
				.latency = 5000,
			},
			[1] = {
				.residency = 10000,
				.latency = 5000,
			},
		},
		.enter = tegra210_enter_sc7,
	},
};

/* XXX: Call on cluster switch */
static void tegra210_set_idle_state_params(int cpu)
{
	int i;
	unsigned int target_residency, exit_latency;
	struct cpuidle_driver *drv = &per_cpu(cpuidle_drv, cpu);
	bool is_slow_cluster = is_lp_cluster();

	for (i = 0; i < drv->state_count; i++) {
		target_residency =
			t210_idle_states[i].cluster[is_slow_cluster].residency;
		exit_latency =
			t210_idle_states[i].cluster[is_slow_cluster].latency;
		drv->states[i].target_residency = target_residency;
		drv->states[i].exit_latency = exit_latency;
	}
}

static int tegra210_enter_state(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	bool is_slow_cluster = is_lp_cluster();
	bool enabled = false;
	int i;

	for (i = index; i > 0; i--) {
		enabled = t210_idle_states[i].cluster[is_slow_cluster].enabled;
		if (enabled && !dev->states_usage[i].disable &&
		    !drv->states[i].disabled)
			break;
	}

	return t210_idle_states[i].enter(dev, drv, i);
}

static int tegra210_cpuidle_register(int cpu)
{
	int ret;
	struct cpuidle_driver *drv;
	struct cpuidle_state *state;

	drv = &per_cpu(cpuidle_drv, cpu);
	drv->name = driver_name;
	drv->owner = owner;
	drv->cpumask = &per_cpu(idle_mask, cpu);
	cpumask_set_cpu(cpu, drv->cpumask);

	state = &drv->states[0];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C1");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU clock gated");
	state->enter = tegra210_enter_state;
	state->power_usage = UINT_MAX;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[1];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "HW controlled Vmin");
	state->enter = tegra210_enter_state;
	state->power_usage = 5000;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[2];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C4");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU retention");
	state->enter = tegra210_enter_state;
	state->power_usage = 5000;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[3];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C7");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU Core Powergate");
	state->enter = tegra210_enter_state;
	state->power_usage = 500;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	c7_index = 3;

	state = &drv->states[4];
	snprintf(state->name, CPUIDLE_NAME_LEN, "CC6");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "Cluster power gate");
	state->enter = tegra210_enter_state;
	state->power_usage = 400;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[5];
	snprintf(state->name, CPUIDLE_NAME_LEN, "CC7");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU rail gate");
	state->enter = tegra210_enter_state;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[6];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC2");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "DRAM SR + MC CG");
	state->enter = tegra210_enter_state;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[7];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "DRAM SR + MC CG + Memory PLL disabled");
	state->enter = tegra210_enter_state;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[8];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC4");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "DRAM SR + MC CG + Memory PLL & PLLP disabled");
	state->enter = tegra210_enter_state;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[9];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC7");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "VDD_SOC turned off");
	state->enter = tegra210_enter_state;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	drv->state_count = 10;

	tegra210_set_idle_state_params(cpu);

	ret = cpuidle_register(drv, NULL);
	if (ret) {
		pr_err("%s: Tegra210 cpuidle driver registration failed: %d",
		       __func__, ret);
		return ret;
	}

	return 0;
}

/* Change CPU tolerance level according to hotplug state */
static int tegra210_cpu_notify(struct notifier_block *nb, unsigned long action,
		void *data)
{
	int e;
	int cpu = (long)data;

	switch (action) {
	case CPU_POST_DEAD:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_PM_CC7, TEGRA_PM_SC7);
		break;
	case CPU_DEAD_FROZEN:
		e = tegra_bpmp_send(MRQ_CPU_FROZEN, &cpu, sizeof(cpu));
		WARN_ON(e);
		break;
	case CPU_UP_PREPARE:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_PM_CC1, TEGRA_PM_SC1);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block tegra210_cpu_nb = {
	.notifier_call = tegra210_cpu_notify
};

#ifdef CONFIG_DEBUG_FS

static struct dentry *cpuidle_debugfs_root;
static unsigned int fast_enable = 1;
static unsigned int slow_enable = 1;

static void set_state_enable(bool slow_cluster)
{
	int i;
	bool enabled;

	for (i = 1; i < ARRAY_SIZE(t210_idle_states); i++) {
		if (slow_cluster) {
			/* CC3 is not supported on slow cluster */
			if (i == 1)
				enabled = false;
			else
				enabled = (slow_enable >> i) & 0x1;
		} else
			enabled = (fast_enable >> i) & 0x1;

		t210_idle_states[i].cluster[slow_cluster].enabled = enabled;
	}
}

static int fast_enable_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%u\n", fast_enable);

	return 0;
}

static int fast_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, fast_enable_show, inode->i_private);
}

static ssize_t fast_enable_write(struct file *fp, const char __user *ubuf,
					size_t count, loff_t *pos)
{
	if (kstrtouint_from_user(ubuf, count, 0, &fast_enable) < 0)
		return -EINVAL;

	set_state_enable(false);

	return count;
}

static const struct file_operations fast_cluster_enable_fops = {
	.open	=	fast_enable_open,
	.read	=	seq_read,
	.llseek	=	seq_lseek,
	.write	=	fast_enable_write,
	.release =	single_release,
};

static int slow_enable_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%u\n", slow_enable);

	return 0;
}

static int slow_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, slow_enable_show, inode->i_private);
}

static ssize_t slow_enable_write(struct file *fp, const char __user *ubuf,
					size_t count, loff_t *pos)
{
	if (kstrtouint_from_user(ubuf, count, 0, &slow_enable) < 0)
		return -EINVAL;

	set_state_enable(true);

	return count;
}

static const struct file_operations slow_cluster_enable_fops = {
	.open	=	slow_enable_open,
	.read	=	seq_read,
	.llseek	=	seq_lseek,
	.write	=	slow_enable_write,
	.release =	single_release,
};

static bool is_timer_irq(struct irq_desc *desc)
{
	return desc && desc->action && (desc->action->flags & IRQF_TIMER);
}

static void suspend_all_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		/* Don't disable the 'wakeup' interrupt */
		if (is_timer_irq(desc))
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__disable_irq(desc, irq, false);
		desc->istate |= IRQS_SUSPENDED;
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}

	for_each_irq_desc(irq, desc) {
		if (is_timer_irq(desc))
			continue;

		if (desc->istate & IRQS_SUSPENDED)
			synchronize_irq(irq);
	}
}

static void resume_all_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		if (is_timer_irq(desc))
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__enable_irq(desc, irq, true);
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

static struct dentry *cpuidle_debugfs_root;
static u64 idle_state;

static int idle_write(void *data, u64 val)
{
	struct cpuidle_device dev;
	struct cpuidle_driver *drv;
	unsigned long timer_interval_us = (ulong)val;
	ktime_t time, interval, sleep;

	preempt_disable();
	drv = &per_cpu(cpuidle_drv, smp_processor_id());

	if (idle_state >= drv->state_count) {
		pr_err("%s: Requested invalid forced idle state\n", __func__);
		preempt_enable_no_resched();
		return -EINVAL;
	}

	memset(&dev, 0, sizeof(dev));

	dev.cpu = smp_processor_id();
	suspend_all_device_irqs();
	tick_nohz_idle_enter();
	stop_critical_timings();
	local_fiq_disable();
	local_irq_disable();

	interval = ktime_set(0, (NSEC_PER_USEC * timer_interval_us));

	time = ktime_get();
	sleep = ktime_add(time, interval);
	tick_program_event(sleep, true);

	t210_idle_states[idle_state].enter(&dev, drv, idle_state);

	sleep = ktime_sub(ktime_get(), time);
	time = ktime_sub(sleep, interval);
	trace_printk("idle: %lld, exit latency: %lld\n", sleep.tv64, time.tv64);

	local_irq_enable();
	local_fiq_enable();
	start_critical_timings();
	tick_nohz_idle_exit();
	resume_all_device_irqs();
	preempt_enable_no_resched();

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(duration_us_fops, NULL, idle_write, "%llu\n");

static int debugfs_init(void)
{
	struct dentry *dfs_file;

	cpuidle_debugfs_root = debugfs_create_dir("cpuidle_t210", NULL);

	if (!cpuidle_debugfs_root)
		return -ENOMEM;

	dfs_file = debugfs_create_file("fast_cluster_states_enable", 0644,
			cpuidle_debugfs_root, NULL, &fast_cluster_enable_fops);
	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("slow_cluster_states_enable", 0644,
			cpuidle_debugfs_root, NULL, &slow_cluster_enable_fops);

	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_u64("forced_idle_state", 0644,
			cpuidle_debugfs_root, &idle_state);

	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("forced_idle_duration_us", 0644,
			cpuidle_debugfs_root, NULL, &duration_us_fops);

	if (!dfs_file)
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(cpuidle_debugfs_root);
	return -ENOMEM;
}
#else
static inline int debugfs_init(void) { return 0; }
#endif

static int tegra210_cc4_volt_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator *reg;
	uint32_t uv;
	int r;

	/* If cc4-microvolt is not found, assume not max77621 */
	if (of_property_read_u32(dev->of_node, "cc4-microvolt", &uv))
		return 0;

	reg = regulator_get(dev, "vdd-cpu");
	if (IS_ERR(reg)) {
		dev_err(dev, "vdd-cpu regulator get failed\n");
		return PTR_ERR(reg);
	}

	r = regulator_set_sleep_voltage(reg, uv - 100000, uv + 100000);
	if (r) {
		dev_err(dev, "failed to set retention voltage: %d\n", r);
		goto err_out;
	}

	dev_info(dev, "retention voltage is %u uv\n", uv);
	return 0;

err_out:
	WARN_ON(0);
	return 0;
}

static int tegra210_cpuidle_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int cpu;

	pr_info("Tegra210 cpuidle driver\n");
	ret = tegra210_cc4_volt_init(pdev);
	if (ret == -EPROBE_DEFER) {
		pr_info("Tegra210 cpuidle driver triggering delayed probe\n");
		return ret;
	}

	do_cc4_init();
	register_syscore_ops(&tegra210_syscore_ops);

	for_each_possible_cpu(cpu) {
		ret = tegra210_cpuidle_register(cpu);
		if (ret)
			return ret;
	}

	debugfs_init();
	return register_cpu_notifier(&tegra210_cpu_nb);
}

static const struct of_device_id tegra210_cpuidle_of[] = {
	{ .compatible = "nvidia,tegra210-cpuidle" },
	{}
};

static struct platform_driver tegra210_cpuidle_driver = {
	.probe = tegra210_cpuidle_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cpuidle",
		.of_match_table = of_match_ptr(tegra210_cpuidle_of)
	}
};

static __init int tegra210_cpuidle_init(void)
{
	return platform_driver_register(&tegra210_cpuidle_driver);
}
device_initcall(tegra210_cpuidle_init);
