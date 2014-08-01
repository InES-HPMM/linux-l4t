/*
 * drivers/cpuidle/cpuidle-t210.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform_data/tegra_bpmp.h>
#include <linux/tegra-soc.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/tegra-pm.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-pmc.h>
#include <linux/tegra_smmu.h>
#include <linux/tegra-timer.h>
#include <linux/syscore_ops.h>

#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <asm/psci.h>

#include "../../arch/arm/mach-tegra/flowctrl.h"
#include "cpuidle-tegra210.h"

/* Core state 0-9 */
#define TEGRA210_CPUIDLE_C4	4
#define TEGRA210_CPUIDLE_C7	7

/* Cluster states 10-19 */
#define TEGRA210_CPUIDLE_CC4	14
#define TEGRA210_CPUIDLE_CC6	16
#define TEGRA210_CPUIDLE_CC7	17

/* SoC states 20-29 */
#define TEGRA210_CPUIDLE_SC2	22
#define TEGRA210_CPUIDLE_SC3	23
#define TEGRA210_CPUIDLE_SC4	24
#define TEGRA210_CPUIDLE_SC7	27

#define TEGRA210_CLUSTER_SWITCH	31

static const char *driver_name = "tegra210_idle";
static struct module *owner = THIS_MODULE;

static DEFINE_PER_CPU(struct cpumask, idle_mask);
static DEFINE_PER_CPU(struct cpuidle_driver, cpuidle_drv);

static int tegra210_enter_hvc(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	/* TODO: fix the counter */
	flowctrl_write_cc4_ctrl(dev->cpu, 0xfffffffd);
	cpu_retention_enable(7);

	cpu_do_idle();

	cpu_retention_enable(0);
	flowctrl_write_cc4_ctrl(dev->cpu, 0);

	return index;
}

static int tegra210_enter_retention(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int index)
{
	/* TODO: fix the counter */
	flowctrl_write_cc4_ctrl(dev->cpu, 0xffffffff);
	cpu_retention_enable(7);

	cpu_do_idle();

	cpu_retention_enable(0);
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

/*
 * tegra210_enter_c7 - Programs CPU to enter power gate state
 *
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @idx: state index
 *
 */
static int tegra210_enter_c7(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	unsigned long arg;
	struct psci_power_state ps = {
		.id = TEGRA210_CPUIDLE_C7,
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 0,
	};

	cpu_pm_enter();
	arg = psci_power_state_pack(ps);
	cpu_suspend(arg, NULL);
	cpu_pm_exit();

	return idx;
}

static int tegra210_enter_cc_state(struct cpuidle_device *dev,
			int cc_state_tolerance, int sc_state_tolerance,
			int state_id, int idx)
{
	unsigned long arg;

	/* Assume C7 config by default */
	struct psci_power_state ps = {
		.id = TEGRA210_CPUIDLE_C7,
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 0,
	};

	cpu_pm_enter();

	if (!tegra_bpmp_do_idle(dev->cpu, cc_state_tolerance,
					sc_state_tolerance)) {
		/*
		 * We are the last core standing and bpmp says GO.
		 * Change to CCx config.
		 */
		ps.id = state_id;
		ps.affinity_level = 1;
	}

	arg = psci_power_state_pack(ps);
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
				struct cpuidle_driver *drv,
				int idx)
{
	return tegra210_enter_cc_state(dev, TEGRA_PM_CC6, TEGRA_PM_SC2,
					TEGRA210_CPUIDLE_CC6, idx);
}

static int tegra210_enter_sc3(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	return tegra210_enter_cc_state(dev, TEGRA_PM_CC6, TEGRA_PM_SC3,
					TEGRA210_CPUIDLE_CC6, idx);
}

static int tegra210_enter_sc4(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	return tegra210_enter_cc_state(dev, TEGRA_PM_CC6, TEGRA_PM_SC4,
					TEGRA210_CPUIDLE_CC6, idx);
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

/* TODO: remove this WAR */
static void tegra210_init_usage(int cpu)
{
	struct cpuidle_device *dev = per_cpu(cpuidle_devices, cpu);
	int i;

	if (!dev)
		return;

	for (i = 0; i < CPUIDLE_STATE_MAX; i++)
		dev->states_usage[i].disable = 1;
}

static int __init tegra210_cpuidle_register(int cpu)
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
	state->enter = tegra210_enter_cg;
	state->exit_latency = 1;
	state->target_residency = 1;
	state->power_usage = UINT_MAX;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[1];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "HW controlled Vmin");
	state->enter = tegra210_enter_hvc;
	state->exit_latency = 10;
	state->target_residency = 10;
	state->power_usage = 5000;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[2];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C4");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU retention");
	state->enter = tegra210_enter_retention;
	state->exit_latency = 10;
	state->target_residency = 10;
	state->power_usage = 5000;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[3];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C7");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU Core Powergate");
	state->enter = tegra210_enter_c7;
	state->exit_latency = 10;
	state->target_residency = 20;
	state->power_usage = 500;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[4];
	snprintf(state->name, CPUIDLE_NAME_LEN, "CC6");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "Cluster power gate");
	state->enter = tegra210_enter_cc6;
	state->exit_latency = 20;
	state->target_residency = 30;
	state->power_usage = 400;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[5];
	snprintf(state->name, CPUIDLE_NAME_LEN, "CC7");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU rail gate");
	state->enter = tegra210_enter_cc7;
	state->exit_latency = 30;
	state->target_residency = 40;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[6];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC2");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "DRAM SR + MC CG");
	state->enter = tegra210_enter_sc2;
	state->exit_latency = 30;
	state->target_residency = 40;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[7];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC3");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "DRAM SR + MC CG + Memory PLL disabled");
	state->enter = tegra210_enter_sc3;
	state->exit_latency = 30;
	state->target_residency = 40;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[8];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC4");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "DRAM SR + MC CG + Memory PLL & PLLP disabled");
	state->enter = tegra210_enter_sc4;
	state->exit_latency = 30;
	state->target_residency = 40;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[9];
	snprintf(state->name, CPUIDLE_NAME_LEN, "SC7");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "VDD_SOC turned off");
	state->enter = tegra210_enter_sc7;
	state->exit_latency = 30;
	state->target_residency = 40;
	state->power_usage = 300;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	drv->state_count = 10;

	ret = cpuidle_register(drv, NULL);
	if (ret) {
		pr_err("%s: Tegra210 cpuidle driver registration failed: %d",
		       __func__, ret);
		return ret;
	}

	tegra210_init_usage(cpu);

	return 0;
}

/* Change CPU tolerance level according to hotplug state */
static int tegra210_cpu_notify(struct notifier_block *nb, unsigned long action,
		void *data)
{
	int cpu = (long)data;

	switch (action) {
	case CPU_POST_DEAD:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_PM_CC7);
		break;
	case CPU_UP_PREPARE:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_PM_CC1);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block tegra210_cpu_nb = {
	.notifier_call = tegra210_cpu_notify
};

/*
 * t210_idle_init
 *
 */
int __init tegra210_idle_init(void)
{
	int ret;
	unsigned int cpu;

	pr_info("Tegra210 cpuidle driver\n");
	if (!of_machine_is_compatible("nvidia,tegra210")) {
		pr_err("%s: not on T210\n", __func__);
		return -ENODEV;
	}

	do_cc4_init();

	for_each_possible_cpu(cpu) {
		ret = tegra210_cpuidle_register(cpu);
		if (ret)
			return ret;
	}

	return register_cpu_notifier(&tegra210_cpu_nb);
}
device_initcall(tegra210_idle_init);
