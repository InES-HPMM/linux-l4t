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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpuidle.h>
#include <linux/of_platform.h>
#include <linux/tegra-soc.h>
#include <asm/proc-fns.h>
#include "../../arch/arm/mach-tegra/flowctrl.h"
#include "cpuidle-tegra210.h"

static bool retention_in_idle __read_mostly;
module_param(retention_in_idle, bool, 0644);
static const char *driver_name = "tegra210_idle";
static struct module *owner = THIS_MODULE;

static DEFINE_PER_CPU(struct cpumask, idle_mask);
static DEFINE_PER_CPU(struct cpuidle_driver, cpuidle_drv);

static int cpu_do_c4(struct cpuidle_device *dev, struct cpuidle_driver *drv,
		int index)
{
	if (!retention_in_idle) {
		cpu_do_idle();
		return 0;
	}

	/* TODO: fix the counter */
	flowctrl_write_cc4_ctrl(dev->cpu, 0xffffffff);
	cpu_retention_enable(1);

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
 * tegra210_enter_state - Programs CPU to enter the specified state
 *
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @idx: state index
 *
 */
static int tegra210_enter_state(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int idx)
{
	if (!idx) {
		cpu_do_idle();
		return idx;
	}

	return idx;
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
	state->enter = tegra210_enter_state;
	state->exit_latency = 1;
	state->target_residency = 1;
	state->power_usage = UINT_MAX;
	state->flags = CPUIDLE_FLAG_TIME_VALID;

	state = &drv->states[1];
	snprintf(state->name, CPUIDLE_NAME_LEN, "C4");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPU retention");
	state->enter = cpu_do_c4;
	state->exit_latency = 10;
	state->target_residency = 10;
	state->power_usage = 5000;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->disabled = true;

	drv->state_count = 2;

	ret = cpuidle_register(drv, NULL);
	if (ret) {
		pr_err("%s: Tegra210 cpuidle driver registration failed: %d",
		       __func__, ret);
		return ret;
	}

	return 0;
}

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

	return 0;
}
device_initcall(tegra210_idle_init);
