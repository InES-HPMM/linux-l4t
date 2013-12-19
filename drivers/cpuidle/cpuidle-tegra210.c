/*
 * drivers/cpuidle/cpuidle-t210.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

struct cpuidle_driver tegra210_idle_driver = {
	.name = "tegra210_idle",
	.owner = THIS_MODULE,
	.states[0] = {
		.enter = tegra210_enter_state,
		.exit_latency = 1,
		.target_residency = 1,
		.power_usage = UINT_MAX,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.name = "C1",
		.desc = "CPU clock gated",
	},
	.states[1] = {
		.enter = cpu_do_c4,
		.exit_latency = 10,
		.target_residency = 10,
		.power_usage = 5000,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.name = "C4",
		.desc = "CPU retention",
	},
	.state_count = 2
};

/*
 * t210_idle_init
 *
 */
int __init tegra210_idle_init(void)
{
	int ret;
	pr_info("Tegra210 cpuidle driver\n");
	if (!of_machine_is_compatible("nvidia,tegra210")) {
		pr_err("%s: not on T210\n", __func__);
		return -ENODEV;
	}

	do_cc4_init();

	/* For ASIM, allow only C1 */
	if (tegra_cpu_is_asim())
		tegra210_idle_driver.state_count = 1;

	ret = cpuidle_register(&tegra210_idle_driver, 0);
	if (ret) {
		pr_err("%s: Tegra210 cpuidle driver registration failed: %d",
		       __func__, ret);
		return ret;
	}

	return 0;
}
device_initcall(tegra210_idle_init);
