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
#include <linux/cpuidle.h>
#include <linux/of_platform.h>
#include <asm/proc-fns.h>

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
	.state_count = 1,
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
	ret = cpuidle_register(&tegra210_idle_driver, 0);
	if (ret) {
		pr_err("%s: Tegra210 cpuidle driver registration failed: %d",
		       __func__, ret);
		return ret;
	}

	return 0;
}
device_initcall(tegra210_idle_init);
