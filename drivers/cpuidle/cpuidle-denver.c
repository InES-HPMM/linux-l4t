/*
 * drivers/cpuidle/cpuidle-denver.c
 *
 * Copyright (C) 2013 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>

void tegra_pd_in_idle(bool enable) {}

static u32 pmstate_map[CPUIDLE_STATE_MAX] = { -1 };

static int denver_enter_c_state(
		struct cpuidle_device *dev,
		struct cpuidle_driver *drv,
		int index)
{
	asm volatile("msr actlr_el1, %0\n" : : "r" (pmstate_map[index]));
	asm volatile("wfi\n");

	local_irq_enable();

	return index;
}

static struct cpuidle_driver denver_idle_driver = {
	.name = "denver_idle",
	.owner = THIS_MODULE,
};

static int __init denver_power_states_init(void)
{
	struct device_node *of_states;
	struct device_node *child;
	struct cpuidle_state *state;
	const char *name;
	struct dentry *cpuidle_denver_dir;
	struct dentry *idle_node;
	u32 state_count = 0;
	u32 prop;

	cpuidle_denver_dir = debugfs_create_dir("cpuidle_denver", NULL);
	if (!cpuidle_denver_dir) {
		pr_err("%s: Couldn't create the \"cpuidle_denver\" debugfs "
			"node.\n",
			__func__);
		return -1;
	}

	of_states = of_find_node_by_name(NULL, "denver_power_states");
	if (!of_states)
		return -ENODEV;

	for_each_child_of_node(of_states, child) {
		state = &denver_idle_driver.states[state_count];
		if (of_property_read_string(child, "state-name", &name))
			continue;
		snprintf(state->name, CPUIDLE_NAME_LEN, child->name);
		snprintf(state->desc, CPUIDLE_DESC_LEN, name);
		if (of_property_read_u32(child, "latency", &prop) == 0)
			state->exit_latency = prop;
		if (of_property_read_u32(
				child, "residency", &prop) == 0) {
			state->flags = CPUIDLE_FLAG_TIME_VALID;
			state->target_residency = prop;
		}
		if (of_property_read_u32(child, "power", &prop) != 0)
			state->exit_latency = prop;

		state->enter = denver_enter_c_state;

		/* Map index to the actual LP state */
		if (of_property_read_u32(child, "pmstate", &prop) != 0)
			continue;
		pmstate_map[state_count] = prop;

		/* Create a debugfs node for the idle state */
		idle_node = debugfs_create_dir(child->name,
					       cpuidle_denver_dir);
		if (!idle_node) {
				return -1;
		}
		if (!debugfs_create_x32("pmstate", S_IRUGO | S_IWUSR, idle_node,
				       &pmstate_map[state_count])) {
			pr_err("%s: Couldn't create the pmstate debugfs node"
			       "for %s.\n",__func__, child->name);
			return -1;
		}

		state_count++;
	}

	denver_idle_driver.state_count = state_count;

	return cpuidle_register_driver(&denver_idle_driver);
}

static int __init denver_cpuidle_devices_init(void)
{
	struct device_node *cpu = NULL;
	struct device_node *of_states;
	struct cpuidle_device *dev;
	u64 cpu_reg;

	for_each_node_by_type(cpu, "cpu") {
		if (!of_device_is_compatible(cpu, "nvidia,denver"))
			continue;

		of_states = of_parse_phandle(cpu, "power-states", 0);
		if (!of_states || !of_device_is_compatible(
				of_states, "nvidia,denver"))
			continue;

		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev)
			return -ENOMEM;

		BUG_ON(of_property_read_u64(cpu, "reg", &cpu_reg));
		dev->cpu = (unsigned long)cpu_reg;
		dev->state_count = denver_idle_driver.state_count;

		if (cpuidle_register_device(dev)) {
			pr_err("%s: failed to register idle device\n",
				cpu->full_name);
			kfree(dev);
			return -EIO;
		}
	}

	return 0;
}

static int __init denver_cpuidle_init(void)
{
	int e;

	e = denver_power_states_init();
	if (e) {
		pr_err("%s: failed to init cpuidle power states.\n", __func__);
		return e;
	}

	e = denver_cpuidle_devices_init();
	if (e) {
		pr_err("%s: failed to init cpuidle devices.\n", __func__);
		return e;
	}

	return 0;
}

device_initcall(denver_cpuidle_init);
