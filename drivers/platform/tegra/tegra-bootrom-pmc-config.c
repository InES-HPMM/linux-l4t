/*
 * tegra-bootrom-pmc-config: Commands for bootrom to configure PMIC trhough PMC
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/tegra-pmc.h>

struct tegra_bootrom_block {
	const char *name;
	bool reg_8bits;
	bool data_8bits;
	bool i2c_controller;
	int controller_id;
	bool enable_reset;
	int ncommands;
	u32 *commands;
};

struct tegra_bootrom_commands {
	u32 command_retry_count;
	u32 delay_between_commands;
	u32 wait_before_bus_clear;
	struct tegra_bootrom_block *blocks;
	int nblocks;
};

static int tegra_bootrom_get_commands_from_dt(struct device *dev,
		struct tegra_bootrom_commands **br_commands)
{
	struct device_node *np = dev->of_node;
	struct device_node *br_np, *child;
	struct tegra_bootrom_commands *bcommands;
	int *command_ptr;
	struct tegra_bootrom_block *block;
	int nblocks;
	u32 reg, data;
	u32 *wr_commands;
	int count, nblock, ncommands, i, reg_shift;
	int ret;

	if (!np) {
		dev_err(dev, "No device node pointer\n");
		return -EINVAL;
	}

	br_np = of_find_node_by_name(np, "bootrom-commands");
	if (!br_np) {
		dev_err(dev, "There is no bootrom commmands\n");
		return -EINVAL;
	}

	nblocks = of_get_child_count(br_np);
	if (!nblocks) {
		dev_err(dev, "There is no command block for bootrom\n");
		return -EINVAL;
	}

	count = 0;
	for_each_child_of_node(br_np, child) {
		ret = of_property_count_u32(child, "nvidia,write-commands");
		if (ret < 0) {
			dev_err(dev, "Node %s does not have write-commnds\n",
					child->full_name);
			return -EINVAL;
		}
		count += ret / 2;
	}

	bcommands = devm_kzalloc(dev,  sizeof(*bcommands) +
			nblocks * sizeof(*block) + count * sizeof(u32),
			GFP_KERNEL);
	if (!bcommands)
		return -ENOMEM;

	bcommands->nblocks = nblocks;
	bcommands->blocks = (void *)(bcommands + 1);
	command_ptr = (void *) (bcommands->blocks + nblocks);

	of_property_read_u32(br_np, "nvidia,command-retries-count",
			&bcommands->command_retry_count);
	of_property_read_u32(br_np, "nvidia,delay-between-commands-us",
			&bcommands->delay_between_commands);
	of_property_read_u32(br_np, "nvidia,wait-before-start-bus-clear-us",
			&bcommands->wait_before_bus_clear);

	nblock = 0;
	for_each_child_of_node(br_np, child) {
		block = &bcommands->blocks[nblock];

		of_property_read_string(child, "nvidia,command-names",
				&block->name);
		block->reg_8bits = !of_property_read_bool(child,
					"nvidia,enable-16bit-register");
		block->data_8bits = !of_property_read_bool(child,
					"nvidia,enable-16bit-data");
		block->i2c_controller = of_property_read_bool(child,
					"nvidia,controller-type-i2c");
		block->enable_reset = of_property_read_bool(child,
					"nvidia,enable-controller-reset");
		count = of_property_count_u32(child, "nvidia,write-commands");
		ncommands = count / 2;

		block->commands = command_ptr;
		command_ptr += ncommands;
		wr_commands = block->commands;
		reg_shift = (block->data_8bits) ? 8 : 16;
		for (i = 0; i < ncommands; ++i) {
			of_property_read_u32_index(child,
				"nvidia,write-commands", i * 2, &reg);
			of_property_read_u32_index(child,
				"nvidia,write-commands", i * 2 + 1, &data);

			wr_commands[i] = (reg << reg_shift) | data;
		}
		block->ncommands = ncommands;
		nblock++;
	}
	return 0;
}

int tegra210_boorom_pmc_init(struct device *dev)
{
	struct tegra_bootrom_commands *br_commands;
	int ret;

	ret = tegra_bootrom_get_commands_from_dt(dev, &br_commands);
	if (ret < 0)
		pr_info("T210 pmc config for bootrom command failed\n");
	else
		pr_info("T210 pmc config for bootrom command passed\n");
	return 0;
}
EXPORT_SYMBOL(tegra210_boorom_pmc_init);
