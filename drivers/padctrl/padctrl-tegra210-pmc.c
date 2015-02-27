/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/padctrl/padctrl.h>
#include <linux/tegra-pmc.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define TEGRA210_PAD_SYS	0
#define TEGRA210_PAD_UART	1
#define TEGRA210_PAD_BB		2
#define TEGRA210_PAD_AUDIO	3
#define TEGRA210_PAD_CAM	4
#define TEGRA210_PAD_PEX_CTRL	5
#define TEGRA210_PAD_SDMMC1	6
#define TEGRA210_PAD_SDMMC3	7
#define TEGRA210_PAD_HV		8
#define TEGRA210_PAD_AUDIO_HV	9
#define TEGRA210_PAD_DBG	10
#define TEGRA210_PAD_DMIC	11
#define TEGRA210_PAD_GPIO	12
#define TEGRA210_PAD_SPI	13
#define TEGRA210_PAD_SPI_HV	14

struct tegra210_pmc_pads {
	const char *pad_name;
	int pad_id;
	int bit_position;
};

struct tegra210_pmc_padcontrl {
	struct padctrl_dev *pad_dev;
};

#define TEGRA_210_PADS(_name, _id, _bit)		\
{							\
	.pad_name = _name,				\
	.pad_id = _id,					\
	.bit_position = _bit,				\
}

static struct tegra210_pmc_pads tegra210_pads[] = {
	TEGRA_210_PADS("sys", TEGRA210_PAD_SYS, 0),
	TEGRA_210_PADS("uart", TEGRA210_PAD_UART, 2),
	TEGRA_210_PADS("bb", TEGRA210_PAD_BB, 3),
	TEGRA_210_PADS("audio", TEGRA210_PAD_AUDIO, 5),
	TEGRA_210_PADS("cam", TEGRA210_PAD_CAM, 10),
	TEGRA_210_PADS("pex_ctrl", TEGRA210_PAD_PEX_CTRL, 11),
	TEGRA_210_PADS("sdmmc1", TEGRA210_PAD_SDMMC1, 12),
	TEGRA_210_PADS("sdmmc3", TEGRA210_PAD_SDMMC3, 13),
	TEGRA_210_PADS("audio-hv", TEGRA210_PAD_AUDIO_HV, 18),
	TEGRA_210_PADS("dbg", TEGRA210_PAD_DBG, 19),
	TEGRA_210_PADS("dmic", TEGRA210_PAD_DMIC, 20),
	TEGRA_210_PADS("gpio", TEGRA210_PAD_GPIO, 21),
	TEGRA_210_PADS("spi", TEGRA210_PAD_SPI, 22),
	TEGRA_210_PADS("spi-hv", TEGRA210_PAD_SPI_HV, 23),
};

static int tegra210_pmc_padctrl_set_voltage(struct padctrl_dev *pad_dev,
		int pad_id, u32 voltage)
{
	u32 offset;
	int val;
	int i;

	if ((voltage != 1800000) && (voltage != 3300000))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tegra210_pads); ++i) {
		if (tegra210_pads[i].pad_id == pad_id)
			break;
	}

	if (i == ARRAY_SIZE(tegra210_pads))
		return -EINVAL;

	offset = BIT(tegra210_pads[i].bit_position);
	val = (voltage == 3300000) ? offset : 0;
	tegra_pmc_pwr_detect_update(offset, val);
	udelay(100);
	return 0;
}

static int tegra210_pmc_padctl_get_voltage(struct padctrl_dev *pad_dev,
		int pad_id, u32 *voltage)
{
	unsigned int pwrdet_mask;
	u32 offset;
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra210_pads); ++i) {
		if (tegra210_pads[i].pad_id == pad_id)
			break;
	}

	if (i == ARRAY_SIZE(tegra210_pads))
		return -EINVAL;

	offset = BIT(tegra210_pads[i].bit_position);
	pwrdet_mask = tegra_pmc_pwr_detect_get(offset);
	if (pwrdet_mask & offset)
		*voltage = 3300000UL;
	else
		*voltage = 1800000UL;

	return 0;
}

static struct padctrl_ops tegra210_pmc_padctrl_ops = {
	.set_voltage = &tegra210_pmc_padctrl_set_voltage,
	.get_voltage = &tegra210_pmc_padctl_get_voltage,
};

static struct padctrl_desc tegra210_pmc_padctrl_desc = {
	.name = "tegra-pmc-padctrl",
	.ops = &tegra210_pmc_padctrl_ops,
};

static int tegra210_pmc_parse_io_pad_voltage(struct device_node *np,
		struct padctrl_dev *pad_dev)
{
	u32 io_pad_id, volt_uv;
	int n_config;
	u32 *configs;
	int i, j, index;
	int ret;

	n_config = of_property_count_u32(np, "platform-io-pad-voltage");
	if (n_config < 0)
		return 0;
	if (!n_config || (n_config % 2))
		return -EINVAL;

	n_config = n_config/2;

	configs = kzalloc(n_config * sizeof(*configs), GFP_KERNEL);
	if (!configs)
		return -ENOMEM;

	for (i = 0; i < n_config; ++i) {
		index = i * 2;
		of_property_read_u32_index(np, "platform-io-pad-voltage",
					index, &io_pad_id);
		of_property_read_u32_index(np, "platform-io-pad-voltage",
					index + 1, &volt_uv);
		for (j = 0; j < ARRAY_SIZE(tegra210_pads); ++j) {
			if (tegra210_pads[j].pad_id == io_pad_id)
				break;
		}
		if (j == ARRAY_SIZE(tegra210_pads)) {
			pr_err("PMC: IO pad ID %u is invalid\n", io_pad_id);
			continue;
		}

		configs[index] = j;
		configs[index + 1] = volt_uv;
	};

	for (i = 0; i < n_config; ++i) {
		index = i * 2;
		if (!configs[index + 1])
			continue;
		ret = tegra210_pmc_padctrl_set_voltage(pad_dev,
				tegra210_pads[configs[index]].pad_id,
				configs[index + 1]);
		if (ret < 0) {
			pr_warn("PMC: IO pad %s voltage config failed: %d\n",
				tegra210_pads[configs[index]].pad_name, ret);
			WARN_ON(1);
		} else {
			pr_info("PMC: IO pad %s voltage is %d\n",
				tegra210_pads[configs[index]].pad_name,
					configs[index + 1]);
		}
	}
	return 0;
}

int tegra210_pmc_padctrl_init(struct device *dev, struct device_node *np)
{
	struct tegra210_pmc_padcontrl *pmc_padctrl;
	struct padctrl_config config = { };
	int ret;

	pmc_padctrl = kzalloc(sizeof(*pmc_padctrl), GFP_KERNEL);
	if (!pmc_padctrl) {
		pr_err("Mem allocation for pmc_padctrl failed\n");
		return -ENOMEM;
	}

	config.of_node = (dev && dev->of_node) ? dev->of_node : np;
	pmc_padctrl->pad_dev = padctrl_register(dev, &tegra210_pmc_padctrl_desc,
					&config);
	if (IS_ERR(pmc_padctrl->pad_dev)) {
		ret = PTR_ERR(pmc_padctrl->pad_dev);
		pr_err("T210 padctrl driver init failed: %d\n", ret);
		kfree(pmc_padctrl);
		return ret;
	}
	padctrl_set_drvdata(pmc_padctrl->pad_dev, pmc_padctrl);
	tegra210_pmc_parse_io_pad_voltage(config.of_node,
				pmc_padctrl->pad_dev);

	pr_info("T210 pmc padctrl driver initialized\n");
	return 0;
}

#ifdef  CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_io_pad_show(struct seq_file *s, void *unused)
{
	unsigned int pwrdet_mask;
	u32 offset;
	int i;
	unsigned long voltage;

	for (i = 0; i < ARRAY_SIZE(tegra210_pads); ++i) {
		offset = BIT(tegra210_pads[i].bit_position);
		pwrdet_mask = tegra_pmc_pwr_detect_get(offset);
		if (pwrdet_mask & offset)
			voltage = 3300000UL;
		else
			voltage = 1800000UL;
		seq_printf(s, "PMC: IO pad %s voltage %lu\n",
			tegra210_pads[i].pad_name, voltage);
	}

	return 0;
}
static int dbg_io_pad_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_io_pad_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open	   = dbg_io_pad_open,
	.read	   = seq_read,
	.llseek	 = seq_lseek,
	.release	= single_release,
};

static int __init tegra_io_pad_debuginit(void)
{
	(void)debugfs_create_file("tegra_io_pad", S_IRUGO,
				NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(tegra_io_pad_debuginit);
#endif
