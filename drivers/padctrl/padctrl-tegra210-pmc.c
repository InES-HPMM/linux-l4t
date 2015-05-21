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


/* IO PAD group */
#define TEGRA_IO_PAD_GROUP_AUDIO	0
#define TEGRA_IO_PAD_GROUP_AUDIO_HV	1
#define TEGRA_IO_PAD_GROUP_CAM		2
#define TEGRA_IO_PAD_GROUP_CSIA		3
#define TEGRA_IO_PAD_GROUP_CSIB		4
#define TEGRA_IO_PAD_GROUP_CSIC		5
#define TEGRA_IO_PAD_GROUP_CSID		6
#define TEGRA_IO_PAD_GROUP_CSIE		7
#define TEGRA_IO_PAD_GROUP_CSIF		8
#define TEGRA_IO_PAD_GROUP_DBG		9
#define TEGRA_IO_PAD_GROUP_DEBUG_NONAO	10
#define TEGRA_IO_PAD_GROUP_DMIC		11
#define TEGRA_IO_PAD_GROUP_DP		12
#define TEGRA_IO_PAD_GROUP_DSI		13
#define TEGRA_IO_PAD_GROUP_DSIB		14
#define TEGRA_IO_PAD_GROUP_DSIC		15
#define TEGRA_IO_PAD_GROUP_DSID		16
#define TEGRA_IO_PAD_GROUP_EMMC		17
#define TEGRA_IO_PAD_GROUP_EMMC2	18
#define TEGRA_IO_PAD_GROUP_GPIO		19
#define TEGRA_IO_PAD_GROUP_HDMI		20
#define TEGRA_IO_PAD_GROUP_HSIC		21
#define TEGRA_IO_PAD_GROUP_LVDS		22
#define TEGRA_IO_PAD_GROUP_MIPI_BIAS	23
#define TEGRA_IO_PAD_GROUP_PEX_BIAS	24
#define TEGRA_IO_PAD_GROUP_PEX_CLK1	25
#define TEGRA_IO_PAD_GROUP_PEX_CLK2	26
#define TEGRA_IO_PAD_GROUP_PEX_CTRL	27
#define TEGRA_IO_PAD_GROUP_SDMMC1	28
#define TEGRA_IO_PAD_GROUP_SDMMC3	29
#define TEGRA_IO_PAD_GROUP_SPI		30
#define TEGRA_IO_PAD_GROUP_SPI_HV	31
#define TEGRA_IO_PAD_GROUP_UART		32
#define TEGRA_IO_PAD_GROUP_USB_BIAS	33
#define TEGRA_IO_PAD_GROUP_USB0		34
#define TEGRA_IO_PAD_GROUP_USB1		35
#define TEGRA_IO_PAD_GROUP_USB2		36
#define TEGRA_IO_PAD_GROUP_USB3		37

struct tegra210_pmc_pads {
	const char *pad_name;
	int pad_id;
	int bit_position;
	int reg_offset;
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

#define TEGRA_210_PAD_GROUPS(_name, _id, _off, _bit)	\
{							\
	.pad_name = _name,				\
	.pad_id = TEGRA_IO_PAD_GROUP_##_id,		\
	.reg_offset = _off,				\
	.bit_position = _bit,				\
}

static struct tegra210_pmc_pads tegra210_pad_groups[] = {
	TEGRA_210_PAD_GROUPS("audio", AUDIO, 0, 17),
	TEGRA_210_PAD_GROUPS("audio-hv", AUDIO_HV, 1, 29),
	TEGRA_210_PAD_GROUPS("cam", CAM, 1, 4),
	TEGRA_210_PAD_GROUPS("csia", CSIA, 0, 0),
	TEGRA_210_PAD_GROUPS("csib", CSIB, 0, 1),
	TEGRA_210_PAD_GROUPS("csic", CSIC, 1, 10),
	TEGRA_210_PAD_GROUPS("csid", CSID, 1, 11),
	TEGRA_210_PAD_GROUPS("csie", CSIE, 1, 12),
	TEGRA_210_PAD_GROUPS("csif", CSIF, 1, 13),
	TEGRA_210_PAD_GROUPS("dbg", DBG, 0, 25),
	TEGRA_210_PAD_GROUPS("debug-nonao", DEBUG_NONAO, 0, 26),
	TEGRA_210_PAD_GROUPS("dmic", DMIC, 1, 18),
	TEGRA_210_PAD_GROUPS("dp", DP, 1, 19),
	TEGRA_210_PAD_GROUPS("dsi", DSI, 0, 2),
	TEGRA_210_PAD_GROUPS("dsib", DSIB, 1, 7),
	TEGRA_210_PAD_GROUPS("dsic", DSIC, 1, 8),
	TEGRA_210_PAD_GROUPS("dsid", DSID, 1, 9),
	TEGRA_210_PAD_GROUPS("emmc", EMMC, 1, 3),
	TEGRA_210_PAD_GROUPS("emmc2", EMMC2, 1, 5),
	TEGRA_210_PAD_GROUPS("gpio", GPIO, 0, 27),
	TEGRA_210_PAD_GROUPS("hdmi", HDMI, 0, 28),
	TEGRA_210_PAD_GROUPS("hsic", HSIC, 0, 19),
	TEGRA_210_PAD_GROUPS("lvds", LVDS, 1, 25),
	TEGRA_210_PAD_GROUPS("mipi-bias", MIPI_BIAS, 0, 3),
	TEGRA_210_PAD_GROUPS("pex-bias", PEX_BIAS, 0, 4),
	TEGRA_210_PAD_GROUPS("pex-clk1", PEX_CLK1, 0, 5),
	TEGRA_210_PAD_GROUPS("pex-clk2", PEX_CLK2, 0, 6),
	TEGRA_210_PAD_GROUPS("pex-ctrl", PEX_CTRL, 1, 0),
	TEGRA_210_PAD_GROUPS("sdmmc1", SDMMC1, 1, 1),
	TEGRA_210_PAD_GROUPS("sdmmc3", SDMMC3, 1, 2),
	TEGRA_210_PAD_GROUPS("spi", SPI, 1, 14),
	TEGRA_210_PAD_GROUPS("spi-hv", SPI_HV, 1, 15),
	TEGRA_210_PAD_GROUPS("uart", UART, 0, 14),
	TEGRA_210_PAD_GROUPS("usb-bias", USB_BIAS, 0, 12),
	TEGRA_210_PAD_GROUPS("usb0", USB0, 0, 9),
	TEGRA_210_PAD_GROUPS("usb1", USB1, 0, 10),
	TEGRA_210_PAD_GROUPS("usb2", USB2, 0, 11),
	TEGRA_210_PAD_GROUPS("usb3", USB3, 0, 18),
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

static int tegra210_pmc_padctrl_set_power(struct padctrl_dev *pad_dev,
		int pad_id, u32 enable)
{
	u32 bit, reg;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(tegra210_pad_groups); ++i) {
		if (tegra210_pad_groups[i].pad_id == pad_id)
			break;
	}

	if (i == ARRAY_SIZE(tegra210_pad_groups))
		return -EINVAL;

	bit = tegra210_pad_groups[i].bit_position;
	reg = tegra210_pad_groups[i].reg_offset;
	if (enable)
		ret = tegra_pmc_io_dpd_disable(reg, bit);
	else
		ret = tegra_pmc_io_dpd_enable(reg, bit);
	return ret;
}

static int tegra210_pmc_padctrl_power_enable(struct padctrl_dev *pad_dev,
		int pad_id)
{
	return tegra210_pmc_padctrl_set_power(pad_dev, pad_id, 1);
}

static int tegra210_pmc_padctrl_power_disable(struct padctrl_dev *pad_dev,
		int pad_id)
{
	return tegra210_pmc_padctrl_set_power(pad_dev, pad_id, 0);
}

static struct padctrl_ops tegra210_pmc_padctrl_ops = {
	.set_voltage = &tegra210_pmc_padctrl_set_voltage,
	.get_voltage = &tegra210_pmc_padctl_get_voltage,
	.power_enable = &tegra210_pmc_padctrl_power_enable,
	.power_disable = &tegra210_pmc_padctrl_power_disable,
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
	kfree(configs);
	return 0;
}

static int tegra210_pmc_parse_io_pad_power(struct device_node *np,
		struct padctrl_dev *pad_dev)
{
	u32 io_pad_id, enable;
	int n_config;
	u32 *configs;
	int i, j, index;
	int ret;

	n_config = of_property_count_u32(np, "platform-io-pad-power");
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
		of_property_read_u32_index(np, "platform-io-pad-power",
					index, &io_pad_id);
		of_property_read_u32_index(np, "platform-io-pad-power",
					index + 1, &enable);
		for (j = 0; j < ARRAY_SIZE(tegra210_pad_groups); ++j) {
			if (tegra210_pad_groups[j].pad_id == io_pad_id)
				break;
		}
		if (j == ARRAY_SIZE(tegra210_pad_groups)) {
			pr_err("PMC: IO pad ID %u is invalid\n", io_pad_id);
			continue;
		}

		configs[index] = j;
		configs[index + 1] = enable;
	};

	for (i = 0; i < n_config; ++i) {
		index = i * 2;
		ret = tegra210_pmc_padctrl_set_power(pad_dev,
				tegra210_pad_groups[configs[index]].pad_id,
				configs[index + 1]);
		if (ret < 0) {
			pr_warn("PMC: IO pad %s power config failed: %d\n",
			     tegra210_pad_groups[configs[index]].pad_name, ret);
			WARN_ON(1);
		} else {
			pr_info("PMC: IO pad %s power is %s\n",
				tegra210_pad_groups[configs[index]].pad_name,
				   (configs[index + 1]) ? "enable" : "disable");
		}
	}
	kfree(configs);
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
	tegra210_pmc_parse_io_pad_power(config.of_node,
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

static int dbg_io_pad_dpd(struct seq_file *s, void *unused)
{
	int i;
	int enable;
	char *estr;
	int bit, reg;

	for (i = 0; i < ARRAY_SIZE(tegra210_pad_groups); ++i) {
		bit = tegra210_pad_groups[i].bit_position;
		reg = tegra210_pad_groups[i].reg_offset;
		enable = tegra_pmc_io_dpd_get_status(reg, bit);
		estr = (enable) ? "enable" : "disable";
		seq_printf(s, "PMC: IO pad DPD %s - %s\n",
			tegra210_pad_groups[i].pad_name, estr);
	}

	return 0;
}

#define DEBUG_IO_PAD(_f) 						\
static int dbg_io_pad_open_##_f(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, _f, &inode->i_private);		\
}									\
static const struct file_operations debug_fops_##_f = {			\
	.open	   = dbg_io_pad_open_##_f,				\
	.read	   = seq_read,						\
	.llseek	 = seq_lseek,						\
	.release	= single_release,				\
}									\

DEBUG_IO_PAD(dbg_io_pad_show);
DEBUG_IO_PAD(dbg_io_pad_dpd);

static int __init tegra_io_pad_debuginit(void)
{
	(void)debugfs_create_file("tegra_io_pad", S_IRUGO,
				NULL, NULL, &debug_fops_dbg_io_pad_show);
	(void)debugfs_create_file("tegra_io_dpd", S_IRUGO,
				NULL, NULL, &debug_fops_dbg_io_pad_dpd);
	return 0;
}
late_initcall(tegra_io_pad_debuginit);
#endif
