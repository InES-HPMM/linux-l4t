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

enum tegra210_pmc_pads_id {
	TEGRA210_PAD_SYS,
	TEGRA210_PAD_UART,
	TEGRA210_PAD_BB,
	TEGRA210_PAD_AUDIO,
	TEGRA210_PAD_CAM,
	TEGRA210_PAD_PEX_CTRL,
	TEGRA210_PAD_SDMMC1,
	TEGRA210_PAD_SDMMC3,
	TEGRA210_PAD_AUDIO_HV,
	TEGRA210_PAD_DBG,
	TEGRA210_PAD_DMIC,
	TEGRA210_PAD_GPIO,
	TEGRA210_PAD_spi,
	TEGRA210_PAD_SPI_HV,
};

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
	TEGRA_210_PADS("spi", TEGRA210_PAD_spi, 22),
	TEGRA_210_PADS("spi-hv", TEGRA210_PAD_SPI_HV, 23),
};

static int tegra210_pmc_padctrl_set_voltage(struct padctrl_dev *pad_dev,
		int pad_id, u32 voltage)
{
	u32 offset;
	int val;
	int i;

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

struct padctrl_desc tegra210_pmc_padctrl_desc = {
	.name = "tegra-pmc-padctrl",
	.ops = &tegra210_pmc_padctrl_ops,
};

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
	pr_info("T210 pmc padctrl driver initialized\n");
	return 0;
}
