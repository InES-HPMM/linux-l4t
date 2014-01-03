/*
 * tegra210_ahub_utils.c - Tegra AHUB common APIs.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Based on code by Stephen Warren <swarren@nvidia.com>
 *
 * Copyright (C) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/*
#define VERBOSE_DEBUG 1
#define DEBUG 1
*/

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "tegra210_ahub_utils.h"

/* API to set acif parameters */
int tegra210_set_axbar_cif_param(struct device *dev, u32 reg,
				 struct tegra210_axbar_cif_param *acif)
{
	struct regmap *regmap = dev_get_regmap(dev, NULL);
	u32 val = 0, mask = 0;

	dev_vdbg(dev, "%s reg 0x%08x ACIF Params : Channels axbar %d client %d"
		      " bits axbar %d client %d expand %d truncate %d %s %d"
		      " mono_conv %d stereo_conv %d replicate %d threshold %d",
		 __func__, reg, acif->axbar_chan, acif->client_chan,
		 acif->axbar_bits, acif->client_bits, acif->expand,
		 acif->truncate, acif->is_rx == true ? "rx pack mode " :
		 "tx unpack mode ", acif->is_rx == true ? acif->pack_mode :
		 acif->unpack_mode, acif->mono_conv, acif->stereo_conv,
		 acif->replicate, acif->threshold);

	if (!regmap) {
		dev_err(dev, "Failed to get regmap");
		return -EINVAL;
	}

	val = ((acif->axbar_chan - 1) <<
		TEGRA210_AXBAR_CIF_CTRL_AXBAR_CHAN_SHIFT) |
	      ((acif->client_chan - 1) <<
		TEGRA210_AXBAR_CIF_CTRL_CLIENT_CHAN_SHIFT) |
	      (((acif->axbar_bits >> 2) - 1) <<
		TEGRA210_AXBAR_CIF_CTRL_AXBAR_BITS_SHIFT) |
	      (((acif->client_bits >> 2) - 1) <<
		TEGRA210_AXBAR_CIF_CTRL_CLIENT_BITS_SHIFT);

	val |= (acif->expand << TEGRA210_AXBAR_CIF_CTRL_EXPAND_SHIFT) |
	    (acif->truncate << TEGRA210_AXBAR_CIF_CTRL_TRUNCATE_SHIFT) |
	    (acif->mono_conv << TEGRA210_AXBAR_CIF_CTRL_MONO_CONV_SHIFT) |
	    (acif->stereo_conv << TEGRA210_AXBAR_CIF_CTRL_STEREO_CONV_SHIFT) |
	    (acif->replicate << TEGRA210_AXBAR_CIF_CTRL_REPLICATE_SHIFT) |
	    ((acif->threshold - 1) <<
		TEGRA210_AXBAR_CIF_CTRL_FIFO_THRESHOLD_SHIFT);

	mask = (TEGRA210_AXBAR_CIF_CTRL_CLIENT_CHAN_MASK |
		TEGRA210_AXBAR_CIF_CTRL_AXBAR_CHAN_MASK |
		TEGRA210_AXBAR_CIF_CTRL_CLIENT_BITS_MASK |
		TEGRA210_AXBAR_CIF_CTRL_AXBAR_BITS_MASK |
		TEGRA210_AXBAR_CIF_CTRL_EXPAND_MASK |
		TEGRA210_AXBAR_CIF_CTRL_TRUNCATE_MASK |
		TEGRA210_AXBAR_CIF_CTRL_MONO_CONV_MASK |
		TEGRA210_AXBAR_CIF_CTRL_STEREO_CONV_MASK |
		TEGRA210_AXBAR_CIF_CTRL_REPLICATE |
		TEGRA210_AXBAR_CIF_CTRL_FIFO_THRESHOLD_MASK);

	if (acif->is_rx) {
		if (acif->pack_mode == CIF_PACK_MODE_8) {
			val |= TEGRA210_AXBAR_CIF_CTRL_PACK8_ENABLE;
			mask |= TEGRA210_AXBAR_CIF_CTRL_PACK8_ENABLE;
		} else if (acif->pack_mode == CIF_PACK_MODE_16) {
			val |= TEGRA210_AXBAR_CIF_CTRL_PACK16_ENABLE;
			mask |= TEGRA210_AXBAR_CIF_CTRL_PACK16_ENABLE;
		}
	} else {
		if (acif->unpack_mode == CIF_UNPACK_MODE_8) {
			val |= TEGRA210_AXBAR_CIF_CTRL_UNPACK8_ENABLE;
			mask |= TEGRA210_AXBAR_CIF_CTRL_UNPACK8_ENABLE;
		} else if (acif->unpack_mode == CIF_UNPACK_MODE_16) {
			val |= TEGRA210_AXBAR_CIF_CTRL_UNPACK16_ENABLE;
			mask |= TEGRA210_AXBAR_CIF_CTRL_UNPACK16_ENABLE;
		}
	}

	regmap_update_bits(regmap, reg, mask, val);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_set_axbar_cif_param);

/* API to get all acif parameters */
int tegra210_get_axbar_cif_param(struct device *dev, u32 reg,
				 struct tegra210_axbar_cif_param *acif)
{
	struct regmap *regmap = dev_get_regmap(dev, NULL);
	u32 val = 0;

	if (!regmap) {
		dev_err(dev, "Failed to get regmap");
		return -EINVAL;
	}

	regmap_read(regmap, reg, &val);

	acif->axbar_chan = ((val & TEGRA210_AXBAR_CIF_CTRL_AXBAR_CHAN_MASK) >>
			     TEGRA210_AXBAR_CIF_CTRL_CLIENT_CHAN_SHIFT) + 1;
	acif->client_chan = ((val & TEGRA210_AXBAR_CIF_CTRL_CLIENT_CHAN_MASK) >>
			      TEGRA210_AXBAR_CIF_CTRL_AXBAR_CHAN_SHIFT) + 1;
	acif->axbar_bits = (((val & TEGRA210_AXBAR_CIF_CTRL_AXBAR_BITS_MASK) >>
			      TEGRA210_AXBAR_CIF_CTRL_AXBAR_BITS_SHIFT) << 2);
	acif->client_bits =
			(((val & TEGRA210_AXBAR_CIF_CTRL_CLIENT_BITS_MASK) >>
			 TEGRA210_AXBAR_CIF_CTRL_CLIENT_BITS_SHIFT) << 2);

	acif->expand = (val & TEGRA210_AXBAR_CIF_CTRL_EXPAND_MASK) >>
			TEGRA210_AXBAR_CIF_CTRL_EXPAND_SHIFT;
	acif->truncate = (val & TEGRA210_AXBAR_CIF_CTRL_TRUNCATE_MASK) >>
			  TEGRA210_AXBAR_CIF_CTRL_TRUNCATE_SHIFT;
	acif->mono_conv = (val & TEGRA210_AXBAR_CIF_CTRL_MONO_CONV_MASK) >>
			  TEGRA210_AXBAR_CIF_CTRL_MONO_CONV_SHIFT;
	acif->stereo_conv = (val & TEGRA210_AXBAR_CIF_CTRL_STEREO_CONV_MASK) >>
			     TEGRA210_AXBAR_CIF_CTRL_STEREO_CONV_SHIFT;
	acif->replicate = (val & TEGRA210_AXBAR_CIF_CTRL_REPLICATE) >>
			   TEGRA210_AXBAR_CIF_CTRL_REPLICATE_SHIFT;
	acif->threshold =
		((val & TEGRA210_AXBAR_CIF_CTRL_FIFO_THRESHOLD_MASK) >>
		 TEGRA210_AXBAR_CIF_CTRL_FIFO_THRESHOLD_SHIFT) + 1;


	if (acif->is_rx) {
		acif->pack_mode = CIF_PACK_MODE_NONE;
		if (val & TEGRA210_AXBAR_CIF_CTRL_PACK8_ENABLE)
			acif->pack_mode = CIF_PACK_MODE_8;
		else if (val & TEGRA210_AXBAR_CIF_CTRL_PACK16_ENABLE)
			acif->pack_mode = CIF_PACK_MODE_16;
	} else {
		acif->unpack_mode = CIF_UNPACK_MODE_NONE;
		if (val & TEGRA210_AXBAR_CIF_CTRL_UNPACK8_ENABLE)
			acif->unpack_mode = CIF_UNPACK_MODE_8;
		else if (val & TEGRA210_AXBAR_CIF_CTRL_UNPACK16_ENABLE)
			acif->unpack_mode = CIF_UNPACK_MODE_16;
	}

	dev_vdbg(dev, "%s reg 0x%08x ACIF Params : Channels axbar %d client %d"
		      " bits axbar %d client %d expand %d truncate %d %s %d"
		      " mono_conv %d stereo_conv %d replicate %d threshold %d",
		 __func__, reg, acif->axbar_chan, acif->client_chan,
		 acif->axbar_bits, acif->client_bits, acif->expand,
		 acif->truncate, acif->is_rx == true ? "rx pack mode " :
		 "tx unpack mode ", acif->is_rx == true ? acif->pack_mode :
		 acif->unpack_mode, acif->mono_conv, acif->stereo_conv,
		 acif->replicate, acif->threshold);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_get_axbar_cif_param);

int tegra210_ahubram_write(struct device *dev,
			   u32 reg_ctrl, u32 reg_data,
			   u32 start_addr, u32 *data, int size)
{
	struct regmap *regmap = dev_get_regmap(dev, NULL);
	u32 val = 0;
	int i = 0;

	dev_vdbg(dev, "%s start addr %x data size %d",
		 __func__, start_addr, size);

	if (!regmap) {
		dev_err(dev, "Failed to get regmap");
		return -EINVAL;
	}

	val = (start_addr << TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_SHIFT) &
	      TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_MASK;
	val |= TEGRA210_AHUBRAMCTL_CTRL_ADDR_INIT_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_SEQ_ACCESS_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_RW_WRITE;

	regmap_write(regmap, reg_ctrl, val);

	for (i = 0; i < size; i++) {
		regmap_write(regmap, reg_data, data[i]);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ahubram_write);

int tegra210_ahubram_read(struct device *dev,
			  u32 reg_ctrl, u32 reg_data,
			  u32 start_addr, u32 *data, int size)
{
	struct regmap *regmap = dev_get_regmap(dev, NULL);
	u32 val = 0;
	int i = 0;

	dev_vdbg(dev, "%s start addr %x data size %d",
		 __func__, start_addr, size);

	if (!regmap) {
		dev_err(dev, "Failed to get regmap");
		return -EINVAL;
	}

	val = (start_addr << TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_SHIFT) &
	      TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_MASK;
	val |= TEGRA210_AHUBRAMCTL_CTRL_ADDR_INIT_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_SEQ_ACCESS_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_RW_READ;

	regmap_write(regmap, reg_ctrl, val);
	for (i = 0; i < size; i++)
		regmap_read(regmap, reg_data, &data[i]);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_ahubram_read);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 AHUB common API code");
MODULE_LICENSE("GPL");
