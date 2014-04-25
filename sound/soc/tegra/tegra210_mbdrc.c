/*
 * tegra210_mbdrc.c - Tegra MBDRC driver.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
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
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_mbdrc.h"
#include "tegra210_axbar.h"

#define DRV_NAME "tegra210-mbdrc"

/* TEGRA210 MBDRC driver context */
struct tegra210_mbdrc_ctx {
	struct device		*dev;
	void __iomem		*regs;
	struct regmap		*regmap;

	int			id;
};

static struct tegra210_mbdrc_ctx *tegra210_mbdrc[TEGRA210_MBDRC_COUNT];

/* API to enable/disable MBDRC clock */
void tegra210_mbdrc_enable_clk(enum tegra210_ahub_cifs cif)
{
	pm_runtime_get_sync(tegra210_mbdrc[OPE_ID(cif)]->dev);
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_enable_clk);

void tegra210_mbdrc_disable_clk(enum tegra210_ahub_cifs cif)
{
	pm_runtime_put_sync(tegra210_mbdrc[OPE_ID(cif)]->dev);
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_disable_clk);

int tegra210_mbdrc_is_slcg_enable(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];
	u32 val = 0;

	regmap_read(mbdrc->regmap, TEGRA210_MBDRC_STATUS, &val);
	val &= TEGRA210_MBDRC_STATUS_SLCG_CLKEN;

	dev_vdbg(mbdrc->dev, "%s MBDRC SLCG enable %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_is_slcg_enable);

int tegra210_mbdrc_is_config_err(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];
	u32 val = 0;

	regmap_read(mbdrc->regmap, TEGRA210_MBDRC_STATUS, &val);
	val &= TEGRA210_MBDRC_STATUS_CONFIG_ERR;

	dev_vdbg(mbdrc->dev, "%s MBDRC Config error %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_is_config_err);

/* Master soft reset for MBDRC */
int tegra210_mbdrc_reset(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];
	int cnt = AHUB_OP_MAX_RETRY;
	u32 val = 0;

	dev_vdbg(mbdrc->dev, "%s", __func__);

	regmap_update_bits(mbdrc->regmap,
			   TEGRA210_MBDRC_SOFT_RST,
			   TEGRA210_MBDRC_SOFT_RST_RESET,
			   TEGRA210_MBDRC_SOFT_RST_RESET);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(mbdrc->regmap, TEGRA210_MBDRC_SOFT_RST, &val);
	} while ((val & TEGRA210_MBDRC_SOFT_RST_RESET) && cnt--);

	if (!cnt) {
		dev_warn(mbdrc->dev, "%s Failed to reset MBDRC", __func__);
		return -ETIME;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_reset);

/* Enable second level clock gating for MBDRC */
int tegra210_mbdrc_set_slcg(enum tegra210_ahub_cifs cif, int en)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];

	dev_vdbg(mbdrc->dev, "%s MBDRC slcg enable %d", __func__, en);

	regmap_update_bits(mbdrc->regmap,
			   TEGRA210_MBDRC_CG,
			   TEGRA210_MBDRC_CG_SLCG_EN,
			   en ? TEGRA210_MBDRC_CG_SLCG_EN : 0);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_set_slcg);

int tegra210_mbdrc_set_bypass(enum tegra210_ahub_cifs cif)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];

	dev_vdbg(mbdrc->dev, "%s MBDRC set bypass", __func__);

	regmap_update_bits(mbdrc->regmap,
		TEGRA210_MBDRC_CONFIG,
		TEGRA210_MBDRC_CONFIG_MBDRC_MODE_MASK,
		TEGRA210_MBDRC_CONFIG_MBDRC_MODE_BYPASS);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_set_bypass);

static int tegra210_mbdrc_set_band_params(enum tegra210_ahub_cifs cif,
			      struct tegra210_mbdrc_band_params *params,
			      unsigned int config_mode)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];
	unsigned int reg_offset;

	dev_vdbg(mbdrc->dev, "%s MBDRC set band %d", __func__, params->band);

	if (params->band >= MBDRC_MAX_BAND) {
		dev_err(mbdrc->dev, "Invalid MBDRC band %d", params->band);
		return -EINVAL;
	}
	reg_offset = (params->band * TEGRA210_MBDRC_FILTER_PARAM_STRIDE);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_IN_ATTACK,
		TEGRA210_MBDRC_IN_ATTACK_TC_MASK,
		params->in_attack_tc << TEGRA210_MBDRC_IN_ATTACK_TC_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_IN_RELEASE,
		TEGRA210_MBDRC_IN_RELEASE_TC_MASK,
		params->in_release_tc << TEGRA210_MBDRC_IN_RELEASE_TC_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_FAST_ATTACK,
		TEGRA210_MBDRC_FAST_ATTACK_TC_MASK,
		params->fast_attack_tc << TEGRA210_MBDRC_FAST_ATTACK_TC_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_IN_THRESH,
		TEGRA210_MBDRC_IN_THRESH_1ST_MASK,
		params->in_thresh[0] << TEGRA210_MBDRC_IN_THRESH_1ST_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_IN_THRESH,
		TEGRA210_MBDRC_IN_THRESH_2ND_MASK,
		params->in_thresh[1] << TEGRA210_MBDRC_IN_THRESH_2ND_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_IN_THRESH,
		TEGRA210_MBDRC_IN_THRESH_3RD_MASK,
		params->in_thresh[2] << TEGRA210_MBDRC_IN_THRESH_3RD_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_IN_THRESH,
		TEGRA210_MBDRC_IN_THRESH_4TH_MASK,
		params->in_thresh[3] << TEGRA210_MBDRC_IN_THRESH_4TH_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_OUT_THRESH,
		TEGRA210_MBDRC_OUT_THRESH_1ST_MASK,
		params->out_thresh[0] << TEGRA210_MBDRC_OUT_THRESH_1ST_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_OUT_THRESH,
		TEGRA210_MBDRC_OUT_THRESH_2ND_MASK,
		params->out_thresh[1] << TEGRA210_MBDRC_OUT_THRESH_2ND_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_OUT_THRESH,
		TEGRA210_MBDRC_OUT_THRESH_3RD_MASK,
		params->out_thresh[2] << TEGRA210_MBDRC_OUT_THRESH_3RD_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_OUT_THRESH,
		TEGRA210_MBDRC_OUT_THRESH_4TH_MASK,
		params->out_thresh[3] << TEGRA210_MBDRC_OUT_THRESH_4TH_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_RATIO_1ST,
		TEGRA210_MBDRC_RATIO_1ST_MASK,
		params->ratio[0] << TEGRA210_MBDRC_RATIO_1ST_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_RATIO_2ND,
		TEGRA210_MBDRC_RATIO_2ND_MASK,
		params->ratio[1] << TEGRA210_MBDRC_RATIO_2ND_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_RATIO_3RD,
		TEGRA210_MBDRC_RATIO_3RD_MASK,
		params->ratio[2] << TEGRA210_MBDRC_RATIO_3RD_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_RATIO_4TH,
		TEGRA210_MBDRC_RATIO_4TH_MASK,
		params->ratio[3] << TEGRA210_MBDRC_RATIO_4TH_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_RATIO_5TH,
		TEGRA210_MBDRC_RATIO_5TH_MASK,
		params->ratio[4] << TEGRA210_MBDRC_RATIO_5TH_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_MAKEUP_GAIN,
		TEGRA210_MBDRC_MAKEUP_GAIN_MASK,
		params->makeup_gain << TEGRA210_MBDRC_MAKEUP_GAIN_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_INIT_GAIN,
		TEGRA210_MBDRC_INIT_GAIN_MASK,
		params->gain_init << TEGRA210_MBDRC_INIT_GAIN_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_GAIN_ATTACK,
		TEGRA210_MBDRC_GAIN_ATTACK_MASK,
		params->gain_attack_tc << TEGRA210_MBDRC_GAIN_ATTACK_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_GAIN_RELEASE,
		TEGRA210_MBDRC_GAIN_RELEASE_MASK,
		params->gain_release_tc << TEGRA210_MBDRC_GAIN_RELEASE_SHIFT);

	regmap_update_bits(mbdrc->regmap,
		reg_offset + TEGRA210_MBDRC_FAST_RELEASE,
		TEGRA210_MBDRC_FAST_RELEASE_MASK,
		params->fast_release_tc << TEGRA210_MBDRC_FAST_RELEASE_SHIFT);

	/* Program biquad filter parameter */
	if ((config_mode != MBDRC_MODE_FULLBAND) && (params->iir_stages > 0)) {
		u32 val = 0;

		/* For all pass tree filter stages are fixed */
		regmap_read(mbdrc->regmap, TEGRA210_MBDRC_CONFIG, &val);
		if (val & TEGRA210_MBDRC_CONFIG_FILT_STRUC_FLEX) {
			regmap_update_bits(mbdrc->regmap,
				reg_offset + TEGRA210_MBDRC_IIR_CONFIG,
				TEGRA210_MBDRC_IIR_CONFIG_NUM_STAGES_MASK,
				(params->iir_stages - 1) <<
				TEGRA210_MBDRC_IIR_CONFIG_NUM_STAGES_SHIFT);
		}

		tegra210_ahubram_write(mbdrc->dev,
			reg_offset + TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_CTRL,
			reg_offset + TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_DATA,
			0,
			(u32 *)&params->biq_params[0].biq_b0,
			TEGRA210_MBDRC_BIQ_PARAMS_PER_STAGE *
			TEGRA210_MBDRC_MAX_BIQ_STAGES);
	}

	return 0;
}

int tegra210_mbdrc_set_config(enum tegra210_ahub_cifs cif,
			      struct tegra210_mbdrc_config *config)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];
	unsigned int mask, val = 0;

	dev_vdbg(mbdrc->dev, "%s MBDRC set mode %d", __func__, config->mode);

	val = ((config->mode << TEGRA210_MBDRC_CONFIG_MBDRC_MODE_SHIFT) |
	    (config->rms_off << TEGRA210_MBDRC_CONFIG_RMS_OFFSET_SHIFT) |
	    (config->peak_rms_mode << TEGRA210_MBDRC_CONFIG_PEAK_RMS_SHIFT) |
	    (config->flit_struc << TEGRA210_MBDRC_CONFIG_FILT_STRUC_SHIFT) |
	    (config->shift_ctrl << TEGRA210_MBDRC_CONFIG_SHIFT_CTRL_SHIFT));

	switch (config->frame_size) {
	case 1:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N1;
		break;
	case 2:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N2;
		break;
	case 4:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N4;
		break;
	case 8:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N8;
		break;
	case 16:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N16;
		break;
	case 32:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N32;
		break;
	case 64:
		val |= TEGRA210_MBDRC_CONFIG_FRAME_SIZE_N64;
		break;
	default:
		dev_err(mbdrc->dev, "Unsupported framesize");
		return -EINVAL;
	}
	mask = (TEGRA210_MBDRC_CONFIG_MBDRC_MODE_MASK |
		TEGRA210_MBDRC_CONFIG_RMS_OFFSET_MASK |
		TEGRA210_MBDRC_CONFIG_PEAK_RMS_MASK |
		TEGRA210_MBDRC_CONFIG_FILT_STRUC_MASK |
		TEGRA210_MBDRC_CONFIG_SHIFT_CTRL_MASK |
		TEGRA210_MBDRC_CONFIG_FRAME_SIZE_MASK);

	regmap_update_bits(mbdrc->regmap,
		TEGRA210_MBDRC_CONFIG, mask, val);

	regmap_update_bits(mbdrc->regmap,
		TEGRA210_MBDRC_CHAN_MASK,
		TEGRA210_MBDRC_CHAN_MASK_MASK,
		config->chan_mask << TEGRA210_MBDRC_CHAN_MASK_SHIFT);

	val = (config->fa_fact << TEGRA210_MBDRC_FAST_FACTOR_FA_FACTOR_SHIFT |
	       config->fr_fact << TEGRA210_MBDRC_FAST_FACTOR_FR_FACTOR_SHIFT);

	mask = (TEGRA210_MBDRC_FAST_FACTOR_FA_FACTOR_MASK |
		TEGRA210_MBDRC_FAST_FACTOR_FR_FACTOR_MASK);

	regmap_update_bits(mbdrc->regmap,
		TEGRA210_MBDRC_FAST_FACTOR, mask, val);

	if (config->mode == MBDRC_MODE_FULLBAND) {
		tegra210_mbdrc_set_band_params(cif,
			&config->band_params[MBDRC_LOW_BAND],
			config->mode);
	} else if (config->mode == MBDRC_MODE_DUALBAND) {
		tegra210_mbdrc_set_band_params(cif,
			&config->band_params[MBDRC_LOW_BAND],
			config->mode);
		tegra210_mbdrc_set_band_params(cif,
			&config->band_params[MBDRC_HIGH_BAND],
			config->mode);
	} else if (config->mode == MBDRC_MODE_MULTIBAND) {
		tegra210_mbdrc_set_band_params(cif,
			&config->band_params[MBDRC_LOW_BAND],
			config->mode);
		tegra210_mbdrc_set_band_params(cif,
			&config->band_params[MBDRC_MID_BAND],
			config->mode);
		tegra210_mbdrc_set_band_params(cif,
			&config->band_params[MBDRC_HIGH_BAND],
			config->mode);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_set_config);

/*Set master volume applied before MBDRC */
int tegra210_mbdrc_set_master_vol(enum tegra210_ahub_cifs cif, unsigned int vol)
{
	int id = OPE_ID(cif);
	struct tegra210_mbdrc_ctx *mbdrc = tegra210_mbdrc[id];

	dev_vdbg(mbdrc->dev, "%s MBDRC master vol %d", __func__, vol);

	regmap_update_bits(mbdrc->regmap,
		TEGRA210_MBDRC_MASTER_VOLUME,
		TEGRA210_MBDRC_MASTER_VOLUME_MASK,
		vol << TEGRA210_MBDRC_MASTER_VOLUME_SHIFT);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mbdrc_set_master_vol);

/* Regmap callback functions */
static bool tegra210_mbdrc_wr_reg(struct device *dev, unsigned int reg)
{
	if (reg >= TEGRA210_MBDRC_IIR_CONFIG)
		reg -= ((reg - TEGRA210_MBDRC_IIR_CONFIG) %
			(TEGRA210_MBDRC_FILTER_PARAM_STRIDE *
			 TEGRA210_MBDRC_FILTER_COUNT));

	switch (reg) {
	case TEGRA210_MBDRC_CG:
	case TEGRA210_MBDRC_SOFT_RST:
	case TEGRA210_MBDRC_CONFIG:
	case TEGRA210_MBDRC_CHAN_MASK:
	case TEGRA210_MBDRC_MASTER_VOLUME:
	case TEGRA210_MBDRC_FAST_FACTOR:
	case TEGRA210_MBDRC_IIR_CONFIG:
	case TEGRA210_MBDRC_IN_ATTACK:
	case TEGRA210_MBDRC_IN_RELEASE:
	case TEGRA210_MBDRC_FAST_ATTACK:
	case TEGRA210_MBDRC_IN_THRESH:
	case TEGRA210_MBDRC_OUT_THRESH:
	case TEGRA210_MBDRC_RATIO_1ST:
	case TEGRA210_MBDRC_RATIO_2ND:
	case TEGRA210_MBDRC_RATIO_3RD:
	case TEGRA210_MBDRC_RATIO_4TH:
	case TEGRA210_MBDRC_RATIO_5TH:
	case TEGRA210_MBDRC_MAKEUP_GAIN:
	case TEGRA210_MBDRC_INIT_GAIN:
	case TEGRA210_MBDRC_GAIN_ATTACK:
	case TEGRA210_MBDRC_GAIN_RELEASE:
	case TEGRA210_MBDRC_FAST_RELEASE:
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_CTRL:
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_mbdrc_rd_reg(struct device *dev, unsigned int reg)
{
	if (reg >= TEGRA210_MBDRC_IIR_CONFIG)
		reg -= ((reg - TEGRA210_MBDRC_IIR_CONFIG) %
			(TEGRA210_MBDRC_FILTER_PARAM_STRIDE *
			 TEGRA210_MBDRC_FILTER_COUNT));
	switch (reg) {
	case TEGRA210_MBDRC_CG:
	case TEGRA210_MBDRC_SOFT_RST:
	case TEGRA210_MBDRC_STATUS:
	case TEGRA210_MBDRC_CONFIG:
	case TEGRA210_MBDRC_CHAN_MASK:
	case TEGRA210_MBDRC_MASTER_VOLUME:
	case TEGRA210_MBDRC_FAST_FACTOR:
	case TEGRA210_MBDRC_IIR_CONFIG:
	case TEGRA210_MBDRC_IN_ATTACK:
	case TEGRA210_MBDRC_IN_RELEASE:
	case TEGRA210_MBDRC_FAST_ATTACK:
	case TEGRA210_MBDRC_IN_THRESH:
	case TEGRA210_MBDRC_OUT_THRESH:
	case TEGRA210_MBDRC_RATIO_1ST:
	case TEGRA210_MBDRC_RATIO_2ND:
	case TEGRA210_MBDRC_RATIO_3RD:
	case TEGRA210_MBDRC_RATIO_4TH:
	case TEGRA210_MBDRC_RATIO_5TH:
	case TEGRA210_MBDRC_MAKEUP_GAIN:
	case TEGRA210_MBDRC_INIT_GAIN:
	case TEGRA210_MBDRC_GAIN_ATTACK:
	case TEGRA210_MBDRC_GAIN_RELEASE:
	case TEGRA210_MBDRC_FAST_RELEASE:
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_CTRL:
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_mbdrc_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg >= TEGRA210_MBDRC_IIR_CONFIG)
		reg -= ((reg - TEGRA210_MBDRC_IIR_CONFIG) %
			(TEGRA210_MBDRC_FILTER_PARAM_STRIDE *
			 TEGRA210_MBDRC_FILTER_COUNT));

	switch (reg) {
	case TEGRA210_MBDRC_SOFT_RST:
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_CTRL:
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_mbdrc_precious_reg(struct device *dev, unsigned int reg)
{
	if (reg >= TEGRA210_MBDRC_IIR_CONFIG)
		reg -= ((reg - TEGRA210_MBDRC_IIR_CONFIG) %
			(TEGRA210_MBDRC_FILTER_PARAM_STRIDE *
			 TEGRA210_MBDRC_FILTER_COUNT));

	switch (reg) {
	case TEGRA210_MBDRC_AHUBRAMCTL_MBDRC_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_mbdrc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_MBDRC_MAX_REG,
	.writeable_reg = tegra210_mbdrc_wr_reg,
	.readable_reg = tegra210_mbdrc_rd_reg,
	.volatile_reg = tegra210_mbdrc_volatile_reg,
	.precious_reg = tegra210_mbdrc_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_mbdrc_runtime_suspend(struct device *dev)
{
	struct tegra210_mbdrc_ctx *mbdrc = dev_get_drvdata(dev);

	dev_dbg(mbdrc->dev, "%s MBDRC %d", __func__, mbdrc->id);
	regcache_cache_only(mbdrc->regmap, true);
	tegra210_axbar_disable_clk();

	return 0;
}

static int tegra210_mbdrc_runtime_resume(struct device *dev)
{
	struct tegra210_mbdrc_ctx *mbdrc = dev_get_drvdata(dev);

	dev_dbg(mbdrc->dev, "%s MBDRC %d", __func__, mbdrc->id);

	tegra210_axbar_enable_clk();
	regcache_cache_only(mbdrc->regmap, false);

	return 0;
}

/*
MBDRC Driver probe and remove functions
*/
static int tegra210_mbdrc_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_mbdrc_ctx *mbdrc;
	int ret = 0;
	int id = 0;

	if (!pdev->dev.of_node) {
		pr_err("%s Probe failed. DT node missing.", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-mbdrc-id",
			     &pdev->id);

	dev_dbg(&pdev->dev, "%s MBDRC %d : starting probe",
		__func__, pdev->id);

	if ((pdev->id < 0) ||
	    (pdev->id >= TEGRA210_MBDRC_COUNT)) {
		dev_err(&pdev->dev, "MBDRC id %d out of range\n", pdev->id);
		return -EINVAL;
	}
	id = pdev->id;

	tegra210_mbdrc[id] = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_mbdrc_ctx),
			   GFP_KERNEL);
	if (!tegra210_mbdrc[id]) {
		dev_err(&pdev->dev, "Can't allocate mbdrc context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_mbdrc[id]->dev = &pdev->dev;

	mbdrc = tegra210_mbdrc[id];
	mbdrc->id = id;
	dev_set_drvdata(&pdev->dev, mbdrc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory 0 resource\n");
		ret = -ENODEV;
		goto err_free;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), pdev->name);
	if (!region) {
		dev_err(&pdev->dev, "Memory region 0 already claimed\n");
		ret = -EBUSY;
		goto err_free;
	}

	mbdrc->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mbdrc->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	mbdrc->regmap = devm_regmap_init_mmio(&pdev->dev, mbdrc->regs,
					    &tegra210_mbdrc_regmap_config);
	if (IS_ERR(mbdrc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(mbdrc->regmap);
		goto err_free;
	}
	regcache_cache_only(mbdrc->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_mbdrc_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(mbdrc->dev, "%s MBDRC %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_mbdrc[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s MBDRC %d probe failed with %d",
		__func__, pdev->id, ret);
	return ret;
}


static int tegra210_mbdrc_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_mbdrc_runtime_suspend(&pdev->dev);

	tegra210_mbdrc[pdev->id] = NULL;

	dev_dbg(&pdev->dev, "%s MBDRC %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_mbdrc_of_match[] = {
	{ .compatible = "nvidia,tegra210-mbdrc",},
	{},
};

static const struct dev_pm_ops tegra210_mbdrc_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_mbdrc_runtime_suspend,
			   tegra210_mbdrc_runtime_resume, NULL)
};

static struct platform_driver tegra210_mbdrc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_mbdrc_of_match,
		.pm = &tegra210_mbdrc_pm_ops,
	},
	.probe = tegra210_mbdrc_probe,
	.remove = tegra210_mbdrc_remove,
};
module_platform_driver(tegra210_mbdrc_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra APE MBDRC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
