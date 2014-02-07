/*
 * tegra210_spkprot.c - Tegra210 SPKPROT driver.
 *
 * Author: Vinod Subbarayalu <vsubbarayalu@nvidia.com>
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

/*#define VERBOSE_DEBUG
#define DEBUG*/
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_spkprot.h"
#include "tegra210_axbar.h"
#include "tegra210_ahub_utils.h"

#define DRV_NAME "tegra210-spkprot"

/* TEGRA210 SPKPROT driver context */
struct tegra210_spkprot_ctx {
	struct device	*dev;
	void __iomem	*regs;
	struct regmap	*regmap;
	int		id;
	bool		in_use;
};

static struct tegra210_spkprot_ctx *tegra210_spkprot[TEGRA210_SPKPROT_COUNT];

/* Regmap callback functions */
static bool tegra210_spkprot_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SPKPROT_XBAR_RX_INT_MASK:
	case TEGRA210_SPKPROT_XBAR_RX_CIF_CTRL:
	case TEGRA210_SPKPROT_XBAR_TX_INT_MASK:
	case TEGRA210_SPKPROT_XBAR_TX_CIF_CTRL:
	case TEGRA210_SPKPROT_ENABLE:
	case TEGRA210_SPKPROT_CG:
	case TEGRA210_SPKPROT_CONFIG:
	case TEGRA210_SPKPROT_LGAINATTACK:
	case TEGRA210_SPKPROT_HGAINATTACK:
	case TEGRA210_SPKPROT_LGAINRELEASE:
	case TEGRA210_SPKPROT_HGAINRELEASE:
	case TEGRA210_SPKPROT_LGAIN_FASTRELEASE:
	case TEGRA210_SPKPROT_HGAIN_FASTRELEASE:
	case TEGRA210_SPKPROT_FASTFACTOR:
	case TEGRA210_SPKPROT_THRESH_DB:
	case TEGRA210_SPKPROT_RATIO:
	case TEGRA210_SPKPROT_SOFT_RESET:
	case TEGRA210_SPKPROT_XBAR_RX_INT_SET:
	case TEGRA210_SPKPROT_XBAR_RX_INT_CLEAR:
	case TEGRA210_SPKPROT_XBAR_TX_INT_SET:
	case TEGRA210_SPKPROT_XBAR_TX_INT_CLEAR:
	case TEGRA210_SPKPROT_ARSWITCH:
	case TEGRA210_SPKPROT_GAINSWITCH:
	case TEGRA210_SPKPROT_THRESHSWITCH:
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_spkprot_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SPKPROT_XBAR_RX_STATUS:
	case TEGRA210_SPKPROT_XBAR_RX_INT_STATUS:
	case TEGRA210_SPKPROT_XBAR_TX_STATUS:
	case TEGRA210_SPKPROT_XBAR_TX_INT_STATUS:
	case TEGRA210_SPKPROT_STATUS:
	case TEGRA210_SPKPROT_INT_STATUS:
	case TEGRA210_SPKPROT_CONFIG_ERR_TYPE:
	case TEGRA210_SPKPROT_XBAR_RX_INT_MASK:
	case TEGRA210_SPKPROT_XBAR_RX_CIF_CTRL:
	case TEGRA210_SPKPROT_XBAR_TX_INT_MASK:
	case TEGRA210_SPKPROT_XBAR_TX_CIF_CTRL:
	case TEGRA210_SPKPROT_ENABLE:
	case TEGRA210_SPKPROT_CG:
	case TEGRA210_SPKPROT_CONFIG:
	case TEGRA210_SPKPROT_LGAINATTACK:
	case TEGRA210_SPKPROT_HGAINATTACK:
	case TEGRA210_SPKPROT_LGAINRELEASE:
	case TEGRA210_SPKPROT_HGAINRELEASE:
	case TEGRA210_SPKPROT_LGAIN_FASTRELEASE:
	case TEGRA210_SPKPROT_HGAIN_FASTRELEASE:
	case TEGRA210_SPKPROT_FASTFACTOR:
	case TEGRA210_SPKPROT_THRESH_DB:
	case TEGRA210_SPKPROT_RATIO:
	case TEGRA210_SPKPROT_XBAR_RX_INT_SET:
	case TEGRA210_SPKPROT_XBAR_RX_INT_CLEAR:
	case TEGRA210_SPKPROT_XBAR_TX_INT_SET:
	case TEGRA210_SPKPROT_XBAR_TX_INT_CLEAR:
	case TEGRA210_SPKPROT_ARSWITCH:
	case TEGRA210_SPKPROT_GAINSWITCH:
	case TEGRA210_SPKPROT_THRESHSWITCH:
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_spkprot_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SPKPROT_SOFT_RESET:
	case TEGRA210_SPKPROT_XBAR_RX_INT_SET:
	case TEGRA210_SPKPROT_XBAR_RX_INT_CLEAR:
	case TEGRA210_SPKPROT_XBAR_TX_INT_SET:
	case TEGRA210_SPKPROT_XBAR_TX_INT_CLEAR:
	case TEGRA210_SPKPROT_ARSWITCH:
	case TEGRA210_SPKPROT_GAINSWITCH:
	case TEGRA210_SPKPROT_THRESHSWITCH:
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_CTRL:
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_spkprot_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA:
	case TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_spkprot_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA,
	.writeable_reg = tegra210_spkprot_wr_reg,
	.readable_reg = tegra210_spkprot_rd_reg,
	.volatile_reg = tegra210_spkprot_volatile_reg,
	.precious_reg = tegra210_spkprot_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* SPKPROT APIs definition*/

/**
 *	tegra210_spkprot_get- allocate a spkprot
 *	@id : spkprot id (pass id = -1 to allocate first available spkprot)
 */
int tegra210_spkprot_get(enum tegra210_ahub_cifs *cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(*cif);
	int i;

	spkprot = tegra210_spkprot[id];

	if ((id < 0) || (id >= TEGRA210_SPKPROT_COUNT)) {
		pr_err("%s Invalid SPKPROT id %d", __func__, id);
		return -EINVAL;
	}

	if (!spkprot->in_use) {
		dev_vdbg(spkprot->dev, "%s SKPROT %d", __func__, id);
		spkprot->in_use = true;
		pm_runtime_get_sync(spkprot->dev);
		return 0;
	} else {
		dev_err(spkprot->dev, "%s SPKPROT %d can't allocate",
				__func__, id);
		return -ENOMEM;
	}

	for (i = 0; i < TEGRA210_SPKPROT_COUNT; i++) {
		spkprot = tegra210_spkprot[i];
		if (!spkprot->in_use) {
			dev_vdbg(spkprot->dev, "%s allocated SPKPROT %d",
				 __func__, id);
			spkprot->in_use = true;
			pm_runtime_get_sync(spkprot->dev);
			id = i;
			return 0;
		}
	}

	pr_err("%s allocation failed, no SPKPROT is free", __func__);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_get);

/**
 *	tegra210_spkprot_put - free a spkprot
 *	@id : spkprot id
 */
int tegra210_spkprot_put(enum tegra210_ahub_cifs cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];

	if ((id < 0) || (id >= TEGRA210_SPKPROT_COUNT)) {
		pr_err("%s Invalid SPKPROT id %d", __func__, id);
		return -EINVAL;
	}

	if (spkprot->in_use) {
		dev_vdbg(spkprot->dev, "%s freed SPKPROT %d", __func__, id);
		pm_runtime_put_sync(spkprot->dev);
		spkprot->in_use = false;
	} else
		dev_info(spkprot->dev, "%s SPKPROT %d is already free",
			__func__, id);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_put);

/**
 *	tegra210_spkprot_set_acif_param - set acif param for a spkprot
 *	@id : spkprot id
 *	@cif_id : cif id
 *	@acif : acif param values
 */
int tegra210_spkprot_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);
	u32 reg;

	reg = (IS_SPKPROT_TX(cif) ? TEGRA210_SPKPROT_XBAR_TX_CIF_CTRL :
			TEGRA210_SPKPROT_XBAR_RX_CIF_CTRL);
	spkprot = tegra210_spkprot[id];

	dev_vdbg(spkprot->dev, "%s SPKPROT cif %d", __func__, cif);
	return tegra210_set_axbar_cif_param(spkprot->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_acif_param);

/**
 *	tegra210_spkprot_get_acif_param - get acif param for a spkprot
 *	@id : spkprot id
 *	@cif_id : cif id
 *	@acif : acif struct to be populated
 */
int tegra210_spkprot_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);
	u32 reg;


	reg = (IS_SPKPROT_TX(cif) ? TEGRA210_SPKPROT_XBAR_TX_CIF_CTRL :
		TEGRA210_SPKPROT_XBAR_RX_CIF_CTRL);

	spkprot = tegra210_spkprot[id];

	dev_vdbg(spkprot->dev, "%s SPKPROT cif %d", __func__, cif);
	return tegra210_get_axbar_cif_param(spkprot->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_get_acif_param);

int tegra210_spkprot_is_enable(enum tegra210_ahub_cifs cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);
	u32 val = 0;

	spkprot = tegra210_spkprot[id];
	regmap_read(spkprot->regmap, TEGRA210_SPKPROT_STATUS, &val);

	val &= TEGRA210_SPKPROT_ENABLE_STATUS;

	dev_vdbg(spkprot->dev, "%s SPKPROT enable %d", __func__, val);

	return val;
}

/**
 *	tegra210_spkprot_enable - enable or disable a spkprot
 *	@id : spkprot id
 *	@en : enable/disable flag
 */
int tegra210_spkprot_enable(enum tegra210_ahub_cifs cif, bool en)
{
	struct tegra210_spkprot_ctx *spkprot;
	int cnt = AHUB_OP_MAX_RETRY;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s SPKPROT %d enable %d", __func__, id, en);

	regmap_update_bits(spkprot->regmap,
			TEGRA210_SPKPROT_ENABLE,
			TEGRA210_SPKPROT_ENABLE_EN,
			en ? TEGRA210_SPKPROT_ENABLE_EN : 0);

	while ((tegra210_spkprot_is_enable(cif) != en) && cnt--)
		udelay(AHUB_OP_SLEEP_US);

	if (cnt < 0) {
		dev_err(spkprot->dev, "%s Failed to %s SPKPROT",
			__func__, en ? "enable" : "disable");
		return -ETIMEDOUT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_enable);

/**
 *	tegra210_spkprot_reset - reset a spkprot
 *	@id : spkprot id
 */
int tegra210_spkprot_reset(enum tegra210_ahub_cifs cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s reset SPKPROT %d", __func__, id);

	regmap_update_bits(spkprot->regmap,
			TEGRA210_SPKPROT_SOFT_RESET,
			TEGRA210_SPKPROT_SOFT_RESET_EN,
			TEGRA210_SPKPROT_SOFT_RESET_EN);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_reset);

/**
 *	tegra210_spkprot_set_slcg - enable/disable second level clock gating for a spkprot
 *	@id : spkprot id
 *	@en : enable/disable flag
 */
int tegra210_spkprot_set_slcg(enum tegra210_ahub_cifs cif, bool en)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s SPKPROT %d slcg enable %d",
			__func__, id, en);

	regmap_update_bits(spkprot->regmap,
			TEGRA210_SPKPROT_CG,
			TEGRA210_SPKPROT_CG_SLCG_EN,
			en ? TEGRA210_SPKPROT_CG_SLCG_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_slcg);

/*Set spkprot config parameters*/
int tegra210_spkprot_set_config(enum tegra210_ahub_cifs cif,
		struct tegra210_spkprot_config_param *param)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s SPKPROT %d", __func__, id);

	regmap_update_bits(spkprot->regmap,
		TEGRA210_SPKPROT_CONFIG,
		TEGRA210_SPKPROT_CONFIG_MODE_MASK,
		param->mode << TEGRA210_SPKPROT_CONFIG_MODE_SHIFT);

	regmap_update_bits(spkprot->regmap,
		TEGRA210_SPKPROT_CONFIG,
		TEGRA210_SPKPROT_CONFIG_ROUND_MASK,
		param->bias << TEGRA210_SPKPROT_CONFIG_ROUND_SHIFT);

	regmap_update_bits(spkprot->regmap,
		TEGRA210_SPKPROT_CONFIG,
		TEGRA210_SPKPROT_CONFIG_BAND_FILTER_MASK,
		param->bandfilter_type <<
			TEGRA210_SPKPROT_CONFIG_BAND_FILTER_SHIFT);


	if ((param->mode == SPKPROT_MODE_DUALBAND)
		&& (param->bandfilter_type == SPKPROT_BANDFILTER_FLEX)) {

		regmap_update_bits(spkprot->regmap,
			TEGRA210_SPKPROT_CONFIG,
			TEGRA210_SPKPROT_CONFIG_BAND_BIQUAD_STAGES_MASK,
			(param->bandfilter_biquad_stages - 1) <<
			TEGRA210_SPKPROT_CONFIG_BAND_BIQUAD_STAGES_SHIFT);
	}

	regmap_update_bits(spkprot->regmap,
			TEGRA210_SPKPROT_CONFIG,
			TEGRA210_SPKPROT_CONFIG_AR_BIQUAD_STAGES_MASK,
			(param->arfilter_biquad_stages - 1) <<
				TEGRA210_SPKPROT_CONFIG_AR_BIQUAD_STAGES_SHIFT);

	regmap_update_bits(spkprot->regmap,
		TEGRA210_SPKPROT_CONFIG,
		TEGRA210_SPKPROT_CONFIG_SP_BIQUAD_STAGES_MASK,
		(param->spfilter_biquad_stages - 1) <<
			TEGRA210_SPKPROT_CONFIG_SP_BIQUAD_STAGES_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_config);

/*Set Antiresonance filter coefficients*/
int tegra210_spkprot_set_arcoef(enum tegra210_ahub_cifs cif,
				struct spkprot_arcoeff_params *params,
				int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 val = 0;
	int cnt = AHUB_OP_MAX_RETRY;
	u32 start_addr = 0;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s SPKPROT %d", __func__, id);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap, TEGRA210_SPKPROT_ARSWITCH , &val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT ARSWITCH-NOTREADY %d", __func__, id);
		return -ETIMEDOUT;
	}

	start_addr = channel_id * ARFILTER_RAM_DEPTH;
	tegra210_ahubram_write(spkprot->dev,
		TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_CTRL,
		TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_DATA,
		start_addr, (u32 *)params, ARFILTER_RAM_DEPTH);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_arcoef);

/*Set Speaker protection filter coefficients*/
int tegra210_spkprot_set_spcoef(enum tegra210_ahub_cifs cif,
				struct spkprot_spcoeff_params *params,
				int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 start_addr = 0;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);

	start_addr = channel_id * SPFILTER_RAM_DEPTH;
	tegra210_ahubram_write(spkprot->dev,
		TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_CTRL,
		TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA,
		start_addr, (u32 *)params, SPFILTER_RAM_DEPTH);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_spcoef);

/*Set Bandfilter coefficiemts*/
int tegra210_spkprot_set_bandcoef(enum tegra210_ahub_cifs cif,
			struct spkprot_bandcoeff_params *params)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);

	tegra210_ahubram_write(spkprot->dev,
		TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_CTRL,
		TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_DATA,
		0, (u32 *)params, BANDFILTER_RAM_DEPTH);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_bandcoef);

/*Set gainattack_l*/
int tegra210_spkprot_set_gainattack_l(enum tegra210_ahub_cifs cif,
					s32 gainattack_l,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);

	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_LGAINATTACK + offset,
		gainattack_l);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_gainattack_l);

/*Set gainattack_h*/
int tegra210_spkprot_set_gainattack_h(enum tegra210_ahub_cifs cif,
					s32 gainattack_h,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_HGAINATTACK + offset,
		gainattack_h);
	return 0;
}

/*Set gainrelease_l*/
int tegra210_spkprot_set_gainrelease_l(enum tegra210_ahub_cifs cif,
					s32 gainrelease_l,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_LGAINRELEASE + offset,
		gainrelease_l);
	return 0;
}

/*Set gainrelease_h*/
int tegra210_spkprot_set_gainrelease_h(enum tegra210_ahub_cifs cif,
					s32 gainrelease_h,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	int id = SPKPROT_ID(cif);
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_HGAINRELEASE + offset,
			gainrelease_h);
	return 0;
}

/*Set fastrelease_l*/
int tegra210_spkprot_set_fastrelease_l(enum tegra210_ahub_cifs cif,
					s32 fastrelease_l,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap,
			TEGRA210_SPKPROT_LGAIN_FASTRELEASE + offset,
			fastrelease_l);
	return 0;
}

/*Set fastrelease_h*/
int tegra210_spkprot_set_fastrelease_h(enum tegra210_ahub_cifs cif,
					s32 fastrelease_h,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap,
		TEGRA210_SPKPROT_HGAIN_FASTRELEASE + offset,
		fastrelease_h);
	return 0;
}

/*Set fastfactor*/
int tegra210_spkprot_set_fastfactor(enum tegra210_ahub_cifs cif,
					s32 fastfactor,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_FASTFACTOR + offset,
		fastfactor);
	return 0;
}

/*Set ratio*/
int tegra210_spkprot_set_ratio(enum tegra210_ahub_cifs cif,
				s32 ratio,
				int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);
	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_RATIO + offset,
		ratio);
	return 0;
}

/*Set threshold*/
int tegra210_spkprot_set_threshold(enum tegra210_ahub_cifs cif, s32 threshold,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 val = 0;
	u32 offset = channel_id * TEGRA210_SPKPROT_STRIDE;
	int cnt = AHUB_OP_MAX_RETRY;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap,
			TEGRA210_SPKPROT_THRESHSWITCH,
			&val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT THSWITCH-NOTREADY %d", __func__, id);
		return -ETIMEDOUT;
	}

	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_THRESH_DB + offset,
		threshold);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_threshold);

/*Set gain*/
int tegra210_spkprot_set_newgain_l(enum tegra210_ahub_cifs cif, u32 *params,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 val = 0;
	int cnt = AHUB_OP_MAX_RETRY;
	u32 start_addr = 0;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s SPKPROT %d", __func__, id);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap, TEGRA210_SPKPROT_GAINSWITCH, &val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT GAINSWITCH-NOTREADY %d", __func__, id);
		return -ETIMEDOUT;
	}

	start_addr = channel_id * GAIN_RAM_DEPTH;
	tegra210_ahubram_write(spkprot->dev,
		TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_CTRL,
		TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA,
		start_addr, params, GAIN_RAM_DEPTH);

	return 0;
}

int tegra210_spkprot_set_newgain_h(enum tegra210_ahub_cifs cif,
					u32 *params,
					int channel_id,
					int max_supported_channels)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 val = 0;
	int cnt = AHUB_OP_MAX_RETRY;
	u32 start_addr = 0;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s SPKPROT %d", __func__, id);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap, TEGRA210_SPKPROT_GAINSWITCH, &val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT GAINSWITCH-NOTREADY %d", __func__, id);
		return -ETIMEDOUT;
	}

	start_addr = max_supported_channels + channel_id;
	tegra210_ahubram_write(spkprot->dev,
	TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_CTRL,
	TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA,
		start_addr, params, GAIN_RAM_DEPTH);

	return 0;
}

int tegra210_spkprot_set_newgain(enum tegra210_ahub_cifs cif,
					u32 *gain_l,
					u32 *gain_h,
					int channel_id,
					int max_supported_channels)
{
	tegra210_spkprot_set_newgain_l(cif, gain_l, channel_id);

	tegra210_spkprot_set_newgain_h(cif, gain_l, channel_id,
					max_supported_channels);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_newgain);

/*Set initgain*/
int tegra210_spkprot_set_initgain_l(enum tegra210_ahub_cifs cif,
					u32 *params,
					int channel_id)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 start_addr = 0;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);

	start_addr = channel_id * GAIN_RAM_DEPTH;

	tegra210_ahubram_write(spkprot->dev,
		TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_CTRL,
		TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA,
		start_addr, params, GAIN_RAM_DEPTH);
	return 0;
}

int tegra210_spkprot_set_initgain_h(enum tegra210_ahub_cifs cif,
					u32 *params,
					int channel_id,
					int max_supported_channels)
{
	struct tegra210_spkprot_ctx *spkprot;
	u32 start_addr = 0;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];
	dev_vdbg(spkprot->dev, "%s  SPKPROT %d", __func__, id);

	start_addr = max_supported_channels + channel_id;

	tegra210_ahubram_write(spkprot->dev,
		TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_CTRL,
		TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA,
		start_addr, params, GAIN_RAM_DEPTH);
	return 0;
}

int tegra210_spkprot_set_initgain(enum tegra210_ahub_cifs cif,
					u32 *gain_l,
					u32 *gain_h,
					int channel_id,
					int max_supported_channels)
{
	tegra210_spkprot_set_initgain_l(cif, gain_l, channel_id);

	tegra210_spkprot_set_initgain_h(cif, gain_l, channel_id,
						max_supported_channels);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_initgain);

int tegra210_spkprot_set_algo_params(enum tegra210_ahub_cifs cif,
				struct tegra210_spkprot_algo_params params)
{
	u32 ch_id = params.channel_id;

	tegra210_spkprot_set_gainattack_l(cif,
				params.gainattack_l, ch_id);
	tegra210_spkprot_set_gainrelease_l(cif,
				params.gainrelease_l, ch_id);
	tegra210_spkprot_set_fastrelease_l(cif,
				params.fastrelease_l, ch_id);

	if (params.mode == SPKPROT_MODE_DUALBAND) {
		tegra210_spkprot_set_gainattack_h(cif,
			params.gainattack_h, ch_id);
		tegra210_spkprot_set_gainrelease_h(cif,
			params.gainrelease_l, ch_id);
		tegra210_spkprot_set_fastrelease_h(cif,
			params.fastrelease_l, ch_id);
	}

	tegra210_spkprot_set_fastfactor(cif,
			params.fastfactor, ch_id);
	tegra210_spkprot_set_ratio(cif,
			params.ratio, ch_id);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_algo_params);

int tegra210_spkprot_set_arswitch(enum tegra210_ahub_cifs cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int cnt = AHUB_OP_MAX_RETRY;
	u32 val = TEGRA210_SPKPROT_SWITCH_ENABLE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];

	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_ARSWITCH , val);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap, TEGRA210_SPKPROT_ARSWITCH , &val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT ARSWITCH-APPLY %d", __func__, id);
		return -ETIMEDOUT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_arswitch);

int tegra210_spkprot_set_gainswitch(enum tegra210_ahub_cifs cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int cnt = AHUB_OP_MAX_RETRY;
	u32 val = TEGRA210_SPKPROT_SWITCH_ENABLE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];

	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_GAINSWITCH , val);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap,
			TEGRA210_SPKPROT_GAINSWITCH , &val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT GAINSWITCH-APPLY %d", __func__, id);
		return -ETIMEDOUT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_gainswitch);

int tegra210_spkprot_set_threshswitch(enum tegra210_ahub_cifs cif)
{
	struct tegra210_spkprot_ctx *spkprot;
	int cnt = AHUB_OP_MAX_RETRY;
	u32 val = TEGRA210_SPKPROT_SWITCH_ENABLE;
	int id = SPKPROT_ID(cif);

	spkprot = tegra210_spkprot[id];

	regmap_write(spkprot->regmap, TEGRA210_SPKPROT_THRESHSWITCH , val);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(spkprot->regmap,
			TEGRA210_SPKPROT_THRESHSWITCH , &val);
	} while ((val & TEGRA210_SPKPROT_SWITCH_ACK) && cnt--);

	if (cnt < 0) {
		dev_err(spkprot->dev,
		"%s Failed SPKPROT THRESHSWITCH-APPLY %d", __func__, id);
		return -ETIMEDOUT;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_spkprot_set_threshswitch);

/* PM runtime callback functions */
static int tegra210_spkprot_runtime_suspend(struct device *dev)
{
		struct tegra210_spkprot_ctx *spkprot = dev_get_drvdata(dev);

		dev_dbg(spkprot->dev, "%s SPKPROT %d", __func__, spkprot->id);
		tegra210_axbar_disable_clk();
		regcache_cache_only(spkprot->regmap, true);

		return 0;
}

static int tegra210_spkprot_runtime_resume(struct device *dev)
{
	struct tegra210_spkprot_ctx *spkprot = dev_get_drvdata(dev);

	dev_dbg(spkprot->dev, "%s SPKPROT %d", __func__, spkprot->id);
	tegra210_axbar_enable_clk();
	regcache_cache_only(spkprot->regmap, false);

	return 0;
}

/* SPKPROT Driver probe and remove functions */
static int tegra210_spkprot_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_spkprot_ctx *spkprot;
	int ret = 0;
	int id = 0;

	dev_dbg(&pdev->dev, "%s SPKPROT %d : starting probe",
		__func__, pdev->id);

	if (pdev->dev.of_node)
		of_property_read_u32(pdev->dev.of_node,
			"nvidia,ahub-spkprot-id",
			&pdev->id);
	else
		dev_info(&pdev->dev, "No device tree node for SPKPROT driver");

	if ((pdev->id < 0) ||
		(pdev->id >= TEGRA210_SPKPROT_COUNT)) {
		dev_err(&pdev->dev, "SPKPROT id %d out of range\n", pdev->id);
		return -EINVAL;
	}
	id = pdev->id;

	tegra210_spkprot[id] = devm_kzalloc(&pdev->dev,
				 sizeof(struct tegra210_spkprot_ctx),
					GFP_KERNEL);
	if (!tegra210_spkprot[id]) {
		dev_err(&pdev->dev, "Can't allocate spkprot context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_spkprot[id]->dev = &pdev->dev;

	spkprot = tegra210_spkprot[id];
	spkprot->id = id;
	dev_set_drvdata(&pdev->dev, spkprot);

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

	spkprot->regs = devm_ioremap(&pdev->dev, res->start,
		resource_size(res));
	if (!spkprot->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	spkprot->regmap = devm_regmap_init_mmio(&pdev->dev,
						spkprot->regs,
					&tegra210_spkprot_regmap_config);
	if (IS_ERR(spkprot->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(spkprot->regmap);
		goto err_free;
	}
	regcache_cache_only(spkprot->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_spkprot_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(spkprot->dev, "%s SPKPROT %d probe successful",
		__func__, pdev->id);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_spkprot[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s SPKPROT %d probe failed with %d",
		__func__, pdev->id, ret);
	return ret;
}

static int tegra210_spkprot_remove(struct platform_device *pdev)
{
	struct tegra210_spkprot_ctx *spkprot;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_spkprot_runtime_suspend(&pdev->dev);

	spkprot = platform_get_drvdata(pdev);
	tegra210_spkprot[pdev->id] = NULL;

	dev_dbg(&pdev->dev, "%s SPKPROT %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_spkprot_of_match[] = {
	{ .compatible = "nvidia,tegra210-spkprot",},
	{},
};

static const struct dev_pm_ops tegra210_spkprot_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_spkprot_runtime_suspend,
				 tegra210_spkprot_runtime_resume, NULL)
};

static struct platform_driver tegra210_spkprot_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_spkprot_of_match,
		.pm = &tegra210_spkprot_pm_ops,
	},
	.probe = tegra210_spkprot_probe,
	.remove = tegra210_spkprot_remove,
};
module_platform_driver(tegra210_spkprot_driver);

MODULE_AUTHOR("Vinod Subbarayalu <vsubbarayalu@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 SPKPROT driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
