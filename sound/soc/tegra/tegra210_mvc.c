/*
 * tegra210_mvc.c - Tegra210 MVC driver.
 *
 * Author: Ashok Mudithanapalli <ashokm@nvidia.com>
 *
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
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
#define VERBOSE_DEBUG
#define DEBUG
*/

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include "tegra210_mvc.h"
#include "tegra210_axbar.h"


#define DRV_NAME "tegra210-mvc"

/* TEGRA210 MVC driver context */
struct tegra210_mvc_ctx {
	struct device	*dev;
	void __iomem	*regs;
	struct regmap	*regmap;
	int			id;
	int		in_use;
	int			enable_count;
};

static struct tegra210_mvc_ctx *tegra210_mvc[TEGRA210_MVC_COUNT];

/* MV API's defination */

/**
 *  tegra210_mvc_get - gets mvc before use
 * If cif = CIF_NONE is passed, mvc driver will allocate any available
 * free instance
 */
int tegra210_mvc_get(enum tegra210_ahub_cifs *cif)
{
	enum tegra210_cif_dir dir = IS_MVC_RX(*cif) ? DIR_RX : DIR_TX;
	int i, id = MVC_ID(*cif);
	struct tegra210_mvc_ctx *mvc;

	if ((id >= 0) && (id < TEGRA210_MVC_COUNT)) {
		mvc = tegra210_mvc[id];
		if (!mvc->in_use) {
			dev_vdbg(mvc->dev, "%s allocated mvc %d", __func__, id);
			mvc->in_use = true;
			pm_runtime_get_sync(mvc->dev);
			return 0;
		} else {
			dev_err(mvc->dev, "%s mvc %d not free", __func__, id);
			return -EBUSY;
		}
	}

	for (i = 0; i < TEGRA210_MVC_COUNT; i++) {
		mvc = tegra210_mvc[i];
		if (!mvc->in_use) {
			dev_vdbg(mvc->dev, "%s allocated mvc %d", __func__, i);
			mvc->in_use = true;
			*cif = (dir == DIR_RX) ? (MVC1_RX0 + i) :
						 (MVC1_TX0 + i);
			pm_runtime_get_sync(mvc->dev);
			return 0;
		}
	}

	pr_err("%s allocation failed, no mvc is free", __func__);
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_get);

/**
 *  tegra210_mvc_put - frees mvc after use
 */
int tegra210_mvc_put(enum tegra210_ahub_cifs cif)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	if (mvc->in_use) {
		dev_vdbg(mvc->dev, "%s freed mvc %d", __func__, id);
		mvc->in_use = false;
		pm_runtime_put_sync(mvc->dev);
	} else
		dev_info(mvc->dev, "%s mvc %d already free", __func__, id);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_put);

/**
 *  tegra210_mvc_enable - enables/disables mvc globally
 *  @en : enable/disable flag
 */
int tegra210_mvc_enable(enum tegra210_ahub_cifs cif, bool en)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];
	int cnt = AHUB_OP_MAX_RETRY;

	dev_vdbg(mvc->dev, "%s MVC enable en=%d count %d",
		 __func__, en, mvc->enable_count);

	if (en) {
		mvc->enable_count++;
		if (mvc->enable_count > 1)
			return 0;
	} else if (mvc->enable_count > 0) {
		mvc->enable_count--;
		if (mvc->enable_count > 0)
			return 0;
	}

	regmap_update_bits(mvc->regmap,
			   TEGRA210_MVC_ENABLE,
			   TEGRA210_MVC_ENABLE_EN,
			   en ? TEGRA210_MVC_ENABLE_EN : 0);

	/* ensure MVC is enabled */
	while ((tegra210_mvc_is_enable(cif) != en) && cnt--)
		udelay(AHUB_OP_SLEEP_US);

	if (!cnt) {
		dev_warn(mvc->dev, "%s Failed to %sMVC",
			 __func__, en ? "enable" : "disable");
		tegra210_mvc_reset(cif);
		return -ETIME;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_enable);

/**
 *  tegra210_mvc_reset - soft resets mvc globally
 */
int tegra210_mvc_reset(enum tegra210_ahub_cifs cif)
{
	u32 val = 0;
	int cnt = AHUB_OP_MAX_RETRY;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s reset mvc id %d ", __func__, id);
	regmap_update_bits(mvc->regmap,
			TEGRA210_MVC_SOFT_RESET,
			TEGRA210_MVC_SOFT_RESET_EN,
			TEGRA210_MVC_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(mvc->regmap, TEGRA210_MVC_SOFT_RESET,
				&val);
	} while ((val & TEGRA210_MVC_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(mvc->dev, "%s Failed to reset mvc",
				__func__);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_reset);

/**
 *  tegra210_mvc_set_slcg - sets second level clock gating of mvc
 *  @en : enable/disable flag
 */
int tegra210_mvc_set_slcg(enum tegra210_ahub_cifs cif, bool en)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d slcg enable %d", __func__, id, en);

	regmap_update_bits(mvc->regmap,
			TEGRA210_MVC_CG,
			TEGRA210_MVC_CG_SLCG_EN,
			en ? TEGRA210_MVC_CG_SLCG_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_slcg);

/**
 *  tegra210_mvc_is_enable - check whether mvc is enable or not
 */
int tegra210_mvc_is_enable(enum tegra210_ahub_cifs cif)
{
	u32 val = 0;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	regmap_read(mvc->regmap, TEGRA210_MVC_STATUS, &val);
	val &= TEGRA210_MVC_STATUS_ENABLE_STATUS;

	dev_vdbg(mvc->dev, "%s MVC enable %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_is_enable);

/**
 *  tegra210_mvc_is_slcg_enable - check whether second clk gating enable
 */
int tegra210_mvc_is_slcg_enable(enum tegra210_ahub_cifs cif)
{
	u32 val = 0;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	regmap_read(mvc->regmap, TEGRA210_MVC_STATUS, &val);
	val &= TEGRA210_MVC_STATUS_SLCG_CLKEN;

	dev_vdbg(mvc->dev, "%s MVC enable %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_is_slcg_enable);

/**
 *  tegra210_mvc_is_config_err - check for any config error
 */
int tegra210_mvc_is_config_err(enum tegra210_ahub_cifs cif)
{
	u32 val = 0;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	regmap_read(mvc->regmap, TEGRA210_MVC_STATUS, &val);
	val &= TEGRA210_MVC_STATUS_CONFIG_ERROR;

	dev_vdbg(mvc->dev, "%s MVC config error %d", __func__, val);
	return val;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_is_config_err);

/**
 *  tegra210_mvc_config_err_type - type of config error
 */
int tegra210_mvc_config_err_type(enum tegra210_ahub_cifs cif)
{
	u32 val = 0;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	regmap_read(mvc->regmap, TEGRA210_MVC_CONFIG_ERR_TYPE, &val);

	dev_vdbg(mvc->dev, "%s MVC type of config error %d", __func__, val);
	return val;

}
EXPORT_SYMBOL_GPL(tegra210_mvc_config_err_type);

/**
 *  tegra210_mvc_set_acif_param - sets acif params of an mvc input/output
 *  @cif : mvc cif
 *  @acif : acif param values
 */
int tegra210_mvc_set_acif_param(enum tegra210_ahub_cifs cif,
		struct tegra210_axbar_cif_param *acif)
{
	u32 reg;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s MVC id %d cif %d", __func__, id, cif);

	reg = (IS_MVC_RX(cif) ? TEGRA210_MVC_RX_CIF_CTRL :
				TEGRA210_MVC_TX_CIF_CTRL);

	return tegra210_set_axbar_cif_param(mvc->dev,  reg , acif);

}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_acif_param);

/**
 *  tegra210_mvc_get_acif_param - gets acif params of an mvc input/output
 *  @cif : mvc cif
 *  @acif : acif ptr to be populated
 */
int tegra210_mvc_get_acif_param(enum tegra210_ahub_cifs cif,
		struct tegra210_axbar_cif_param *acif)
{
	u32 reg;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s MVC id %d cif %d", __func__, id, cif);

	reg = (IS_MVC_RX(cif) ? TEGRA210_MVC_RX_CIF_CTRL :
				TEGRA210_MVC_TX_CIF_CTRL);

	return tegra210_get_axbar_cif_param(mvc->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_mvc_get_acif_param);

/**
 *  tegra210_mvc_rx_set_peak_ctrl - sets peak type and window size on an input
 *  @type : peak type (WINDOW_BASED = 0, RESET_ON_READ = 1)
 *  @win_size : Window size (only applicable for WINDOW_BASED peak type)
 */
int tegra210_mvc_rx_set_peak_ctrl(enum tegra210_ahub_cifs cif, int type,
		unsigned int win_size)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s peak_type %d win_size %d",
				__func__, type, win_size);

	if (type == MVC_PEAK_WINDOW_BASED) {
		if (win_size > TEGRA210_MVC_MAX_PEAK_WIN_SIZE) {
			dev_err(mvc->dev, "%s Invalid win_size %d",
					__func__, win_size);
			return -EINVAL;
		}

		regmap_update_bits(mvc->regmap, TEGRA210_MVC_PEAK_CTRL,
		  TEGRA210_MVC_PEAK_CTRL_PEAK_WINDOW_SIZE_MASK,
		  win_size << TEGRA210_MVC_PEAK_CTRL_PEAK_WINDOW_SIZE_SHIFT);
	}

	if ((type == MVC_PEAK_WINDOW_BASED) || (type == MVC_PEAK_RESET_ON_READ))
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_PEAK_CTRL,
			TEGRA210_MVC_PEAK_CTRL_PEAK_TYPE_MASK,
			type << TEGRA210_MVC_PEAK_CTRL_PEAK_TYPE_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_rx_set_peak_ctrl);

/**
 * tegra210_mvc_rx_get_peak_values - gets peak values on input for all channels
 * @peak : ptr to be populated with peaks
 */
int tegra210_mvc_rx_get_peak_values(enum tegra210_ahub_cifs cif, u32 *peak)
{
	u32 reg = TEGRA210_MVC_PEAK_VALUE, i;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d  ", __func__, id);

	for (i = 0; i < TEGRA210_MVC_MAX_CHANNEL_COUNT; i++) {
		reg = reg + (i * 0x4);
		regmap_read(mvc->regmap, reg, peak);
		peak++;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_rx_get_peak_values);

/**
 *  tegra210_mvc_enable_bypass - bypass volume application on input
 */
int tegra210_mvc_enable_bypass(enum tegra210_ahub_cifs cif, bool en)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d bypass %d", __func__, id, en);

	regmap_update_bits(mvc->regmap, TEGRA210_MVC_CTRL,
			TEGRA210_MVC_CTRL_BYPASS_ENABLE,
			en ? TEGRA210_MVC_CTRL_BYPASS_ENABLE : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_enable_bypass);

/**
 * tegra210_mvc_enable_per_ch_ctrl - enable per channle volume control
 *
 */
int tegra210_mvc_enable_per_ch_ctrl(enum tegra210_ahub_cifs cif, bool en)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s per channel control %d", __func__, en);

	regmap_update_bits(mvc->regmap, TEGRA210_MVC_CTRL,
			TEGRA210_MVC_CTRL_PER_CH_CTRL_ENABLE,
			en ? TEGRA210_MVC_CTRL_PER_CH_CTRL_ENABLE : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_enable_per_ch_ctrl);

/**
 * tegra210_mvc_mute_unmute_ctrl - control the mute/unmute input of input stream
 *
 */
int tegra210_mvc_mute_unmute_ctrl(enum tegra210_ahub_cifs cif, u8 ctrlbits)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mute_unmute_control %d", __func__, ctrlbits);

	if (tegra210_mvc_is_switch_done(cif, VOLUME_SWITCH)) {
		dev_vdbg(mvc->dev, "%s mute_unmute_control reg update %d",
		__func__, ctrlbits);
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_CTRL,
			TEGRA210_MVC_CTRL_MUTE_UNMUTE_MASK,
			ctrlbits << TEGRA210_MVC_CTRL_MUTE_UNMUTE_SHIFT);

		return 0;
	}
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_mute_unmute_ctrl);

/**
 * tegra210_mvc_curve_type - set the volume curve type
 *
 */
int tegra210_mvc_curve_type(enum tegra210_ahub_cifs cif, bool en)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d curve type %d", __func__, id, en);

	regmap_update_bits(mvc->regmap, TEGRA210_MVC_CTRL,
			TEGRA210_MVC_CTRL_CURVE_TYPE_LINEAR,
			en ? TEGRA210_MVC_CTRL_CURVE_TYPE_LINEAR : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_curve_type);

/**
 * tegra210_mvc_rounding_type - set the rounding type
 *
 */
int tegra210_mvc_rounding_type(enum tegra210_ahub_cifs cif, bool en)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d rounding type %d", __func__, id, en);

	regmap_update_bits(mvc->regmap, TEGRA210_MVC_CTRL,
			TEGRA210_MVC_CTRL_ROUNDING_TYPE_UNBIASED,
			en ? TEGRA210_MVC_CTRL_ROUNDING_TYPE_UNBIASED : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_rounding_type);

/**
 * tegra210_mvc_set_switch - set the switch
 *
 */
int tegra210_mvc_set_switch(enum tegra210_ahub_cifs cif,
		enum mvc_switch_type switch_type, u8 switch_data)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d set switch %d", __func__, id,
		switch_type);
	if (switch_type & DURATION_SWITCH) {
		dev_vdbg(mvc->dev, "%s mvc id %d set dur_switch %d reg update",
		__func__, id, switch_type);
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_SWITCH,
			TEGRA210_MVC_SWITCH_DURATION_SET,
			(switch_data & DURATION_SWITCH) ?
				TEGRA210_MVC_SWITCH_DURATION_SET : 0);
	}
	if (switch_type & COEFF_SWITCH) {
		dev_vdbg(mvc->dev, "%s mvc id %d set coef_switch %d reg update",
		__func__, id, switch_type);
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_SWITCH,
			TEGRA210_MVC_SWITCH_COEF_SET,
			(switch_data & COEFF_SWITCH) ?
			TEGRA210_MVC_SWITCH_COEF_SET : 0);
	}
	if (switch_type & VOLUME_SWITCH) {
		dev_vdbg(mvc->dev, "%s mvc id %d set vol_switch %d reg update ",
		__func__, id, switch_type);
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_SWITCH,
			TEGRA210_MVC_SWITCH_VOLUME_SET,
			(switch_data & VOLUME_SWITCH) ?
			TEGRA210_MVC_SWITCH_VOLUME_SET : 0);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_switch);

/**
 * tegra210_mvc_is_switch_done - is sswitching done
 *
 */
bool tegra210_mvc_is_switch_done(enum tegra210_ahub_cifs cif,
		enum mvc_switch_type switch_type)
{
	u32 val, retvalue = 0;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	regmap_read(mvc->regmap, TEGRA210_MVC_SWITCH, &val);

	switch (switch_type) {
	case DURATION_SWITCH:
		retvalue = (val & 0x1);
		break;
	case COEFF_SWITCH:
		retvalue = ((val>>1) & 0x1);
		break;
	case VOLUME_SWITCH:
		retvalue = ((val>>2) & 0x1);
		break;
	default:
			break;
	}
	dev_vdbg(mvc->dev, "%s switch_type %d switch-done=0x%x ", __func__,
				switch_type, (retvalue ? false : true));
	return retvalue ? false : true;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_is_switch_done);

/**
 * tegra210_mvc_set_init_vol - set the init vol level
 *
 */
int tegra210_mvc_set_init_vol(enum tegra210_ahub_cifs cif, int *init_vol)
{
	u32 reg = TEGRA210_MVC_INIT_VOL_CH0, i;
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mv id %d ", __func__, id);

	if (tegra210_mvc_is_switch_done(cif, VOLUME_SWITCH)) {
		for (i = 0; i < TEGRA210_MVC_MAX_CHANNEL_COUNT; i++) {
			regmap_write(mvc->regmap, reg, *init_vol);
			init_vol++;
			reg = reg + 0x4;
		}
	return 0;
	}

	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_init_vol);

/**
 * tegra210_mvc_set_target_vol - set the target vol level
 *
 */
int tegra210_mvc_set_target_vol(enum tegra210_ahub_cifs cif, int *target_vol)
{
	u32 reg = TEGRA210_MVC_TARGET_VOL_CH0, i;
	int id = MVC_ID(cif);
	int *t_vol = target_vol;
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d", __func__, id);

	if (tegra210_mvc_is_switch_done(cif, VOLUME_SWITCH)) {
		for (i = 0; i < TEGRA210_MVC_MAX_CHANNEL_COUNT; i++) {
			regmap_write(mvc->regmap, reg, t_vol[i]);
			target_vol++;
			reg = reg + 0x4;
		}
		return 0;
	}
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_target_vol);


/**
 * tegra210_mvc_set_duration - set the duration in number of samples
 *
 */
int tegra210_mvc_set_duration(enum tegra210_ahub_cifs cif,
		u32 number_of_samples)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d", __func__, id);

	if (tegra210_mvc_is_switch_done(cif, DURATION_SWITCH)) {
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_DURATION,
			TEGRA210_MVC_DURATION_MASK,
			number_of_samples << TEGRA210_MVC_DURATION_SHIFT);
	}
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_duration);

/**
 * tegra210_mvc_set_inv_duration - set the inverse duration in number of samples
 *
 */
int tegra210_mvc_set_inv_duration(enum tegra210_ahub_cifs cif,
		u32 number_of_samples)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d", __func__, id);
	if (tegra210_mvc_is_switch_done(cif, DURATION_SWITCH)) {
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_DURATION_INV,
			TEGRA210_MVC_DURATION_INV_MASK,
			number_of_samples << TEGRA210_MVC_DURATION_INV_SHIFT);
	}
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_inv_duration);

/**
 * tegra210_mvc_set_poly_curve_split_points - set the N1, N2 split points
 *
 */
int tegra210_mvc_set_poly_curve_split_points(enum tegra210_ahub_cifs cif,
		u32 split_points[2])
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d, split_points: 0x%x 0x%0x",
		__func__, id,	split_points[0], split_points[1]);

	if (tegra210_mvc_is_switch_done(cif, DURATION_SWITCH)) {
		regmap_update_bits(mvc->regmap, TEGRA210_MVC_POLY_N1,
			TEGRA210_MVC_POLY_N1_MASK,
			split_points[0] << TEGRA210_MVC_POLY_N1_SHIFT);

		regmap_update_bits(mvc->regmap, TEGRA210_MVC_POLY_N2,
			TEGRA210_MVC_POLY_N2_MASK,
			split_points[1] << TEGRA210_MVC_POLY_N2_SHIFT);
	}

	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_poly_curve_split_points);

/**
 * tegra210_mvc_set_poly_curve_coefs - set the curve coefs
 *
 */
int tegra210_mvc_set_poly_curve_coefs(enum tegra210_ahub_cifs cif, u32 *coef)
{
	int id = MVC_ID(cif);
	struct tegra210_mvc_ctx *mvc = tegra210_mvc[id];

	dev_vdbg(mvc->dev, "%s mvc id %d ", __func__, id);

	if (tegra210_mvc_is_switch_done(cif, COEFF_SWITCH)) {
		tegra210_ahubram_write(mvc->dev,
				TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL,
				TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA,
				0,
				coef, TEGRA210_MVC_NUMBER_OF_COEFS);

		return 0;
	}

	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_mvc_set_poly_curve_coefs);


/* Regmap callback functions */
static bool tegra210_mvc_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_MVC_RX_INT_MASK:
	case TEGRA210_MVC_RX_INT_SET:
	case TEGRA210_MVC_RX_INT_CLEAR:
	case TEGRA210_MVC_RX_CIF_CTRL:

	case TEGRA210_MVC_TX_INT_MASK:
	case TEGRA210_MVC_TX_INT_SET:
	case TEGRA210_MVC_TX_INT_CLEAR:
	case TEGRA210_MVC_TX_CIF_CTRL:

	case TEGRA210_MVC_ENABLE:
	case TEGRA210_MVC_SOFT_RESET:
	case TEGRA210_MVC_CG:
	case TEGRA210_MVC_CTRL:
	case TEGRA210_MVC_SWITCH:
	case TEGRA210_MVC_INIT_VOL_CH0:
	case TEGRA210_MVC_INIT_VOL_CH1:
	case TEGRA210_MVC_INIT_VOL_CH2:
	case TEGRA210_MVC_INIT_VOL_CH3:
	case TEGRA210_MVC_INIT_VOL_CH4:
	case TEGRA210_MVC_INIT_VOL_CH5:
	case TEGRA210_MVC_INIT_VOL_CH6:
	case TEGRA210_MVC_INIT_VOL_CH7:
	case TEGRA210_MVC_TARGET_VOL_CH0:
	case TEGRA210_MVC_TARGET_VOL_CH1:
	case TEGRA210_MVC_TARGET_VOL_CH2:
	case TEGRA210_MVC_TARGET_VOL_CH3:
	case TEGRA210_MVC_TARGET_VOL_CH4:
	case TEGRA210_MVC_TARGET_VOL_CH5:
	case TEGRA210_MVC_TARGET_VOL_CH6:
	case TEGRA210_MVC_TARGET_VOL_CH7:
	case TEGRA210_MVC_DURATION:
	case TEGRA210_MVC_DURATION_INV:
	case TEGRA210_MVC_POLY_N1:
	case TEGRA210_MVC_POLY_N2:
	case TEGRA210_MVC_PEAK_CTRL:
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL:
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_mvc_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_MVC_RX_STATUS:
	case TEGRA210_MVC_RX_INT_STATUS:
	case TEGRA210_MVC_RX_INT_MASK:
	case TEGRA210_MVC_RX_INT_SET:
	case TEGRA210_MVC_RX_INT_CLEAR:
	case TEGRA210_MVC_RX_CIF_CTRL:

	case TEGRA210_MVC_TX_STATUS:
	case TEGRA210_MVC_TX_INT_STATUS:
	case TEGRA210_MVC_TX_INT_MASK:
	case TEGRA210_MVC_TX_INT_SET:
	case TEGRA210_MVC_TX_INT_CLEAR:
	case TEGRA210_MVC_TX_CIF_CTRL:

	case TEGRA210_MVC_ENABLE:
	case TEGRA210_MVC_SOFT_RESET:
	case TEGRA210_MVC_CG:
	case TEGRA210_MVC_STATUS:
	case TEGRA210_MVC_INT_STATUS:
	case TEGRA210_MVC_CTRL:
	case TEGRA210_MVC_SWITCH:
	case TEGRA210_MVC_INIT_VOL_CH0:
	case TEGRA210_MVC_INIT_VOL_CH1:
	case TEGRA210_MVC_INIT_VOL_CH2:
	case TEGRA210_MVC_INIT_VOL_CH3:
	case TEGRA210_MVC_INIT_VOL_CH4:
	case TEGRA210_MVC_INIT_VOL_CH5:
	case TEGRA210_MVC_INIT_VOL_CH6:
	case TEGRA210_MVC_INIT_VOL_CH7:
	case TEGRA210_MVC_TARGET_VOL_CH0:
	case TEGRA210_MVC_TARGET_VOL_CH1:
	case TEGRA210_MVC_TARGET_VOL_CH2:
	case TEGRA210_MVC_TARGET_VOL_CH3:
	case TEGRA210_MVC_TARGET_VOL_CH4:
	case TEGRA210_MVC_TARGET_VOL_CH5:
	case TEGRA210_MVC_TARGET_VOL_CH6:
	case TEGRA210_MVC_TARGET_VOL_CH7:
	case TEGRA210_MVC_DURATION:
	case TEGRA210_MVC_DURATION_INV:
	case TEGRA210_MVC_POLY_N1:
	case TEGRA210_MVC_POLY_N2:
	case TEGRA210_MVC_PEAK_CTRL:
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL:
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA:
	case TEGRA210_MVC_PEAK_VALUE:
	case TEGRA210_MVC_CONFIG_ERR_TYPE:
		return true;
	default:
		return false;
	};
}

static bool tegra210_mvc_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_MVC_RX_STATUS:
	case TEGRA210_MVC_RX_INT_STATUS:
	case TEGRA210_MVC_RX_INT_SET:

	case TEGRA210_MVC_TX_STATUS:
	case TEGRA210_MVC_TX_INT_STATUS:
	case TEGRA210_MVC_TX_INT_SET:

	case TEGRA210_MVC_SOFT_RESET:
	case TEGRA210_MVC_STATUS:
	case TEGRA210_MVC_INT_STATUS:
	case TEGRA210_MVC_SWITCH:
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL:
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA:
	case TEGRA210_MVC_PEAK_VALUE:
		return true;
	default:
		return false;
	};
}

static bool tegra210_mvc_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_mvc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_MVC_CONFIG_ERR_TYPE,
	.writeable_reg = tegra210_mvc_wr_reg,
	.readable_reg = tegra210_mvc_rd_reg,
	.volatile_reg = tegra210_mvc_volatile_reg,
	.precious_reg = tegra210_mvc_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_mvc_runtime_suspend(struct device *dev)
{
	struct tegra210_mvc_ctx *mvc = dev_get_drvdata(dev);

	dev_dbg(mvc->dev, "%s mvc", __func__);
	regcache_cache_only(mvc->regmap, true);
	tegra210_axbar_disable_clk();

	return 0;
}

static int tegra210_mvc_runtime_resume(struct device *dev)
{
	struct tegra210_mvc_ctx *mvc = dev_get_drvdata(dev);

	dev_dbg(mvc->dev, "%s mvc", __func__);
	tegra210_axbar_enable_clk();
	regcache_cache_only(mvc->regmap, false);

	return 0;
}

/*
MVC Driver probe and remove functions
*/
static int tegra210_mvc_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_mvc_ctx *mvc;
	int ret = 0;
	int id = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s fail, no DT node for MVC", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-mvc-id",
						 &pdev->id);

	dev_dbg(&pdev->dev, "%s mvc : starting probe", __func__);
	id = pdev->id;

	tegra210_mvc[id] = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_mvc_ctx), GFP_KERNEL);
	if (!tegra210_mvc[id]) {
		dev_err(&pdev->dev, "Can't allocate mvc context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_mvc[id]->dev = &pdev->dev;

	mvc = tegra210_mvc[id];
	mvc->id = id;
	dev_set_drvdata(&pdev->dev, mvc);

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

	mvc->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mvc->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	mvc->regmap = devm_regmap_init_mmio(&pdev->dev, mvc->regs,
					    &tegra210_mvc_regmap_config);
	if (IS_ERR(mvc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(mvc->regmap);
		goto err_free;
	}
	regcache_cache_only(mvc->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_mvc_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(mvc->dev, "%s mvc probe successful", __func__);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_mvc[pdev->id] = NULL;
exit:
	dev_dbg(&pdev->dev, "%s mvc %d probe failed with %d", __func__,
				pdev->id, ret);
	return ret;
}

static int tegra210_mvc_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_mvc_runtime_suspend(&pdev->dev);

	tegra210_mvc[pdev->id] = NULL;

	dev_dbg(&pdev->dev, "%s MVC %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_mvc_of_match[] = {
	{ .compatible = "nvidia,tegra210-mvc",},
	{},
};

static const struct dev_pm_ops tegra210_mvc_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_mvc_runtime_suspend,
			   tegra210_mvc_runtime_resume, NULL)
};

static struct platform_driver tegra210_mvc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_mvc_of_match,
		.pm = &tegra210_mvc_pm_ops,
	},
	.probe = tegra210_mvc_probe,
	.remove = tegra210_mvc_remove,
};

module_platform_driver(tegra210_mvc_driver);

MODULE_AUTHOR("Ashok Mudithanapalli  <ashokm@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 MVC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
