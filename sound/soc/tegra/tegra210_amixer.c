/*
 * tegra210_amixer.c - Tegra210 AMIXER driver.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
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

#include "tegra210_amixer.h"
#include "tegra210_axbar.h"
#include "tegra210_ahub_utils.h"

#define DRV_NAME "tegra210-amixer"

/* TEGRA210 AMIXER driver context */
struct tegra210_amixer_ctx {
	struct device	*dev;
	void __iomem	*regs;
	struct regmap	*regmap;
	int		in_use;
};

static struct tegra210_amixer_ctx *tegra210_amixer;

/* AMIXER APIs definition */
/**
 *  tegra210_amixer_get - gets amixer before use
 */
int tegra210_amixer_get()
{
	if (!tegra210_amixer->in_use) {
		dev_vdbg(tegra210_amixer->dev, "%s", __func__);
		pm_runtime_get_sync(tegra210_amixer->dev);
	}

	tegra210_amixer->in_use++;
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_get);

/**
 *  tegra210_amixer_put - frees amixer after use
 */
int tegra210_amixer_put()
{
	tegra210_amixer->in_use--;
	if (!tegra210_amixer->in_use) {
		dev_vdbg(tegra210_amixer->dev, "%s", __func__);
		pm_runtime_put_sync(tegra210_amixer->dev);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_put);

/**
 *  tegra210_amixer_enable - enables/disables amixer globally
 *  @en : enable/disable flag
 */
int tegra210_amixer_enable(bool en)
{
	dev_vdbg(tegra210_amixer->dev, "%s enable %d", __func__, en);

	regmap_update_bits(tegra210_amixer->regmap,
			TEGRA210_AMIXER_ENABLE,
			TEGRA210_AMIXER_ENABLE_EN,
			en ? TEGRA210_AMIXER_ENABLE_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_enable);

/**
 *  tegra210_amixer_reset - soft resets amixer globally
 */
int tegra210_amixer_reset()
{
	u32 val = 0;
	int cnt = AHUB_OP_MAX_RETRY;

	dev_vdbg(tegra210_amixer->dev, "%s reset amixer", __func__);
	regmap_update_bits(tegra210_amixer->regmap,
			TEGRA210_AMIXER_SOFT_RESET,
			TEGRA210_AMIXER_SOFT_RESET_EN,
			TEGRA210_AMIXER_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(tegra210_amixer->regmap, TEGRA210_AMIXER_SOFT_RESET,
				&val);
	} while ((val & TEGRA210_AMIXER_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(tegra210_amixer->dev, "%s Failed to reset amixer",
				__func__);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_reset);

/**
 *  tegra210_amixer_set_slcg - sets second level clock gating of amixer
 *  @en : enable/disable flag
 */
int tegra210_amixer_set_slcg(bool en)
{
	dev_vdbg(tegra210_amixer->dev, "%s amixer slcg enable %d",
			__func__, en);

	regmap_update_bits(tegra210_amixer->regmap,
			TEGRA210_AMIXER_CG,
			TEGRA210_AMIXER_CG_SLCG_EN,
			en ? TEGRA210_AMIXER_CG_SLCG_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_set_slcg);

/**
 *  tegra210_amixer_enable_bypass_mode - sets amixer global bypass mode
 *  @en : enable/disable flag
 */
int tegra210_amixer_enable_bypass_mode(bool en)
{
	dev_vdbg(tegra210_amixer->dev, "%s bypass mode enable %d",
			__func__, en);

	regmap_update_bits(tegra210_amixer->regmap,
			TEGRA210_AMIXER_CTRL,
			TEGRA210_AMIXER_CTRL_BYPASS_MODE_EN,
			en ? TEGRA210_AMIXER_CTRL_BYPASS_MODE_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_enable_bypass_mode);

/**
 *  tegra210_amixer_set_acif_param - sets acif params of an amixer input/output
 *  @cif : mixer cif
 *  @acif : acif param values
 */
int tegra210_amixer_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	u32 reg;
	dev_vdbg(tegra210_amixer->dev, "%s amixer cif %d", __func__, cif);

	reg = IS_MIXER_RX(cif) ?
		TEGRA210_AMIXER_RX_CIF_CTRL +
			(MIXER_CIF_ID(cif) * TEGRA210_AMIXER_RX_STRIDE) :
		TEGRA210_AMIXER_TX_CIF_CTRL +
			(MIXER_CIF_ID(cif) * TEGRA210_AMIXER_TX_STRIDE);

	return tegra210_set_axbar_cif_param(tegra210_amixer->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_amixer_set_acif_param);

/**
 *  tegra210_amixer_get_acif_param - gets acif params of an amixer input/output
 *  @cif : mixer cif
 *  @acif : acif ptr to be populated
 */
int tegra210_amixer_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	u32 reg;
	dev_vdbg(tegra210_amixer->dev, "%s amixer cif %d", __func__, cif);

	reg = IS_MIXER_RX(cif) ?
		TEGRA210_AMIXER_RX_CIF_CTRL +
			(MIXER_CIF_ID(cif) * TEGRA210_AMIXER_RX_STRIDE) :
		TEGRA210_AMIXER_TX_CIF_CTRL +
			(MIXER_CIF_ID(cif) * TEGRA210_AMIXER_TX_STRIDE);

	return tegra210_get_axbar_cif_param(tegra210_amixer->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_amixer_get_acif_param);

/**
 *  tegra210_amixer_rx_reset - soft resets an input
 *  @cif : mixer rxcif
 */
int tegra210_amixer_rx_reset(enum tegra210_ahub_cifs cif)
{
	u32 val = 0, reg, cif_id = MIXER_CIF_ID(cif);
	int cnt = AHUB_OP_MAX_RETRY;

	dev_vdbg(tegra210_amixer->dev, "%s reset rx %d", __func__, cif_id);
	reg = TEGRA210_AMIXER_RX_SOFT_RESET +
		(cif_id * TEGRA210_AMIXER_RX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_RX_SOFT_RESET_EN,
			TEGRA210_AMIXER_RX_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(tegra210_amixer->regmap, reg, &val);
	} while ((val & TEGRA210_AMIXER_RX_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(tegra210_amixer->dev, "%s Failed to reset rx %d",
					__func__, cif_id);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_reset);

/**
 *  tegra210_amixer_rx_set_act_threshold - sets activation threshold of an input
 *  @cif : mixer rxcif
 *  @act_th : activation threshold (in terms of number of frames)
 */
int tegra210_amixer_rx_set_act_threshold(enum tegra210_ahub_cifs cif,
						int act_th)
{
	u32 reg, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d act_threshold %d", __func__,
			cif_id, act_th);

	reg = TEGRA210_AMIXER_RX_CTRL + (cif_id * TEGRA210_AMIXER_RX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
		TEGRA210_AMIXER_RX_CTRL_ACT_THRESHOLD_MASK,
		act_th << TEGRA210_AMIXER_RX_CTRL_ACT_THRESHOLD_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_set_act_threshold);

/**
 *  tegra210_amixer_rx_set_deact_threshold - sets deactivation threshold of an input
 *  @cif : mixer rxcif
 *  @deact_th : deactivation threshold (in terms of mixer clock cycles)
 */
int tegra210_amixer_rx_set_deact_threshold(enum tegra210_ahub_cifs cif,
						int deact_th)
{
	u32 reg, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d deact_threshold %d", __func__,
			cif_id, deact_th);

	reg = TEGRA210_AMIXER_RX_CTRL + (cif_id * TEGRA210_AMIXER_RX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
		TEGRA210_AMIXER_RX_CTRL_DEACT_THRESHOLD_MASK,
		deact_th << TEGRA210_AMIXER_RX_CTRL_DEACT_THRESHOLD_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_set_deact_threshold);

/**
 *  tegra210_amixer_rx_reset_sample_counter - resets sample count of an input
 *  @cif : mixer rxcif
 */
int tegra210_amixer_rx_reset_sample_counter(enum tegra210_ahub_cifs cif)
{
	u32 reg, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d", __func__, cif_id);

	reg = TEGRA210_AMIXER_RX_CTRL + (cif_id * TEGRA210_AMIXER_RX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_RX_CTRL_SAMPLE_COUNTER_RESET_EN,
			TEGRA210_AMIXER_RX_CTRL_SAMPLE_COUNTER_RESET_EN);

	udelay(AHUB_OP_SLEEP_US);
	regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_RX_CTRL_SAMPLE_COUNTER_RESET_EN, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_reset_sample_counter);

/**
 *  tegra210_amixer_rx_bypass_gain - bypasses gain application on an input
 *  @cif : mixer rxcif
 *  @en : enable/disable flag
 */
int tegra210_amixer_rx_bypass_gain(enum tegra210_ahub_cifs cif, bool en)
{
	u32 reg, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d bypass %d", __func__,
			cif_id, en);

	reg = TEGRA210_AMIXER_RX_CTRL + (cif_id * TEGRA210_AMIXER_RX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_RX_CTRL_GAIN_BYPASS_EN,
			en ? TEGRA210_AMIXER_RX_CTRL_GAIN_BYPASS_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_bypass_gain);

/**
 *  tegra210_amixer_rx_set_gain_coef - sets gain ram coefficients for an input
 *  @cif : mixer rxcif
 *  @coef : ptr to coefficients
 */
int tegra210_amixer_rx_set_gain_coef(enum tegra210_ahub_cifs cif, u32 *coef)
{
	u32 cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d", __func__, cif_id);

	tegra210_ahubram_write(tegra210_amixer->dev,
			TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_CTRL,
			TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_DATA,
			cif_id * TEGRA210_AMIXER_GAIN_COEF_DEPTH,
			coef, TEGRA210_AMIXER_GAIN_COEF_DEPTH);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_set_gain_coef);

/**
 *  tegra210_amixer_rx_set_peak_ctrl - sets peak type and window size on an input
 *  @cif : mixer rxcif
 *  @type : peak type (WINDOW_BASED = 0, RESET_ON_READ = 1)
 *  @win_size : Window size (only applicable for WINDOW_BASED peak type)
 */
int tegra210_amixer_rx_set_peak_ctrl(enum tegra210_ahub_cifs cif, int type,
					unsigned int win_size)
{
	u32 reg, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d peak_type %d win_size %d",
				__func__, cif_id, type, win_size);

	reg = TEGRA210_AMIXER_RX_PEAK_CTRL +
		(cif_id * TEGRA210_AMIXER_RX_STRIDE);

	if (type == PEAK_WINDOW_BASED) {
		if (win_size > TEGRA210_AMIXER_MAX_PEAK_WIN_SIZE) {
			dev_err(tegra210_amixer->dev, "%s Invalid win_size %d",
					__func__, win_size);
			return -EINVAL;
		}

		regmap_update_bits(tegra210_amixer->regmap, reg,
		  TEGRA210_AMIXER_RX_PEAK_CTRL_WIN_SIZE_MASK,
		  win_size << TEGRA210_AMIXER_RX_PEAK_CTRL_WIN_SIZE_SHIFT);
	}

	if ((type == PEAK_WINDOW_BASED) || (type == PEAK_RESET_ON_READ))
		regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_RX_PEAK_CTRL_PEAK_TYPE_MASK,
			type << TEGRA210_AMIXER_RX_PEAK_CTRL_PEAK_TYPE_SHIFT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_set_peak_ctrl);

/**
 *  tegra210_amixer_rx_get_peak_values - gets peak values on an input for all channels
 *  @cif : mixer rxcif
 *  @peak : ptr to be populated with peaks
 */
int tegra210_amixer_rx_get_peak_values(enum tegra210_ahub_cifs cif, u32 *peak)
{
	u32 cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d", __func__, cif_id);

	tegra210_ahubram_read(tegra210_amixer->dev,
			TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_CTRL,
			TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_DATA,
			cif_id * TEGRA210_AMIXER_MAX_CHANNEL_COUNT,
			peak, TEGRA210_AMIXER_MAX_CHANNEL_COUNT);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_get_peak_values);

/**
 *  tegra210_amixer_rx_get_sample_count - gets sample count on an input
 *  @cif : mixer rxcif
 */
u32 tegra210_amixer_rx_get_sample_count(enum tegra210_ahub_cifs cif)
{
	u32 reg, sample_cnt = 0, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s rx %d", __func__, cif_id);

	reg = TEGRA210_AMIXER_RX_SAMPLE_COUNT +
		(cif_id * TEGRA210_AMIXER_RX_STRIDE);

	regmap_read(tegra210_amixer->regmap, reg, &sample_cnt);
	return sample_cnt;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_rx_get_sample_count);

/**
 *  tegra210_amixer_tx_enable - enables/disables an output
 *  @cif : mixer txcif
 *  @en : enable/disable flag
 */
int tegra210_amixer_tx_enable(enum tegra210_ahub_cifs cif, bool en)
{
	u32 reg, cif_id = MIXER_CIF_ID(cif);
	dev_vdbg(tegra210_amixer->dev, "%s tx %d enable %d", __func__,
			cif_id, en);

	reg = TEGRA210_AMIXER_TX_ENABLE + (cif_id * TEGRA210_AMIXER_TX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_TX_ENABLE_EN,
			en ? TEGRA210_AMIXER_TX_ENABLE_EN : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_tx_enable);

/**
 *  tegra210_amixer_tx_reset - soft resets an output
 *  @cif : mixer txcif
 */
int tegra210_amixer_tx_reset(enum tegra210_ahub_cifs cif)
{
	u32 val = 0, reg, cif_id = MIXER_CIF_ID(cif);
	int cnt = AHUB_OP_MAX_RETRY;

	dev_vdbg(tegra210_amixer->dev, "%s reset tx %d", __func__, cif_id);
	reg = TEGRA210_AMIXER_TX_SOFT_RESET +
		(cif_id * TEGRA210_AMIXER_TX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
			TEGRA210_AMIXER_TX_SOFT_RESET_EN,
			TEGRA210_AMIXER_TX_SOFT_RESET_EN);

	do {
		udelay(AHUB_OP_SLEEP_US);
		regmap_read(tegra210_amixer->regmap, reg, &val);
	} while ((val & TEGRA210_AMIXER_TX_SOFT_RESET_EN) && cnt--);

	if (!cnt) {
		dev_err(tegra210_amixer->dev, "%s Failed to reset tx %d",
					__func__, cif_id);
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_tx_reset);

/**
 *  tegra210_amixer_tx_enable_input - enables/disables input to be mixed on an output
 *  @txcif : mixer txcif
 *  @rxcif : mixer rxcif
 *  @en : enable/disable flag
 */
int tegra210_amixer_tx_enable_input(enum tegra210_ahub_cifs txcif,
					enum tegra210_ahub_cifs rxcif, bool en)
{
	u32 reg, txcif_id = MIXER_CIF_ID(txcif), rxcif_id = MIXER_CIF_ID(rxcif);
	dev_vdbg(tegra210_amixer->dev, "%s tx %d : rx %d enable %d",
				__func__, txcif_id, rxcif_id, en);

	reg = TEGRA210_AMIXER_TX_ADDER_CONFIG +
		(txcif_id * TEGRA210_AMIXER_TX_STRIDE);

	regmap_update_bits(tegra210_amixer->regmap, reg,
	  TEGRA210_AMIXER_TX_ADDER_CONFIG_INPUT_ENABLE(rxcif_id),
	  en ? TEGRA210_AMIXER_TX_ADDER_CONFIG_INPUT_ENABLE(rxcif_id) : 0);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_amixer_tx_enable_input);

/* Regmap callback functions */
static bool tegra210_amixer_wr_reg(struct device *dev, unsigned int reg)
{
	if (reg < TEGRA210_AMIXER_RX_LIMIT)
		reg %= TEGRA210_AMIXER_RX_STRIDE;
	else if (reg < TEGRA210_AMIXER_TX_LIMIT)
		reg = (reg % TEGRA210_AMIXER_TX_STRIDE) +
			TEGRA210_AMIXER_TX_ENABLE;

	switch (reg) {
	case TEGRA210_AMIXER_RX_SOFT_RESET:
	case TEGRA210_AMIXER_RX_CIF_CTRL:
	case TEGRA210_AMIXER_RX_CTRL:
	case TEGRA210_AMIXER_RX_PEAK_CTRL:

	case TEGRA210_AMIXER_TX_ENABLE:
	case TEGRA210_AMIXER_TX_SOFT_RESET:
	case TEGRA210_AMIXER_TX_INT_MASK:
	case TEGRA210_AMIXER_TX_INT_SET:
	case TEGRA210_AMIXER_TX_INT_CLEAR:
	case TEGRA210_AMIXER_TX_CIF_CTRL:
	case TEGRA210_AMIXER_TX_ADDER_CONFIG:

	case TEGRA210_AMIXER_ENABLE:
	case TEGRA210_AMIXER_SOFT_RESET:
	case TEGRA210_AMIXER_CG:
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_CTRL:
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_DATA:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_CTRL:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_DATA:
	case TEGRA210_AMIXER_CTRL:
		return true;
	default:
		return false;
	};
}

static bool tegra210_amixer_rd_reg(struct device *dev, unsigned int reg)
{
	if (reg < TEGRA210_AMIXER_RX_LIMIT)
		reg %= TEGRA210_AMIXER_RX_STRIDE;
	else if (reg < TEGRA210_AMIXER_TX_LIMIT)
		reg = (reg % TEGRA210_AMIXER_TX_STRIDE) +
			TEGRA210_AMIXER_TX_ENABLE;

	switch (reg) {
	case TEGRA210_AMIXER_RX_SOFT_RESET:
	case TEGRA210_AMIXER_RX_STATUS:
	case TEGRA210_AMIXER_RX_CIF_CTRL:
	case TEGRA210_AMIXER_RX_CTRL:
	case TEGRA210_AMIXER_RX_PEAK_CTRL:
	case TEGRA210_AMIXER_RX_SAMPLE_COUNT:

	case TEGRA210_AMIXER_TX_ENABLE:
	case TEGRA210_AMIXER_TX_SOFT_RESET:
	case TEGRA210_AMIXER_TX_STATUS:
	case TEGRA210_AMIXER_TX_INT_STATUS:
	case TEGRA210_AMIXER_TX_INT_MASK:
	case TEGRA210_AMIXER_TX_INT_SET:
	case TEGRA210_AMIXER_TX_INT_CLEAR:
	case TEGRA210_AMIXER_TX_CIF_CTRL:
	case TEGRA210_AMIXER_TX_ADDER_CONFIG:

	case TEGRA210_AMIXER_ENABLE:
	case TEGRA210_AMIXER_SOFT_RESET:
	case TEGRA210_AMIXER_CG:
	case TEGRA210_AMIXER_STATUS:
	case TEGRA210_AMIXER_INT_STATUS:
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_CTRL:
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_DATA:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_CTRL:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_DATA:
	case TEGRA210_AMIXER_CTRL:
		return true;
	default:
		return false;
	};
}

static bool tegra210_amixer_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg < TEGRA210_AMIXER_RX_LIMIT)
		reg %= TEGRA210_AMIXER_RX_STRIDE;
	else if (reg < TEGRA210_AMIXER_TX_LIMIT)
		reg = (reg % TEGRA210_AMIXER_TX_STRIDE) +
			TEGRA210_AMIXER_TX_ENABLE;

	switch (reg) {
	case TEGRA210_AMIXER_RX_SOFT_RESET:
	case TEGRA210_AMIXER_RX_STATUS:

	case TEGRA210_AMIXER_TX_SOFT_RESET:
	case TEGRA210_AMIXER_TX_STATUS:
	case TEGRA210_AMIXER_TX_INT_STATUS:
	case TEGRA210_AMIXER_TX_INT_SET:

	case TEGRA210_AMIXER_SOFT_RESET:
	case TEGRA210_AMIXER_STATUS:
	case TEGRA210_AMIXER_INT_STATUS:
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_CTRL:
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_DATA:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_CTRL:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_amixer_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_DATA:
	case TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_amixer_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_AMIXER_CTRL,
	.writeable_reg = tegra210_amixer_wr_reg,
	.readable_reg = tegra210_amixer_rd_reg,
	.volatile_reg = tegra210_amixer_volatile_reg,
	.precious_reg = tegra210_amixer_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* PM runtime callback functions */
static int tegra210_amixer_runtime_suspend(struct device *dev)
{
	struct tegra210_amixer_ctx *amixer = dev_get_drvdata(dev);

	dev_dbg(amixer->dev, "%s amixer", __func__);
	tegra210_axbar_disable_clk();
	regcache_cache_only(amixer->regmap, true);

	return 0;
}

static int tegra210_amixer_runtime_resume(struct device *dev)
{
	struct tegra210_amixer_ctx *amixer = dev_get_drvdata(dev);

	dev_dbg(amixer->dev, "%s amixer", __func__);
	tegra210_axbar_enable_clk();
	regcache_cache_only(amixer->regmap, false);

	return 0;
}

/* AMIXER driver probe and remove functions */
static int tegra210_amixer_probe(struct platform_device *pdev)
{
	struct resource *res, *region;
	struct tegra210_amixer_ctx *amixer;
	int ret = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s fail, no DT node for AMIXER", __func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-amixer-id",
						 &pdev->id);

	dev_dbg(&pdev->dev, "%s amixer : starting probe", __func__);

	tegra210_amixer = devm_kzalloc(&pdev->dev,
			   sizeof(struct tegra210_amixer_ctx), GFP_KERNEL);
	if (!tegra210_amixer) {
		dev_err(&pdev->dev, "Can't allocate amixer context\n");
		ret = -ENOMEM;
		goto exit;
	}
	tegra210_amixer->dev = &pdev->dev;

	amixer = tegra210_amixer;
	dev_set_drvdata(&pdev->dev, amixer);

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

	amixer->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!amixer->regs) {
		dev_err(&pdev->dev, "ioremap 0 failed\n");
		ret = -ENOMEM;
		goto err_free;
	}

	amixer->regmap = devm_regmap_init_mmio(&pdev->dev, amixer->regs,
					    &tegra210_amixer_regmap_config);
	if (IS_ERR(amixer->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(amixer->regmap);
		goto err_free;
	}
	regcache_cache_only(amixer->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_amixer_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	dev_dbg(amixer->dev, "%s amixer probe successful", __func__);

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_free:
	tegra210_amixer = NULL;
exit:
	dev_dbg(&pdev->dev, "%s amixer probe failed with %d", __func__, ret);
	return ret;
}

static int tegra210_amixer_remove(struct platform_device *pdev)
{
	struct tegra210_amixer_ctx *amixer;

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_amixer_runtime_suspend(&pdev->dev);

	amixer = platform_get_drvdata(pdev);
	tegra210_amixer = NULL;

	dev_dbg(&pdev->dev, "%s amixer %d removed",
		__func__, pdev->id);

	return 0;
}

static const struct of_device_id tegra210_amixer_of_match[] = {
	{ .compatible = "nvidia,tegra210-amixer",},
	{},
};

static const struct dev_pm_ops tegra210_amixer_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_amixer_runtime_suspend,
			   tegra210_amixer_runtime_resume, NULL)
};

static struct platform_driver tegra210_amixer_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_amixer_of_match,
		.pm = &tegra210_amixer_pm_ops,
	},
	.probe = tegra210_amixer_probe,
	.remove = tegra210_amixer_remove,
};
module_platform_driver(tegra210_amixer_driver);

MODULE_AUTHOR("Rahul Mittal <rmittal@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 AMIXER driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
