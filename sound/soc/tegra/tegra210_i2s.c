/*
 * tegra210_i2s.c - Tegra210 I2S driver
 *
 * Author: Dara Ramesh <dramesh@nvidia.com>
 *
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
#define VERBOSE_DEBUG 1
#define DEBUG 1
*/
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

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra210_i2s.h"
#include "tegra210_axbar.h"
#include "tegra210_ahub_utils.h"

#define DRV_NAME "tegra210-i2s"

struct tegra210_i2s_ctx {
	/* struct snd_soc_dai_driver dai; */
	struct device *dev;
	struct regmap *regmap;

	struct clk *clk_i2s;
	struct clk *clk_i2s_sync;
	struct clk *clk_audio_2x;
	struct clk *clk_pll_a_out0;
	int id;
	int daifmt;
	bool in_use;
	int playback_ref_count;
	int capture_ref_count;
	int is_call_mode_rec;
	int i2s_bit_clk;
};

struct tegra210_i2s_ctx *tegra210_i2s[TEGRA210_I2S_COUNT];

/* i2s pm runtime sync resume. */
int tegra210_i2s_get(enum tegra210_ahub_cifs cif)
{
	int id = I2S_ID(cif);
	struct tegra210_i2s_ctx *i2s = tegra210_i2s[id];

	if (id >= TEGRA210_I2S_COUNT) {
		pr_err("%s Invalid I2S id %d", __func__, id);
		return -EINVAL;
	}

	if (IS_I2S(cif)) {
		i2s = tegra210_i2s[id];
		if (i2s->in_use) {
			dev_warn(i2s->dev, "%s I2S %d in use", __func__, id);
			return -EBUSY;
		} else {
			i2s->in_use = 1;
			pm_runtime_get_sync(i2s->dev);
			dev_dbg(i2s->dev, "%s I2S %d", __func__, id);
			return 0;
		}
	} else {
		pr_err("%s Invalid I2S id %d", __func__, id);
		return -EINVAL;
	}
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(tegra210_i2s_get);

/* i2s pm runtime sync idle */
int tegra210_i2s_put(enum tegra210_ahub_cifs cif)
{
	int id = I2S_ID(cif);
	struct tegra210_i2s_ctx *i2s = tegra210_i2s[id];

	if (!IS_I2S(cif)) {
		pr_err("%s Invalid I2S id %d", __func__, id);
		return -EINVAL;
	}
	i2s = tegra210_i2s[id];

	if (i2s->in_use) {
		i2s->in_use = 0;
		pm_runtime_put_sync(i2s->dev);
	} else
		dev_warn(i2s->dev, "%s I2S %d not in use", __func__, id);

	dev_dbg(i2s->dev, "%s I2S %d", __func__, id);
	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_i2s_put);

void tegra210_i2s_enable(enum tegra210_ahub_cifs cif, bool en)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S ID %d", __func__, id);
	regmap_write(i2s->regmap, TEGRA210_I2S_ENABLE, en);
}
EXPORT_SYMBOL(tegra210_i2s_enable);

void tegra210_i2s_slcg_enable(enum tegra210_ahub_cifs cif, bool en)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	regmap_write(i2s->regmap, TEGRA210_I2S_CG, en);
}
EXPORT_SYMBOL(tegra210_i2s_slcg_enable);

int tegra210_i2s_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);
	u32 reg;

	reg = (IS_I2S_TX(cif) ? TEGRA210_I2S_AXBAR_TX_CIF_CTRL :
			TEGRA210_I2S_AXBAR_RX_CIF_CTRL);
	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s i2s cif %d", __func__, cif);

	return tegra210_set_axbar_cif_param(i2s->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_i2s_set_acif_param);

int tegra210_i2s_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);
	u32 reg;

	reg = (IS_I2S_TX(cif) ? TEGRA210_I2S_AXBAR_TX_CIF_CTRL :
		TEGRA210_I2S_AXBAR_RX_CIF_CTRL);
	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S cif %d", __func__, cif);

	return tegra210_get_axbar_cif_param(i2s->dev, reg, acif);
}
EXPORT_SYMBOL_GPL(tegra210_i2s_get_acif_param);

void tegra210_i2s_loopback_enable(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S %d loopback enabled", __func__, id);

	regmap_update_bits(i2s->regmap, TEGRA210_I2S_CTRL,
			   TEGRA210_I2S_CTRL_LPBK_ENABLE,
			   TEGRA210_I2S_CTRL_LPBK_ENABLE);
}
EXPORT_SYMBOL(tegra210_i2s_loopback_enable);

int tegra210_i2s_set_fmt(enum tegra210_ahub_cifs cif,
				unsigned int fmt, unsigned int bit_size,
				int fsync_width)
{
	struct tegra210_i2s_ctx *i2s;
	unsigned int mask, val, mask_highz, val_highz;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	mask = TEGRA210_I2S_CTRL_EGDE_CTRL_MASK;
	mask_highz = TEGRA210_I2S_AXBAR_RX_CTRL_HIGHZ_CTRL_MASK;
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val = TEGRA210_I2S_CTRL_EGDE_CTRL_POS_EDGE;
		if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK)
			== SND_SOC_DAIFMT_DSP_A) {
			val = TEGRA210_I2S_CTRL_EGDE_CTRL_NEG_EDGE;
			val_highz =
			TEGRA210_I2S_AXBAR_RX_CTRL_HIGHZ_CTRL_ON_HALF_BIT_CLK;
		}
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val = TEGRA210_I2S_CTRL_EGDE_CTRL_NEG_EDGE;
		if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK)
			== SND_SOC_DAIFMT_DSP_A) {
			val = TEGRA210_I2S_CTRL_EGDE_CTRL_POS_EDGE;
		}
		break;
	default:
		return -EINVAL;
	}

	if (IS_I2S_RX(cif))
		regmap_update_bits(i2s->regmap, TEGRA210_I2S_AXBAR_RX_CTRL,
			mask_highz, val_highz);

	mask |= TEGRA210_I2S_CTRL_MASTER_ENABLE;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= TEGRA210_I2S_CTRL_MASTER_ENABLE;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		val = 0;
		break;
	default:
		return -EINVAL;
	}

	mask |= TEGRA210_I2S_CTRL_FRAME_FORMAT_MASK |
		TEGRA210_I2S_CTRL_LRCK_MASK;
	i2s->daifmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	switch (i2s->daifmt) {
	case SND_SOC_DAIFMT_DSP_A:
		val |= TEGRA210_I2S_CTRL_FRAME_FORMAT_FSYNC |
		      TEGRA210_I2S_CTRL_LRCK_R_LOW;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		val |= TEGRA210_I2S_CTRL_FRAME_FORMAT_FSYNC |
		      TEGRA210_I2S_CTRL_LRCK_R_LOW;
		break;
	case SND_SOC_DAIFMT_I2S:
		val |= TEGRA210_I2S_CTRL_FRAME_FORMAT_LRCK |
		      TEGRA210_I2S_CTRL_LRCK_L_LOW;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val |= TEGRA210_I2S_CTRL_FRAME_FORMAT_LRCK |
		      TEGRA210_I2S_CTRL_LRCK_R_LOW;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val |= TEGRA210_I2S_CTRL_FRAME_FORMAT_LRCK |
		      TEGRA210_I2S_CTRL_LRCK_R_LOW;
		break;
	default:
		return -EINVAL;
	}

	mask |= TEGRA210_I2S_CTRL_BIT_SIZE_MASK;
	switch (bit_size) {
	case SNDRV_PCM_FORMAT_S8:
		val |= TEGRA210_I2S_CTRL_BIT_SIZE_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		val |= TEGRA210_I2S_CTRL_BIT_SIZE_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val |= TEGRA210_I2S_CTRL_BIT_SIZE_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val |= TEGRA210_I2S_CTRL_BIT_SIZE_32;
		break;
	default:
		return -EINVAL;
	}

	mask |= TEGRA210_I2S_CTRL_FSYNC_WIDTH_MASK;
	val |= fsync_width <<
			TEGRA210_I2S_CTRL_FSYNC_WIDTH_SHIFT;

	regmap_update_bits(i2s->regmap, TEGRA210_I2S_CTRL, mask, val);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_i2s_set_fmt);

void tegra210_i2s_set_channel_bit_count(enum tegra210_ahub_cifs cif,
				int i2sclock, int srate)
{
	struct tegra210_i2s_ctx *i2s;
	int sym_bitclk, bitcnt;
	unsigned int mask, val;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];

	if (i2s->daifmt == SND_SOC_DAIFMT_DSP_A) {
		bitcnt = (i2sclock / srate) - 1;
		sym_bitclk = !(i2sclock % srate);
	} else {
		bitcnt = (i2sclock / (2 * srate)) - 1;
		sym_bitclk = !(i2sclock % (2 * srate));
	}

	mask = TEGRA210_I2S_TIMING_CHANNEL_BIT_COUNT_MASK;
	val = bitcnt << TEGRA210_I2S_TIMING_CHANNEL_BIT_COUNT_SHIFT;
	dev_vdbg(i2s->dev,
		"%s sample rate %d,  i2sclock %d , bitcnt %d  sym_bitclk %d ",
		__func__, srate, i2sclock, bitcnt, sym_bitclk);

	mask |= TEGRA210_I2S_TIMING_NON_SYM_ENABLE;
	if (!sym_bitclk)
		val |= TEGRA210_I2S_TIMING_NON_SYM_ENABLE;

	regmap_update_bits(i2s->regmap, TEGRA210_I2S_TIMING, mask, val);

}
EXPORT_SYMBOL_GPL(tegra210_i2s_set_channel_bit_count);

void tegra210_i2s_set_data_offset(enum tegra210_ahub_cifs cif, int data_offset)
{
	struct tegra210_i2s_ctx *i2s;
	unsigned int mask, val;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	if (IS_I2S_RX(cif)) {
		mask = TEGRA210_I2S_AXBAR_RX_CTRL_DATA_OFFSET_MASK;
		val = (data_offset <<
			TEGRA210_I2S_AXBAR_RX_CTRL_DATA_OFFSET_SHIFT);
		regmap_update_bits(i2s->regmap, TEGRA210_I2S_AXBAR_RX_CTRL,
			mask, val);
	} else {
		mask = TEGRA210_I2S_AXBAR_TX_CTRL_DATA_OFFSET_MASK;
		val = (data_offset <<
			TEGRA210_I2S_AXBAR_TX_CTRL_DATA_OFFSET_SHIFT);
		regmap_update_bits(i2s->regmap, TEGRA210_I2S_AXBAR_TX_CTRL,
			mask, val);
	}
}
EXPORT_SYMBOL_GPL(tegra210_i2s_set_data_offset);

static int tegra210_i2s_axbar_tx_is_enabled(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);
	u32 val = 0;

	i2s = tegra210_i2s[id];
	regmap_read(i2s->regmap, TEGRA210_I2S_STATUS, &val);
	val &= TEGRA210_I2S_STATUS_TX_ENABLED;

	dev_vdbg(i2s->dev, "%s I2S TX enable %d", __func__, val);

	return val;
}

static int tegra210_i2s_axbar_rx_is_enabled(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);
	u32 val = 0;

	i2s = tegra210_i2s[id];
	regmap_read(i2s->regmap, TEGRA210_I2S_STATUS, &val);
	val &= TEGRA210_I2S_STATUS_RX_ENABLED;

	dev_vdbg(i2s->dev, "%s I2S RX enable %d", __func__, val);

	return val;
}

static int tegra210_i2s_axbar_tx_fifo_is_empty(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);
	u32 val = 0;

	i2s = tegra210_i2s[id];
	regmap_read(i2s->regmap, TEGRA210_I2S_STATUS, &val);
	val &= TEGRA210_I2S_STATUS_TXCIF_FIFO_EMPTY;

	dev_vdbg(i2s->dev, "%s I2S TX FIFO empty %d", __func__, val);

	return val;
}

static int tegra210_i2s_axbar_rx_fifo_is_empty(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);
	u32 val = 0;

	i2s = tegra210_i2s[id];
	regmap_read(i2s->regmap, TEGRA210_I2S_STATUS, &val);
	val &= TEGRA210_I2S_STATUS_RXCIF_FIFO_EMPTY;

	dev_vdbg(i2s->dev, "%s I2S RX FIFO empty %d", __func__, val);
	return val;

}

void tegra210_i2s_set_slot_control(enum tegra210_ahub_cifs cif,
	    int slot_enables, int total_slots)
{
	struct tegra210_i2s_ctx *i2s;
	unsigned int mask, val;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	if (IS_I2S_TX(cif)) {
		mask = TEGRA210_I2S_SLOT_CTRL_TX_SLOT_ENABLES_MASK;
		val = (slot_enables <<
			TEGRA210_I2S_SLOT_CTRL_TX_SLOT_ENABLES_SHIFT);
		regmap_update_bits(i2s->regmap, TEGRA210_I2S_AXBAR_TX_SLOT_CTRL,
			mask, val);
	} else {
		mask = TEGRA210_I2S_SLOT_CTRL_RX_SLOT_ENABLES_MASK;
		val = (slot_enables <<
			TEGRA210_I2S_SLOT_CTRL_RX_SLOT_ENABLES_SHIFT);
		regmap_update_bits(i2s->regmap, TEGRA210_I2S_AXBAR_RX_SLOT_CTRL,
			mask, val);
	}
	mask = TEGRA210_I2S_SLOT_CTRL_TOTAL_SLOTS_MASK;
	val = (total_slots) <<
	       TEGRA210_I2S_SLOT_CTRL_TOTAL_SLOTS_SHIFT;
	regmap_update_bits(i2s->regmap, TEGRA210_I2S_SLOT_CTRL, mask, val);
}
EXPORT_SYMBOL_GPL(tegra210_i2s_set_slot_control);

int tegra210_i2s_soft_reset(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	unsigned int val;
	int dcnt = 10,  id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S ID %d", __func__, id);
	regmap_write(i2s->regmap, TEGRA210_I2S_SOFT_RESET, 1);

	do {
		udelay(100);
		regmap_read(i2s->regmap, TEGRA210_I2S_SOFT_RESET, &val);
	} while (val && dcnt--);

	return (dcnt < 0) ? -ETIMEDOUT : 0;
}
EXPORT_SYMBOL_GPL(tegra210_i2s_soft_reset);

void tegra210_i2s_start_capture(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S ID %d", __func__, id);

	regmap_write(i2s->regmap, TEGRA210_I2S_AXBAR_TX_ENABLE, 1);

}
EXPORT_SYMBOL_GPL(tegra210_i2s_start_playback);

void tegra210_i2s_stop_capture(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int dcnt = 10,  id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S ID %d", __func__, id);

	regmap_write(i2s->regmap, TEGRA210_I2S_AXBAR_TX_ENABLE, 0);

	while (tegra210_i2s_axbar_tx_is_enabled(cif) && dcnt--)
		udelay(100);

	dcnt = 10;
	while (!tegra210_i2s_axbar_tx_fifo_is_empty(cif) && dcnt--)
		udelay(100);

	/* In case I2S FIFO does not get empty do a soft reset of the
	   I2S channel to prevent channel reversal in next session */
	if (dcnt < 0) {
		tegra210_i2s_soft_reset(cif);

		dcnt = 10;
		while (!tegra210_i2s_axbar_tx_fifo_is_empty(cif) &&
		       dcnt--)
			udelay(100);
		}

}
EXPORT_SYMBOL_GPL(tegra210_i2s_stop_playback);

void tegra210_i2s_start_playback(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S ID %d", __func__, id);

	regmap_write(i2s->regmap, TEGRA210_I2S_AXBAR_RX_ENABLE, 1);
}
EXPORT_SYMBOL_GPL(tegra210_i2s_start_capture);

void tegra210_i2s_stop_playback(enum tegra210_ahub_cifs cif)
{
	struct tegra210_i2s_ctx *i2s;
	int dcnt = 10,  id = I2S_ID(cif);

	i2s = tegra210_i2s[id];
	dev_vdbg(i2s->dev, "%s I2S ID %d", __func__, id);

	regmap_write(i2s->regmap, TEGRA210_I2S_AXBAR_RX_ENABLE, 0);
	while (tegra210_i2s_axbar_rx_is_enabled(cif) && dcnt--)
		udelay(100);

	dcnt = 10;
	while (!tegra210_i2s_axbar_rx_fifo_is_empty(cif) && dcnt--)
		udelay(100);

	/* In case I2S FIFO does not get empty do a soft reset of
	   the I2S channel to prevent channel reversal in next capture
	   session */
	if (dcnt < 0) {
		tegra210_i2s_soft_reset(cif);

		dcnt = 10;
		while (!tegra210_i2s_axbar_rx_fifo_is_empty(cif) && dcnt--)
			udelay(100);
	}
}
EXPORT_SYMBOL_GPL(tegra210_i2s_stop_capture);

static bool tegra210_i2s_rw_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_I2S_AXBAR_RX_ENABLE:
	case TEGRA210_I2S_AXBAR_RX_INT_MASK:
	case TEGRA210_I2S_AXBAR_RX_INT_SET:
	case TEGRA210_I2S_AXBAR_RX_INT_CLEAR:
	case TEGRA210_I2S_AXBAR_RX_CIF_CTRL:
	case TEGRA210_I2S_AXBAR_RX_CTRL:
	case TEGRA210_I2S_AXBAR_RX_SLOT_CTRL:
	case TEGRA210_I2S_AXBAR_RX_CLK_TRIM:
	case TEGRA210_I2S_AXBAR_TX_ENABLE:
	case TEGRA210_I2S_AXBAR_TX_INT_MASK:
	case TEGRA210_I2S_AXBAR_TX_INT_SET:
	case TEGRA210_I2S_AXBAR_TX_INT_CLEAR:
	case TEGRA210_I2S_AXBAR_TX_CIF_CTRL:
	case TEGRA210_I2S_AXBAR_TX_CTRL:
	case TEGRA210_I2S_AXBAR_TX_SLOT_CTRL:
	case TEGRA210_I2S_AXBAR_TX_CLK_TRIM:
	case TEGRA210_I2S_ENABLE:
	case TEGRA210_I2S_SOFT_RESET:
	case TEGRA210_I2S_CG:
	case TEGRA210_I2S_CTRL:
	case TEGRA210_I2S_TIMING:
	case TEGRA210_I2S_SLOT_CTRL:
	case TEGRA210_I2S_CLK_TRIM:
		return true;
	default:
		return false;
	};
}

static bool tegra210_i2s_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_I2S_AXBAR_RX_STATUS:
	case TEGRA210_I2S_AXBAR_TX_STATUS:
	case TEGRA210_I2S_STATUS:
	case TEGRA210_I2S_AXBAR_RX_CIF_FIFO_STATUS:
	case TEGRA210_I2S_AXBAR_TX_CIF_FIFO_STATUS:
	case TEGRA210_I2S_AXBAR_RX_ENABLE:
	case TEGRA210_I2S_AXBAR_RX_INT_MASK:
	case TEGRA210_I2S_AXBAR_RX_INT_SET:
	case TEGRA210_I2S_AXBAR_RX_INT_CLEAR:
	case TEGRA210_I2S_AXBAR_RX_CIF_CTRL:
	case TEGRA210_I2S_AXBAR_RX_CTRL:
	case TEGRA210_I2S_AXBAR_RX_SLOT_CTRL:
	case TEGRA210_I2S_AXBAR_RX_CLK_TRIM:
	case TEGRA210_I2S_AXBAR_TX_ENABLE:
	case TEGRA210_I2S_AXBAR_TX_INT_MASK:
	case TEGRA210_I2S_AXBAR_TX_INT_SET:
	case TEGRA210_I2S_AXBAR_TX_INT_CLEAR:
	case TEGRA210_I2S_AXBAR_TX_CIF_CTRL:
	case TEGRA210_I2S_AXBAR_TX_CTRL:
	case TEGRA210_I2S_AXBAR_TX_SLOT_CTRL:
	case TEGRA210_I2S_AXBAR_TX_CLK_TRIM:
	case TEGRA210_I2S_ENABLE:
	case TEGRA210_I2S_SOFT_RESET:
	case TEGRA210_I2S_CG:
	case TEGRA210_I2S_CTRL:
	case TEGRA210_I2S_TIMING:
	case TEGRA210_I2S_SLOT_CTRL:
	case TEGRA210_I2S_CLK_TRIM:
	case TEGRA210_I2S_AXBAR_RX_INT_STATUS:
	case TEGRA210_I2S_AXBAR_TX_INT_STATUS:
	case TEGRA210_I2S_INT_STATUS:
	case TEGRA210_I2S_AXBAR_RX_SOFT_RESET:
	case TEGRA210_I2S_AXBAR_TX_SOFT_RESET:
		return true;
	default:
		return false;
	};
}

static bool tegra210_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_I2S_AXBAR_RX_INT_STATUS:
	case TEGRA210_I2S_AXBAR_TX_INT_STATUS:
	case TEGRA210_I2S_INT_STATUS:
	case TEGRA210_I2S_AXBAR_RX_SOFT_RESET:
	case TEGRA210_I2S_AXBAR_TX_SOFT_RESET:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_I2S_SLOT_CTRL,
	.writeable_reg = tegra210_i2s_rw_reg,
	.readable_reg = tegra210_i2s_rd_reg,
	.volatile_reg = tegra210_i2s_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int tegra210_i2s_runtime_suspend(struct device *dev)
{
	struct tegra210_i2s_ctx *i2s = dev_get_drvdata(dev);

	regcache_cache_only(i2s->regmap, true);
#ifndef CONFIG_MACH_GRENADA
	tegra210_axbar_disable_clk();
	clk_disable_unprepare(i2s->clk_i2s);
#endif
	return 0;
}

static int tegra210_i2s_runtime_resume(struct device *dev)
{
	struct tegra210_i2s_ctx *i2s = dev_get_drvdata(dev);
#ifndef CONFIG_MACH_GRENADA
	int ret;
	tegra210_axbar_enable_clk();

	ret = clk_prepare_enable(i2s->clk_i2s);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}
#endif
	regcache_cache_only(i2s->regmap, false);

	return 0;
}

static int tegra210_i2s_platform_probe(struct platform_device *pdev)
{
	struct tegra210_i2s_ctx *i2s;
	struct resource *mem, *memregion;
	void __iomem *regs;
	int ret = 0;
	int id = 0, srate = 48000, channels = 2, bps = 16;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, " %s failed, no DT node for I2S ",
			__func__);
		return -EINVAL;
	}

	of_property_read_u32(pdev->dev.of_node, "nvidia,ahub-i2s-id",
						 &pdev->id);

	dev_dbg(&pdev->dev, "%s I2S %d : starting probe",
		__func__, pdev->id);

	id = pdev->id;

	i2s = devm_kzalloc(&pdev->dev, sizeof(struct tegra210_i2s_ctx),
		GFP_KERNEL);
	if (!i2s) {
		dev_err(&pdev->dev, "Can't allocate tegra210_i2s_ctx\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, i2s);
	i2s->dev = &pdev->dev;
	i2s->id = id;

	if ((i2s->id < 0) ||
		(i2s->id > TEGRA210_I2S_COUNT)) {
		dev_err(&pdev->dev, "I2S id %d out of range\n", i2s->id);
		return -EINVAL;
	}

	tegra210_i2s[i2s->id] = i2s;

	i2s->clk_i2s = clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2s->clk_i2s)) {
		dev_err(&pdev->dev, "Can't retrieve I2S %d clock\n", i2s->id);
		ret = PTR_ERR(i2s->clk_i2s);
		goto err;
	}

	i2s->clk_pll_a_out0 = clk_get_sys(NULL, "pll_a_out0");
	if (IS_ERR(i2s->clk_pll_a_out0)) {
		dev_err(&pdev->dev, "Can't retrieve pll_a_out0 clock\n");
		ret = PTR_ERR(i2s->clk_pll_a_out0);
		goto err_i2s_clk_put;
	}

	ret = clk_set_parent(i2s->clk_i2s, i2s->clk_pll_a_out0);
	if (ret) {
		dev_err(&pdev->dev, "Can't set parent of I2S clock\n");
		goto err_pll_a_out0_clk_put;
	}

	ret = clk_set_rate(i2s->clk_i2s, srate * channels * bps);
	if (ret) {
		dev_err(&pdev->dev, "Can't set I2S clock rate: %d\n", ret);
		goto err_pll_a_out0_clk_put;
	}

	ret = clk_prepare_enable(i2s->clk_i2s);
	if (ret) {
		dev_err(&pdev->dev, "clk_enable failed: %d\n", ret);
		goto err_pll_a_out0_clk_put;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_pll_a_out0_clk_put;
	}

	memregion = devm_request_mem_region(&pdev->dev, mem->start,
					    resource_size(mem), DRV_NAME);
	if (!memregion) {
		dev_err(&pdev->dev, "Memory region already claimed\n");
		ret = -EBUSY;
		goto err_pll_a_out0_clk_put;
	}

	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_pll_a_out0_clk_put;
	}

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &tegra210_i2s_regmap_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(i2s->regmap);
		goto err_pll_a_out0_clk_put;
	}
	regcache_cache_only(i2s->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_i2s_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	return 0;
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_pll_a_out0_clk_put:
	clk_put(i2s->clk_pll_a_out0);
err_i2s_clk_put:
	clk_put(i2s->clk_i2s);
err:
	return ret;
}

static int tegra210_i2s_platform_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_i2s_runtime_suspend(&pdev->dev);
	return 0;
}

static const struct of_device_id tegra210_i2s_of_match[] = {
	{ .compatible = "nvidia,tegra210-i2s",},
	{},
};

static const struct dev_pm_ops tegra210_i2s_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_i2s_runtime_suspend,
			   tegra210_i2s_runtime_resume, NULL)
};

static struct platform_driver tegra210_i2s_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_i2s_of_match,
		.pm = &tegra210_i2s_pm_ops,
	},
	.probe = tegra210_i2s_platform_probe,
	.remove = tegra210_i2s_platform_remove,
};
module_platform_driver(tegra210_i2s_driver);

MODULE_AUTHOR("Dara Ramesh <dramesh@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 I2S ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
