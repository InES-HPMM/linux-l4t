/*
 * tegra30_i2s.c - Tegra30 I2S driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2010-2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * Based on code copyright/by:
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Copyright (c) 2012, NVIDIA CORPORATION. All rights reserved.
 * Scott Peterson <speterson@nvidia.com>
 *
 * Copyright (C) 2010 Google, Inc.
 * Iliyan Malchev <malchev@google.com>
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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <mach/tegra_asoc_pdata.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra30_ahub.h"
#include "tegra30_dam.h"
#include "tegra30_i2s.h"

#define DRV_NAME "tegra30-i2s"

static struct tegra30_i2s  i2scont[TEGRA30_NR_I2S_IFC];

static int tegra30_i2s_runtime_suspend(struct device *dev)
{
	struct tegra30_i2s *i2s = dev_get_drvdata(dev);

	regcache_cache_only(i2s->regmap, true);

	clk_disable_unprepare(i2s->clk_i2s);

	return 0;
}

static int tegra30_i2s_runtime_resume(struct device *dev)
{
	struct tegra30_i2s *i2s = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(i2s->clk_i2s);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	regcache_cache_only(i2s->regmap, false);

	return 0;
}

int tegra30_i2s_startup(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* increment the playback ref count */
		i2s->playback_ref_count++;

		ret = tegra30_ahub_allocate_tx_fifo(&i2s->playback_fifo_cif,
					&i2s->playback_dma_data.addr,
					&i2s->playback_dma_data.req_sel);
		i2s->playback_dma_data.wrap = 4;
		i2s->playback_dma_data.width = 32;

		if (!i2s->is_dam_used)
			tegra30_ahub_set_rx_cif_source(i2s->playback_i2s_cif,
						       i2s->playback_fifo_cif);
	} else {
		i2s->capture_ref_count++;
		ret = tegra30_ahub_allocate_rx_fifo(&i2s->capture_fifo_cif,
					&i2s->capture_dma_data.addr,
					&i2s->capture_dma_data.req_sel);
		i2s->capture_dma_data.wrap = 4;
		i2s->capture_dma_data.width = 32;
		tegra30_ahub_set_rx_cif_source(i2s->capture_fifo_cif,
					       i2s->capture_i2s_cif);
	}

	return ret;
}

void tegra30_i2s_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (i2s->playback_ref_count == 1)
			tegra30_ahub_unset_rx_cif_source(i2s->playback_i2s_cif);

		/* free the apbif dma channel*/
		tegra30_ahub_free_tx_fifo(i2s->playback_fifo_cif);

		/* decrement the playback ref count */
		i2s->playback_ref_count--;
	} else {
		if (i2s->capture_ref_count == 1)
			tegra30_ahub_unset_rx_cif_source(i2s->capture_fifo_cif);
		tegra30_ahub_free_rx_fifo(i2s->capture_fifo_cif);
		i2s->capture_ref_count--;
	}
}

static int tegra30_i2s_set_fmt(struct snd_soc_dai *dai,
				unsigned int fmt)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int mask, val;

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	mask = TEGRA30_I2S_CTRL_MASTER_ENABLE;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		val = TEGRA30_I2S_CTRL_MASTER_ENABLE;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	mask |= TEGRA30_I2S_CTRL_FRAME_FORMAT_MASK |
		TEGRA30_I2S_CTRL_LRCK_MASK;
	i2s->reg_ch_ctrl &= ~TEGRA30_I2S_CH_CTRL_EGDE_CTRL_MASK;
	i2s->daifmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
	switch (i2s->daifmt) {
	case SND_SOC_DAIFMT_DSP_A:
		val |= TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC;
		val |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_NEG_EDGE;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		val |= TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC;
		val |= TEGRA30_I2S_CTRL_LRCK_R_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_POS_EDGE;
		break;
	case SND_SOC_DAIFMT_I2S:
		val |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		val |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_POS_EDGE;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		val |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_POS_EDGE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		val |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_POS_EDGE;
		break;
	default:
		return -EINVAL;
	}

	pm_runtime_get_sync(dai->dev);
	regmap_update_bits(i2s->regmap, TEGRA30_I2S_CTRL, mask, val);
	pm_runtime_put(dai->dev);

	return 0;
}

static void tegra30_i2s_set_channel_bit_count(struct tegra30_i2s *i2s,
				int i2sclock, int srate)
{
	int sym_bitclk, bitcnt;
	u32 val;

	bitcnt = (i2sclock / (2 * srate)) - 1;
	sym_bitclk = !(i2sclock % (2 * srate));

	val = bitcnt << TEGRA30_I2S_TIMING_CHANNEL_BIT_COUNT_SHIFT;

	if (!sym_bitclk)
		val |= TEGRA30_I2S_TIMING_NON_SYM_ENABLE;

	regmap_write(i2s->regmap, TEGRA30_I2S_TIMING, val);
}

static void tegra30_i2s_set_data_offset(struct tegra30_i2s *i2s)
{
	u32 val;
	int rx_data_offset = i2s->dsp_config.rx_data_offset;
	int tx_data_offset = i2s->dsp_config.tx_data_offset;

	val = (rx_data_offset <<
				TEGRA30_I2S_OFFSET_RX_DATA_OFFSET_SHIFT) |
			(tx_data_offset <<
				TEGRA30_I2S_OFFSET_TX_DATA_OFFSET_SHIFT);

	regmap_write(i2s->regmap, TEGRA30_I2S_OFFSET, val);
}

static void tegra30_i2s_set_slot_control(struct tegra30_i2s *i2s, int stream)
{
	u32 mask, val = 0;
	int tx_mask = i2s->dsp_config.tx_mask;
	int rx_mask = i2s->dsp_config.rx_mask;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mask = TEGRA30_I2S_SLOT_CTRL_TX_SLOT_ENABLES_MASK;
		val |= (tx_mask << TEGRA30_I2S_SLOT_CTRL_TX_SLOT_ENABLES_SHIFT);
	} else {
		mask = TEGRA30_I2S_SLOT_CTRL_RX_SLOT_ENABLES_MASK;
		val |= (rx_mask << TEGRA30_I2S_SLOT_CTRL_RX_SLOT_ENABLES_SHIFT);
	}

	mask |= TEGRA30_I2S_SLOT_CTRL_TOTAL_SLOTS_MASK;
	val |= (i2s->dsp_config.num_slots - 1)
			<< TEGRA30_I2S_SLOT_CTRL_TOTAL_SLOTS_SHIFT;

	regmap_update_bits(i2s->regmap, TEGRA30_I2S_SLOT_CTRL, mask, val);
}

static int tegra30_i2s_tdm_setup_clocks(struct device *dev,
				struct tegra30_i2s *i2s, int *i2sclock)
{
	int ret;

	if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_MASTER_ENABLE) {

		ret = clk_set_parent(i2s->clk_i2s, i2s->clk_pll_a_out0);
		if (ret) {
			dev_err(dev, "Can't set parent of I2S clock\n");
			return ret;
		}

		ret = clk_set_rate(i2s->clk_i2s, *i2sclock);
		if (ret) {
			dev_err(dev, "Can't set I2S clock rate: %d\n", ret);
			return ret;
		}
	} else {

		ret = clk_set_rate(i2s->clk_i2s_sync, *i2sclock);
		if (ret) {
			dev_err(dev, "Can't set I2S sync clock rate\n");
			return ret;
		}

		ret = clk_set_parent(clk_get_parent(i2s->clk_audio_2x),
						i2s->clk_i2s_sync);
		if (ret) {
			dev_err(dev, "Can't set parent of audio2x clock\n");
			return ret;
		}

		ret = clk_set_rate(i2s->clk_audio_2x, *i2sclock);
		if (ret) {
			dev_err(dev, "Can't set audio2x clock rate\n");
			return ret;
		}

		ret = clk_set_parent(i2s->clk_i2s, i2s->clk_audio_2x);
		if (ret) {
			dev_err(dev, "Can't set parent of i2s clock\n");
			return ret;
		}
	}
	return ret;
}


static int tegra30_i2s_tdm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct device *dev = substream->pcm->card->dev;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val;
	int i2s_client_ch, i2s_audio_ch;
	int i2s_audio_bits = 0, i2s_client_bits = 0;
	int i2sclock, srate;
	int ret;

	srate = params_rate(params);

	i2sclock = srate *
				i2s->dsp_config.num_slots *
				i2s->dsp_config.slot_width;

	ret = tegra30_i2s_tdm_setup_clocks(dev, i2s, &i2sclock);
	if (ret)
		return -EINVAL;

	/* Run ahub clock greater than i2sclock */
	tegra30_ahub_clock_set_rate(i2sclock*2);

	tegra30_i2s_set_channel_bit_count(i2s, i2sclock*2, srate);

	i2s_client_ch = i2s->dsp_config.num_slots;
	i2s_audio_ch = i2s->dsp_config.num_slots;

	i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_BIT_SIZE_MASK;
	switch (i2s->dsp_config.slot_width) {
	case 16:
		i2s_audio_bits = TEGRA30_AUDIOCIF_BITS_16;
		i2s_client_bits = TEGRA30_AUDIOCIF_BITS_16;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_16;
		break;
	case 32:
		i2s_audio_bits = TEGRA30_AUDIOCIF_BITS_32;
		i2s_client_bits = TEGRA30_AUDIOCIF_BITS_32;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_32;
		break;
	default:
		dev_err(dev, "unknown slot_width %d\n",
				i2s->dsp_config.slot_width);
		return -EINVAL;
	}

	val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
			((i2s_audio_ch - 1) <<
			 TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
			((i2s_client_ch - 1) <<
			 TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
			(i2s_audio_bits <<
			 TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT) |
			(i2s_client_bits <<
			 TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_RX;
		tegra30_i2s_write(i2s, TEGRA30_I2S_CIF_RX_CTRL, val);

		tegra30_ahub_set_tx_cif_channels(i2s->playback_fifo_cif,
						i2s_audio_ch,
						i2s_client_ch);
		tegra30_ahub_set_tx_cif_bits(i2s->playback_fifo_cif,
						i2s_audio_bits,
						i2s_client_bits);
		tegra30_ahub_set_tx_fifo_pack_mode(i2s->playback_fifo_cif, 0);

	} else {
		val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_TX;
		tegra30_i2s_write(i2s, TEGRA30_I2S_CIF_TX_CTRL, val);

		tegra30_ahub_set_rx_cif_channels(i2s->capture_fifo_cif,
						i2s_audio_ch,
						i2s_client_ch);
		tegra30_ahub_set_rx_cif_bits(i2s->capture_fifo_cif,
						i2s_audio_bits,
						i2s_client_bits);
		tegra30_ahub_set_rx_fifo_pack_mode(i2s->capture_fifo_cif, 0);
	}

	tegra30_i2s_set_slot_control(i2s, substream->stream);

	tegra30_i2s_set_data_offset(i2s);

	i2s->reg_ch_ctrl &= ~TEGRA30_I2S_CH_CTRL_FSYNC_WIDTH_MASK;
	i2s->reg_ch_ctrl |= (i2s->dsp_config.slot_width - 1) <<
			TEGRA30_I2S_CH_CTRL_FSYNC_WIDTH_SHIFT;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CH_CTRL, i2s->reg_ch_ctrl);

	return 0;
}

static int tegra30_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int mask, val, reg, i;
	int ret, sample_size, srate, i2sclock, bitcnt, sym_bitclk;
	int i2s_client_ch;

	mask = TEGRA30_I2S_CTRL_BIT_SIZE_MASK;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_8;
		sample_size = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		val = TEGRA30_I2S_CTRL_BIT_SIZE_16;
		sample_size = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_24;
		sample_size = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_32;
		sample_size = 32;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, TEGRA30_I2S_CTRL, mask, val);

	bitcnt = sample_size;
	i = 0;

	/* TDM mode */
	if ((i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC) &&
		(i2s->dsp_config.slot_width > 2))
		return tegra30_i2s_tdm_hw_params(substream, params, dai);

	srate = params_rate(params);

	if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_MASTER_ENABLE) {
		i2sclock = srate * params_channels(params) * sample_size;

		/* Additional "* 4" is needed for FSYNC mode */
		if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC)
			i2sclock *= 4;

		ret = clk_set_parent(i2s->clk_i2s, i2s->clk_pll_a_out0);
		if (ret) {
			dev_err(dev, "Can't set parent of I2S clock\n");
			return ret;
		}

		ret = clk_set_rate(i2s->clk_i2s, i2sclock);
		if (ret) {
			dev_err(dev, "Can't set I2S clock rate: %d\n", ret);
			return ret;
		}

		if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC) {
			bitcnt = (i2sclock / srate) - 1;
			sym_bitclk = !(i2sclock % srate);
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
			val = 0;
			for (i = 0; i < params_channels(params); i++)
				val |= (1 << i);
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				val = val <<
				  TEGRA30_I2S_SLOT_CTRL2_TX_SLOT_ENABLES_SHIFT;
			else
				val = val <<
				  TEGRA30_I2S_SLOT_CTRL2_RX_SLOT_ENABLES_SHIFT;
			tegra30_i2s_write(i2s, TEGRA30_I2S_SLOT_CTRL2, val);
#endif
		} else {
			bitcnt = (i2sclock / (2 * srate)) - 1;
			sym_bitclk = !(i2sclock % (2 * srate));
			i2s_client_ch = 2;
		}
		val = bitcnt << TEGRA30_I2S_TIMING_CHANNEL_BIT_COUNT_SHIFT;

		if (!sym_bitclk)
			val |= TEGRA30_I2S_TIMING_NON_SYM_ENABLE;

		regmap_write(i2s->regmap, TEGRA30_I2S_TIMING, val);
	} else {
		i2sclock = srate * params_channels(params) * sample_size;

		/* Additional "* 2" is needed for FSYNC mode */
		if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC)
			i2sclock *= 2;

		ret = clk_set_rate(i2s->clk_i2s_sync, i2sclock);
		if (ret) {
			dev_err(dev, "Can't set I2S sync clock rate\n");
			return ret;
		}

		ret = clk_set_parent(clk_get_parent(i2s->clk_audio_2x),
						i2s->clk_i2s_sync);
		if (ret) {
			dev_err(dev, "Can't set parent of audio2x clock\n");
			return ret;
		}

		ret = clk_set_rate(i2s->clk_audio_2x, i2sclock);
		if (ret) {
			dev_err(dev, "Can't set I2S sync clock rate\n");
			return ret;
		}

		ret = clk_set_parent(i2s->clk_i2s, i2s->clk_audio_2x);
		if (ret) {
			dev_err(dev, "Can't set parent of audio2x clock\n");
			return ret;
		}
	}

	i2s_client_ch = (i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC) ?
			params_channels(params) : 2;

	switch (sample_size) {
	case 8:
		val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
	      ((params_channels(params) - 1) <<
			TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
	      ((i2s_client_ch - 1) <<
			TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
	      TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_8 |
	      TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_8;
		break;

	case 16:
		val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
	      ((params_channels(params) - 1) <<
			TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
	      ((i2s_client_ch - 1) <<
			TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
	      TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_16 |
	      TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_16;
		break;

	case 24:
		val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
	      ((params_channels(params) - 1) <<
			TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
	      ((i2s_client_ch - 1) <<
			TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
	      TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_24 |
	      TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_24;
		break;

	case 32:
		val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
	      ((params_channels(params) - 1) <<
			TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
	      ((i2s_client_ch - 1) <<
			TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
	      TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_32 |
	      TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_32;
		break;

	default:
		pr_err("Error in sample size\n");
		val = 0;
		break;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_RX;
		reg = TEGRA30_I2S_CIF_RX_CTRL;

		tegra30_ahub_set_tx_cif_channels(i2s->playback_fifo_cif,
						 params_channels(params),
						 params_channels(params));

		switch (sample_size) {
		case 8:
			tegra30_ahub_set_tx_cif_bits(i2s->playback_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_8, TEGRA30_AUDIOCIF_BITS_8);
			tegra30_ahub_set_tx_fifo_pack_mode(i2s->playback_fifo_cif,
			  TEGRA30_AHUB_CHANNEL_CTRL_TX_PACK_8_4);
			break;

		case 16:
			tegra30_ahub_set_tx_cif_bits(i2s->playback_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_16, TEGRA30_AUDIOCIF_BITS_16);
			tegra30_ahub_set_tx_fifo_pack_mode(i2s->playback_fifo_cif,
			  TEGRA30_AHUB_CHANNEL_CTRL_TX_PACK_16);
			break;

		case 24:
			tegra30_ahub_set_tx_cif_bits(i2s->playback_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_24, TEGRA30_AUDIOCIF_BITS_24);
			tegra30_ahub_set_tx_fifo_pack_mode(i2s->playback_fifo_cif, 0);
			break;

		case 32:
			tegra30_ahub_set_tx_cif_bits(i2s->playback_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_32, TEGRA30_AUDIOCIF_BITS_32);
			tegra30_ahub_set_tx_fifo_pack_mode(i2s->playback_fifo_cif, 0);
			break;

		default:
			pr_err("Error in sample_size\n");
			break;
		}
	} else {
		val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_TX;
		reg = TEGRA30_I2S_CIF_TX_CTRL;

		tegra30_ahub_set_rx_cif_channels(i2s->capture_fifo_cif,
						 params_channels(params),
						 params_channels(params));

		switch (sample_size) {
		case 8:
			tegra30_ahub_set_rx_cif_bits(i2s->capture_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_8, TEGRA30_AUDIOCIF_BITS_8);
			tegra30_ahub_set_rx_fifo_pack_mode(i2s->capture_fifo_cif,
			  TEGRA30_AHUB_CHANNEL_CTRL_RX_PACK_8_4);
			break;

		case 16:
			tegra30_ahub_set_rx_cif_bits(i2s->capture_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_16, TEGRA30_AUDIOCIF_BITS_16);
			tegra30_ahub_set_rx_fifo_pack_mode(i2s->capture_fifo_cif,
			  TEGRA30_AHUB_CHANNEL_CTRL_RX_PACK_16);
			break;

		case 24:
			tegra30_ahub_set_rx_cif_bits(i2s->capture_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_24, TEGRA30_AUDIOCIF_BITS_24);
			tegra30_ahub_set_rx_fifo_pack_mode(i2s->capture_fifo_cif, 0);
			break;

		case 32:
			tegra30_ahub_set_rx_cif_bits(i2s->capture_fifo_cif,
			  TEGRA30_AUDIOCIF_BITS_32, TEGRA30_AUDIOCIF_BITS_32);
			tegra30_ahub_set_rx_fifo_pack_mode(i2s->capture_fifo_cif, 0);
			break;

		default:
			pr_err("Error in sample_size\n");
			break;
		}
	}

	regmap_write(i2s->regmap, reg, val);

	switch (i2s->daifmt) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = 0;
		if ((bitcnt - sample_size) > 0)
			val = bitcnt - sample_size;
		break;
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_LEFT_J:
		val = 0;
		break;
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A: /* fall through */
		val = 1;
		break;
	default:
		return -EINVAL;
	}
	val = (val << TEGRA30_I2S_OFFSET_RX_DATA_OFFSET_SHIFT) |
		  (val << TEGRA30_I2S_OFFSET_TX_DATA_OFFSET_SHIFT);
	regmap_write(i2s->regmap, TEGRA30_I2S_OFFSET, val);

	tegra30_i2s_write(i2s, TEGRA30_I2S_CH_CTRL, i2s->reg_ch_ctrl);

	val = 0;
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mask = TEGRA30_I2S_SLOT_CTRL_TX_SLOT_ENABLES_MASK;
		val |= (1 << TEGRA30_I2S_SLOT_CTRL_TX_SLOT_ENABLES_SHIFT);
	} else {
		mask = TEGRA30_I2S_SLOT_CTRL_RX_SLOT_ENABLES_MASK;
		val |= (1 << TEGRA30_I2S_SLOT_CTRL_RX_SLOT_ENABLES_SHIFT);
	}
	regmap_update_bits(i2s->regmap, TEGRA30_I2S_SLOT_CTRL, mask, val);
#else
	if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC)
		val = params_channels(params) - 1;
	regmap_write(i2s->regmap, TEGRA30_I2S_SLOT_CTRL, val);
#endif

	return 0;
}

static void tegra30_i2s_start_playback(struct tegra30_i2s *i2s)
{
	tegra30_ahub_enable_tx_fifo(i2s->playback_fifo_cif);
	/* if this is the only user of i2s tx then enable it*/
	if (i2s->playback_ref_count == 1)
		regmap_update_bits(i2s->regmap, TEGRA30_I2S_CTRL,
				   TEGRA30_I2S_CTRL_XFER_EN_TX,
				   TEGRA30_I2S_CTRL_XFER_EN_TX);
}

static void tegra30_i2s_stop_playback(struct tegra30_i2s *i2s)
{
	int dcnt = 10;
	/* if this is the only user of i2s tx then disable it*/
	tegra30_ahub_disable_tx_fifo(i2s->playback_fifo_cif);
	if (i2s->playback_ref_count == 1)
		regmap_update_bits(i2s->regmap, TEGRA30_I2S_CTRL,
				   TEGRA30_I2S_CTRL_XFER_EN_TX, 0);
	while (!tegra30_ahub_tx_fifo_is_empty(i2s->id) && dcnt--)
		udelay(100);
}

static void tegra30_i2s_start_capture(struct tegra30_i2s *i2s)
{
	tegra30_ahub_enable_rx_fifo(i2s->capture_fifo_cif);
	if (!i2s->is_call_mode_rec && (i2s->capture_ref_count == 1))
		regmap_update_bits(i2s->regmap, TEGRA30_I2S_CTRL,
				   TEGRA30_I2S_CTRL_XFER_EN_RX,
				   TEGRA30_I2S_CTRL_XFER_EN_RX);
}

static void tegra30_i2s_stop_capture(struct tegra30_i2s *i2s)
{
	int dcnt = 10;
	if (!i2s->is_call_mode_rec && (i2s->capture_ref_count == 1)) {
		tegra30_ahub_disable_rx_fifo(i2s->capture_fifo_cif);
		regmap_update_bits(i2s->regmap, TEGRA30_I2S_CTRL,
				   TEGRA30_I2S_CTRL_XFER_EN_RX, 0);
		while (tegra30_ahub_rx_fifo_is_enabled(i2s->id) && dcnt--)
			udelay(100);
	}

	while (!tegra30_ahub_rx_fifo_is_empty(i2s->id) && dcnt--)
		udelay(100);
}

static int tegra30_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra30_i2s_start_playback(i2s);
		else
			tegra30_i2s_start_capture(i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra30_i2s_stop_playback(i2s);
		else
			tegra30_i2s_stop_capture(i2s);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra30_i2s_probe(struct snd_soc_dai *dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	dai->capture_dma_data = &i2s->capture_dma_data;
	dai->playback_dma_data = &i2s->playback_dma_data;

	/* Default values for DSP mode */
	i2s->dsp_config.num_slots = 1;
	i2s->dsp_config.slot_width = 2;
	i2s->dsp_config.tx_mask = 1;
	i2s->dsp_config.rx_mask = 1;
	i2s->dsp_config.rx_data_offset = 1;
	i2s->dsp_config.tx_data_offset = 1;


	return 0;
}

int tegra30_i2s_set_tdm_slot(struct snd_soc_dai *cpu_dai,
							unsigned int tx_mask,
							unsigned int rx_mask,
							int slots,
							int slot_width)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	i2s->dsp_config.num_slots = slots;
	i2s->dsp_config.slot_width = slot_width;
	i2s->dsp_config.tx_mask = tx_mask;
	i2s->dsp_config.rx_mask = rx_mask;
	i2s->dsp_config.rx_data_offset = 0;
	i2s->dsp_config.tx_data_offset = 0;

	return 0;
}

#ifdef CONFIG_PM
int tegra30_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	int ret = 0;

	//tegra30_ahub_apbif_resume();

	if (i2s->dam_ch_refcount)
		ret = tegra30_dam_resume(i2s->dam_ifc);

	return ret;
}
#else
#define tegra30_i2s_resume NULL
#endif

static struct snd_soc_dai_ops tegra30_i2s_dai_ops = {
	.startup	= tegra30_i2s_startup,
	.shutdown	= tegra30_i2s_shutdown,
	.set_fmt	= tegra30_i2s_set_fmt,
	.hw_params	= tegra30_i2s_hw_params,
	.trigger	= tegra30_i2s_trigger,
	.set_tdm_slot = tegra30_i2s_set_tdm_slot,
};

static const struct snd_soc_dai_driver tegra30_i2s_dai_template = {
	.probe = tegra30_i2s_probe,
	.resume = tegra30_i2s_resume,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			   SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			   SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &tegra30_i2s_dai_ops,
	.symmetric_rates = 1,
};

static bool tegra30_i2s_wr_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA30_I2S_CTRL:
	case TEGRA30_I2S_TIMING:
	case TEGRA30_I2S_OFFSET:
	case TEGRA30_I2S_CH_CTRL:
	case TEGRA30_I2S_SLOT_CTRL:
	case TEGRA30_I2S_CIF_RX_CTRL:
	case TEGRA30_I2S_CIF_TX_CTRL:
	case TEGRA30_I2S_FLOWCTL:
	case TEGRA30_I2S_TX_STEP:
	case TEGRA30_I2S_FLOW_STATUS:
	case TEGRA30_I2S_FLOW_TOTAL:
	case TEGRA30_I2S_FLOW_OVER:
	case TEGRA30_I2S_FLOW_UNDER:
	case TEGRA30_I2S_LCOEF_1_4_0:
	case TEGRA30_I2S_LCOEF_1_4_1:
	case TEGRA30_I2S_LCOEF_1_4_2:
	case TEGRA30_I2S_LCOEF_1_4_3:
	case TEGRA30_I2S_LCOEF_1_4_4:
	case TEGRA30_I2S_LCOEF_1_4_5:
	case TEGRA30_I2S_LCOEF_2_4_0:
	case TEGRA30_I2S_LCOEF_2_4_1:
	case TEGRA30_I2S_LCOEF_2_4_2:
		return true;
	default:
		return false;
	};
}

static bool tegra30_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA30_I2S_FLOW_STATUS:
	case TEGRA30_I2S_FLOW_TOTAL:
	case TEGRA30_I2S_FLOW_OVER:
	case TEGRA30_I2S_FLOW_UNDER:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra30_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA30_I2S_LCOEF_2_4_2,
	.writeable_reg = tegra30_i2s_wr_rd_reg,
	.readable_reg = tegra30_i2s_wr_rd_reg,
	.volatile_reg = tegra30_i2s_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int configure_baseband_i2s(struct tegra30_i2s  *i2s, int is_i2smaster,
		int i2s_mode, int channels, int rate, int bitsize, int bit_clk)
{
	u32 val;
	int i2sclock, bitcnt, ret, is_formatdsp;
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
	u32  i;
#endif

	is_formatdsp = (i2s_mode == TEGRA_DAIFMT_DSP_A) ||
					(i2s_mode == TEGRA_DAIFMT_DSP_B);

	if (bit_clk) {
		i2sclock = bit_clk;
	} else {
	    i2sclock = rate * channels * bitsize * 2;
	    /* additional 8 for baseband */
		if (is_formatdsp)
			i2sclock *= 8;
	}

	if (is_i2smaster) {
		ret = clk_set_parent(i2s->clk_i2s, i2s->clk_pll_a_out0);
		if (ret) {
			pr_err("Can't set parent of I2S clock\n");
			return ret;
		}

		ret = clk_set_rate(i2s->clk_i2s, i2sclock);
		if (ret) {
			pr_err("Can't set I2S clock rate: %d\n", ret);
			return ret;
		}
	} else {
		ret = clk_set_rate(i2s->clk_i2s_sync, i2sclock);
		if (ret) {
			pr_err("Can't set I2S sync clock rate\n");
			return ret;
		}

		ret = clk_set_rate(i2s->clk_audio_2x, i2sclock);
		if (ret) {
			pr_err("Can't set I2S sync clock rate\n");
			return ret;
		}

		ret = clk_set_parent(i2s->clk_i2s, i2s->clk_audio_2x);
		if (ret) {
			pr_err("Can't set parent of audio2x clock\n");
			return ret;
		}
	}

	i2s->reg_ctrl &= ~(TEGRA30_I2S_CTRL_FRAME_FORMAT_MASK |
					TEGRA30_I2S_CTRL_LRCK_MASK |
					TEGRA30_I2S_CTRL_MASTER_ENABLE);
	i2s->reg_ch_ctrl &= ~TEGRA30_I2S_CH_CTRL_EGDE_CTRL_MASK;

	i2s->reg_ctrl |= TEGRA30_I2S_CTRL_BIT_SIZE_16;

	if (is_i2smaster)
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_MASTER_ENABLE;

	if (i2s_mode == TEGRA_DAIFMT_DSP_A) {
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_R_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_NEG_EDGE;
	} else if (i2s_mode == TEGRA_DAIFMT_DSP_B) {
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_R_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_POS_EDGE;
	} else {
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_FRAME_FORMAT_LRCK;
		i2s->reg_ctrl |= TEGRA30_I2S_CTRL_LRCK_L_LOW;
		i2s->reg_ch_ctrl |= TEGRA30_I2S_CH_CTRL_EGDE_CTRL_POS_EDGE;
	}

	tegra30_i2s_write(i2s, TEGRA30_I2S_CH_CTRL, i2s->reg_ch_ctrl);

#ifndef CONFIG_ARCH_TEGRA_3x_SOC
	val = 0;
	for (i = 0; i < channels; i++)
		val |= (1 << i);

	val |= val <<
	  TEGRA30_I2S_SLOT_CTRL2_TX_SLOT_ENABLES_SHIFT;
	val |= val <<
	  TEGRA30_I2S_SLOT_CTRL2_RX_SLOT_ENABLES_SHIFT;
	tegra30_i2s_write(i2s, TEGRA30_I2S_SLOT_CTRL2, val);

	val = 0;
	if (i2s->reg_ctrl & TEGRA30_I2S_CTRL_FRAME_FORMAT_FSYNC)
		val = channels  - 1;

	tegra30_i2s_write(i2s, TEGRA30_I2S_SLOT_CTRL, val);
#else
	val = tegra30_i2s_read(i2s, TEGRA30_I2S_SLOT_CTRL);
	val &= ~(TEGRA30_I2S_SLOT_CTRL_TX_SLOT_ENABLES_MASK |
		TEGRA30_I2S_SLOT_CTRL_RX_SLOT_ENABLES_MASK);
	val |= (1 << TEGRA30_I2S_SLOT_CTRL_TX_SLOT_ENABLES_SHIFT |
		1 << TEGRA30_I2S_SLOT_CTRL_RX_SLOT_ENABLES_SHIFT);
	tegra30_i2s_write(i2s, TEGRA30_I2S_SLOT_CTRL, val);
#endif

	val = (1 << TEGRA30_I2S_OFFSET_RX_DATA_OFFSET_SHIFT) |
	      (1 << TEGRA30_I2S_OFFSET_TX_DATA_OFFSET_SHIFT);
	tegra30_i2s_write(i2s, TEGRA30_I2S_OFFSET, val);

	if (is_formatdsp) {
		bitcnt = (i2sclock/rate) - 1;
		val = bitcnt << TEGRA30_I2S_TIMING_CHANNEL_BIT_COUNT_SHIFT;
		if (i2sclock % (rate))
			val |= TEGRA30_I2S_TIMING_NON_SYM_ENABLE;
	} else {
		bitcnt = (i2sclock/(2*rate)) - 1;
		val = bitcnt << TEGRA30_I2S_TIMING_CHANNEL_BIT_COUNT_SHIFT;
		if (i2sclock % (2*rate))
			val |= TEGRA30_I2S_TIMING_NON_SYM_ENABLE;
	}

	tegra30_i2s_write(i2s, TEGRA30_I2S_TIMING, val);

	/* configure the i2s cif*/
	val = (0 << TEGRA30_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
	      ((channels - 1) << TEGRA30_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
	      ((channels - 1) << TEGRA30_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
	      TEGRA30_AUDIOCIF_CTRL_AUDIO_BITS_16 |
	      TEGRA30_AUDIOCIF_CTRL_CLIENT_BITS_16;
	val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_RX;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CIF_RX_CTRL, val);

	val &= ~TEGRA30_AUDIOCIF_CTRL_DIRECTION_MASK;
	val |= TEGRA30_AUDIOCIF_CTRL_DIRECTION_TX;
	tegra30_i2s_write(i2s, TEGRA30_I2S_CIF_TX_CTRL, val);

	return 0;
}

static int configure_dam(struct tegra30_i2s  *i2s, int out_channel,
		int out_rate, int out_bitsize, int in_channels,
		int in_rate, int in_bitsize)
{

	if (!i2s->dam_ch_refcount)
		i2s->dam_ifc = tegra30_dam_allocate_controller();

	if (i2s->dam_ifc < 0)
		return -ENOENT;

	tegra30_dam_allocate_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN0_SRC);
	i2s->dam_ch_refcount++;
	tegra30_dam_enable_clock(i2s->dam_ifc);
	tegra30_dam_set_samplerate(i2s->dam_ifc, TEGRA30_DAM_CHOUT, out_rate);
	tegra30_dam_set_samplerate(i2s->dam_ifc, TEGRA30_DAM_CHIN0_SRC,
				in_rate);
	tegra30_dam_set_gain(i2s->dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
	tegra30_dam_set_acif(i2s->dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			in_channels, in_bitsize, 1, 32);
	tegra30_dam_set_acif(i2s->dam_ifc, TEGRA30_DAM_CHOUT,
			out_channel, out_bitsize, out_channel, 32);
#else
	tegra30_dam_set_acif(i2s->dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			in_channels, in_bitsize, 1, 16);
	tegra30_dam_set_acif(i2s->dam_ifc, TEGRA30_DAM_CHOUT,
			out_channel, out_bitsize, out_channel, out_bitsize);
#endif

#ifndef CONFIG_ARCH_TEGRA_3x_SOC
	if (in_rate != out_rate) {
		tegra30_dam_write_coeff_ram(i2s->dam_ifc, in_rate, out_rate);
		tegra30_dam_set_farrow_param(i2s->dam_ifc, in_rate, out_rate);
		tegra30_dam_set_biquad_fixed_coef(i2s->dam_ifc);
		tegra30_dam_enable_coeff_ram(i2s->dam_ifc);
		tegra30_dam_set_filter_stages(i2s->dam_ifc, in_rate, out_rate);
	} else {
		tegra30_dam_enable_stereo_mixing(i2s->dam_ifc);
	}
#endif

	return 0;
}


int tegra30_make_voice_call_connections(struct codec_config *codec_info,
				struct codec_config *bb_info,
				int uses_voice_codec)
{
	struct tegra30_i2s  *codec_i2s;
	struct tegra30_i2s  *bb_i2s;
	int reg;

	codec_i2s = &i2scont[codec_info->i2s_id];
	bb_i2s = &i2scont[bb_info->i2s_id];

	/* increment the codec i2s playback ref count */
	codec_i2s->playback_ref_count++;
	bb_i2s->playback_ref_count++;
	codec_i2s->capture_ref_count++;
	bb_i2s->capture_ref_count++;

	/* Make sure i2s is disabled during the configiration */
	tegra30_i2s_enable_clocks(codec_i2s);
	reg = codec_i2s->reg_ctrl;
	reg &= ~TEGRA30_I2S_CTRL_TX_FLOWCTL_EN;
	reg &= ~TEGRA30_I2S_CTRL_XFER_EN_TX;
	reg &= ~TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(codec_i2s, TEGRA30_I2S_CTRL,
		codec_i2s->reg_ctrl);
	tegra30_i2s_disable_clocks(codec_i2s);

	tegra30_i2s_enable_clocks(bb_i2s);
	reg = bb_i2s->reg_ctrl;
	reg &= ~TEGRA30_I2S_CTRL_TX_FLOWCTL_EN;
	reg &= ~TEGRA30_I2S_CTRL_XFER_EN_TX;
	reg &= ~TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(bb_i2s, TEGRA30_I2S_CTRL,
		bb_i2s->reg_ctrl);
	tegra30_i2s_disable_clocks(bb_i2s);

	msleep(20);

	/*Configure codec i2s*/
	configure_baseband_i2s(codec_i2s, codec_info->is_i2smaster,
		codec_info->i2s_mode, codec_info->channels,
		codec_info->rate, codec_info->bitsize, codec_info->bit_clk);

	/*Configure bb i2s*/
	configure_baseband_i2s(bb_i2s, bb_info->is_i2smaster,
		bb_info->i2s_mode, bb_info->channels,
		bb_info->rate, bb_info->bitsize, bb_info->bit_clk);

	if (uses_voice_codec) {
		/* The following two lines are a hack */
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_APBIF_RX0);
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0+1);

		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0 +
			    bb_info->i2s_id, TEGRA30_AHUB_TXCIF_I2S0_TX0 +
			    codec_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0 +
			    codec_info->i2s_id, TEGRA30_AHUB_TXCIF_I2S0_TX0 +
			    bb_info->i2s_id);
		if (!(codec_info->is_i2smaster && bb_info->is_i2smaster)) {
			tegra30_i2s_write(codec_i2s, TEGRA30_I2S_FLOWCTL,
				TEGRA30_I2S_FILTER_QUAD);
			tegra30_i2s_write(bb_i2s, TEGRA30_I2S_FLOWCTL,
				TEGRA30_I2S_FILTER_QUAD);
			tegra30_i2s_write(codec_i2s, TEGRA30_I2S_TX_STEP, 4);
			tegra30_i2s_write(bb_i2s, TEGRA30_I2S_TX_STEP, 4);
			codec_i2s->reg_ctrl |= TEGRA30_I2S_CTRL_TX_FLOWCTL_EN;
			bb_i2s->reg_ctrl |= TEGRA30_I2S_CTRL_TX_FLOWCTL_EN;
		}
	} else {

		/*configure codec dam*/
		configure_dam(codec_i2s, codec_info->channels,
		   codec_info->rate, codec_info->bitsize, bb_info->channels,
		   bb_info->rate, bb_info->bitsize);

		/*configure bb dam*/
		configure_dam(bb_i2s, bb_info->channels,
			bb_info->rate, bb_info->bitsize, codec_info->channels,
			codec_info->rate, codec_info->bitsize);

		/*make ahub connections*/

		/*if this is the only user of i2s tx, make i2s rx connection*/
		if (codec_i2s->playback_ref_count == 1) {
			tegra30_ahub_set_rx_cif_source(
			  TEGRA30_AHUB_RXCIF_I2S0_RX0 + codec_info->i2s_id,
			  TEGRA30_AHUB_TXCIF_DAM0_TX0 + codec_i2s->dam_ifc);
		}

		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0 +
				bb_info->i2s_id, TEGRA30_AHUB_TXCIF_DAM0_TX0 +
				bb_i2s->dam_ifc);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(codec_i2s->dam_ifc*2), TEGRA30_AHUB_TXCIF_I2S0_TX0 +
			bb_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(bb_i2s->dam_ifc*2), TEGRA30_AHUB_TXCIF_I2S0_TX0 +
			codec_info->i2s_id);

		/*enable dam and i2s*/
		tegra30_dam_enable(codec_i2s->dam_ifc, TEGRA30_DAM_ENABLE,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_enable(bb_i2s->dam_ifc, TEGRA30_DAM_ENABLE,
			TEGRA30_DAM_CHIN0_SRC);
	}

	msleep(20);

	codec_i2s->reg_ctrl |= TEGRA30_I2S_CTRL_XFER_EN_TX;
	codec_i2s->reg_ctrl |= TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(codec_i2s, TEGRA30_I2S_CTRL,
		codec_i2s->reg_ctrl);

	msleep(20);

	bb_i2s->reg_ctrl |= TEGRA30_I2S_CTRL_XFER_EN_TX;
	bb_i2s->reg_ctrl |= TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(bb_i2s, TEGRA30_I2S_CTRL,
		bb_i2s->reg_ctrl);

	return 0;
}

int tegra30_break_voice_call_connections(struct codec_config *codec_info,
				struct codec_config *bb_info,
				int uses_voice_codec)
{
	struct tegra30_i2s  *codec_i2s;
	struct tegra30_i2s  *bb_i2s;
	int dcnt = 10;

	codec_i2s = &i2scont[codec_info->i2s_id];
	bb_i2s = &i2scont[bb_info->i2s_id];

	/*Disable Codec I2S RX (TX to ahub)*/
	if (codec_i2s->capture_ref_count == 1)
		codec_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_XFER_EN_RX;

	tegra30_i2s_write(codec_i2s, TEGRA30_I2S_CTRL, codec_i2s->reg_ctrl);

	while (!tegra30_ahub_rx_fifo_is_empty(codec_i2s->id) && dcnt--)
		udelay(100);

	dcnt = 10;

	/*Disable baseband I2S TX (RX from ahub)*/
	bb_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_XFER_EN_TX;
	tegra30_i2s_write(bb_i2s, TEGRA30_I2S_CTRL, bb_i2s->reg_ctrl);

	while (!tegra30_ahub_tx_fifo_is_empty(bb_i2s->id) && dcnt--)
		udelay(100);

	dcnt = 10;

	/*Disable baseband I2S RX (TX to ahub)*/
	bb_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_XFER_EN_RX;
	tegra30_i2s_write(bb_i2s, TEGRA30_I2S_CTRL, bb_i2s->reg_ctrl);

	while (!tegra30_ahub_rx_fifo_is_empty(bb_i2s->id) && dcnt--)
		udelay(100);

	dcnt = 10;

	/*Disable Codec I2S TX (RX from ahub)*/
	if (codec_i2s->playback_ref_count == 1)
			codec_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_XFER_EN_TX;

	tegra30_i2s_write(codec_i2s, TEGRA30_I2S_CTRL, codec_i2s->reg_ctrl);

	while (!tegra30_ahub_tx_fifo_is_empty(codec_i2s->id) && dcnt--)
		udelay(100);

	dcnt = 10;

	if (uses_voice_codec) {
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0 +
			    bb_info->i2s_id);
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0 +
			    codec_info->i2s_id);
	} else {

		/*Disable baseband DAM*/
		tegra30_dam_enable(bb_i2s->dam_ifc, TEGRA30_DAM_DISABLE,
				TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_free_channel(bb_i2s->dam_ifc,
				TEGRA30_DAM_CHIN0_SRC);
		bb_i2s->dam_ch_refcount--;
		if (!bb_i2s->dam_ch_refcount)
			tegra30_dam_free_controller(bb_i2s->dam_ifc);

		/*Disable Codec DAM*/
		tegra30_dam_enable(codec_i2s->dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_free_channel(codec_i2s->dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		codec_i2s->dam_ch_refcount--;
		if (!codec_i2s->dam_ch_refcount)
			tegra30_dam_free_controller(codec_i2s->dam_ifc);

		/* Disconnect the ahub connections */
		/* If this is the only user of i2s tx then break ahub
		i2s rx connection */
		if (codec_i2s->playback_ref_count == 1)
			tegra30_ahub_unset_rx_cif_source(
			   TEGRA30_AHUB_RXCIF_I2S0_RX0 + codec_info->i2s_id);

		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_I2S0_RX0
					+ bb_info->i2s_id);
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0
					+ (codec_i2s->dam_ifc*2));
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0
					+ (bb_i2s->dam_ifc*2));

		tegra30_dam_disable_clock(codec_i2s->dam_ifc);
		tegra30_dam_disable_clock(bb_i2s->dam_ifc);
	}

	/* Decrement the codec and bb i2s playback ref count */
	codec_i2s->playback_ref_count--;
	bb_i2s->playback_ref_count--;
	codec_i2s->capture_ref_count--;
	bb_i2s->capture_ref_count--;

	/* Soft reset */
	tegra30_i2s_write(codec_i2s, TEGRA30_I2S_CTRL,
		codec_i2s->reg_ctrl | TEGRA30_I2S_CTRL_SOFT_RESET);
	tegra30_i2s_write(bb_i2s, TEGRA30_I2S_CTRL,
		bb_i2s->reg_ctrl | TEGRA30_I2S_CTRL_SOFT_RESET);

	codec_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_TX_FLOWCTL_EN;
	bb_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_TX_FLOWCTL_EN;
	codec_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_SOFT_RESET;
	bb_i2s->reg_ctrl &= ~TEGRA30_I2S_CTRL_SOFT_RESET;

	while ((tegra30_i2s_read(codec_i2s, TEGRA30_I2S_CTRL) &
			TEGRA30_I2S_CTRL_SOFT_RESET)  && dcnt--)
		udelay(100);
	dcnt = 10;
	while ((tegra30_i2s_read(bb_i2s, TEGRA30_I2S_CTRL) &
			TEGRA30_I2S_CTRL_SOFT_RESET)  && dcnt--)
		udelay(100);

	return 0;
}

static int tegra30_i2s_platform_probe(struct platform_device *pdev)
{
	struct tegra30_i2s *i2s;
	u32 cif_ids[2];
	struct resource *mem, *memregion;
	void __iomem *regs;
	int ret;

	/* This needs to be updated for DT */
	return -ENODEV;

	i2s = &i2scont[pdev->id];
	dev_set_drvdata(&pdev->dev, i2s);
	i2s->id = pdev->id;

	ret = of_property_read_u32_array(pdev->dev.of_node,
					 "nvidia,ahub-cif-ids", cif_ids,
					 ARRAY_SIZE(cif_ids));
	if (ret < 0)
		goto err;

	i2s->playback_i2s_cif = cif_ids[0];
	i2s->capture_i2s_cif = cif_ids[1];

	i2s->clk_i2s = clk_get(&pdev->dev, "i2s");
	if (IS_ERR(i2s->clk_i2s)) {
		dev_err(&pdev->dev, "Can't retrieve i2s clock\n");
		ret = PTR_ERR(i2s->clk_i2s);
		goto err;
	}
	i2s->clk_i2s_sync = clk_get(&pdev->dev, "ext_audio_sync");
	if (IS_ERR(i2s->clk_i2s_sync)) {
		dev_err(&pdev->dev, "Can't retrieve i2s_sync clock\n");
		ret = PTR_ERR(i2s->clk_i2s_sync);
		goto err_i2s_clk_put;
	}

	i2s->clk_audio_2x = clk_get(&pdev->dev, "audio_sync_2x");
	if (IS_ERR(i2s->clk_audio_2x)) {
		dev_err(&pdev->dev, "Can't retrieve audio 2x clock\n");
		ret = PTR_ERR(i2s->clk_audio_2x);
		goto err_i2s_sync_clk_put;
	}

	i2s->clk_pll_a_out0 = clk_get_sys(NULL, "pll_a_out0");
	if (IS_ERR(i2s->clk_pll_a_out0)) {
		dev_err(&pdev->dev, "Can't retrieve pll_a_out0 clock\n");
		ret = PTR_ERR(i2s->clk_pll_a_out0);
		goto err_audio_2x_clk_put;
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
					    &tegra30_i2s_regmap_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(i2s->regmap);
		goto err_pll_a_out0_clk_put;
	}
	regcache_cache_only(i2s->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra30_i2s_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = snd_soc_register_dai(&pdev->dev, NULL);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI: %d\n", ret);
		ret = -ENOMEM;
		goto err_suspend;
	}

	ret = tegra_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_dai;
	}

	return 0;

err_unregister_dai:
	snd_soc_unregister_dai(&pdev->dev);
err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra30_i2s_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_pll_a_out0_clk_put:
	clk_put(i2s->clk_pll_a_out0);
err_audio_2x_clk_put:
	clk_put(i2s->clk_audio_2x);
err_i2s_sync_clk_put:
	clk_put(i2s->clk_i2s_sync);
err_i2s_clk_put:
	clk_put(i2s->clk_i2s);
err:
	return ret;
}

static int tegra30_i2s_platform_remove(struct platform_device *pdev)
{
	struct tegra30_i2s *i2s = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra30_i2s_runtime_suspend(&pdev->dev);

	tegra_pcm_platform_unregister(&pdev->dev);
	snd_soc_unregister_dai(&pdev->dev);

	clk_put(i2s->clk_pll_a_out0);
	clk_put(i2s->clk_audio_2x);
	clk_put(i2s->clk_i2s_sync);
	clk_put(i2s->clk_i2s);

	return 0;
}

static const struct of_device_id tegra30_i2s_of_match[] = {
	{ .compatible = "nvidia,tegra30-i2s", },
	{},
};

static const struct dev_pm_ops tegra30_i2s_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra30_i2s_runtime_suspend,
			   tegra30_i2s_runtime_resume, NULL)
};

static struct platform_driver tegra30_i2s_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra30_i2s_of_match,
		.pm = &tegra30_i2s_pm_ops,
	},
	.probe = tegra30_i2s_platform_probe,
	.remove = tegra30_i2s_platform_remove,
};
module_platform_driver(tegra30_i2s_driver);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra30 I2S ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra30_i2s_of_match);
