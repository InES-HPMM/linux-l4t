/*
 * ak4618.c - AK4618 Audio Codec driver supporting AK4618
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "ak4618.h"

/* codec private data */
struct ak4618_priv {
	struct regmap *regmap;
	int sysclk;
};

/*
 * AK4618 volume etc. controls
 */
static const DECLARE_TLV_DB_SCALE(ak4618_dac_volume_tlv, -12700, 50, 1);

/*
 * Mic amp gain control:
 * 0 dB and from 15 to 33 dB in 3 dB steps
 * Displayed gain will be 21 dB
 */
static DECLARE_TLV_DB_SCALE(ak4618_mic_amp_tlv, 0, 300, 0);

static const struct snd_kcontrol_new ak4618_snd_controls[] = {
	/* DAC volume control */
	SOC_DOUBLE_R_TLV("DAC1 Volume", AK4618_DAC1L_VOLUME,
			AK4618_DAC1R_VOLUME, 0, 0xFF, 1, ak4618_dac_volume_tlv),
	SOC_DOUBLE_R_TLV("DAC2 Volume", AK4618_DAC2L_VOLUME,
			AK4618_DAC2R_VOLUME, 0, 0xFF, 1, ak4618_dac_volume_tlv),
	SOC_DOUBLE_R_TLV("DAC3 Volume", AK4618_DAC3L_VOLUME,
			AK4618_DAC3R_VOLUME, 0, 0xFF, 1, ak4618_dac_volume_tlv),
	SOC_DOUBLE_R_TLV("DAC4 Volume", AK4618_DAC4L_VOLUME,
			AK4618_DAC4R_VOLUME, 0, 0xFF, 1, ak4618_dac_volume_tlv),
	SOC_DOUBLE_R_TLV("DAC5 Volume", AK4618_DAC5L_VOLUME,
			AK4618_DAC5R_VOLUME, 0, 0xFF, 1, ak4618_dac_volume_tlv),
	SOC_DOUBLE_R_TLV("DAC6 Volume", AK4618_DAC6L_VOLUME,
			AK4618_DAC6R_VOLUME, 0, 0xFF, 1, ak4618_dac_volume_tlv),

	/* Mic amplifier gain */
	SOC_SINGLE_TLV("MIC1 Volume", AK4618_MICROPHONE_GAIN_0,
			0, 0x7, 0, ak4618_mic_amp_tlv),
	SOC_SINGLE_TLV("MIC2 Volume", AK4618_MICROPHONE_GAIN_0,
			3, 0x7, 0, ak4618_mic_amp_tlv),
	SOC_SINGLE_TLV("MIC3 Volume", AK4618_MICROPHONE_GAIN_1,
			0, 0x7, 0, ak4618_mic_amp_tlv),
	SOC_SINGLE_TLV("MIC4 Volume", AK4618_MICROPHONE_GAIN_1,
			3, 0x7, 0, ak4618_mic_amp_tlv),
	SOC_SINGLE_TLV("MIC5 Volume", AK4618_MICROPHONE_GAIN_2,
			0, 0x7, 0, ak4618_mic_amp_tlv),
	SOC_SINGLE_TLV("MIC6 Volume", AK4618_MICROPHONE_GAIN_2,
			3, 0x7, 0, ak4618_mic_amp_tlv),
};

static const struct snd_soc_dapm_widget ak4618_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIFIN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIFOUT", "Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC("DAC", NULL, AK4618_POWER_MANAGEMENT_1, 4, 0),
	SND_SOC_DAPM_ADC("ADC", NULL, AK4618_POWER_MANAGEMENT_1, 5, 0),

	/* Mic Bias */
	SND_SOC_DAPM_SUPPLY("MICBIAS", AK4618_POWER_MANAGEMENT_1, 6, 0,
			NULL, 0),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("DACOUT1"),
	SND_SOC_DAPM_OUTPUT("DACOUT2"),
	SND_SOC_DAPM_OUTPUT("DACOUT3"),
	SND_SOC_DAPM_OUTPUT("DACOUT4"),
	SND_SOC_DAPM_OUTPUT("DACOUT5"),
	SND_SOC_DAPM_OUTPUT("DACOUT6"),

	/* Inputs */
	SND_SOC_DAPM_INPUT("IN1"),
	SND_SOC_DAPM_INPUT("IN2"),
	SND_SOC_DAPM_INPUT("IN3"),
	SND_SOC_DAPM_INPUT("IN4"),
	SND_SOC_DAPM_INPUT("IN5"),
	SND_SOC_DAPM_INPUT("IN6"),
};

static const struct snd_soc_dapm_route audio_paths[] = {
	{ "ADC", NULL, "IN1" },
	{ "ADC", NULL, "IN2" },
	{ "ADC", NULL, "IN3" },
	{ "ADC", NULL, "IN4" },
	{ "ADC", NULL, "IN5" },
	{ "ADC", NULL, "IN6" },
	{ "AIFOUT", NULL, "ADC"},

	{ "DAC", NULL, "AIFIN"},
	{ "DACOUT1", NULL, "DAC" },
	{ "DACOUT2", NULL, "DAC" },
	{ "DACOUT3", NULL, "DAC" },
	{ "DACOUT4", NULL, "DAC" },
	{ "DACOUT5", NULL, "DAC" },
	{ "DACOUT6", NULL, "DAC" },
};

/*
 * DAI ops entries
 */

static int ak4618_mute(struct snd_soc_dai *dai, int mute)
{
	struct ak4618_priv *ak4618 = snd_soc_codec_get_drvdata(dai->codec);

	regmap_update_bits(ak4618->regmap, AK4618_SOFT_MUTE,
			AK4618_SOFT_MUTE_MASK,
			mute ? AK4618_SOFT_MUTE_ENABLE :
			AK4618_SOFT_UNMUTE_ENABLE);

	return 0;
}

static int ak4618_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
			       unsigned int rx_mask, int slots, int width)
{
	struct ak4618_priv *ak4618 = snd_soc_codec_get_drvdata(dai->codec);
	unsigned int dac_power, adc_power, tdm_format;

	dac_power = 0;
	adc_power = 0;
	tdm_format = 0;

	switch (slots) {
	case 12:
		dac_power |= AK4618_PM_DAC6_ON;
		/* fall-through */
	case 10:
		dac_power |= AK4618_PM_DAC5_ON;
		/* fall-through */
	case 8:
		dac_power |= AK4618_PM_DAC4_ON;
		/* fall-through */
	case 6:
		dac_power |= AK4618_PM_DAC3_ON;
		adc_power |= AK4618_PM_ADC56_ON;
		/* fall-through */
	case 4:
		dac_power |= AK4618_PM_DAC2_ON;
		adc_power |= AK4618_PM_ADC34_ON;
		/* fall-through */
	case 2:
		dac_power |= AK4618_PM_DAC1_ON;
		adc_power |= AK4618_PM_ADC12_ON;
		break;
	default:
		return -EINVAL;
	}

	if (slots > 8)
		tdm_format = AK4618_FMT_TDM_512;
	else if (slots > 4)
		tdm_format = AK4618_FMT_TDM_256;
	else
		tdm_format = AK4618_FMT_TDM_128;

	regmap_update_bits(ak4618->regmap, AK4618_POWER_MANAGEMENT_2,
		AK4618_PM_DAC_ON_MASK, dac_power);

	regmap_update_bits(ak4618->regmap, AK4618_POWER_MANAGEMENT_1,
		AK4618_PM_ADC_ON_MASK, adc_power);

	regmap_update_bits(ak4618->regmap, AK4618_AUDIO_INTERFACE_FORMAT,
		AK4618_FMT_TDM_FORMAT_MASK, tdm_format);

	return 0;
}

static int ak4618_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct ak4618_priv *ak4618;
	unsigned int format = 0;

	ak4618 = snd_soc_codec_get_drvdata(codec_dai->codec);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
		format = AK4618_FMT_I2S;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(ak4618->regmap, AK4618_AUDIO_INTERFACE_FORMAT,
			AK4618_FMT_FORMAT_MASK, format);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM: /* codec clk & frm master */
		regmap_update_bits(ak4618->regmap, AK4618_POWER_MANAGEMENT_1,
		AK4618_MODE_SELECT_MASK, AK4618_MASTER_MODE);

		regmap_update_bits(ak4618->regmap, AK4618_SYSTEM_CLOCK,
			AK4618_SYS_CLK_AUTO_MASK, AK4618_SYS_CLK_MANUAL);

		break;
	case SND_SOC_DAIFMT_CBS_CFS: /* codec clk & frm slave */
		regmap_update_bits(ak4618->regmap, AK4618_POWER_MANAGEMENT_1,
			AK4618_MODE_SELECT_MASK, AK4618_SLAVE_MODE);

		regmap_update_bits(ak4618->regmap, AK4618_SYSTEM_CLOCK,
			AK4618_SYS_CLK_AUTO_MASK, AK4618_SYS_CLK_AUTO);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ak4618_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ak4618_priv *ak4618 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 16384000:
	case 18432000:
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		ak4618->sysclk = freq;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ak4618_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	int srate = 0, system_clk = 0;
	struct snd_soc_codec *codec = dai->codec;
	struct ak4618_priv *ak4618 = snd_soc_codec_get_drvdata(codec);

	srate = params_rate(params);
	switch (ak4618->sysclk / srate) {
	case 256:
		system_clk = AK4618_SYS_CLK_MCLK_SEL_256;
		break;
	case 384:
		system_clk = AK4618_SYS_CLK_MCLK_SEL_384;
		break;
	case 512:
		system_clk = AK4618_SYS_CLK_MCLK_SEL_512;
		break;
	default:
		return -EINVAL;
	}

	if (srate > 96000)
		system_clk |= AK4618_SYS_CLK_SAMPLING_RATE_QUAD;
	else if (srate > 48000)
		system_clk |= AK4618_SYS_CLK_SAMPLING_RATE_DOUBLE;
	else
		system_clk |= AK4618_SYS_CLK_SAMPLING_RATE_NORMAL;

	regmap_update_bits(ak4618->regmap, AK4618_SYSTEM_CLOCK,
			    AK4618_SYS_CLK_MASK, system_clk);

	return 0;
}

static const struct snd_soc_dai_ops ak4618_dai_ops = {
	.hw_params = ak4618_hw_params,
	.digital_mute = ak4618_mute,
	.set_tdm_slot = ak4618_set_tdm_slot,
	.set_sysclk = ak4618_set_dai_sysclk,
	.set_fmt = ak4618_set_dai_fmt,
};

/* codec DAI instance */
static struct snd_soc_dai_driver ak4618_dai = {
	.name = "ak4618-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 16,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &ak4618_dai_ops,
};

static int ak4618_probe(struct snd_soc_codec *codec)
{
	struct ak4618_priv *ak4618 = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = ak4618->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 0, 0, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "failed to set cache I/O: %d\n", ret);
		return ret;
	}

	/* default setting for ak4618 */
	/* release reset, power down dac, adc, slave mode, power down adc3-6 */
	regmap_write(ak4618->regmap, AK4618_POWER_MANAGEMENT_1, 0x03);
	/* power down dac2-6 */
	regmap_write(ak4618->regmap, AK4618_POWER_MANAGEMENT_2, 0x01);
	/* Master Clock Frequency Auto Setting Mode Enable for slave mode */
	regmap_write(ak4618->regmap, AK4618_SYSTEM_CLOCK, 0x81);
	/* stereo mode */
	regmap_write(ak4618->regmap, AK4618_AUDIO_INTERFACE_FORMAT, 0x04);
	/* MIC amp */
	regmap_write(ak4618->regmap, AK4618_MICROPHONE_GAIN_0, 0x00);

	return ret;
}

static struct snd_soc_codec_driver soc_codec_dev_ak4618 = {
	.probe = ak4618_probe,
	.controls = ak4618_snd_controls,
	.num_controls = ARRAY_SIZE(ak4618_snd_controls),
	.dapm_widgets = ak4618_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4618_dapm_widgets),
	.dapm_routes = audio_paths,
	.num_dapm_routes = ARRAY_SIZE(audio_paths),
};

static const struct regmap_config ak4618_i2c_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,
	.max_register = AK4618_NUM_REGS - 1,
};

static int ak4618_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct ak4618_priv *ak4618;

	ak4618 = devm_kzalloc(&client->dev, sizeof(struct ak4618_priv),
			      GFP_KERNEL);
	if (ak4618 == NULL)
		return -ENOMEM;

	ak4618->regmap = devm_regmap_init_i2c(client,
						&ak4618_i2c_regmap_config);
	if (IS_ERR(ak4618->regmap))
		return PTR_ERR(ak4618->regmap);

	i2c_set_clientdata(client, ak4618);

	return snd_soc_register_codec(&client->dev, &soc_codec_dev_ak4618,
			&ak4618_dai, 1);
}

static int ak4618_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id ak4618_id[] = {
	{ "ak4618", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4618_id);

static const struct of_device_id ak4618_of_match[] = {
	{ .compatible = "akm,ak4618", },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4618_of_match);

static struct i2c_driver ak4618_i2c_driver = {
	.driver = {
		.name = "ak4618",
		.owner = THIS_MODULE,
		.of_match_table = ak4618_of_match,
	},
	.probe    = ak4618_i2c_probe,
	.remove   = ak4618_i2c_remove,
	.id_table = ak4618_id,
};
module_i2c_driver(ak4618_i2c_driver);

MODULE_DESCRIPTION("ASoC ak4618 driver");
MODULE_AUTHOR("Songhee Baek <sbeak@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ak4618-codec");
