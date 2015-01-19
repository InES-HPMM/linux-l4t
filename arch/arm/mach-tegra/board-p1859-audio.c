/*
 * arch/arm/mach-tegra/board-p1859-audio.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <sound/soc.h>
#include <mach/tegra_vcm30t124_pdata.h>
#include <mach/board_id.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "board.h"

#define TDM_SLOT_MAP(stream_id, nth_channel, nth_byte)	\
	((stream_id << 16) | (nth_channel << 8) | nth_byte)

static unsigned int tegra_p1859_amx_slot_map[] = {
	/* jack 0 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 1, 0),
	TDM_SLOT_MAP(0, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 2, 0),
	TDM_SLOT_MAP(0, 2, 1),
	/* jack 1 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 1, 0),
	TDM_SLOT_MAP(1, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 2, 0),
	TDM_SLOT_MAP(1, 2, 1),
	/* jack 2 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 1, 0),
	TDM_SLOT_MAP(2, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 2, 0),
	TDM_SLOT_MAP(2, 2, 1),
	/* jack 3 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 1, 0),
	TDM_SLOT_MAP(3, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 2, 0),
	TDM_SLOT_MAP(3, 2, 1),
};

static unsigned int tegra_p1859_adx_slot_map[] = {
	/* jack 0 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 1, 0),
	TDM_SLOT_MAP(0, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 2, 0),
	TDM_SLOT_MAP(0, 2, 1),
	/* jack 1 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 1, 0),
	TDM_SLOT_MAP(1, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(1, 2, 0),
	TDM_SLOT_MAP(1, 2, 1),
	/* jack 2 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 1, 0),
	TDM_SLOT_MAP(2, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(2, 2, 0),
	TDM_SLOT_MAP(2, 2, 1),
	/* jack 3 */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 1, 0),
	TDM_SLOT_MAP(3, 1, 1),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(3, 2, 0),
	TDM_SLOT_MAP(3, 2, 1),
};

static struct snd_soc_pcm_stream tegra_p1859_amx_input_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[1] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[2] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[3] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
};

static struct snd_soc_pcm_stream tegra_p1859_amx_output_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 8,
		.channels_max = 8,
	},
};

static struct snd_soc_pcm_stream tegra_p1859_adx_output_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[1] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[2] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
	[3] = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
	},
};

static struct snd_soc_pcm_stream tegra_p1859_adx_input_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 8,
		.channels_max = 8,
	},
};

static const struct snd_soc_dapm_route tegra_e1860_a0x_audio_map[] = {
	{"Headphone-y",	NULL,	"y DAC1OUT"},
	{"Headphone-y", NULL,	"y DAC2OUT"},
	{"Headphone-y",	NULL,	"y DAC3OUT"},
	{"Headphone-y", NULL,	"y DAC4OUT"},
	{"y ADC1IN",	NULL,	"LineIn-y"},
	{"y ADC2IN",	NULL,	"LineIn-y"},
	{"Headphone-x",	NULL,	"x ROUT"},
	{"Headphone-x",	NULL,	"x LOUT"},
	{"x LLINEIN",	NULL,	"LineIn-x"},
	{"x RLINEIN",	NULL,	"LineIn-x"},
	{"Spdif-out",	NULL,	"z OUT"},
	{"z IN",	NULL,	"Spdif-in"},
};

static const struct snd_soc_dapm_route tegra_e1860_b00_audio_map[] = {
	{"Headphone-x",	NULL,	"x DACOUT1"},
	{"Headphone-x",	NULL,	"x DACOUT2"},
	{"Headphone-x",	NULL,	"x DACOUT3"},
	{"Headphone-x",	NULL,	"x DACOUT4"},
	{"Headphone-x",	NULL,	"x DACOUT5"},
	{"Headphone-x",	NULL,	"x DACOUT6"},
	{"Mic-x", NULL, "x MICBIAS"},
	{"x IN1",	NULL,	"Mic-x"},
	{"x IN2",	NULL,	"Mic-x"},
	{"x IN3",	NULL,	"Mic-x"},
	{"x IN4",	NULL,	"Mic-x"},
	{"x IN5",	NULL,	"Mic-x"},
	{"x IN6",	NULL,	"Mic-x"},
	{"Headphone-y",	NULL,	"y DAC1OUT"},
	{"Headphone-y", NULL,	"y DAC2OUT"},
	{"Headphone-y",	NULL,	"y DAC3OUT"},
	{"Headphone-y", NULL,	"y DAC4OUT"},
	{"y ADC1IN",	NULL,	"LineIn-y"},
	{"y ADC2IN",	NULL,	"LineIn-y"},
	{"Spdif-out",	NULL,	"z OUT"},
	{"z IN",	NULL,	"Spdif-in"},
	{"BT-out",	NULL,	"b OUT"},
	{"b IN",	NULL,	"BT-in"},
};

static const struct snd_soc_dapm_route tegra_voice_call_audio_map[] = {
	{"Headphone-x",	NULL,	"x DACOUT1"},
	{"Headphone-x",	NULL,	"x DACOUT2"},
	{"Headphone-x",	NULL,	"x DACOUT3"},
	{"Headphone-x",	NULL,	"x DACOUT4"},
	{"Headphone-x",	NULL,	"x DACOUT5"},
	{"Headphone-x",	NULL,	"x DACOUT6"},
	{"Mic-x", NULL, "x MICBIAS"},
	{"x IN1",	NULL,	"Mic-x"},
	{"x IN2",	NULL,	"Mic-x"},
	{"x IN3",	NULL,	"Mic-x"},
	{"x IN4",	NULL,	"Mic-x"},
	{"x IN5",	NULL,	"Mic-x"},
	{"x IN6",	NULL,	"Mic-x"},
	{"Spdif-out",	NULL,	"y OUT"},
	{"y IN",	NULL,	"Spdif-in"},
	{"BT-out",	NULL,	"b OUT"},
	{"b IN",	NULL,	"BT-in"},
};

static struct tegra_vcm30t124_platform_data tegra_e1860_a0x_pdata = {
	/* initialize DAI link config */
	.dai_config[0] = {
		.link_name = "wm-playback",
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "wm8731.0-001a",
		.codec_dai_name = "wm8731-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
		.bclk_ratio = 2,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dai_config[1] = {
		.link_name = "ad-playback",
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "ad193x.0-0007",
		.codec_dai_name = "ad193x-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "y",
		.bclk_ratio = 1,
		.tx_mask = (1 << 8) - 1,
		.rx_mask = (1 << 8) - 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[2] = {
		.link_name = "spdif-playback",
		.cpu_name = "tegra30-spdif",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "SPDIF",
		.codec_prefix = "z",
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dev_num = 3,
	/* initialize AMX config */
	.amx_config[0] = {
		.slot_map = tegra_p1859_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_amx_slot_map),
		.in_params = tegra_p1859_amx_input_params,
		.out_params = tegra_p1859_amx_output_params,
	},
	.amx_config[1] = {
		.slot_map = tegra_p1859_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_amx_slot_map),
		.in_params = tegra_p1859_amx_input_params,
		.out_params = tegra_p1859_amx_output_params,
	},
	.num_amx = 2,
	/* initialize ADX config */
	.adx_config[0] = {
		.slot_map = tegra_p1859_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_adx_slot_map),
		.in_params = tegra_p1859_adx_input_params,
		.out_params = tegra_p1859_adx_output_params,
	},
	.adx_config[1] = {
		.slot_map = tegra_p1859_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_adx_slot_map),
		.in_params = tegra_p1859_adx_input_params,
		.out_params = tegra_p1859_adx_output_params,
	},
	.num_adx = 2,
	/* initialize DAPM routes */
	.dapm_routes = tegra_e1860_a0x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_e1860_a0x_audio_map),
	/* sound card: tegra-wm8731-ad1937 */
	.card_name = "tegra-wm-ad",
};

static struct tegra_vcm30t124_platform_data tegra_e1860_b00_pdata = {
	/* intialize DAI link config */
	.dai_config[0] = {
		.link_name = "ak-playback",
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "ak4618.0-0010",
		.codec_dai_name = "ak4618-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
		.bclk_ratio = 1,
		.tx_mask = (1 << 8) - 1,
		.rx_mask = (1 << 8) - 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[1] = {
		.link_name = "ad-playback",
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "ad193x.0-0007",
		.codec_dai_name = "ad193x-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "y",
		.bclk_ratio = 1,
		.tx_mask = (1 << 8) - 1,
		.rx_mask = (1 << 8) - 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[2] = {
		.link_name = "spdif-playback",
		.cpu_name = "tegra30-spdif",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "SPDIF",
		.codec_prefix = "z",
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dai_config[3] = {
		.link_name = "bt-playback",
		.cpu_name = "tegra30-i2s.2",
		.codec_name = "spdif-dit.1",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S2",
		.codec_prefix = "b",
		.bclk_ratio = 4,
		.tx_mask = 1,
		.rx_mask = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_IF |
			SND_SOC_DAIFMT_CBM_CFM,
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 8000,
		.params.rate_max = 8000,
		.params.channels_min = 1,
		.params.channels_max = 1,
	},
	.dev_num = 4,
	/* initialize AMX config */
	.amx_config[0] = {
		.slot_map = tegra_p1859_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_amx_slot_map),
		.in_params = tegra_p1859_amx_input_params,
		.out_params = tegra_p1859_amx_output_params,
	},
	.amx_config[1] = {
		.slot_map = tegra_p1859_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_amx_slot_map),
		.in_params = tegra_p1859_amx_input_params,
		.out_params = tegra_p1859_amx_output_params,
	},
	.num_amx = 2,
	/* initialize ADX config */
	.adx_config[0] = {
		.slot_map = tegra_p1859_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_adx_slot_map),
		.in_params = tegra_p1859_adx_input_params,
		.out_params = tegra_p1859_adx_output_params,
	},
	.adx_config[1] = {
		.slot_map = tegra_p1859_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_adx_slot_map),
		.in_params = tegra_p1859_adx_input_params,
		.out_params = tegra_p1859_adx_output_params,
	},
	.num_adx = 2,
	/* initialize DAPM routes */
	.dapm_routes = tegra_e1860_b00_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_e1860_b00_audio_map),
	/* sound card: tegra-ak4618-ad1937 */
	.card_name = "tegra-ak-ad",
};

/* voice call related structures
 * I2S0<-DAM0<-I2S4 (DAM0 in_rate = 16khz)
 * I2S0->DAM1->I2S4 (DAM1 in_rate = 48khz) */
static unsigned int tegra_voice_call_in_srate[2] = {
	16000, /* DAM0 in_srate */
	48000, /* DAM1 in_srate */
};

static struct tegra_vcm30t124_platform_data tegra_voice_call_pdata = {
	/* initialize DAI link config */
	.dai_config[0] = {
		.link_name = "ak-playback",
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "ak4618.0-0010",
		.codec_dai_name = "ak4618-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
		.bclk_ratio = 1,
		.tx_mask = (1 << 8) - 1,
		.rx_mask = (1 << 8) - 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.params.rate_min = 48000,
		.params.rate_max = 48000,
		.params.channels_min = 8,
		.params.channels_max = 8,
	},
	.dai_config[1] = {
		.link_name = "vc-playback",
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "y",
		.bclk_ratio = 2,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 16000,
		.params.rate_max = 16000,
		.params.channels_min = 2,
		.params.channels_max = 2,
	},
	.dai_config[2] = {
		.link_name = "bt-playback",
		.cpu_name = "tegra30-i2s.2",
		.codec_name = "spdif-dit.1",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S2",
		.codec_prefix = "b",
		.bclk_ratio = 4,
		.tx_mask = 1,
		.rx_mask = 1,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_IF |
			SND_SOC_DAIFMT_CBM_CFM,
		.params.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.params.rate_min = 8000,
		.params.rate_max = 8000,
		.params.channels_min = 1,
		.params.channels_max = 1,
	},
	.dev_num = 3,
	/* initialize DAPM routes */
	.dapm_routes = tegra_voice_call_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_voice_call_audio_map),
	/* initialize AMX config */
	.amx_config[0] = {
		.slot_map = tegra_p1859_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_amx_slot_map),
		.in_params = tegra_p1859_amx_input_params,
		.out_params = tegra_p1859_amx_output_params,
	},
	.amx_config[1] = {
		.slot_map = tegra_p1859_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_amx_slot_map),
		.in_params = tegra_p1859_amx_input_params,
		.out_params = tegra_p1859_amx_output_params,
	},
	.num_amx = 2,
	/* initialize ADX config */
	.adx_config[0] = {
		.slot_map = tegra_p1859_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_adx_slot_map),
		.in_params = tegra_p1859_adx_input_params,
		.out_params = tegra_p1859_adx_output_params,
	},
	.adx_config[1] = {
		.slot_map = tegra_p1859_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1859_adx_slot_map),
		.in_params = tegra_p1859_adx_input_params,
		.out_params = tegra_p1859_adx_output_params,
	},
	.num_adx = 2,
	/* initialize DAM input sampling rate */
	.dam_in_srate = tegra_voice_call_in_srate,
	.num_dam = 2,
	/* sound card: tegra-ak4618-voicecall */
	.card_name = "tegra-ak-vc",
};

static struct platform_device tegra_spdif_dit[] = {
	[0] = {
		.name = "spdif-dit",
		.id = 0,
	},
	[1] = {
		.name = "spdif-dit",
		.id = 1,
	},
};

void __init p1859_audio_init(void)
{
	int is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);
	int modem_id = tegra_get_modem_id();
	int is_e1892 = 0, i;
	struct device_node *np;
	static struct property tegra_audio_property = {
		.name = "platform_data",
		.value = &tegra_e1860_b00_pdata,
		.length = sizeof(tegra_e1860_b00_pdata),
	};

	/* check the version of embedded breakout board */
	is_e1892 = tegra_is_board(NULL, "61892", NULL, NULL, NULL);

	/* set max9485 addr as priv data for a0x and b00 */
	tegra_e1860_a0x_pdata.priv_data =
	tegra_e1860_b00_pdata.priv_data =
	tegra_voice_call_pdata.priv_data =
		(void *)(is_e1892 ? 0x70 : 0x60);

	np = of_find_compatible_node(NULL, NULL,
		"nvidia,tegra-audio-vcm30t124");
	if (NULL != np) {
		if (is_e1860_b00) {
			if (modem_id) {
				tegra_audio_property.value =
					(void *)(&tegra_voice_call_pdata);
				tegra_audio_property.length =
					sizeof(tegra_voice_call_pdata);
			}
		} else {
				tegra_audio_property.value =
					(void *)(&tegra_e1860_a0x_pdata);
				tegra_audio_property.length =
					sizeof(tegra_e1860_a0x_pdata);
			}
		of_update_property(np, &tegra_audio_property);
	}

	for (i = 0; i < ARRAY_SIZE(tegra_spdif_dit); i++)
		platform_device_register(&tegra_spdif_dit[i]);

}

void __init p1859_audio_dap_d_sel(void)
{
	int modem_id = tegra_get_modem_id();
	struct device_node *np;
	int gpio;

	if (modem_id) {
		np = of_find_compatible_node(NULL, NULL, "nvidia,dap-d-mux");
		if (NULL != np) {
			gpio = of_get_named_gpio(np,
				"nvidia,dap-d-sel-gpio", 0);
			pr_info("dap-d-sel-gpio:%d\n", gpio);
			if (gpio_is_valid(gpio))
				gpio_direction_output(gpio, 1);
		}
	}
}
