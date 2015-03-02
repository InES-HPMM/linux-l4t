/*
 * arch/arm/mach-tegra/board-p1889-audio.c
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
#include <linux/i2c.h>
#include <sound/soc.h>
#include <mach/tegra_vcm30t124_pdata.h>
#include <mach/board_id.h>
#include "board.h"

#define TDM_SLOT_MAP(stream_id, nth_channel, nth_byte)	\
	((stream_id << 16) | (nth_channel << 8) | nth_byte)

static unsigned int tegra_p1889_amx_slot_map[] = {
	/* slot 0 - Hi */
	TDM_SLOT_MAP(0, 1, 0),
	TDM_SLOT_MAP(0, 1, 1),
	/* slot 1 - Hi */
	TDM_SLOT_MAP(0, 2, 0),
	TDM_SLOT_MAP(0, 2, 1),
	/* slot 2 - Hi */
	TDM_SLOT_MAP(1, 1, 0),
	TDM_SLOT_MAP(1, 1, 1),
	/* slot 3 - Hi */
	TDM_SLOT_MAP(1, 2, 0),
	TDM_SLOT_MAP(1, 2, 1),
	/* slot 4 - Hi */
	TDM_SLOT_MAP(2, 1, 0),
	TDM_SLOT_MAP(2, 1, 1),
	/* slot 5 - Hi */
	TDM_SLOT_MAP(2, 2, 0),
	TDM_SLOT_MAP(2, 2, 1),
	/* slot 6 - Hi */
	TDM_SLOT_MAP(3, 1, 0),
	TDM_SLOT_MAP(3, 1, 1),
	/* slot 7 - Hi */
	TDM_SLOT_MAP(3, 2, 0),
	TDM_SLOT_MAP(3, 2, 1),
	/* slot 0 to 7 - Low */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
};

static unsigned int tegra_p1889_adx_slot_map[] = {
	/* slot 0 - Hi */
	TDM_SLOT_MAP(0, 1, 0),
	TDM_SLOT_MAP(0, 1, 1),
	/* slot 1 - Hi */
	TDM_SLOT_MAP(0, 2, 0),
	TDM_SLOT_MAP(0, 2, 1),
	/* slot 2 - Hi */
	TDM_SLOT_MAP(1, 1, 0),
	TDM_SLOT_MAP(1, 1, 1),
	/* slot 3 - Hi */
	TDM_SLOT_MAP(1, 2, 0),
	TDM_SLOT_MAP(1, 2, 1),
	/* slot 4 - Hi */
	TDM_SLOT_MAP(2, 1, 0),
	TDM_SLOT_MAP(2, 1, 1),
	/* slot 5 - Hi */
	TDM_SLOT_MAP(2, 2, 0),
	TDM_SLOT_MAP(2, 2, 1),
	/* slot 6 - Hi */
	TDM_SLOT_MAP(3, 1, 0),
	TDM_SLOT_MAP(3, 1, 1),
	/* slot 7 - Hi */
	TDM_SLOT_MAP(3, 2, 0),
	TDM_SLOT_MAP(3, 2, 1),
	/* slot 0 to 7 - Low */
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
	TDM_SLOT_MAP(0, 0, 0),
};

static struct snd_soc_pcm_stream tegra_p1889_amx_input_params[] = {
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

static struct snd_soc_pcm_stream tegra_p1889_amx_output_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 8,
		.channels_max = 8,
	},
};

static struct snd_soc_pcm_stream tegra_p1889_adx_output_params[] = {
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

static struct snd_soc_pcm_stream tegra_p1889_adx_input_params[] = {
	[0] = {
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 8,
		.channels_max = 8,
	},
};

static const struct snd_soc_dapm_route tegra_p1889_audio_map[] = {
	{"Headphone-x",	NULL,	"x OUT"},
	{"x IN",	NULL,	"LineIn-x"},
	{"Headphone-y", NULL,	"y OUT"},
	{"y IN",	NULL,	"LineIn-y"},
	{"Headphone-z", NULL,	"z OUT"},
	{"z IN",	NULL,	"LineIn-z"},
};

static struct tegra_vcm30t124_platform_data tegra_p1889_pdata = {
	/* intialize DAI link config */
	.dai_config[0] = {
		.link_name = "p1889-audio-dsp-tdm1",
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "spdif-dit.0",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S0",
		.codec_prefix = "x",
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
	.dai_config[1] = {
		.link_name = "p1889-audio-dsp-tdm2",
		.cpu_name = "tegra30-i2s.2",
		.codec_name = "spdif-dit.1",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S2",
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
		.link_name = "vc-playback",
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "spdif-dit.2",
		.codec_dai_name = "dit-hifi",
		.cpu_dai_name = "I2S4",
		.codec_prefix = "z",
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
	.dev_num = 3,
	/* initialize AMX config */
	.amx_config[0] = {
		.slot_map = tegra_p1889_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1889_amx_slot_map),
		.in_params = tegra_p1889_amx_input_params,
		.out_params = tegra_p1889_amx_output_params,
	},
	.amx_config[1] = {
		.slot_map = tegra_p1889_amx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1889_amx_slot_map),
		.in_params = tegra_p1889_amx_input_params,
		.out_params = tegra_p1889_amx_output_params,
	},
	.num_amx = 2,
	/* initialize ADX config */
	.adx_config[0] = {
		.slot_map = tegra_p1889_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1889_adx_slot_map),
		.in_params = tegra_p1889_adx_input_params,
		.out_params = tegra_p1889_adx_output_params,
	},
	.adx_config[1] = {
		.slot_map = tegra_p1889_adx_slot_map,
		.slot_size = ARRAY_SIZE(tegra_p1889_adx_slot_map),
		.in_params = tegra_p1889_adx_input_params,
		.out_params = tegra_p1889_adx_output_params,
	},
	.num_adx = 2,
	/* initialize DAPM routes */
	.dapm_routes = tegra_p1889_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_p1889_audio_map),
};

static struct platform_device tegra_snd_p1889 = {
	.name = "tegra-snd-p1889ref",
	.id = 0,
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
	[2] = {
		.name = "spdif-dit",
		.id = 2,
	},
};

void __init p1889_audio_init(void)
{
	int i;

	/* assign audio-dsp platform data structure */
	tegra_snd_p1889.dev.platform_data = &tegra_p1889_pdata;

	/* register the platform device and dummy codec if any */
	platform_device_register(&tegra_snd_p1889);
	for (i = 0; i < ARRAY_SIZE(tegra_spdif_dit); i++)
		platform_device_register(&tegra_spdif_dit[i]);
}
