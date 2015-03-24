/*
 * tegra124_virt_alt_dam.c - Tegra124 DAM driver
 *
 * Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_device.h>

#include "tegra124_virt_alt_dam.h"
#include "tegra124_virt_alt_ivc.h"

#define DRV_NAME "tegra124-virt-alt-dam"

#define TEGRA_DAM_CH_REG(id)	TEGRA_DAM_CH##id##_CONV

static int tegra124_virt_alt_dam_get_out_srate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra124_dam *dam = snd_soc_codec_get_drvdata(codec);
		int err;
		struct nvaudio_ivc_msg msg;

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		msg.cmd				= NVAUDIO_DAM_GET_OUT_SRATE;
		msg.params.dam_info.id		= dam->dev_id;
		err = nvaudio_ivc_send(dam->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
		if (err < 0)
				pr_err("error on ivc_send\n");

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		err = nvaudio_ivc_receive_cmd(dam->hivc_client,
					&msg,
					sizeof(struct nvaudio_ivc_msg),
					NVAUDIO_DAM_GET_OUT_SRATE);
		if (err < 0) {
				pr_err("error on ivc_send\n");
				return 0;
		}

	ucontrol->value.integer.value[0] = msg.params.dam_info.out_srate + 1;
	return 0;
}

static int tegra124_virt_alt_dam_get_in_srate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra124_dam *dam = snd_soc_codec_get_drvdata(codec);
		int err;
		struct nvaudio_ivc_msg msg;

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		msg.cmd				= NVAUDIO_DAM_GET_IN_SRATE;
		msg.params.dam_info.id		= dam->dev_id;
		err = nvaudio_ivc_send(dam->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
		if (err < 0)
				pr_err("error on ivc_send\n");

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		err = nvaudio_ivc_receive_cmd(dam->hivc_client,
					&msg,
					sizeof(struct nvaudio_ivc_msg),
					NVAUDIO_DAM_GET_IN_SRATE);
		if (err < 0) {
				pr_err("error on ivc_send\n");
				return 0;
		}

	ucontrol->value.integer.value[0] = msg.params.dam_info.in_srate + 1;
	return 0;
}

static int tegra124_virt_alt_dam_put_in_srate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra124_dam *dam = snd_soc_codec_get_drvdata(codec);
		int err;
		struct nvaudio_ivc_msg msg;

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		msg.cmd				= NVAUDIO_DAM_SET_IN_SRATE;
		msg.params.dam_info.id		= dam->dev_id;
		msg.params.dam_info.in_srate	=
				ucontrol->value.integer.value[0] - 1;
		err = nvaudio_ivc_send(dam->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
		if (err < 0)
				pr_err("error on ivc_send\n");

	return 0;
}

static int tegra124_virt_alt_dam_put_out_srate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra124_dam *dam = snd_soc_codec_get_drvdata(codec);
		int err;
		struct nvaudio_ivc_msg msg;

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		msg.cmd				= NVAUDIO_DAM_SET_OUT_SRATE;
		msg.params.dam_info.id		= dam->dev_id;
		msg.params.dam_info.out_srate	=
					ucontrol->value.integer.value[0] - 1;
		err = nvaudio_ivc_send(dam->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
		if (err < 0)
				pr_err("error on ivc_send\n");

	return 0;
}

static int tegra124_virt_alt_dam_get_ch_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra124_dam *dam = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd				= NVAUDIO_DAM_CHANNEL_GET_GAIN;
	msg.params.dam_info.id		= dam->dev_id;
	msg.params.dam_info.channel_reg	= mc->reg;
	err = nvaudio_ivc_send(dam->hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
			pr_err("error on ivc_send\n");

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	err = nvaudio_ivc_receive_cmd(dam->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg),
				NVAUDIO_DAM_CHANNEL_GET_GAIN);
	if (err < 0) {
			pr_err("error on ivc_receive\n");
			return 0;
	}
	if ((msg.cmd == NVAUDIO_DAM_CHANNEL_GET_GAIN) &&
		(msg.params.dam_info.channel_reg == mc->reg)) {
		ucontrol->value.integer.value[0] =
				msg.params.dam_info.gain;
	} else
		pr_err("invalid reply from server\n");

	return 0;
}

static int tegra124_virt_alt_dam_put_ch_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra124_dam *dam = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
		int err;
		struct nvaudio_ivc_msg msg;

		memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
		msg.cmd			= NVAUDIO_DAM_CHANNEL_SET_GAIN;
		msg.params.dam_info.id	= dam->dev_id;
		msg.params.dam_info.channel_reg	= mc->reg;
		msg.params.dam_info.gain = ucontrol->value.integer.value[0];

		err = nvaudio_ivc_send(dam->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
		if (err < 0)
				pr_err("error on ivc_send\n");
		return 0;

}

static int tegra124_virt_alt_dam_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

#define IN_DAI(id)						\
	{							\
		.name = "IN" #id,				\
		.playback = {					\
			.stream_name = "IN" #id " Receive",	\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
	}

#define OUT_DAI(sname)					\
	{							\
		.name = #sname,					\
		.capture = {					\
			.stream_name = #sname " Transmit",	\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
	}

static struct snd_soc_dai_driver tegra124_virt_alt_dam_dais[] = {
	IN_DAI(0),
	IN_DAI(1),
	OUT_DAI(OUT),
};

static const struct snd_soc_dapm_widget tegra124_virt_alt_dam_widgets[] = {
	SND_SOC_DAPM_AIF_IN("IN0", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN1", NULL, 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("OUT", NULL, 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route tegra124_virt_alt_dam_routes[] = {
	{ "IN0",	   NULL, "IN0 Receive" },
	{ "IN1",	   NULL, "IN1 Receive" },
	{ "OUT",	   NULL, "IN0" },
	{ "OUT",	   NULL, "IN1" },
	{ "OUT Transmit", NULL, "OUT" },
};

static const char * const tegra124_virt_alt_dam_srate_text[] = {
	"None",
	"8kHz",
	"16kHz",
	"44kHz",
	"48kHz",
	"11kHz",
	"22kHz",
	"24kHz",
	"32kHz",
	"88kHz",
	"96kHz",
	"176kHz",
	"192kHz",
};

static const struct soc_enum tegra124_virt_alt_dam_srate =
	SOC_ENUM_SINGLE_EXT(13, tegra124_virt_alt_dam_srate_text);

static const struct snd_kcontrol_new tegra124_virt_alt_dam_controls[] = {
	SOC_ENUM_EXT("output rate", tegra124_virt_alt_dam_srate,
		tegra124_virt_alt_dam_get_out_srate,
		tegra124_virt_alt_dam_put_out_srate),
	SOC_ENUM_EXT("input rate", tegra124_virt_alt_dam_srate,
		tegra124_virt_alt_dam_get_in_srate,
		tegra124_virt_alt_dam_put_in_srate),
	SOC_SINGLE_EXT("ch0 gain", TEGRA_DAM_CH_REG(0), 0, 0xFFFF, 0,
		tegra124_virt_alt_dam_get_ch_gain,
		tegra124_virt_alt_dam_put_ch_gain),
	SOC_SINGLE_EXT("ch1 gain", TEGRA_DAM_CH_REG(1), 0, 0xFFFF, 0,
		tegra124_virt_alt_dam_get_ch_gain,
		tegra124_virt_alt_dam_put_ch_gain),
};

static struct snd_soc_codec_driver tegra124_virt_alt_dam_codec = {
	.probe = tegra124_virt_alt_dam_codec_probe,
	.dapm_widgets = tegra124_virt_alt_dam_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra124_virt_alt_dam_widgets),
	.dapm_routes = tegra124_virt_alt_dam_routes,
	.num_dapm_routes = ARRAY_SIZE(tegra124_virt_alt_dam_routes),
	.controls = tegra124_virt_alt_dam_controls,
	.num_controls = ARRAY_SIZE(tegra124_virt_alt_dam_controls),
	.idle_bias_off = 1,
};

static const struct of_device_id tegra124_virt_alt_dam_of_match[] = {
	{ .compatible = "nvidia,tegra124-virt-alt-dam" },
	{},
};

static int tegra124_virt_alt_dam_platform_probe(struct platform_device *pdev)
{
	struct tegra124_dam *dam;
	int ret;

	dam = devm_kzalloc(&pdev->dev, sizeof(struct tegra124_dam), GFP_KERNEL);
	if (!dam) {
		dev_err(&pdev->dev, "Can't allocate tegra124_dam\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, dam);

	if (of_property_read_u32(pdev->dev.of_node,
		"nvidia,ahub-dam-id", &pdev->dev.id) < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,ahub-dam-id\n");
		ret = -ENODEV;
		goto err;
	}
	dam->dev_id = pdev->dev.id;
	dam->hivc_client = nvaudio_ivc_alloc_ctxt(&pdev->dev);

	ret = snd_soc_register_codec(&pdev->dev,
				&tegra124_virt_alt_dam_codec,
				tegra124_virt_alt_dam_dais,
				ARRAY_SIZE(tegra124_virt_alt_dam_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err;
	}

	return 0;

err:
	return ret;
}

static int tegra124_virt_alt_dam_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver tegra124_virt_alt_dam_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra124_virt_alt_dam_of_match,
	},
	.probe = tegra124_virt_alt_dam_platform_probe,
	.remove = tegra124_virt_alt_dam_platform_remove,
};
module_platform_driver(tegra124_virt_alt_dam_driver);

MODULE_AUTHOR("Ranjith Kannikara <rkannikara@nvidia.com>");
MODULE_DESCRIPTION("Tegra124 DAM ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra124_virt_alt_dam_of_match);
