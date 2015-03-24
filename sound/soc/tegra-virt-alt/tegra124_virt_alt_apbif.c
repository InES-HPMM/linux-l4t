/*
 * tegra124_virt_alt_apbif.c
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
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra_pcm_alt.h"
#include "tegra124_virt_alt_apbif.h"
#include "tegra124_virt_alt_ivc.h"

#define DRV_NAME	"tegra124-virt-alt-pcm"

static struct tegra124_virt_apbif_client *apbif_client;

static int tegra124_virt_apbif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra124_virt_apbif_client_data *data =
				&apbif_client->client_data;
	struct tegra124_virt_audio_cif *cif_conf = &data->cif;
	int ret = 0, value, err = 0;
	struct nvaudio_ivc_msg msg;
	data->apbif_id = dai->id;

	/* initialize the audio cif */
	cif_conf->audio_channels = params_channels(params);
	cif_conf->client_channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		cif_conf->client_bits = AUDIO_BITS_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf->client_bits = AUDIO_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		cif_conf->client_bits = AUDIO_BITS_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf->client_bits = AUDIO_BITS_32;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}
	cif_conf->audio_bits = AUDIO_BITS_32;

	cif_conf->direction = substream->stream;
	cif_conf->threshold = 0;
	cif_conf->expand = 0;
	cif_conf->stereo_conv = 0;
	cif_conf->replicate = 0;
	cif_conf->truncate = 0;
	cif_conf->mono_conv = 0;

	/* set the audio cif */
	value = (cif_conf->threshold << 24) |
			((cif_conf->audio_channels - 1) << 20) |
			((cif_conf->client_channels - 1) << 16) |
			(cif_conf->audio_bits << 12) |
			(cif_conf->client_bits << 8) |
			(cif_conf->expand << 6) |
			(cif_conf->stereo_conv << 4) |
			(cif_conf->replicate << 3) |
			(cif_conf->direction << 2) |
			(cif_conf->truncate << 1) |
			(cif_conf->mono_conv << 0);

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.params.apbif_info.id	= data->apbif_id;
	msg.params.apbif_info.value	= value;
	if (!cif_conf->direction)
		msg.cmd = NVAUDIO_APBIF_SET_TXCIF;
	else
		msg.cmd = NVAUDIO_APBIF_SET_RXCIF;

	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
	return ret;
}

static void tegra124_virt_apbif_start_playback(struct snd_soc_dai *dai)
{
	struct tegra124_virt_apbif_client_data *data =
				&apbif_client->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->apbif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_APBIF_START_PLAYBACK;
	msg.params.apbif_info.id = data->apbif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static void tegra124_virt_apbif_stop_playback(struct snd_soc_dai *dai)
{
	struct tegra124_virt_apbif_client_data *data =
				&apbif_client->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->apbif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_APBIF_STOP_PLAYBACK;
	msg.params.apbif_info.id = data->apbif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static void tegra124_virt_apbif_start_capture(struct snd_soc_dai *dai)
{
	struct tegra124_virt_apbif_client_data *data =
				&apbif_client->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->apbif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_APBIF_START_CAPTURE;
	msg.params.apbif_info.id = data->apbif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static void tegra124_virt_apbif_stop_capture(struct snd_soc_dai *dai)
{
	struct tegra124_virt_apbif_client_data *data =
				&apbif_client->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->apbif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_APBIF_STOP_CAPTURE;
	msg.params.apbif_info.id = data->apbif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static int tegra124_virt_apbif_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra124_virt_apbif_start_playback(dai);
		else
			tegra124_virt_apbif_start_capture(dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra124_virt_apbif_stop_playback(dai);
		else
			tegra124_virt_apbif_stop_capture(dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops tegra124_virt_apbif_dai_ops = {
	.hw_params	= tegra124_virt_apbif_hw_params,
	.trigger	= tegra124_virt_apbif_trigger,
};

static int tegra124_virt_apbif_dai_probe(struct snd_soc_dai *dai)
{

	dai->capture_dma_data = &apbif_client->capture_dma_data[dai->id];
	dai->playback_dma_data = &apbif_client->playback_dma_data[dai->id];

	return 0;
}

#define APBIF_DAI(id)							\
	{							\
		.name = "APBIF" #id,				\
		.probe = tegra124_virt_apbif_dai_probe,		\
		.playback = {					\
			.stream_name = "Playback " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 | \
				SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = "Capture " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 | \
				SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.ops = &tegra124_virt_apbif_dai_ops,			\
	}

static struct snd_soc_dai_driver tegra124_apbif_dais[] = {
	APBIF_DAI(0),
	APBIF_DAI(1),
	APBIF_DAI(2),
	APBIF_DAI(3),
	APBIF_DAI(4),
	APBIF_DAI(5),
	APBIF_DAI(6),
	APBIF_DAI(7),
	APBIF_DAI(8),
	APBIF_DAI(9),
};

static const struct snd_soc_component_driver tegra124_virt_apbif_dai_driver = {
	.name		= DRV_NAME,
};

int tegra124_virt_apbif_register_component(struct platform_device *pdev)
{
	int i, ret;
	u32 of_dma[20][2];

	apbif_client = devm_kzalloc(&pdev->dev,
					sizeof(*apbif_client),	GFP_KERNEL);
	if (!apbif_client) {
		dev_err(&pdev->dev, "Can't allocate tegra124_virt_apbif\n");
		ret = -ENOMEM;
		goto err;
	}

	apbif_client->client_data.hivc_client =
				nvaudio_ivc_alloc_ctxt(&pdev->dev);

	apbif_client->capture_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				MAX_APBIF_IDS,
			GFP_KERNEL);
	if (!apbif_client->capture_dma_data) {
		dev_err(&pdev->dev, "Can't allocate tegra_alt_pcm_dma_params\n");
		ret = -ENOMEM;
		goto err;
	}

	apbif_client->playback_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				MAX_APBIF_IDS,
			GFP_KERNEL);
	if (!apbif_client->playback_dma_data) {
		dev_err(&pdev->dev, "Can't allocate tegra_alt_pcm_dma_params\n");
		ret = -ENOMEM;
		goto err;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
				"dmas",
				&of_dma[0][0],
				MAX_APBIF_IDS * 2) < 0) {
			dev_err(&pdev->dev,
			"Missing property nvidia,dma-request-selector\n");
			ret = -ENODEV;
		goto err;
	}

	/* default DAI number is 4 */
	for (i = 0; i < MAX_APBIF_IDS; i++) {
		if (i < APBIF_ID_4) {
			apbif_client->playback_dma_data[i].addr =
			TEGRA_APBIF_BASE + TEGRA_APBIF_CHANNEL_TXFIFO +
			(i * TEGRA_APBIF_CHANNEL_TXFIFO_STRIDE);

			apbif_client->capture_dma_data[i].addr =
			TEGRA_APBIF_BASE + TEGRA_APBIF_CHANNEL_RXFIFO +
			(i * TEGRA_APBIF_CHANNEL_RXFIFO_STRIDE);
		} else {
			apbif_client->playback_dma_data[i].addr =
			TEGRA_APBIF2_BASE2 + TEGRA_APBIF_CHANNEL_TXFIFO +
			((i - APBIF_ID_4) * TEGRA_APBIF_CHANNEL_TXFIFO_STRIDE);

			apbif_client->capture_dma_data[i].addr =
			TEGRA_APBIF2_BASE2 + TEGRA_APBIF_CHANNEL_RXFIFO +
			((i - APBIF_ID_4) * TEGRA_APBIF_CHANNEL_RXFIFO_STRIDE);
		}

		apbif_client->playback_dma_data[i].wrap = 4;
		apbif_client->playback_dma_data[i].width = 32;
		apbif_client->playback_dma_data[i].req_sel =
			of_dma[(i * 2) + 1][1];

		if (of_property_read_string_index(pdev->dev.of_node,
			"dma-names",
			(i * 2) + 1,
			&apbif_client->playback_dma_data[i].chan_name) < 0) {
				dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
				ret = -ENODEV;
				goto err;
		}

		apbif_client->capture_dma_data[i].wrap = 4;
		apbif_client->capture_dma_data[i].width = 32;
		apbif_client->capture_dma_data[i].req_sel = of_dma[(i * 2)][1];
		if (of_property_read_string_index(pdev->dev.of_node,
			"dma-names",
			(i * 2),
			&apbif_client->capture_dma_data[i].chan_name) < 0) {
				dev_err(&pdev->dev,
								"Missing property nvidia,dma-names\n");
				ret = -ENODEV;
				goto err;
		}
	}

	ret = snd_soc_register_component(&pdev->dev,
					&tegra124_virt_apbif_dai_driver,
					tegra124_apbif_dais,
					ARRAY_SIZE(tegra124_apbif_dais));
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAIs %d: %d\n",
			i, ret);
		ret = -ENOMEM;
		goto err;
	}

	ret = tegra_alt_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_dais;
	}
	return 0;

err_unregister_dais:
	snd_soc_unregister_component(&pdev->dev);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra124_virt_apbif_register_component);

MODULE_AUTHOR("Ranjith Kannikara <rkannikara@nvidia.com>");
MODULE_DESCRIPTION("Tegra124 virt apbif component");
MODULE_LICENSE("GPL");
