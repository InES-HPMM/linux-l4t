/*
 * tegra30_offload.c - Tegra platform driver for offload rendering
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

/*
#define VERBOSE_DEBUG 1
#define DEBUG 1
*/

#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/compress_driver.h>
#include <sound/dmaengine_pcm.h>

#include "tegra_pcm.h"
#include "tegra_offload.h"

#define DRV_NAME "tegra-offload"

enum {
	PCM_OFFLOAD_DAI,
	COMPR_OFFLOAD_DAI,
	PCM_CAPTURE_OFFLOAD_DAI,
	MAX_OFFLOAD_DAI
};

struct tegra_offload_pcm_data {
	struct tegra_offload_pcm_ops *ops;
	int appl_ptr;
	int stream_id;
};

struct tegra_offload_compr_data {
	struct tegra_offload_compr_ops *ops;
	struct snd_codec codec;
	int stream_id;
	int stream_vol[2];
	struct snd_kcontrol *kcontrol;
};

static struct tegra_offload_ops offload_ops;
static int tegra_offload_init_done;
static DEFINE_MUTEX(tegra_offload_lock);

static int codec, spk;
static const struct snd_pcm_hardware tegra_offload_pcm_hardware_playback = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 128,
	.period_bytes_max	= PAGE_SIZE * 2,
	.periods_min		= 1,
	.periods_max		= 8,
	.buffer_bytes_max	= PAGE_SIZE * 8,
	.fifo_size		= 4,
};

static const struct snd_pcm_hardware tegra_offload_pcm_hardware_capture = {
	.info                   = SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats                = SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min           = 2,
	.channels_max           = 2,
	.period_bytes_min       = 128,
	.period_bytes_max       = PAGE_SIZE * 2,
	.periods_min            = 1,
	.periods_max            = 8,
	.buffer_bytes_max       = PAGE_SIZE * 8,
	.fifo_size              = 4,
};

int tegra_register_offload_ops(struct tegra_offload_ops *ops)
{
	int ret = 0;

	mutex_lock(&tegra_offload_lock);
	if (!ops) {
		pr_err("Invalid ops pointer.");
		ret = -EINVAL;
		goto errout;
	}

	if (tegra_offload_init_done) {
		pr_err("Offload ops already registered.");
		ret = -EBUSY;
		goto errout;
	}

	memcpy(&offload_ops, ops, sizeof(offload_ops));
	tegra_offload_init_done = 1;
	pr_info("succefully registered offload ops");

errout:
	mutex_unlock(&tegra_offload_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tegra_register_offload_ops);

void tegra_deregister_offload_ops(void)
{
	mutex_lock(&tegra_offload_lock);

	if (!tegra_offload_init_done) {
		pr_err("Offload ops not registered.");
		mutex_unlock(&tegra_offload_lock);
		return;
	}

	memset(&offload_ops, 0, sizeof(offload_ops));
	tegra_offload_init_done = 0;

	mutex_unlock(&tegra_offload_lock);

	pr_info("succefully deregistered offload ops");
	return;
}
EXPORT_SYMBOL_GPL(tegra_deregister_offload_ops);


/* Compress playback related APIs */
static void tegra_offload_compr_fragment_elapsed(void *arg, unsigned int is_eos)
{
	struct snd_compr_stream *stream = arg;

	if (stream) {
		snd_compr_fragment_elapsed(stream);
		if (is_eos)
			snd_compr_drain_notify(stream);
	}
}

static int tegra_set_compress_volume(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 1;
	struct snd_compr_stream *stream = snd_kcontrol_chip(kcontrol);
	struct tegra_offload_compr_data *data = stream->runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = stream->device->private_data;
	struct device *dev = rtd->platform->dev;

	pr_debug("%s: value[0]: %d value[1]: %d\n", __func__,
		(int)ucontrol->value.integer.value[0],
		(int)ucontrol->value.integer.value[1]);
	ret = data->ops->set_stream_volume(data->stream_id,
			(int)ucontrol->value.integer.value[0],
			(int)ucontrol->value.integer.value[1]);
	if (ret < 0) {
		dev_err(dev, "Failed to get compr caps. ret %d", ret);
		return ret;
	} else {
		data->stream_vol[0] = (int)ucontrol->value.integer.value[0];
		data->stream_vol[1] = (int)ucontrol->value.integer.value[1];
	}
	return 1;
}


static int tegra_get_compress_volume(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_compr_stream *stream = snd_kcontrol_chip(kcontrol);
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	ucontrol->value.integer.value[0] = data->stream_vol[0];
	ucontrol->value.integer.value[1] = data->stream_vol[1];

	return 0;
}

struct snd_kcontrol_new tegra_offload_volume =
		SOC_DOUBLE_EXT("Compress Playback Volume", 0, 1, 0, 0xFFFFFFFF,
		1, tegra_get_compress_volume, tegra_set_compress_volume);

static int tegra_offload_compr_add_controls(struct snd_compr_stream *stream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = stream->device->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	data->kcontrol =  snd_ctl_new1(&tegra_offload_volume, stream);
	ret = snd_ctl_add(rtd->card->snd_card, data->kcontrol);
	if (ret < 0) {
		dev_err(dev, "Can't add offload volume");
		return ret;
	}
	return ret;
}


static int tegra_offload_compr_remove_controls(struct snd_compr_stream *stream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = stream->device->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	ret = snd_ctl_remove(rtd->card->snd_card, data->kcontrol);
	if (ret < 0) {
		dev_err(dev, "Can't remove offload volume");
		return ret;
	}
	return ret;
}

static int tegra_offload_compr_open(struct snd_compr_stream *stream)
{
	struct snd_soc_pcm_runtime *rtd = stream->device->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_compr_data *data;
	int ret = 0;

	dev_vdbg(dev, "%s", __func__);

	if (!tegra_offload_init_done) {
		dev_err(dev, "Offload interface is not registered");
		return -ENODEV;
	}

	if (!stream->device->dev)
		stream->device->dev = dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_vdbg(dev, "Failed to allocate tegra_offload_compr_data.");
		return -ENOMEM;
	}
	data->ops = &offload_ops.compr_ops;

	ret = data->ops->stream_open(&data->stream_id);
	if (ret < 0) {
		dev_err(dev, "Failed to open offload stream. err %d", ret);
		return ret;
	}

	stream->runtime->private_data = data;

	ret = tegra_offload_compr_add_controls(stream);
	if (ret)
		dev_err(dev, "Failed to add controls\n");

	return 0;
}

static int tegra_offload_compr_free(struct snd_compr_stream *stream)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;
	int ret = 0;

	dev_vdbg(dev, "%s", __func__);

	ret = tegra_offload_compr_remove_controls(stream);
	if (ret)
		dev_err(dev, "Failed to remove controls\n");

	data->ops->stream_close(data->stream_id);
	devm_kfree(dev, data);
	return 0;
}

static int tegra_offload_compr_set_params(struct snd_compr_stream *stream,
			struct snd_compr_params *params)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = stream->device->private_data;
	struct tegra_pcm_dma_params *dmap;
	struct tegra_offload_compr_params offl_params;
	int dir;
	int ret = 0;

	dev_vdbg(dev, "%s", __func__);

	if (stream->direction == SND_COMPRESS_PLAYBACK)
		dir = SNDRV_PCM_STREAM_PLAYBACK;
	else
		dir = SNDRV_PCM_STREAM_CAPTURE;

	dmap = rtd->cpu_dai->playback_dma_data;
	if (!dmap) {
		struct snd_soc_dpcm *dpcm;

		if (list_empty(&rtd->dpcm[dir].be_clients)) {
			dev_err(dev, "No backend DAIs enabled for %s\n",
					rtd->dai_link->name);
			return -EINVAL;
		}

		list_for_each_entry(dpcm,
			&rtd->dpcm[dir].be_clients, list_be) {
			struct snd_soc_pcm_runtime *be = dpcm->be;
			struct snd_pcm_substream *be_substream =
				snd_soc_dpcm_get_substream(be, dir);
			struct snd_soc_dai_link *dai_link = be->dai_link;

			dmap = snd_soc_dai_get_dma_data(be->cpu_dai,
						be_substream);

			if (spk && strstr(dai_link->name, "speaker")) {
				dmap = snd_soc_dai_get_dma_data(be->cpu_dai,
						be_substream);
				break;
			}
			if (codec && strstr(dai_link->name, "codec")) {
				dmap = snd_soc_dai_get_dma_data(be->cpu_dai,
						be_substream);
				break;
			}
			/* TODO : Multiple BE to single FE not yet supported */
		}
	}
	if (!dmap) {
		dev_err(dev, "Failed to get DMA params.");
		return -ENODEV;
	}

	offl_params.codec_type = params->codec.id;
	offl_params.bits_per_sample = 16;
	offl_params.rate = snd_pcm_rate_bit_to_rate(params->codec.sample_rate);
	offl_params.channels = params->codec.ch_in;
	offl_params.fragment_size = params->buffer.fragment_size;
	offl_params.fragments = params->buffer.fragments;
	offl_params.dma_params.addr = dmap->addr;
	offl_params.dma_params.width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	offl_params.dma_params.req_sel = dmap->req_sel;
	offl_params.dma_params.max_burst = 4;
	offl_params.fragments_elapsed_cb = tegra_offload_compr_fragment_elapsed;
	offl_params.fragments_elapsed_args = (void *)stream;

	ret = data->ops->set_stream_params(data->stream_id, &offl_params);
	if (ret < 0) {
		dev_err(dev, "Failed to set offload params. ret %d", ret);
		return ret;
	}
	memcpy(&data->codec, &params->codec, sizeof(struct snd_codec));
	return 0;
}

static int tegra_offload_compr_get_params(struct snd_compr_stream *stream,
					struct snd_codec *codec)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	dev_vdbg(dev, "%s", __func__);

	memcpy(codec, &data->codec, sizeof(struct snd_codec));
	return 0;
}

static int tegra_offload_compr_trigger(struct snd_compr_stream *stream, int cmd)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	dev_vdbg(dev, "%s : cmd %d", __func__, cmd);

	data->ops->set_stream_state(data->stream_id, cmd);
	return 0;
}

static int tegra_offload_compr_pointer(struct snd_compr_stream *stream,
		struct snd_compr_tstamp *tstamp)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	dev_vdbg(dev, "%s", __func__);

	return data->ops->get_stream_position(data->stream_id, tstamp);
}

static int tegra_offload_compr_copy(struct snd_compr_stream *stream,
		char __user *buf, size_t count)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;

	dev_vdbg(dev, "%s : bytes %d", __func__, (int)count);

	return data->ops->write(data->stream_id, buf, count);
}

static int tegra_offload_compr_get_caps(struct snd_compr_stream *stream,
		struct snd_compr_caps *caps)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;
	int ret = 0;

	dev_vdbg(dev, "%s", __func__);

	caps->direction = stream->direction;
	ret = data->ops->get_caps(caps);
	if (ret < 0) {
		dev_err(dev, "Failed to get compr caps. ret %d", ret);
		return ret;
	}
	return 0;
}

static int tegra_offload_compr_codec_caps(struct snd_compr_stream *stream,
		struct snd_compr_codec_caps *codec_caps)
{
	struct device *dev = stream->device->dev;
	struct tegra_offload_compr_data *data = stream->runtime->private_data;
	int ret = 0;

	dev_vdbg(dev, "%s", __func__);

	if (!codec_caps->codec)
		codec_caps->codec = data->codec.id;

	ret = data->ops->get_codec_caps(codec_caps);
	if (ret < 0) {
		dev_err(dev, "Failed to get codec caps. ret %d", ret);
		return ret;
	}
	return 0;
}

static struct snd_compr_ops tegra_compr_ops = {
	.open = tegra_offload_compr_open,
	.free = tegra_offload_compr_free,
	.set_params = tegra_offload_compr_set_params,
	.get_params = tegra_offload_compr_get_params,
	.trigger = tegra_offload_compr_trigger,
	.pointer = tegra_offload_compr_pointer,
	.copy = tegra_offload_compr_copy,
	.get_caps = tegra_offload_compr_get_caps,
	.get_codec_caps = tegra_offload_compr_codec_caps,
};

/* PCM playback related APIs */
static void tegra_offload_pcm_period_elapsed(void *arg, unsigned int is_eos)
{
	struct snd_pcm_substream *substream = arg;

	if (substream)
		snd_pcm_period_elapsed(substream);
}

static int tegra_offload_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_pcm_data *data;
	int ret = 0;

	dev_vdbg(dev, "%s", __func__);

	if (!tegra_offload_init_done) {
		dev_err(dev, "Offload interface is not registered");
		return -ENODEV;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_vdbg(dev,
			"Failed to allocate tegra_offload_pcm_data\n");
		return -ENOMEM;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		/* Set HW params now that initialization is complete */
		snd_soc_set_runtime_hwparams(substream,
				&tegra_offload_pcm_hardware_playback);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		snd_soc_set_runtime_hwparams(substream,
				&tegra_offload_pcm_hardware_capture);

	/* Ensure period size is multiple of 4 */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 0x4);
	if (ret) {
		dev_err(dev, "failed to set constraint %d\n", ret);
		return ret;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		data->ops = &offload_ops.pcm_ops;

		ret = data->ops->stream_open(&data->stream_id, "pcm");
		if (ret < 0) {
			dev_err(dev,
				"Failed to open offload stream err %d", ret);
			return ret;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		data->ops = &offload_ops.loopback_ops;

		ret = data->ops->stream_open(&data->stream_id, "loopback");
		if (ret < 0) {
			dev_err(dev,
				"Failed to open offload stream err %d", ret);
			return ret;
		}
	}
	offload_ops.device_ops.set_hw_rate(48000);
	substream->runtime->private_data = data;
	return 0;
}

static int tegra_offload_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_pcm_data *data = substream->runtime->private_data;

	dev_vdbg(dev, "%s", __func__);

	data->ops->stream_close(data->stream_id);
	devm_kfree(dev, data);
	return 0;
}

static int tegra_offload_pcm_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_pcm_data *data = substream->runtime->private_data;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct tegra_offload_pcm_params offl_params;
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		struct tegra_pcm_dma_params *dmap;
		dmap = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
		if (!dmap) {
			struct snd_soc_dpcm *dpcm;

			if
			(list_empty(&rtd->dpcm[substream->stream].be_clients)) {
				dev_err(dev, "No backend DAIs enabled for %s\n",
					rtd->dai_link->name);
				return -EINVAL;
			}

			list_for_each_entry(dpcm,
				&rtd->dpcm[substream->stream].be_clients,
				list_be) {
				struct snd_soc_pcm_runtime *be = dpcm->be;
				struct snd_pcm_substream *be_substream =
					snd_soc_dpcm_get_substream(be,
					substream->stream);
				struct snd_soc_dai_link *dai_link =
							be->dai_link;

				dmap = snd_soc_dai_get_dma_data(be->cpu_dai,
							be_substream);

				if (spk && strstr(dai_link->name, "speaker")) {
					dmap = snd_soc_dai_get_dma_data(
							be->cpu_dai,
							be_substream);
					break;
				}
				if (codec && strstr(dai_link->name, "codec")) {
					dmap = snd_soc_dai_get_dma_data(
							be->cpu_dai,
							be_substream);
					break;
				}
				/* TODO : Multiple BE to
				 * single FE not yet supported */
			}
		}
		if (!dmap) {
			dev_err(dev, "Failed to get DMA params.");
			return -ENODEV;
		}
		offl_params.dma_params.addr = dmap->addr;
		offl_params.dma_params.width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		offl_params.dma_params.req_sel = dmap->req_sel;
		offl_params.dma_params.max_burst = 4;
	}

	offl_params.bits_per_sample =
		snd_pcm_format_width(params_format(params));
	offl_params.rate = params_rate(params);
	offl_params.channels = params_channels(params);
	offl_params.buffer_size = params_buffer_bytes(params);
	offl_params.period_size = params_period_size(params) *
		((offl_params.bits_per_sample >> 3) * offl_params.channels);

	offl_params.source_buf.virt_addr = buf->area;
	offl_params.source_buf.phys_addr = buf->addr;
	offl_params.source_buf.bytes = buf->bytes;

	offl_params.period_elapsed_cb = tegra_offload_pcm_period_elapsed;
	offl_params.period_elapsed_args = (void *)substream;

	ret = data->ops->set_stream_params(data->stream_id, &offl_params);
	if (ret < 0) {
		dev_err(dev, "Failed to set avp params. ret %d", ret);
		return ret;
	}
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int tegra_offload_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;

	dev_vdbg(dev, "%s", __func__);

	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int tegra_offload_pcm_trigger(struct snd_pcm_substream *substream,
				     int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_pcm_data *data = substream->runtime->private_data;

	dev_vdbg(dev, "%s : cmd %d", __func__, cmd);

	data->ops->set_stream_state(data->stream_id, cmd);
	if ((cmd == SNDRV_PCM_TRIGGER_STOP) ||
		(cmd == SNDRV_PCM_TRIGGER_SUSPEND) ||
		(cmd == SNDRV_PCM_TRIGGER_PAUSE_PUSH))
		data->appl_ptr = 0;
	return 0;
}

static snd_pcm_uframes_t tegra_offload_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct tegra_offload_pcm_data *data = substream->runtime->private_data;
	size_t position;

	dev_vdbg(dev, "%s", __func__);

	position = data->ops->get_stream_position(data->stream_id);
	return bytes_to_frames(substream->runtime, position);
}

static int tegra_offload_pcm_ack(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct tegra_offload_pcm_data *data = runtime->private_data;

	int data_size = runtime->control->appl_ptr - data->appl_ptr;

	if (data_size < 0)
		data_size += runtime->boundary;

	data->ops->data_ready(data->stream_id,
		frames_to_bytes(runtime, data_size));
	data->appl_ptr = runtime->control->appl_ptr;
	return 0;
}

static int codec_get_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = codec;
	return 0;
}

static int codec_put_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];

	if (ucontrol->value.integer.value[0]) {
		codec = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
	} else {
		codec = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
	}
	return 1;
}

static int spk_get_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = spk;
	return 0;
}

static int spk_put_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];

	if (ucontrol->value.integer.value[0]) {
		spk = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
	} else {
		spk = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
	}
	return 1;
}

static const struct snd_kcontrol_new codec_control =
	SOC_SINGLE_EXT("Codec Switch", SND_SOC_NOPM, 0, 1, 0,
			codec_get_mixer, codec_put_switch);

static const struct snd_kcontrol_new spk_control =
	SOC_SINGLE_EXT("SPK Switch", SND_SOC_NOPM, 1, 1, 0,
			spk_get_mixer, spk_put_switch);

static const struct snd_soc_dapm_widget tegra_offload_widgets[] = {
	/* BackEnd DAIs */
	SND_SOC_DAPM_AIF_OUT("I2S0_OUT", "tegra30-i2s.0 Playback", 0,
		0/*wreg*/, 0/*wshift*/, 0/*winvert*/),
	SND_SOC_DAPM_AIF_OUT("I2S1_OUT", "tegra30-i2s.1 Playback", 0,
		0/*wreg*/, 0/*wshift*/, 0/*winvert*/),
	SND_SOC_DAPM_AIF_OUT("I2S2_OUT", "tegra30-i2s.2 Playback", 0,
		0/*wreg*/, 0/*wshift*/, 0/*winvert*/),
	SND_SOC_DAPM_AIF_OUT("I2S3_OUT", "tegra30-i2s.3 Playback", 0,
		0/*wreg*/, 0/*wshift*/, 0/*winvert*/),
	SND_SOC_DAPM_AIF_OUT("I2S4_OUT", "tegra30-i2s.4 Playback", 0,
		0/*wreg*/, 0/*wshift*/, 0/*winvert*/),

	SND_SOC_DAPM_MIXER("Codec VMixer", SND_SOC_NOPM, 0, 0,
		&codec_control, 1),
	SND_SOC_DAPM_MIXER("SPK VMixer", SND_SOC_NOPM, 0, 0,
		&spk_control, 1),
};

static const struct snd_soc_dapm_route graph[] = {
	{"Codec VMixer", "Codec Switch", "offload-pcm-playback"},
	{"Codec VMixer", "Codec Switch", "offload-compr-playback"},
	{"I2S1_OUT", NULL, "Codec VMixer"},

	{"SPK VMixer", "SPK Switch", "offload-pcm-playback"},
	{"SPK VMixer", "SPK Switch", "offload-compr-playback"},
	{"I2S2_OUT", NULL, "SPK VMixer"},
};

static struct snd_pcm_ops tegra_pcm_ops = {
	.open		= tegra_offload_pcm_open,
	.close		= tegra_offload_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= tegra_offload_pcm_hw_params,
	.hw_free	= tegra_offload_pcm_hw_free,
	.trigger	= tegra_offload_pcm_trigger,
	.pointer	= tegra_offload_pcm_pointer,
	.ack		= tegra_offload_pcm_ack,
};

static void tegra_offload_dma_free(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	struct tegra_offload_mem mem;

	substream = pcm->streams[stream].substream;
	if (!substream)
		return;

	buf = &substream->dma_buffer;
	if (!buf->area)
		return;

	mem.dev = buf->dev.dev;
	mem.bytes = buf->bytes;
	mem.virt_addr = buf->area;
	mem.phys_addr = buf->addr;

	offload_ops.device_ops.free_shared_mem(&mem);
	buf->area = NULL;
}

static u64 tegra_dma_mask = DMA_BIT_MASK(32);
static int tegra_offload_dma_allocate(struct snd_soc_pcm_runtime *rtd,
			int stream, size_t size)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	struct tegra_offload_mem mem;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &tegra_dma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	ret = offload_ops.device_ops.alloc_shared_mem(&mem, size);
	if (ret < 0) {
		dev_err(pcm->card->dev, "Failed to allocate memory");
		return -ENOMEM;
	}
	buf->area = mem.virt_addr;
	buf->addr = mem.phys_addr;
	buf->private_data = NULL;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = mem.dev;
	buf->bytes = mem.bytes;
	return 0;
}

static int tegra_offload_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct device *dev = rtd->platform->dev;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;


	dev_vdbg(dev, "%s", __func__);
	dev_info(pcm->card->dev, "Allocating for stream playback\n");
	ret = tegra_offload_dma_allocate(rtd , SNDRV_PCM_STREAM_PLAYBACK,
			tegra_offload_pcm_hardware_playback.buffer_bytes_max);
	if (ret < 0) {
		dev_err(pcm->card->dev, "failing in pcm_new:1 goto err");
		goto err;
	}
	dev_info(pcm->card->dev, "Allocating for stream capture\n");
	ret = tegra_offload_dma_allocate(rtd , SNDRV_PCM_STREAM_CAPTURE,
			tegra_offload_pcm_hardware_capture.buffer_bytes_max);
	if (ret < 0) {
		dev_err(pcm->card->dev, "failing in pcm_new:1 goto err");
		goto err;
	}
err:
	return ret;
}

static void tegra_offload_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("%s", __func__);
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		dev_info(pcm->card->dev, "PCM free for stream playback\n");
		tegra_offload_dma_free(pcm, SNDRV_PCM_STREAM_PLAYBACK);
	}
	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		dev_info(pcm->card->dev, "PCM free for stream capture\n");
		tegra_offload_dma_free(pcm, SNDRV_PCM_STREAM_CAPTURE);
	}
}

static int tegra_offload_pcm_probe(struct snd_soc_platform *platform)
{
	pr_debug("%s", __func__);

	platform->dapm.idle_bias_off = 1;
	return 0;
}

unsigned int tegra_offload_pcm_read(struct snd_soc_platform *platform,
		unsigned int reg)
{
	return 0;
}

int tegra_offload_pcm_write(struct snd_soc_platform *platform, unsigned int reg,
		unsigned int val)
{
	return 0;
}

static struct snd_soc_platform_driver tegra_offload_platform = {
	.ops		= &tegra_pcm_ops,
	.compr_ops	= &tegra_compr_ops,
	.pcm_new	= tegra_offload_pcm_new,
	.pcm_free	= tegra_offload_pcm_free,
	.probe		= tegra_offload_pcm_probe,
	.read		= tegra_offload_pcm_read,
	.write		= tegra_offload_pcm_write,

	.dapm_widgets	= tegra_offload_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tegra_offload_widgets),
	.dapm_routes	= graph,
	.num_dapm_routes	= ARRAY_SIZE(graph),
};

static struct snd_soc_dai_driver tegra_offload_dai[] = {
	[PCM_OFFLOAD_DAI] = {
		.name = "tegra-offload-pcm",
		.id = 0,
		.playback = {
			.stream_name = "offload-pcm-playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "offload-pcm-capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	[COMPR_OFFLOAD_DAI] = {
		.name = "tegra-offload-compr",
		.id = 0,
		.compress_dai = 1,
		.playback = {
			.stream_name = "offload-compr-playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	}
};

static const struct snd_soc_component_driver tegra_offload_component = {
	.name		= "tegra-offload",
};

static int tegra_offload_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug("tegra30_avp_pcm platform probe started\n");

	ret = snd_soc_register_platform(&pdev->dev, &tegra_offload_platform);
	if (ret) {
		dev_err(&pdev->dev, "Could not register platform: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_component(&pdev->dev, &tegra_offload_component,
			tegra_offload_dai, ARRAY_SIZE(tegra_offload_dai));
	if (ret) {
		dev_err(&pdev->dev, "Could not register component: %d\n", ret);
		goto err_unregister_platform;
	}
	pr_info("tegra_offload_platform probe successfull.");
	return 0;

err_unregister_platform:
	snd_soc_unregister_platform(&pdev->dev);
err:
	return ret;
}

static int tegra_offload_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id tegra_offload_of_match[] = {
	{ .compatible = "nvidia,tegra-offload", },
	{},
};

static struct platform_driver tegra_offload_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra_offload_of_match,
	},
	.probe = tegra_offload_probe,
	.remove = tegra_offload_remove,
};
module_platform_driver(tegra_offload_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra offload platform driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_offload_of_match);
