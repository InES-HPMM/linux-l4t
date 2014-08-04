/*
 * tegra_es755.c - Tegra machine ASoC driver for boards using
 * EarSmart codec.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

#include <asm/mach-types.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/pm_runtime.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/gpio-tegra.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/audience/es755.h"
#include "../codecs/tas2552.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"

#define DRV_NAME "tegra-snd-es755"

#define DAI_LINK_VOICE		0
#define NUM_DAI_LINKS		1

extern int g_is_call_mode;

const char *tegra_es755_i2s_dai_name[TEGRA30_NR_I2S_IFC] = {
	"tegra30-i2s.0",
	"tegra30-i2s.1",
	"tegra30-i2s.2",
	"tegra30-i2s.3",
	"tegra30-i2s.4",
};

#define GPIO_HP_DET		BIT(0)

struct tegra_es755 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_asoc_platform_data *pdata;
	int gpio_requested;
	int is_call_mode;
	int is_device_bt;
	struct codec_config codec_info[NUM_I2S_DEVICES];
	int clock_enabled;
	struct snd_soc_card *pcard;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
	struct regulator *bat_reg;
	struct regulator *io_reg;
	struct regulator *acore_reg;
	struct regulator *dcore_reg;
};

void tegra_asoc_enable_clocks(void);
void tegra_asoc_disable_clocks(void);

static int tegra_es755_call_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_es755_call_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_es755 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->is_call_mode;

	return 0;
}

static int tegra_es755_call_mode_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_es755 *machine = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new = ucontrol->value.integer.value[0];
	int codec_index;
	unsigned int i;
	int uses_voice_codec;

	pr_info("ALSA:%s\n", __func__);

	if (machine->is_call_mode == is_call_mode_new)
		return 0;

	machine->is_device_bt = 0;

	if (machine->is_device_bt) {
		codec_index = BT_SCO;
		uses_voice_codec = 0;
	} else {
		codec_index = VOICE_CODEC;
		uses_voice_codec = 0;
	}

	machine->codec_info[codec_index].rate = 48000;
	machine->codec_info[codec_index].channels = 2;

	if (is_call_mode_new) {
		if (machine->codec_info[codec_index].rate == 0 ||
			machine->codec_info[codec_index].channels == 0)
				return -EINVAL;

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 1;

		tegra_asoc_utils_tristate_dap(
			machine->codec_info[codec_index].i2s_id, false);
		tegra_asoc_utils_tristate_dap(
			machine->codec_info[BASEBAND].i2s_id, false);

		if (machine->is_device_bt)
			tegra30_make_bt_voice_call_connections(
				&machine->codec_info[codec_index],
				&machine->codec_info[BASEBAND],
				uses_voice_codec);
		else
			tegra30_make_voice_call_connections(
				&machine->codec_info[codec_index],
				&machine->codec_info[BASEBAND],
				uses_voice_codec);
	} else {

		if (machine->is_device_bt)
			tegra30_break_bt_voice_call_connections(
				&machine->codec_info[codec_index],
				&machine->codec_info[BASEBAND],
				uses_voice_codec);
		else
			tegra30_break_voice_call_connections(
				&machine->codec_info[codec_index],
				&machine->codec_info[BASEBAND],
				uses_voice_codec);

		tegra_asoc_utils_tristate_dap(
			machine->codec_info[codec_index].i2s_id, true);
		tegra_asoc_utils_tristate_dap(
			machine->codec_info[BASEBAND].i2s_id, true);

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 0;
	}

	machine->is_call_mode = is_call_mode_new;
	g_is_call_mode = machine->is_call_mode;

	return 1;
}

struct snd_kcontrol_new tegra_es755_call_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Call Mode Switch",
	.private_value = 0xffff,
	.info = tegra_es755_call_mode_info,
	.get = tegra_es755_call_mode_get,
	.put = tegra_es755_call_mode_put,
};

static int tegra_set_dam_cif(int dam_ifc,
	int out_srate, int out_channels, int out_bit_size,
	int src_on, int src_srate, int src_channels, int src_bit_size)
{
	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHOUT, out_srate);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN1, out_srate);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		out_channels, out_bit_size, out_channels, 32);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		out_channels, out_bit_size, out_channels, 32);

	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
	if (src_on) {
		tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_srate);
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 32);
	}

	return 0;
}

#ifdef USE_FULLY_ROUTE
static const struct snd_soc_dapm_widget tegra_es755_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Earpiece"),
	SND_SOC_DAPM_OUTPUT("Port A.0"),
	SND_SOC_DAPM_OUTPUT("Port A.1"),
	SND_SOC_DAPM_OUTPUT("Port B.0"),
	SND_SOC_DAPM_OUTPUT("Port B.1"),
	SND_SOC_DAPM_OUTPUT("Port C.0"),
	SND_SOC_DAPM_OUTPUT("Port C.1"),
};

static const struct snd_soc_dapm_route tegra_es755_audio_map[] = {
	{"Earpiece", NULL, "EP"},
	{"Port A.0", NULL, "PCM PORT A.0"},
	{"Port A.1", NULL, "PCM PORT A.1"},
	{"Port B.0", NULL, "PCM PORT B.0"},
	{"Port B.1", NULL, "PCM PORT B.1"},
	{"Port C.0", NULL, "PCM PORT C.0"},
	{"Port C.1", NULL, "PCM PORT C.1"},
};

static const struct snd_kcontrol_new tegra_es755_controls[] = {
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Port A"),
	SOC_DAPM_PIN_SWITCH("Port B"),
	SOC_DAPM_PIN_SWITCH("Port C"),
};
#endif

/* Headphone jack */
static struct snd_soc_jack tegra_es755_hp_jack;

#ifdef CONFIG_SWITCH
static struct switch_dev tegra_es755_headset_switch = {
	.name = "h2w",
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

/* Headphone jack detection gpios */
static struct snd_soc_jack_gpio tegra_es755_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

static int tegra_es755_jack_notifier(struct notifier_block *self,
	unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;
	static bool button_pressed;

	if (button_pressed) {
		button_pressed = false;
		return NOTIFY_OK;
	}

	if (action & (SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2)) {
		button_pressed = true;
		return NOTIFY_OK;
	}

	if (jack == &tegra_es755_hp_jack) {
		machine->jack_status &= ~SND_JACK_HEADPHONE;
		machine->jack_status &= ~SND_JACK_MICROPHONE;
		machine->jack_status |= (action & SND_JACK_HEADSET);
	}

	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
		pr_info("%s: SND_JACK_HEADPHONE\n", __func__);
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		pr_info("%s: SND_JACK_HEADSET\n", __func__);
		state = BIT_HEADSET;
		break;
	case SND_JACK_MICROPHONE:
		pr_info("%s: SND_JACK_MICROPHONE\n", __func__);
		/* mic: would not report */
	default:
		pr_info("%s: SND_JACK_NO_HEADSET\n", __func__);
		state = BIT_NO_HEADSET;
	}

	pr_info("%s: Report switch_state\n", __func__);
	switch_set_state(&tegra_es755_headset_switch, state);

	return NOTIFY_OK;
}

static struct notifier_block tegra_es755_jack_detect_nb = {
	.notifier_call = tegra_es755_jack_notifier,
};
#else
/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin tegra_es755_hs_jack_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif

static int tegra_es755_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int ret = 0;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	pr_info("%s: I2S id: %d\n", __func__, i2s->id);

	if (i2s->id == machine->codec_info[VOICE_CODEC].i2s_id)
		i2s->is_dam_used = true;

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_es755_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		tegra_es755_hp_jack_gpio.invert =
			!pdata->gpio_hp_det_active_high;
		ret = snd_soc_jack_new(codec, "Headphone Jack",
			       (SND_JACK_HEADSET | SND_JACK_BTN_0 |
			       SND_JACK_BTN_1 | SND_JACK_BTN_2),
			       &tegra_es755_hp_jack);
		if (ret) {
			pr_err("failed to create a jack for es755\n");
			return ret;
		}

		ret = snd_jack_set_key(tegra_es755_hp_jack.jack, SND_JACK_BTN_0,
				KEY_MEDIA);
		if (ret < 0)
			pr_err("Failed to set KEY_MEDIA: %d\n", ret);

		ret = snd_jack_set_key(tegra_es755_hp_jack.jack, SND_JACK_BTN_1,
				KEY_VOLUMEUP);
		if (ret < 0)
			pr_err("Failed to set KEY_VOLUMEUP: %d\n", ret);

		ret = snd_jack_set_key(tegra_es755_hp_jack.jack, SND_JACK_BTN_2,
				KEY_VOLUMEDOWN);
		if (ret < 0)
			pr_err("Failed to set KEY_VOLUMEDOWN: %d\n", ret);
#ifdef CONFIG_SWITCH
		snd_soc_jack_notifier_register(&tegra_es755_hp_jack,
						&tegra_es755_jack_detect_nb);
#else /* gpio based headset detection */
		snd_soc_jack_add_pins(&tegra_es755_hp_jack,
					ARRAY_SIZE(tegra_es755_hs_jack_pins),
					tegra_es755_hs_jack_pins);
#endif
		snd_soc_jack_add_gpios(&tegra_es755_hp_jack,
					1, &tegra_es755_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;

		es755_detect(codec, &tegra_es755_hp_jack);
	}

	/* Add call mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
		snd_ctl_new1(&tegra_es755_call_mode_control, machine));

	return ret;
}

static int tegra_es755_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(card);
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	pr_info("ALSA:%s\n", __func__);

	tegra_asoc_utils_tristate_dap(i2s->id, false);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!machine->is_call_mode)
			tegra30_ahub_set_rx_cif_source(
				i2s->playback_i2s_cif,
				i2s->playback_fifo_cif);
		else {
			struct codec_config *codec_info;
			/*dam configuration*/
			if (!i2s->dam_ch_refcount)
				i2s->dam_ifc =
					tegra30_dam_allocate_controller();
			if (i2s->dam_ifc < 0)
				return i2s->dam_ifc;
			tegra30_dam_allocate_channel(i2s->dam_ifc,
				TEGRA30_DAM_CHIN1);
			i2s->dam_ch_refcount++;

			tegra30_dam_enable_clock(i2s->dam_ifc);
			codec_info = &machine->codec_info[VOICE_CODEC];
			tegra_set_dam_cif(i2s->dam_ifc,
				codec_info->rate,
				codec_info->channels,
				codec_info->bitsize,
				0, 0, 0, 0);

			tegra30_ahub_set_rx_cif_source(
			  TEGRA30_AHUB_RXCIF_DAM0_RX1 + (i2s->dam_ifc*2),
			  i2s->playback_fifo_cif);

			/* make the dam tx to i2s rx connection if this is
			 * the only client using i2s for playback */
			if (i2s->playback_ref_count == 1)
				tegra30_ahub_set_rx_cif_source(
				  TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id,
				  TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->dam_ifc);

			/* enable the dam*/
			tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
		}
	} else {
		if (!machine->is_call_mode)
			tegra30_ahub_set_rx_cif_source(
				i2s->capture_fifo_cif,
				i2s->capture_i2s_cif);
		else {
			struct codec_config *codec_info;
			struct codec_config *bb_info;
			/* allocate a dam for voice call recording */
			i2s->call_record_dam_ifc =
				tegra30_dam_allocate_controller();
			if (i2s->call_record_dam_ifc < 0)
				return i2s->call_record_dam_ifc;
			pr_info("%s allocate a DAM\n", __func__);
			codec_info = &machine->codec_info[VOICE_CODEC];
			bb_info = &machine->codec_info[BASEBAND];

			tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
				TEGRA30_DAM_CHIN0_SRC);
			tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
				TEGRA30_DAM_CHIN1);
			tegra30_dam_enable_clock(i2s->call_record_dam_ifc);

			/* configure the dam */
			tegra_set_dam_cif(i2s->call_record_dam_ifc,
				codec_info->rate,
				codec_info->channels,
				codec_info->bitsize,
				1,
				bb_info->rate,
				bb_info->channels,
				bb_info->bitsize);

			/* setup the connections for voice call record */
			tegra30_ahub_unset_rx_cif_source(i2s->capture_fifo_cif);

			tegra30_ahub_set_rx_cif_source(
				TEGRA30_AHUB_RXCIF_DAM0_RX0 +
					(i2s->call_record_dam_ifc*2),
				TEGRA30_AHUB_TXCIF_I2S0_TX0 +
					bb_info->i2s_id);

			tegra30_ahub_set_rx_cif_source(
				TEGRA30_AHUB_RXCIF_DAM0_RX1 +
					(i2s->call_record_dam_ifc*2),
				TEGRA30_AHUB_TXCIF_I2S0_TX0 +
					codec_info->i2s_id);

			tegra30_ahub_set_rx_cif_source(
				i2s->capture_fifo_cif,
				TEGRA30_AHUB_TXCIF_DAM0_TX0 +
					i2s->call_record_dam_ifc);

			/* enable the dam*/
			tegra30_dam_enable(i2s->call_record_dam_ifc,
				TEGRA30_DAM_ENABLE, TEGRA30_DAM_CHIN1);
			tegra30_dam_enable(i2s->call_record_dam_ifc,
				TEGRA30_DAM_ENABLE, TEGRA30_DAM_CHIN0_SRC);
		}
	}
	return 0;
}

static void tegra_es755_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(card);

	pr_info("ALSA:%s\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!machine->is_call_mode) {
			tegra30_ahub_unset_rx_cif_source(
				i2s->playback_i2s_cif);
		} else {
			/* disable the dam*/
			tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_DISABLE,
				TEGRA30_DAM_CHIN1);

			/* disconnect the ahub connections*/
			tegra30_ahub_unset_rx_cif_source(
			  TEGRA30_AHUB_RXCIF_DAM0_RX1 + (i2s->dam_ifc*2));

			/* disable the dam and free the controller */
			tegra30_dam_disable_clock(i2s->dam_ifc);
			tegra30_dam_free_channel(i2s->dam_ifc,
				TEGRA30_DAM_CHIN1);

			i2s->dam_ch_refcount--;
			if (!i2s->dam_ch_refcount)
				tegra30_dam_free_controller(i2s->dam_ifc);
		}
	} else {
		if (!machine->is_call_mode)
			tegra30_ahub_unset_rx_cif_source(
				i2s->capture_fifo_cif);
		else {
			/* disable the dam*/
			tegra30_dam_enable(i2s->call_record_dam_ifc,
				TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN1);
			tegra30_dam_enable(i2s->call_record_dam_ifc,
				TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN0_SRC);

			/* disconnect the ahub connections*/
			tegra30_ahub_unset_rx_cif_source(
				i2s->capture_fifo_cif);
			tegra30_ahub_unset_rx_cif_source(
				TEGRA30_AHUB_RXCIF_DAM0_RX0 +
				(i2s->call_record_dam_ifc*2));
			tegra30_ahub_unset_rx_cif_source(
				TEGRA30_AHUB_RXCIF_DAM0_RX1 +
				(i2s->call_record_dam_ifc*2));

			/* free the dam channels and dam controller */
			tegra30_dam_disable_clock(i2s->call_record_dam_ifc);
			tegra30_dam_free_channel(i2s->call_record_dam_ifc,
				TEGRA30_DAM_CHIN1);
			tegra30_dam_free_channel(i2s->call_record_dam_ifc,
				TEGRA30_DAM_CHIN0_SRC);
			tegra30_dam_free_controller(i2s->call_record_dam_ifc);
		}
	}
	tegra_asoc_utils_tristate_dap(i2s->id, true);
}

static int tegra_es755_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int srate, mclk, i2s_daifmt;
	int err, rate, sample_size;
	pr_info("ALSA:%s\n", __func__);

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF;
	i2s_daifmt |= pdata->i2s_param[VOICE_CODEC].is_i2s_master ?
			SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;

	switch (pdata->i2s_param[VOICE_CODEC].i2s_mode) {
	case TEGRA_DAIFMT_I2S:
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;
		break;
	case TEGRA_DAIFMT_DSP_A:
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
		break;
	case TEGRA_DAIFMT_DSP_B:
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_B;
		break;
	case TEGRA_DAIFMT_LEFT_J:
		i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
		break;
	case TEGRA_DAIFMT_RIGHT_J:
		i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
		break;
	default:
		dev_err(card->dev, "Can't configure i2s format\n");
		return -EINVAL;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	srate = params_rate(params);
	mclk = 64 * srate;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sample_size = 32;
		break;
	default:
		return -EINVAL;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk)) {
			mclk = machine->util_data.set_mclk;
		} else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	rate = clk_get_rate(machine->util_data.clk_cdev1);

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(rtd->card);
	pr_info("ALSA:%s\n", __func__);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_es755_ops = {
	.hw_params = tegra_es755_hw_params,
	.hw_free = tegra_hw_free,
	.startup = tegra_es755_startup,
	.shutdown = tegra_es755_shutdown,
};

static struct snd_soc_dai_link tegra_es755_dai[NUM_DAI_LINKS] = {
	[DAI_LINK_VOICE] = {
		.name = "Prototype earSmart I2S PortA",
		.stream_name = "I2S-PCM PortA",
		.codec_dai_name = "earSmart-porta",
#if defined(CONFIG_SND_SOC_ES_I2C)
		.codec_name = "earSmart-codec.0-003e",
#endif
		.init = &tegra_es755_init,
		.ops = &tegra_es755_ops,
	},
};

static int tegra_es755_suspend_post(struct snd_soc_card *card)
{
	struct snd_soc_jack_gpio *gpio = &tegra_es755_hp_jack_gpio;
	int suspend_allowed = 1;

	if (suspend_allowed) {
		/* Disable the irq so that device goes to suspend */
		if (gpio_is_valid(gpio->gpio))
			disable_irq(gpio_to_irq(gpio->gpio));
	}

	return 0;
}

static int tegra_es755_resume_pre(struct snd_soc_card *card)
{
	int val;
	struct snd_soc_jack_gpio *gpio = &tegra_es755_hp_jack_gpio;
	int suspend_allowed = 1;

	if (suspend_allowed) {
		/* Convey jack status after resume and
		 * re-enable the interrupts */
		if (gpio_is_valid(gpio->gpio)) {
			val = gpio_get_value(gpio->gpio);
			val = gpio->invert ? !val : val;
			snd_soc_jack_report(gpio->jack, val, gpio->report);
			enable_irq(gpio_to_irq(gpio->gpio));
		}
	}

	return 0;
}

static struct snd_soc_card snd_soc_tegra_es755 = {
	.name = "tegra-es755",
	.owner = THIS_MODULE,
	.dai_link = tegra_es755_dai,
	.num_links = ARRAY_SIZE(tegra_es755_dai),
	.suspend_post = tegra_es755_suspend_post,
	.resume_pre = tegra_es755_resume_pre,
#ifdef USE_FULLY_ROUTE
	.controls = tegra_es755_controls,
	.num_controls = ARRAY_SIZE(tegra_es755_controls),
	.dapm_widgets = tegra_es755_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_es755_dapm_widgets),
	.dapm_routes = tegra_es755_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tegra_es755_audio_map),
	.fully_routed = true,
#endif
};

static int tegra_es755_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_es755;
	struct tegra_es755 *machine;
	struct tegra_asoc_platform_data *pdata = NULL;
	int codec_id, ret;
	int i;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	pdata = pdev->dev.platform_data;

	machine = kzalloc(sizeof(struct tegra_es755), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_es755 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;
	machine->pcard = card;
	machine->clock_enabled = 1;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);
	if (ret)
		goto err_free_machine;

#ifdef CONFIG_SWITCH
	/* Add h2w switch class support */
	ret = tegra_asoc_switch_register(&tegra_es755_headset_switch);
	if (ret < 0) {
		dev_err(&pdev->dev, "not able to register switch device\n");
		goto err_fini_utils;
	}
#endif
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	codec_id = pdata->i2s_param[VOICE_CODEC].audio_port_id;
	tegra_es755_dai[DAI_LINK_VOICE].cpu_dai_name =
		tegra_es755_i2s_dai_name[codec_id];
	tegra_es755_dai[DAI_LINK_VOICE].platform_name =
		tegra_es755_i2s_dai_name[codec_id];

	for (i = 0; i < NUM_I2S_DEVICES; i++) {
		machine->codec_info[i].i2s_id =
			pdata->i2s_param[i].audio_port_id;
		machine->codec_info[i].bitsize =
			pdata->i2s_param[i].sample_size;
		machine->codec_info[i].is_i2smaster =
			pdata->i2s_param[i].is_i2s_master;
		machine->codec_info[i].rate =
			pdata->i2s_param[i].rate;
		machine->codec_info[i].channels =
			pdata->i2s_param[i].channels;
		machine->codec_info[i].i2s_mode =
			pdata->i2s_param[i].i2s_mode;
		machine->codec_info[i].bit_clk =
			pdata->i2s_param[i].bit_clk;
	}

	card->dapm.idle_bias_off = 1;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_switch;
	}

	if (!card->instantiated) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "sound card not instantiated (%d)\n",
			ret);
		goto err_unregister_card;
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_unregister_switch:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_es755_headset_switch);
err_fini_utils:
#endif
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:

	kfree(machine);

	return ret;
}

static int tegra_es755_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_es755 *machine = snd_soc_card_get_drvdata(card);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_es755_headset_switch);
#endif
	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

	kfree(machine);

	return 0;
}

static struct platform_driver tegra_es755_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_es755_driver_probe,
	.remove = tegra_es755_driver_remove,
};

static int __init tegra_es755_modinit(void)
{
	return platform_driver_register(&tegra_es755_driver);
}
module_init(tegra_es755_modinit);

static void __exit tegra_es755_modexit(void)
{
	platform_driver_unregister(&tegra_es755_driver);
}
module_exit(tegra_es755_modexit);

MODULE_AUTHOR("Andy Park <andyp@nvidia.com>");
MODULE_DESCRIPTION("Tegra+es755 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
