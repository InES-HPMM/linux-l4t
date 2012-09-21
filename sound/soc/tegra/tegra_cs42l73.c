/*
 * tegra_cs42l73.c - Tegra machine ASoC driver for boards using CS42L73 codec.
 *
 * Author: Vijay Mali <vmali@nvidia.com>
 * Copyright (C) 2011-2012, NVIDIA, Inc.
 *
 * Copyright (c) 2012, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <mach/tegra_asoc_pdata.h>
#include <mach/gpio-tegra.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/cs42l73.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#define DRV_NAME "tegra-snd-cs42l73"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)

struct tegra_cs42l73 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_asoc_platform_data *pdata;
	int gpio_requested;
	bool init_done;
	int is_call_mode;
	int is_device_bt;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	struct regulator *cdc_en;
	enum snd_soc_bias_level bias_level;
	struct snd_soc_card *pcard;
	int clock_enabled;
};

static int tegra_cs42l73_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int srate, mclk, i2s_daifmt;
	int err, rate;

	srate = params_rate(params);

	switch (srate) {
	case 8000:
	case 16000:
	case 32000:
		mclk = 12288000;
		break;
	case 11025:
		mclk = 512 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF;
	i2s_daifmt |= pdata->i2s_param[HIFI_CODEC].is_i2s_master ?
			SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;

	switch (pdata->i2s_param[HIFI_CODEC].i2s_mode) {
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

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, rate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_cs42l73_ops = {
	.hw_params = tegra_cs42l73_hw_params,
	.hw_free = tegra_hw_free,
};


/* Headset jack */
struct snd_soc_jack tegra_cs42l73_hp_jack;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin tegra_cs42l73_hs_jack_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
};

#ifdef CONFIG_SWITCH
/* Headset jack detection gpios */
static struct snd_soc_jack_gpio tegra_cs42l73_hp_jack_gpio = {
	.name = "hsdet-gpio",
	.report = SND_JACK_HEADSET, /* SND_JACK_HEADSET */
	.debounce_time = 400,
	.invert = 1,
};

enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static struct switch_dev tegra_cs42l73_headset_switch = {
	.name = "h2w",
};

static int cs42l73_report_jack(struct snd_soc_codec *codec, int jack_report)
{
	enum headset_state state = BIT_NO_HEADSET;
	unsigned int  mic_status, mic_ref , mic_intr_status;

	if (jack_report > 0) {
		/* Read the MIC Status Register*/
		mic_intr_status = snd_soc_read(codec, CS42L73_IS1);
		mic_ref = snd_soc_read(codec, CS42L73_OLMBMSDC);
		mic_status = (snd_soc_read(codec, CS42L73_IS1))
			& (MIC2_SDET);

		if (mic_status > 0)
			state = BIT_HEADSET_NO_MIC; /* FIX ME BIT_HEADSET */
		else
			state = BIT_HEADSET_NO_MIC;
	}

	switch_set_state(&tegra_cs42l73_headset_switch, state);
	return 0;
}


static void cs42l73_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{

	int jack_report;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;

	jack_report = gpio_get_value(gpio->gpio);

	if (gpio->debounce_time > 0)
		mdelay(gpio->debounce_time);

	if (gpio->invert)
		jack_report = !jack_report;

	cs42l73_report_jack(codec, jack_report);
}

/* irq handler for gpio pin */
static irqreturn_t cs42l73_gpio_handler(int irq, void *data)
{
		struct snd_soc_jack_gpio *gpio = data;

		schedule_delayed_work(&gpio->work,
					msecs_to_jiffies(gpio->debounce_time));

		return IRQ_HANDLED;
}

/* gpio work */
static void cs42l73_gpio_work(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;

	gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
	cs42l73_soc_jack_gpio_detect(gpio);
}

/*
	Add the INTn_CS42L73 -> GPIOxxx
	INTn_CS42L73 is the L73's interrupt and requires
	a different type of handling.
*/
int cs42l73_soc_jack_add_gpio(struct snd_soc_jack *jack,
				struct snd_soc_jack_gpio *gpio)
{
	int ret;
	struct snd_soc_codec *codec = jack->codec;

	if (!gpio->name) {
		pr_err("No name for gpio %d\n", gpio->gpio);
		ret = -EINVAL;
		goto undo;
	}

	ret = gpio_request(gpio->gpio, gpio->name);
	if (ret)
		goto undo;

	ret = gpio_direction_input(gpio->gpio);
	if (ret)
		goto err;

	INIT_DELAYED_WORK(&gpio->work, cs42l73_gpio_work);
		gpio->jack = jack;

	ret = request_threaded_irq(gpio_to_irq(gpio->gpio),  NULL,
			cs42l73_gpio_handler,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING,
			codec->name,
			gpio);

	if (ret)
		goto err;

	snd_soc_update_bits(codec, CS42L73_DMMCC, MCLKDIS, 0);
	snd_soc_update_bits(codec, CS42L73_PWRCTL1, PDN, 0);
	snd_soc_update_bits(codec, CS42L73_PWRCTL2, PDN_MIC2_BIAS, 0);

	/* Unmask MIC2_SDET interrupt */
	snd_soc_update_bits(codec, CS42L73_IM1, MIC2_SDET, 1);

#ifdef CONFIG_GPIO_SYSFS
	/* Expose GPIO value over sysfs for diagnostic purposes */
	gpio_export(gpio->gpio, false);
#endif
	/* Update initial jack status */
	cs42l73_soc_jack_gpio_detect(gpio);
	return 0;

err:
	gpio_free(gpio->gpio);
undo:
	snd_soc_jack_free_gpios(jack, 1, gpio);
	return ret;

}
#endif

static int tegra_cs42l73_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

/* CS42L73 widgets */
static const struct snd_soc_dapm_widget tegra_cs42l73_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_cs42l73_event_int_spk),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

/* cs42l73 Audio Map */
static const struct snd_soc_dapm_route tegra_cs42l73_audio_map[] = {
	{"Int Spk", NULL, "SPKOUT"},
	{"MIC2", NULL, "Headset Mic"},
	/* Headphone (L+R)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
};

static const struct snd_kcontrol_new tegra_cs42l73_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int tegra_cs42l73_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int ret;


	if (machine->init_done)
		return 0;

	machine->init_done = true;

/*
	machine->pcard = card;
	machine->bias_level = SND_SOC_BIAS_STANDBY;
	machine->clock_enabled = 1;

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}
*/

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret)
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
		else {
			machine->gpio_requested |= GPIO_EXT_MIC_EN;
			/* Disable ext mic; enable signal is active-low */
			gpio_direction_output(pdata->gpio_ext_mic_en, 1);
		}
	}

	machine->bias_level = SND_SOC_BIAS_STANDBY;
	machine->clock_enabled = 1;

	ret = snd_soc_add_card_controls(card, tegra_cs42l73_controls,
			ARRAY_SIZE(tegra_cs42l73_controls));
	if (ret < 0)
		return ret;

	/* Add cs42l73 specific widgets */
	ret = snd_soc_dapm_new_controls(dapm,
			tegra_cs42l73_dapm_widgets,
			ARRAY_SIZE(tegra_cs42l73_dapm_widgets));
	if (ret)
		return ret;

	/* Set up cs42l73 specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm,
		tegra_cs42l73_audio_map,
		ARRAY_SIZE(tegra_cs42l73_audio_map));

		snd_soc_dapm_sync(dapm);

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		/* Headset and button jack detection */
		tegra_cs42l73_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		ret = snd_soc_jack_new(codec, "CS42L73 Audio Jack",
				SND_JACK_HEADSET, &tegra_cs42l73_hp_jack);
		if (ret) {
			pr_err("jack creation failed\n");
			return ret;
		}

		ret = snd_soc_jack_add_pins(&tegra_cs42l73_hp_jack,
				ARRAY_SIZE(tegra_cs42l73_hs_jack_pins),
				tegra_cs42l73_hs_jack_pins);
		if (ret) {
			pr_err("adding jack pins failed\n");
			return ret;
		}

		ret = cs42l73_soc_jack_add_gpio(&tegra_cs42l73_hp_jack,
						&tegra_cs42l73_hp_jack_gpio);
		if (ret) {
			pr_err("adding jack GPIO failed\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_DET;
	}

	ret = tegra_asoc_utils_register_ctls(&machine->util_data);
	if (ret < 0)
		return ret;

	/* FIXME: Calculate automatically based on DAPM routes? */
	/*snd_soc_dapm_nc_pin(dapm, "LOUTL");
	snd_soc_dapm_nc_pin(dapm, "LOUTR"); */

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link tegra_cs42l73_dai[] = {
	{
		.name = "CS42L73",
		.stream_name = "VSP Playback Record",
		.codec_name = "cs42l73.0-004a",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra30-i2s.1",
		.codec_dai_name = "cs42l73-vsp",
		.init = tegra_cs42l73_init,
		.ops = &tegra_cs42l73_ops,
	},
};

/*static int tegra_cs42l73_resume_pre(struct snd_soc_card *card)
{
	int val;
	struct snd_soc_jack_gpio *gpio = &tegra_cs42l73_hp_jack_gpio;

	if (gpio_is_valid(gpio->gpio)) {
		val = gpio_get_value(gpio->gpio);
		val = gpio->invert ? !val : val;
		snd_soc_jack_report(gpio->jack, val, gpio->report);
	}

	return 0;
} */

static int tegra_cs42l73_set_bias_level(struct snd_soc_card *card,
	struct snd_soc_dapm_context *dapm, enum snd_soc_bias_level level)
{
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level == SND_SOC_BIAS_OFF &&
		level != SND_SOC_BIAS_OFF && (!machine->clock_enabled)) {
		machine->clock_enabled = 1;
		tegra_asoc_utils_clk_enable(&machine->util_data);
		machine->bias_level = level;
	}

	return 0;
}

static int tegra_cs42l73_set_bias_level_post(struct snd_soc_card *card,
	struct snd_soc_dapm_context *dapm, enum snd_soc_bias_level level)
{
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level != SND_SOC_BIAS_OFF &&
		level == SND_SOC_BIAS_OFF && machine->clock_enabled) {
		machine->clock_enabled = 0;
		tegra_asoc_utils_clk_disable(&machine->util_data);
	}

	machine->bias_level = level;

	return 0 ;
}

static struct snd_soc_card snd_soc_tegra_cs42l73 = {
	.name = "tegra-cs42l73",
	.owner = THIS_MODULE,
	.dai_link = tegra_cs42l73_dai,
	.num_links = ARRAY_SIZE(tegra_cs42l73_dai),
/*	.resume_pre = tegra_cs42l73_resume_pre, */
	.set_bias_level = tegra_cs42l73_set_bias_level,
	.set_bias_level_post = tegra_cs42l73_set_bias_level_post,
};

static __devinit int tegra_cs42l73_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_cs42l73;
	struct tegra_cs42l73 *machine;
	struct tegra_asoc_platform_data *pdata;
	int ret;
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	if (pdata->codec_name)
		card->dai_link->codec_name = pdata->codec_name;

	if (pdata->codec_dai_name)
		card->dai_link->codec_dai_name = pdata->codec_dai_name;

	machine = kzalloc(sizeof(struct tegra_cs42l73), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_cs42l73 struct\n");
		return -ENOMEM;
	}

	if (gpio_is_valid(pdata->gpio_ldo1_en)) {
		ret = gpio_request(pdata->gpio_ldo1_en, "cs42l73");
		if (ret)
			dev_err(&pdev->dev, "Fail gpio_request AUDIO_LDO1\n");

		ret = gpio_direction_output(pdata->gpio_ldo1_en, 0);
		if (ret)
			dev_err(&pdev->dev, "Fail gpio_direction AUDIO_LDO1\n");

		msleep(200);
	}

	machine->pdata = pdata;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);
	if (ret)
		goto err_free_machine;

	machine->spk_reg = regulator_get(&pdev->dev, "vdd_spk");
	if (IS_ERR(machine->spk_reg)) {
		dev_info(&pdev->dev, "No speaker regulator found\n");
		machine->spk_reg = 0;
	}

	machine->dmic_reg = regulator_get(&pdev->dev, "vdd_mic");
	if (IS_ERR(machine->dmic_reg)) {
		dev_info(&pdev->dev, "No digital mic regulator found\n");
		machine->dmic_reg = 0;
	}

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = switch_dev_register(&tegra_cs42l73_headset_switch);
	if (ret < 0)
		goto err_fini_utils;
#endif

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);
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

	ret = tegra_asoc_utils_set_parent(&machine->util_data,
				pdata->i2s_param[HIFI_CODEC].is_i2s_master);
	if (ret) {
		dev_err(&pdev->dev, "tegra_asoc_utils_set_parent failed (%d)\n",
			ret);
		goto err_unregister_card;
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_unregister_switch:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_cs42l73_headset_switch);
err_fini_utils:
#endif
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;

}

static int __devexit tegra_cs42l73_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_cs42l73 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_cs42l73_hp_jack,
					1,
					&tegra_cs42l73_hp_jack_gpio);
	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);
	machine->gpio_requested = 0;

	if (machine->spk_reg)
		regulator_put(machine->spk_reg);
	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);

	if (machine->cdc_en) {
		regulator_disable(machine->cdc_en);
		regulator_put(machine->cdc_en);
	}

	if (gpio_is_valid(pdata->gpio_ldo1_en)) {
		gpio_set_value(pdata->gpio_ldo1_en, 0);
		gpio_free(pdata->gpio_ldo1_en);
	}

	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

	kfree(machine);

	return 0;
}

static struct platform_driver tegra_cs42l73_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_cs42l73_driver_probe,
	.remove = __devexit_p(tegra_cs42l73_driver_remove),
};

static int __init tegra_cs42l73_modinit(void)
{
	return platform_driver_register(&tegra_cs42l73_driver);
}
module_init(tegra_cs42l73_modinit);

static void __exit tegra_cs42l73_modexit(void)
{
	platform_driver_unregister(&tegra_cs42l73_driver);
}
module_exit(tegra_cs42l73_modexit);

MODULE_AUTHOR("Vijay Mali <vmali@nvidia.com>");
MODULE_DESCRIPTION("Tegra+CS42L73 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
