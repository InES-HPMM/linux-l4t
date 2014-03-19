/*
 * tegra_dummy_machine.c - Tegra machine ASoC driver for boards
 * that doesn't have codecs.
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

#include <linux/module.h>
#include <linux/slab.h>

#include <sound/soc.h>

#include "tegra_asoc_utils.h"

#define DRV_NAME "tegra-snd-dummy"

#define DAI_LINK_SPDIF		0
#define NUM_DAI_LINKS		1

struct tegra_dummy_machine {
	struct tegra_asoc_utils_data util_data;
	struct snd_soc_card *pcard;
};

static int tegra_dummy_machine_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_dummy_machine *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_dummy_machine *machine = snd_soc_card_get_drvdata(card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_dummy_machine_ops = {
	.hw_params = tegra_dummy_machine_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_dai_link tegra_dummy_machine_dai[NUM_DAI_LINKS] = {
	[DAI_LINK_SPDIF] = {
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra30-spdif",
		.cpu_dai_name = "tegra30-spdif",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_dummy_machine_ops,
	},
};

static struct snd_soc_card snd_soc_tegra_dummy_machine = {
	.name = "tegra-dummy",
	.owner = THIS_MODULE,
	.dai_link = tegra_dummy_machine_dai,
	.num_links = ARRAY_SIZE(tegra_dummy_machine_dai),
	.fully_routed = true,
};


static int tegra_dummy_machine_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_dummy_machine;
	struct tegra_dummy_machine *machine;
	int ret;

	machine = kzalloc(sizeof(struct tegra_dummy_machine), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_dummy_machine struct\n");
		return -ENOMEM;
	}

	machine->pcard = card;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);
	if (ret)
		goto err_free_machine;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

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
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);

	return ret;
}

static int tegra_dummy_machine_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_dummy_machine *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

	kfree(machine);

	return 0;
}

static const struct of_device_id tegra_dummy_machine_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-dummy", },
	{},
};

static struct platform_driver tegra_dummy_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_dummy_machine_of_match,
	},
	.probe = tegra_dummy_machine_driver_probe,
	.remove = tegra_dummy_machine_driver_remove,
};

static int __init tegra_dummy_machine_modinit(void)
{
	return platform_driver_register(&tegra_dummy_machine_driver);
}
module_init(tegra_dummy_machine_modinit);

static void __exit tegra_dummy_machine_modexit(void)
{
	platform_driver_unregister(&tegra_dummy_machine_driver);
}
module_exit(tegra_dummy_machine_modexit);

MODULE_AUTHOR("Arun Kannan <akannan@nvidia.com>");
MODULE_DESCRIPTION("Tegra+dummy machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
