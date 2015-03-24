/*
 * tegra_virt_alt_vcm30t124_pcm.c - Tegra VCM30 T124 virt alt PCM driver
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "tegra124_virt_alt_apbif.h"
#include "tegra124_virt_alt_ivc.h"

#define DRV_NAME "tegra124-virt-alt-pcm"

#define DAI_NAME(i) "AUDIO" #i
#define STREAM_NAME "playback"
#define CODEC_NAME "spdif-dit"
#define LINK_CPU_NAME   "tegra124-virt-alt-pcm"
#define CPU_DAI_NAME(i) "APBIF" #i
#define CODEC_DAI_NAME "dit-hifi"
#define PLATFORM_NAME LINK_CPU_NAME
#define MAX_APBIF_IDS	10

#define GPIO_PR0 136
#define CODEC_TO_DAP 0
#define DAP_TO_CODEC 1

static struct snd_soc_pcm_stream default_params = {
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_dai_link tegra_vcm30t124_pcm_links[] = {
	{
		/* 0 */
		.name = DAI_NAME(0),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(0),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 1 */
		.name = DAI_NAME(1),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(1),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 2 */
		.name = DAI_NAME(2),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(2),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 3 */
		.name = DAI_NAME(3),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(3),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 4 */
		.name = DAI_NAME(4),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(4),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 5 */
		.name = DAI_NAME(5),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(5),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 6 */
		.name = DAI_NAME(6),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(6),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 7 */
		.name = DAI_NAME(7),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(7),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 8 */
		.name = DAI_NAME(8),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(8),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 9 */
		.name = DAI_NAME(9),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(9),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
};

static const struct of_device_id tegra124_virt_snd_pcm_of_match[] = {
	{ .compatible = "nvidia,tegra124-virt-alt-pcm", },
	{},
};

static struct snd_soc_card snd_soc_tegra_vcm30t124_pcm = {
	.name = "tegra-virt-pcm",
	.owner = THIS_MODULE,
	.dai_link = tegra_vcm30t124_pcm_links,
	.num_links = ARRAY_SIZE(tegra_vcm30t124_pcm_links),
	.fully_routed = true,
};

static void tegra_vcm30t124_set_dai_params(
		struct snd_soc_dai_link *tegra_vcm_dai_link,
		struct snd_soc_pcm_stream *user_params,
		unsigned int dai_id)
{
	tegra_vcm_dai_link[dai_id].params = user_params;
}

static struct platform_device spdif_dit = {
	.name = "spdif-dit",
	.id = -1,
};

static int tegra_vcm30t124_pcm_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_vcm30t124_pcm;
	int ret = 0;
	int i;
	int apbif_ch_num = 0;
	int set_codec_direction = 0;
	unsigned int apbif_ch_list[MAX_APBIF_IDS];

	card->dev = &pdev->dev;

	if (!of_property_read_u32(pdev->dev.of_node,
		"configure_ad_codec", &set_codec_direction) &&
		(set_codec_direction == 1)) {
		ret = devm_gpio_request(&pdev->dev, GPIO_PR0,
						"dap_dir_control");
		if (ret) {
			dev_err(&pdev->dev, "cannot get dap_dir_control gpio\n");
			return -EINVAL;
		}
		gpio_direction_output(GPIO_PR0, CODEC_TO_DAP);
	}

	platform_device_register(&spdif_dit);

	tegra124_virt_apbif_register_component(pdev);

	if (of_property_read_u32(pdev->dev.of_node,
				"apbif_ch_num", &apbif_ch_num)) {
		dev_err(&pdev->dev, "number of apbif channels is not set\n");
		return -EINVAL;
	}

	if (of_property_read_string(pdev->dev.of_node,
		"cardname", &card->name))
			dev_warn(&pdev->dev, "Use default card name\n");

	if (apbif_ch_num > 0) {

		if (of_property_read_u32_array(pdev->dev.of_node,
						"apbif_ch_list",
						apbif_ch_list,
						apbif_ch_num)) {
			dev_err(&pdev->dev, "apbif_ch_list os not populated\n");
			return -EINVAL;
		}
		for (i = 0; i < apbif_ch_num; i++) {
			tegra_vcm30t124_set_dai_params(
						tegra_vcm30t124_pcm_links,
						NULL,
						apbif_ch_list[i]);
		}
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
	}
	return ret;
}

static int tegra_vcm30t124_pcm_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver tegra_vcm30t124_pcm_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table =
			of_match_ptr(tegra124_virt_snd_pcm_of_match),
	},
	.probe = tegra_vcm30t124_pcm_driver_probe,
	.remove = tegra_vcm30t124_pcm_driver_remove,
};
module_platform_driver(tegra_vcm30t124_pcm_driver);

MODULE_AUTHOR("Ranjith Kannikara <rkannikara@nvidia.com>");
MODULE_DESCRIPTION("Tegra+VCM30T124 virt alt pcm driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, tegra124_virt_snd_pacm_of_match);
MODULE_ALIAS("platform:" DRV_NAME);
