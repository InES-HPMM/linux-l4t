/*
 * tegra_virt_vcm30t124_control.c - Tegra VCM30 T124 control card driver
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

#define DRV_NAME "tegra124-virt-control"

#define LINK_CPU_NAME		"tegra124-virt-xbar-control"
#define S_NAME_DAM_OUT(i)	"DAM"#i" OUT"
#define CPU_NAME_DAM(i, j)	"DAM"#i"-"#j
#define CODEC_NAME_DAM(i)	"tegra124-dam."#i
#define DAI_NAME_DAM(i)		"DAM"#i
#define DAI_NAME_OUT		"OUT"
#define DAI_NAME_IN(i)		"IN"#i

static struct snd_soc_pcm_stream default_link_params = {
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 2,
		.channels_max = 2,
};
static struct snd_soc_codec_conf tegra124_virt_control_codec_confs[] = {
	{
		.dev_name = CODEC_NAME_DAM(0),
		.name_prefix = DAI_NAME_DAM(0),
	},
	{
		.dev_name = CODEC_NAME_DAM(1),
		.name_prefix = DAI_NAME_DAM(1),
	},
	{
		.dev_name = CODEC_NAME_DAM(2),
		.name_prefix = DAI_NAME_DAM(2),
	},
};

static struct snd_soc_dai_link tegra124_virt_control_links[] = {
	{
		.name = "DAM0 IN0",
		.stream_name = "DAM0 IN0",
		.cpu_dai_name = CPU_NAME_DAM(0, 0),
		.codec_dai_name = DAI_NAME_IN(0),
		.cpu_name = LINK_CPU_NAME,
		.codec_name = CODEC_NAME_DAM(0),
		.params = &default_link_params,
	},
	{
		.name = "DAM0 IN1",
		.stream_name = "DAM0 IN1",
		.cpu_dai_name = CPU_NAME_DAM(0, 1),
		.codec_dai_name = DAI_NAME_IN(1),
		.cpu_name = LINK_CPU_NAME,
		.codec_name = CODEC_NAME_DAM(0),
		.params = &default_link_params,
	},
	{
		.name = "DAM0 OUT",
		.stream_name = "DAM0 OUT",
		.cpu_dai_name = DAI_NAME_OUT,
		.codec_dai_name = DAI_NAME_DAM(0),
		.cpu_name = CODEC_NAME_DAM(0),
		.codec_name = LINK_CPU_NAME,
		.params = &default_link_params,
	},
	{
		.name = "DAM1 IN0",
		.stream_name = "DAM1 IN0",
		.cpu_dai_name = CPU_NAME_DAM(1, 0),
		.codec_dai_name = DAI_NAME_IN(0),
		.cpu_name = LINK_CPU_NAME,
		.codec_name = CODEC_NAME_DAM(1),
		.params = &default_link_params,
	},
	{
		.name = "DAM1 IN1",
		.stream_name = "DAM1 IN1",
		.cpu_dai_name = CPU_NAME_DAM(1, 1),
		.codec_dai_name = DAI_NAME_IN(1),
		.cpu_name = LINK_CPU_NAME,
		.codec_name = CODEC_NAME_DAM(1),
		.params = &default_link_params,
	},
	{
		.name = "DAM1 OUT",
		.stream_name = "DAM1 OUT",
		.cpu_dai_name = DAI_NAME_OUT,
		.codec_dai_name = DAI_NAME_DAM(1),
		.cpu_name = CODEC_NAME_DAM(1),
		.codec_name = LINK_CPU_NAME,
		.params = &default_link_params,
	},
	{
		.name = "DAM2 IN0",
		.stream_name = "DAM2 IN0",
		.cpu_dai_name = CPU_NAME_DAM(2, 0),
		.codec_dai_name = DAI_NAME_IN(0),
		.cpu_name = LINK_CPU_NAME,
		.codec_name = CODEC_NAME_DAM(2),
		.params = &default_link_params,
	},
	{
		.name = "DAM2 IN1",
		.stream_name = "DAM2 IN1",
		.cpu_dai_name = CPU_NAME_DAM(2, 1),
		.codec_dai_name = DAI_NAME_IN(1),
		.cpu_name = LINK_CPU_NAME,
		.codec_name = CODEC_NAME_DAM(2),
		.params = &default_link_params,
	},
	{
		.name = "DAM2 OUT",
		.stream_name = "DAM2 OUT",
		.cpu_dai_name = DAI_NAME_OUT,
		.codec_dai_name = DAI_NAME_DAM(2),
		.cpu_name = CODEC_NAME_DAM(2),
		.codec_name = LINK_CPU_NAME,
		.params = &default_link_params,
	},
};

static const struct of_device_id tegra124_virt_control_of_match[] = {
	{ .compatible = "nvidia,tegra124-virt-control", },
	{},
};

static struct snd_soc_card snd_soc_tegra124_virt_control = {
	.name = "tegra-ak-ad",
	.owner = THIS_MODULE,
	.dai_link = tegra124_virt_control_links,
	.num_links = ARRAY_SIZE(tegra124_virt_control_links),
	.codec_conf = tegra124_virt_control_codec_confs,
	.num_configs = ARRAY_SIZE(tegra124_virt_control_codec_confs),
	.fully_routed = true,
};

static int tegra124_virt_control_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra124_virt_control;
	int ret = 0;

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
	}
	return ret;
}

static int tegra124_virt_control_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver tegra124_virt_control_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table =
			of_match_ptr(tegra124_virt_control_of_match),
	},
	.probe = tegra124_virt_control_driver_probe,
	.remove = tegra124_virt_control_driver_remove,
};
module_platform_driver(tegra124_virt_control_driver);

MODULE_AUTHOR("Ranjith Kannikara <rkannikara@nvidia.com>");
MODULE_DESCRIPTION("Tegra+VCM30T124 virt control machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, tegra124_virt_virt_control_of_match);
MODULE_ALIAS("platform:" DRV_NAME);
