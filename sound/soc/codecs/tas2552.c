/*
 * tas2552.c -- TAS2552 ALSA SoC Audio driver
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "tas2552.h"

static struct reg_default tas2552_reg[] = {
	{ 0x00, 0x00 }, /* 00 Device Status */
	{ 0x01, 0x22 }, /* 01 Config1 */
	{ 0x02, 0xEF }, /* 02 Config2 */
	{ 0x03, 0x80 }, /* 03 Config3 */
	{ 0x04, 0x00 }, /* 04 DOUT Tristate */
	{ 0x05, 0x00 }, /* 05 Serial IF Control1 */
	{ 0x06, 0x00 }, /* 06 Serial IF Control2 */
	{ 0x07, 0xC0 }, /* 07 Output Data */
	{ 0x08, 0x10 }, /* 08 PLL Control1 */
	{ 0x09, 0x00 }, /* 09 PLL Control2 */
	{ 0x0A, 0x00 }, /* 0A PLL Control3 */
	{ 0x0B, 0x8F }, /* 0B Bat Track Inf Point */
	{ 0x0C, 0x80 }, /* 0C Bat Track Slope Control */
	{ 0x0D, 0xBE }, /* 0D Limit LVL Control */
	{ 0x0E, 0x08 }, /* 0E Limit Attk Rate */
	{ 0x0F, 0x04 }, /* 0F Limit Release Rate */
	{ 0x10, 0x00 }, /* 10 Limit Integ CNT Control */
	{ 0x11, 0x01 }, /* 11 PDM Config */
	{ 0x12, 0x00 }, /* 12 PGA Gain */
	{ 0x13, 0x40 }, /* 13 Class D Edge Rate */
	{ 0x14, 0x00 }, /* 14 Boost Auto Pass Thru */
	{ 0x16, 0x08 }, /* 16 Version Number */
	{ 0x19, 0x00 }, /* 19 Vbat Data */
};

static bool tas2552_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2552_REG_00_DEVICE_STATUS:
	case TAS2552_REG_01_CONFIG1:
	case TAS2552_REG_02_CONFIG2:
	case TAS2552_REG_03_CONFIG3:
	case TAS2552_REG_04_DOUT_TRISTATE_MODE:
	case TAS2552_REG_05_SERIAL_IF_CONTROL1:
	case TAS2552_REG_06_SERIAL_IF_CONTROL2:
	case TAS2552_REG_07_OUTPUT_DATA:
	case TAS2552_REG_08_PLL_CONTROL1:
	case TAS2552_REG_09_PLL_CONTROL2:
	case TAS2552_REG_0A_PLL_CONTROL3:
	case TAS2552_REG_0B_BAT_TRACK_INF_POINT:
	case TAS2552_REG_0C_BAT_TRACK_SLOPE_CONTROL:
	case TAS2552_REG_0D_LIMIT_LVL_CONTROL:
	case TAS2552_REG_0E_LIMIT_ATTK_RATE:
	case TAS2552_REG_0F_LIMIT_RELEASE_RATE:
	case TAS2552_REG_10_LIMIT_INTEG_CNT_CONTROL:
	case TAS2552_REG_11_PDM_CONFIG:
	case TAS2552_REG_12_PGA_GAIN:
	case TAS2552_REG_13_CLASS_D_EDGE_RATE:
	case TAS2552_REG_14_BOOST_AUTO_PASS_THRU:
		return true;
	default:
		return false;
	}
}

static bool tas2552_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS2552_REG_00_DEVICE_STATUS:
	case TAS2552_REG_01_CONFIG1:
	case TAS2552_REG_02_CONFIG2:
	case TAS2552_REG_03_CONFIG3:
	case TAS2552_REG_04_DOUT_TRISTATE_MODE:
	case TAS2552_REG_05_SERIAL_IF_CONTROL1:
	case TAS2552_REG_06_SERIAL_IF_CONTROL2:
	case TAS2552_REG_07_OUTPUT_DATA:
	case TAS2552_REG_08_PLL_CONTROL1:
	case TAS2552_REG_09_PLL_CONTROL2:
	case TAS2552_REG_0A_PLL_CONTROL3:
	case TAS2552_REG_0B_BAT_TRACK_INF_POINT:
	case TAS2552_REG_0C_BAT_TRACK_SLOPE_CONTROL:
	case TAS2552_REG_0D_LIMIT_LVL_CONTROL:
	case TAS2552_REG_0E_LIMIT_ATTK_RATE:
	case TAS2552_REG_0F_LIMIT_RELEASE_RATE:
	case TAS2552_REG_10_LIMIT_INTEG_CNT_CONTROL:
	case TAS2552_REG_11_PDM_CONFIG:
	case TAS2552_REG_12_PGA_GAIN:
	case TAS2552_REG_13_CLASS_D_EDGE_RATE:
	case TAS2552_REG_14_BOOST_AUTO_PASS_THRU:
	case TAS2552_REG_16_VERSION_NUMBER:
	case TAS2552_REG_19_VBAT_DATA:
		return true;
	default:
		return false;
	}
}

static struct snd_soc_dai_ops tas2552_dai_ops = {
};

static struct snd_soc_dai_driver tas2552_dai[] = {
	{
		.name = "tas2552-SmartPA",
		.playback = {
			.stream_name = "SmartPA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = 0,
			.formats = 0,
		},
		.ops = &tas2552_dai_ops,
	},
};

static int tas2552_start_config(struct tas2552_priv *tas2552)
{
	int ret;

	ret = regmap_update_bits(tas2552->regmap, TAS2552_REG_01_CONFIG1,
			TAS2552_SWS_MASK, 0x1);

	return ret;
}

static int tas2552_finish_config(struct tas2552_priv *tas2552)
{
	int ret;

	ret = regmap_update_bits(tas2552->regmap, TAS2552_REG_01_CONFIG1,
			TAS2552_SWS_MASK, 0x0);

	return ret;
}

static int tas2552_init(struct tas2552_priv *tas2552)
{
	int ret;

	/* Software shutdown to start configuration */
	ret = tas2552_start_config(tas2552);

	regmap_write(tas2552->regmap, TAS2552_REG_01_CONFIG1, 0x12);
	regmap_write(tas2552->regmap, TAS2552_REG_08_PLL_CONTROL1, 0x10);
	regmap_write(tas2552->regmap, TAS2552_REG_03_CONFIG3, 0x5D);
	regmap_write(tas2552->regmap, TAS2552_REG_04_DOUT_TRISTATE_MODE, 0x00);
	regmap_write(tas2552->regmap, TAS2552_REG_05_SERIAL_IF_CONTROL1, 0x00);
	regmap_write(tas2552->regmap, TAS2552_REG_06_SERIAL_IF_CONTROL2, 0x00);
	regmap_write(tas2552->regmap, TAS2552_REG_07_OUTPUT_DATA, 0xC8);
	regmap_write(tas2552->regmap, TAS2552_REG_09_PLL_CONTROL2, 0x00);
	regmap_write(tas2552->regmap, TAS2552_REG_0A_PLL_CONTROL3, 0x00);
	regmap_write(tas2552->regmap, TAS2552_REG_12_PGA_GAIN, 0x15);
	regmap_write(tas2552->regmap, TAS2552_REG_0D_LIMIT_LVL_CONTROL, 0xC0);
	regmap_write(tas2552->regmap, TAS2552_REG_0E_LIMIT_ATTK_RATE, 0x20);
	regmap_write(tas2552->regmap, TAS2552_REG_02_CONFIG2, 0xCA);
	regmap_write(tas2552->regmap, TAS2552_REG_01_CONFIG1, 0x10);

	/* End software shutdown to finish configuration */
	ret |= tas2552_finish_config(tas2552);

	return 0;
}

static int tas2552_probe(struct snd_soc_codec *codec)
{
	struct tas2552_priv *tas2552 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;
	int ret;

	tas2552->codec = codec;
	codec->control_data = tas2552->regmap;

	ret = regmap_read(tas2552->regmap,
			TAS2552_REG_16_VERSION_NUMBER, &reg);
	if (ret < 0) {
		dev_err(codec->dev, "Cannot read device version: %d\n", ret);
		goto err_access;
	}

	reg &= TAS2552_SILICON_VER_MASK;
	reg >>= TAS2552_SILICON_VER_SHIFT;
	dev_info(codec->dev, "TAS2552 Version Number = %#x\n", reg);

	if (reg == TAS2552_VERSION) {
		tas2552->devtype = TAS2552;
	} else {
		dev_err(codec->dev, "Unrecognized device %#x\n", reg);
		ret = -1;
		goto err_access;
	}

	ret = tas2552_init(tas2552);
	if (ret < 0) {
		dev_err(codec->dev, "Initialization failed: %d\n", ret);
		goto err_access;
	}

	dev_info(codec->dev, "TAS2552 codec probed\n");

err_access:
	return ret;
}

static int tas2552_remove(struct snd_soc_codec *codec)
{
	return 0;
}

#if CONFIG_PM
static int tas2552_soc_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int tas2552_soc_resume(struct snd_soc_codec *codec)
{
	return 0;
}
#else
#define tas2552_soc_suspend NULL
#define tas2552_soc_resume NULL
#endif

static struct snd_soc_codec_driver soc_codec_dev_tas2552 = {
	.probe = tas2552_probe,
	.remove = tas2552_remove,
	.suspend = tas2552_soc_suspend,
	.resume = tas2552_soc_resume,
};

static const struct regmap_config tas2552_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = TAS2552_MAX_REGISTER,
	.reg_defaults = tas2552_reg,
	.num_reg_defaults = ARRAY_SIZE(tas2552_reg),
	.volatile_reg = tas2552_volatile_register,
	.readable_reg = tas2552_readable_register,
	.cache_type = REGCACHE_RBTREE,
};

static int tas2552_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct tas2552_priv *tas2552;
	int ret;

	tas2552 = kzalloc(sizeof(struct tas2552_priv), GFP_KERNEL);
	if (tas2552 == NULL)
		return -ENOMEM;

	tas2552->devtype = id->driver_data;
	i2c_set_clientdata(i2c, tas2552);
	tas2552->control_data = i2c;

	tas2552->regmap = regmap_init_i2c(i2c, &tas2552_regmap);
	if (IS_ERR(tas2552->regmap)) {
		ret = PTR_ERR(tas2552->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap: %d\n", ret);
		goto err_enable;
	}

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_tas2552, tas2552_dai,
			ARRAY_SIZE(tas2552_dai));

	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register codec: %d\n", ret);
		regmap_exit(tas2552->regmap);
	}

	dev_info(&i2c->dev, "TAS2552 I2C registered\n");

err_enable:
	return ret;
}

static int tas2552_i2c_remove(struct i2c_client *client)
{
	struct tas2552_priv *tas2552 = dev_get_drvdata(&client->dev);
	snd_soc_unregister_codec(&client->dev);
	regmap_exit(tas2552->regmap);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static int tas2552_pm_suspend(struct device *dev)
{
	return 0;
}

static int tas2552_pm_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops tas2552_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(tas2552_pm_suspend, tas2552_pm_resume)
};

static const struct i2c_device_id tas2552_i2c_id[] = {
	{ "tas2552", TAS2552 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2552_i2c_id);

static struct i2c_driver tas2552_i2c_driver = {
	.driver = {
		.name = "tas2552",
		.owner = THIS_MODULE,
		.pm = &tas2552_pm,
	},
	.probe = tas2552_i2c_probe,
	.remove = tas2552_i2c_remove,
	.id_table = tas2552_i2c_id,
};

module_i2c_driver(tas2552_i2c_driver);

MODULE_DESCRIPTION("ALSA SoC TAS2552 driver");
MODULE_AUTHOR("Andy Park <andyp@nvidia.com>");
MODULE_LICENSE("GPL");
