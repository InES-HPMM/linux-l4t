/*
 * Max97236 Audio AMP driver
 *
 * Author: Ravindra Lokhande
 *         rlokhande@nvidia.com
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
 *
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <sound/core.h>
#include <sound/soc.h>

#include "max97236.h"

struct max97236 {
	struct regmap *regmap;
};

static struct reg_default max97236_default_regs[] = {
	{  0, 0x00 },
	{  1, 0x00 },
	{  2, 0x00 },
	{  4, 0x00 },
	{  5, 0x00 },
	{  7, 0xC0 },
	{  8, 0x40 },
	{  9, 0x00 },
	{ 10, 0x00 },
	{ 11, 0x90 },
	{ 12, 0x00 },
	{ 13, 0x00 },
	{ 14, 0x00 },
	{ 15, 0x00 },
	{ 16, 0x00 },
	{ 17, 0x00 },
	{ 18, 0x00 },
	{ 19, 0x00 },
	{ 20, 0x00 },
	{ 21, 0x00 },
	{ 22, 0x00 },
	{ 23, 0x00 },
	{ 24, 0x00 },
	{ 25, 0x20 },
	{ 26, 0x05 },
	{ 29, 0x00 },
	{ 30, 0x00 },
};

static int max97236_probe(struct snd_soc_codec *codec)
{
	struct max97236 *max97236 = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = max97236->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_REGMAP);
	if (ret)
		return ret;

	snd_soc_write(codec, M97236_REG_1D_ENABLE1, M97236_EN1_SHDN);
	snd_soc_write(codec, M97236_REG_1E_ENABLE2, M97236_EN2_LFTEN |
			M97236_EN2_RGHEN | M97236_EN2_FAST | 0x10);
	snd_soc_write(codec, M97236_REG_07_LEFT_VOLUME, ((M97236_LVOL_L_EQ_R |
			0x39) & (~M97236_LVOL_MUTEL)));

	return 0;
}

static const struct regmap_config max97236_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 31,
	.reg_defaults = max97236_default_regs,
	.num_reg_defaults = ARRAY_SIZE(max97236_default_regs),
	.cache_type = REGCACHE_RBTREE,
};

static struct snd_soc_codec_driver soc_codec_dev_max97236 = {
	.probe = max97236_probe,
};

static int __devinit max97236_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct max97236 *max97236;
	int err;

	max97236 = devm_kzalloc(&i2c->dev, sizeof(*max97236), GFP_KERNEL);
	if (!max97236)
		return -ENOMEM;

	i2c_set_clientdata(i2c, max97236);

	max97236->regmap = regmap_init_i2c(i2c, &max97236_i2c_regmap_config);
	if (IS_ERR(max97236->regmap)) {
		err = PTR_ERR(max97236->regmap);
		dev_err(&i2c->dev, "regmap_init() failed: %d\n", err);
		return err;
	}

	err = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_max97236,
					NULL, 0);
	if (err)
		goto err_regmap_free;

	return 0;

 err_regmap_free:
	regmap_exit(max97236->regmap);

	return err;
}

static int __devexit max97236_i2c_remove(struct i2c_client *i2c)
{
	struct max97236 *max97236 = i2c_get_clientdata(i2c);

	snd_soc_unregister_codec(&i2c->dev);
	regmap_exit(max97236->regmap);

	return 0;
}

static const struct i2c_device_id max97236_i2c_id[] = {
	{ "max97236", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max97236_i2c_id);

static struct i2c_driver max97236_i2c_driver = {
	.driver = {
		.name = "max97236",
		.owner = THIS_MODULE,
	},
	.probe = max97236_i2c_probe,
	.remove = __devexit_p(max97236_i2c_remove),
	.id_table = max97236_i2c_id,
};

static int __init max97236_init(void)
{
	return i2c_add_driver(&max97236_i2c_driver);
}
module_init(max97236_init);

static void __exit max97236_exit(void)
{
	i2c_del_driver(&max97236_i2c_driver);
}
module_exit(max97236_exit);

MODULE_AUTHOR("Ravindra Lokhande <rlokhande@nvidia.com>");
MODULE_DESCRIPTION("max97236 amplifier driver");
MODULE_LICENSE("GPL");
