/*
 * saf775x.c -- SAF775X Soc Audio driver
 *
 * Copyright (c) 2014-2015 NVIDIA CORPORATION.  All rights reserved.
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
#include <asm/mach-types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-tegra.h>
#include <linux/delay.h>

#include "saf775x_ioctl.h"
#include "audio_limits.h"
#define CONFIG_TMPM32X_MODULE

#ifdef CONFIG_TMPM32X_MODULE
#include <linux/gpio.h>
#include <linux/gpio-tmpm32x.h>
#endif

#define DRV_NAME	"saf775x"

#define  ENUM_EXT(xname, xhandler_get, xhandler_put,\
		xstruct, xmin, xmax, xstep, xval) \
{       .name = xname, .arr = xstruct, \
	.get = xhandler_get, .set = xhandler_put, \
	.min = xmin, .max = xmax, .val = xval, .step = xstep}

struct saf775x_controls {
	const unsigned char *name;
	int min;
	int max;
	int val;
	int step;
	int *arr;
	int (*get)(void *, struct saf775x_control_info *);
	int (*set)(void *, int *, unsigned int ,
		int, unsigned int);
};

struct saf775x_priv {
	struct mutex mutex;
	unsigned char msg_seq[16];
	void *control_data;
	struct saf775x_controls *controls;
	unsigned int num_controls;
};

static struct i2c_client *tdf8530;
static struct i2c_board_info tdf8530_info = {
	I2C_BOARD_INFO("tdf8530", 0x31),
};

/* Hard-Coding Amp for now
   TODO: Seprate driver for tdf8530*/
static int enable_tdf8530(struct i2c_client *i2c)
{
	unsigned char data[4] = {0x01, 0x10, 0x00, 0x10};
	return i2c_master_send(i2c, data, 4);
}

static int saf775x_soc_write_i2c(struct i2c_client *codec, unsigned int reg,
		unsigned int val, unsigned int reg_len, unsigned int val_len)
{
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)i2c_get_clientdata(codec);
	unsigned char *data = (unsigned char *)&saf775x->msg_seq;
	int i, ret = 0;

	mutex_lock(&saf775x->mutex);

	if (reg_len) {
		for (i = reg_len - 1; i >= 0; i--)
			*data++ = ((reg & CHAR_BIT_MASK(i)) >>
					BYTEPOS_IN_WORD(i));

	}
	if (val_len) {
		for (i = val_len - 1; i >= 0; i--)
			*data++ = ((val & CHAR_BIT_MASK(i)) >>
					BYTEPOS_IN_WORD(i));
	}
	ret = i2c_master_send(saf775x->control_data,
				saf775x->msg_seq, reg_len + val_len);
	mutex_unlock(&saf775x->mutex);

	return ret;
}

static int saf775x_soc_write_spi(struct spi_device *codec, unsigned int reg,
		unsigned int val, unsigned int reg_len, unsigned int val_len)
{
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)spi_get_drvdata(codec);
	unsigned char *data = (unsigned char *)&saf775x->msg_seq;
	int i, ret = 0;

	mutex_lock(&saf775x->mutex);

	*data++ = reg_len + val_len + 1;

	if (reg_len) {
		for (i = reg_len - 1; i >= 0; i--)
			*data++ = ((reg & CHAR_BIT_MASK(i)) >>
					BYTEPOS_IN_WORD(i));

	}
	if (val_len) {
		for (i = val_len - 1; i >= 0; i--)
			*data++ = ((val & CHAR_BIT_MASK(i)) >>
					BYTEPOS_IN_WORD(i));
	}
	ret = spi_write(saf775x->control_data,
				saf775x->msg_seq, reg_len + val_len + 1);
	mdelay(10);
	mutex_unlock(&saf775x->mutex);

	return ret;
}


static int saf775x_soc_write(void *codec, unsigned int reg,
		unsigned int val, unsigned int reg_len, unsigned int val_len)
{
	if (saf775x_get_active_if() == I2C) {
		struct i2c_client *i2c = (struct i2c_client *)codec;
		return saf775x_soc_write_i2c(i2c, reg, val, reg_len, val_len);
	} else {
		struct spi_device *spi = (struct spi_device *)codec;
		return saf775x_soc_write_spi(spi, reg, val, reg_len, val_len);
	}
}

#if defined(CONFIG_SPI_MASTER)
static int saf775x_soc_flash_image(struct spi_device *codec,
		char *image_data, unsigned int image_size)
{
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)spi_get_drvdata(codec);

	int ret = 0;

	mutex_lock(&saf775x->mutex);
	ret = spi_write(saf775x->control_data, image_data, image_size);
	mdelay(10);
	mutex_unlock(&saf775x->mutex);

	return ret;
}

static int saf775x_soc_read_status(struct spi_device *codec,
		char *status, unsigned int size)
{
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)spi_get_drvdata(codec);

	int ret = 0;
	mutex_lock(&saf775x->mutex);
	ret = spi_read(saf775x->control_data, status, size);
	mdelay(10);
	mutex_unlock(&saf775x->mutex);

	return ret;
}
#endif

int saf775x_set_default_ctrl(
		void *codec,
		int *arr,
		unsigned int reg,
		int val,
		unsigned int num_reg) {

	unsigned int *_reg = (unsigned int *)reg;
	unsigned int i = 0;

	for (i = 0; i < num_reg; i++)
		saf775x_soc_write(codec, _reg[i],
			(unsigned int)arr[(val*num_reg) + i], 3, 2);
	return 0;
}

struct saf775x_controls ctrls[] = {
ENUM_EXT("pri-vol", NULL, saf775x_set_default_ctrl, saf775x_vol_level[0], 0, 23, 1, 0),
ENUM_EXT("sec-vol", NULL, saf775x_set_default_ctrl, saf775x_vol_level[0], 0, 23, 1, 0),
ENUM_EXT("sec2-vol", NULL, saf775x_set_default_ctrl, saf775x_vol_level[0], 0, 23, 1, 0),
ENUM_EXT("pri-bal-l", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("pri-bal-r", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("sec-bal-l", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("sec-bal-l", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("sec2-bal-r", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("sec2-bal-r", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("front-fad", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("rear-fad", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("swl-fad", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("swr-fad", NULL, saf775x_set_default_ctrl, saf775x_bal_level[0], 0, 120, 10, 0),
ENUM_EXT("pri-bas", NULL, saf775x_set_default_ctrl, saf775x_bas_level[0], -14, 24, 2, 0),
ENUM_EXT("sec-bas", NULL, saf775x_set_default_ctrl, saf775x_bas_level[0], -14, 24, 2, 0),
ENUM_EXT("sec2-bas", NULL, saf775x_set_default_ctrl, saf775x_bas_level[0], -14, 24, 2, 0),
ENUM_EXT("pri-mid", NULL, saf775x_set_default_ctrl, saf775x_mid_level[0], -14, 14, 2, 0),
ENUM_EXT("sec-mid", NULL, saf775x_set_default_ctrl, saf775x_mid_level[0], -14, 14, 2, 0),
ENUM_EXT("sec2-mid", NULL, saf775x_set_default_ctrl, saf775x_mid_level[0], -14, 14, 2, 0),
ENUM_EXT("pri-tre", NULL, saf775x_set_default_ctrl, saf775x_mid_level[0], -14, 14, 2, 0),
ENUM_EXT("sec-tre", NULL, saf775x_set_default_ctrl, saf775x_mid_level[0], -14, 14, 2, 0),
ENUM_EXT("sec2-tre", NULL, saf775x_set_default_ctrl, saf775x_mid_level[0], -14, 14, 2, 0),
ENUM_EXT("front-sign-l", NULL, saf775x_set_default_ctrl, saf775x_sign_level[0], 0, 1, 1, 0),
ENUM_EXT("front-sign-r", NULL, saf775x_set_default_ctrl, saf775x_sign_level[0], 0, 1, 1, 0),
ENUM_EXT("rear-sign-l", NULL, saf775x_set_default_ctrl, saf775x_sign_level[0], 0, 1, 1, 0),
ENUM_EXT("rear-sign-r", NULL, saf775x_set_default_ctrl, saf775x_sign_level[0], 0, 1, 1, 0),
ENUM_EXT("swl-sign-l", NULL, saf775x_set_default_ctrl, saf775x_sign_level[0], 0, 1, 1, 0),
ENUM_EXT("swr-sign-r", NULL, saf775x_set_default_ctrl, saf775x_sign_level[0], 0, 1, 1, 0),
ENUM_EXT("pri-mute", NULL, saf775x_set_default_ctrl, saf775x_mute_level[0], 0, 1, 1, 0),
ENUM_EXT("sec-mute", NULL, saf775x_set_default_ctrl, saf775x_mute_level[0], 0, 1, 1, 0),
ENUM_EXT("sec2-mute", NULL, saf775x_set_default_ctrl, saf775x_mute_level[0], 0, 1, 1, 0),
ENUM_EXT("pri-mute-rise", NULL, saf775x_set_default_ctrl, saf775x_mute_att_level[0], 10, 100, 10, 10),
ENUM_EXT("sec-mute-rise", NULL, saf775x_set_default_ctrl, saf775x_mute_att_level[0], 10, 100, 10, 10),
ENUM_EXT("sec2-mute-rise", NULL, saf775x_set_default_ctrl, saf775x_mute_att_level[0], 10, 100, 10, 10),
ENUM_EXT("pri-mute-fall", NULL, saf775x_set_default_ctrl, saf775x_mute_att_level[0], 10, 100, 10, 100),
ENUM_EXT("sec-mute-fall", NULL, saf775x_set_default_ctrl, saf775x_mute_att_level[0], 10, 100, 10, 100),
ENUM_EXT("sec2-mute-fall", NULL, saf775x_set_default_ctrl, saf775x_mute_att_level[0], 10, 100, 10, 100),
};

static int saf775x_soc_set_ctrl_i2c(struct i2c_client *codec, char *ctrl_name,
		unsigned int reg, int val,
		unsigned int num_reg) {
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)i2c_get_clientdata(codec);
	unsigned int i = 0;
	struct saf775x_controls *c;

	for (i = 0; i < saf775x->num_controls; i++) {
		c = &saf775x->controls[i];
		if (!strcmp(ctrl_name, c->name)) {
			if (val > c->max)
				val = c->max;
			else if (val < c->min)
				val = c->min;
			else
			;

			c->val = val;
			val = (val - c->min)/c->step;
			c->set(codec, c->arr, reg, val, num_reg);
			return 0;
		}
	}
	return -EFAULT;
}

static int saf775x_soc_set_ctrl_spi(struct spi_device *codec, char *ctrl_name,
		unsigned int reg, int val,
		unsigned int num_reg) {
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)spi_get_drvdata(codec);
	unsigned int i = 0;
	struct saf775x_controls *c;

	for (i = 0; i < saf775x->num_controls; i++) {
		c = &saf775x->controls[i];
		if (!strcmp(ctrl_name, c->name)) {
			if (val > c->max)
				val = c->max;
			else if (val < c->min)
				val = c->min;
			else
			;

			c->val = val;
			val = (val - c->min)/c->step;
			c->set(codec, c->arr, reg, val, num_reg);
			return 0;
		}
	}
	return -EFAULT;
}

static int saf775x_soc_set_ctrl(void *codec, char *ctrl_name,
		unsigned int reg, int val,
		unsigned int num_reg) {
	if (saf775x_get_active_if() == I2C) {
		struct i2c_client *i2c = (struct i2c_client *)codec;
		return saf775x_soc_set_ctrl_i2c(i2c, ctrl_name,
					reg, val, num_reg);
	} else {
		struct spi_device *spi = (struct spi_device *)codec;
		return saf775x_soc_set_ctrl_spi(spi, ctrl_name,
					reg, val, num_reg);
	}
}


static int saf775x_soc_get_ctrl_i2c(struct i2c_client *codec,
		struct saf775x_control_info *info) {
	struct saf775x_priv *saf775x =
			(struct saf775x_priv *)i2c_get_clientdata(codec);
	unsigned int i = 0;
	struct saf775x_controls *c;

	for (i = 0; i < saf775x->num_controls; i++) {
		c = &saf775x->controls[i];
		if (!strcmp(info->name, c->name)) {
			info->min = c->min;
			info->max = c->max;
			info->val = c->val;
			info->step = c->step;

			return 0;
		}
	}
	return -EFAULT;
}

static int saf775x_soc_get_ctrl_spi(struct spi_device *codec,
		struct saf775x_control_info *info) {
	struct saf775x_priv *saf775x =
			(struct saf775x_priv *)spi_get_drvdata(codec);
	unsigned int i = 0;
	struct saf775x_controls *c;

	for (i = 0; i < saf775x->num_controls; i++) {
		c = &saf775x->controls[i];
		if (!strcmp(info->name, c->name)) {
			info->min = c->min;
			info->max = c->max;
			info->val = c->val;
			info->step = c->step;

			return 0;
		}
	}
	return -EFAULT;
}

static int saf775x_soc_get_ctrl(void *codec,
		struct saf775x_control_info *info) {
	if (saf775x_get_active_if() == I2C) {
		struct i2c_client *i2c = (struct i2c_client *)codec;
		return saf775x_soc_get_ctrl_i2c(i2c, info);
	} else {
		struct spi_device *spi = (struct spi_device *)codec;
		return saf775x_soc_get_ctrl_spi(spi, info);
	}
}

static int saf775x_soc_read_i2c(struct i2c_client *codec,
		unsigned char *val, unsigned int val_len)
{
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)i2c_get_clientdata(codec);
	int ret = 0;

	mutex_lock(&saf775x->mutex);

	ret = i2c_master_recv(saf775x->control_data,
				val, val_len);
	mutex_unlock(&saf775x->mutex);

	return ret;
}

static int saf775x_soc_read_spi(struct spi_device *codec,
		unsigned char *val, unsigned int val_len)
{
	struct saf775x_priv *saf775x =
		(struct saf775x_priv *)spi_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&saf775x->mutex);

	ret = spi_read(saf775x->control_data,
				val, val_len);
	mdelay(10);
	mutex_unlock(&saf775x->mutex);

	return ret;
}

static int saf775x_soc_read(void *codec,
		unsigned char *val, unsigned int val_len)
{
	if (saf775x_get_active_if() == I2C) {
		struct i2c_client *i2c = (struct i2c_client *)codec;
		return saf775x_soc_read_i2c(i2c, val, val_len);
	} else {
		struct spi_device *spi = (struct spi_device *)codec;
		return saf775x_soc_read_spi(spi, val, val_len);
	}
}

#ifdef CONFIG_TMPM32X_MODULE
static int saf775x_chip_reset(void)
{
	int err;
	err = gpio_request(TMPM32X_GPIO_PA7, "rst_dirana");
	if (err < 0) {
		pr_err("Err %d: reset dirana GPIO request failed\n", err);
		return err;
	}

	gpio_direction_output(TMPM32X_GPIO_PA7, 0);
	msleep(10);
	gpio_direction_output(TMPM32X_GPIO_PA7, 1);
	msleep(100);
	gpio_free(TMPM32X_GPIO_PA7);
	return 0;
}
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int saf775x_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct saf775x_priv *saf775x;
	struct saf775x_device_interfaces *devifs = saf775x_get_devifs();
	int ret = 0;

	devifs->client = client;

	saf775x = devm_kzalloc(&client->dev, sizeof(struct saf775x_priv),
			     GFP_KERNEL);
	if (!saf775x) {
		dev_err(&client->dev, "Can't allocate saf775x private struct\n");
		return -ENOMEM;
	}

	saf775x->control_data = client;
	saf775x->controls = ctrls;
	saf775x->num_controls = ARRAY_SIZE(ctrls);

	mutex_init(&saf775x->mutex);

	i2c_set_clientdata(client, saf775x);

	return 0;
}

static int saf775x_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id saf775x_i2c_id[] = {
	{
		.name = "saf775x",
		.driver_data = 0,
	},
	{ },
};
MODULE_DEVICE_TABLE(i2c, saf775x_i2c_id);

static struct i2c_driver saf775x_i2c_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= saf775x_i2c_probe,
	.remove		= saf775x_i2c_remove,
	.id_table	= saf775x_i2c_id,
};

#endif

#if defined(CONFIG_SPI_MASTER)
static int saf775x_spi_probe(struct spi_device *spi)
{
	struct saf775x_priv *saf775x;
	struct saf775x_device_interfaces *devifs = saf775x_get_devifs();
	int ret = 0;

	devifs->spi = spi;

	saf775x = devm_kzalloc(&spi->dev, sizeof(struct saf775x_priv),
				GFP_KERNEL);
	if (!saf775x) {
		dev_err(&spi->dev, "Can't allocate saf775x private struct\n");
		return -ENOMEM;
	}
	saf775x->control_data = spi;
	saf775x->controls = ctrls;
	saf775x->num_controls = ARRAY_SIZE(ctrls);

	mutex_init(&saf775x->mutex);

	spi_set_drvdata(spi, saf775x);

	return 0;
}

static int saf775x_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver saf775x_spi_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = saf775x_spi_probe,
	.remove = saf775x_spi_remove,
};
#endif

static int __init saf775x_modinit(void)
{
	struct saf775x_ioctl_ops *saf775x_ops;
	int ret = 0;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&saf775x_i2c_driver);
	if (ret != 0)
		return ret;
#endif
#if defined(CONFIG_SPI_MASTER)
	ret = spi_register_driver(&saf775x_spi_driver);
	if (ret != 0)
		return ret;
#endif

	saf775x_hwdep_create();

	tdf8530 = i2c_new_device(i2c_get_adapter(0),
							&tdf8530_info);
	if (!tdf8530) {
		pr_err(KERN_ERR "cannot get i2c device for TDF8530\n");
		return -1;
	}

	ret = enable_tdf8530(tdf8530);
	if (ret < 0) {
		pr_err(KERN_ERR "tdf8530: i2c send failure %d\n", ret);
		i2c_unregister_device(tdf8530);
		return -1;
	}

	saf775x_ops = saf775x_get_ioctl_ops();
	saf775x_ops->codec_write = saf775x_soc_write;
	saf775x_ops->codec_reset = saf775x_chip_reset;
	saf775x_ops->codec_read  = saf775x_soc_read;
	saf775x_ops->codec_set_ctrl  = saf775x_soc_set_ctrl;
	saf775x_ops->codec_get_ctrl  = saf775x_soc_get_ctrl;
#if defined(CONFIG_SPI_MASTER)
	saf775x_ops->codec_flash = saf775x_soc_flash_image;
	saf775x_ops->codec_read_status = saf775x_soc_read_status;
#endif

	return ret;
}
module_init(saf775x_modinit);

static void __exit saf775x_modexit(void)
{
	i2c_unregister_device(tdf8530);
	saf775x_hwdep_cleanup();
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&saf775x_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&saf775x_spi_driver);
#endif
}
module_exit(saf775x_modexit);

MODULE_AUTHOR("Arun S L <aruns@nvidia.com>");
MODULE_DESCRIPTION("SAF775X Soc Audio driver");
MODULE_LICENSE("GPL");
