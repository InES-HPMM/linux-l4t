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

#include "saf775x_ioctl.h"
#define CONFIG_TMPM32X_MODULE

#ifdef CONFIG_TMPM32X_MODULE
#include <linux/gpio.h>
#include <linux/gpio-tmpm32x.h>
#endif

#define DRV_NAME	"saf775x"

struct saf775x_priv {
	struct mutex mutex;
	unsigned char msg_seq[8];
	void *control_data;
	struct i2c_client *tdf8530;
};

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


static int saf775x_soc_write(struct i2c_client *codec, unsigned int reg,
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

static int saf775x_soc_read(struct i2c_client *codec,
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
	struct saf775x_ioctl_ops *saf775x_ops;
	int ret = 0;

	saf775x = devm_kzalloc(&client->dev, sizeof(struct saf775x_priv),
			     GFP_KERNEL);
	if (!saf775x) {
		dev_err(&client->dev, "Can't allocate saf775x private struct\n");
		return -ENOMEM;
	}
	saf775x->control_data = client;
	mutex_init(&saf775x->mutex);

	i2c_set_clientdata(client, saf775x);

	saf775x_hwdep_create(client);
	saf775x_ops = saf775x_get_ioctl_ops();
	saf775x_ops->codec_write = saf775x_soc_write;
	saf775x_ops->codec_reset = saf775x_chip_reset;
	saf775x_ops->codec_read  = saf775x_soc_read;

	saf775x->tdf8530 = i2c_new_device(i2c_get_adapter(0),
				&tdf8530_info);
	if (!saf775x->tdf8530) {
		dev_err(&client->dev, "cannot get i2c device for TDF8530\n");
		return -1;
	}

	ret = enable_tdf8530(saf775x->tdf8530);
	if (ret < 0) {
		dev_err(&client->dev, "tdf8530: i2c send failure %d\n", ret);
		i2c_unregister_device(saf775x->tdf8530);
		return -1;
	}
	return 0;
}

static int saf775x_i2c_remove(struct i2c_client *client)
{
	struct saf775x_priv *saf775x;
	saf775x = (struct saf775x_priv *)i2c_get_clientdata(client);
	i2c_unregister_device(saf775x->tdf8530);

	saf775x_hwdep_cleanup();
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

module_i2c_driver(saf775x_i2c_driver);
#endif

MODULE_AUTHOR("Arun S L <aruns@nvidia.com>");
MODULE_DESCRIPTION("SAF775X Soc Audio driver");
MODULE_LICENSE("GPL");
