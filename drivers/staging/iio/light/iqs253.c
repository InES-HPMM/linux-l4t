/*
 * A iio driver for the capacitive sensor IQS253.
 *
 * IIO Light driver for monitoring proximity.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irqchip/tegra.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/light/ls_sysfs.h>
#include <linux/iio/light/ls_dt.h>

/* registers */
#define SYSFLAGS		0x10
#define PROX_STATUS		0x31
#define TOUCH_STATUS		0x35
#define TARGET			0xC4
#define COMP0			0xC5
#define CH0_ATI_BASE		0xC8
#define CH1_ATI_BASE		0xC9
#define CH2_ATI_BASE		0xCA
#define CH0_PTH			0xCB
#define CH1_PTH			0xCC
#define CH2_PTH			0xCD
#define PROX_SETTINGS0		0xD1
#define PROX_SETTINGS1		0xD2
#define PROX_SETTINGS2		0xD3
#define PROX_SETTINGS3		0xD4
#define ACTIVE_CHAN		0xD5
#define LOW_POWER		0xD6
#define DYCAL_CHANS		0xD8
#define EVENT_MODE_MASK		0xD9
#define DEFAULT_COMMS_POINTER	0xDD

#define IQS253_PROD_ID		41

#define PROX_CH0		0x01
#define PROX_CH1		0x02
#define PROX_CH2		0x04

#define STYLUS_ONLY		PROX_CH0
#define PROXIMITY_ONLY		(PROX_CH1 | PROX_CH2)

#define CH0_COMPENSATION	0x55

#define PROX_TH_CH0		0x03
#define PROX_TH_CH1		0x03
#define PROX_TH_CH2		0x03

#define DISABLE_DYCAL		0x00

#define CH0_ATI_TH		0xCC
#define CH1_ATI_TH		0x40
#define CH2_ATI_TH		0x40

#define EVENT_PROX_ONLY		0x01

#define PROX_SETTING_NORMAL	0x25
#define PROX_SETTING_STYLUS	0x26

#define AUTO_ATI_DISABLE	BIT(7)
#define ATI_IN_PROGRESS		0x04

/* initial values */

#define EVENT_MODE_DISABLE_MASK	0x04
#define LTA_DISABLE		BIT(5)
#define ACF_DISABLE		BIT(4)
/* LTA always halt */
#define LTA_HALT_11		(BIT(0) | BIT(1))

#define ATI_ENABLED_MASK	0x80

#define IQS253_RESET		BIT(5)

#define ATI			0xAF

#define NUM_REG 17

struct iqs253_chip {
	struct i2c_client	*client;
	const struct i2c_device_id	*id;
	u32			rdy_gpio;
	u32			wake_gpio;
	u32			sar_gpio;
	u32			mode;
	u32			value;
	struct regulator	*vddhi;
	u32			using_regulator;

	struct workqueue_struct	*sar_wq;
	struct delayed_work	sar_dw;
};

enum mode {
	MODE_NONE = -1,
	INIT_MODE,
	FORCE_ATI_MODE,
	POST_INIT_MODE,
	NUM_MODE
};

struct reg_val_pair {
	u8 reg;
	u8 val;
};

struct reg_val_pair reg_val_map_iqs253[NUM_MODE][NUM_REG] = {
	{	/* init settings */
		{ ACTIVE_CHAN, PROXIMITY_ONLY},
		{ CH0_PTH, PROX_TH_CH0},
		{ CH1_PTH, PROX_TH_CH1},
		{ CH2_PTH, PROX_TH_CH2},
		{ TARGET, 0xE6},
		{ CH1_ATI_BASE, CH1_ATI_TH},
		{ CH2_ATI_BASE, CH2_ATI_TH},
		{ PROX_SETTINGS3, 0x05},
	},
	{	/* force on ATI */
		{ PROX_SETTINGS0, 0x57}, /* enable ATI and force auto ATI */
		{ ATI, 0x0}, /* wait for ATI to finish */
	},
	{
		{ PROX_SETTINGS1, 0x40},
		{ PROX_SETTINGS2, 0x57},
	},
};

static void iqs253_detect_comwindow(struct iqs253_chip *chip)
{
	int gpio_count = 500;
	while (gpio_get_value(chip->rdy_gpio) && gpio_count--)
		usleep_range(100, 110);
}

static int iqs253_i2c_firsthand_shake(struct iqs253_chip *iqs253_chip)
{
	int retry_count = 10;
	int ret = 0;

	pr_debug("%s :<-- HANDSHAKE START -->\n", __func__);

	do {
		gpio_direction_output(iqs253_chip->rdy_gpio, 0);
		usleep_range(12 * 1000, 12 * 1000);
		/* put to tristate */
		gpio_direction_input(iqs253_chip->rdy_gpio);
		iqs253_detect_comwindow(iqs253_chip);
	} while (gpio_get_value(iqs253_chip->rdy_gpio) && retry_count--);

	/* got the window */
	ret = i2c_smbus_write_byte_data(iqs253_chip->client,
					PROX_SETTINGS2 , 0x17);
	if (ret)
		return ret;
	usleep_range(1000, 1100);

	pr_debug("%s :<-- HANDSHAKE DONE -->\n", __func__);
	return 0;
}

static int iqs253_i2c_read_byte(struct iqs253_chip *chip, int reg)
{
	int ret = 0;
	iqs253_detect_comwindow(chip);
	ret = i2c_smbus_read_byte_data(chip->client, reg);
	usleep_range(1000, 1100);
	return ret;
}

static int iqs253_i2c_write_byte(struct iqs253_chip *chip, int reg, int val)
{
	int ret = 0;

	iqs253_detect_comwindow(chip);
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	usleep_range(1000, 1100);
	return ret;
}

static void iqs253_wait_for_ati_finish(struct iqs253_chip *iqs253_chip)
{
	int ret;
	do {
		usleep_range(10 * 1000, 10 * 1000);
		ret = iqs253_i2c_read_byte(iqs253_chip, SYSFLAGS);
	} while (ret & ATI_IN_PROGRESS);
}

/* must call holding lock */
static int iqs253_set(struct iqs253_chip *iqs253_chip)
{
	int ret = 0, i, j;
	struct reg_val_pair *reg_val_pair_map;
	int modes[NUM_MODE] = {INIT_MODE, FORCE_ATI_MODE, POST_INIT_MODE};

	for (j = 0; j < NUM_MODE; j++) {

		if (modes[j] == MODE_NONE)
			break;

		reg_val_pair_map = reg_val_map_iqs253[modes[j]];

		for (i = 0; i < NUM_REG; i++) {
			if (!reg_val_pair_map[i].reg &&
			    !reg_val_pair_map[i].val)
				continue;

			if (reg_val_pair_map[i].reg == ATI) {
				iqs253_wait_for_ati_finish(iqs253_chip);
				continue;
			}

			ret = iqs253_i2c_write_byte(iqs253_chip,
						reg_val_pair_map[i].reg,
						reg_val_pair_map[i].val);
			if (ret) {
				dev_err(&iqs253_chip->client->dev,
					"iqs253 write val:%x to reg:%x fail\n",
					reg_val_pair_map[i].val,
					reg_val_pair_map[i].reg);
				return ret;
			}
		}
	}
	return 0;
}

static void iqs253_sar_proximity_detect_work(struct work_struct *ws)
{
	int ret;
	struct iqs253_chip *chip;

	chip = container_of(ws, struct iqs253_chip, sar_dw.work);

	if (!chip->using_regulator) {
		ret = regulator_enable(chip->vddhi);
		if (ret)
			goto finish;
		chip->using_regulator = true;
	}

	ret = iqs253_i2c_read_byte(chip, PROX_STATUS);
	if (ret < 0) {
		pr_err(
		  "iqs253: read proximity status fails in func:%s at line:%d\n",
		  __func__, __LINE__);
		return;
	}

	if (ret & IQS253_RESET) {
		ret = iqs253_set(chip);
		if (ret)
			goto finish;
	}

	ret = iqs253_i2c_read_byte(chip, PROX_STATUS);
	chip->value = -1;
	if (ret >= 0) {
		ret = ret & PROXIMITY_ONLY;
		/*
		 * if both channel detect proximity => distance = 0;
		 * if one channel detects proximity => distance = 1;
		 * if no channel detects proximity => distance = 2;
		 */
		chip->value = (ret == (PROX_CH1 | PROX_CH2)) ? 0 :
							ret ? 1 : 2;
	}
	if (chip->value == -1)
		goto finish;
	/* provide input to SAR */
	if (chip->value/2)
		gpio_direction_output(chip->sar_gpio, 0);
	else
		gpio_direction_output(chip->sar_gpio, 1);

	ret = regulator_disable(chip->vddhi);
	if (ret)
		goto finish;
	chip->using_regulator = false;

finish:
	queue_delayed_work(chip->sar_wq, &chip->sar_dw, msecs_to_jiffies(1000));
}

static int iqs253_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct iqs253_chip *iqs253_chip;
	struct iio_dev *indio_dev;
	int rdy_gpio = -1, wake_gpio = -1, sar_gpio = -1;

	rdy_gpio = of_get_named_gpio(client->dev.of_node, "rdy-gpio", 0);
	if (rdy_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (!gpio_is_valid(rdy_gpio))
		return -EINVAL;

	wake_gpio = of_get_named_gpio(client->dev.of_node, "wake-gpio", 0);
	if (wake_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (!gpio_is_valid(wake_gpio))
		return -EINVAL;

	sar_gpio = of_get_named_gpio(client->dev.of_node, "sar-gpio", 0);
	if (sar_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	ret = gpio_request_one(sar_gpio, GPIOF_OUT_INIT_LOW, NULL);
	if (ret < 0)
		return -EINVAL;

	indio_dev = iio_device_alloc(sizeof(*iqs253_chip));
	if (!indio_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, indio_dev);
	iqs253_chip = iio_priv(indio_dev);

	iqs253_chip->client = client;
	iqs253_chip->id = id;
	iqs253_chip->mode = MODE_NONE;
	iqs253_chip->vddhi = devm_regulator_get(&client->dev, "vddhi");
	if (IS_ERR(iqs253_chip->vddhi)) {
		dev_err(&client->dev,
			"devname:%s func:%s regulator vddhi not found\n",
			id->name, __func__);
		goto err_regulator_get;
	}

	ret = gpio_request(rdy_gpio, "iqs253");
	if (ret) {
			dev_err(&client->dev,
			"devname:%s func:%s regulator vddhi not found\n",
			id->name, __func__);
		goto err_gpio_request;
	}
	iqs253_chip->rdy_gpio = rdy_gpio;
	iqs253_chip->wake_gpio = wake_gpio;
	iqs253_chip->sar_gpio = sar_gpio;

	ret = regulator_enable(iqs253_chip->vddhi);
	if (ret) {
		dev_err(&client->dev,
			"devname:%s func:%s regulator enable failed\n",
			id->name, __func__);
		goto err_gpio_request;
	}

	ret = iqs253_i2c_firsthand_shake(iqs253_chip);
	if (ret < 0) {
		dev_err(&client->dev,
			"devname:%s func:%s first i2c handshake failed\n",
			id->name, __func__);
		goto err_gpio_request;
	}

	ret = iqs253_i2c_read_byte(iqs253_chip, 0);
	if (ret != IQS253_PROD_ID) {
		dev_err(&client->dev,
			"devname:%s func:%s device not present\n",
			id->name, __func__);
		goto err_gpio_request;
	}

	iqs253_chip->sar_wq = create_freezable_workqueue("iqs253_sar");
	if (!iqs253_chip->sar_wq) {
		dev_err(&iqs253_chip->client->dev, "unable to create work queue\n");
		goto err_gpio_request;
	}

	INIT_DELAYED_WORK(&iqs253_chip->sar_dw,
				iqs253_sar_proximity_detect_work);

	queue_delayed_work(iqs253_chip->sar_wq, &iqs253_chip->sar_dw, 0);

	dev_info(&client->dev, "devname:%s func:%s line:%d probe success\n",
			id->name, __func__, __LINE__);

	return 0;

err_gpio_request:
	if (iqs253_chip->sar_wq)
		destroy_workqueue(iqs253_chip->sar_wq);
err_regulator_get:
	iio_device_free(indio_dev);

	dev_err(&client->dev, "devname:%s func:%s line:%d probe failed\n",
			id->name, __func__, __LINE__);
	return ret;
}

static int iqs253_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct iqs253_chip *chip = iio_priv(indio_dev);
	gpio_free(chip->rdy_gpio);

	if (chip->sar_wq)
		destroy_workqueue(chip->sar_wq);
	iio_device_free(indio_dev);
	return 0;
}

static void iqs253_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct iqs253_chip *chip = iio_priv(indio_dev);

	if (chip->sar_wq)
		cancel_delayed_work_sync(&chip->sar_dw);
}

static const struct i2c_device_id iqs253_id[] = {
	{"iqs253", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, iqs253_id);

static const struct of_device_id iqs253_of_match[] = {
	{ .compatible = "azoteq,iqs253", },
	{ },
};
MODULE_DEVICE_TABLE(of, iqs253_of_match);

static struct i2c_driver iqs253_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "iqs253",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(iqs253_of_match),
	},
	.probe = iqs253_probe,
	.remove = iqs253_remove,
	.shutdown = iqs253_shutdown,
	.id_table = iqs253_id,
};

module_i2c_driver(iqs253_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IQS253 Driver");
MODULE_AUTHOR("Sri Krishna chowdary <schowdary@nvidia.com>");
