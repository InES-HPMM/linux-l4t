/*
 * drivers/video/ds90uh949_ser.c
 * FPDLink Serializer driver
 *
 * Copyright (C) 2014 NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c/ds90uh949_ser.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regmap.h>

struct ds90uh949_rec {
	struct ds90uh949_platform_data *pdata;
	struct regmap *regmap;
};

static int ds90uh949_ser_config(struct i2c_client *client)
{
	struct ds90uh949_rec *data = i2c_get_clientdata(client);
	int err = 0;

	err = regmap_update_bits(data->regmap, DS90UH949_SER_REG_RESET,
			DS90UH949_SER_REG_RESET_DIGRESET0,
			DS90UH949_SER_REG_RESET_DIGRESET0);
	return (err < 0 ? err : 0);
}

static int ds90uh949_enable(struct i2c_client *client)
{
	struct ds90uh949_rec *data = i2c_get_clientdata(client);
	int err;

	/* Turn on serializer chip */
	if (data->pdata->en_gpio > 0)
		gpio_set_value(data->pdata->en_gpio, 1);

		mdelay(data->pdata->power_on_delay);

	err = ds90uh949_ser_config(client);
	if (err)
		pr_err("%s: failed\n", __func__);

	return err;
}

static void ds90uh949_disable(struct i2c_client *client)
{
	struct ds90uh949_rec *data = i2c_get_clientdata(client);

	/* Turn off serializer chip */
	if (data->pdata->en_gpio > 0)
		gpio_set_value(data->pdata->en_gpio, 0);
}

static struct ds90uh949_platform_data
		*of_ds90uh949_parse_platform_data(struct i2c_client *client)
{
	struct ds90uh949_platform_data *pdata;
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags flags;
	u32 temp;

	pdata = devm_kzalloc(&client->dev,
			sizeof(struct ds90uh949_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "not enough memory\n");
		devm_kfree(&client->dev, pdata);
		return NULL;
	}

	pdata->en_gpio = of_get_named_gpio_flags(np,
				"ti,enable-gpio", 0, &flags);
	pdata->en_gpio_flags = flags;

	if (!of_property_read_u32(np, "ti,power-on-delay", &temp))
		pdata->power_on_delay = temp;

	return pdata;
}

static struct regmap_config ds90uh949_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ds90uh949_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct ds90uh949_rec *data;
	struct ds90uh949_platform_data *pdata =
		client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	struct ds90uh949_platform_data *dt_pdata;
	int err;

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	if (!np && !pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENOENT;
	}

	if (np) {
		dt_pdata = of_ds90uh949_parse_platform_data(client);
		if (dt_pdata == NULL)
			goto err_out;
	} else {
		pdata = client->dev.platform_data;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);

	if (data == NULL)
		return -ENOMEM;

	if (np)
		data->pdata = dt_pdata;
	else
		data->pdata = pdata;

	data->regmap = devm_regmap_init_i2c(client, &ds90uh949_regmap_config);
	if (IS_ERR(data->regmap)) {
		err = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			err);
		goto err_release_memory;
	}

	if (data->pdata->en_gpio > 0) {
		err = gpio_request(data->pdata->en_gpio, "fpdlink");
		if (err < 0) {
			pr_err("Err %d: FPDLink Power GPIO request failed\n",
					err);
			goto err_release_memory;
		}
		if (data->pdata->en_gpio_flags & OF_GPIO_ACTIVE_LOW)
			gpio_direction_output(data->pdata->en_gpio, 0);
		else
			gpio_direction_output(data->pdata->en_gpio, 1);

	}
	mdelay(data->pdata->power_on_delay);

	i2c_set_clientdata(client, data);

	err = ds90uh949_enable(client);
	if (err)
		goto err_out;

	return 0;

err_release_memory:
	devm_kfree(&client->dev, data);
err_out:
	i2c_set_clientdata(client, NULL);
	return err;
}

static int ds90uh949_remove(struct i2c_client *client)
{
	ds90uh949_disable(client);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static int ds90uh949_suspend(struct i2c_client *client, pm_message_t message)
{
	ds90uh949_disable(client);

	return 0;
}

static int ds90uh949_resume(struct i2c_client *client)
{
	return ds90uh949_enable(client);
}


static const struct i2c_device_id ds90uh949_id[] = {
	{ "ds90uh949", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds90uh949_id);

#ifdef CONFIG_OF
static struct of_device_id ds90uh949_of_match[] = {
	{.compatible = "ti,ds90uh949", },
	{ },
};
#endif

static struct i2c_driver ds90uh949_driver = {
	.driver = {
		.name = "ds90uh949",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(ds90uh949_of_match),
#endif
	},
	.probe    = ds90uh949_probe,
	.remove   = ds90uh949_remove,
	.suspend  = ds90uh949_suspend,
	.resume   = ds90uh949_resume,
	.id_table = ds90uh949_id,
};

module_i2c_driver(ds90uh949_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ds90uh949 FPDLink Serializer driver");
MODULE_ALIAS("i2c:ds90uh949_ser");
