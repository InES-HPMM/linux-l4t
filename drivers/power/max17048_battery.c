/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/max17048_battery.h>

#define MAX17048_VCELL		0x02
#define MAX17048_SOC		0x04
#define MAX17048_VER		0x08
#define MAX17048_HIBRT		0x0A
#define MAX17048_CONFIG		0x0C
#define MAX17048_OCV		0x0E
#define MAX17048_VALRT		0x14
#define MAX17048_VRESET		0x18
#define MAX17048_STATUS		0x1A
#define MAX17048_UNLOCK		0x3E
#define MAX17048_TABLE		0x40
#define MAX17048_RCOMPSEG1	0x80
#define MAX17048_RCOMPSEG2	0x90
#define MAX17048_CMD		0xFF
#define MAX17048_UNLOCK_VALUE	0x4a57
#define MAX17048_RESET_VALUE	0x5400
#define MAX17048_DELAY		1000
#define MAX17048_BATTERY_FULL	100
#define MAX17048_BATTERY_LOW	15
#define MAX17048_VERSION_NO	0x11

struct max17048_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct power_supply		ac;
	struct power_supply		usb;
	struct max17048_platform_data *pdata;

	/* State Of Connect */
	int ac_online;
	int usb_online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	int lasttime_vcell;
	int lasttime_soc;
	int lasttime_status;
	int use_usb:1;
	int use_ac:1;
	int shutdown_complete;
	struct mutex mutex;
};
struct max17048_chip *max17048_data;

static int max17048_write_word(struct i2c_client *client, int reg, u16 value)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}


	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x err %d\n", __func__, reg, ret);

	mutex_unlock(&chip->mutex);
	return ret;
}


static int max17048_write_block(const struct i2c_client *client,
		uint8_t command, uint8_t length, const uint8_t *values)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing block data to"
				"0x%02x err %d\n", __func__, command, ret);
	mutex_unlock(&chip->mutex);
	return ret;
}


static int max17048_read_word(struct i2c_client *client, int reg)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x err %d\n", __func__, reg, ret);

		mutex_unlock(&chip->mutex);
		return ret;
	} else {
		ret = (int)swab16((uint16_t)(ret & 0x0000ffff));

		mutex_unlock(&chip->mutex);
		return ret;

	}
}

/* Return value in uV */
static int max17048_get_ocv(struct max17048_chip *chip)
{
	int r;
	int reg;
	int ocv;

	r = max17048_write_word(chip->client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (r)
		return r;

	reg = max17048_read_word(chip->client, MAX17048_OCV);
	ocv = (reg >> 4) * 1250;

	r = max17048_write_word(chip->client, MAX17048_UNLOCK, 0);
	WARN_ON(r);

	return ocv;
}

static int max17048_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = max17048_get_ocv(chip);
		break;
	default:
	return -EINVAL;
	}
	return 0;
}

static int max17048_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, ac);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->ac_online;
	else
		return -EINVAL;

	return 0;
}

static int max17048_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
					struct max17048_chip, usb);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = chip->usb_online;
	else
		return -EINVAL;

	return 0;
}

static void max17048_get_vcell(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int vcell;

	vcell = max17048_read_word(client, MAX17048_VCELL);
	if (vcell < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, vcell);
	else
		chip->vcell = (uint16_t)(((vcell >> 4) * 125) / 100);
}

static void max17048_get_soc(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int soc;

	soc = max17048_read_word(client, MAX17048_SOC);
	if (soc < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, soc);
	else
		chip->soc = (uint16_t)soc >> 9;

	if (chip->soc >= MAX17048_BATTERY_FULL) {
		chip->soc = MAX17048_BATTERY_FULL;
		chip->status = POWER_SUPPLY_STATUS_FULL;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (chip->soc < MAX17048_BATTERY_LOW) {
		chip->status = chip->lasttime_status;
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		chip->status = chip->lasttime_status;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static uint16_t max17048_get_version(struct i2c_client *client)
{
	return swab16(max17048_read_word(client, MAX17048_VER));
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;

	chip = container_of(work, struct max17048_chip, work.work);

	max17048_get_vcell(chip->client);
	max17048_get_soc(chip->client);

	if (chip->vcell != chip->lasttime_vcell ||
		chip->soc != chip->lasttime_soc ||
		chip->status != chip->lasttime_status) {

		chip->lasttime_vcell = chip->vcell;
		chip->lasttime_soc = chip->soc;

		power_supply_changed(&chip->battery);
	}

	schedule_delayed_work(&chip->work, MAX17048_DELAY);
}

void max17048_battery_status(int status,
				int chrg_type)
{
	if (!max17048_data)
		return;

	max17048_data->ac_online = 0;
	max17048_data->usb_online = 0;

	if (status == progress) {
		max17048_data->status = POWER_SUPPLY_STATUS_CHARGING;
		if (chrg_type == AC)
			max17048_data->ac_online = 1;
		else if (chrg_type == USB)
			max17048_data->usb_online = 1;
	} else
		max17048_data->status = POWER_SUPPLY_STATUS_DISCHARGING;

	max17048_data->lasttime_status = max17048_data->status;
	power_supply_changed(&max17048_data->battery);
	if (max17048_data->use_usb)
		power_supply_changed(&max17048_data->usb);
	if (max17048_data->use_ac)
		power_supply_changed(&max17048_data->ac);
}
EXPORT_SYMBOL_GPL(max17048_battery_status);

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static enum power_supply_property max17048_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property max17048_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int max17048_write_rcomp_seg(struct i2c_client *client,
						uint16_t rcomp_seg)
{
	uint8_t rs1, rs2;
	int ret;
	uint8_t rcomp_seg_table[16];

	rs2 = (rcomp_seg >> 8) & 0xff;
	rs1 = rcomp_seg & 0xff;

	rcomp_seg_table[0] = rcomp_seg_table[2] = rcomp_seg_table[4] =
		rcomp_seg_table[6] = rcomp_seg_table[8] = rcomp_seg_table[10] =
			rcomp_seg_table[12] = rcomp_seg_table[14] = rs1;

	rcomp_seg_table[1] = rcomp_seg_table[3] = rcomp_seg_table[5] =
		rcomp_seg_table[7] = rcomp_seg_table[9] = rcomp_seg_table[11] =
			rcomp_seg_table[13] = rcomp_seg_table[15] = rs2;

	ret = max17048_write_block(client, MAX17048_RCOMPSEG1,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	ret = max17048_write_block(client, MAX17048_RCOMPSEG2,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int max17048_load_model_data(struct max17048_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->pdata->model_data;
	uint16_t soc_tst, ocv;
	int i, ret = 0;

	/* read OCV */
	ret = max17048_read_word(client, MAX17048_OCV);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	ocv = (uint16_t)ret;
	if (ocv == 0xffff) {
		dev_err(&client->dev, "%s: Failed in unlocking"
					"max17048 err: %d\n", __func__, ocv);
		return -1;
	}

	/* write custom model data */
	for (i = 0; i < 4; i += 1) {
		if (max17048_write_block(client,
			(MAX17048_TABLE+i*16), 16,
				&mdata->data_tbl[i*0x10]) < 0) {
			dev_err(&client->dev, "%s: error writing model data:\n",
								__func__);
			return -1;
		}
	}

	/* Write OCV Test value */
	ret = max17048_write_word(client, MAX17048_OCV, mdata->ocvtest);
	if (ret < 0)
		return ret;

	ret = max17048_write_rcomp_seg(client, mdata->rcomp_seg);
	if (ret < 0)
		return ret;

	/* Disable hibernate */
	ret = max17048_write_word(client, MAX17048_HIBRT, 0x0000);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Delay between 150ms to 600ms */
	mdelay(200);

	/* Read SOC Register and compare to expected result */
	ret = max17048_read_word(client, MAX17048_SOC);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	soc_tst = (uint16_t)ret;
	if (!((soc_tst >> 8) >= mdata->soccheck_A &&
				(soc_tst >> 8) <=  mdata->soccheck_B)) {
		dev_err(&client->dev, "%s: soc comparison failed %d\n",
					__func__, ret);
		return ret;
	} else {
		dev_info(&client->dev, "MAX17048 Custom data"
						" loading successfull\n");
	}

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
					MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* Restore OCV */
	ret = max17048_write_word(client, MAX17048_OCV, ocv);
	if (ret < 0)
		return ret;

	return ret;
}

static int max17048_initialize(struct max17048_chip *chip)
{
	uint8_t ret, config = 0;
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->pdata->model_data;

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* load model data */
	ret = max17048_load_model_data(chip);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	if (mdata->bits == 19)
		config = 32 - (mdata->alert_threshold * 2);
	else if (mdata->bits == 18)
		config = 32 - mdata->alert_threshold;
	else
		WARN_ON("Unknown mdata->bits");

	config = mdata->one_percent_alerts | config;

	ret = max17048_write_word(client, MAX17048_CONFIG,
			((mdata->rcomp << 8) | config));
	if (ret < 0)
		return ret;

	/* Voltage Alert configuration */
	ret = max17048_write_word(client, MAX17048_VALRT, mdata->valert);
	if (ret < 0)
		return ret;

	ret = max17048_write_word(client, MAX17048_VRESET, mdata->vreset);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Add delay */
	mdelay(200);
	return 0;
}

int max17048_check_battery()
{
	uint16_t version;

	if (!max17048_data)
		return -ENODEV;

	version = max17048_get_version(max17048_data->client);
	if (version != MAX17048_VERSION_NO)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL_GPL(max17048_check_battery);

#ifdef CONFIG_OF
static struct max17048_platform_data *max17048_parse_dt(struct device *dev)
{
	struct max17048_platform_data *pdata;
	struct max17048_battery_model *model_data;
	struct device_node *np = dev->of_node;
	u32 val, val_array[MAX17048_DATA_SIZE];
	int i, ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	model_data = devm_kzalloc(dev, sizeof(*model_data), GFP_KERNEL);
	if (!model_data)
		return ERR_PTR(-ENOMEM);

	pdata->model_data = model_data;

	ret = of_property_read_u32(np, "bits", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	if ((val == 18) || (val == 19))
		model_data->bits = val;

	ret = of_property_read_u32(np, "alert-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	model_data->alert_threshold = val;
	if (model_data->bits == 19) /* LSB is 0.5%, if 19-bit model. */
		model_data->alert_threshold /= 2;

	ret = of_property_read_u32(np, "one-percent-alerts", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	if (val)
		model_data->one_percent_alerts = 0x40;

	ret = of_property_read_u32(np, "valert-max", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->valert = ((val / 20) & 0xFF) << 8; /* LSB is 20mV. */

	ret = of_property_read_u32(np, "valert-min", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->valert |= (val / 20) & 0xFF; /* LSB is 20mV. */

	ret = of_property_read_u32(np, "vreset-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->vreset = ((val / 40) & 0xFE) << 8; /* LSB is 40mV. */

	ret = of_property_read_u32(np, "vreset-disable", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->vreset |= (val & 0x01) << 8;

	ret = of_property_read_u32(np, "hib-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->hibernate = (val & 0xFF) << 8;

	ret = of_property_read_u32(np, "hib-active-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->hibernate |= val & 0xFF;

	ret = of_property_read_u32(np, "rcomp", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->rcomp = val;

	ret = of_property_read_u32(np, "rcomp-seg", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->rcomp_seg = val;

	ret = of_property_read_u32(np, "soccheck-a", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->soccheck_A = val;

	ret = of_property_read_u32(np, "soccheck-b", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->soccheck_B = val;

	ret = of_property_read_u32(np, "ocvtest", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->ocvtest = val;

	ret = of_property_read_u32_array(np, "data-tbl", val_array,
					 MAX17048_DATA_SIZE);
	if (ret < 0)
		return ERR_PTR(ret);

	for (i = 0; i < MAX17048_DATA_SIZE; i++)
		model_data->data_tbl[i] = val_array[i];

	pdata->use_ac = of_property_read_bool(np, "use-ac");
	pdata->use_usb = of_property_read_bool(np, "use-usb");

	return pdata;
}
#else
static struct max17048_platform_data *max17048_parse_dt(struct device *dev)
{
	return NULL;
}
#endif /* CONFIG_OF */

static int max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max17048_chip *chip;
	int ret;
	uint16_t version;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	if (client->dev.of_node) {
		chip->pdata = max17048_parse_dt(&client->dev);
		if (IS_ERR(chip->pdata))
			return PTR_ERR(chip->pdata);
	} else {
		chip->pdata = client->dev.platform_data;
		if (!chip->pdata)
			return -ENODATA;
	}

	chip->ac_online = 0;
	chip->usb_online = 0;
	max17048_data = chip;
	mutex_init(&chip->mutex);
	chip->shutdown_complete = 0;
	i2c_set_clientdata(client, chip);

	version = max17048_check_battery();
	if (version < 0) {
		ret = -ENODEV;
		goto error2;
	}
	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver 0x%x\n", version);

	ret = max17048_initialize(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Error: Initializing fuel-gauge\n");
		goto error2;
	}

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17048_get_property;
	chip->battery.properties	= max17048_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17048_battery_props);
	chip->status			= POWER_SUPPLY_STATUS_DISCHARGING;

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error2;
	}

	if (chip->pdata->use_ac) {
		chip->ac.name		= "maxim-ac";
		chip->ac.type		= POWER_SUPPLY_TYPE_MAINS;
		chip->ac.get_property	= max17048_ac_get_property;
		chip->ac.properties	= max17048_ac_props;
		chip->ac.num_properties	= ARRAY_SIZE(max17048_ac_props);

		ret = power_supply_register(&client->dev, &chip->ac);
		if (ret) {
			dev_err(&client->dev, "failed: power supply register\n");
			goto error1;
		}
	}

	if (chip->pdata->use_usb) {
		chip->usb.name		= "maxim-usb";
		chip->usb.type		= POWER_SUPPLY_TYPE_USB;
		chip->usb.get_property	= max17048_usb_get_property;
		chip->usb.properties	= max17048_usb_props;
		chip->usb.num_properties = ARRAY_SIZE(max17048_usb_props);

		ret = power_supply_register(&client->dev, &chip->usb);
		if (ret) {
			dev_err(&client->dev, "failed: power supply register\n");
			goto error;
		}
	}

	INIT_DEFERRABLE_WORK(&chip->work, max17048_work);
	schedule_delayed_work(&chip->work, MAX17048_DELAY);

	return 0;
error:
	power_supply_unregister(&chip->ac);
error1:
	power_supply_unregister(&chip->battery);
error2:
	mutex_destroy(&chip->mutex);

	return ret;
}

static int max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	power_supply_unregister(&chip->usb);
	power_supply_unregister(&chip->ac);
	cancel_delayed_work_sync(&chip->work);
	mutex_destroy(&chip->mutex);

	return 0;
}

static void max17048_shutdown(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->work);
	mutex_lock(&chip->mutex);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->mutex);

}

#ifdef CONFIG_PM

static int max17048_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	cancel_delayed_work_sync(&chip->work);
	ret = max17048_write_word(client, MAX17048_HIBRT, 0xffff);
	if (ret < 0) {
		dev_err(&client->dev, "failed in entering hibernate mode\n");
		return ret;
	}

	return 0;
}

static int max17048_resume(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;
	struct max17048_battery_model *mdata = chip->pdata->model_data;

	ret = max17048_write_word(client, MAX17048_HIBRT, mdata->hibernate);
	if (ret < 0) {
		dev_err(&client->dev, "failed in exiting hibernate mode\n");
		return ret;
	}

	schedule_delayed_work(&chip->work, MAX17048_DELAY);

	return 0;
}

#else

#define max17048_suspend NULL
#define max17048_resume NULL

#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id max17048_dt_match[] = {
	{ .compatible = "maxim,max17048" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17048_dt_match);
#endif

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
		.of_match_table = of_match_ptr(max17048_dt_match),
	},
	.probe		= max17048_probe,
	.remove		= max17048_remove,
	.suspend	= max17048_suspend,
	.resume		= max17048_resume,
	.id_table	= max17048_id,
	.shutdown	= max17048_shutdown,
};

static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
subsys_initcall(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_AUTHOR("Chandler Zhang <chazhang@nvidia.com>");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
