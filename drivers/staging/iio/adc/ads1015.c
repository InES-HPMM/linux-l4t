/*
 * Driver for TI,ADS1015 ADC
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * Author: Mallikarjun Kasoju <mkasoju@nvidia.com>
 *         Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define ADS1015_CONVERSION_REG				0x00
#define ADS1015_CONFIG_REG				0x01
#define ADS1015_LO_THRESH_REG				0x02
#define ADS1015_HI_THRESH_REG				0x03

#define ADS1015_MASK_CONV_START				BIT(15)
#define ADS1015_INPUT_MULTIPLEXER_SHIFT			12
#define ADS1015_INPUT_MULTIPLEXER_MASK			0x7000

#define ADS1015_AMPLIFIER_GAIN_SHIFT			9
#define ADS1015_6144MV_AMPLIFIER_GAIN			0
#define ADS1015_4096MV_AMPLIFIER_GAIN			1
#define ADS1015_2048MV_AMPLIFIER_GAIN			2
#define ADS1015_1024MV_AMPLIFIER_GAIN			3
#define ADS1015_0512MV_AMPLIFIER_GAIN			4
#define ADS1015_0256MV_AMPLIFIER_GAIN			5

#define ADS1015_OPERATION_MODE_SHIFT			8
#define ADS1015_COTINUOUS_MODE				0
#define ADS1015_SINGLE_SHOT_MODE			1

#define ADS1015_DATA_RATE_SHIFT				5
#define ADS1015_128_SPS					0
#define ADS1015_250_SPS					1
#define ADS1015_490_SPS					2
#define ADS1015_920_SPS					3
#define ADS1015_1600_SPS				4
#define ADS1015_2400_SPS				5
#define ADS1015_3300_SPS				6

#define ADS1015_COMPARATOR_MODE_SHIFT			4
#define ADS1015_TRADITIONAL_COMPARATOR			0
#define ADS1015_WINDOW_COMPARATOR			1

#define ADS1015_COMPARATOR_POLARITY_SHIFT		3
#define ADS1015_COMPARATOR_POLARITY_ACTIVITY_LOW	0
#define ADS1015_COMPARATOR_POLARITY_ACTIVITY_HIGH	1

#define ADS1015_COMPARATOR_LATCHING_SHIFT		2
#define ADS1015_COMPARATOR_NON_LATCHING			0
#define ADS1015_COMPARATOR_LATCHING			1

#define ADS1015_COMPARATOR_QUEUE_SHIFT			0
#define ADS1015_ONE_CONVERSION				0
#define ADS1015_TWO_CONVERSIONS				1
#define ADS1015_FOUR_CONVERSIONS			2
#define ADS1015_DISABLE_CONVERSION			3

#define ADS1015_CHANNEL_NAME(_name)	"ads1015-chan-"#_name

#define ADS1015_CHAN_IIO(chan, name)				\
{								\
	.datasheet_name = "ads1015-chan-"#name,			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
}

static const struct iio_chan_spec ads1015_iio_channel[] = {
	ADS1015_CHAN_IIO(0, "in0-in1"),
	ADS1015_CHAN_IIO(1, "in0-in3"),
	ADS1015_CHAN_IIO(2, "in1-in3"),
	ADS1015_CHAN_IIO(3, "in2-in3"),
	ADS1015_CHAN_IIO(4, "in0-gnd"),
	ADS1015_CHAN_IIO(5, "in1-gnd"),
	ADS1015_CHAN_IIO(6, "in2-gnd"),
	ADS1015_CHAN_IIO(7, "in3-gnd"),
};

static const struct i2c_device_id ads1015_i2c_id[] = {
	{ "ti,ads1015", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1015_i2c_id);

static const struct regmap_config ads1015_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 4,
};

struct ads1015_property {
	int pga;		/* Programmable gain amplifier */
	int sampling_freq;
	int comparator_mode;
};

struct ads1015 {
	struct device *dev;
	struct i2c_client *clients;
	struct regmap *rmap;
	struct ads1015_property adc_prop;
	u16 config;
};

static int ads1015_write(struct regmap *regmap, unsigned int reg, u16 val)
{
	val = swab16(val & 0xffff);
	return regmap_raw_write(regmap, reg, &val, 2);
}

static int ads1015_read(struct regmap *regmap, unsigned int reg, u16 *val)
{
	int ret;

	ret = regmap_raw_read(regmap, reg, val, 2);
	if (ret == 0)
		*val = swab16(*val & 0xffff);
	return ret;
}

static int ads1015_start_conversion(struct ads1015 *adc, int chan)
{
	u16 channel_mux_val = chan;
	u16 reg_val;
	int ret;
	int timeout = 10;

	reg_val = adc->config;
	reg_val &= ~ADS1015_INPUT_MULTIPLEXER_MASK;
	reg_val |= (channel_mux_val << ADS1015_INPUT_MULTIPLEXER_SHIFT);
	ret = ads1015_write(adc->rmap, ADS1015_CONFIG_REG, reg_val);
	if (ret < 0) {
		dev_err(adc->dev, "Config reg write failed %d\n", ret);
		return ret;
	}

	reg_val |= ADS1015_MASK_CONV_START;
	ret = ads1015_write(adc->rmap, ADS1015_CONFIG_REG, reg_val);
	if (ret < 0) {
		dev_err(adc->dev, "Config reg write failed %d\n", ret);
		return ret;
	}

	/* Wait for conversion */
	do {
		udelay(100);
		ret = ads1015_read(adc->rmap, ADS1015_CONFIG_REG, &reg_val);
		if (ret < 0) {
			dev_err(adc->dev, "Config reg read failed %d\n", ret);
			return ret;
		}
		if (reg_val & ADS1015_MASK_CONV_START)
			return 0;
	} while (--timeout > 0);
	return -ETIMEDOUT;
}

static int ads1015_read_raw(struct iio_dev *iodev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct ads1015 *adc = iio_priv(iodev);
	int ret = 0;
	u16 reg_val = 0;

	if (mask != IIO_CHAN_INFO_RAW) {
		dev_err(adc->dev, "Invalid mask 0x%08lx\n", mask);
		return -EINVAL;
	}

	mutex_lock(&iodev->mlock);

	ret = ads1015_start_conversion(adc, chan->channel);
	if (ret < 0) {
		dev_err(adc->dev, "Start conversion failed %d\n", ret);
		goto done;
	}

	ret = ads1015_read(adc->rmap, ADS1015_CONVERSION_REG, &reg_val);
	if (ret < 0) {
		dev_err(adc->dev, "Reg 0x%x Read failed, %d\n",
			ADS1015_CONVERSION_REG, ret);
		goto done;
	}
	*val = (reg_val >> 4);
done:
	mutex_unlock(&iodev->mlock);
	if (!ret)
		return IIO_VAL_INT;
	return ret;
}

static const struct iio_info ads1015_iio_info = {
	.read_raw = ads1015_read_raw,
	.driver_module = THIS_MODULE,
};

static int ads1015_configure(struct ads1015 *adc)
{
	u16 reg_val = 0;
	int ret;

	/* Set PGA */
	reg_val |= (adc->adc_prop.pga << ADS1015_AMPLIFIER_GAIN_SHIFT);
	/* Set single-shot mode */
	reg_val |= (ADS1015_SINGLE_SHOT_MODE << ADS1015_OPERATION_MODE_SHIFT);
	/* Set Sampling frequence rate */
	reg_val |= (adc->adc_prop.sampling_freq << ADS1015_DATA_RATE_SHIFT);
	/* Set Comparator mode */
	reg_val |= (adc->adc_prop.comparator_mode <<
				ADS1015_COMPARATOR_MODE_SHIFT);
	/* Set Comparator polarity to active low */
	reg_val |= (ADS1015_COMPARATOR_POLARITY_ACTIVITY_LOW <<
					ADS1015_COMPARATOR_POLARITY_SHIFT);
	/* Set Comparator latch */
	reg_val |= (ADS1015_COMPARATOR_LATCHING <<
				ADS1015_COMPARATOR_LATCHING_SHIFT);
	/* Assert after one conversion */
	reg_val |= (ADS1015_ONE_CONVERSION << ADS1015_COMPARATOR_QUEUE_SHIFT);

	ret = ads1015_write(adc->rmap, 1, reg_val);
	if (ret < 0) {
		dev_err(adc->dev, "CONFIG reg write failed %d\n", ret);
		return ret;
	}
	adc->config = reg_val;
	return ret;
}

static int ads1015_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct ads1015 *adc;
	struct device_node *node = i2c->dev.of_node;
	struct iio_dev *iodev;
	struct ads1015_property *adc_prop;
	int ret;
	u32 val;

	if (!node) {
		dev_err(&i2c->dev, "Device DT not found\n");
		return -ENODEV;
	}

	iodev = devm_iio_device_alloc(&i2c->dev, sizeof(*adc));
	if (!iodev) {
		dev_err(&i2c->dev, "iio_device_alloc failed\n");
		return -ENOMEM;
	}

	adc = iio_priv(iodev);
	adc->dev = &i2c->dev;
	i2c_set_clientdata(i2c, adc);
	adc_prop = &adc->adc_prop;

	ret = of_property_read_u32(node, "ti,programmable-gain-amplifier",
			&val);
	if (!ret)
		adc_prop->pga = val;
	else
		adc_prop->pga = ADS1015_2048MV_AMPLIFIER_GAIN;

	ret = of_property_read_u32(node, "sampling-frequency", &val);
	if (!ret)
		adc_prop->sampling_freq = val;
	else
		adc_prop->sampling_freq = ADS1015_1600_SPS;

	ret = of_property_read_u32(node, "ti,comparator-mode", &val);
	if (!ret)
		adc_prop->comparator_mode = val;
	else
		adc_prop->comparator_mode = ADS1015_TRADITIONAL_COMPARATOR;

	adc->rmap = devm_regmap_init_i2c(i2c, &ads1015_regmap_config);
	if (IS_ERR(adc->rmap)) {
		ret = PTR_ERR(adc->rmap);
		dev_err(adc->dev, "regmap_init failed %d\n", ret);
		return ret;
	}

	iodev->name = "ads1015";
	iodev->dev.parent = &i2c->dev;
	iodev->info = &ads1015_iio_info;
	iodev->modes = INDIO_DIRECT_MODE;
	iodev->channels = ads1015_iio_channel;
	iodev->num_channels = ARRAY_SIZE(ads1015_iio_channel);
	ret = devm_iio_device_register(&i2c->dev, iodev);
	if (ret < 0) {
		dev_err(adc->dev, "iio_device_register() failed: %d\n", ret);
		return ret;
	}

	ret = ads1015_configure(adc);
	if (ret < 0) {
		dev_err(adc->dev, "ADC configuration failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static const struct of_device_id ads1015_of_match[] = {
	{ .compatible = "ads1015", },
	{},
};

struct i2c_driver ads1015_driver = {
	.probe = ads1015_probe,
	.driver = {
		.name = "ads1015",
		.owner = THIS_MODULE,
		.of_match_table = ads1015_of_match,
	},
	.id_table = ads1015_i2c_id,
};

module_i2c_driver(ads1015_driver);

MODULE_DESCRIPTION("ads1015 driver");
MODULE_AUTHOR("Mallikarjun Kasoju <mkasoju@nvidia.com>");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_ALIAS("i2c:ads1015");
MODULE_LICENSE("GPL v2");
