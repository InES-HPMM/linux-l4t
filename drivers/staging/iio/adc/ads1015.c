/*
 * Driver for TI,ADS1015 ADC
 *
 * Copyright (c) 2014 - 2015, NVIDIA Corporation. All rights reserved.
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
#define ADS_1015_MAX_RETRIES			6
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

struct ads1015_adc_threshold_ranges {
	int lower;
	int upper;
};

struct ads1015_property {
	bool is_continuous_mode;
	int pga;		/* Programmable gain amplifier */
	int sampling_freq;
	int comparator_mode;
	int comparator_queue;
	int channel_number;
	int tolerance;
	struct ads1015_adc_threshold_ranges *threshold_ranges;
	int num_conditions;
	int num_retries;
	int retry_delay_ms;
};

struct ads1015 {
	struct device *dev;
	struct i2c_client *clients;
	struct regmap *rmap;
	struct ads1015_property adc_prop;
	struct ads1015_property adc_os_prop;
	u16 config;
	u16 os_config;
	bool continuous_dt_node;
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

	if (adc->continuous_dt_node && (chan != adc->adc_prop.channel_number))
		reg_val = adc->os_config;
	else
		reg_val = adc->config;

	reg_val |= (ADS1015_SINGLE_SHOT_MODE << ADS1015_OPERATION_MODE_SHIFT);
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
		udelay(200);
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

static int ads1015_threshold_update(struct ads1015 *adc, int *adc_val)
{
	struct ads1015_property *adc_prop = &adc->adc_prop;
	struct ads1015_adc_threshold_ranges *thres_range;
	u16 reg_val = 0;
	int ret;
	int lower, upper;
	s16 lower_s16;
	s16 upper_s16;
	int i;
	int stable_count = 0;
	int last_val;
	int cur_val;
	int max_retry = adc_prop->num_retries;
	bool in_valid_range = false;

	ret = ads1015_read(adc->rmap, ADS1015_CONVERSION_REG, &reg_val);
	if (ret < 0) {
		dev_err(adc->dev, "CONVERSION_REG Read failed %d\n", ret);
		return ret;
	}

	last_val = (s16)reg_val;
	last_val = (last_val / 16);

	do {
		mdelay(adc_prop->retry_delay_ms);
		ret = ads1015_read(adc->rmap, ADS1015_CONVERSION_REG, &reg_val);
		if (ret < 0) {
			dev_err(adc->dev,
				"CONVERSION Read failed %d\n", ret);
			return ret;
		}

		cur_val = (s16)reg_val;
		cur_val = (cur_val / 16);

		if (cur_val >= (last_val - adc_prop->tolerance) &&
			cur_val <= (last_val + adc_prop->tolerance))
			stable_count++;
		else
			stable_count = 0;

		if (stable_count == 3)
			break;
		last_val = cur_val;
	} while (--max_retry > 0);

	lower = -2047;
	upper = 2047;
	for (i = 0; i < adc_prop->num_conditions; i++) {
		thres_range = &adc_prop->threshold_ranges[i];
		if (thres_range->lower <= cur_val &&
			thres_range->upper >= cur_val) {
			lower = thres_range->lower;
			upper = thres_range->upper;
			in_valid_range = true;
			break;
		}

		if (thres_range->upper < cur_val) {
			if (lower < thres_range->upper)
				lower = thres_range->upper;
		}

		if (thres_range->lower > cur_val) {
			if (upper > thres_range->lower)
				upper = thres_range->lower;
		}
	}

	*adc_val = cur_val;
	lower_s16 = (s16) (lower * 16);
	upper_s16 = (s16)(upper * 16);
	if (!in_valid_range) {
		dev_info(adc->dev,
			"Not in valid threshold range. Val: %d\n", cur_val);
		WARN_ON(1);
	}

	ret = ads1015_write(adc->rmap, ADS1015_LO_THRESH_REG, (u16)lower_s16);
	if (ret < 0) {
		dev_err(adc->dev, "LO_THRESH_REG write failed %d\n", ret);
		return ret;
	}

	ret = ads1015_write(adc->rmap, ADS1015_HI_THRESH_REG, (u16)upper_s16);
	if (ret < 0) {
		dev_err(adc->dev, "HI_THRESH_REG write failed %d\n", ret);
		return ret;
	}

	dev_info(adc->dev,
		"ADC TH values LTV: UTV: RV: 0x%04x 0x%04x 0x%04x\n",
		(u16)lower_s16, (u16)upper_s16, reg_val);
	return 0;
}

static int ads1015_read_raw(struct iio_dev *iodev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct ads1015 *adc = iio_priv(iodev);
	int ret = 0;
	u16 reg_val = 0;
	int rval;

	if (mask != IIO_CHAN_INFO_RAW) {
		dev_err(adc->dev, "Invalid mask 0x%08lx\n", mask);
		return -EINVAL;
	}

	mutex_lock(&iodev->mlock);

	if ((adc->adc_prop.is_continuous_mode) &&
		(chan->channel == adc->adc_prop.channel_number)) {
		ret = ads1015_threshold_update(adc, val);
		goto done;
	}

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
	rval = (s16)reg_val;
	*val = (rval >> 4);

	/* if device is enabled in cotinuous mode set it here again */
	if (adc->adc_prop.is_continuous_mode) {
		ret = ads1015_write(adc->rmap, ADS1015_CONFIG_REG, adc->config);
		if (ret < 0) {
			dev_err(adc->dev, "CONFIG reg write failed %d\n", ret);
			return ret;
		}
	}
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
	int ret, val;
	u16 reg_val = 0;
	u16 reg_os_val = 0;

	/* Set PGA */
	reg_val |= (adc->adc_prop.pga << ADS1015_AMPLIFIER_GAIN_SHIFT);
	reg_val |= (adc->adc_prop.channel_number <<
				ADS1015_INPUT_MULTIPLEXER_SHIFT);

	/* Set operation mode */
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
	reg_val |= (ADS1015_COMPARATOR_NON_LATCHING <<
				ADS1015_COMPARATOR_LATCHING_SHIFT);
	/* Assert after one conversion */
	reg_val |= (adc->adc_prop.comparator_queue <<
				ADS1015_COMPARATOR_QUEUE_SHIFT);

	ret = ads1015_write(adc->rmap, ADS1015_CONFIG_REG, reg_val);
	if (ret < 0) {
		dev_err(adc->dev, "CONFIG reg write failed %d\n", ret);
		return ret;
	}
	adc->config = reg_val;

	if (!adc->adc_prop.is_continuous_mode)
		return 0;

	ret = ads1015_start_conversion(adc, adc->adc_prop.channel_number);
	if (ret < 0) {
		dev_err(adc->dev, "Start conversion failed %d\n", ret);
		return ret;
	}
	ret = ads1015_threshold_update(adc, &val);
	if (ret < 0) {
		dev_err(adc->dev, "Continuous mode config failed: %d\n", ret);
		return ret;
	}

	/* Configure in continuous mode */
	adc->config &= ~(1 << ADS1015_OPERATION_MODE_SHIFT);
	adc->config |= (ADS1015_COTINUOUS_MODE << ADS1015_OPERATION_MODE_SHIFT);

	ret = ads1015_write(adc->rmap, ADS1015_CONFIG_REG, adc->config);
	if (ret < 0) {
		dev_err(adc->dev, "CONFIG reg write failed %d\n", ret);
		return ret;
	}

	if (adc->continuous_dt_node) {
		/* Set PGA */
		reg_os_val |= (adc->adc_os_prop.pga <<
					ADS1015_AMPLIFIER_GAIN_SHIFT);
		reg_os_val |= (adc->adc_os_prop.channel_number <<
					ADS1015_INPUT_MULTIPLEXER_SHIFT);
		/* Set operation mode */
		reg_os_val |= (ADS1015_SINGLE_SHOT_MODE <<
					ADS1015_OPERATION_MODE_SHIFT);
		/* Set Sampling frequence rate */
		reg_os_val |= (adc->adc_os_prop.sampling_freq <<
					ADS1015_DATA_RATE_SHIFT);
		/* Set Comparator mode */
		reg_os_val |= (adc->adc_os_prop.comparator_mode <<
					ADS1015_COMPARATOR_MODE_SHIFT);
		/* Set Comparator polarity to active low */
		reg_os_val |= (ADS1015_COMPARATOR_POLARITY_ACTIVITY_LOW <<
					ADS1015_COMPARATOR_POLARITY_SHIFT);
		/* Set Comparator latch */
		reg_os_val |= (ADS1015_COMPARATOR_NON_LATCHING <<
					ADS1015_COMPARATOR_LATCHING_SHIFT);
		/* Assert after one conversion */
		reg_os_val |= (adc->adc_os_prop.comparator_queue <<
					ADS1015_COMPARATOR_QUEUE_SHIFT);

		adc->os_config = reg_os_val;
	}

	return ret;
}

static int ads1015_get_dt_data(struct ads1015 *adc, struct device_node *np)
{
	struct ads1015_property *adc_prop = &adc->adc_prop;
	struct ads1015_property *adc_os_prop = &adc->adc_os_prop;
	struct device_node *cnode;
	struct device_node *tnode;
	int ret = 0;
	u32 val, lower_adc, upper_adc;
	int nranges, i;

	ret = of_property_read_u32(np, "ti,programmable-gain-amplifier", &val);
	adc_os_prop->pga = (ret) ? ADS1015_2048MV_AMPLIFIER_GAIN : val;

	ret = of_property_read_u32(np, "sampling-frequency", &val);
	adc_os_prop->sampling_freq = (ret) ? ADS1015_920_SPS : val;

	ret = of_property_read_u32(np, "ti,comparator-mode", &val);
	adc_os_prop->comparator_mode = (ret) ? ADS1015_WINDOW_COMPARATOR : val;

	ret = of_property_read_u32(np, "ti,comparator-queue", &val);
	adc_os_prop->comparator_queue = (ret) ? ADS1015_FOUR_CONVERSIONS : val;

	adc_os_prop->is_continuous_mode = of_property_read_bool(np,
					"ti,enable-continuous-mode");

	ret = of_property_read_u32(np, "ti,maximum-retries", &val);
	adc_os_prop->num_retries = (ret) ? 0 : val;

	if (adc_os_prop->num_retries > ADS_1015_MAX_RETRIES)
		adc_os_prop->num_retries = ADS_1015_MAX_RETRIES;

	ret = of_property_read_u32(np, "ti,tolerance-val", &val);
	adc_os_prop->tolerance = (ret) ? 10 : val;

	ret = of_property_read_u32(np, "ti,delay-between-retry-ms", &val);
	adc_os_prop->retry_delay_ms = (ret) ? 10 : val;

	ret = of_property_read_u32(np, "ti,continuous-channel-number", &val);
	if (!ret) {
		adc_os_prop->channel_number = val;
	} else {
		adc_os_prop->channel_number = -EINVAL;
		if (adc_os_prop->is_continuous_mode) {
			dev_err(adc->dev,
				"Continuous channel number not available\n");
			return -EINVAL;
		}
	}

	/* If continuous mode then initialised with default */
	if (adc_os_prop->is_continuous_mode) {
		adc_prop->pga = adc_os_prop->pga;
		adc_prop->sampling_freq = adc_os_prop->sampling_freq;
		adc_prop->comparator_mode = adc_os_prop->comparator_mode;
		adc_prop->comparator_queue = adc_os_prop->comparator_queue;
		adc_prop->is_continuous_mode = adc_os_prop->is_continuous_mode;
		adc_prop->num_retries = adc_os_prop->num_retries;
		adc_prop->tolerance = adc_os_prop->tolerance;
		adc_prop->retry_delay_ms = adc_os_prop->retry_delay_ms;
		adc_prop->channel_number = adc_os_prop->channel_number;
		adc_prop->is_continuous_mode = true;
	}

	tnode = np;
	cnode = of_find_node_by_name(np, "continuous-mode");
	if (!cnode) {
		if (adc_os_prop->is_continuous_mode)
			goto parse_range;
		goto parse_done;
	}

	tnode = cnode;
	adc_prop->is_continuous_mode = true;
	adc->continuous_dt_node = true;

	ret = of_property_read_u32(cnode, "ti,programmable-gain-amplifier",
				&val);
	adc_prop->pga = (ret) ? ADS1015_2048MV_AMPLIFIER_GAIN : val;

	ret = of_property_read_u32(cnode, "sampling-frequency", &val);
	adc_prop->sampling_freq = (ret) ? ADS1015_920_SPS : val;

	ret = of_property_read_u32(cnode, "ti,comparator-mode", &val);
	adc_prop->comparator_mode = (ret) ? ADS1015_WINDOW_COMPARATOR : val;

	ret = of_property_read_u32(cnode, "ti,comparator-queue", &val);
	adc_prop->comparator_queue = (ret) ? ADS1015_FOUR_CONVERSIONS : val;

	ret = of_property_read_u32(cnode, "ti,maximum-retries", &val);
	if (!ret) {
		adc_prop->num_retries = val;
		if (adc_prop->num_retries > ADS_1015_MAX_RETRIES)
			adc_prop->num_retries = ADS_1015_MAX_RETRIES;
	}

	ret = of_property_read_u32(cnode, "ti,tolerance-val", &val);
	if (!ret)
		adc_prop->tolerance = val;

	ret = of_property_read_u32(cnode, "ti,delay-between-retry-ms", &val);
	if (!ret)
		adc_prop->retry_delay_ms = val;

	ret = of_property_read_u32(cnode, "ti,continuous-channel-number", &val);
	if (!ret) {
		adc_prop->channel_number = val;
	} else {
		dev_err(adc->dev, "Continuous channel not provided\n");
		return ret;
	}


parse_range:
	nranges = of_property_count_u32(tnode, "ti,adc-valid-threshold-ranges");
	if (nranges <= 0)
		return -EINVAL;

	nranges = (nranges  / 2);
	adc_prop->threshold_ranges = devm_kzalloc(adc->dev, nranges *
		sizeof(*adc_prop->threshold_ranges), GFP_KERNEL);
	if (!adc_prop->threshold_ranges)
		return -ENOMEM;

	for (i = 0; i < nranges; ++i) {
		lower_adc = 0;
		upper_adc = 0;

		of_property_read_u32_index(tnode,
			"ti,adc-valid-threshold-ranges", i * 2 + 0, &lower_adc);
		of_property_read_u32_index(tnode,
			"ti,adc-valid-threshold-ranges", i * 2 + 1, &upper_adc);

		adc_prop->threshold_ranges[i].lower = (int)lower_adc;
		adc_prop->threshold_ranges[i].upper = (int)upper_adc;
	};
	adc_prop->num_conditions = nranges;

parse_done:
	return 0;
}

static int ads1015_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct ads1015 *adc;
	struct device_node *node = i2c->dev.of_node;
	struct iio_dev *iodev;
	struct ads1015_property *adc_prop;
	int ret;

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
	adc->continuous_dt_node = false;

	ret = ads1015_get_dt_data(adc, node);
	if (ret < 0)
		return ret;

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
