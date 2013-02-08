/*
 * A iio driver for the light sensor MAX44005.
 *
 * IIO Light driver for monitoring ambient light intensity in lux and proximity
 * ir.
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define INT_STATUS_REG_ADDR	0x00
#define MAIN_CONF_REG_ADDR	0x01
#define AMB_CONF_REG_ADDR	0x02
#define PROX_CONF_REG_ADDR	0x03
#define AMB_CLEAR_HIGH_ADDR	0x04
#define PROX_HIGH_ADDR		0x10
#define AMB_TEMP_HIGH_ADDR	0x12

#define MAX_SHDN_ENABLE		0x08
#define MAX_SHDN_DISABLE	0x00

#define MODE_SHIFT		4
#define MODE_CLEAR_ONLY		0x0
#define MODE_CLEAR_IR		0x1
#define MODE_CRGB		0x2
#define MODE_CLEAR_PROX		0x3
#define MODE_ALL		0x4
#define MODE_PROX_ONLY		0x5

#define COMP_ENABLE		0x40
#define TEMP_ENABLE		0x20
#define TEMP_DISABLE		0xDF

#define AMB_PGA_1x		0x00
#define AMB_PGA_4x		0x01
#define AMB_PGA_16x		0x02
#define AMB_PGA_256x		0x03

#define LED_DRV_SHIFT		4
#define LED_DRV_STRENGTH		110 /* mA */

#define POWER_ON_DELAY		20 /* 20ms */

#define MAX44005_SYSFS_SHOW(reg_addr, nbytes)				 \
	do {								 \
		int ret;						 \
		int value;						 \
		struct iio_dev *indio_dev = dev_to_iio_dev(dev);	 \
		struct max44005_chip *chip = iio_priv(indio_dev);	 \
		mutex_lock(&chip->lock);				 \
		ret = max44005_read(chip, &value, reg_addr, nbytes);	 \
		if (ret < 0) {						 \
			mutex_unlock(&chip->lock);			 \
			return sprintf(buf, "-1");			 \
		}							 \
		mutex_unlock(&chip->lock);				 \
		return sprintf(buf, "%d", value);			 \
	} while (0);							 \

/* clear is needed for both temp and clear */
#define CLEAR_ENABLED		(chip->using_temp || chip->using_als)

#define USING_CLEAR		(chip->using_als)

#define PROXIMITY_ENABLED	(chip->using_proximity)

#define TEMP_ENABLED		(chip->using_temp)

enum {
	CHIP = 0,
	LED
};

static struct class *sensor_class;

struct max44005_chip {
	struct i2c_client	*client;
	struct mutex		lock;

	struct regulator	*supply[2];
	bool			power_utilization[2];

	bool			using_als;
	bool			using_proximity;
	bool			using_temp;

	bool			is_standby;
	int			shutdown_complete;
};

static int max44005_read(struct max44005_chip *chip, int *rval, u8 reg_addr,
				int nbytes)
{
	u8 val[2];
	int ret;

	if (chip->supply[CHIP] && !regulator_is_enabled(chip->supply[CHIP]))
		return -EINVAL;

	if (chip->shutdown_complete)
		return -EINVAL;

	ret = i2c_smbus_read_i2c_block_data(chip->client, reg_addr,
						nbytes, val);

	if (ret != nbytes) {
		dev_err(&chip->client->dev, "[MAX44005] i2c_read_failed" \
			"in func: %s\n", __func__);
		if (ret < 0)
			return ret;
		return -EINVAL;
	}

	*rval = val[0];
	if (nbytes == 2)
		*rval = ((*rval) << 8) | val[1];
	return 0;
}

static int max44005_write(struct max44005_chip *chip, u8 val, u8 reg_addr)
{
	int ret;

	if (chip->supply[CHIP] && !regulator_is_enabled(chip->supply[CHIP]))
		return -EINVAL;

	if (chip->shutdown_complete)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(chip->client, reg_addr, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "[MAX44005] i2c_write_failed" \
			"in func: %s\n", __func__);
	}
	return ret;
}

/* assumes chip is power on */
static void max44005_standby(struct max44005_chip *chip, bool shutdown)
{
	int ret = 0;

	if (chip->is_standby == shutdown)
		return;

	if (shutdown == chip->power_utilization[CHIP])
		return;

	if (shutdown == false) {
		ret = max44005_write(chip, MAX_SHDN_DISABLE,
					INT_STATUS_REG_ADDR);
		if (!ret)
			chip->is_standby = false;
	} else {
		ret = max44005_write(chip, MAX_SHDN_ENABLE,
					INT_STATUS_REG_ADDR);
		if (!ret)
			chip->is_standby = true;
	}
}

static bool set_main_conf(struct max44005_chip *chip, int mode)
{
	return max44005_write(chip, mode << MODE_SHIFT,
				MAIN_CONF_REG_ADDR) == 0;
}

/* current is in mA */
static bool set_led_drive_strength(struct max44005_chip *chip, int cur)
{
	if (!chip->supply[LED])
		goto finish;

	if (cur && !chip->power_utilization[LED])
		regulator_enable(chip->supply[LED]);
	else if (!cur && chip->power_utilization[LED])
		regulator_disable(chip->supply[LED]);

finish:
	chip->power_utilization[LED] = cur ? 1 : 0;
	return max44005_write(chip, 0xA1, PROX_CONF_REG_ADDR) == 0;
}

/* assumes power is on */
static bool enable_temp_channel(struct max44005_chip *chip, int enable_temp)
{
	int ret = 0;

	if (enable_temp) {
		/* if LED is not used, then clear is enabled */
		if (!CLEAR_ENABLED && PROXIMITY_ENABLED)
			set_main_conf(chip, MODE_CLEAR_PROX);
		ret = max44005_write(chip, TEMP_ENABLE | COMP_ENABLE |
					AMB_PGA_256x, AMB_CONF_REG_ADDR);
	} else {
		ret = max44005_write(chip, TEMP_DISABLE &
					(COMP_ENABLE | AMB_PGA_256x),
					AMB_CONF_REG_ADDR);
	}

	return ret == 0;
}

static bool max44005_power(struct max44005_chip *chip, int power_on)
{
	int was_regulator_already_on = false;

	if (power_on && chip->power_utilization[CHIP])
		return true;

	if (power_on) {
		if (chip->supply[CHIP]) {
			was_regulator_already_on =
				regulator_is_enabled(chip->supply[CHIP]);
			if (regulator_enable(chip->supply[CHIP]))
				return false;
			if (!was_regulator_already_on)
				msleep(POWER_ON_DELAY);
		}
		chip->power_utilization[CHIP] = 1;

		/* wakeup if still in shutdown state */
		max44005_standby(chip, false);
		return true;
	}

	/* power off request */
	/* disable the power source as chip doesnot need it anymore */
	if (chip->supply[CHIP] && chip->power_utilization[CHIP] &&
			regulator_disable(chip->supply[CHIP]))
		return false;
	chip->power_utilization[CHIP] = 0;
	/* chip doesnt utilize power now, power being
	 * supplied is being wasted, so put the device to standby
	 * to reduce wastage */
	max44005_standby(chip, true);

	return true;
}

/* assumes power is on */
static bool max44005_restore_state(struct max44005_chip *chip)
{
	int ret;

	if (PROXIMITY_ENABLED)
		ret = set_led_drive_strength(chip, LED_DRV_STRENGTH);
	else
		ret = set_led_drive_strength(chip, 0);

	if (!ret)
		return false;

	switch ((CLEAR_ENABLED << 1) | PROXIMITY_ENABLED) {
	case 0:
		ret = max44005_power(chip, false);
		break;
	case 1:
		ret = set_main_conf(chip, MODE_PROX_ONLY);
		break;
	case 2:
		ret = set_main_conf(chip, MODE_CLEAR_ONLY);
		break;
	case 3:
		ret = set_main_conf(chip, MODE_CLEAR_PROX);
		break;
	}

	if (ret)
		return enable_temp_channel(chip, TEMP_ENABLED);
	return false;
}

/* sysfs name begin */
static ssize_t show_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max44005_chip *chip = iio_priv(indio_dev);
	return sprintf(buf, "%s\n", chip->client->name);
}
/* sysfs name end */

/* amb clear begin */
static ssize_t show_amb_clear_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	MAX44005_SYSFS_SHOW(AMB_CLEAR_HIGH_ADDR, 2);
}

static ssize_t amb_clear_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 lval;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max44005_chip *chip = iio_priv(indio_dev);

	if (kstrtou32(buf, 10, &lval))
		return -EINVAL;

	if (lval && (lval != 1))
		return -EINVAL;

	if (lval == chip->using_als)
		return count;

	mutex_lock(&chip->lock);
	/* if clear channel is being used by temp */
	if (TEMP_ENABLED)
		goto success;

	if (lval) {
		if (!max44005_power(chip, true))
			goto fail;
		/* enabling clear also enabled temp channel
		 * although there are no users for it,
		 * so disable it explicitly */
		enable_temp_channel(chip, false);

		if (!PROXIMITY_ENABLED)
			goto success;

		/* if clear not enabled and LED enabled
		 * change the mode to CLEAR+LED enabled*/
		if (PROXIMITY_ENABLED &&
				set_main_conf(chip, MODE_CLEAR_PROX))
			goto success;
		/* CLEAR channel remains intact due to lost communication */
		goto fail;
	} else {
		if (PROXIMITY_ENABLED && set_main_conf(chip, MODE_PROX_ONLY))
			goto success;

		if (!PROXIMITY_ENABLED && max44005_power(chip, false))
			goto success;

		goto fail;
	}

success:
	chip->using_als = lval;
	mutex_unlock(&chip->lock);
	return count;
fail:
	mutex_unlock(&chip->lock);
	return -EBUSY;
}
/* amb clear end */

/* amb LED begin */
static ssize_t show_prox_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	MAX44005_SYSFS_SHOW(PROX_HIGH_ADDR, 2);
}

static ssize_t prox_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 lval;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max44005_chip *chip = iio_priv(indio_dev);

	if (kstrtou32(buf, 10, &lval))
		return -EINVAL;

	if (lval && (lval != 1))
		return -EINVAL;

	if (lval == PROXIMITY_ENABLED)
		return count;

	mutex_lock(&chip->lock);
	if (lval) {
		if (!max44005_power(chip, true))
			goto fail;

		if (!set_led_drive_strength(chip, LED_DRV_STRENGTH))
			goto fail;

		if (CLEAR_ENABLED && set_main_conf(chip, MODE_CLEAR_PROX))
			goto success;

		if (!CLEAR_ENABLED && set_main_conf(chip, MODE_PROX_ONLY))
			goto success;

		goto fail;
	} else {
		/* power off if no other channel is active */
		if (!CLEAR_ENABLED && max44005_power(chip, false))
			goto success;

		if (CLEAR_ENABLED && set_led_drive_strength(chip, 0) &&
			set_main_conf(chip, MODE_CLEAR_ONLY))
			goto success;

		goto fail;
	}

success:
	chip->using_proximity = lval;
	mutex_unlock(&chip->lock);
	return count;
fail:
	mutex_unlock(&chip->lock);
	return -EBUSY;
}
/* amb LED end */

/* amb TEMP begin */
static ssize_t show_amb_temp_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	MAX44005_SYSFS_SHOW(AMB_TEMP_HIGH_ADDR, 2);
}

static ssize_t amb_temp_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 lval;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct max44005_chip *chip = iio_priv(indio_dev);

	if (kstrtou32(buf, 10, &lval))
		return -EINVAL;

	if (lval && (lval != 1))
		return -EINVAL;

	if (lval == TEMP_ENABLED)
		return count;

	mutex_lock(&chip->lock);
	if (lval) {
		if (!max44005_power(chip, true))
			goto fail;

		if (PROXIMITY_ENABLED && !USING_CLEAR &&
				!set_main_conf(chip, MODE_CLEAR_PROX))
			goto fail;

		if (enable_temp_channel(chip, true))
			goto success;

		goto fail;
	} else {
		if (!USING_CLEAR && !PROXIMITY_ENABLED &&
			max44005_power(chip, false))
			goto success;

		if (!USING_CLEAR && PROXIMITY_ENABLED &&
			set_main_conf(chip, MODE_PROX_ONLY) &&
			enable_temp_channel(chip, false))
			goto success;

		if (USING_CLEAR && enable_temp_channel(chip, false))
			goto success;

		goto fail;
	}

success:
	chip->using_temp = lval;
	mutex_unlock(&chip->lock);
	return count;
fail:
	mutex_unlock(&chip->lock);
	return -EBUSY;
}
/* amb TEMP end */

static IIO_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);
static IIO_DEVICE_ATTR(amb_clear, S_IRUGO | S_IWUSR, show_amb_clear_value,
			amb_clear_enable, 0);
static IIO_DEVICE_ATTR(proximity, S_IRUGO | S_IWUSR, show_prox_value,
			prox_enable, 0);
static IIO_DEVICE_ATTR(amb_temp, S_IRUGO | S_IWUSR, show_amb_temp_value,
			amb_temp_enable, 0);

/* sysfs attr */
static struct attribute *max44005_iio_attr[] = {
	&iio_dev_attr_name.dev_attr.attr,
	&iio_dev_attr_amb_clear.dev_attr.attr,
	&iio_dev_attr_proximity.dev_attr.attr,
	&iio_dev_attr_amb_temp.dev_attr.attr,
	NULL
};

static const struct attribute_group max44005_iio_attr_group = {
	.attrs = max44005_iio_attr,
};

static const struct iio_info max44005_iio_info = {
	.attrs = &max44005_iio_attr_group,
	.driver_module = THIS_MODULE,
};

static int max44005_sysfs_init(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct device *class_device;

	sensor_class = class_create(THIS_MODULE, "sensors");
	if (!sensor_class) {
		dev_err(&client->dev, "create /sys/class/sensors fails\n");
		return -EINVAL;
	}

	class_device = device_create(sensor_class, &indio_dev->dev,
					0, NULL, "%s", "light");
	if (!class_device) {
		dev_err(&client->dev, "create ...sensors/light fails\n");
		class_destroy(sensor_class);
		return -EINVAL;
	}

	return 0;
}

static int max44005_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct max44005_chip *chip;
	int err;

	indio_dev = iio_device_alloc(sizeof(struct max44005_chip));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "iio allocation fails\n");
		return -ENOMEM;
	}

	chip = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	chip->client = client;
	mutex_init(&chip->lock);

	indio_dev->info = &max44005_iio_info;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	err = iio_device_register(indio_dev);
	if (err | max44005_sysfs_init(client)) {
		dev_err(&client->dev, "iio registration fails\n");
		device_destroy(sensor_class, 0);
		class_destroy(sensor_class);
		mutex_destroy(&chip->lock);
		iio_device_free(indio_dev);
		return err;
	}

	chip->supply[CHIP] = regulator_get(&client->dev, "vdd");

	if (IS_ERR_OR_NULL(chip->supply[CHIP])) {
		dev_err(&client->dev, "could not get regulator\n"
				"assuming power supply is always on\n");
		kfree(chip->supply);
		chip->supply[CHIP] = NULL;
		chip->supply[LED] = NULL;
	} else {
		chip->supply[LED] = regulator_get(&client->dev, "vdd_prox");

		if (IS_ERR_OR_NULL(chip->supply[LED])) {
			dev_err(&client->dev, "als_prox regulator not present\n"
					"proximity sensor may not work fine\n");
			chip->supply[LED] = NULL;
		}
	}

	mutex_lock(&chip->lock);
	max44005_power(chip, false);
	mutex_unlock(&chip->lock);

	chip->using_als = false;
	chip->using_proximity = false;
	chip->using_temp = false;
	chip->shutdown_complete = 0;
	dev_info(&client->dev, "%s() success\n", __func__);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max44005_suspend(struct device *dev)
{
	int ret = 0;
/* Keep suspend/resume disabled till LP0 is stable on ceres.
 * iio core adds NULL pm_ops if driver doesnt have them.
 * This causes crash during suspend/resume. Hence, instead
 * of removing them, make them "noop".
 *
 * struct iio_dev *indio_dev = i2c_get_clientdata(client);
 * struct max44005_chip *chip = iio_priv(indio_dev);
 * mutex_lock(&chip->lock);
 * max44005_power(chip, false);
 * mutex_unlock(&chip->lock);
 */
	return ret;
}

static int max44005_resume(struct device *dev)
{
	int ret = 0;
/* struct iio_dev *indio_dev = i2c_get_clientdata(client);
 * struct max44005_chip *chip = iio_priv(indio_dev);
 * mutex_lock(&chip->lock);
 * max44005_power(chip, true);
 * max44005_restore_state(chip);
 * mutex_unlock(&chip->lock);
 */
	return ret;
}

static SIMPLE_DEV_PM_OPS(max44005_pm_ops, max44005_suspend, max44005_resume);
#define MAX44005_PM_OPS (&max44005_pm_ops)
#else
#define MAX44005_PM_OPS NULL
#endif

static int max44005_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct max44005_chip *chip = iio_priv(indio_dev);
	dev_dbg(&client->dev, "%s()\n", __func__);
	if (chip->supply[CHIP])
		regulator_put(chip->supply[CHIP]);
	mutex_destroy(&chip->lock);
	device_destroy(sensor_class, 0);
	class_destroy(sensor_class);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
	return 0;
}

static void max44005_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct max44005_chip *chip = iio_priv(indio_dev);
	mutex_lock(&chip->lock);
	if (chip->supply[CHIP])
		regulator_put(chip->supply[CHIP]);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->lock);
	mutex_destroy(&chip->lock);
	device_destroy(sensor_class, 0);
	class_destroy(sensor_class);
	iio_device_unregister(indio_dev);
	iio_device_free(indio_dev);
}

static const struct i2c_device_id max44005_id[] = {
	{"max44005", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, max44005_id);

static struct i2c_driver max44005_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver  = {
		.name = "max44005",
		.owner = THIS_MODULE,
		.pm = MAX44005_PM_OPS,
	},
	.probe	 = max44005_probe,
	.shutdown = max44005_shutdown,
	.remove  = max44005_remove,
	.id_table = max44005_id,
};

static int __init max44005_init(void)
{
	return i2c_add_driver(&max44005_driver);
}

static void __exit max44005_exit(void)
{
	i2c_del_driver(&max44005_driver);
}

module_init(max44005_init);
module_exit(max44005_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX44005 Driver");
MODULE_AUTHOR("Sri Krishna chowdary <schowdary@nvidia.com>");
