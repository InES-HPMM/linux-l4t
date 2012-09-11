/*
* Copyright (C) 2012 Invensense, Inc.
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

/**
 *  @addtogroup  DRIVERS
 *  @brief	   Hardware drivers.
 *
 *  @{
 *	  @file	ak8975_input.c
 *	  @brief   A sysfs device driver for Invensense devices
 *	  @details This driver currently works for the AK8975
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#include <linux/mpu.h>

#define AK8975_DEBUG_IF	0
#define AK8975_DEBUG_DATA	0

#define AK8975_I2C_NAME "ak8975"

#define SENSOR_DATA_SIZE	8
#define YPR_DATA_SIZE		12
#define RWBUF_SIZE		16

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define ORI_DATA_FLAG		2
#define AKM_NUM_SENSORS		3

#define ACC_DATA_READY		(1 << (ACC_DATA_FLAG))
#define MAG_DATA_READY		(1 << (MAG_DATA_FLAG))
#define ORI_DATA_READY		(1 << (ORI_DATA_FLAG))

/*! \name AK8975 constant definition
 \anchor AK8975_Def
 Constant definitions of the AK8975.*/
#define AK8975_MEASUREMENT_TIME_US	10000

/*! \name AK8975 operation mode
 \anchor AK8975_Mode
 Defines an operation mode of the AK8975.*/
/*! @{*/
#define AK8975_CNTL_MODE_SNG_MEASURE	0x01
#define	AK8975_CNTL_MODE_SELF_TEST	0x08
#define	AK8975_CNTL_MODE_FUSE_ACCESS	0x0F
#define	AK8975_CNTL_MODE_POWER_DOWN	0x00
/*! @}*/

/*! \name AK8975 register address
\anchor AK8975_REG
Defines a register address of the AK8975.*/
/*! @{*/
#define AK8975_REG_WIA		0x00
#define AK8975_REG_INFO		0x01
#define AK8975_REG_ST1		0x02
#define AK8975_REG_HXL		0x03
#define AK8975_REG_HXH		0x04
#define AK8975_REG_HYL		0x05
#define AK8975_REG_HYH		0x06
#define AK8975_REG_HZL		0x07
#define AK8975_REG_HZH		0x08
#define AK8975_REG_ST2		0x09
#define AK8975_REG_CNTL		0x0A
#define AK8975_REG_RSV		0x0B
#define AK8975_REG_ASTC		0x0C
#define AK8975_REG_TS1		0x0D
#define AK8975_REG_TS2		0x0E
#define AK8975_REG_I2CDIS	0x0F
/*! @}*/

/*! \name AK8975 fuse-rom address
\anchor AK8975_FUSE
Defines a read-only address of the fuse ROM of the AK8975.*/
/*! @{*/
#define AK8975_FUSE_ASAX	0x10
#define AK8975_FUSE_ASAY	0x11
#define AK8975_FUSE_ASAZ	0x12
/*! @}*/

#define AK8975_MAX_DELAY	(100)
#define AK8975_MIN_DELAY	(10)

/**
 *  struct inv_gyro_state_s - Driver state variables.
 *  @dev:		Represents read-only node for accessing buffered data.
 *  @idev:		Handle to input device.
 *  @sl_handle:		Handle to I2C port.
 */
struct inv_compass_state {
	struct i2c_client *i2c;
	atomic_t enable;
	atomic_t delay;
	struct mpu_platform_data plat_data;
	short i2c_addr;
	void *sl_handle;
	struct device *inv_dev;
	struct input_dev *idev;
	struct delayed_work work;

	struct mutex value_mutex;
	struct mutex enable_mutex;
	short value[3];
	char asa[3];	/* axis sensitivity adjustment */
};

/* -------------------------------------------------------------------------- */
/**
 *  inv_serial_read() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
 *  NOTE: The slave register will not increment when reading from the FIFO.
 */
static int inv_serial_read(struct inv_compass_state *st,
	unsigned char reg, unsigned short length, unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data || !st->sl_handle)
		return -EINVAL;

	msgs[0].addr = st->i2c_addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = st->i2c_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = length;

	pr_debug("%s RD%02X%02X%02X\n",
		 st->idev->name, st->i2c_addr, reg, length);
	res = i2c_transfer(st->sl_handle, msgs, 2);
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

/**
 *  inv_serial_single_write() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
static int inv_serial_single_write(struct inv_compass_state *st,
	unsigned char reg, unsigned char data)
{
	unsigned char tmp[2];
	struct i2c_msg msg;
	int res;

	if (!st->sl_handle)
		return -EINVAL;

	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = st->i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

	pr_debug("%s WS%02X%02X%02X\n",
		 st->idev->name, st->i2c_addr, reg, data);
	res = i2c_transfer(st->sl_handle, &msg, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else
		return 0;
}

static int ak8975_init(struct inv_compass_state *st)
{
	int result = 0;
	unsigned char serial_data[3];

	result = inv_serial_single_write(st, AK8975_REG_CNTL,
					 AK8975_CNTL_MODE_POWER_DOWN);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}
	/* Wait at least 100us */
	udelay(100);

	result = inv_serial_single_write(st, AK8975_REG_CNTL,
					 AK8975_CNTL_MODE_FUSE_ACCESS);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}

	/* Wait at least 200us */
	udelay(200);

	result = inv_serial_read(st, AK8975_FUSE_ASAX, 3, serial_data);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}

	st->asa[0] = serial_data[0];
	st->asa[1] = serial_data[1];
	st->asa[2] = serial_data[2];

	result = inv_serial_single_write(st, AK8975_REG_CNTL,
					 AK8975_CNTL_MODE_POWER_DOWN);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}
	udelay(100);

	return result;
}

static int ak8975_read(struct inv_compass_state *st, short rawfixed[3])
{
	unsigned char regs[8];
	unsigned char *stat = &regs[0];
	unsigned char *stat2 = &regs[7];
	int result = 0;
	int status = 0;

	result = inv_serial_read(st, AK8975_REG_ST1, 8, regs);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
	return result;
	}

	rawfixed[0] = (short)((regs[2]<<8) | regs[1]);
	rawfixed[1] = (short)((regs[4]<<8) | regs[3]);
	rawfixed[2] = (short)((regs[6]<<8) | regs[5]);

	/*
	 * ST : data ready -
	 * Measurement has been completed and data is ready to be read.
	 */
	if (*stat & 0x01)
		status = 0;

	/*
	 * ST2 : data error -
	 * occurs when data read is started outside of a readable period;
	 * data read would not be correct.
	 * Valid in continuous measurement mode only.
	 * In single measurement mode this error should not occour but we
	 * stil account for it and return an error, since the data would be
	 * corrupted.
	 * DERR bit is self-clearing when ST2 register is read.
	 */
	if (*stat2 & 0x04)
		status = 0x04;
	/*
	 * ST2 : overflow -
	 * the sum of the absolute values of all axis |X|+|Y|+|Z| < 2400uT.
	 * This is likely to happen in presence of an external magnetic
	 * disturbance; it indicates, the sensor data is incorrect and should
	 * be ignored.
	 * An error is returned.
	 * HOFL bit clears when a new measurement starts.
	 */
	if (*stat2 & 0x08)
		status = 0x08;
	/*
	 * ST : overrun -
	 * the previous sample was not fetched and lost.
	 * Valid in continuous measurement mode only.
	 * In single measurement mode this error should not occour and we
	 * don't consider this condition an error.
	 * DOR bit is self-clearing when ST2 or any meas. data register is
	 * read.
	 */
	if (*stat & 0x02) {
		/* status = INV_ERROR_COMPASS_DATA_UNDERFLOW; */
		status = 0;
	}

	/*
	 * trigger next measurement if:
	 *	- stat is non zero;
	 *	- if stat is zero and stat2 is non zero.
	 * Won't trigger if data is not ready and there was no error.
	 */
	if (*stat != 0x00 || *stat2 != 0x00) {
		result = inv_serial_single_write(st, AK8975_REG_CNTL,
					 AK8975_CNTL_MODE_SNG_MEASURE);
		if (result) {
			pr_err("%s, line=%d\n", __func__, __LINE__);
			return result;
		}
	}

	if (status)
		pr_err("%s, line=%d, status=%d\n", __func__, __LINE__, status);

	return status;
}

static void ak8975_work_func(struct work_struct *work)
{
	struct inv_compass_state *st =
		container_of((struct delayed_work *)work,
			struct inv_compass_state, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&st->delay));
	short c[3];
	c[0] = c[1] = c[2] = 0;
	if (0 == ak8975_read(st, c)) {
		c[0] = ((c[0] * (st->asa[0] + 128)) >> 8);
		c[1] = ((c[1] * (st->asa[1] + 128)) >> 8);
		c[2] = ((c[2] * (st->asa[2] + 128)) >> 8);
		input_report_rel(st->idev, REL_X, c[0]);
		input_report_rel(st->idev, REL_Y, c[1]);
		input_report_rel(st->idev, REL_Z, c[2]);
		input_sync(st->idev);
	}

	mutex_lock(&st->value_mutex);
	st->value[0] = c[0];
	st->value[1] = c[1];
	st->value[2] = c[2];
	mutex_unlock(&st->value_mutex);
	schedule_delayed_work(&st->work, delay);
}

static ssize_t ak8975_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	short c[3];

	mutex_lock(&st->value_mutex);
	c[0] = st->value[0];
	c[1] = st->value[1];
	c[2] = st->value[2];
	mutex_unlock(&st->value_mutex);
	return sprintf(buf, "%d, %d, %d\n", c[0], c[1], c[2]);
}

static ssize_t ak8975_scale_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%ld\n", 9830L * 32768L);
}
static ssize_t ak8975_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	int result;
	result = ak8975_init(st);
	return sprintf(buf, "%d\n", result);
}

static ssize_t ak8975_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", atomic_read(&st->enable));
}

static ssize_t ak8975_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	/* transform delay in ms to rate */
	return sprintf(buf, "%d\n", 1000 / atomic_read(&st->delay));
}

static ssize_t akm8975_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	signed char *m;
	m = st->plat_data.orientation;
	return sprintf(buf,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

static ssize_t ak8975_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct inv_compass_state *st = dev_get_drvdata(dev);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	/* transform rate to delay in ms */
	data = 1000 / data;
	if (data > AK8975_MAX_DELAY)
		data = AK8975_MAX_DELAY;
	if (data < AK8975_MIN_DELAY)
		data = AK8975_MIN_DELAY;
	atomic_set(&st->delay, (unsigned int) data);
	return count;
}

static void ak8975_set_enable(struct device *dev, int enable)
{
	struct inv_compass_state *st = dev_get_drvdata(dev);
	int result = 0;
	int pre_enable = atomic_read(&st->enable);

	mutex_lock(&st->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			result = inv_serial_single_write(st, AK8975_REG_CNTL,
						AK8975_CNTL_MODE_SNG_MEASURE);
			if (result)
				pr_err("%s, line=%d\n", __func__, __LINE__);
			schedule_delayed_work(&st->work,
				msecs_to_jiffies(atomic_read(&st->delay)));
			atomic_set(&st->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			cancel_delayed_work_sync(&st->work);
			atomic_set(&st->enable, 0);
			result = inv_serial_single_write(st, AK8975_REG_CNTL,
						AK8975_CNTL_MODE_POWER_DOWN);
			if (result)
				pr_err("%s, line=%d\n", __func__, __LINE__);
			mdelay(1);	/* wait at least 100us */
		}
	}
	mutex_unlock(&st->enable_mutex);
}

static ssize_t ak8975_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data == 0 || data == 1)
		ak8975_set_enable(dev, data);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		ak8975_enable_show, ak8975_enable_store);
static DEVICE_ATTR(value, S_IRUGO, ak8975_value_show, NULL);
static DEVICE_ATTR(scale, S_IRUGO, ak8975_scale_show, NULL);
static DEVICE_ATTR(reset, S_IRUGO, ak8975_reset_show, NULL);
static DEVICE_ATTR(rate, S_IRUGO | S_IWUSR, ak8975_rate_show,
		ak8975_rate_store);
static DEVICE_ATTR(compass_matrix, S_IRUGO, akm8975_matrix_show, NULL);

static struct attribute *ak8975_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_value.attr,
	&dev_attr_scale.attr,
	&dev_attr_reset.attr,
	&dev_attr_rate.attr,
	&dev_attr_compass_matrix.attr,
	NULL
};

static struct attribute_group ak8975_attribute_group = {
	.name = "ak8975",
	.attrs = ak8975_attributes
};


/**
 *  inv_setup_input() - internal setup input device.
 *  @st:	Device driver instance.
 *  @**idev_in  pointer to input device
 *  @*client	i2c client
 *  @*name	  name of the input device.
 */
static int inv_setup_input(struct inv_compass_state *st,
	struct input_dev **idev_in, struct i2c_client *client,
	unsigned char *name) {
	int result;
	struct input_dev *idev;
	idev = input_allocate_device();
	if (!idev) {
		result = -ENOMEM;
		return result;
	}
	/* Setup input device. */
	idev->name = name;

	idev->id.bustype = BUS_I2C;
	idev->id.product = 'S';
	idev->id.vendor	 = ('I'<<8) | 'S';
	idev->id.version	= 1;
	idev->dev.parent = &client->dev;
	/* Open and close method. */
	idev->open = NULL;
	idev->close = NULL;

	__set_bit(EV_REL, idev->evbit);
	input_set_capability(idev, EV_REL, REL_X);
	input_set_capability(idev, EV_REL, REL_Y);
	input_set_capability(idev, EV_REL, REL_Z);

	input_set_capability(idev, EV_REL, REL_MISC);
	input_set_capability(idev, EV_REL, REL_WHEEL);

	input_set_drvdata(idev, st);
	result = input_register_device(idev);
	if (result)
		input_free_device(idev);

	*idev_in = idev;
	return result;
}
static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int ak8975_mod_probe(struct i2c_client *client,
			   const struct i2c_device_id *devid)
{
	struct mpu_platform_data *pdata;
	struct inv_compass_state *st;
	struct input_dev *idev;
	int result = 0;

	dev_info(&client->adapter->dev, "%s: %s\n", __func__, devid->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

	pdata = (struct mpu_platform_data *)dev_get_platdata(&client->dev);
	if (!pdata) {
		dev_err(&client->adapter->dev,
			"Missing platform data for slave %s\n", devid->name);
		result = -EFAULT;
		goto out_no_free;
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		result = -ENOMEM;
		goto out_no_free;
	}

	i2c_set_clientdata(client, st);
	st->i2c = client;
	mutex_init(&st->value_mutex);
	mutex_init(&st->enable_mutex);
	atomic_set(&st->delay, 100);
	st->sl_handle = client->adapter;
	st->plat_data = *pdata;
	st->i2c_addr = client->addr;
	INIT_DELAYED_WORK(&st->work, ak8975_work_func);
	result = inv_setup_input(st, &idev, client, "INV_AK8975");
	if (result)
		goto out_free_memory;
	st->idev = idev;
	result = sysfs_create_group(&st->idev->dev.kobj,
						 &ak8975_attribute_group);
	if (result < 0)
		goto error_sysfs;

	result = ak8975_init(st);
	if (result < 0)
		goto error_sysfs;

	return result;
error_sysfs:
	input_unregister_device(st->idev);
out_free_memory:
	kfree(st);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return result;

}

static int ak8975_mod_remove(struct i2c_client *client)
{
	struct inv_compass_state *st =
		i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	ak8975_set_enable(&st->idev->dev, 0);
	sysfs_remove_group(&st->idev->dev.kobj, &ak8975_attribute_group);
	input_unregister_device(st->idev);
	kfree(st);
	return 0;
}

static const struct i2c_device_id ak8975_mod_id[] = {
	{"ak8975", COMPASS_ID_AK8975},
	{}
};

MODULE_DEVICE_TABLE(i2c, ak8975_mod_id);

static struct i2c_driver ak8975_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = ak8975_mod_probe,
	.remove = ak8975_mod_remove,
	.id_table = ak8975_mod_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ak8975_mod",
		   },
	.address_list = normal_i2c,
};

static int __init ak8975_mod_init(void)
{
	int res = i2c_add_driver(&ak8975_mod_driver);
	pr_info("%s: Probe name %s\n", __func__, "ak8975_mod");
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit ak8975_mod_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&ak8975_mod_driver);
}

module_init(ak8975_mod_init);
module_exit(ak8975_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver for AK8975 sensor with input subsystem");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ak8975_mod");

/**
 *  @}
 */
