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
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_gyro.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This driver currently works for the ITG3500, MPU6050, MPU9150
 *               MPU3050
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/byteorder/generic.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "inv_gyro.h"

static struct inv_reg_map_s chip_reg = {
	.who_am_i		= 0x75,
	.sample_rate_div	= 0x19,
	.lpf			= 0x1A,
	.product_id		= 0x0C,
	.bank_sel		= 0x6D,
	.user_ctrl		= 0x6A,
	.fifo_en		= 0x23,
	.gyro_config		= 0x1B,
	.accl_config		= 0x1C,
	.fifo_count_h		= 0x72,
	.fifo_r_w		= 0x74,
	.raw_gyro		= 0x43,
	.raw_accl		= 0x3B,
	.temperature		= 0x41,
	.int_enable		= 0x38,
	.int_status		= 0x3A,
	.pwr_mgmt_1		= 0x6B,
	.pwr_mgmt_2		= 0x6C,
	.mem_start_addr		= 0x6E,
	.mem_r_w		= 0x6F,
	.prgm_strt_addrh	= 0x70
};
static const struct inv_hw_s hw_info[INV_NUM_PARTS] = {
	{119, "ITG3500"},
	{ 63, "MPU3050"},
	{118, "MPU6050"},
	{118, "MPU9150"}
};

s64 get_time_ns(void)
{
	struct timespec ts;
	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

/**
 *  inv_i2c_read_base() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
 *  NOTE: The slave register will not increment when reading from the FIFO.
 */
int inv_i2c_read_base(struct inv_gyro_state_s *st, unsigned short i2c_addr,
		      unsigned char reg, unsigned short length,
		      unsigned char *data)
{
	struct i2c_msg msgs[2];
	int res;

	if (!data)
		return -EINVAL;

	msgs[0].addr = i2c_addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = i2c_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = length;

	pr_debug("%s RD%02X%02X%02X\n", st->hw->name, i2c_addr, reg, length);
	res = i2c_transfer(st->sl_handle, msgs, 2);
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	}

	return 0;
}

/**
 *  inv_i2c_single_write_base() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
int inv_i2c_single_write_base(struct inv_gyro_state_s *st,
			      unsigned short i2c_addr, unsigned char reg,
			      unsigned char data)
{
	unsigned char tmp[2];
	struct i2c_msg msg;
	int res;

	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

	pr_debug("%s WS%02X%02X%02X\n", st->hw->name, i2c_addr, reg, data);
	res = i2c_transfer(st->sl_handle, &msg, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	}

	return 0;
}
/**
 *  inv_clear_kfifo() - clear time stamp fifo
 *  @st:	Device driver instance.
 */
void inv_clear_kfifo(struct inv_gyro_state_s *st)
{
	unsigned long flags;
	spin_lock_irqsave(&st->time_stamp_lock, flags);
	kfifo_reset(&st->trigger.timestamps);
	spin_unlock_irqrestore(&st->time_stamp_lock, flags);
}

static int set_power_itg(struct inv_gyro_state_s *st, unsigned char power_on)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;

	reg = st->reg;
	if (power_on)
		data = 0;
	else
		data = BIT_SLEEP;
	data |= (st->chip_config.lpa_mode << 5);
	if (st->chip_config.gyro_enable) {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_PLL);
		if (result)
			return result;

		st->chip_config.clk_src = INV_CLK_PLL;
	} else {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_INTERNAL);
		if (result)
			return result;

		st->chip_config.clk_src = INV_CLK_INTERNAL;
	}

	if (power_on) {
		mdelay(POWER_UP_TIME);
		data = 0;
		if (0 == st->chip_config.accl_enable)
			data |= BIT_PWR_ACCL_STBY;
		if (0 == st->chip_config.gyro_enable)
			data |= BIT_PWR_GYRO_STBY;
		data |= (st->chip_config.lpa_freq << 6);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;

		mdelay(POWER_UP_TIME);
		st->chip_config.is_asleep = 0;
	} else {
		st->chip_config.is_asleep = 1;
	}
	return 0;
}

/**
 *  inv_set_power_state() - Turn device on/off.
 *  @st:	Device driver instance.
 *  @power_on:	1 to turn on, 0 to suspend.
 */
int inv_set_power_state(struct inv_gyro_state_s *st, unsigned char power_on)
{
	if (INV_MPU3050 == st->chip_type)
		return set_power_mpu3050(st, power_on);
	else
		return set_power_itg(st, power_on);
}

/**
 *  reset_fifo_itg() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 */
static int reset_fifo_itg(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char val;

	reg = st->reg;
	/* disable interrupt */
	result = inv_i2c_single_write(st, reg->int_enable, 0);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}

	/* disable the sensor output to FIFO */
	result = inv_i2c_single_write(st, reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;

	/* disable fifo reading */
	result = inv_i2c_single_write(st, reg->user_ctrl, 0);
	if (result)
		goto reset_fifo_fail;

	if (st->chip_config.dmp_on) {
		val = (BIT_FIFO_RST | BIT_DMP_RST);
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_RST;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;

		mdelay(POWER_UP_TIME);
		st->last_isr_time = get_time_ns();
		result = inv_i2c_single_write(st, reg->int_enable,
					      BIT_DMP_INT_EN);
		if (result)
			return result;

		val = (BIT_DMP_EN | BIT_FIFO_EN);
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;

	} else {
		/* reset FIFO and possibly reset I2C*/
		val = BIT_FIFO_RST;
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_RST;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;

		mdelay(POWER_UP_TIME);
		st->last_isr_time = get_time_ns();
		/* enable interrupt */
		if (st->chip_config.accl_fifo_enable ||
			st->chip_config.gyro_fifo_enable ||
			st->chip_config.compass_enable){
			result = inv_i2c_single_write(st, reg->int_enable,
						BIT_DATA_RDY_EN);
			if (result)
				return result;
		}

		/* enable FIFO reading and I2C master interface*/
		val = BIT_FIFO_EN;
		if (st->chip_config.compass_enable)
			val |= BIT_I2C_MST_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, val);
		if (result)
			goto reset_fifo_fail;

		/* enable sensor output to FIFO */
		val = 0;
		if (st->chip_config.gyro_fifo_enable)
			val |= BITS_GYRO_OUT;
		if (st->chip_config.accl_fifo_enable)
			val |= BIT_ACCEL_OUT;
		result = inv_i2c_single_write(st, reg->fifo_en, val);
		if (result)
			goto reset_fifo_fail;
	}

	return 0;

reset_fifo_fail:
	if (st->chip_config.dmp_on)
		val = BIT_DMP_INT_EN;
	else
		val = BIT_DATA_RDY_EN;
	inv_i2c_single_write(st, reg->int_enable, val);
	pr_err("%s failed\n", __func__);
	return result;
}

/**
 *  inv_reset_fifo() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 */
int inv_reset_fifo(struct inv_gyro_state_s *st)
{
	if (INV_MPU3050 == st->chip_type)
		return reset_fifo_mpu3050(st);
	else
		return reset_fifo_itg(st);
}

/**
 *  set_inv_enable() - Reset FIFO related registers.
 *  @st:	Device driver instance.
 *  @fifo_enable: enable/disable
 */
int set_inv_enable(struct inv_gyro_state_s *st, unsigned long enable)
{
	struct inv_reg_map_s *reg;
	int result;

	if (st->chip_config.is_asleep)
		return -EINVAL;

	reg = st->reg;
	if (enable) {
		result = inv_reset_fifo(st);
		if (result)
			return result;

		inv_clear_kfifo(st);
		st->chip_config.enable = 1;

	} else {
		result = inv_i2c_single_write(st, reg->fifo_en, 0);
		if (result)
			return result;

		result = inv_i2c_single_write(st, reg->int_enable, 0);
		if (result)
			return result;

		/* disable fifo reading */
		if (INV_MPU3050 != st->chip_type) {
			result = inv_i2c_single_write(st, reg->user_ctrl, 0);
			if (result)
				return result;
		}

		st->chip_config.enable = 0;
	}

	return 0;
}

static void inv_input_close(struct input_dev *d)
{
	struct inv_gyro_state_s *st;

	st = input_get_drvdata(d);
	set_inv_enable(st, 0);
	inv_set_power_state(st, 0);
}

/**
 *  inv_setup_input() - internal setup input device.
 *  @st:	Device driver instance.
 *  @**idev_in  pointer to input device
 *  @*client    i2c client
 *  @*name      name of the input device.
 */
static int inv_setup_input(struct inv_gyro_state_s *st,
			   struct input_dev **idev_in,
			   struct i2c_client *client, unsigned char *name)
{
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
	idev->id.vendor     = ('I'<<8) | 'S';
	idev->id.version    = 1;
	idev->dev.parent = &client->dev;
	/* Open and close method. */
	if (strcmp(name, "INV_DMP") && strcmp(name, "INV_COMPASS"))
		idev->close = inv_input_close;
	input_set_capability(idev, EV_REL, REL_X);
	input_set_capability(idev, EV_REL, REL_Y);
	input_set_capability(idev, EV_REL, REL_Z);
	input_set_capability(idev, EV_REL, REL_RX);
	input_set_capability(idev, EV_REL, REL_RY);
	input_set_capability(idev, EV_REL, REL_RZ);
	input_set_capability(idev, EV_REL, REL_MISC);
	input_set_capability(idev, EV_REL, REL_WHEEL);
	input_set_drvdata(idev, st);
	result = input_register_device(idev);
	if (result)
		input_free_device(idev);
	*idev_in = idev;
	return result;
}

/**
 *  inv_init_config() - Initialize hardware, disable FIFO.
 *  @st:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_init_config(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;

	if (st->chip_config.is_asleep)
		return -EPERM;

	reg = st->reg;
	result = set_inv_enable(st, 0);
	if (result)
		return result;

	result = inv_i2c_single_write(st, reg->gyro_config,
		INV_FSR_2000DPS << 3);
	if (result)
		return result;

	st->chip_config.fsr = INV_FSR_2000DPS;
	result = inv_i2c_single_write(st, reg->lpf, INV_FILTER_42HZ);
	if (result)
		return result;

	st->chip_config.lpf = INV_FILTER_42HZ;
	result = inv_i2c_single_write(st, reg->sample_rate_div, 19);
	if (result)
		return result;

	st->chip_config.fifo_rate = 50;
	st->irq_dur_us            = 20*1000;
	st->chip_config.enable = 0;
	st->chip_config.dmp_on = 0;
	st->compass_divider = 0;
	st->compass_counter = 0;
	st->chip_config.compass_enable = 0;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.gyro_enable = 1;
	st->chip_config.gyro_fifo_enable = 1;
	if (INV_ITG3500 != st->chip_type) {
		st->chip_config.accl_enable = 1;
		st->chip_config.accl_fifo_enable = 1;
		st->chip_config.accl_fs = INV_FS_02G;
		result = inv_i2c_single_write(st, reg->accl_config,
			(INV_FS_02G << 3));
		if (result)
			return result;

	} else {
		st->chip_config.accl_enable = 0;
		st->chip_config.accl_fifo_enable = 0;
	}

	return 0;
}

/**
 *  inv_raw_gyro_show() - Read gyro data directly from registers.
 */
static ssize_t inv_raw_gyro_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	int result;
	unsigned char data[6];

	st = dev_get_drvdata(dev);
	reg = st->reg;
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (0 == st->chip_config.gyro_enable)
		return -EPERM;

	result = inv_i2c_read(st, reg->raw_gyro, 6, data);
	if (result) {
		printk(KERN_ERR "Could not read raw registers.\n");
		return result;
	}

	return sprintf(buf, "%d %d %d %lld\n",
		(signed short)(be16_to_cpup((short *)&data[0])),
		(signed short)(be16_to_cpup((short *)&data[2])),
		(signed short)(be16_to_cpup((short *)&data[4])),
		get_time_ns());
}

/**
 *  inv_raw_accl_show() - Read accel data directly from registers.
 */
static ssize_t inv_raw_accl_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	int result;
	unsigned char data[6];

	st = dev_get_drvdata(dev);
	reg = st->reg;
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (0 == st->chip_config.accl_enable)
		return -EPERM;

	result = inv_i2c_read(st, reg->raw_accl, 6, data);
	if (result) {
		printk(KERN_ERR "Could not read raw registers.\n");
		return result;
	}

	return sprintf(buf, "%d %d %d %lld\n",
		((signed short)(be16_to_cpup((short *)&data[0]))*
				st->chip_info.multi),
		((signed short)(be16_to_cpup((short *)&data[2]))*
				st->chip_info.multi),
		((signed short)(be16_to_cpup((short *)&data[4]))*
				st->chip_info.multi),
		get_time_ns());
}

/**
 *  inv_temperature_show() - Read temperature data directly from registers.
 */
static ssize_t inv_temperature_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	int result;
	unsigned char data[2];

	st = dev_get_drvdata(dev);
	reg = st->reg;
	if (st->chip_config.is_asleep)
		return -EPERM;

	result = inv_i2c_read(st, reg->temperature, 2, data);
	if (result) {
		printk(KERN_ERR "Could not read temperature register.\n");
		return result;
	}

	return sprintf(buf, "%d %lld\n",
		(signed short)(be16_to_cpup((short *)&data[0])),
		get_time_ns());
}

static int inv_set_lpf(struct inv_gyro_state_s *st, int rate)
{
	short hz[6] = {188, 98, 42, 20, 10, 5};
	int   d[6] = {INV_FILTER_188HZ, INV_FILTER_98HZ,
			INV_FILTER_42HZ, INV_FILTER_20HZ,
			INV_FILTER_10HZ, INV_FILTER_5HZ};
	int i, h, data, result;
	struct inv_reg_map_s *reg;

	reg = st->reg;
	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < 6))
		i++;
	data = d[i];
	if (INV_MPU3050 == st->chip_type) {
		if (st->mpu_slave != NULL) {
			result = st->mpu_slave->set_lpf(st, rate);
			if (result)
				return result;
		}

		result = inv_i2c_single_write(st, reg->lpf,
			data | (st->chip_config.fsr << 3));

	} else {
		result = inv_i2c_single_write(st, reg->lpf, data);
	}

	if (result)
		return result;

	st->chip_config.lpf = data;
	return 0;
}

/**
 *  inv_fifo_rate_store() - Set fifo rate.
 */
static ssize_t inv_fifo_rate_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long fifo_rate;
	unsigned char data;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;

	st = dev_get_drvdata(dev);
	reg = st->reg;
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (kstrtoul(buf, 10, &fifo_rate))
		return -EINVAL;

	if ((fifo_rate < MIN_FIFO_RATE) || (fifo_rate > MAX_FIFO_RATE))
		return -EINVAL;

	if (fifo_rate == st->chip_config.fifo_rate)
		return count;

	if (st->has_compass) {
		data = 10*fifo_rate/ONE_K_HZ;
		if (data > 0)
			data -= 1;
		st->compass_divider = data;
		st->compass_counter = 0;
		/* I2C_MST_DLY is set according to sample rate,
		   AKM cannot be read or set at sample rate higher than 100Hz*/
		result = inv_i2c_single_write(st, REG_I2C_SLV4_CTRL, data);
		if (result)
			return result;
	}

	data = ONE_K_HZ / fifo_rate - 1;
	result = inv_i2c_single_write(st, reg->sample_rate_div, data);
	if (result)
		return result;

	st->chip_config.fifo_rate = fifo_rate;
	result = inv_set_lpf(st, fifo_rate);
	if (result)
		return result;

	st->irq_dur_us = (data + 1) * ONE_K_HZ;
	st->last_isr_time = get_time_ns();
	return count;
}

/**
 *  inv_power_state_store() - Turn device on/off.
 */
static ssize_t inv_power_state_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int result;
	unsigned long power_state;
	struct inv_gyro_state_s *st;

	st = dev_get_drvdata(dev);
	if (kstrtoul(buf, 10, &power_state))
		return -EINVAL;

	if (!power_state == st->chip_config.is_asleep)
		return count;

	result = inv_set_power_state(st, power_state);
	return count;
}

/**
 *  inv_enable_store() - Enable/disable chip operation.
 */
static ssize_t inv_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long enable;
	struct inv_gyro_state_s *st;
	int result;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (!enable == !st->chip_config.enable)
		return count;

	result = set_inv_enable(st, enable);
	if (result)
		return result;

	return count;
}

/**
 *  inv_accl_fifo_enable_store() - Enable/disable accl fifo output.
 */
static ssize_t inv_accl_fifo_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	unsigned long data, en;
	int result;
	struct inv_gyro_state_s *st;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;

	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.accl_fifo_enable)
		return count;

	if (en && (0 == st->chip_config.accl_enable)) {
		result = inv_set_power_state(st, 0);
		if (result)
			return result;

		st->chip_config.accl_enable = en;
		result = inv_set_power_state(st, 1);
		if (result)
			return result;
	}

	st->chip_config.accl_fifo_enable = en;
	return count;
}

/**
 *  inv_gyro_fifo_enable_store() - Enable/disable gyro fifo output.
 */
ssize_t inv_gyro_fifo_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long data, en;
	int result;
	struct inv_gyro_state_s *st;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;

	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.gyro_fifo_enable)
		return count;

	if (en && (0 == st->chip_config.gyro_enable)) {
		result = inv_set_power_state(st, 0);
		if (result)
			return result;

		st->chip_config.gyro_enable = en;
		result = inv_set_power_state(st, 1);
		if (result)
			return result;
	}

	st->chip_config.gyro_fifo_enable = en;
	return count;
}

/**
 *  inv_gyro_enable_store() - Enable/disable gyro.
 */
ssize_t inv_gyro_enable_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long data, en;
	struct inv_gyro_state_s *st;
	int result;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;

	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.gyro_enable)
		return count;

	if (0 == en)
		st->chip_config.gyro_fifo_enable = 0;
	result = inv_set_power_state(st, 0);
	if (result)
		return result;

	st->chip_config.gyro_enable = en;
	result = inv_set_power_state(st, 1);
	if (result) {
		st->chip_config.gyro_enable ^= 1;
		return result;
	}

	return count;
}

/**
 *  inv_accl_enable_store() - Enable/disable accl.
 */
static ssize_t inv_accl_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long data, en;
	struct inv_gyro_state_s *st;
	int result;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;

	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.accl_enable)
		return count;

	if (0 == en)
		st->chip_config.accl_fifo_enable = 0;
	result = inv_set_power_state(st, 0);
	if (result)
		return result;

	st->chip_config.accl_enable = en;
	result = inv_set_power_state(st, 1);
	if (result) {
		st->chip_config.accl_enable ^= 1;
		return result;
	}

	return count;
}

/**
 *  inv_gyro_fs_store() - Change the gyro full-scale range (and scale factor).
 */
static ssize_t inv_gyro_fs_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long fsr;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, &fsr);
	if (result)
		return -EINVAL;

	if (fsr > 3)
		return -EINVAL;

	if (fsr == st->chip_config.fsr)
		return count;

	reg = st->reg;
	if (INV_MPU3050 == st->chip_type)
		result = inv_i2c_single_write(st, reg->lpf,
			(fsr << 3) | st->chip_config.lpf);
	else
		result = inv_i2c_single_write(st, reg->gyro_config,
			fsr << 3);
	if (result)
		return result;

	st->chip_config.fsr = fsr;
	return count;
}

/**
 *  inv_accl_fs_store() - Configure the accelerometer's scale range.
 */
ssize_t inv_accl_fs_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	unsigned long fs;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (kstrtoul(buf, 10, &fs))
		return -EINVAL;

	if (fs > 3)
		return -EINVAL;

	if (fs == st->chip_config.accl_fs)
		return count;

	reg = st->reg;
	if ((INV_MPU3050 == st->chip_type) && (st->mpu_slave != NULL))
		result = st->mpu_slave->set_fs(st, fs);
	else
		result = inv_i2c_single_write(st, reg->accl_config, (fs << 3));
	if (result)
		return result;

	/* reset fifo because the data could be mixed with old bad data */
	st->chip_config.accl_fs = fs;
	return count;
}

/**
 * inv_firmware_loaded_store() -  calling this function will change
 *                        firmware load
 */
static ssize_t inv_firmware_loaded_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long data, result;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data != 0)
		return -EINVAL;

	st->chip_config.firmware_loaded = 0;
	st->chip_config.dmp_on = 0;
	return count;
}

/**
 *  inv_lpa_mode_store() - store current low power settings
 */
static ssize_t inv_lpa_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, lpa_mode;
	unsigned char d;
	struct inv_reg_map_s *reg;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, &lpa_mode);
	if (result)
		return result;

	reg = st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_1, 1, &d);
	if (result)
		return result;

	d &= ~BIT_CYCLE;
	if (lpa_mode)
		d |= BIT_CYCLE;
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, d);
	if (result)
		return result;

	st->chip_config.lpa_mode = lpa_mode;
	return count;
}

/**
 *  inv_lpa_freq_store() - store current low power frequency setting.
 */
static ssize_t inv_lpa_freq_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, lpa_freq;
	unsigned char d;
	struct inv_reg_map_s *reg;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, &lpa_freq);
	if (result)
		return result;

	if (lpa_freq > 3)
		return -EINVAL;

	reg = st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &d);
	if (result)
		return result;

	d &= ~BIT_LPA_FREQ;
	d |= (unsigned char)(lpa_freq << 6);
	result = inv_i2c_single_write(st, reg->pwr_mgmt_2, d);
	if (result)
		return result;

	st->chip_config.lpa_freq = lpa_freq;
	return count;
}

/**
 * inv_compass_en_store() -  calling this function will store compass
 *                         enable
 */
static ssize_t inv_compass_en_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	unsigned long data, result, en;

	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EPERM;

	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.compass_enable)
		return count;

	st->chip_config.compass_enable = en;
	return count;
}

/**
 *  inv_compass_scale_store() - show current compass scale settings
 */
static ssize_t inv_compass_scale_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	unsigned long data, result, en;
	char d;

	st = dev_get_drvdata(dev);
	if (COMPASS_ID_AK8963 != st->plat_data.sec_slave_id)
		return count;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data)
		en = 1;
	else
		en = 0;
	if (st->compass_scale == en)
		return count;

	st->compass_scale = en;
	d = (1 | (st->compass_scale << 4));
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, d);
	if (result)
		return result;

	return count;
}

/**
 * inv_flick_lower_store() -  calling this function will store current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data, out;
	unsigned char *p;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtol(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_LOWER, 4, p);
	if (result)
		return result;

	st->flick.lower = data;
	return count;
}

/**
 * inv_flick_upper_store() -  calling this function will store current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_UPPER, 4, p);
	if (result)
		return result;

	st->flick.upper = data;
	return count;
}

/**
 * inv_flick_counter_store() -  calling this function will store current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_COUNTER, 4, p);
	if (result)
		return result;

	st->flick.counter = data;
	return count;
}

/**
 * inv_flick_int_on_store() -  calling this function will store current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, data;
	unsigned char d[4];

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data)
		/* Use interrupt to signal when gesture was observed */
		d[0] = DIND40+4;
	else
		d[0] = DINAA0+8;
	result = mem_w_key(KEY_CGNOTICE_INTR, 1, d);
	if (result)
		return result;

	st->flick.int_on = data;
	return count;
}

/**
 * inv_flick_axis_store() -  calling this function will store current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned long result, data;
	unsigned char d[4];

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data == 0)
		d[0] = DINBC2;
	else if (data == 2)
		d[2] = DINBC6;
	else
		d[0] = DINBC4;
	result = mem_w_key(KEY_CFG_FLICK_IN, 1, d);
	if (result)
		return result;

	st->flick.axis = data;
	return count;
}

/**
 * inv_flick_msg_on_store() -  calling this function will store current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	if (data)
		data = DATA_MSG_ON;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_MSG, 4, p);
	if (result)
		return result;

	st->flick.msg_on = data;
	return count;
}

/**
 * inv_pedometer_steps_store() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_STEPCTR, 4, p);
	if (result)
		return result;

	return count;
}

/**
 * inv_pedometer_time_store() -  calling this function will store current
 *                        pedometer time into MPU memory
 */
static ssize_t inv_pedometer_time_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p;

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_TIMECTR, 4, p);
	if (result)
		return result;

	return count;
}

/**
 * inv_key_store() -  calling this function will store authenticate key
 */
static ssize_t inv_key_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned int result, data, out;
	unsigned char *p, d[4];

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w(D_AUTH_IN, 4, p);
	if (result)
		return result;

	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		D_AUTH_IN, 4, d);
	return count;
}

/**
 *  inv_gyro_fs_show() - Get the current gyro full-scale range.
 */
static ssize_t inv_gyro_fs_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", (1 << st->chip_config.fsr)*250);
}

/**
 *  inv_accl_fs_show() - Get the current gyro full-scale range.
 */
ssize_t inv_accl_fs_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", 2 << st->chip_config.accl_fs);
}

/**
 *  inv_clk_src_show() - Show the device's clock source.
 */
static ssize_t inv_clk_src_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	switch (st->chip_config.clk_src) {
	case INV_CLK_INTERNAL:
		return sprintf(buf, "INTERNAL\n");

	case INV_CLK_PLL:
		return sprintf(buf, "Gyro PLL\n");

	default:
		return sprintf(buf, "Oh no!\n");
	}
}

/**
 *  inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

/**
 *  inv_enable_show() - Check if the chip are enabled.
 */
static ssize_t inv_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.enable);
}

/**
 *  inv_gyro_fifo_enable_show() - Check if gyro FIFO are enabled.
 */
ssize_t inv_gyro_fifo_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.gyro_fifo_enable);
}

/**
 *  inv_accl_fifo_enable_show() - Check if accl FIFO are enabled.
 */
ssize_t inv_accl_fifo_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.accl_fifo_enable);
}

/**
 *  inv_gyro_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
ssize_t inv_gyro_enable_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.gyro_enable);
}

/**
 *  inv_accl_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
ssize_t inv_accl_enable_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.accl_enable);
}

/**
 *  inv_power_state_show() - Check if the device is on or in sleep mode.
 */
static ssize_t inv_power_state_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	if (st->chip_config.is_asleep)
		return sprintf(buf, "0\n");

	else
		return sprintf(buf, "1\n");
}

/**
 *  inv_temp_scale_show() - Get the temperature scale factor in LSBs/degree C.
 */
static ssize_t inv_temp_scale_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	if (INV_MPU3050 == st->chip_type)
		return sprintf(buf, "280\n");

	else
		return sprintf(buf, "340\n");
}

/**
 *  inv_temp_offset_show() - Get the temperature offset in LSBs/degree C.
 */
static ssize_t inv_temp_offset_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	if (INV_MPU3050 == st->chip_type)
		return sprintf(buf, "-13200\n");

	else
		return sprintf(buf, "-521\n");
}

/**
 *  inv_lpa_mode_show() - show current low power settings
 */
static ssize_t inv_lpa_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.lpa_mode);
}

/**
 *  inv_lpa_freq_show() - show current low power frequency setting
 */
static ssize_t inv_lpa_freq_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	switch (st->chip_config.lpa_freq) {
	case 0:
		return sprintf(buf, "1.25\n");

	case 1:
		return sprintf(buf, "5\n");

	case 2:
		return sprintf(buf, "20\n");

	case 3:
		return sprintf(buf, "40\n");

	default:
		return sprintf(buf, "0\n");
	}
}

/**
 *  inv_compass_scale_show() - show current compass scale settings
 */
static ssize_t inv_compass_scale_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	long scale;

	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id)
		scale = DATA_AKM8975_SCALE;
	else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id)
		scale = DATA_AKM8972_SCALE;
	else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id)
		if (st->compass_scale)
			scale = DATA_AKM8963_SCALE1;
		else
			scale = DATA_AKM8963_SCALE0;
	else
		return -EINVAL;

	scale *= (1L << 15);
	return sprintf(buf, "%ld\n", scale);
}

/**
 *  inv_reg_dump_show() - Register dump for testing.
 *  TODO: Only for testing.
 */
static ssize_t inv_reg_dump_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	ssize_t bytes_printed = 0;
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	for (ii = 0; ii < st->hw->num_reg; ii++) {
		/* don't read fifo r/w register */
		if (ii == st->reg->fifo_r_w)
			data = 0;
		else
			inv_i2c_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	return bytes_printed;
}

/**
 * inv_self_test_show() - self test result. 0 for fail; 1 for success.
 *                        calling this function will trigger self test
 *                        and return test result.
 */
static ssize_t inv_self_test_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int result;
	int bias[3];
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	if (INV_MPU3050 == st->chip_type) {
		bias[0] = bias[1] = bias[2] = 0;
		result = 0;
	} else {
		result = inv_hw_self_test(st, bias);
	}
	return sprintf(buf, "%d, %d, %d, %d\n",
		bias[0], bias[1], bias[2], result);
}

/**
 * inv_get_accl_bias_show() - show accl bias value
 */
static ssize_t inv_get_accl_bias_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int result;
	int bias[3];
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	result = inv_get_accl_bias(st, bias);
	if (result)
		return -EINVAL;

	return sprintf(buf, "%d, %d, %d\n", bias[0], bias[1], bias[2]);
}

/**
 * inv_gyro_matrix_show() - show orientation matrix
 */
static ssize_t inv_gyro_matrix_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	signed char *m;

	m = st->plat_data.orientation;
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		       m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * inv_accl_matrix_show() - show orientation matrix
 */
ssize_t inv_accl_matrix_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	signed char *m;

	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_ACCEL)
		m = st->plat_data.secondary_orientation;
	else
		m = st->plat_data.orientation;
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		       m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * inv_compass_matrix_show() - show orientation matrix
 */
static ssize_t inv_compass_matrix_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	signed char *m;

	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_COMPASS)
		m = st->plat_data.secondary_orientation;
	else
		return -1;

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		       m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * inv_key_show() -  calling this function will show the key
 *
 */
static ssize_t inv_key_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	unsigned char *key;
	key = st->plat_data.key;

	return sprintf(buf,
			"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			key[0], key[1], key[2], key[3], key[4], key[5], key[6],
			key[7], key[8], key[9], key[10], key[11], key[12],
			key[13], key[14], key[15]);
}

/**
 * inv_firmware_loaded_show() -  calling this function will show current
 *                        firmware load status
 */
static ssize_t inv_firmware_loaded_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.firmware_loaded);
}

/**
 * inv_compass_en_show() -  calling this function will show compass
 *                         enable status
 */
static ssize_t inv_compass_en_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->chip_config.compass_enable);
}

/**
 * inv_flick_lower_show() -  calling this function will show current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.lower);
}

/**
 * inv_flick_upper_show() -  calling this function will show current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.upper);
}

/**
 * inv_flick_counter_show() -  calling this function will show current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.counter);
}

/**
 * inv_flick_int_on_show() -  calling this function will show current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.int_on);
}

/**
 * inv_flick_axis_show() -  calling this function will show current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.axis);
}

/**
 * inv_flick_msg_on_show() -  calling this function will show current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->flick.msg_on);
}

/**
 * inv_pedometer_steps_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data;
	unsigned char d[4];

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
				 inv_dmp_get_address(KEY_D_PEDSTD_STEPCTR),
				 4, d);
	if (result)
		return result;

	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data);
}

/**
 * inv_pedometer_time_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_time_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	int result, data;
	unsigned char d[4];

	if (st->chip_config.is_asleep)
		return -EPERM;

	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
				 inv_dmp_get_address(KEY_D_PEDSTD_TIMECTR),
				 4, d);
	if (result)
		return result;

	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data);
}

static void inv_report_gyro_accl(struct inv_gyro_state_s *st, s64 t,
				 unsigned char *data)
{
	short x, y, z;
	int ind;
	struct inv_chip_config_s *conf;

	conf = &st->chip_config;
	ind = 0;
	if (conf->accl_fifo_enable | conf->dmp_on) {
		x = ((data[ind] << 8)|data[ind + 1])*st->chip_info.multi;
		y = ((data[ind + 2] << 8)|data[ind + 3])*st->chip_info.multi;
		z = ((data[ind + 4] << 8)|data[ind + 5])*st->chip_info.multi;
		if (conf->accl_fifo_enable) {
			/*it is possible that accl disabled when dmp is on*/
			input_report_rel(st->idev, REL_RX,  x);
			input_report_rel(st->idev, REL_RY,  y);
			input_report_rel(st->idev, REL_RZ,  z);
		}
		ind += 6;
	}
	if (conf->gyro_fifo_enable | conf->dmp_on) {
		x = (data[ind] << 8)     | data[ind + 1];
		y = (data[ind + 2] << 8) | data[ind + 3];
		z = (data[ind + 4] << 8) | data[ind + 5];
		if (conf->gyro_fifo_enable) {
			/*it is possible that gyro disabled when dmp is on*/
			input_report_rel(st->idev, REL_X,  x);
			input_report_rel(st->idev, REL_Y,  y);
			input_report_rel(st->idev, REL_Z,  z);
		}
		ind += 6;
	}
	if (conf->dmp_on) {
		/* report tap information */
		if (data[ind + 1] & 1) {
			input_report_rel(st->idev_dmp, REL_RX, data[ind+3]);
			input_sync(st->idev_dmp);
		}
		/* report orientation information */
		if (data[ind + 1] & 2) {
			input_report_rel(st->idev_dmp, REL_RY, data[ind+2]);
			input_sync(st->idev_dmp);
		}
	}
	if (conf->accl_fifo_enable | conf->gyro_fifo_enable) {
		input_report_rel(st->idev, REL_MISC, (unsigned int)(t >> 32));
		input_report_rel(st->idev, REL_WHEEL,
			(unsigned int)(t & 0xffffffff));
		input_sync(st->idev);
	}
}

static int inv_report_compass(struct inv_gyro_state_s *st, s64 t)
{
	short x, y, z;
	int result;
	unsigned char data[8];

	/*mpu_memory_read(st->sl_handle,
		st->i2c_addr,
		14,
		10, data);*/
	/*divider and counter is used to decrease the speed of read in
		high frequency sample rate*/
	if (st->compass_divider == st->compass_counter) {
		/*read from external sensor data register */
		result = inv_i2c_read(st, REG_EXT_SENS_DATA_00, 8, data);
		if (result)
			return result;

		/* data[7] is status 2 register */
		/*for AKM8975, bit 2 and 3 should be all be zero*/
		/* for AMK8963, bit 3 should be zero*/
		if ((DATA_AKM_DRDY == data[0]) &&
			(0 == (data[7] & DATA_AKM_STAT_MASK))) {
			unsigned char *sens;
			sens = st->chip_info.compass_sens;
			x = (short)((data[2] << 8) | data[1]);
			y = (short)((data[4] << 8) | data[3]);
			z = (short)((data[6] << 8) | data[5]);
			x = ((x * (sens[0] + 128)) >> 8);
			y = ((y * (sens[1] + 128)) >> 8);
			z = ((z * (sens[2] + 128)) >> 8);
			input_report_rel(st->idev_compass, REL_X, x);
			input_report_rel(st->idev_compass, REL_Y, y);
			input_report_rel(st->idev_compass, REL_Z, z);
			input_report_rel(st->idev_compass, REL_MISC,
					 (unsigned int)(t >> 32));
			input_report_rel(st->idev_compass, REL_WHEEL,
					 (unsigned int)(t & 0xffffffff));
			input_sync(st->idev_compass);
		}
		st->compass_counter = 0;

	} else if (st->compass_divider != 0) {
		st->compass_counter++;
	}

	return 0;
}

/**
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
static irqreturn_t inv_read_fifo(int irq, void *dev_id)
{
	struct inv_gyro_state_s *st;
	unsigned char bytes_per_datum;
	const unsigned short fifo_thresh = 500;
	int result;
	unsigned char data[16];
	unsigned short fifo_count;
	unsigned int copied;
	s64 timestamp;
	struct inv_reg_map_s *reg;

	st = (struct inv_gyro_state_s *)dev_id;
	reg = st->reg;
	if (st->chip_config.is_asleep)
		goto end_session;

	if (!(st->chip_config.enable))
		goto end_session;

	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.dmp_on |
		st->chip_config.compass_enable))
		goto end_session;

	if (st->chip_config.dmp_on && st->flick.int_on) {
		/*dmp interrupt status */
		inv_i2c_read(st, REG_DMP_INT_STATUS, 2, data);
		if (data[0] & 8) {
			input_report_rel(st->idev_dmp, REL_RZ, data[0]);
			input_sync(st->idev_dmp);
		}
	}
	if (st->chip_config.lpa_mode) {
		result = inv_i2c_read(st, reg->raw_accl, 6, data);
		if (result)
			goto end_session;

		inv_report_gyro_accl(st, get_time_ns(), data);
		goto end_session;
	}

	if (st->chip_config.dmp_on)
		bytes_per_datum = BYTES_FOR_DMP;
	else
		bytes_per_datum = (st->chip_config.accl_fifo_enable +
			    st->chip_config.gyro_fifo_enable)*BYTES_PER_SENSOR;
	fifo_count = 0;
	if (bytes_per_datum != 0) {
		result = inv_i2c_read(st, reg->fifo_count_h, 2, data);
		if (result)
			goto end_session;

		fifo_count = (data[0] << 8) + data[1];
		if (fifo_count < bytes_per_datum)
			goto end_session;

		if (fifo_count%2)
			goto flush_fifo;

		if (fifo_count > fifo_thresh)
			goto flush_fifo;

		/* Timestamp mismatch. */
		if (kfifo_len(&st->trigger.timestamps) <
			fifo_count / bytes_per_datum)
			goto flush_fifo;

		if (kfifo_len(&st->trigger.timestamps) >
			fifo_count / bytes_per_datum + TIME_STAMP_TOR) {
			if (st->chip_config.dmp_on) {
				result = kfifo_to_user(&st->trigger.timestamps,
				&timestamp, sizeof(timestamp), &copied);
				if (result)
					goto flush_fifo;

			} else {
				goto flush_fifo;
			}
		}
	}

	if (bytes_per_datum == 0) {
		result = kfifo_to_user(&st->trigger.timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
	}

	while ((bytes_per_datum != 0) && (fifo_count >= bytes_per_datum)) {
		result = inv_i2c_read(st, reg->fifo_r_w, bytes_per_datum,
			data);
		if (result)
			goto flush_fifo;

		result = kfifo_to_user(&st->trigger.timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;

		inv_report_gyro_accl(st, timestamp, data);
		fifo_count -= bytes_per_datum;
	}

	if (st->chip_config.compass_enable)
		inv_report_compass(st, timestamp);

end_session:
	return IRQ_HANDLED;

flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(st);
	inv_clear_kfifo(st);
	return IRQ_HANDLED;
}

/**
 *  inv_irq_handler() - Cache a timestamp at each data ready interrupt.
 */
static irqreturn_t inv_irq_handler(int irq, void *dev_id)
{
	struct inv_gyro_state_s *st;
	long long timestamp;
	int result, catch_up;
	unsigned int time_since_last_irq;

	st = (struct inv_gyro_state_s *)dev_id;
	timestamp = get_time_ns();
	time_since_last_irq = ((unsigned int)(timestamp - st->last_isr_time)) /
			      ONE_K_HZ;
	spin_lock(&st->time_stamp_lock);
	catch_up = 0;
	while ((time_since_last_irq > st->irq_dur_us*2) &&
	       (catch_up < MAX_CATCH_UP) && (0 == st->chip_config.lpa_mode)) {
		st->last_isr_time += st->irq_dur_us * ONE_K_HZ;
		result = kfifo_in(&st->trigger.timestamps,
				  &st->last_isr_time, 1);
		time_since_last_irq = ((unsigned int)(timestamp -
					st->last_isr_time)) / ONE_K_HZ;
		catch_up++;
	}
	result = kfifo_in(&st->trigger.timestamps, &timestamp, 1);
	st->last_isr_time = timestamp;
	spin_unlock(&st->time_stamp_lock);
	return IRQ_WAKE_THREAD;
}

#ifdef CONFIG_PM
static int inv_suspend(struct device *dev)
{
	int result;
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	if (inv_set_power_state(st, 0))
		dev_err(dev, "inv_set_power_state in inv_suspend failed\n");
	/* vdd is used to gate the vlogic. Disable vlogic first. */
	if (st->inv_regulator.regulator_vdd &&
		st->inv_regulator.regulator_vlogic) {
		result = regulator_disable(st->inv_regulator.regulator_vlogic);
		if (result < 0)
			dev_err(dev, "regulator_disable for vlogic failed\n");
		result = regulator_disable(st->inv_regulator.regulator_vdd);
		if (result < 0)
			dev_err(dev, "regulator_disable for vdd failed\n");
	}
	return 0;
}

static int inv_resume(struct device *dev)
{
	int result;
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	/* vdd is used to gate the vlogic. Enable vdd first. */
	if (st->inv_regulator.regulator_vdd &&
		st->inv_regulator.regulator_vlogic) {
		result = regulator_enable(st->inv_regulator.regulator_vdd);
		if (result < 0)
			dev_err(dev, "regulator_enable for vdd failed\n");
		result = regulator_enable(st->inv_regulator.regulator_vlogic);
		if (result < 0)
			dev_err(dev, "regulator_enable for vlogic failed\n");
	}
	if (inv_set_power_state(st, 1))
		dev_err(dev, "inv_set_power_state in inv_resume failed\n");
	return 0;
}

static const struct dev_pm_ops inv_pm_ops = {
	.suspend = inv_suspend,
	.resume = inv_resume,
};
#endif

static DEVICE_ATTR(raw_gyro, S_IRUGO, inv_raw_gyro_show, NULL);
static DEVICE_ATTR(raw_accl, S_IRUGO, inv_raw_accl_show, NULL);
static DEVICE_ATTR(temperature, S_IRUGO, inv_temperature_show, NULL);
static DEVICE_ATTR(fifo_rate, S_IRUGO | S_IWUSR, inv_fifo_rate_show,
		   inv_fifo_rate_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, inv_enable_show,
		   inv_enable_store);
static DEVICE_ATTR(gyro_fifo_enable, S_IRUGO | S_IWUSR,
		   inv_gyro_fifo_enable_show, inv_gyro_fifo_enable_store);
static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR, inv_gyro_enable_show,
		   inv_gyro_enable_store);
static DEVICE_ATTR(accl_fifo_enable, S_IRUGO | S_IWUSR,
		   inv_accl_fifo_enable_show, inv_accl_fifo_enable_store);
static DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR, inv_accl_enable_show,
		   inv_accl_enable_store);
static DEVICE_ATTR(accl_fs, S_IRUGO | S_IWUSR, inv_accl_fs_show,
		   inv_accl_fs_store);
static DEVICE_ATTR(gyro_fs, S_IRUGO | S_IWUSR, inv_gyro_fs_show,
		   inv_gyro_fs_store);
static DEVICE_ATTR(clock_source, S_IRUGO, inv_clk_src_show, NULL);
static DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR, inv_power_state_show,
		   inv_power_state_store);
static DEVICE_ATTR(firmware_loaded, S_IRUGO | S_IWUSR,
		   inv_firmware_loaded_show, inv_firmware_loaded_store);
static DEVICE_ATTR(lpa_mode, S_IRUGO | S_IWUSR, inv_lpa_mode_show,
		   inv_lpa_mode_store);
static DEVICE_ATTR(lpa_freq, S_IRUGO | S_IWUSR, inv_lpa_freq_show,
		   inv_lpa_freq_store);
static DEVICE_ATTR(compass_enable, S_IRUGO | S_IWUSR, inv_compass_en_show,
		   inv_compass_en_store);
static DEVICE_ATTR(compass_scale, S_IRUGO | S_IWUSR, inv_compass_scale_show,
		   inv_compass_scale_store);
static DEVICE_ATTR(temp_scale, S_IRUGO, inv_temp_scale_show, NULL);
static DEVICE_ATTR(temp_offset, S_IRUGO, inv_temp_offset_show, NULL);
static DEVICE_ATTR(reg_dump, S_IRUGO, inv_reg_dump_show, NULL);
static DEVICE_ATTR(self_test, S_IRUGO, inv_self_test_show, NULL);
static DEVICE_ATTR(key, S_IRUGO | S_IWUSR, inv_key_show, inv_key_store);
static DEVICE_ATTR(gyro_matrix, S_IRUGO, inv_gyro_matrix_show, NULL);
static DEVICE_ATTR(accl_matrix, S_IRUGO, inv_accl_matrix_show, NULL);
static DEVICE_ATTR(compass_matrix, S_IRUGO, inv_compass_matrix_show, NULL);
static DEVICE_ATTR(accl_bias, S_IRUGO, inv_get_accl_bias_show, NULL);
static DEVICE_ATTR(flick_lower, S_IRUGO | S_IWUSR, inv_flick_lower_show,
		   inv_flick_lower_store);
static DEVICE_ATTR(flick_upper, S_IRUGO | S_IWUSR, inv_flick_upper_show,
		   inv_flick_upper_store);
static DEVICE_ATTR(flick_counter, S_IRUGO | S_IWUSR, inv_flick_counter_show,
		   inv_flick_counter_store);
static DEVICE_ATTR(flick_message_on, S_IRUGO | S_IWUSR, inv_flick_msg_on_show,
		   inv_flick_msg_on_store);
static DEVICE_ATTR(flick_int_on, S_IRUGO | S_IWUSR, inv_flick_int_on_show,
		   inv_flick_int_on_store);
static DEVICE_ATTR(flick_axis, S_IRUGO | S_IWUSR, inv_flick_axis_show,
		   inv_flick_axis_store);
static DEVICE_ATTR(pedometer_time, S_IRUGO | S_IWUSR, inv_pedometer_time_show,
		   inv_pedometer_time_store);
static DEVICE_ATTR(pedometer_steps, S_IRUGO | S_IWUSR,
		   inv_pedometer_steps_show, inv_pedometer_steps_store);

static struct device_attribute *inv_attributes[] = {
	&dev_attr_raw_gyro,
	&dev_attr_temperature,
	&dev_attr_fifo_rate,
	&dev_attr_enable,
	&dev_attr_clock_source,
	&dev_attr_power_state,
	&dev_attr_gyro_fs,
	&dev_attr_temp_scale,
	&dev_attr_temp_offset,
	&dev_attr_reg_dump,
	&dev_attr_self_test,
	&dev_attr_key,
	&dev_attr_gyro_matrix,
	NULL
};

static struct device_attribute *inv_mpu6050_attributes[] = {
	&dev_attr_gyro_fifo_enable,
	&dev_attr_gyro_enable,
	&dev_attr_accl_fifo_enable,
	&dev_attr_accl_enable,
	&dev_attr_accl_fs,
	&dev_attr_accl_bias,
	&dev_attr_raw_accl,
	&dev_attr_accl_matrix,
	&dev_attr_firmware_loaded,
	&dev_attr_lpa_mode,
	&dev_attr_lpa_freq,
	&dev_attr_flick_lower,
	&dev_attr_flick_upper,
	&dev_attr_flick_counter,
	&dev_attr_flick_message_on,
	&dev_attr_flick_int_on,
	&dev_attr_flick_axis,
	&dev_attr_pedometer_time,
	&dev_attr_pedometer_steps,
	NULL
};

static struct device_attribute *inv_compass_attributes[] = {
	&dev_attr_compass_enable,
	&dev_attr_compass_scale,
	&dev_attr_compass_matrix,
	NULL
};

static void inv_init_regulator(struct inv_gyro_state_s *st,
					struct i2c_client *client)
{
	int result;

	/* Power regulator registration*/
	st->inv_regulator.regulator_vdd =
			devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(st->inv_regulator.regulator_vdd)) {
		dev_info(&client->adapter->dev,
			"regulator_get for vdd failed: %s\n",
			dev_name(&client->dev));
		dev_info(&client->adapter->dev,
			"Error code for vdd: %d",
				PTR_ERR(st->inv_regulator.regulator_vdd));
		goto err_null_regulator;
	}

	st->inv_regulator.regulator_vlogic =
			devm_regulator_get(&client->dev, "vlogic");
	if (IS_ERR(st->inv_regulator.regulator_vlogic)) {
		dev_info(&client->adapter->dev,
			"regulator_get for vlogic failed: %s\n",
			dev_name(&client->dev));
		dev_info(&client->adapter->dev,
			"Error code for vlogic: %d\n",
				PTR_ERR(st->inv_regulator.regulator_vlogic));
		goto err_put_regulator;
	}

	dev_info(&client->adapter->dev,
		"regulator_get for vdd and vlogic succeeded: %s\n",
		dev_name(&client->dev));

	result = regulator_enable(st->inv_regulator.regulator_vdd);
	if (result < 0)
		dev_err(&client->adapter->dev,
			"regulator_enable for vdd failed: %d\n", result);

	result = regulator_enable(st->inv_regulator.regulator_vlogic);
	if (result < 0)
		dev_err(&client->adapter->dev,
			"regulator_enable for vlogic failed: %d\n", result);

	return;

err_put_regulator:
	devm_regulator_put(st->inv_regulator.regulator_vdd);
err_null_regulator:
	st->inv_regulator.regulator_vdd = NULL;
	st->inv_regulator.regulator_vlogic = NULL;
}

static int inv_setup_compass(struct inv_gyro_state_s *st)
{
	int result;
	unsigned char data[4];

	result = inv_i2c_read(st, 1, 1, data);
	if (result)
		return result;

	data[0] &= ~(0x80);
	data[0] |= (st->plat_data.level_shifter << 7);
	/*set up VDDIO register */
	result = inv_i2c_single_write(st, 1, data[0]);
	if (result)
		return result;

	/* set to bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, BIT_BYPASS_EN);
	if (result)
		return result;

	/*read secondary i2c ID register */
	result = inv_secondary_read(REG_AKM_ID, 2, data);
	if (result)
		return result;

	if (data[0] != DATA_AKM_ID)
		return -ENXIO;

	/*set AKM to Fuse ROM access mode */
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_FR);
	if (result)
		return result;

	result = inv_secondary_read(REG_AKM_SENSITIVITY, 3,
					st->chip_info.compass_sens);
	if (result)
		return result;

	/*revert to power down mode */
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_DN);
	if (result)
		return result;

	printk(KERN_ERR"senx=%d, seny=%d,senz=%d\n",
		st->chip_info.compass_sens[0],
		st->chip_info.compass_sens[1],
		st->chip_info.compass_sens[2]);
	/*restore to non-bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, 0);
	if (result)
		return result;

	/*setup master mode and master clock and ES bit*/
	result = inv_i2c_single_write(st, REG_I2C_MST_CTRL, BIT_WAIT_FOR_ES);
	if (result)
		return result;

	/* slave 0 is used to read data from compass */
	/*read mode */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_ADDR, BIT_I2C_READ|
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;

	/* AKM status register address is 2 */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_REG, REG_AKM_STATUS);
	if (result)
		return result;

	/* slave 0 is enabled at the beginning, read 8 bytes from here */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_CTRL, BIT_SLV_EN | 8);
	if (result)
		return result;

	/*slave 1 is used for AKM mode change only*/
	/*write mode, slave address 0x0E */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_ADDR,
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;

	/* AKM mode register address is 0x0A */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_REG, REG_AKM_MODE);
	if (result)
		return result;

	/* slave 1 is enabled, byte length is 1 */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_CTRL, BIT_SLV_EN | 1);
	if (result)
		return result;

	/* output data for slave 1 is fixed, single measure mode*/
	st->compass_scale = 1;
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8975_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8975_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8975_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8975_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8975_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8975_ST_Z_LW;
		data[0] = 1;
	} else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8972_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8972_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8972_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8972_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8972_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8972_ST_Z_LW;
		data[0] = 1;
	} else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8963_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8963_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8963_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8963_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8963_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8963_ST_Z_LW;
		data[0] = (1 | (st->compass_scale << 4));
	}
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, data[0]);
	if (result)
		return result;

	/* slave 0 and 1 timer action is enabled every sample*/
	result = inv_i2c_single_write(st, REG_I2C_MST_DELAY_CTRL,
				BIT_SLV0_DLY_EN | BIT_SLV1_DLY_EN);
	return result;
}

static int inv_check_chip_type(struct inv_gyro_state_s *st,
			       const struct i2c_device_id *id)
{
	struct inv_reg_map_s *reg;
	int result;

	reg = st->reg;
	st->mpu_slave = NULL;
	if (!strcmp(id->name, "itg3500"))
		st->chip_type = INV_ITG3500;
	else if (!strcmp(id->name, "mpu3050")) {
		st->chip_type = INV_MPU3050;
		inv_setup_reg_mpu3050(reg);
	} else if (!strcmp(id->name, "mpu6050"))
		st->chip_type = INV_MPU6050;
	else if (!strcmp(id->name, "mpu9150"))
		st->chip_type = INV_MPU9150;
	if (INV_MPU9150 == st->chip_type) {
		st->plat_data.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS;
		st->plat_data.sec_slave_id = COMPASS_ID_AK8975;
		/*is MPU9150 i2c address hard coded? */
		/*st->plat_data.secondary_i2c_addr = 0xE;*/
		st->has_compass = 1;
	}
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		if (st->plat_data.sec_slave_id == ACCEL_ID_KXTF9)
			inv_register_kxtf9_slave(st);
	}
	if (SECONDARY_SLAVE_TYPE_COMPASS == st->plat_data.sec_slave_type)
		st->has_compass = 1;
	else
		st->has_compass = 0;
	st->chip_config.gyro_enable = 1;
	/*reset register to power up default*/
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, BIT_RESET);
	if (result)
		return result;

	mdelay(POWER_UP_TIME);
	result = inv_set_power_state(st, 1);
	if (result)
		return result;

	if (st->has_compass) {
		result = inv_setup_compass(st);
		if (result)
			return result;
	}

	return 0;
}

static int inv_create_input(struct inv_gyro_state_s *st,
		struct i2c_client *client){
	struct input_dev *idev;
	int result;

	idev = NULL;
	result = inv_setup_input(st, &idev, client, st->hw->name);
	if (result)
		return result;

	st->idev = idev;
	if (INV_ITG3500 == st->chip_type)
		return 0;

	result = inv_setup_input(st, &idev, client, "INV_DMP");
	if (result) {
		input_unregister_device(st->idev);
		return result;
	}

	st->idev_dmp = idev;
	if (!st->has_compass)
		return 0;

	if (st->has_compass) {
		result = inv_setup_input(st, &idev, client, "INV_COMPASS");
		if (result) {
			input_unregister_device(st->idev);
			input_unregister_device(st->idev_dmp);
			return result;
		}

		st->idev_compass = idev;
	}

	return 0;
}

int create_device_attributes(struct device *dev,
	struct device_attribute **attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i] ; ++i) {
		err = sysfs_create_file(&dev->kobj, &attrs[i]->attr);
		if (0 != err)
			break;
	}
	if (0 != err) {
		for (; i >= 0 ; --i)
			sysfs_remove_file(&dev->kobj, &attrs[i]->attr);
	}
	return err;
}

void remove_device_attributes(struct device *dev,
	struct device_attribute **attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i] ; ++i)
		device_remove_file(dev, attrs[i]);
}

static char const *const inv_class_name = "invensense";
static char const *const inv_device_name = "mpu";
static dev_t const inv_device_dev_t = MKDEV(MISC_MAJOR, MISC_DYNAMIC_MINOR);
static struct bin_attribute dmp_firmware = {
	.attr = {
		.name = "dmp_firmware",
		.mode = S_IRUGO | S_IWUSR
	},
	.size = 4096,
	.read = inv_dmp_firmware_read,
	.write = inv_dmp_firmware_write,
};

static int create_sysfs_interfaces(struct inv_gyro_state_s *st)
{
	int result;
	result = 0;

	st->inv_class = class_create(THIS_MODULE, inv_class_name);
	if (IS_ERR(st->inv_class)) {
		result = PTR_ERR(st->inv_class);
		goto exit_nullify_class;
	}

	st->inv_dev = device_create(st->inv_class, &st->i2c->dev,
			inv_device_dev_t, st, inv_device_name);
	if (IS_ERR(st->inv_dev)) {
		result = PTR_ERR(st->inv_dev);
		goto exit_destroy_class;
	}

	result = create_device_attributes(st->inv_dev, inv_attributes);
	if (result < 0)
		goto exit_destroy_device;

	if (INV_ITG3500 == st->chip_type)
		return 0;

	result = sysfs_create_bin_file(&st->inv_dev->kobj, &dmp_firmware);
	if (result < 0)
		goto exit_remove_device_attributes;

	if (INV_MPU3050 == st->chip_type) {
		result = inv_mpu3050_create_sysfs(st);
		if (result)
			goto exit_remove_bin_file;

		return 0;
	}

	result = create_device_attributes(st->inv_dev, inv_mpu6050_attributes);
	if (result < 0)
		goto exit_remove_bin_file;

	if (!st->has_compass)
		return 0;

	result = create_device_attributes(st->inv_dev, inv_compass_attributes);
	if (result < 0)
		goto exit_remove_6050_attributes;

	return 0;

exit_remove_6050_attributes:
	remove_device_attributes(st->inv_dev, inv_mpu6050_attributes);
exit_remove_bin_file:
	sysfs_remove_bin_file(&st->inv_dev->kobj, &dmp_firmware);
exit_remove_device_attributes:
	remove_device_attributes(st->inv_dev, inv_attributes);
exit_destroy_device:
	device_destroy(st->inv_class, inv_device_dev_t);
exit_destroy_class:
	st->inv_dev = NULL;
	class_destroy(st->inv_class);
exit_nullify_class:
	st->inv_class = NULL;
	return result;
}

static void remove_sysfs_interfaces(struct inv_gyro_state_s *st)
{
	remove_device_attributes(st->inv_dev, inv_attributes);
	if (INV_ITG3500 != st->chip_type)
		sysfs_remove_bin_file(&st->inv_dev->kobj, &dmp_firmware);
	if ((INV_ITG3500 != st->chip_type) && (INV_MPU3050 != st->chip_type))
		remove_device_attributes(st->inv_dev, inv_mpu6050_attributes);
	if (INV_MPU3050 == st->chip_type)
		inv_mpu3050_remove_sysfs(st);
	if (st->has_compass)
		remove_device_attributes(st->inv_dev, inv_compass_attributes);
	device_destroy(st->inv_class, inv_device_dev_t);
	class_destroy(st->inv_class);
	st->inv_dev = NULL;
	st->inv_class = NULL;
}

static int inv_mod_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct inv_gyro_state_s *st;
	int result;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		result = -ENOMEM;
		goto out_no_free;
	}

	inv_init_regulator(st, client);

	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, st);
	st->i2c = client;
	st->sl_handle = client->adapter;
	st->reg = (struct inv_reg_map_s *)&chip_reg ;
	st->hw  = (struct inv_hw_s *)hw_info;
	st->i2c_addr = client->addr;
	st->plat_data =
		*(struct mpu_platform_data *)dev_get_platdata(&client->dev);

	/* power is turned on inside check chip type*/
	result = inv_check_chip_type(st, id);
	if (result)
		goto out_free;

	st->hw  = (struct inv_hw_s *)(hw_info  + st->chip_type);
	if (INV_MPU3050 == st->chip_type)
		result = inv_init_config_mpu3050(st);
	else
		result = inv_init_config(st);
	if (result) {
		dev_err(&client->adapter->dev,
			"Could not initialize device.\n");
		goto out_free;
	}

	if (INV_ITG3500 != st->chip_type && INV_MPU3050 != st->chip_type) {
		result = inv_get_silicon_rev_mpu6050(st);
		if (result) {
			dev_err(&client->adapter->dev,
				"%s get silicon error.\n", st->hw->name);
			goto out_free;
		}
	}

	result = inv_set_power_state(st, 0);
	if (result) {
		dev_err(&client->adapter->dev,
			"%s could not be turned off.\n", st->hw->name);
		goto out_free;
	}

	INIT_KFIFO(st->trigger.timestamps);
	result = create_sysfs_interfaces(st);
	if (result)
		goto out_free_kfifo;

	if (!client->irq) {
		dev_err(&client->adapter->dev, "IRQ not assigned.\n");
		result = -EPERM;
		goto out_close_sysfs;
	}

	st->trigger.irq = client->irq;
	if (INV_MPU3050 == st->chip_type)
		result = request_threaded_irq(client->irq, inv_irq_handler,
					      inv_read_fifo_mpu3050,
					      IRQF_TRIGGER_RISING |
					      IRQF_SHARED, "inv_irq", st);
	else
		result = request_threaded_irq(client->irq, inv_irq_handler,
					      inv_read_fifo,
					      IRQF_TRIGGER_RISING |
					      IRQF_SHARED, "inv_irq", st);
	if (result)
		goto out_close_sysfs;

	spin_lock_init(&st->time_stamp_lock);
	result = inv_create_input(st, client);
	if (result) {
		free_irq(client->irq, st);
		goto out_close_sysfs;
	}

	pr_info("%s: Probe name %s\n", __func__, id->name);
	dev_info(&client->adapter->dev, "%s is ready to go!\n", st->hw->name);
	return 0;

out_close_sysfs:
	remove_sysfs_interfaces(st);
out_free_kfifo:
	kfifo_free(&st->trigger.timestamps);
out_free:
	result = inv_i2c_single_write(st, st->reg->pwr_mgmt_1, BIT_RESET);
	if (st->inv_regulator.regulator_vlogic &&
			st->inv_regulator.regulator_vdd) {
		regulator_disable(st->inv_regulator.regulator_vlogic);
		regulator_disable(st->inv_regulator.regulator_vdd);
	}
	kfree(st);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
}

static int inv_mod_remove(struct i2c_client *client)
{
	int result;
	struct inv_gyro_state_s *st = i2c_get_clientdata(client);

	result = inv_set_power_state(st, 0);
	if (result)
		dev_err(&client->adapter->dev, "%s could not be turned off.\n",
			st->hw->name);
	remove_sysfs_interfaces(st);
	result = inv_i2c_single_write(st, st->reg->pwr_mgmt_1, BIT_RESET);
	kfifo_free(&st->trigger.timestamps);
	free_irq(client->irq, st);
	input_unregister_device(st->idev);
	if (INV_ITG3500 != st->chip_type)
		input_unregister_device(st->idev_dmp);
	if (st->has_compass)
		input_unregister_device(st->idev_compass);
	if (st->inv_regulator.regulator_vlogic &&
			st->inv_regulator.regulator_vdd) {
		regulator_disable(st->inv_regulator.regulator_vlogic);
		regulator_disable(st->inv_regulator.regulator_vdd);
	}
	kfree(st);
	dev_info(&client->adapter->dev, "Gyro module removed.\n");
	return 0;
}

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* device id table is used to identify what device can be
 * supported by this driver
 */
static struct i2c_device_id inv_mod_id[] = {
	{"itg3500", 0},
	{"mpu3050", 0},
	{"mpu6050", 0},
	{"mpu9150", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_mod_id);

static struct i2c_driver inv_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_mod_probe,
	.remove		=	inv_mod_remove,
	.id_table	=	inv_mod_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv_dev",
#ifdef CONFIG_PM
		.pm	=	&inv_pm_ops,
#endif
	},
	.address_list = normal_i2c,
};

static int __init inv_mod_init(void)
{
	int result = i2c_add_driver(&inv_mod_driver);

	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}

	return 0;
}

static void __exit inv_mod_exit(void)
{
	i2c_del_driver(&inv_mod_driver);
}

module_init(inv_mod_init);
module_exit(inv_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv_dev");

