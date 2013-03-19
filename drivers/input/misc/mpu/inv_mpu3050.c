/*
* Copyright (C) 2012 Invensense, Inc.
* Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
 *      @file    inv_mpu3050.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This file is part of inv_gyro driver code
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
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "inv_gyro.h"

static unsigned long nvi_lpf_us_tbl[] = {
	0,	/* N/A */
	5319,	/* 188Hz */
	10204,	/* 98Hz */
	23810,	/* 42Hz */
	50000,	/* 20Hz */
	100000,	/* 10Hz */
	/* 200000, 5Hz */
};


/**
 *  inv_clear_kfifo() - clear time stamp fifo
 *  @st:	Device driver instance.
 */
static void inv_clear_kfifo(struct inv_gyro_state_s *st)
{
	unsigned long flags;
	spin_lock_irqsave(&st->time_stamp_lock, flags);
	kfifo_reset(&st->trigger.timestamps);
	spin_unlock_irqrestore(&st->time_stamp_lock, flags);
}

/**
 *  mpu3050_fifo_reset() - Reset FIFO related registers
 *  @st:	Device driver instance.
 */
static int mpu3050_fifo_reset(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char val, user_ctrl;

	reg = st->reg;
	/* disable interrupt */
	result = inv_i2c_single_write(st, reg->int_enable, 0);
	if (result)
		return result;

	inv_clear_kfifo(st);
	/* disable the sensor output to FIFO */
	result = inv_i2c_single_write(st, reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	result = inv_i2c_read(st, reg->user_ctrl, 1, &user_ctrl);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	user_ctrl &= ~BIT_FIFO_EN;
	st->fifo_counter = 0;
	/* reset fifo */
	val = (BIT_3050_FIFO_RST | user_ctrl);
	result = inv_i2c_single_write(st, reg->user_ctrl, val);
	if (result)
		goto reset_fifo_fail;
	mdelay(POWER_UP_TIME);
	st->last_isr_time = get_time_ns();
	if (st->chip_config.dmp_on) {
		/* enable interrupt when DMP is done */
		result = inv_i2c_single_write(st, reg->int_enable,
					BIT_DMP_INT_EN);
		if (result)
			return result;

		result = inv_i2c_single_write(st, reg->user_ctrl,
			BIT_FIFO_EN|user_ctrl);
		if (result)
			return result;
	} else {
		/* enable interrupt */
		if (st->chip_config.accl_fifo_enable ||
			st->chip_config.gyro_fifo_enable){
			result = inv_i2c_single_write(st, reg->int_enable,
							BIT_DATA_RDY_EN);
			if (result)
				return result;
		}
		/* enable FIFO reading and I2C master interface*/
		result = inv_i2c_single_write(st, reg->user_ctrl,
			BIT_FIFO_EN | user_ctrl);
		if (result)
			return result;
		/* enable sensor output to FIFO and FIFO footer*/
		val = 1;
		if (st->chip_config.accl_fifo_enable)
			val |= BITS_3050_ACCL_OUT;
		if (st->chip_config.gyro_fifo_enable)
			val |= BITS_GYRO_OUT;
		result = inv_i2c_single_write(st, reg->fifo_en, val);
		if (result)
			return result;
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
 *  set_power_mpu3050() - set power of mpu3050.
 *  @st:	Device driver instance.
 *  @power_on:  on/off
 */
int set_power_mpu3050(struct inv_gyro_state_s *st, unsigned char power_on)
{
	struct inv_reg_map_s *reg;
	unsigned char data, p;
	int result;
	reg = st->reg;
	if (power_on) {
		data = 0;
	} else {
		if (st->mpu_slave) {
			result = st->mpu_slave->suspend(st);
			if (result)
				return result;
		}

		data = BIT_SLEEP;
	}
	if (st->chip_config.gyro_enable) {
		p = (BITS_3050_POWER1 | INV_CLK_PLL);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data | p);
		if (result)
			return result;

		p = (BITS_3050_POWER2 | INV_CLK_PLL);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data | p);
		if (result)
			return result;

		p = INV_CLK_PLL;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data | p);
		if (result)
			return result;

		st->chip_config.clk_src = INV_CLK_PLL;
	} else {
		data |= (BITS_3050_GYRO_STANDBY | INV_CLK_INTERNAL);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_INTERNAL;
	}
	if (power_on) {
		mdelay(POWER_UP_TIME);
		if (st->mpu_slave) {
			result = st->mpu_slave->resume(st);
			if (result)
				return result;
		}
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
	return set_power_mpu3050(st, power_on);
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

	reg = st->reg;
	if (enable) {
		result = mpu3050_fifo_reset(st);
		if (result)
			return result;

		st->chip_config.enable = 1;
	} else {
		result = inv_i2c_single_write(st, reg->fifo_en, 0);
		if (result)
			return result;

		result = inv_i2c_single_write(st, reg->int_enable, 0);
		if (result)
			return result;

		st->chip_config.enable = 0;
	}

	return 0;
}

static int mpu3050_global_delay(struct inv_gyro_state_s *inf)
{
	unsigned long delay_us;
	unsigned char val;
	int rate;
	int err = 0;

	/* find the fastest polling of all the devices */
	delay_us = -1;
	if (inf->chip_config.gyro_enable && inf->chip_config.gyro_delay_us) {
		if (inf->chip_config.gyro_delay_us < delay_us)
			delay_us = inf->chip_config.gyro_delay_us;
	}
	if (inf->chip_config.accl_enable && inf->chip_config.accl_delay_us) {
		if (inf->chip_config.accl_delay_us < delay_us)
			delay_us = inf->chip_config.accl_delay_us;
	}
	if (delay_us == -1)
		delay_us = NVI_DELAY_DEFAULT; /* default if nothing found */
	if (delay_us < MIN_FIFO_RATE)
		delay_us = MIN_FIFO_RATE;
	if (delay_us > MAX_FIFO_RATE)
		delay_us = MAX_FIFO_RATE;
	/* program if new value */
	if (delay_us != inf->sample_delay_us) {
		dev_dbg(&inf->i2c->dev, "%s %lu\n", __func__, delay_us);
		inf->sample_delay_us = delay_us;
		val = delay_us / ONE_K_HZ - 1;
		err = inv_i2c_single_write(inf, inf->reg->sample_rate_div,
					   val);
		rate = 1000000 / delay_us;
		inf->irq_dur_us = delay_us;
		delay_us <<= 1;
		for (val = 1; val < ARRAY_SIZE(nvi_lpf_us_tbl); val++) {
			if (delay_us < nvi_lpf_us_tbl[val])
				break;
		}
		if (inf->mpu_slave != NULL)
			err |= inf->mpu_slave->set_lpf(inf, rate);
		err |= inv_i2c_single_write(inf, inf->reg->lpf, val |
					    (inf->chip_config.gyro_fsr << 3));
		inf->last_isr_time = get_time_ns();
	}
	return err;
}

static int mpu3050_gyro_enable(struct inv_gyro_state_s *inf,
			       unsigned char enable, unsigned char fifo_enable)
{
	unsigned char enable_old;
	unsigned char fifo_enable_old;
	unsigned char val;
	int err = 0;

	enable_old = inf->chip_config.gyro_enable;
	fifo_enable_old = inf->chip_config.gyro_fifo_enable;
	inf->chip_config.gyro_fifo_enable = fifo_enable;
	inf->chip_config.gyro_enable = enable;
	set_power_mpu3050(inf, 1);
	if (enable != enable_old) {
		if (enable) {
			val = (inf->chip_config.gyro_fsr << 3);
			val |= inf->chip_config.lpf;
			err = inv_i2c_single_write(inf, inf->reg->lpf, val);
		}
		mpu3050_global_delay(inf);
	}
	if (fifo_enable_old != fifo_enable)
		mpu3050_fifo_reset(inf);
	if (err) {
		inf->chip_config.gyro_enable = enable_old;
		inf->chip_config.gyro_fifo_enable = fifo_enable_old;
	}
	set_power_mpu3050(inf, 1);
	return err;
}

/**
 *  inv_raw_accl_show() - Read accel data directly from registers.
 */
static ssize_t mpu3050_raw_accl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned char data[6];
	short out[3];
	int result;
	struct inv_gyro_state_s *st;

	st = dev_get_drvdata(dev);
	if (st->mpu_slave != NULL) {
		if (0 == st->mpu_slave->get_mode(st))
			return -EINVAL;
	}
	result = inv_i2c_read(st, REG_3050_AUX_XOUT_H, 6, data);
	out[0] = out[1] = out[2];
	if ((0 == result) && (st->mpu_slave != NULL))
		st->mpu_slave->combine_data(data, out);
	return sprintf(buf, "%d %d %d %lld\n", out[0],
		out[1], out[2], get_time_ns());
}

/**
 *  inv_accl_fifo_enable_store() - Enable/disable accl fifo output.
 */
static ssize_t mpu3050_accl_fifo_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long en;
	struct inv_gyro_state_s *st;
	st = dev_get_drvdata(dev);
	if (kstrtoul(buf, 10, &en))
		return -EINVAL;
	if (en == !(!(st->chip_config.accl_fifo_enable)))
		return count;
	st->chip_config.accl_fifo_enable = en;
	if (en && (0 == st->chip_config.accl_enable)) {
		st->chip_config.accl_enable = en;
		set_power_mpu3050(st, 1);
	}
	return count;
}
/**
 *  mpu3050_accl_enable_store() - Enable/disable accl.
 */
static ssize_t mpu3050_accl_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long en;
	struct inv_gyro_state_s *st;
	st = dev_get_drvdata(dev);
	if (kstrtoul(buf, 10, &en))
		return -EINVAL;
	if (en == !(!(st->chip_config.accl_enable)))
		return count;
	st->chip_config.accl_enable = en;
	if ((0 == en) && st->chip_config.accl_fifo_enable)
		st->chip_config.accl_fifo_enable = 0;
	if (st->mpu_slave != NULL) {
		if (en)
			set_power_mpu3050(st, 1);
		else
			set_power_mpu3050(st, 0);
	}
	return count;
}

static ssize_t mpu3050_accl_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned long accl_delay_us;
	unsigned long accl_delay_us_old;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &accl_delay_us);
	if (err)
		return err;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %lu\n", __func__, accl_delay_us);
	if (accl_delay_us < NVI_INPUT_ACCL_DELAY_US_MIN)
		accl_delay_us = NVI_INPUT_ACCL_DELAY_US_MIN;
	if (accl_delay_us != inf->chip_config.accl_delay_us) {
		accl_delay_us_old = inf->chip_config.accl_delay_us;
		inf->chip_config.accl_delay_us = accl_delay_us;
		if (inf->chip_config.accl_enable) {
			err = mpu3050_global_delay(inf);
			if (!err)
				inf->chip_config.accl_delay_us =
							     accl_delay_us_old;
		}
	}
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %lu ERR=%d\n",
			__func__, accl_delay_us, err);
		return err;
	}

	return count;
}

static ssize_t mpu3050_accl_max_range_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char fsr;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &fsr);
	if (err)
		return -EINVAL;

	if (fsr > 3)
		return -EINVAL;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %x\n", __func__, fsr);
	if (fsr != inf->chip_config.accl_fsr) {
		inf->chip_config.accl_fsr = fsr;
		if (inf->chip_config.accl_enable) {
			if ((inf->chip_type == INV_MPU3050) &&
			    (inf->mpu_slave != NULL)) {
				err = inf->mpu_slave->set_fs(inf, fsr);
				/* reset fifo to purge old data */
				mpu3050_fifo_reset(inf);
			}
		}
	}
	mutex_unlock(&inf->mutex);
	if (err < 0) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n", __func__, fsr, err);
		return err;
	}

	return count;
}

static ssize_t mpu3050_gyro_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char enable;
	unsigned char fifo_enable;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &enable);
	if (err)
		return -EINVAL;

	if (enable > 7)
		return -EINVAL;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %x\n", __func__, enable);
	if (enable != inf->chip_config.gyro_enable) {
		if (enable)
			fifo_enable = inf->chip_config.gyro_fifo_enable;
		else
			fifo_enable = 0;
		err = mpu3050_gyro_enable(inf, enable, fifo_enable);
	}
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n",
			__func__, enable, err);
		return err;
	}

	return count;
}

static ssize_t mpu3050_gyro_fifo_enable_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char fifo_enable;
	unsigned char enable;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &fifo_enable);
	if (err)
		return -EINVAL;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %x\n", __func__, fifo_enable);
	enable = inf->chip_config.gyro_enable;
	if (fifo_enable) {
		fifo_enable = 1;
		if (!enable)
			enable = 7;
	}
	if (fifo_enable != inf->chip_config.gyro_fifo_enable)
		err = mpu3050_gyro_enable(inf, enable, fifo_enable);
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n",
			__func__, fifo_enable, err);
		return err;
	}

	return count;
}

static ssize_t mpu3050_gyro_max_range_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char fsr;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &fsr);
	if (err)
		return -EINVAL;

	if (fsr > 3)
		return -EINVAL;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %x\n", __func__, fsr);
	if (fsr != inf->chip_config.gyro_fsr) {
		inf->chip_config.gyro_fsr = fsr;
		if (inf->chip_config.gyro_enable) {
			fsr = (fsr << 3) | inf->chip_config.lpf;
			err = inv_i2c_single_write(inf, inf->reg->lpf, fsr);
			mpu3050_fifo_reset(inf);
		}
	}
	mutex_unlock(&inf->mutex);
	if (err < 0) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n", __func__, fsr, err);
		return err;
	}

	return count;
}

static ssize_t mpu3050_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char enable;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &enable);
	if (err)
		return -EINVAL;

	if (enable)
		enable = 1;
	inf->chip_config.enable = enable;
	return count;
}

static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR,
		   nvi_gyro_enable_show, mpu3050_gyro_enable_store);
static DEVICE_ATTR(gyro_fifo_enable, S_IRUGO | S_IWUSR,
		   nvi_gyro_fifo_enable_show, mpu3050_gyro_fifo_enable_store);
static DEVICE_ATTR(gyro_max_range, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_gyro_max_range_show, mpu3050_gyro_max_range_store);
static DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR,
		   nvi_accl_enable_show, mpu3050_accl_enable_store);
static DEVICE_ATTR(accl_fifo_enable, S_IRUGO | S_IWUSR,
		   nvi_accl_fifo_enable_show, mpu3050_accl_fifo_enable_store);
static DEVICE_ATTR(accl_delay, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_accl_delay_show, mpu3050_accl_delay_store);
static DEVICE_ATTR(accl_max_range, S_IRUGO | S_IWUSR,
		   nvi_accl_max_range_show, mpu3050_accl_max_range_store);
static DEVICE_ATTR(accl_orientation, S_IRUGO,
		   inv_accl_matrix_show, NULL);
static DEVICE_ATTR(raw_accl, S_IRUGO,
		   mpu3050_raw_accl_show, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_enable_show, mpu3050_enable_store);

static struct device_attribute *inv_mpu3050_attributes[] = {
	&dev_attr_gyro_enable,
	&dev_attr_gyro_fifo_enable,
	&dev_attr_gyro_max_range,
	&dev_attr_accl_enable,
	&dev_attr_accl_fifo_enable,
	&dev_attr_accl_delay,
	&dev_attr_accl_max_range,
	&dev_attr_accl_orientation,
	&dev_attr_raw_accl,
	&dev_attr_enable,
	NULL
};

int inv_mpu3050_create_sysfs(struct inv_gyro_state_s *st)
{
	int result;
	result = create_device_attributes(st->inv_dev, inv_mpu3050_attributes);
	if (result)
		return result;
	return result;
}
int inv_mpu3050_remove_sysfs(struct inv_gyro_state_s *st)
{
	remove_device_attributes(st->inv_dev, inv_mpu3050_attributes);
	return 0;
}

static void mpu3050_report(struct inv_gyro_state_s *st, s64 t,
			   int counter, unsigned char *data)
{
	short x = 0, y = 0, z = 0;
	int ind;
	short out[3];
	struct inv_chip_config_s *conf;
	conf = &st->chip_config;
	ind = 0;
	if (counter)
		ind += 2;
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

	if (conf->accl_fifo_enable | conf->dmp_on) {
		if (st->mpu_slave != NULL) {
			st->mpu_slave->combine_data(&data[ind], out);
			x = out[0];
			y = out[1];
			z = out[2];
		}
		if (conf->accl_fifo_enable) {
			/*it is possible that accl disabled when dmp is on*/
			input_report_rel(st->idev, REL_RX,  x);
			input_report_rel(st->idev, REL_RY,  y);
			input_report_rel(st->idev, REL_RZ,  z);
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
	/* always report time */
	{
		input_report_rel(st->idev, REL_MISC, (unsigned int)(t >> 32));
		input_report_rel(st->idev, REL_WHEEL,
			(unsigned int)(t & 0xffffffff));
		input_sync(st->idev);
	}
}

/**
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
irqreturn_t inv_read_fifo_mpu3050(int irq, void *dev_id)
{
	struct inv_gyro_state_s *st;
	unsigned char bytes_per_datum;
	const unsigned short fifo_thresh = 500;
	int result;
	unsigned char data[16];
	short fifo_count, byte_read;
	unsigned int copied;
	s64 timestamp;
	struct inv_reg_map_s *reg;

	st = (struct inv_gyro_state_s *)dev_id;
	reg = st->reg;

	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.dmp_on))
		goto end_session;

	result = inv_i2c_read(st, reg->temperature, 2, data);
	if (!result) {
		mutex_lock(&st->mutex_temp);
		st->temp_val = (data[0] << 8) | data[1];
		st->temp_ts = timestamp;
		mutex_unlock(&st->mutex_temp);
	}

	if (st->chip_config.dmp_on)
		bytes_per_datum = BYTES_FOR_DMP;
	else
		bytes_per_datum = (st->chip_config.accl_fifo_enable +
					st->chip_config.gyro_fifo_enable)*
					BYTES_PER_SENSOR;
	if (st->fifo_counter == 0)
		byte_read = bytes_per_datum;
	else
		byte_read = bytes_per_datum + 2;

	fifo_count = 0;
	if (byte_read != 0) {
		result = inv_i2c_read(st, reg->fifo_count_h, 2, data);
		if (result)
			goto end_session;
		fifo_count = (data[0] << 8) + data[1];
		if (fifo_count < byte_read)
			goto end_session;
		if (fifo_count%2)
			goto flush_fifo;
		if (fifo_count > fifo_thresh)
			goto flush_fifo;
		/* Timestamp mismatch. */
		if (kfifo_len(&st->trigger.timestamps) <
			fifo_count / byte_read)
			goto flush_fifo;
		if (kfifo_len(&st->trigger.timestamps) >
			fifo_count / byte_read + TIME_STAMP_TOR) {
			if (st->chip_config.dmp_on) {
				result = kfifo_to_user(&st->trigger.timestamps,
				&timestamp, sizeof(timestamp), &copied);
				if (result)
					goto flush_fifo;
			} else
				goto flush_fifo;
		}
	}

	while ((bytes_per_datum != 0) && (fifo_count >= byte_read)) {
		result = inv_i2c_read(st, reg->fifo_r_w, byte_read, data);
		if (result)
			goto flush_fifo;

		result = kfifo_to_user(&st->trigger.timestamps,
			&timestamp, sizeof(timestamp), &copied);
		if (result)
			goto flush_fifo;
		mpu3050_report(st, timestamp, st->fifo_counter, data);
		fifo_count -= byte_read;
		if (st->fifo_counter == 0) {
			st->fifo_counter = 1;
			byte_read = bytes_per_datum + 2;
		}
	}
end_session:
	return IRQ_HANDLED;
flush_fifo:
	/* Flush HW and SW FIFOs. */
	mpu3050_fifo_reset(st);
	return IRQ_HANDLED;
}
void inv_setup_reg_mpu3050(struct inv_reg_map_s *reg)
{
	reg->fifo_en         = 0x12;
	reg->sample_rate_div = 0x15;
	reg->lpf             = 0x16;
	reg->fifo_count_h    = 0x3a;
	reg->fifo_r_w        = 0x3c;
	reg->user_ctrl       = 0x3d;
	reg->pwr_mgmt_1      = 0x3e;
	reg->raw_gyro        = 0x1d;
	reg->raw_accl        = 0x23;
	reg->temperature     = 0x1b;
	reg->int_enable      = 0x17;
	reg->int_status      = 0x1a;
}

/**
 *  inv_init_config_mpu3050() - Initialize hardware, disable FIFO.
 *  @st:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
int inv_init_config_mpu3050(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char data;

	/*reading AUX VDDIO register */
	result = inv_i2c_read(st, REG_3050_AUX_VDDIO, 1, &data);
	if (result)
		return result;
	data &= ~BIT_3050_VDDIO;
	data |= (st->plat_data.level_shifter << 2);
	result = inv_i2c_single_write(st, REG_3050_AUX_VDDIO, data);
	if (result)
		return result;

	reg = st->reg;
	result = set_inv_enable(st, 0);
	if (result)
		return result;
	/*2000dps full scale range*/
	result = inv_i2c_single_write(st, reg->lpf, (INV_FSR_2000DPS << 3)
					| INV_FILTER_42HZ);
	if (result)
		return result;
	st->chip_config.gyro_fsr = INV_FSR_2000DPS;
	st->chip_config.lpf = INV_FILTER_42HZ;
	result = inv_i2c_single_write(st, reg->sample_rate_div, 19);
	if (result)
		return result;
	st->chip_config.fifo_rate = 50;
	st->irq_dur_us            = 20*1000;
	st->chip_config.enable = 0;
	st->chip_config.dmp_on = 0;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.gyro_enable = 1;
	st->chip_config.gyro_fifo_enable = 1;
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		if (st->mpu_slave != NULL) {
			result = st->mpu_slave->setup(st);
			if (result)
				return result;

			result = st->mpu_slave->set_fs(st, 0);
			if (result)
				return result;

			result = st->mpu_slave->set_lpf(st, 50);
			if (result)
				return result;
		}

		st->chip_config.accl_enable = 1;
		st->chip_config.accl_fifo_enable = 1;
	} else {
		st->chip_config.accl_enable = 0;
		st->chip_config.accl_fifo_enable = 0;
	}
	return 0;
}

int set_3050_bypass(struct inv_gyro_state_s *st, int enable)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char b;

	reg = st->reg;
	result = inv_i2c_read(st, reg->user_ctrl, 1, &b);
	if (result)
		return result;
	if (((b & BIT_3050_AUX_IF_EN) == 0) && enable)
		return 0;
	if ((b & BIT_3050_AUX_IF_EN) && (enable == 0))
		return 0;
	b &= ~BIT_3050_AUX_IF_EN;
	if (!enable) {
		b |= BIT_3050_AUX_IF_EN;
		result = inv_i2c_single_write(st, reg->user_ctrl, b);
		return result;
	} else {
		/* Coming out of I2C is tricky due to several erratta.  Do not
		* modify this algorithm
		*/
		/*
		* 1) wait for the right time and send the command to change
		* the aux i2c slave address to an invalid address that will
		* get nack'ed
		*
		* 0x00 is broadcast.  0x7F is unlikely to be used by any aux.
		*/
		result = inv_i2c_single_write(st, REG_3050_SLAVE_ADDR,
						0x7F);
		if (result)
			return result;
		/*
		* 2) wait enough time for a nack to occur, then go into
		*    bypass mode:
		*/
		mdelay(2);
		result = inv_i2c_single_write(st, reg->user_ctrl, b);
		if (result)
			return result;
		/*
		* 3) wait for up to one MPU cycle then restore the slave
		*    address
		*/
		mdelay(20);
		result = inv_i2c_single_write(st, REG_3050_SLAVE_REG,
				     st->plat_data.secondary_read_reg);
		if (result)
			return result;

		result = inv_i2c_single_write(st, REG_3050_SLAVE_ADDR,
			st->plat_data.secondary_i2c_addr);
		if (result)
			return result;

		result = inv_i2c_single_write(st, reg->user_ctrl,
						(b | BIT_3050_AUX_IF_RST));
		if (result)
			return result;

		mdelay(2);
	}
	return 0;
}
/**
 *  @}
 */

