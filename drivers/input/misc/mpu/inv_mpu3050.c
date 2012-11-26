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
 *  inv_raw_accl_show() - Read accel data directly from registers.
 */
static ssize_t inv_raw_accl_mpu3050_show(struct device *dev,
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
static ssize_t inv_accl_fifo_mpu3050_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long en;
	struct inv_gyro_state_s *st;
	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EINVAL;
	if (kstrtoul(buf, 10, &en))
		return -EINVAL;
	if (en == !(!(st->chip_config.accl_fifo_enable)))
		return count;
	st->chip_config.accl_fifo_enable = en;
	if (en && (0 == st->chip_config.accl_enable)) {
		st->chip_config.accl_enable = en;
		if (st->mpu_slave != NULL)
			st->mpu_slave->resume(st);
	}
	return count;
}
/**
 *  inv_accl_mpu3050_enable_store() - Enable/disable accl.
 */
static ssize_t inv_accl_mpu3050_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long en;
	struct inv_gyro_state_s *st;
	st = dev_get_drvdata(dev);
	if (st->chip_config.is_asleep)
		return -EINVAL;
	if (kstrtoul(buf, 10, &en))
		return -EINVAL;
	if (en == !(!(st->chip_config.accl_enable)))
		return count;
	st->chip_config.accl_enable = en;
	if ((0 == en) && st->chip_config.accl_fifo_enable)
		st->chip_config.accl_fifo_enable = 0;
	if (st->mpu_slave != NULL) {
		if (en)
			st->mpu_slave->resume(st);
		else
			st->mpu_slave->suspend(st);
	}
	return count;
}

static DEVICE_ATTR(gyro_fifo_enable, S_IRUGO | S_IWUSR,
	inv_gyro_fifo_enable_show,
	inv_gyro_fifo_enable_store);
static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR, inv_gyro_enable_show,
	inv_gyro_enable_store);
static DEVICE_ATTR(raw_accl, S_IRUGO, inv_raw_accl_mpu3050_show, NULL);
static DEVICE_ATTR(accl_fifo_enable, S_IRUGO | S_IWUSR,
	inv_accl_fifo_enable_show,
	inv_accl_fifo_mpu3050_enable_store);
static DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR, inv_accl_enable_show,
	inv_accl_mpu3050_enable_store);
static DEVICE_ATTR(accl_matrix, S_IRUGO, inv_accl_matrix_show, NULL);
static DEVICE_ATTR(accl_fs, S_IRUGO | S_IWUSR, inv_accl_fs_show,
			inv_accl_fs_store);

static struct device_attribute *inv_mpu3050_attributes[] = {
	&dev_attr_gyro_fifo_enable,
	&dev_attr_gyro_enable,
	&dev_attr_accl_fifo_enable,
	&dev_attr_accl_enable,
	&dev_attr_raw_accl,
	&dev_attr_accl_matrix,
	&dev_attr_accl_fs,
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

static void inv_report_data_3050(struct inv_gyro_state_s *st, s64 t,
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

	if (st->chip_config.is_asleep)
		goto end_session;
	if (!(st->chip_config.enable))
		goto end_session;
	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.dmp_on |
		st->chip_config.compass_enable))
		goto end_session;
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
		inv_report_data_3050(st, timestamp, st->fifo_counter, data);
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
	inv_reset_fifo(st);
	inv_clear_kfifo(st);
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

	if (st->chip_config.is_asleep)
		return -EPERM;

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
	st->chip_config.fsr = INV_FSR_2000DPS;
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
/**
 *  set_power_mpu3050() - set power of mpu3050.
 *  @st:	Device driver instance.
 *  @power_on:  on/off
 */
int set_power_mpu3050(struct inv_gyro_state_s *st,
	unsigned char power_on)
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
		st->chip_config.is_asleep = 0;
	} else
		st->chip_config.is_asleep = 1;
	return 0;
}
/**
 *  reset_fifo_mpu3050() - Reset FIFO related registers
 *  @st:	Device driver instance.
 */
int reset_fifo_mpu3050(struct inv_gyro_state_s *st)
{
	struct inv_reg_map_s *reg;
	int result;
	unsigned char val, user_ctrl;

	reg = st->reg;
	/* disable interrupt */
	result = inv_i2c_single_write(st, reg->int_enable, 0);
	if (result)
		return result;
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
 *  @}
 */

