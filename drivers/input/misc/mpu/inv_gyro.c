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
#include <linux/regulator/consumer.h>
#include <linux/byteorder/generic.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/mpu.h>

#include "inv_gyro.h"


/* regulator names in order of powering on */
static char *nvi_vregs[] = {
	"vdd",
	"vlogic",
};

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

static unsigned long nvi_lpf_us_tbl[] = {
	3906,	/* 256Hz */
	5319,	/* 188Hz */
	10204,	/* 98Hz */
	23810,	/* 42Hz */
	50000,	/* 20Hz */
	100000,	/* 10Hz */
	/* 200000, 5Hz */
};

static unsigned long nvi_lpa_delay_us_tbl[] = {
	800000,
	200000,
	50000,
	/* 25000, */
};

static struct inv_gyro_state_s *inf_local;

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

	if (st->shutdown)
		return 0;

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

	res = i2c_transfer(st->sl_handle, msgs, 2);
	pr_debug("%s RD%02X%02X%02X res=%d\n",
		st->hw_s->name, i2c_addr, reg, length, res);
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

	if (st->shutdown)
		return 0;

	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

	res = i2c_transfer(st->sl_handle, &msg, 1);
	pr_debug("%s WS%02X%02X%02X res=%d\n",
		 st->hw_s->name, i2c_addr, reg, data, res);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	}

	return 0;
}


/* Register SMPLRT_DIV (0x19) */
static int nvi_smplrt_div_wr(struct inv_gyro_state_s *inf,
			     unsigned char smplrt_div)
{
	int err = 0;

	if (smplrt_div != inf->hw.smplrt_div) {
		err = inv_i2c_single_write(inf, inf->reg->sample_rate_div,
					   smplrt_div);
		if (!err)
			inf->hw.smplrt_div = smplrt_div;
	}
	return err;
}

/* Register CONFIG (0x1A) */
static int nvi_config_wr(struct inv_gyro_state_s *inf, unsigned char lpf)
{
	int err = 0;

	if (lpf != inf->hw.config) {
		err = inv_i2c_single_write(inf, inf->reg->lpf, lpf);
		if (!err)
			inf->hw.config = lpf;
	}
	return err;
}

/* Register GYRO_CONFIG (0x1B) */
static int nvi_gyro_config_wr(struct inv_gyro_state_s *inf, unsigned char fsr)
{
	unsigned char val;
	int err = 0;

	if (fsr != inf->hw.gyro_config) {
		val = (fsr << 3);
		err = inv_i2c_single_write(inf, inf->reg->gyro_config, val);
		if (!err) {
			inf->hw.gyro_config = val;
			err = 1; /* flag change made */
		}
	}
	return err;
}

/* Register ACCEL_CONFIG (0x1C) */
static int nvi_accel_config_wr(struct inv_gyro_state_s *inf,
			       unsigned char fsr, unsigned char hpf)
{
	unsigned char val;
	int err = 0;

	val = (fsr << 3) | hpf;
	if (val != inf->hw.accl_config) {
		err = inv_i2c_single_write(inf, inf->reg->accl_config,
					   val);
		if (!err) {
			inf->hw.accl_config = val;
			err = 1; /* flag change made */
			if (hpf != 7)
				inf->mot_enable = false;
		}
	}
	return err;
}

/* Register MOT_THR (0x1F) */
static int nvi_mot_thr_wr(struct inv_gyro_state_s *inf, unsigned char mot_thr)
{
	int err = 0;

	if (mot_thr != inf->hw.mot_thr) {
		err = inv_i2c_single_write(inf, REG_MOT_THR, mot_thr);
		if (!err)
			inf->hw.mot_thr = mot_thr;
	}
	return err;
}

/* Register MOT_DUR (0x20) */
static int nvi_mot_dur_wr(struct inv_gyro_state_s *inf, unsigned char mot_dur)
{
	int err = 0;

	if (mot_dur != inf->hw.mot_dur) {
		err = inv_i2c_single_write(inf, REG_MOT_DUR, mot_dur);
		if (!err)
			inf->hw.mot_dur = mot_dur;
	}
	return err;
}

/* Register FIFO_EN (0x23) */
static int nvi_fifo_en_wr(struct inv_gyro_state_s *inf, unsigned char fifo_en)
{
	int err = 0;

	if (fifo_en != inf->hw.fifo_en) {
		err = inv_i2c_single_write(inf, inf->reg->fifo_en, fifo_en);
		if (!err)
			inf->hw.fifo_en = fifo_en;
	}
	return err;
}

/* Register I2C_MST_CTRL (0x24) */
static int nvi_i2c_mst_ctrl_wr(struct inv_gyro_state_s *inf,
			       bool port3_fifo_en)
{
	unsigned char val;
	int err = 0;

	val = inf->aux.clock_i2c;
	val |= BIT_WAIT_FOR_ES | BIT_I2C_MST_P_NSR;
	if (port3_fifo_en)
		val |= BIT_SLV3_FIFO_EN;
	if (val != inf->hw.i2c_mst_ctrl) {
		err = inv_i2c_single_write(inf, REG_I2C_MST_CTRL, val);
		if (!err)
			inf->hw.i2c_mst_ctrl = val;
	}
	return err;
}

/* Register I2C_SLV4_CTRL (0x34) */
static int nvi_i2c_slv4_ctrl_wr(struct inv_gyro_state_s *inf, bool slv4_en)
{
	unsigned char val;
	int err = 0;

	val = inf->aux.delay_hw;
	val |= (inf->aux.port[AUX_PORT_SPECIAL].nmp.ctrl &
							 BITS_I2C_SLV_REG_DIS);
	if (slv4_en)
		val |= BIT_SLV_EN;
	if (val != inf->hw.i2c_slv4_ctrl) {
		err = inv_i2c_single_write(inf, REG_I2C_SLV4_CTRL, val);
		if (!err)
			inf->hw.i2c_slv4_ctrl = val;
	}
	return err;
}

/* Register INT_PIN_CFG (0x37) */
static int nvi_int_pin_cfg_wr(struct inv_gyro_state_s *inf, unsigned char val)
{
	int err = 0;

	if (val != inf->hw.int_pin_cfg) {
		err = inv_i2c_single_write(inf, REG_INT_PIN_CFG, val);
		if (!err)
			inf->hw.int_pin_cfg = val;
	}
	return err;
}

/* Register INT_ENABLE (0x38) */
static int nvi_int_enable_wr(struct inv_gyro_state_s *inf, bool enable)
{
	unsigned char val = 0;
	int err = 0;

	if (enable) {
		if ((inf->hw.user_ctrl & BIT_I2C_MST_EN) ||
						inf->chip_config.gyro_enable) {
			val = BIT_DATA_RDY_EN;
		} else if (inf->chip_config.accl_enable) {
			if (inf->mot_enable && (!inf->mot_cnt)) {
				val = BIT_MOT_EN;
				if (inf->mot_dbg)
					pr_info("%s motion detect on",
						__func__);
			} else {
				val = BIT_DATA_RDY_EN;
			}
		}
	}
	if ((val != inf->hw.int_enable) && (inf->pm > NVI_PM_OFF)) {
		err = inv_i2c_single_write(inf, inf->reg->int_enable, val);
		if (!err)
			inf->hw.int_enable = val;
	}
	return err;
}

/* Register I2C_MST_DELAY_CTRL (0x67) */
static int nvi_i2c_mst_delay_ctrl_wr(struct inv_gyro_state_s *inf,
				     unsigned char i2c_mst_delay_ctrl)
{
	int err = 0;

	if (i2c_mst_delay_ctrl != inf->hw.i2c_mst_delay_ctrl) {
		err = inv_i2c_single_write(inf, REG_I2C_MST_DELAY_CTRL,
					   i2c_mst_delay_ctrl);
		if (!err)
			inf->hw.i2c_mst_delay_ctrl = i2c_mst_delay_ctrl;
	}
	return err;
}

/* Register MOT_DETECT_CTRL (0x69) */
static int nvi_mot_detect_ctrl_wr(struct inv_gyro_state_s *inf,
				  unsigned char val)
{
	int err = 0;

	if (val != inf->hw.mot_detect_ctrl) {
		err = inv_i2c_single_write(inf, REG_MOT_DETECT_CTRL, val);
		if (!err)
			inf->hw.mot_detect_ctrl = val;
	}
	return err;
}

/* Register USER_CTRL (0x6A) */
static int nvi_user_ctrl_reset_wr(struct inv_gyro_state_s *inf,
				  unsigned char val)
{
	int i;
	int err;
	int err_t;

	err_t =  inv_i2c_single_write(inf, inf->reg->user_ctrl, val);
	for (i = 0; i < POWER_UP_TIME; i++) {
		val = -1;
		err = inv_i2c_read(inf, inf->reg->user_ctrl, 1, &val);
		if (!(val & (BIT_FIFO_RST | BIT_I2C_MST_RST)))
			break;

		mdelay(1);
	}
	err_t |= err;
	inf->hw.user_ctrl = val;
	return err_t;
}

/* Register USER_CTRL (0x6A) */
static int nvi_user_ctrl_en_wr(struct inv_gyro_state_s *inf,
			       bool fifo_enable, bool i2c_enable)
{
	unsigned char val;
	bool en;
	int i;
	int err = 0;

	if (inf->lpa_enable)
		fifo_enable = false;
	val = 0;
	if (fifo_enable) {
		for (i = 0; i < (AUX_PORT_SPECIAL - 1); i++) {
			if (inf->aux.port[i].fifo_en && inf->aux.port[i].hw_en)
				val |= (1 << i);
		}
		if (inf->chip_config.gyro_fifo_enable)
			val |= (inf->chip_config.gyro_enable << 4);
		if (inf->chip_config.accl_fifo_enable)
			val |= BIT_ACCEL_OUT;
		if (inf->chip_config.temp_fifo_enable)
			val |= BIT_TEMP_FIFO_EN;
		if (inf->aux.port[3].fifo_en && inf->aux.port[3].hw_en)
			en = true;
		else
			en = false;
		err |= nvi_i2c_mst_ctrl_wr(inf, en);
		if (val || en)
			en = true;
		else
			en = false;
	} else {
		err |= nvi_i2c_mst_ctrl_wr(inf, false);
		en = false;
	}
	err |= nvi_fifo_en_wr(inf, val);
	val = 0;
	if (fifo_enable && en)
		val |= BIT_FIFO_EN;
	if (i2c_enable && inf->aux.enable)
		val |= BIT_I2C_MST_EN;
	if (val != inf->hw.user_ctrl) {
		err |= inv_i2c_single_write(inf, inf->reg->user_ctrl, val);
		if (!err)
			inf->hw.user_ctrl = val;
	}
	return err;
}

/* Register PWR_MGMT_1 (0x6B) */
static int nvi_pwr_mgmt_1_wr(struct inv_gyro_state_s *inf, unsigned char pm1)
{
	unsigned char val;
	int i;
	int err;

	for (i = 0; i < POWER_UP_TIME; i++) {
		inv_i2c_single_write(inf, inf->reg->pwr_mgmt_1, pm1);
		val = -1;
		err = inv_i2c_read(inf, inf->reg->pwr_mgmt_1, 1, &val);
		if (!val)
			break;

		mdelay(1);
	}
	inf->hw.pwr_mgmt_1 = val;
	return err;
}

static bool nvi_lpa_able(struct inv_gyro_state_s *inf)
{
	bool lpa_enable;
	int i;

	if (inf->mot_enable) {
		lpa_enable = true;
	} else if (inf->chip_config.lpa_delay_us) {
		if (inf->chip_config.accl_delay_us <
						 inf->chip_config.lpa_delay_us)
			lpa_enable = false;
		else
			lpa_enable = true;
	} else {
		lpa_enable = false;
	}
	for (i = 0; i < ARRAY_SIZE(nvi_lpa_delay_us_tbl); i++) {
		if (inf->chip_config.accl_delay_us >= nvi_lpa_delay_us_tbl[i])
			break;
	}
	inf->lpa_hw = i;
	return lpa_enable;
}

static int nvi_vreg_dis(struct inv_gyro_state_s *inf, unsigned int i)
{
	int err = 0;

	if (inf->vreg[i].ret && (inf->vreg[i].consumer != NULL)) {
		err = regulator_disable(inf->vreg[i].consumer);
		if (!err) {
			inf->vreg[i].ret = 0;
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		} else {
			dev_err(&inf->i2c->dev, "%s %s ERR\n",
				__func__, inf->vreg[i].supply);
		}
	}
	return err;
}

static int nvi_vreg_dis_all(struct inv_gyro_state_s *inf)
{
	unsigned int i;
	int err = 0;

	for (i = ARRAY_SIZE(nvi_vregs); i > 0; i--)
		err |= nvi_vreg_dis(inf, (i - 1));
	return err;
}

static int nvi_vreg_en(struct inv_gyro_state_s *inf, unsigned int i)
{
	int err = 0;

	if ((!inf->vreg[i].ret) && (inf->vreg[i].consumer != NULL)) {
		err = regulator_enable(inf->vreg[i].consumer);
		if (!err) {
			inf->vreg[i].ret = 1;
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
			err = 1; /* flag regulator state change */
		} else {
			dev_err(&inf->i2c->dev, "%s %s ERR\n",
				__func__, inf->vreg[i].supply);
		}
	}
	return err;
}

static int nvi_vreg_en_all(struct inv_gyro_state_s *inf)
{
	unsigned i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(nvi_vregs); i++)
		err |= nvi_vreg_en(inf, i);
	return err;
}

static void nvi_vreg_exit(struct inv_gyro_state_s *inf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nvi_vregs); i++) {
		if (inf->vreg[i].consumer != NULL) {
			devm_regulator_put(inf->vreg[i].consumer);
			inf->vreg[i].consumer = NULL;
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		}
	}
}

static int nvi_vreg_init(struct inv_gyro_state_s *inf)
{
	unsigned int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(nvi_vregs); i++) {
		inf->vreg[i].supply = nvi_vregs[i];
		inf->vreg[i].ret = 0;
		inf->vreg[i].consumer = devm_regulator_get(&inf->i2c->dev,
							  inf->vreg[i].supply);
		if (IS_ERR(inf->vreg[i].consumer)) {
			err = PTR_ERR(inf->vreg[i].consumer);
			dev_err(&inf->i2c->dev, "%s ERR %d for %s\n",
				__func__, err, inf->vreg[i].supply);
			inf->vreg[i].consumer = NULL;
		} else {
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		}
	}
	return err;
}

static int nvi_pm_wr_on(struct inv_gyro_state_s *inf, u8 pm1, u8 pm2)
{
	unsigned char val;
	int i;
	int err;
	int err_t = 0;

	err = nvi_vreg_en_all(inf);
	if (err) {
		err_t |= nvi_pwr_mgmt_1_wr(inf, 0);
		err_t |= inv_i2c_single_write(inf, inf->reg->pwr_mgmt_1,
					      BIT_RESET);
		for (i = 0; i < POWER_UP_TIME; i++) {
			val = -1;
			err = inv_i2c_read(inf, inf->reg->pwr_mgmt_1, 1, &val);
			if (!(val & BIT_RESET))
				break;

			mdelay(1);
		}
		err_t |= err;
		err_t |= nvi_pwr_mgmt_1_wr(inf, 0);
		err_t |= nvi_user_ctrl_reset_wr(inf, (BIT_FIFO_RST |
						      BIT_I2C_MST_RST));
		for (i = 0; i < AUX_PORT_MAX; i++) {
			inf->aux.port[i].hw_valid = false;
			inf->aux.port[i].hw_en = false;
		}
		memset(&inf->hw, 0, sizeof(struct nvi_hw));
		inf->sample_delay_us = 0;
		inf->mot_enable = false;
	} else {
		err_t |= nvi_pwr_mgmt_1_wr(inf, 0);
	}
	if (pm2 != inf->hw.pwr_mgmt_2) {
		err = inv_i2c_single_write(inf, inf->reg->pwr_mgmt_2, pm2);
		if (err)
			err_t |= err;
		else
			inf->hw.pwr_mgmt_2 = pm2;
	}
	if (pm1 != inf->hw.pwr_mgmt_1) {
		err = inv_i2c_single_write(inf, inf->reg->pwr_mgmt_1, pm1);
		if (err)
			err_t |= err;
		else
			inf->hw.pwr_mgmt_1 = pm1;
	}
	return err_t;
}

static int nvi_pm_wr(struct inv_gyro_state_s *inf, int pm, int stby)
{
	int err = 0;

	if ((pm == inf->pm) && (stby == inf->stby))
		return err;

	switch (pm) {
	case NVI_PM_OFF_FORCE:
	case NVI_PM_OFF:
		err = nvi_pm_wr_on(inf, BIT_SLEEP, stby);
		err |= nvi_vreg_dis_all(inf);
		break;

	case NVI_PM_STDBY:
		err = nvi_pm_wr_on(inf, BIT_SLEEP, stby);
		break;

	case NVI_PM_ON_CYCLE:
		err = nvi_pm_wr_on(inf, BIT_CYCLE, stby);
		break;

	case NVI_PM_ON:
		err = nvi_pm_wr_on(inf, INV_CLK_INTERNAL, stby);
		break;

	case NVI_PM_ON_FULL:
		err = nvi_pm_wr_on(inf, INV_CLK_PLL, stby);
		break;

	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(&inf->i2c->dev, "%s requested pm=%d ERR=%d\n",
			__func__, pm, err);
		pm = NVI_PM_ERR;
	} else {
		inf->stby = stby;
	}
	inf->pm = pm;
	dev_dbg(&inf->i2c->dev, "%s pm=%d stby=%x\n", __func__, pm, stby);
	if (err > 0)
		err = 0;
	return err;
}

static int nvi_pm(struct inv_gyro_state_s *inf, int pm_req)
{
	bool irq;
	int stby;
	int pm;
	int err;

	nvi_int_enable_wr(inf, false);
	inf->lpa_enable = false;
	if ((pm_req == NVI_PM_OFF_FORCE) || (pm_req == NVI_PM_OFF)) {
		stby = 0x3F;
		pm = NVI_PM_OFF_FORCE;
	} else {
		stby = ((~inf->chip_config.accl_enable) & 0x07) << 3;
		stby |= (~inf->chip_config.gyro_enable) & 0x07;
		if (inf->chip_config.gyro_enable ||
					(inf->hw.user_ctrl & BIT_I2C_MST_EN)) {
			if (inf->chip_config.gyro_enable)
				pm = NVI_PM_ON_FULL;
			else
				pm = NVI_PM_ON;
		} else if (inf->chip_config.accl_enable) {
			if (nvi_lpa_able(inf)) {
				inf->lpa_enable = true;
				stby |= (inf->lpa_hw << 6);
				pm = NVI_PM_ON_CYCLE;
			} else {
				pm = NVI_PM_ON;
			}
		} else if (inf->chip_config.enable || inf->aux.bypass_lock) {
			pm = NVI_PM_STDBY;
		} else {
			pm = NVI_PM_OFF;
		}
	}
	if (pm_req > pm)
		pm = pm_req;
	err = nvi_pm_wr(inf, pm, stby);
	if (pm_req == NVI_PM_AUTO) {
		nvi_user_ctrl_en_wr(inf, true, true);
		irq = true;
	} else {
		irq = false;
	}
	nvi_int_enable_wr(inf, irq);
	return err;
}

static void nvi_pm_exit(struct inv_gyro_state_s *inf)
{
	nvi_pm(inf, NVI_PM_OFF_FORCE);
	nvi_vreg_exit(inf);
}

static int nvi_pm_init(struct inv_gyro_state_s *inf)
{
	int err = 0;

	nvi_vreg_init(inf);
	inf->pm = NVI_PM_ERR;
	inf->stby = 0;
	err = nvi_pm(inf, NVI_PM_ON_FULL);
	return err;
}

static int nvi_motion_detect_enable(struct inv_gyro_state_s *inf, u8 mot_thr)
{
	int err;
	int err_t = 0;

	if (mot_thr) {
		err = nvi_accel_config_wr(inf, inf->chip_config.accl_fsr, 0);
		if (err < 0)
			err_t |= err;
		err_t |= nvi_config_wr(inf, 0);
		if (!inf->hw.mot_dur)
			err_t |= nvi_mot_dur_wr(inf, 1);
		err_t |= nvi_mot_detect_ctrl_wr(inf, inf->chip_config.mot_ctrl);
		err_t |= nvi_mot_thr_wr(inf, mot_thr);
		mdelay(5);
		err = nvi_accel_config_wr(inf, inf->chip_config.accl_fsr, 7);
		if (err < 0)
			err_t |= err;
		if (!err_t)
			inf->mot_enable = true;
	} else {
		err = nvi_accel_config_wr(inf, inf->chip_config.accl_fsr, 0);
		if (err < 0)
			err_t |= err;
	}
	return err_t;
}

static int nvi_aux_delay(struct inv_gyro_state_s *inf,
			 int port, unsigned int delay_ms)
{
	unsigned char val;
	unsigned char i;
	unsigned int ms;
	unsigned int delay_new;
	int delay_rtn;
	int err;

	if (port != AUX_PORT_BYPASS)
		inf->aux.port[port].nmp.delay_ms = delay_ms;
	/* determine valid delays by ports enabled */
	delay_new = 0;
	delay_rtn = 0;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (delay_rtn < inf->aux.port[i].nmp.delay_ms)
			delay_rtn = inf->aux.port[i].nmp.delay_ms;
		if (inf->aux.port[i].hw_en) {
			if (delay_new < inf->aux.port[i].nmp.delay_ms)
				delay_new = inf->aux.port[i].nmp.delay_ms;
		}
	}
	if (!(inf->hw.user_ctrl & BIT_I2C_MST_EN)) {
		/* delay will execute when re-enabled */
		if (delay_ms)
			return delay_rtn;
		else
			return 0;
	}

	/* HW global delay */
	for (i = 1; i < (BITS_I2C_MST_DLY - 1); i++) {
		ms = (inf->sample_delay_us - (inf->sample_delay_us /
					      (i + 1))) * 1000;
		if (ms >= delay_new)
			break;
	}
	inf->aux.delay_hw = i;
	err = nvi_i2c_slv4_ctrl_wr(inf, inf->aux.port[AUX_PORT_SPECIAL].hw_en);
	/* HW port delay enable */
	val = BIT_DELAY_ES_SHADOW;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (inf->aux.port[i].nmp.delay_ms)
			val |= (1 << i);
	}
	nvi_i2c_mst_delay_ctrl_wr(inf, val);
	if (delay_ms)
		return delay_rtn;
	else
		return 0;
}

static int nvi_global_delay(struct inv_gyro_state_s *inf)
{
	unsigned long delay_us;
	unsigned long delay_min;
	unsigned char val;
	int i;
	int err = 0;

	/* find the fastest polling of all the devices */
	delay_us = -1;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (inf->aux.port[i].enable && inf->aux.port[i].nmp.delay_us) {
			if (inf->aux.port[i].nmp.delay_us < delay_us)
				delay_us = inf->aux.port[i].nmp.delay_us;
		}
	}
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
	/* find what the fastest the external devices will let MPU go */
	delay_min = 0;
	for (i = 0; i < AUX_PORT_MAX; i++) {
		if (inf->aux.port[i].enable) {
			if (inf->aux.port[i].nmp.delay_ms > delay_min)
				delay_min = inf->aux.port[i].nmp.delay_ms;
		}
	}
	if (!delay_min)
		/* default if nothing found */
		delay_min = inf->chip_config.min_delay_us;
	else
		delay_min *= 1000L; /* ms => us */
	/* set the limits */
	if (delay_us < delay_min)
		delay_us = delay_min;
	if (delay_us > MAX_FIFO_RATE)
		delay_us = MAX_FIFO_RATE;
	/* program if new value */
	if (delay_us != inf->sample_delay_us) {
		dev_dbg(&inf->i2c->dev, "%s %lu\n", __func__, delay_us);
		inf->sample_delay_us = delay_us;
		inf->irq_dur_us = delay_us;
		delay_us <<= 1;
		for (val = 1; val < ARRAY_SIZE(nvi_lpf_us_tbl); val++) {
			if (delay_us < nvi_lpf_us_tbl[val])
				break;
		}
		err |= nvi_config_wr(inf, val);
		if (val)
			delay_min = 1000;
		else
			delay_min = 8000;
		val = inf->sample_delay_us / delay_min - 1;
		err |= nvi_smplrt_div_wr(inf, val);
		inf->last_isr_time = get_time_ns();
	}
	nvi_aux_delay(inf, AUX_PORT_BYPASS, 0);
	return err;
}

static void nvi_aux_dbg(struct inv_gyro_state_s *inf, char *tag, int val)
{
	struct nvi_mpu_port *n;
	struct aux_port *p;
	struct aux_ports *a;
	unsigned char data[4];
	int i;

	if (!inf->aux.dbg)
		return;

	pr_info("%s %s %d\n", __func__, tag, val);
	for (i = 0; i < AUX_PORT_MAX; i++) {
		inv_i2c_read(inf, (REG_I2C_SLV0_ADDR + (i * 3)), 3, data);
		inv_i2c_read(inf, (REG_I2C_SLV0_DO + i), 1, &data[3]);
		pr_info("PT=%d AD=%x RG=%x CL=%x DO=%x\n",
			i, data[0], data[1], data[2], data[3]);
		n = &inf->aux.port[i].nmp;
		pr_info("PT=%d AD=%x RG=%x CL=%x DO=%x MS=%u US=%lu SB=%x\n",
			i, n->addr, n->reg, n->ctrl, n->data_out, n->delay_ms,
			n->delay_us, n->shutdown_bypass);
		p = &inf->aux.port[i];
		pr_info("PT=%d OF=%u EN=%x FE=%x HE=%x HD=%x HV=%x NS=%lld\n",
			i, p->ext_data_offset, p->enable, p->fifo_en, p->hw_en,
			p->hw_do, p->hw_valid, p->delay_ns);
	}
	a = &inf->aux;
	pr_info("EN=%x GE=%x MD=%x GD=%lu DN=%u BE=%x BL=%d SB=%d\n",
		a->enable, (inf->hw.user_ctrl & BIT_I2C_MST_EN),
		(inf->hw.i2c_slv4_ctrl & BITS_I2C_MST_DLY),
		inf->sample_delay_us, a->ext_data_n,
		(inf->hw.int_pin_cfg & BIT_BYPASS_EN), a->bypass_lock,
		atomic_read(&inf->mutex.count));
}

static void nvi_aux_read(struct inv_gyro_state_s *inf)
{
	struct aux_port *ap;
	long long timestamp1;
	long long timestamp2;
	unsigned int i;
	unsigned int len;
	u8 *p;
	int err;

	if ((inf->aux.ext_data_n == 0) ||
				       (!(inf->hw.user_ctrl & BIT_I2C_MST_EN)))
		return;

	timestamp1 = get_time_ns();
	err = inv_i2c_read(inf, REG_EXT_SENS_DATA_00,
			   inf->aux.ext_data_n,
			   (unsigned char *)&inf->aux.ext_data);
	if (err)
		return;

	timestamp2 = get_time_ns();
	timestamp1 = timestamp1 + ((timestamp2 - timestamp1) / 2);
	for (i = 0; i < AUX_PORT_SPECIAL; i++) {
		ap = &inf->aux.port[i];
		if ((ap->nmp.addr & BIT_I2C_READ) &&
						   (ap->nmp.handler != NULL)) {
			if ((unsigned long)(timestamp2 - ap->delay_ns)
					     >= (ap->nmp.delay_us * 1000)) {
				ap->delay_ns = timestamp2;
				p = &inf->aux.ext_data[ap->ext_data_offset];
				len = ap->nmp.ctrl & BITS_I2C_SLV_CTRL_LEN;
				ap->nmp.handler(p, len, timestamp1,
						ap->nmp.ext_driver);
			}
		}
	}
}

static void nvi_aux_ext_data_offset(struct inv_gyro_state_s *inf)
{
	int i;
	unsigned short offset;

	offset = 0;
	for (i = 0; i < AUX_PORT_SPECIAL; i++) {
		if ((inf->aux.port[i].hw_en) && (inf->aux.port[i].nmp.addr &
						 BIT_I2C_READ)) {
			inf->aux.port[i].ext_data_offset = offset;
			offset += (inf->aux.port[i].nmp.ctrl &
				   BITS_I2C_SLV_CTRL_LEN);
		}
	}
	if (offset > AUX_EXT_DATA_REG_MAX) {
		offset = AUX_EXT_DATA_REG_MAX;
		dev_err(&inf->i2c->dev,
			"%s ERR MPU slaves exceed data storage\n", __func__);
	}
	inf->aux.ext_data_n = offset;
	return;
}

static int nvi_aux_port_do(struct inv_gyro_state_s *inf,
			   int port, u8 data_out)
{
	unsigned char reg;
	int err;

	if (port == AUX_PORT_SPECIAL)
		reg = REG_I2C_SLV4_DO;
	else
		reg = (REG_I2C_SLV0_DO + port);
	err = inv_i2c_single_write(inf, reg, data_out);
	return err;
}

static int nvi_aux_port_data_out(struct inv_gyro_state_s *inf,
				 int port, u8 data_out)
{
	int err;

	err = nvi_aux_port_do(inf, port, data_out);
	if (!err) {
		inf->aux.port[port].nmp.data_out = data_out;
		inf->aux.port[port].hw_do = true;
	} else {
		inf->aux.port[port].hw_do = false;
	}
	return err;
}

static int nvi_aux_port_wr(struct inv_gyro_state_s *inf, int port)
{
	struct aux_port *ap;
	int err;

	ap = &inf->aux.port[port];
	err = inv_i2c_single_write(inf, (REG_I2C_SLV0_ADDR + (port * 3)),
				   ap->nmp.addr);
	err |= inv_i2c_single_write(inf, (REG_I2C_SLV0_REG + (port * 3)),
				    ap->nmp.reg);
	err |= nvi_aux_port_do(inf, port, ap->nmp.data_out);
	return err;
}

static int nvi_aux_port_en(struct inv_gyro_state_s *inf,
			   int port, bool en)
{
	struct aux_port *ap;
	unsigned char reg;
	unsigned char val;
	int err = 0;

	inf->aux.ext_data_n = 0;
	ap = &inf->aux.port[port];
	if ((!ap->hw_valid) && en) {
		err = nvi_aux_port_wr(inf, port);
		if (!err) {
			ap->hw_valid = true;
			ap->hw_do = true;
		}
	}
	if ((!ap->hw_do) && en)
		nvi_aux_port_data_out(inf, port, ap->nmp.data_out);
	if (port == AUX_PORT_SPECIAL) {
		err = nvi_i2c_slv4_ctrl_wr(inf, en);
	} else {
		reg = (REG_I2C_SLV0_CTRL + (port * 3));
		if (en)
			val = (ap->nmp.ctrl | BIT_SLV_EN);
		else
			val = 0;
		err = inv_i2c_single_write(inf, reg, val);
	}
	if (!err) {
		ap->hw_en = en;
		nvi_aux_ext_data_offset(inf);
	}
	return err;
}

static int nvi_aux_enable(struct inv_gyro_state_s *inf, bool enable)
{
	bool en;
	unsigned int i;
	int err;

	if (inf->hw.int_pin_cfg & BIT_BYPASS_EN)
		enable = false;

	en = false;
	if (enable) {
		/* global enable is honored only if a port is enabled */
		for (i = 0; i < AUX_PORT_MAX; i++) {
			if (inf->aux.port[i].enable) {
				en = true;
				break;
			}
		}
		if (en == (inf->hw.user_ctrl & BIT_I2C_MST_EN)) {
			/* if already on then just update delays */
			nvi_global_delay(inf);
		}
	}
	inf->aux.enable = en;
	if ((inf->hw.user_ctrl & BIT_I2C_MST_EN) == en)
		return 0;

	if (en) {
		for (i = 0; i < AUX_PORT_MAX; i++) {
			if (inf->aux.port[i].enable)
				err |= nvi_aux_port_en(inf, i, true);
		}
		nvi_motion_detect_enable(inf, 0);
	} else {
		for (i = 0; i < AUX_PORT_MAX; i++) {
			if (inf->aux.port[i].hw_valid)
				nvi_aux_port_en(inf, i, false);
		}
	}
	err = nvi_global_delay(inf);
	err |= nvi_user_ctrl_en_wr(inf, true, en);
	return err;
}

static int nvi_aux_port_enable(struct inv_gyro_state_s *inf,
			       int port, bool enable, bool fifo_enable)
{
	struct aux_port *ap;
	int err;

	ap = &inf->aux.port[port];
	ap->enable = enable;
	ap->fifo_en = false;
	if (enable && (inf->hw.int_pin_cfg & BIT_BYPASS_EN))
		return 0;

	err = nvi_aux_port_en(inf, port, enable);
	err |= nvi_aux_enable(inf, true);
	return err;
}

static int nvi_reset(struct inv_gyro_state_s *inf,
		     bool reset_fifo, bool reset_i2c)
{
	unsigned long flags;
	unsigned char val;
	int i;
	int err;

	err = nvi_int_enable_wr(inf, false);
	val = 0;
	if (reset_i2c) {
		/* nvi_aux_bypass_enable(inf, false)? */
		err |= nvi_aux_enable(inf, false);
		for (i = 0; i < AUX_PORT_MAX; i++) {
			inf->aux.port[i].hw_valid = false;
			inf->aux.port[i].hw_en = false;
		}
		inf->aux.need_reset = false;
		val |= BIT_I2C_MST_RST;
	}
	if (reset_fifo)
		val |= BIT_FIFO_RST;
	err |= nvi_user_ctrl_en_wr(inf, ~reset_fifo, ~reset_i2c);
	val |= inf->hw.user_ctrl;
	err |= nvi_user_ctrl_reset_wr(inf, val);
	if (reset_fifo) {
		spin_lock_irqsave(&inf->time_stamp_lock, flags);
		kfifo_reset(&inf->trigger.timestamps);
		spin_unlock_irqrestore(&inf->time_stamp_lock, flags);
		inf->last_isr_time = get_time_ns();
	}
	if (reset_i2c)
		err |= nvi_aux_enable(inf, true);
	else
		err |= nvi_user_ctrl_en_wr(inf, true, true);
	err |= nvi_int_enable_wr(inf, true);
	return err;
}

static int nvi_aux_port_free(struct inv_gyro_state_s *inf, int port)
{
	bool hw_valid;
	int err = 0;

	hw_valid = inf->aux.port[port].hw_valid;
	memset(&inf->aux.port[port], 0, sizeof(struct aux_port));
	if (hw_valid) {
		nvi_aux_port_wr(inf, port);
		nvi_aux_port_en(inf, port, false);
		nvi_aux_enable(inf, false);
		nvi_aux_enable(inf, true);
		if (port != AUX_PORT_SPECIAL)
			inf->aux.need_reset = true;
	}
	return err;
}

static int nvi_aux_port_alloc(struct inv_gyro_state_s *inf,
			      struct nvi_mpu_port *nmp, int port)
{
	int i;

	if (inf->aux.need_reset)
		nvi_reset(inf, false, true);
	if (port < 0) {
		for (i = 0; i < AUX_PORT_SPECIAL; i++) {
			if (inf->aux.port[i].nmp.addr == 0)
				break;
		}
		if (i == AUX_PORT_SPECIAL)
			return -ENODEV;
	} else {
		if (inf->aux.port[port].nmp.addr == 0)
			i = port;
		else
			return -ENODEV;
	}

	memset(&inf->aux.port[i], 0, sizeof(struct aux_port));
	memcpy(&inf->aux.port[i].nmp, nmp, sizeof(struct nvi_mpu_port));
	return i;
}

static int nvi_aux_bypass_enable(struct inv_gyro_state_s *inf, bool enable)
{
	unsigned char val;
	int err;

	if ((bool)(inf->hw.int_pin_cfg & BIT_BYPASS_EN) == enable)
		return 0;

	val = inf->hw.int_pin_cfg;
	if (enable) {
		err = nvi_aux_enable(inf, false);
		if (!err) {
			val |= BIT_BYPASS_EN;
			err = nvi_int_pin_cfg_wr(inf, val);
		}
	} else {
		val &= ~BIT_BYPASS_EN;
		err = nvi_int_pin_cfg_wr(inf, val);
		if (!err)
			nvi_aux_enable(inf, true);
	}
	return err;
}

static int nvi_aux_bypass_request(struct inv_gyro_state_s *inf, bool enable)
{
	int err = 0;

	if ((bool)(inf->hw.int_pin_cfg & BIT_BYPASS_EN) == enable) {
		inf->aux.bypass_lock++;
	} else {
		if (inf->aux.bypass_lock) {
			err = -EBUSY;
		} else {
			err = nvi_aux_bypass_enable(inf, enable);
			if (err)
				dev_err(&inf->i2c->dev, "%s ERR=%d\n",
					__func__, err);
			else
				inf->aux.bypass_lock++;
		}
	}
	return err;
}

static int nvi_aux_bypass_release(struct inv_gyro_state_s *inf)
{
	int err;

	if (inf->aux.bypass_lock)
		inf->aux.bypass_lock--;
	if (!inf->aux.bypass_lock) {
		err = nvi_aux_bypass_enable(inf, false);
		if (err)
			dev_err(&inf->i2c->dev, "%s ERR=%d\n", __func__, err);
	}
	return 0;
}

static int nvi_aux_dev_valid(struct inv_gyro_state_s *inf,
			     struct nvi_mpu_port *nmp, u8 *data)
{
	unsigned char val;
	int i;
	int err;

	/* turn off bypass */
	err = nvi_aux_bypass_request(inf, false);
	if (err)
		return -EBUSY;

	/* grab the special port */
	err = nvi_aux_port_alloc(inf, nmp, AUX_PORT_SPECIAL);
	if (err != AUX_PORT_SPECIAL) {
		nvi_aux_bypass_release(inf);
		return -EBUSY;
	}

	/* enable it */
	inf->aux.port[AUX_PORT_SPECIAL].nmp.delay_ms = 0;
	err = nvi_aux_port_enable(inf, AUX_PORT_SPECIAL, true, false);
	if (err) {
		nvi_aux_port_free(inf, AUX_PORT_SPECIAL);
		nvi_aux_bypass_release(inf);
		return -EBUSY;
	}

	/* now turn off all the other ports for fastest response */
	for (i = 0; i < AUX_PORT_SPECIAL; i++) {
		if (inf->aux.port[i].hw_valid)
			nvi_aux_port_en(inf, i, false);
	}
	/* start reading the results */
	for (i = 0; i < AUX_DEV_VALID_READ_MAX; i++) {
		mdelay(1);
		val = 0;
		err = inv_i2c_read(inf, REG_I2C_MST_STATUS, 1, &val);
		if (err)
			continue;

		if (val & 0x50)
			break;
	}
	/* these will restore all previously disabled ports */
	nvi_aux_bypass_release(inf);
	nvi_aux_port_free(inf, AUX_PORT_SPECIAL);
	if (i == AUX_DEV_VALID_READ_MAX)
		return -ENODEV;

	if (val & 0x10) /* NACK */
		return -EIO;

	if (nmp->addr & BIT_I2C_READ) {
		err = inv_i2c_read(inf, REG_I2C_SLV4_DI, 1, &val);
		if (err)
			return -EBUSY;

		*data = (u8)val;
		dev_info(&inf->i2c->dev, "%s MPU read 0x%x from device 0x%x\n",
			__func__, val, (nmp->addr & ~BIT_I2C_READ));
	} else {
		dev_info(&inf->i2c->dev, "%s MPU found device 0x%x\n",
			__func__, (nmp->addr & ~BIT_I2C_READ));
	}
	return 0;
}

static int nvi_aux_mpu_call_pre(struct inv_gyro_state_s *inf, int port)
{
	if ((port < 0) || (port >= AUX_PORT_SPECIAL))
		return -EINVAL;

	if (inf->shutdown)
		return -EPERM;

	if (!inf->aux.port[port].nmp.addr)
		return -EINVAL;

	mutex_lock(&inf->mutex);
	return 0;
}

static int nvi_aux_mpu_call_post(struct inv_gyro_state_s *inf,
				 char *tag, int err)
{
	if (err < 0)
		err = -EBUSY;
	mutex_unlock(&inf->mutex);
	nvi_aux_dbg(inf, tag, err);
	return err;
}

/* See the mpu.h file for details on the nvi_mpu_ calls.
 */
int nvi_mpu_dev_valid(struct nvi_mpu_port *nmp, u8 *data)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s\n", __func__);
		return -EAGAIN;
	}

	if (nmp == NULL)
		return -EINVAL;

	if ((nmp->addr & BIT_I2C_READ) && (data == NULL))
		return -EINVAL;

	if (inf->shutdown)
		return -EPERM;

	mutex_lock(&inf->mutex);
	nvi_pm(inf, NVI_PM_ON);
	err = nvi_aux_dev_valid(inf, nmp, data);
	nvi_pm(inf, NVI_PM_AUTO);
	mutex_unlock(&inf->mutex);
	nvi_aux_dbg(inf, "nvi_mpu_dev_valid err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_dev_valid);

int nvi_mpu_port_alloc(struct nvi_mpu_port *nmp)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s\n", __func__);
		return -EAGAIN;
	}

	if (nmp == NULL)
		return -EINVAL;

	if (!(nmp->ctrl & BITS_I2C_SLV_CTRL_LEN))
		return -EINVAL;

	if (inf->shutdown)
		return -EPERM;

	mutex_lock(&inf->mutex);
	nvi_pm(inf, NVI_PM_ON);
	err = nvi_aux_port_alloc(inf, nmp, -1);
	nvi_pm(inf, NVI_PM_AUTO);
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_port_alloc err/port: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_port_alloc);

int nvi_mpu_port_free(int port)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s port %d\n", __func__, port);
	} else {
		pr_debug("%s port %d\n", __func__, port);
		return -EAGAIN;
	}

	err = nvi_aux_mpu_call_pre(inf, port);
	if (err)
		return err;

	nvi_pm(inf, NVI_PM_ON);
	err = nvi_aux_port_free(inf, port);
	nvi_pm(inf, NVI_PM_AUTO);
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_port_free err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_port_free);

int nvi_mpu_enable(int port, bool enable, bool fifo_enable)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s port %d: %x\n", __func__, port, enable);
	} else {
		pr_debug("%s port %d: %x\n", __func__, port, enable);
		return -EAGAIN;
	}

	err = nvi_aux_mpu_call_pre(inf, port);
	if (err)
		return err;

	nvi_pm(inf, NVI_PM_ON);
	err = nvi_aux_port_enable(inf, port, enable, fifo_enable);
	nvi_pm(inf, NVI_PM_AUTO);
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_enable err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_enable);

int nvi_mpu_delay_ms(int port, u8 delay_ms)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s port %d: %u\n", __func__, port, delay_ms);
	} else {
		pr_debug("%s port %d: %u\n", __func__, port, delay_ms);
		return -EAGAIN;
	}

	err = nvi_aux_mpu_call_pre(inf, port);
	if (err)
		return err;

	if (inf->aux.port[port].hw_en) {
		err = nvi_aux_delay(inf, port, delay_ms);
		nvi_global_delay(inf);
	} else {
		inf->aux.port[port].nmp.delay_ms = delay_ms;
	}
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_delay_ms err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_delay_ms);

int nvi_mpu_delay_us(int port, unsigned long delay_us)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s port %d: %lu\n", __func__, port, delay_us);
	} else {
		pr_debug("%s port %d: %lu\n", __func__, port, delay_us);
		return -EAGAIN;
	}

	err = nvi_aux_mpu_call_pre(inf, port);
	if (err)
		return err;

	inf->aux.port[port].nmp.delay_us = delay_us;
	if (inf->aux.port[port].hw_en)
		err = nvi_global_delay(inf);
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_delay_us err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_delay_us);

int nvi_mpu_data_out(int port, u8 data_out)
{
	struct inv_gyro_state_s *inf;
	int err = 0;

	inf = inf_local;
	if (inf == NULL)
		return -EAGAIN;

	err = nvi_aux_mpu_call_pre(inf, port);
	if (err)
		return err;

	if (inf->aux.port[port].hw_en) {
		err = nvi_aux_port_data_out(inf, port, data_out);
	} else {
		inf->aux.port[port].nmp.data_out = data_out;
		inf->aux.port[port].hw_do = false;
	}
	if (err < 0)
		err = -EBUSY;
	mutex_unlock(&inf->mutex);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_data_out);

int nvi_mpu_bypass_request(bool enable)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s\n", __func__);
		return -EAGAIN;
	}

	if (inf->shutdown)
		return -EPERM;

	mutex_lock(&inf->mutex);
	nvi_pm(inf, NVI_PM_ON);
	err = nvi_aux_bypass_request(inf, enable);
	nvi_pm(inf, NVI_PM_AUTO);
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_bypass_request err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_bypass_request);

int nvi_mpu_bypass_release(void)
{
	struct inv_gyro_state_s *inf;
	int err;

	inf = inf_local;
	if (inf != NULL) {
		if (inf->aux.dbg)
			pr_info("%s\n", __func__);
	} else {
		pr_debug("%s\n", __func__);
		return -EAGAIN;
	}

	if (inf->shutdown)
		return -EPERM;

	mutex_lock(&inf->mutex);
	nvi_pm(inf, NVI_PM_ON);
	err = nvi_aux_bypass_release(inf);
	nvi_pm(inf, NVI_PM_AUTO);
	err = nvi_aux_mpu_call_post(inf, "nvi_mpu_bypass_release err: ", err);
	return err;
}
EXPORT_SYMBOL(nvi_mpu_bypass_release);


static int nvi_gyro_enable(struct inv_gyro_state_s *inf,
			   unsigned char enable, unsigned char fifo_enable)
{
	unsigned char enable_old;
	unsigned char fifo_enable_old;
	int err;
	int err_t;

	enable_old = inf->chip_config.gyro_enable;
	fifo_enable_old = inf->chip_config.gyro_fifo_enable;
	inf->chip_config.gyro_fifo_enable = fifo_enable;
	inf->chip_config.gyro_enable = enable;
	err_t = nvi_pm(inf, NVI_PM_ON_FULL);
	if (enable != enable_old) {
		if (enable) {
			err = nvi_gyro_config_wr(inf,
						 inf->chip_config.gyro_fsr);
			if (err < 0)
				err_t |= err;
			nvi_motion_detect_enable(inf, 0);
		}
		nvi_global_delay(inf);
	}
	if (fifo_enable_old != fifo_enable)
		err_t = nvi_reset(inf, true, false);
	if (err_t) {
		inf->chip_config.gyro_enable = enable_old;
		inf->chip_config.gyro_fifo_enable = fifo_enable_old;
	}
	err_t |= nvi_pm(inf, NVI_PM_AUTO);
	return err_t;
}

static int nvi_accl_enable(struct inv_gyro_state_s *inf,
			   unsigned char enable, unsigned char fifo_enable)
{
	unsigned char enable_old;
	unsigned char fifo_enable_old;
	int err;
	int err_t;

	enable_old = inf->chip_config.accl_enable;
	fifo_enable_old = inf->chip_config.accl_fifo_enable;
	inf->chip_config.accl_fifo_enable = fifo_enable;
	inf->chip_config.accl_enable = enable;
	err_t = nvi_pm(inf, NVI_PM_ON);
	if (enable != enable_old) {
		if (enable) {
			err = nvi_accel_config_wr(inf,
						 inf->chip_config.accl_fsr, 0);
			if (err < 0)
				err_t |= err;
		}
		nvi_global_delay(inf);
	}
	if (fifo_enable_old != fifo_enable)
		err_t = nvi_reset(inf, true, false);
	if (err_t) {
		inf->chip_config.accl_enable = enable_old;
		inf->chip_config.accl_fifo_enable = fifo_enable_old;
	}
	err_t |= nvi_pm(inf, NVI_PM_AUTO);
	return err_t;
}


static ssize_t nvi_gyro_enable_store(struct device *dev,
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
		err = nvi_gyro_enable(inf, enable, fifo_enable);
	}
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n",
			__func__, enable, err);
		return err;
	}

	return count;
}

ssize_t nvi_gyro_enable_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.gyro_enable);
}

ssize_t nvi_gyro_fifo_enable_store(struct device *dev,
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
		err = nvi_gyro_enable(inf, enable, fifo_enable);
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n",
			__func__, fifo_enable, err);
		return err;
	}

	return count;
}

ssize_t nvi_gyro_fifo_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.gyro_fifo_enable);
}

static ssize_t inv_gyro_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned long gyro_delay_us;
	unsigned long gyro_delay_us_old;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &gyro_delay_us);
	if (err)
		return err;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %lu\n", __func__, gyro_delay_us);
	if (gyro_delay_us < NVI_INPUT_GYRO_DELAY_US_MIN)
		gyro_delay_us = NVI_INPUT_GYRO_DELAY_US_MIN;
	if (gyro_delay_us != inf->chip_config.gyro_delay_us) {
		gyro_delay_us_old = inf->chip_config.gyro_delay_us;
		inf->chip_config.gyro_delay_us = gyro_delay_us;
		if (inf->chip_config.gyro_enable) {
			err = nvi_global_delay(inf);
			if (err)
				inf->chip_config.gyro_delay_us =
							     gyro_delay_us_old;
		}
	}
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %lu ERR=%d\n",
			__func__, gyro_delay_us, err);
		return err;
	}

	return count;
}

static ssize_t nvi_gyro_resolution_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (kstrtouint(buf, 10, &resolution))
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s %u", __func__, resolution);
	inf->chip_config.gyro_resolution = resolution;
	return count;
}

static ssize_t nvi_gyro_resolution_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct inv_gyro_state_s *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (inf->chip_config.gyro_enable)
		resolution = inf->chip_config.gyro_resolution;
	else
		resolution = GYRO_INPUT_RESOLUTION;
	return sprintf(buf, "%u\n", resolution);
}

static ssize_t nvi_gyro_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf;

	inf = dev_get_drvdata(dev);
	if (inf->chip_config.gyro_enable)
		return sprintf(buf, "%lu\n", inf->sample_delay_us);

	return sprintf(buf, "%d\n", NVI_INPUT_GYRO_DELAY_US_MIN);
}

static ssize_t nvi_gyro_max_range_store(struct device *dev,
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
		if (inf->chip_config.gyro_enable) {
			err = nvi_gyro_config_wr(inf, fsr);
			if (err > 0)
				/* reset fifo to purge old data */
				nvi_reset(inf, true, false);
		}
		if (err >= 0)
			inf->chip_config.gyro_fsr = fsr;
	}
	mutex_unlock(&inf->mutex);
	if (err < 0) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n", __func__, fsr, err);
		return err;
	}

	return count;
}

ssize_t nvi_gyro_max_range_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);
	unsigned int range;

	if (inf->chip_config.gyro_enable)
		range = inf->chip_config.gyro_fsr;
	else
		range = (1 << inf->chip_config.gyro_fsr) * 250;
	return sprintf(buf, "%u\n", range);
}

static ssize_t nvi_accl_enable_store(struct device *dev,
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
	if (enable != inf->chip_config.accl_enable) {
		if (enable)
			fifo_enable = inf->chip_config.accl_fifo_enable;
		else
			fifo_enable = 0;
		err = nvi_accl_enable(inf, enable, fifo_enable);
	}
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n",
			__func__, enable, err);
		return err;
	}

	return count;
}

ssize_t nvi_accl_enable_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.accl_enable);
}

static ssize_t nvi_accl_fifo_enable_store(struct device *dev,
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
	enable = inf->chip_config.accl_enable;
	if (fifo_enable) {
		fifo_enable = 1;
		if (!enable)
			enable = 7;
	}
	if (fifo_enable != inf->chip_config.accl_fifo_enable)
		err = nvi_accl_enable(inf, enable, fifo_enable);
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n",
			__func__, fifo_enable, err);
		return err;
	}

	return count;
}

ssize_t nvi_accl_fifo_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.accl_fifo_enable);
}

static ssize_t nvi_accl_delay_store(struct device *dev,
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
			err = nvi_global_delay(inf);
			if (err)
				inf->chip_config.accl_delay_us =
							     accl_delay_us_old;
			else
				nvi_pm(inf, NVI_PM_AUTO);
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

ssize_t nvi_accl_delay_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf;

	inf = dev_get_drvdata(dev);
	if (inf->chip_config.accl_enable)
		return sprintf(buf, "%lu\n", inf->sample_delay_us);

	return sprintf(buf, "%d\n", NVI_INPUT_ACCL_DELAY_US_MIN);
}

static ssize_t nvi_accl_resolution_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (kstrtouint(buf, 10, &resolution))
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s %u", __func__, resolution);
	inf->chip_config.accl_resolution = resolution;
	return count;
}

static ssize_t nvi_accl_resolution_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct inv_gyro_state_s *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (inf->chip_config.accl_enable)
		resolution = inf->chip_config.accl_resolution;
	else
		resolution = ACCL_INPUT_RESOLUTION;
	return sprintf(buf, "%u\n", resolution);
}

static ssize_t nvi_accl_max_range_store(struct device *dev,
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
		if (inf->chip_config.accl_enable) {
			err = nvi_accel_config_wr(inf, fsr, 0);
			if (err > 0)
				/* reset fifo to purge old data */
				nvi_reset(inf, true, false);
			nvi_pm(inf, NVI_PM_AUTO);
		}
		if (err >= 0)
			inf->chip_config.accl_fsr = fsr;
	}
	mutex_unlock(&inf->mutex);
	if (err < 0) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d\n", __func__, fsr, err);
		return err;
	}

	return count;
}

ssize_t nvi_accl_max_range_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);
	unsigned int range;

	if (inf->chip_config.accl_enable)
		range = inf->chip_config.accl_fsr;
	else
		range = 0x4000 >> inf->chip_config.accl_fsr;
	return sprintf(buf, "%u\n", range);
}

static ssize_t nvi_lpa_delay_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned long lpa_delay_us;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &lpa_delay_us);
	if (err)
		return err;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %lu\n", __func__, lpa_delay_us);
	inf->chip_config.lpa_delay_us = lpa_delay_us;
	err = nvi_pm(inf, NVI_PM_AUTO);
	mutex_unlock(&inf->mutex);
	if (err)
		dev_err(&inf->i2c->dev, "%s: %lu ERR=%d\n",
			__func__, lpa_delay_us, err);
	return count;
}

static ssize_t nvi_lpa_delay_enable_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", inf->chip_config.lpa_delay_us);
}

static ssize_t nvi_motion_thr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char mot_thr;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &mot_thr);
	if (err)
		return -EINVAL;

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %u\n", __func__, mot_thr);
	nvi_pm(inf, NVI_PM_ON);
	err = nvi_motion_detect_enable(inf, mot_thr);
	err |= nvi_pm(inf, NVI_PM_AUTO);
	mutex_unlock(&inf->mutex);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %u ERR=%d\n",
			__func__, mot_thr, err);
		return err;
	}

	return count;
}

static ssize_t nvi_motion_thr_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf;
	unsigned char mot_thr;

	inf = dev_get_drvdata(dev);
	if (inf->mot_enable)
		mot_thr = inf->hw.mot_thr;
	else
		mot_thr = 0;
	return sprintf(buf, "%u\n", mot_thr);
}

static ssize_t nvi_motion_dur_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char mot_dur;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &mot_dur);
	if (err)
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s: %u\n", __func__, mot_dur);
	inf->chip_config.mot_dur = mot_dur;
	return count;
}

static ssize_t nvi_motion_dur_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.mot_dur);
}

static ssize_t nvi_motion_ctrl_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char mot_ctrl;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &mot_ctrl);
	if (err)
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s: %u\n", __func__, mot_ctrl);
	inf->chip_config.mot_ctrl = mot_ctrl;
	return count;
}

static ssize_t nvi_motion_ctrl_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.mot_ctrl);
}

static ssize_t nvi_motion_count_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned int mot_cnt;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &mot_cnt);
	if (err)
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s: %u\n", __func__, mot_cnt);
	inf->chip_config.mot_cnt = mot_cnt;
	return count;
}

static ssize_t nvi_motion_count_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.mot_cnt);
}

static ssize_t nvi_enable_store(struct device *dev,
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

	mutex_lock(&inf->mutex);
	dev_dbg(&inf->i2c->dev, "%s: %u\n", __func__, enable);
	if (enable)
		enable = 1;
	if (enable != inf->chip_config.enable) {
		inf->chip_config.enable = enable;
		err = nvi_pm(inf, NVI_PM_AUTO);
	}
	mutex_unlock(&inf->mutex);
	if (err)
		dev_err(&inf->i2c->dev, "%s: %u ERR=%d\n",
			__func__, enable, err);
	return count;
}

ssize_t nvi_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", inf->chip_config.enable);
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

static ssize_t inv_temperature_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf;
	ssize_t rtn;

	inf = dev_get_drvdata(dev);
	mutex_lock(&inf->mutex_temp);
	rtn = sprintf(buf, "%d %lld\n", inf->temp_val, inf->temp_ts);
	mutex_unlock(&inf->mutex_temp);
	return rtn;
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

	if (st->chip_config.enable)
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

	for (ii = 0; ii < st->hw_s->num_reg; ii++) {
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
 * inv_gyro_orientation_show() - show orientation matrix
 */
static ssize_t inv_gyro_orientation_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
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

#if DEBUG_SYSFS_INTERFACE
static ssize_t nvi_dbg_i2c_addr_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char dbg_i2c_addr;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 16, &dbg_i2c_addr);
	if (err)
		return -EINVAL;

	inf->dbg_i2c_addr = dbg_i2c_addr;
	return count;
}

static ssize_t nvi_dbg_i2c_addr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	ssize_t bytes_printed = 0;

	bytes_printed += sprintf(buf + bytes_printed,
				 "%#2x\n", st->dbg_i2c_addr);
	return bytes_printed;
}

static ssize_t nvi_dbg_reg_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned char dbg_reg;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 16, &dbg_reg);
	if (err)
		return -EINVAL;

	inf->dbg_reg = dbg_reg;
	return count;
}

static ssize_t nvi_dbg_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);
	ssize_t bytes_printed = 0;

	bytes_printed += sprintf(buf + bytes_printed, "%#2x\n", st->dbg_reg);
	return bytes_printed;
}

static ssize_t nvi_dbg_dat_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned short dbg_i2c_addr;
	unsigned char dbg_dat;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 16, &dbg_dat);
	if (err)
		return -EINVAL;

	if (inf->dbg_i2c_addr) {
		err = inv_i2c_single_write_base(inf, inf->dbg_i2c_addr,
						inf->dbg_reg, dbg_dat);
		dbg_i2c_addr = inf->dbg_i2c_addr;
	} else {
		err = inv_i2c_single_write(inf, inf->dbg_reg, dbg_dat);
		dbg_i2c_addr = inf->i2c->addr;
	}
	pr_info("%s dev=%x reg=%x data=%x err=%d\n",
		__func__, dbg_i2c_addr, inf->dbg_reg, dbg_dat, err);
	return count;
}

static ssize_t nvi_dbg_dat_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf;
	unsigned short dbg_i2c_addr;
	unsigned char data;
	ssize_t bytes_printed = 0;

	inf = dev_get_drvdata(dev);
	if (inf->dbg_i2c_addr) {
		inv_i2c_read_base(inf, inf->dbg_i2c_addr,
				  inf->dbg_reg, 1, &data);
		dbg_i2c_addr = inf->dbg_i2c_addr;
	} else {
		inv_i2c_read(inf, inf->dbg_reg, 1, &data);
		dbg_i2c_addr = inf->i2c->addr;
	}
	bytes_printed += sprintf(buf + bytes_printed, "%#2x:%#2x=%#2x\n",
				 dbg_i2c_addr, inf->dbg_reg, data);
	return bytes_printed;
}

static ssize_t nvi_aux_dbg_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned int enable;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &enable);
	if (err)
		return err;

	inf->aux.dbg = true;
	nvi_aux_dbg(inf, "SNAPSHOT", 0);
	if (enable)
		inf->aux.dbg = true;
	else
		inf->aux.dbg = false;
	return count;
}

static ssize_t nvi_aux_dbg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", inf->aux.dbg);
}

static ssize_t nvi_mot_dbg_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned int mot_dbg;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &mot_dbg);
	if (err)
		return err;

	if (mot_dbg)
		inf->mot_dbg = true;
	else
		inf->mot_dbg = false;
	return count;
}

static ssize_t nvi_mot_dbg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", inf->mot_dbg);
}
#endif /* DEBUG_SYSFS_INTERFACE */

static ssize_t nvi_min_delay_us_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct inv_gyro_state_s *inf;
	unsigned long min_delay_us;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &min_delay_us);
	if (err)
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s: %lu\n", __func__, min_delay_us);
	inf->chip_config.min_delay_us = min_delay_us;
	return count;
}

static ssize_t nvi_min_delay_us_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *inf = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", inf->chip_config.min_delay_us);
}


static void inv_report_gyro_accl(struct inv_gyro_state_s *st, s64 t,
				 unsigned char *data)
{
	short x, y, z;
	int ind;
	struct inv_chip_config_s *conf;

	conf = &st->chip_config;
	ind = 0;
	if (conf->accl_fifo_enable) {
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
	if (conf->gyro_fifo_enable) {
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
	if (conf->accl_fifo_enable | conf->gyro_fifo_enable) {
		input_report_rel(st->idev, REL_MISC, (unsigned int)(t >> 32));
		input_report_rel(st->idev, REL_WHEEL,
			(unsigned int)(t & 0xffffffff));
		input_sync(st->idev);
	}
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

	timestamp = get_time_ns();
	if ((!(st->hw.fifo_en & BIT_TEMP_FIFO_EN)) &&
						 st->chip_config.gyro_enable) {
		result = inv_i2c_read(st, st->reg->temperature, 2, data);
		if (!result) {
			mutex_lock(&st->mutex_temp);
			st->temp_val = (data[0] << 8) | data[1];
			st->temp_ts = timestamp;
			mutex_unlock(&st->mutex_temp);
		}
	}
	if (st->mot_cnt)
		st->mot_cnt--;
	if (st->lpa_enable || (st->hw.int_enable & BIT_MOT_EN)) {
		if (st->hw.int_enable & BIT_MOT_EN) {
			st->mot_cnt = st->chip_config.mot_cnt;
			st->mot_enable = false;
			nvi_int_enable_wr(st, true);
			if (st->mot_dbg)
				pr_info("%s motion detect off", __func__);
		}
		result = inv_i2c_read(st, reg->raw_accl, 6, data);
		if (result)
			goto end_session;

		inv_report_gyro_accl(st, timestamp, data);
		if (st->mot_enable && st->mot_dbg)
			pr_info("%s SENDING MOTION DETECT DATA", __func__);
		if (st->mot_enable && (!st->mot_cnt))
			nvi_int_enable_wr(st, true);
		goto end_session;
	}

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
		if (kfifo_len(&st->trigger.timestamps) < (fifo_count /
							  bytes_per_datum))
			goto flush_fifo;

		if (kfifo_len(&st->trigger.timestamps) > (fifo_count /
							  bytes_per_datum +
							  TIME_STAMP_TOR))
			goto flush_fifo;
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

	nvi_aux_read(st);

end_session:
	return IRQ_HANDLED;

flush_fifo:
	/* Flush HW and SW FIFOs. */
	nvi_reset(st, true, false);
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
	       (catch_up < MAX_CATCH_UP) && (!st->lpa_enable)) {
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

static int inv_pm(struct inv_gyro_state_s *inf, int pm_req)
{
	int err;

	if (inf->nvi) {
		err = nvi_pm(inf, pm_req);
	} else {
		if ((pm_req > NVI_PM_OFF) || (pm_req == NVI_PM_AUTO))
			err = set_power_mpu3050(inf, 1);
		else
			err = set_power_mpu3050(inf, 0);
	}
	return err;
}

#ifdef CONFIG_PM
static int inv_suspend(struct device *dev)
{
	int result;
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	result = inv_pm(st, NVI_PM_OFF_FORCE);
	if (result)
		dev_err(dev, "%s ERR\n", __func__);
	return result;
}

static int inv_resume(struct device *dev)
{
	int result;
	struct inv_gyro_state_s *st = dev_get_drvdata(dev);

	result = inv_pm(st, NVI_PM_AUTO);
	if (result)
		dev_err(dev, "%s ERR\n", __func__);
	return result;
}

static const struct dev_pm_ops inv_pm_ops = {
	.suspend = inv_suspend,
	.resume = inv_resume,
};
#endif

static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_gyro_enable_show, nvi_gyro_enable_store);
static DEVICE_ATTR(gyro_fifo_enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_gyro_fifo_enable_show, nvi_gyro_fifo_enable_store);
static DEVICE_ATTR(gyro_delay, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_gyro_delay_show, inv_gyro_delay_store);
static DEVICE_ATTR(gyro_resolution, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_gyro_resolution_show, nvi_gyro_resolution_store);
static DEVICE_ATTR(gyro_max_range, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_gyro_max_range_show, nvi_gyro_max_range_store);
static DEVICE_ATTR(gyro_orientation, S_IRUGO,
		   inv_gyro_orientation_show, NULL);
static DEVICE_ATTR(raw_gyro, S_IRUGO,
		   inv_raw_gyro_show, NULL);
static DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_accl_enable_show, nvi_accl_enable_store);
static DEVICE_ATTR(accl_fifo_enable, S_IRUGO | S_IWUSR,
		   nvi_accl_fifo_enable_show, nvi_accl_fifo_enable_store);
static DEVICE_ATTR(accl_delay, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_accl_delay_show, nvi_accl_delay_store);
static DEVICE_ATTR(accl_resolution, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_accl_resolution_show, nvi_accl_resolution_store);
static DEVICE_ATTR(accl_max_range, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_accl_max_range_show, nvi_accl_max_range_store);
static DEVICE_ATTR(accl_orientation, S_IRUGO,
		   inv_accl_matrix_show, NULL);
static DEVICE_ATTR(raw_accl, S_IRUGO,
		   inv_raw_accl_show, NULL);
static DEVICE_ATTR(lpa_delay, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_lpa_delay_enable_show, nvi_lpa_delay_enable_store);
static DEVICE_ATTR(motion_threshold, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_motion_thr_show, nvi_motion_thr_store);
static DEVICE_ATTR(motion_duration, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_motion_dur_show, nvi_motion_dur_store);
static DEVICE_ATTR(motion_count, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_motion_count_show, nvi_motion_count_store);
static DEVICE_ATTR(motion_control, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_motion_ctrl_show, nvi_motion_ctrl_store);
static DEVICE_ATTR(temp_scale, S_IRUGO,
		   inv_temp_scale_show, NULL);
static DEVICE_ATTR(temp_offset, S_IRUGO,
		   inv_temp_offset_show, NULL);
static DEVICE_ATTR(temperature, S_IRUGO,
		   inv_temperature_show, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_enable_show, nvi_enable_store);
static DEVICE_ATTR(dbg_reg, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_dbg_reg_show, nvi_dbg_reg_store);
static DEVICE_ATTR(dbg_dat, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_dbg_dat_show, nvi_dbg_dat_store);
static DEVICE_ATTR(dbg_i2c_addr, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_dbg_i2c_addr_show, nvi_dbg_i2c_addr_store);
static DEVICE_ATTR(aux_dbg, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_aux_dbg_show, nvi_aux_dbg_store);
static DEVICE_ATTR(mot_dbg, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_mot_dbg_show, nvi_mot_dbg_store);
static DEVICE_ATTR(key, S_IRUGO | S_IWUSR,
		   inv_key_show, inv_key_store);
static DEVICE_ATTR(reg_dump, S_IRUGO,
		   inv_reg_dump_show, NULL);
static DEVICE_ATTR(min_delay_us, S_IRUGO | S_IWUSR | S_IWOTH,
		   nvi_min_delay_us_show, nvi_min_delay_us_store);

static struct device_attribute *inv_attributes[] = {
	&dev_attr_gyro_delay,
	&dev_attr_gyro_orientation,
	&dev_attr_raw_gyro,
	&dev_attr_temp_scale,
	&dev_attr_temp_offset,
	&dev_attr_temperature,
	&dev_attr_reg_dump,
	&dev_attr_key,
#if DEBUG_SYSFS_INTERFACE
	&dev_attr_dbg_reg,
	&dev_attr_dbg_dat,
	&dev_attr_dbg_i2c_addr,
#endif /* DEBUG_SYSFS_INTERFACE */
	NULL
};

static struct device_attribute *inv_mpu6050_attributes[] = {
	&dev_attr_gyro_enable,
	&dev_attr_gyro_fifo_enable,
	&dev_attr_gyro_max_range,
	&dev_attr_gyro_resolution,
	&dev_attr_accl_enable,
	&dev_attr_accl_fifo_enable,
	&dev_attr_accl_delay,
	&dev_attr_accl_max_range,
	&dev_attr_accl_resolution,
	&dev_attr_accl_orientation,
	&dev_attr_raw_accl,
	&dev_attr_lpa_delay,
	&dev_attr_motion_threshold,
	&dev_attr_motion_duration,
	&dev_attr_motion_count,
	&dev_attr_motion_control,
	&dev_attr_enable,
#if DEBUG_SYSFS_INTERFACE
	&dev_attr_min_delay_us,
	&dev_attr_aux_dbg,
	&dev_attr_mot_dbg,
#endif /* DEBUG_SYSFS_INTERFACE */
	NULL
};

static int inv_check_chip_type(struct inv_gyro_state_s *st,
			       const struct i2c_device_id *id)
{
	struct inv_reg_map_s *reg;
	int result = 0;

	reg = st->reg;
	st->mpu_slave = NULL;
	if (!strcmp(id->name, "itg3500")) {
		st->chip_type = INV_ITG3500;
		st->nvi = false;
	} else if (!strcmp(id->name, "mpu3050")) {
		st->chip_type = INV_MPU3050;
		inv_setup_reg_mpu3050(reg);
		st->nvi = false;
	} else if (!strcmp(id->name, "mpu6050")) {
		st->chip_type = INV_MPU6050;
		st->nvi = true;
	} else if (!strcmp(id->name, "mpu9150")) {
		st->chip_type = INV_MPU9150;
		st->nvi = true;
	}
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		if (st->plat_data.sec_slave_id == ACCEL_ID_KXTF9)
			inv_register_kxtf9_slave(st);
	}
	if (st->nvi) {
		nvi_pm_init(st);
	} else {
		nvi_vreg_init(st);
		/*reset register to power up default*/
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, 0);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1, BIT_RESET);
		if (!result)
			mdelay(POWER_UP_TIME);
	}
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
	st->chip_config.min_delay_us = MIN_FIFO_RATE;
	st->chip_config.lpf = INV_FILTER_42HZ;
	st->chip_config.gyro_enable = 0;
	st->chip_config.gyro_fifo_enable = 0;
	st->chip_config.gyro_fsr = INV_FSR_2000DPS;
	st->chip_config.accl_enable = 0;
	st->chip_config.accl_fifo_enable = 0;
	st->chip_config.accl_fsr = INV_FS_02G;
	st->chip_config.mot_dur = 1;
	st->chip_config.mot_ctrl = 1;
	st->chip_config.mot_cnt = 10;
	st->irq_dur_us = 20 * ONE_K_HZ;
	st->chip_config.fifo_rate = 50;
	st->chip_config.enable = 0;
	st->chip_config.dmp_on = 0;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	return 0;
}

static void inv_input_close(struct input_dev *d)
{
	struct inv_gyro_state_s *st;

	st = input_get_drvdata(d);
	inv_pm(st, NVI_PM_OFF_FORCE);
	nvi_vreg_exit(st);
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
	if (strcmp(name, "INV_DMP"))
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

static int inv_create_input(struct inv_gyro_state_s *st,
		struct i2c_client *client){
	struct input_dev *idev;
	int result;

	idev = NULL;
	result = inv_setup_input(st, &idev, client, st->hw_s->name);
	if (result)
		return result;

	st->idev = idev;
	if (INV_ITG3500 == st->chip_type)
		return 0;

	result = inv_setup_input(st, &idev, client, "INV_DMP");
	if (result)
		input_unregister_device(st->idev);
	else
		st->idev_dmp = idev;
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

	return 0;

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
	device_destroy(st->inv_class, inv_device_dev_t);
	class_destroy(st->inv_class);
	st->inv_dev = NULL;
	st->inv_class = NULL;
}

static void nvi_shutdown(struct i2c_client *client)
{
	struct inv_gyro_state_s *inf;
	int i;

	inf = i2c_get_clientdata(client);
	if (inf != NULL) {
		if (inf->nvi) {
			for (i = 0; i < AUX_PORT_SPECIAL; i++) {
				if (inf->aux.port[i].nmp.shutdown_bypass) {
					nvi_aux_bypass_enable(inf, true);
					break;
				}
			}
		}
		inf->shutdown = true;
		if (inf->inv_dev)
			remove_sysfs_interfaces(inf);
		kfifo_free(&inf->trigger.timestamps);
		free_irq(client->irq, inf);
		if (inf->idev)
			input_unregister_device(inf->idev);
		if ((INV_ITG3500 != inf->chip_type) && (inf->idev_dmp))
			input_unregister_device(inf->idev_dmp);
	}
}

static int nvi_remove(struct i2c_client *client)
{
	struct inv_gyro_state_s *inf;

	nvi_shutdown(client);
	inf = i2c_get_clientdata(client);
	if (inf != NULL) {
		if (inf->nvi) {
			nvi_pm_exit(inf);
		} else {
			inv_pm(inf, NVI_PM_OFF_FORCE);
			nvi_vreg_exit(inf);
		}
		kfree(inf);
	}
	dev_info(&client->dev, "Gyro module removed.\n");
	return 0;
}

static int nvi_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct inv_gyro_state_s *st;
	int result;

	pr_info("%s: Probe name %s\n", __func__, id->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st) {
		result = -ENOMEM;
		goto out_no_free;
	}

	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, st);
	st->i2c = client;
	st->sl_handle = client->adapter;
	st->reg = (struct inv_reg_map_s *)&chip_reg ;
	st->hw_s = (struct inv_hw_s *)hw_info;
	st->i2c_addr = client->addr;
	st->plat_data =
		   *(struct mpu_platform_data *)dev_get_platdata(&client->dev);

	/* power is turned on inside check chip type*/
	result = inv_check_chip_type(st, id);
	if (result)
		goto out_free;

	st->hw_s = (struct inv_hw_s *)(hw_info  + st->chip_type);
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
				"%s get silicon error.\n", st->hw_s->name);
			goto out_free;
		}
	}

	result = inv_pm(st, NVI_PM_OFF);
	if (result) {
		dev_err(&client->adapter->dev,
			"%s could not be turned off.\n", st->hw_s->name);
		goto out_free;
	}

	mutex_init(&st->mutex);
	mutex_init(&st->mutex_temp);
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

	inf_local = st;
	dev_info(&client->adapter->dev, "%s is ready to go!\n", st->hw_s->name);
	return 0;

out_close_sysfs:
	remove_sysfs_interfaces(st);
out_free_kfifo:
	kfifo_free(&st->trigger.timestamps);
out_free:
	if (st->nvi) {
		nvi_pm_exit(st);
	} else {
		inv_pm(st, NVI_PM_OFF_FORCE);
		nvi_vreg_exit(st);
	}
	kfree(st);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
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
	.probe		=	nvi_probe,
	.remove		=	nvi_remove,
	.id_table	=	inv_mod_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv_dev",
#ifdef CONFIG_PM
		.pm	=	&inv_pm_ops,
#endif
	},
	.address_list = normal_i2c,
	.shutdown	=	nvi_shutdown,
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

