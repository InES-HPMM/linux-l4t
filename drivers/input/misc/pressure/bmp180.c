/* Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/mpu.h>


#define BMP180_NAME			"bmp180"
#define BMP180_I2C_ADDR			(0x77)
#define BMP180_HW_DELAY_MS		(10)
#define BMP180_POLL_DELAY_MS_DFLT	(200)
#define BMP180_MPU_RETRY_COUNT		(20)
#define BMP180_MPU_RETRY_DELAY_MS	(20)
#define BMP180_ERR_CNT_MAX		(20)
/* sampling delays */
#define BMP180_DELAY_ULP		(5)
#define	BMP180_DELAY_ST			(8)
#define	BMP180_DELAY_HIGH_RES		(14)
#define	BMP180_DELAY_UHIGH_RES		(26)
/* input poll values*/
#define BMP180_INPUT_RESOLUTION		(1)
#define BMP180_INPUT_DIVISOR		(100)
#define BMP180_INPUT_DELAY_MS_MIN	(BMP180_DELAY_UHIGH_RES)
#define BMP180_INPUT_POWER_UA		(12)
#define BMP180_PRESSURE_MIN		(30000)
#define BMP180_PRESSURE_MAX		(110000)
#define BMP180_PRESSURE_FUZZ		(5)
#define BMP180_PRESSURE_FLAT		(5)
/* BMP180 registers */
#define BMP180_REG_ID			(0xD0)
#define BMP180_REG_ID_VAL		(0x55)
#define BMP180_REG_RESET		(0xE0)
#define BMP180_REG_RESET_VAL		(0xB6)
#define BMP180_REG_CTRL			(0xF4)
#define BMP180_REG_CTRL_MODE_MASK	(0x1F)
#define BMP180_REG_CTRL_MODE_PRES	(0x34)
#define BMP180_REG_CTRL_MODE_TEMP	(0x2E)
#define BMP180_REG_CTRL_OSS		(6)
#define BMP180_REG_CTRL_SCO		(5)
#define BMP180_REG_OUT_MSB		(0xF6)
#define BMP180_REG_OUT_LSB		(0xF7)
#define BMP180_REG_OUT_XLSB		(0xF8)
/* ROM registers */
#define BMP180_REG_AC1			(0xAA)
#define BMP180_REG_AC2			(0xAC)
#define BMP180_REG_AC3			(0xAE)
#define BMP180_REG_AC4			(0xB0)
#define BMP180_REG_AC5			(0xB2)
#define BMP180_REG_AC6			(0xB4)
#define BMP180_REG_B1			(0xB6)
#define BMP180_REG_B2			(0xB8)
#define BMP180_REG_MB			(0xBA)
#define BMP180_REG_MC			(0xBC)
#define BMP180_REG_MD			(0xBE)

#define WR				(0)
#define RD				(1)


/* regulator names in order of powering on */
static char *bmp180_vregs[] = {
	"vdd",
	"vddio",
};

static unsigned long bmp180_delay_ms_tbl[] = {
	BMP180_DELAY_ULP,
	BMP180_DELAY_ST,
	BMP180_DELAY_HIGH_RES,
	BMP180_DELAY_UHIGH_RES,
};


struct bmp180_rom {
	s16 ac1;
	s16 ac2;
	s16 ac3;
	u16 ac4;
	u16 ac5;
	u16 ac6;
	s16 b1;
	s16 b2;
	s16 mb;
	s16 mc;
	s16 md;
};

struct bmp180_inf {
	struct i2c_client *i2c;
	struct input_dev *idev;
	struct workqueue_struct *wq;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(bmp180_vregs)];
	struct mpu_platform_data pdata;
	struct bmp180_rom rom;		/* data for calibration */
	unsigned long poll_delay_us;	/* requested sampling delay (us) */
	unsigned int resolution;	/* report when new data outside this */
	bool use_mpu;			/* if device behind MPU */
	bool initd;			/* set if initialized */
	bool enable;			/* requested enable value */
	bool report;			/* used to report first valid sample */
	bool port_en[2];		/* enable status of MPU write port */
	int port_id[2];			/* MPU port ID */
	u8 data_out;			/* write value to trigger a sample */
	u8 range_index;			/* oversampling value */
	long UT;			/* uncompensated temperature */
	long UP;			/* uncompensated pressure */
	long temperature;		/* true temperature */
	int pressure;			/* true pressure hPa/100 Pa/1 mBar */
};


static int bmp180_i2c_rd(struct bmp180_inf *inf, u8 reg, u16 len, u8 *val)
{
	struct i2c_msg msg[2];

	msg[0].addr = BMP180_I2C_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = BMP180_I2C_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = val;
	if (i2c_transfer(inf->i2c->adapter, msg, 2) != 2)
		return -EIO;

	return 0;
}

static int bmp180_i2c_wr(struct bmp180_inf *inf, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	msg.addr = BMP180_I2C_ADDR;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	if (i2c_transfer(inf->i2c->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}

static int bmp180_vreg_dis(struct bmp180_inf *inf, int i)
{
	int err = 0;

	if (inf->vreg[i].ret && (inf->vreg[i].consumer != NULL)) {
		err = regulator_disable(inf->vreg[i].consumer);
		if (err)
			dev_err(&inf->i2c->dev, "%s %s ERR\n",
				__func__, inf->vreg[i].supply);
		else
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
	}
	inf->vreg[i].ret = 0;
	return err;
}

static int bmp180_vreg_dis_all(struct bmp180_inf *inf)
{
	unsigned int i;
	int err = 0;

	for (i = ARRAY_SIZE(bmp180_vregs); i > 0; i--)
		err |= bmp180_vreg_dis(inf, (i - 1));
	return err;
}

static int bmp180_vreg_en(struct bmp180_inf *inf, int i)
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

static int bmp180_vreg_en_all(struct bmp180_inf *inf)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(bmp180_vregs); i++)
		err |= bmp180_vreg_en(inf, i);
	return err;
}

static void bmp180_vreg_exit(struct bmp180_inf *inf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bmp180_vregs); i++) {
		if (inf->vreg[i].consumer != NULL) {
			devm_regulator_put(inf->vreg[i].consumer);
			inf->vreg[i].consumer = NULL;
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		}
	}
}

static int bmp180_vreg_init(struct bmp180_inf *inf)
{
	unsigned int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(bmp180_vregs); i++) {
		inf->vreg[i].supply = bmp180_vregs[i];
		inf->vreg[i].ret = 0;
		inf->vreg[i].consumer = devm_regulator_get(&inf->i2c->dev,
							  inf->vreg[i].supply);
		if (IS_ERR(inf->vreg[i].consumer)) {
			err |= PTR_ERR(inf->vreg[i].consumer);
			dev_err(&inf->i2c->dev, "%s err %d for %s\n",
				__func__, err, inf->vreg[i].supply);
			inf->vreg[i].consumer = NULL;
		} else {
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		}
	}
	return err;
}

static int bmp180_pm(struct bmp180_inf *inf, bool enable)
{
	int err;

	if (enable) {
		err = bmp180_vreg_en_all(inf);
		if (err)
			mdelay(BMP180_HW_DELAY_MS);
	} else {
		err = bmp180_vreg_dis_all(inf);
	}
	if (err > 0)
		err = 0;
	if (err)
		dev_err(&inf->i2c->dev, "%s enable=%x ERR=%d\n",
			__func__, enable, err);
	else
		dev_dbg(&inf->i2c->dev, "%s enable=%x\n",
			__func__, enable);
	return err;
}

static int bmp180_port_free(struct bmp180_inf *inf, int port)
{
	int err = 0;

	if ((inf->use_mpu) && (inf->port_id[port] >= 0)) {
		err = nvi_mpu_port_free(inf->port_id[port]);
		if (!err)
			inf->port_id[port] = -1;
	}
	return err;
}

static int bmp180_ports_free(struct bmp180_inf *inf)
{
	int err;

	err = bmp180_port_free(inf, WR);
	err |= bmp180_port_free(inf, RD);
	return err;
}

static void bmp180_pm_exit(struct bmp180_inf *inf)
{
	bmp180_ports_free(inf);
	bmp180_pm(inf, false);
	bmp180_vreg_exit(inf);
}

static int bmp180_pm_init(struct bmp180_inf *inf)
{
	int err;

	inf->initd = false;
	inf->enable = false;
	inf->port_en[WR] = false;
	inf->port_en[RD] = false;
	inf->port_id[WR] = -1;
	inf->port_id[RD] = -1;
	inf->resolution = 0;
	inf->range_index = 0;
	inf->poll_delay_us = (BMP180_POLL_DELAY_MS_DFLT * 1000);
	bmp180_vreg_init(inf);
	err = bmp180_pm(inf, true);
	return err;
}

static int bmp180_nvi_mpu_bypass_request(struct bmp180_inf *inf)
{
	int i;
	int err = 0;

	if (inf->use_mpu) {
		for (i = 0; i < BMP180_MPU_RETRY_COUNT; i++) {
			err = nvi_mpu_bypass_request(true);
			if ((!err) || (err == -EPERM))
				break;

			mdelay(BMP180_MPU_RETRY_DELAY_MS);
		}
		if (err == -EPERM)
			err = 0;
	}
	return err;
}

static int bmp180_nvi_mpu_bypass_release(struct bmp180_inf *inf)
{
	int err = 0;

	if (inf->use_mpu)
		err = nvi_mpu_bypass_release();
	return err;
}

static int bmp180_wr(struct bmp180_inf *inf, u8 reg, u8 val)
{
	int err = 0;

	err = bmp180_nvi_mpu_bypass_request(inf);
	if (!err) {
		err = bmp180_i2c_wr(inf, reg, val);
		bmp180_nvi_mpu_bypass_release(inf);
	}
	return err;
}

static int bmp180_port_enable(struct bmp180_inf *inf, int port, bool enable)
{
	int err = 0;

	if (enable != inf->port_en[port]) {
		err = nvi_mpu_enable(inf->port_id[port], enable, false);
		if (!err)
			inf->port_en[port] = enable;
	}
	return err;
}

static int bmp180_ports_enable(struct bmp180_inf *inf, bool enable)
{
	int err;

	err = bmp180_port_enable(inf, WR, enable);
	err |= bmp180_port_enable(inf, RD, enable);
	return err;
}

static int bmp180_reset(struct bmp180_inf *inf)
{
	int err = 0;

	if (inf->use_mpu)
		err = bmp180_ports_enable(inf, false);
	else
		cancel_delayed_work_sync(&inf->dw);
	if (err)
		return err;

	err = bmp180_wr(inf, BMP180_REG_RESET, BMP180_REG_RESET_VAL);
	if (!err)
		mdelay(BMP180_HW_DELAY_MS);
	if (inf->use_mpu) {
		err |= nvi_mpu_data_out(inf->port_id[WR],
					BMP180_REG_CTRL_MODE_TEMP);
		err |= bmp180_ports_enable(inf, true);
	} else {
		err = bmp180_wr(inf, BMP180_REG_CTRL,
				BMP180_REG_CTRL_MODE_TEMP);
		queue_delayed_work(inf->wq, &inf->dw,
				   usecs_to_jiffies(inf->poll_delay_us));
	}
	return err;
}

static int bmp180_delay(struct bmp180_inf *inf, unsigned long delay_us)
{
	int err = 0;

	if (inf->use_mpu)
		err = nvi_mpu_delay_us(inf->port_id[RD], delay_us);
	return err;
}

static int bmp180_init_hw(struct bmp180_inf *inf)
{
	u8 *p_rom1;
	u8 *p_rom2;
	u8 tmp;
	int i;
	int err = 0;

	inf->UT = 0;
	inf->UP = 0;
	inf->temperature = 0;
	inf->pressure = 0;
	p_rom1 = (u8 *)&inf->rom.ac1;
	err = bmp180_nvi_mpu_bypass_request(inf);
	if (!err) {
		err = bmp180_i2c_rd(inf, BMP180_REG_AC1, 22, p_rom1);
		bmp180_nvi_mpu_bypass_release(inf);
	}
	if (err)
		return err;

	for (i = 0; i < 11; i++) {
		p_rom2 = p_rom1;
		tmp = *p_rom1;
		*p_rom1++ = *++p_rom2;
		*p_rom2 = tmp;
		p_rom1++;
	}
	inf->initd = true;
	return err;
}

static void bmp180_calc(struct bmp180_inf *inf)
{
	long X1, X2, X3, B3, B5, B6, p;
	unsigned long B4, B7;
	long pressure;

	X1 = ((inf->UT - inf->rom.ac6) * inf->rom.ac5) >> 15;
	X2 = inf->rom.mc * (1 << 11) / (X1 + inf->rom.md);
	B5 = X1 + X2;
	inf->temperature = (B5 + 8) >> 4;
	B6 = B5 - 4000;
	X1 = (inf->rom.b2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (inf->rom.ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((inf->rom.ac1 << 2) + X3) << inf->range_index) + 2) >> 2;
	X1 = (inf->rom.ac3 * B6) >> 13;
	X2 = (inf->rom.b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (inf->rom.ac4 * (unsigned long)(X3 + 32768)) >> 15;
	B7 = ((unsigned long)inf->UP - B3) * (50000 >> inf->range_index);
	if (B7 < 0x80000000)
		p = (B7 << 1) / B4;
	else
		p = (B7 / B4) << 1;
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	pressure = p + ((X1 + X2 + 3791) >> 4);
	inf->pressure = (int)pressure;
}

static void bmp180_report(struct bmp180_inf *inf, u8 *data, s64 ts)
{
	input_report_abs(inf->idev, ABS_PRESSURE, inf->pressure);
	input_sync(inf->idev);
}

static int bmp180_read_sts(struct bmp180_inf *inf, u8 *data)
{
	long val;
	int limit_lo;
	int limit_hi;
	int pres;
	bool report;
	int err = 0;

	/* BMP180_REG_CTRL_SCO is 0 when data is ready */
	val = data[0] & (1 << BMP180_REG_CTRL_SCO);
	if (!val) {
		err = -1;
		if (data[0] == 0x0A) { /* temperature */
			inf->UT = ((data[2] << 8) + data[3]);
			inf->data_out = BMP180_REG_CTRL_MODE_PRES |
				     (inf->range_index << BMP180_REG_CTRL_OSS);
		} else { /* pressure */
			val = ((data[2] << 16) + (data[3] << 8) + data[4]) >>
							(8 - inf->range_index);
			inf->data_out = BMP180_REG_CTRL_MODE_TEMP;
			if (inf->resolution && (!inf->report)) {
				if (inf->UP == val)
					return err;
			}

			inf->UP = val;
			bmp180_calc(inf);
			pres = inf->pressure;
			if (inf->resolution) {
				limit_lo = pres;
				limit_hi = pres;
				limit_lo -= (inf->resolution / 2);
				limit_hi += (inf->resolution / 2);
				if (limit_lo < 0)
					limit_lo = 0;
				if ((pres < limit_lo) || (pres > limit_hi))
					report = true;
				else
					report = false;
			} else {
				report = true;
			}
			if (report || inf->report) {
				inf->report = false;
				err = 1;
			}
		}
	}
	return err;
}

static s64 bmp180_timestamp_ns(void)
{
	struct timespec ts;
	s64 ns;

	ktime_get_ts(&ts);
	ns = timespec_to_ns(&ts);
	return ns;
}

static int bmp180_read(struct bmp180_inf *inf)
{
	long long timestamp1;
	long long timestamp2;
	u8 data[5];
	int err;

	timestamp1 = bmp180_timestamp_ns();
	err = bmp180_i2c_rd(inf, BMP180_REG_CTRL, 5, data);
	timestamp2 = bmp180_timestamp_ns();
	if (err)
		return err;

	err = bmp180_read_sts(inf, data);
	if (err > 0) {
		timestamp2 = (timestamp2 - timestamp1) / 2;
		timestamp1 += timestamp2;
		bmp180_report(inf, data, timestamp1);
		bmp180_i2c_wr(inf, BMP180_REG_CTRL, inf->data_out);
	} else if (err < 0) {
		bmp180_i2c_wr(inf, BMP180_REG_CTRL, inf->data_out);
	}
	return err;
}

static void bmp180_mpu_handler(u8 *data, unsigned int len, s64 ts, void *p_val)
{
	struct bmp180_inf *inf;
	int err;

	inf = (struct bmp180_inf *)p_val;
	if (inf->enable) {
		err = bmp180_read_sts(inf, data);
		if (err > 0) {
			bmp180_report(inf, data, ts);
			nvi_mpu_data_out(inf->port_id[WR], inf->data_out);
		} else if (err < 0) {
			nvi_mpu_data_out(inf->port_id[WR], inf->data_out);
		}
	}
}

static int bmp180_id(struct bmp180_inf *inf)
{
	struct nvi_mpu_port nmp;
	u8 config_boot;
	u8 val = 0;
	int err;

	config_boot = inf->pdata.config & NVI_CONFIG_BOOT_MASK;
	if (config_boot == NVI_CONFIG_BOOT_AUTO) {
		nmp.addr = BMP180_I2C_ADDR | 0x80;
		nmp.reg = BMP180_REG_ID;
		nmp.ctrl = 1;
		err = nvi_mpu_dev_valid(&nmp, &val);
		/* see mpu.h for possible return values */
		if ((err == -EAGAIN) || (err == -EBUSY))
			return -EAGAIN;

		if (((!err) && (val == BMP180_REG_ID_VAL)) || (err == -EIO))
			config_boot = NVI_CONFIG_BOOT_MPU;
	}
	if (config_boot == NVI_CONFIG_BOOT_MPU) {
		inf->use_mpu = true;
		nmp.addr = BMP180_I2C_ADDR | 0x80;
		nmp.reg = BMP180_REG_CTRL;
		nmp.ctrl = 5;
		nmp.data_out = 0;
		nmp.delay_ms = 0;
		nmp.delay_us = inf->poll_delay_us;
		nmp.shutdown_bypass = false;
		nmp.handler = &bmp180_mpu_handler;
		nmp.ext_driver = (void *)inf;
		err = nvi_mpu_port_alloc(&nmp);
		if (err < 0)
			return err;

		inf->port_id[RD] = err;
		nmp.addr = BMP180_I2C_ADDR;
		nmp.reg = BMP180_REG_CTRL;
		nmp.ctrl = 1;
		nmp.data_out = BMP180_REG_CTRL_MODE_TEMP;
		nmp.delay_ms = bmp180_delay_ms_tbl[inf->range_index];
		nmp.delay_us = 0;
		nmp.shutdown_bypass = false;
		nmp.handler = NULL;
		nmp.ext_driver = NULL;
		err = nvi_mpu_port_alloc(&nmp);
		if (err < 0) {
			bmp180_ports_free(inf);
			dev_err(&inf->i2c->dev, "%s ERR %d", __func__, err);
		} else {
			inf->port_id[WR] = err;
			err = 0;
		}
		return err;
	}

	/* NVI_CONFIG_BOOT_EXTERNAL */
	inf->use_mpu = false;
	err = bmp180_i2c_rd(inf, BMP180_REG_ID, 1, &val);
	if ((!err) && (val == BMP180_REG_ID_VAL))
		return 0;

	return -ENODEV;
}

static void bmp180_work(struct work_struct *ws)
{
	struct bmp180_inf *inf;

	inf = container_of(ws, struct bmp180_inf, dw.work);
	bmp180_read(inf);
	queue_delayed_work(inf->wq, &inf->dw,
			   usecs_to_jiffies(inf->poll_delay_us));
}

static int bmp180_enable(struct bmp180_inf *inf, bool enable)
{
	int err = 0;

	bmp180_pm(inf, true);
	if (!inf->initd)
		err = bmp180_init_hw(inf);
	if (enable) {
		inf->report = true;
		err |= bmp180_delay(inf, inf->poll_delay_us);
		err |= bmp180_reset(inf);
		if (!err)
			inf->enable = true;
	} else {
		inf->enable = false;
		if (inf->use_mpu)
			err = bmp180_ports_enable(inf, false);
		else
			cancel_delayed_work_sync(&inf->dw);
		if (!err)
			bmp180_pm(inf, false);
	}
	return err;
}

static ssize_t bmp180_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct bmp180_inf *inf;
	unsigned int enable;
	bool en;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &enable);
	if (err)
		return -EINVAL;

	if (enable)
		en = true;
	else
		en = false;
	dev_dbg(&inf->i2c->dev, "%s: %x", __func__, en);
	err = bmp180_enable(inf, en);
	if (err) {
		dev_err(&inf->i2c->dev, "%s: %x ERR=%d", __func__, en, err);
		return err;
	}

	return count;
}

static ssize_t bmp180_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;
	unsigned int enable = 0;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		enable = 1;
	return sprintf(buf, "%u\n", enable);
}

static ssize_t bmp180_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct bmp180_inf *inf;
	unsigned long delay_us;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtoul(buf, 10, &delay_us);
	if (err)
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s: %lu\n", __func__, delay_us);
	/* since we rotate between acquiring data for pressure and temperature
	 * we need to go twice as fast.
	 */
	delay_us >>= 1;
	if (delay_us < (bmp180_delay_ms_tbl[inf->range_index] * 1000))
		delay_us = (bmp180_delay_ms_tbl[inf->range_index] * 1000);
	if (inf->enable && (delay_us != inf->poll_delay_us))
		err = bmp180_delay(inf, delay_us);
	if (!err) {
		inf->poll_delay_us = delay_us;
	} else {
		dev_err(&inf->i2c->dev, "%s: %lu ERR=%d\n",
			__func__, delay_us, err);
		return err;
	}

	return count;
}

static ssize_t bmp180_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;
	unsigned long delay_us;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		delay_us = inf->poll_delay_us;
	else
		delay_us = bmp180_delay_ms_tbl[inf->range_index] * 1000;
	return sprintf(buf, "%lu\n", delay_us);
}

static ssize_t bmp180_resolution_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct bmp180_inf *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (kstrtouint(buf, 10, &resolution))
		return -EINVAL;

	inf->resolution = resolution;
	dev_dbg(&inf->i2c->dev, "%s %u", __func__, resolution);
	return count;
}

static ssize_t bmp180_resolution_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		resolution = inf->resolution;
	else
		resolution = BMP180_INPUT_RESOLUTION;
	return sprintf(buf, "%u\n", resolution);
}

static ssize_t bmp180_max_range_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct bmp180_inf *inf;
	u8 range_index;
	int err;

	inf = dev_get_drvdata(dev);
	err = kstrtou8(buf, 10, &range_index);
	if (err)
		return -EINVAL;

	if (range_index > 3)
		return -EINVAL;

	dev_dbg(&inf->i2c->dev, "%s %u", __func__, range_index);
	inf->range_index = range_index;
	nvi_mpu_delay_ms(inf->port_id[WR], bmp180_delay_ms_tbl[range_index]);
	return count;
}

static ssize_t bmp180_max_range_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;
	unsigned int range;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		range = inf->range_index;
	else
		range = (BMP180_PRESSURE_MAX * BMP180_INPUT_DIVISOR);
	return sprintf(buf, "%u\n", range);
}

static ssize_t bmp180_divisor_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", BMP180_INPUT_DIVISOR);
}

static ssize_t bmp180_microamp_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", BMP180_INPUT_POWER_UA);
}

static ssize_t bmp180_pressure_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bmp180_inf *inf;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		return sprintf(buf, "%d\n", inf->pressure);

	return -EPERM;
}

static ssize_t bmp180_temperature_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct bmp180_inf *inf;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		return sprintf(buf, "%ld\n", inf->temperature);

	return -EPERM;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   bmp180_enable_show, bmp180_enable_store);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR | S_IWOTH,
		   bmp180_delay_show, bmp180_delay_store);
static DEVICE_ATTR(resolution, S_IRUGO | S_IWUSR | S_IWOTH,
		   bmp180_resolution_show, bmp180_resolution_store);
static DEVICE_ATTR(max_range, S_IRUGO | S_IWUSR | S_IWOTH,
		   bmp180_max_range_show, bmp180_max_range_store);
static DEVICE_ATTR(divisor, S_IRUGO,
		   bmp180_divisor_show, NULL);
static DEVICE_ATTR(microamp, S_IRUGO,
		   bmp180_microamp_show, NULL);
static DEVICE_ATTR(pressure, S_IRUGO,
		   bmp180_pressure_show, NULL);
static DEVICE_ATTR(temperature, S_IRUGO,
		   bmp180_temperature_show, NULL);

static struct attribute *bmp180_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_resolution.attr,
	&dev_attr_max_range.attr,
	&dev_attr_divisor.attr,
	&dev_attr_microamp.attr,
	&dev_attr_pressure.attr,
	&dev_attr_temperature.attr,
	NULL
};

static struct attribute_group bmp180_attr_group = {
	.name = BMP180_NAME,
	.attrs = bmp180_attrs
};

static int bmp180_sysfs_create(struct bmp180_inf *inf)
{
	int err;

	err = sysfs_create_group(&inf->idev->dev.kobj, &bmp180_attr_group);
	return err;
}

static void bmp180_input_close(struct input_dev *idev)
{
	struct bmp180_inf *inf;

	inf = input_get_drvdata(idev);
	if (inf != NULL)
		bmp180_enable(inf, false);
}

static int bmp180_input_create(struct bmp180_inf *inf)
{
	int err;

	inf->idev = input_allocate_device();
	if (!inf->idev) {
		err = -ENOMEM;
		dev_err(&inf->i2c->dev, "%s ERR %d\n", __func__, err);
		return err;
	}

	inf->idev->name = BMP180_NAME;
	inf->idev->dev.parent = &inf->i2c->dev;
	inf->idev->close = bmp180_input_close;
	input_set_drvdata(inf->idev, inf);
	input_set_capability(inf->idev, EV_ABS, ABS_PRESSURE);
	input_set_abs_params(inf->idev, ABS_PRESSURE,
			     BMP180_PRESSURE_MIN, BMP180_PRESSURE_MAX,
			     BMP180_PRESSURE_FUZZ, BMP180_PRESSURE_FLAT);
	err = input_register_device(inf->idev);
	if (err) {
		input_free_device(inf->idev);
		inf->idev = NULL;
	}
	return err;
}

static int bmp180_remove(struct i2c_client *client)
{
	struct bmp180_inf *inf;

	inf = i2c_get_clientdata(client);
	if (inf != NULL) {
		if (inf->idev) {
			input_unregister_device(inf->idev);
			input_free_device(inf->idev);
		}
		if (inf->wq)
			destroy_workqueue(inf->wq);
		bmp180_pm_exit(inf);
		kfree(inf);
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static void bmp180_shutdown(struct i2c_client *client)
{
	bmp180_remove(client);
}

static int bmp180_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bmp180_inf *inf;
	struct mpu_platform_data *pd;
	int err;

	dev_info(&client->dev, "%s\n", __func__);
	inf = kzalloc(sizeof(*inf), GFP_KERNEL);
	if (IS_ERR_OR_NULL(inf)) {
		dev_err(&client->dev, "%s kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	inf->i2c = client;
	i2c_set_clientdata(client, inf);
	pd = (struct mpu_platform_data *)dev_get_platdata(&client->dev);
	if (pd != NULL)
		inf->pdata = *pd;
	bmp180_pm_init(inf);
	err = bmp180_id(inf);
	bmp180_pm(inf, false);
	if (err == -EAGAIN)
		goto bmp180_probe_again;
	else if (err)
		goto bmp180_probe_err;

	err = bmp180_input_create(inf);
	if (err)
		goto bmp180_probe_err;

	inf->wq = create_singlethread_workqueue(BMP180_NAME);
	if (!inf->wq) {
		dev_err(&client->dev, "%s workqueue ERR\n", __func__);
		err = -ENOMEM;
		goto bmp180_probe_err;
	}

	INIT_DELAYED_WORK(&inf->dw, bmp180_work);
	err = bmp180_sysfs_create(inf);
	if (err)
		goto bmp180_probe_err;

	return 0;

bmp180_probe_err:
	dev_err(&client->dev, "%s ERR %d\n", __func__, err);
bmp180_probe_again:
	bmp180_remove(client);
	return err;
}

static const struct i2c_device_id bmp180_i2c_device_id[] = {
	{BMP180_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bmp180_i2c_device_id);

static struct i2c_driver bmp180_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= bmp180_probe,
	.remove		= bmp180_remove,
	.driver = {
		.name	= BMP180_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table	= bmp180_i2c_device_id,
	.shutdown	= bmp180_shutdown,
};

static int __init bmp180_init(void)
{
	return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
	i2c_del_driver(&bmp180_driver);
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMP180 driver");
MODULE_AUTHOR("NVIDIA Corp");

