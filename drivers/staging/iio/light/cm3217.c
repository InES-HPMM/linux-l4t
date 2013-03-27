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


/* IT = Integration Time.  The amount of time the photons hit the sensor.
 * STEP = the value from HW which is the photon count during IT.
 * LUX = STEP * (CM3217_RESOLUTION_STEP / IT) / CM3217_RESOLUTION_DIVIDER
 * The above LUX reported as LUX * CM3217_INPUT_LUX_DIVISOR.
 * The final value is done in user space to get a float value of
 * LUX / CM3217_INPUT_LUX_DIVISOR.
 */
#define CM3217_NAME			"cm3217"
#define CM3217_I2C_ADDR_CMD1_WR		(0x10)
#define CM3217_I2C_ADDR_CMD2_WR		(0x11)
#define CM3217_I2C_ADDR_RD		(0x10)
#define CM3217_HW_CMD1_DFLT		(0x22)
#define CM3217_HW_CMD1_BIT_SD		(0)
#define CM3217_HW_CMD1_BIT_IT_T		(2)
#define CM3217_HW_CMD2_BIT_FD_IT	(5)
#define CM3217_HW_DELAY			(10)
#define CM3217_POWER_UA			(90)
#define CM3217_RESOLUTION		(1)
#define CM3217_RESOLUTION_STEP		(6000000L)
#define CM3217_RESOLUTION_DIVIDER	(10000L)
#define CM3217_POLL_DELAY_MS_DFLT	(1600)
#define CM3217_POLL_DELAY_MS_MIN	(33 + CM3217_HW_DELAY)
#define CM3217_INPUT_LUX_DIVISOR	(10)
#define CM3217_INPUT_LUX_MIN		(0)
#define CM3217_INPUT_LUX_MAX		(119156)
#define CM3217_INPUT_LUX_FUZZ		(0)
#define CM3217_INPUT_LUX_FLAT		(0)


/* regulator names in order of powering on */
static char *cm3217_vregs[] = {
	"vdd",
};

struct cm3217_inf {
	struct i2c_client *i2c;
	struct mutex mutex;
	struct input_dev *idev;
	struct device *dev;
	struct workqueue_struct *wq;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[ARRAY_SIZE(cm3217_vregs)];
	s64 timestamp;			/* timestamp at last report */
	unsigned int queue_delay;	/* workqueue delay time (ms) */
	unsigned int poll_delay;	/* OS requested reporting delay (ms) */
	unsigned long mult;		/* used to calc lux from HW values */
	unsigned int index;		/* index into HW IT settings table */
	unsigned int resolution;	/* report when new lux outside this */
	int lux;			/* the final value that is reported */
	bool enable;			/* global enable flag */
	bool hw_change;			/* HW changed so drop first sample */
	bool hw_sync;			/* queue time match HW sample time */
	bool report;			/* used to report first valid sample */
};

struct cm3217_it {			/* integration time */
	unsigned int ms;		/* time ms */
	__u8 fd_it;			/* FD_IT HW */
	__u8 it_t;			/* IT_T HW */
};

static struct cm3217_it cm3217_it_tbl[] = {
	{3200, 0, 3},			/* 800ms * 4 */
	{1600, 0, 2},			/* 800ms * 2 */
	{1064, 2, 3},			/* 266ms * 4 */
	{800, 0, 1},			/* 800ms * 1 */
	{532, 2, 2},			/* 266ms * 2 */
	{520, 4, 3},			/* 130ms * 4 */
	{400, 1, 1},			/* 400ms * 1 */
	{320, 6, 3},			/* 80ms * 4 */
	{266, 2, 1},			/* 200ms * 1 */
	{264, 7, 3},			/* 66ms * 4 */
	{260, 4, 2},			/* 130ms * 2 */
	{200, 3, 1},			/* 200ms * 1 */
	{160, 6, 2},			/* 80ms * 2 */
	{133, 2, 0},			/* 266ms / 2 */
	{132, 7, 2},			/* 66ms * 2 */
	{130, 4, 1},			/* 130ms * 1 */
	{100, 5, 1},			/* 100ms * 1 */
	{80, 6, 1},			/* 80ms * 1 */
	{66, 7, 1},			/* 66ms * 1 */
	{65, 4, 0},			/* 130ms / 2 */
	{50, 5, 0},			/* 100ms / 2 */
	{40, 6, 0},			/* 80ms / 2 */
	{33, 7, 0}			/* 66ms / 2 */
};


static int cm3217_i2c_rd(struct cm3217_inf *inf, __u16 *val)
{
	struct i2c_msg msg[2];
	__u8 buf[2];

	msg[0].addr = CM3217_I2C_ADDR_RD + 1;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = CM3217_I2C_ADDR_RD;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	if (i2c_transfer(inf->i2c->adapter, msg, 2) != 2)
		return -EIO;

	*val = (__u16)((buf[1] << 8) | buf[0]);
	return 0;
}

static int cm3217_i2c_wr(struct cm3217_inf *inf, __u8 cmd1, __u8 cmd2)
{
	struct i2c_msg msg[2];
	__u8 buf[2];

	buf[0] = cmd1;
	buf[1] = cmd2;
	msg[0].addr = CM3217_I2C_ADDR_CMD1_WR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = CM3217_I2C_ADDR_CMD2_WR;
	msg[1].flags = 0;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	if (i2c_transfer(inf->i2c->adapter, msg, 2) != 2)
		return -EIO;

	return 0;
}

static int cm3217_cmd_wr(struct cm3217_inf *inf, __u8 it_t, __u8 fd_it)
{
	__u8 cmd1;
	__u8 cmd2;
	int err;

	cmd1 = (CM3217_HW_CMD1_DFLT);
	if (!inf->enable)
		cmd1 |= (1 << CM3217_HW_CMD1_BIT_SD);
	cmd1 |= (it_t << CM3217_HW_CMD1_BIT_IT_T);
	cmd2 = fd_it << CM3217_HW_CMD2_BIT_FD_IT;
	err = cm3217_i2c_wr(inf, cmd1, cmd2);
	return err;
}

static int cm3217_vreg_dis(struct cm3217_inf *inf, unsigned int i)
{
	int err = 0;

	if (inf->vreg[i].ret && (inf->vreg[i].consumer != NULL)) {
		err = regulator_disable(inf->vreg[i].consumer);
		if (!err)
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		else
			dev_err(&inf->i2c->dev, "%s %s ERR\n",
				__func__, inf->vreg[i].supply);
	}
	inf->vreg[i].ret = 0;
	return err;
}

static int cm3217_vreg_dis_all(struct cm3217_inf *inf)
{
	unsigned int i;
	int err = 0;

	for (i = ARRAY_SIZE(cm3217_vregs); i > 0; i--)
		err |= cm3217_vreg_dis(inf, (i - 1));
	return err;
}

static int cm3217_vreg_en(struct cm3217_inf *inf, unsigned int i)
{
	int err = 0;

	if (!inf->vreg[i].ret && (inf->vreg[i].consumer != NULL)) {
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

static int cm3217_vreg_en_all(struct cm3217_inf *inf)
{
	unsigned i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(cm3217_vregs); i++)
		err |= cm3217_vreg_en(inf, i);
	return err;
}

static void cm3217_vreg_exit(struct cm3217_inf *inf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cm3217_vregs); i++) {
		regulator_put(inf->vreg[i].consumer);
		inf->vreg[i].consumer = NULL;
	}
}

static int cm3217_vreg_init(struct cm3217_inf *inf)
{
	unsigned int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(cm3217_vregs); i++) {
		inf->vreg[i].supply = cm3217_vregs[i];
		inf->vreg[i].ret = 0;
		inf->vreg[i].consumer = regulator_get(&inf->i2c->dev,
						      inf->vreg[i].supply);
		if (IS_ERR(inf->vreg[i].consumer)) {
			err = PTR_ERR(inf->vreg[i].consumer);
			dev_err(&inf->i2c->dev, "%s err %d for %s\n",
				__func__, err, inf->vreg[i].supply);
			inf->vreg[i].consumer = NULL;
		}
	}
	return err;
}

static int cm3217_pm(struct cm3217_inf *inf, bool en)
{
	int err = 0;

	mutex_lock(&inf->mutex);
	if (inf->enable != en) {
		inf->enable = en;
		if (en) {
			cm3217_vreg_en_all(inf);
		} else {
			err = cm3217_vreg_en_all(inf);
			if (err)
				mdelay(CM3217_HW_DELAY);
			err = cm3217_cmd_wr(inf, 0, 0);
			cm3217_vreg_dis_all(inf);
		}
	}
	dev_dbg(&inf->i2c->dev, "%s enable=%x err=%d\n",
		__func__, inf->enable, err);
	mutex_unlock(&inf->mutex);
	return err;
}

static void pm_exit(struct cm3217_inf *inf)
{
	cm3217_vreg_dis_all(inf);
	cm3217_vreg_exit(inf);
}

static int pm_init(struct cm3217_inf *inf)
{
	int err;

	cm3217_vreg_init(inf);
	inf->enable = true;
	err = cm3217_pm(inf, false);
	return err;
}

static s64 cm3217_timestamp_ns(void)
{
	struct timespec ts;
	s64 ns;

	ktime_get_ts(&ts);
	ns = timespec_to_ns(&ts);
	return ns;
}

static void cm3217_delay(struct cm3217_inf *inf, bool hw_sync)
{
	unsigned int ms;

	if (inf->poll_delay < CM3217_POLL_DELAY_MS_MIN)
		inf->poll_delay = CM3217_POLL_DELAY_MS_MIN;
	if (hw_sync)
		inf->hw_sync = true;
	ms = cm3217_it_tbl[inf->index].ms;
	ms += CM3217_HW_DELAY;
	if ((ms < inf->poll_delay) && !hw_sync)
		inf->queue_delay = inf->poll_delay;
	else
		/* we're either outside the HW integration time (IT) window and
		 * want to get within the window as fast as HW allows us
		 * (hw_sync = true)
		 * OR
		 * HW IT is not as fast as requested polling time
		 */
		inf->queue_delay = ms;
	dev_dbg(&inf->i2c->dev, "%s queue delay=%ums\n",
		__func__, inf->queue_delay);
}

static int cm3217_it_wr(struct cm3217_inf *inf, unsigned int ms)
{
	unsigned int i;
	int err;

	/* get the HW settings for integration time (IT) ms */
	for (i = 0; i < ARRAY_SIZE(cm3217_it_tbl); i++) {
		if (ms >= cm3217_it_tbl[i].ms)
			break;
	}
	if (i >= ARRAY_SIZE(cm3217_it_tbl))
		i = (ARRAY_SIZE(cm3217_it_tbl) - 1);
	err = cm3217_cmd_wr(inf, cm3217_it_tbl[i].it_t,
			    cm3217_it_tbl[i].fd_it);
	if (!err) {
		inf->hw_change = true;
		inf->index = i;
		ms = cm3217_it_tbl[i].ms;
		inf->mult = CM3217_RESOLUTION_STEP / (long)ms;
		inf->lux = 0;
	}
	dev_dbg(&inf->i2c->dev, "%s IT=%u err=%d\n",
		__func__, cm3217_it_tbl[i].ms, err);
	return err;
}

static int cm3217_rd(struct cm3217_inf *inf)
{
	__u16 step;
	s64 timestamp;
	unsigned long calc;
	unsigned int ms;
	bool report;
	int lux;
	int limit_hi;
	int limit_lo;
	int err;

	if ((inf->hw_change) || !inf->enable) {
		inf->hw_change = false;
		/* drop first sample or !enable */
		return 0;
	}

	err = cm3217_i2c_rd(inf, &step);
	if (err)
		return err;

	calc = (unsigned long)step;
	calc *= inf->mult;
	calc /= CM3217_RESOLUTION_DIVIDER;
	lux = (int)calc;
	if ((step == 0xFFFF) && (inf->index <
				 (ARRAY_SIZE(cm3217_it_tbl) - 1))) {
		/* too many photons - need to decrease integration time */
		ms = cm3217_it_tbl[inf->index + 1].ms;
		err = cm3217_it_wr(inf, ms);
		if (!err)
			cm3217_delay(inf, true);
	} else if ((lux <= CM3217_INPUT_LUX_DIVISOR) && (inf->index > 0)) {
		/* not enough photons - need to increase integration time */
		ms = cm3217_it_tbl[inf->index - 1].ms;
		err = cm3217_it_wr(inf, ms);
		if (!err)
			cm3217_delay(inf, true);
	} else if (inf->hw_sync) {
		/* adjust queue time to max(polling delay, HW IT) */
		inf->hw_sync = false;
		cm3217_delay(inf, false);
	}
	timestamp = cm3217_timestamp_ns();
	if (inf->report) {
		ms = 0;
	} else {
		ms = (unsigned int)(timestamp - inf->timestamp);
		ms /= 1000000;
	}
	/* Use of resolution:
	 * - if resolution == 0 then report every polling delay.
	 * - if resolution == 1 then report every polling delay
	 *      only when lux changes.
	 * - if resolution > 1 then report every polling delay
	 *      only when lux is outside the resolution window.
	 */
	if (inf->resolution) {
		limit_hi = inf->lux;
		limit_lo = inf->lux;
		limit_hi += (inf->resolution / 2);
		limit_lo -= (inf->resolution / 2);
		if (limit_lo < 0)
			limit_lo = 0;
		if ((lux > limit_hi) || (lux < limit_lo))
			report = true;
		else
			report = false;
	} else {
		report = true;
	}
	/* report if:
	 * - inf->report (usually used to report the first sample regardless)
	 * - time since last report >= polling delay &&
	 *    - lux outside resolution window from last reported lux
	 *    OR
	 *    - lux on every polling delay regardless of change
	 */
	if (inf->report || (report && (ms >= inf->poll_delay))) {
		inf->lux = lux;
		inf->timestamp = timestamp;
		input_report_abs(inf->idev, ABS_MISC, lux);
		input_sync(inf->idev);
		inf->report = false;
		dev_dbg(&inf->i2c->dev, "%s elapsed=%ums hw=%u lux=%d\n",
			__func__, ms, step, lux);
	}
	return err;
}

static void cm3217_work(struct work_struct *ws)
{
	struct cm3217_inf *inf;

	inf = container_of(ws, struct cm3217_inf, dw.work);
	cm3217_rd(inf);
	if (inf->enable) {
		queue_delayed_work(inf->wq, &inf->dw,
				   msecs_to_jiffies(inf->queue_delay));
	}
}

static int cm3217_enable(struct cm3217_inf *inf, bool en)
{
	int err;

	if (en) {
		cm3217_pm(inf, true);
		inf->index = 0;
		inf->report = true;
		err = cm3217_it_wr(inf, inf->poll_delay);
		cm3217_delay(inf, true);
		queue_delayed_work(inf->wq, &inf->dw, CM3217_HW_DELAY);
	} else {
		cancel_delayed_work_sync(&inf->dw);
		err = cm3217_pm(inf, false);
		inf->poll_delay = CM3217_POLL_DELAY_MS_DFLT;
	}
	return err;
}

static ssize_t cm3217_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;
	unsigned int enable = 0;

	inf = dev_get_drvdata(dev);
	if (inf->enable)
		enable = 1;
	return sprintf(buf, "%u\n", enable);
}

static ssize_t cm3217_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct cm3217_inf *inf;
	unsigned long enable;
	bool en;
	int err;

	inf = dev_get_drvdata(dev);
	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		en = true;
	else
		en = false;
	if (en == inf->enable)
		return count;

	err = cm3217_enable(inf, en);
	if (err)
		return err;

	return count;
}

static ssize_t cm3217_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", (CM3217_POLL_DELAY_MS_MIN * 1000));
}

static ssize_t cm3217_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cm3217_inf *inf;
	unsigned int delay;

	inf = dev_get_drvdata(dev);
	if (kstrtouint(buf, 10, &delay))
		return -EINVAL;

	/* us => ms */
	delay /= 1000;
	if (delay == inf->poll_delay)
		return count;

	inf->poll_delay = delay;
	cm3217_delay(inf, false);
	return count;
}

static ssize_t cm3217_resolution_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", CM3217_RESOLUTION);
}

static ssize_t cm3217_resolution_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct cm3217_inf *inf;
	unsigned int resolution;

	inf = dev_get_drvdata(dev);
	if (kstrtouint(buf, 10, &resolution))
		return -EINVAL;

	inf->resolution = resolution;
	return count;
}

static ssize_t cm3217_divisor_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", CM3217_INPUT_LUX_DIVISOR);
}

static ssize_t cm3217_max_range_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (CM3217_INPUT_LUX_MAX *
				     CM3217_INPUT_LUX_DIVISOR));
}

static ssize_t cm3217_microamp_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", CM3217_POWER_UA);
}

static ssize_t cm3217_lux_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct cm3217_inf *inf;

	inf = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", inf->lux);
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWOTH,
		   cm3217_enable_show, cm3217_enable_store);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR | S_IWOTH,
		   cm3217_delay_show, cm3217_delay_store);
static DEVICE_ATTR(resolution, S_IRUGO | S_IWUSR | S_IWOTH,
		   cm3217_resolution_show, cm3217_resolution_store);
static DEVICE_ATTR(divisor, S_IRUGO,
		   cm3217_divisor_show, NULL);
static DEVICE_ATTR(max_range, S_IRUGO,
		   cm3217_max_range_show, NULL);
static DEVICE_ATTR(microamp, S_IRUGO,
		   cm3217_microamp_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO,
		   cm3217_lux_show, NULL);

static struct attribute *cm3217_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_resolution.attr,
	&dev_attr_divisor.attr,
	&dev_attr_max_range.attr,
	&dev_attr_microamp.attr,
	&dev_attr_lux.attr,
	NULL
};

static struct attribute_group cm3217_attr_group = {
	.name = "cm3217",
	.attrs = cm3217_attrs
};

static int cm3217_sysfs_create(struct cm3217_inf *inf)
{
	int err;

	err = sysfs_create_group(&inf->idev->dev.kobj, &cm3217_attr_group);
	return err;
}

static void cm3217_input_close(struct input_dev *idev)
{
	struct cm3217_inf *inf;

	inf = input_get_drvdata(idev);
	cm3217_enable(inf, false);
}

static int cm3217_input_create(struct cm3217_inf *inf)
{
	int err;

	inf->idev = input_allocate_device();
	if (!inf->idev) {
		dev_err(inf->dev, "%s ERR\n", __func__);
		return -ENOMEM;
	}

	inf->idev->name = CM3217_NAME;
	inf->idev->dev.parent = &inf->i2c->dev;
	inf->idev->close = cm3217_input_close;
	input_set_drvdata(inf->idev, inf);
	input_set_capability(inf->idev, EV_ABS, ABS_MISC);
	input_set_abs_params(inf->idev, ABS_MISC,
			     CM3217_INPUT_LUX_MIN, CM3217_INPUT_LUX_MAX,
			     CM3217_INPUT_LUX_FUZZ, CM3217_INPUT_LUX_FLAT);
	err = input_register_device(inf->idev);
	if (err)
		input_free_device(inf->idev);
	return err;
}

static int cm3217_remove(struct i2c_client *client)
{
	struct cm3217_inf *inf;

	inf = i2c_get_clientdata(client);
	input_unregister_device(inf->idev);
	input_free_device(inf->idev);
	destroy_workqueue(inf->wq);
	pm_exit(inf);
	mutex_destroy(&inf->mutex);
	kfree(inf);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	return 0;
}

static void cm3217_shutdown(struct i2c_client *client)
{
	cm3217_remove(client);
}

static int cm3217_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct cm3217_inf *inf;
	int err;

	inf = kzalloc(sizeof(*inf), GFP_KERNEL);
	if (IS_ERR_OR_NULL(inf)) {
		dev_err(&client->dev, "%s kzalloc err\n", __func__);
		return -ENOMEM;
	}

	inf->i2c = client;
	i2c_set_clientdata(client, inf);
	mutex_init(&inf->mutex);
	err = cm3217_input_create(inf);
	if (err)
		goto err_mutex;

	inf->wq = create_singlethread_workqueue(CM3217_NAME);
	if (!inf->wq) {
		dev_err(&client->dev, "%s workqueue err\n", __func__);
		err = -ENOMEM;
		goto err_wq;
	}

	INIT_DELAYED_WORK(&inf->dw, cm3217_work);
	pm_init(inf);
	err = cm3217_sysfs_create(inf);
	if (err)
		goto err_pm;

	dev_dbg(&client->dev, "%s\n", __func__);
	return 0;

err_pm:
	pm_exit(inf);
err_wq:
	destroy_workqueue(inf->wq);
err_mutex:
	mutex_destroy(&inf->mutex);
	kfree(inf);
	dev_err(&client->dev, "%s err=%d\n", __func__, err);
	return err;
}

static const struct i2c_device_id cm3217_i2c_device_id[] = {
	{"cm3217", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cm3217_i2c_device_id);

#ifdef CONFIG_OF
static const struct of_device_id cm3217_of_match[] = {
	{ .compatible = "capella,cm3217", },
	{ },
};
MODULE_DEVICE_TABLE(of, cm3217_of_match);
#endif

static struct i2c_driver cm3217_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= cm3217_probe,
	.remove		= cm3217_remove,
	.id_table	= cm3217_i2c_device_id,
	.driver = {
		.name	= "cm3217",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(cm3217_of_match),
	},
	.shutdown	= cm3217_shutdown,
};
module_i2c_driver(cm3217_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3217 driver");
MODULE_AUTHOR("NVIDIA Corp");

