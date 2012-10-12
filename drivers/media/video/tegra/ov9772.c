/*
 * ov9772.c - ov9772 sensor driver
 *
 *  * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 *
 * Contributors:
 *	  Krupal Divvela <kdivvela@nvidia.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov9772.h>
#include <linux/gpio.h>
#include <linux/module.h>

struct ov9772_reg {
	u16 addr;
	u16 val;
};

struct ov9772_info {
	int                         mode;
	struct ov9772_sensordata    sensor_data;
	struct i2c_client           *i2c_client;
	struct ov9772_platform_data *pdata;
	atomic_t                    in_use;
};

#define OV9772_TABLE_WAIT_MS		0
#define OV9772_TABLE_END		1
#define OV9772_MAX_RETRIES		3

#define OV9772_WAIT_MS_100		100

static struct ov9772_reg mode_1284x724[] = {
	/* Stand by */
	{0x0103, 0x01},
	{OV9772_TABLE_WAIT_MS, OV9772_WAIT_MS_100},

	{0x0200, 0x00},
	{0x0201, 0x00},

	{0x0301, 0x0a},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0307, 0x3c},

	{0x0340, 0x02},
	{0x0341, 0xf8},
	{0x0342, 0x06},
	{0x0343, 0x2a},
	{0x034c, 0x05},
	{0x034d, 0x04},
	{0x034e, 0x02},
	{0x034f, 0xd4},

	{0x300c, 0x22},
	{0x300d, 0x1e},
	{0x300e, 0xc2},
	{0x3010, 0x81},
	{0x3012, 0x70},
	{0x3014, 0x0d},
	{0x3022, 0x20},
	{0x3025, 0x03},
	{0x303c, 0x23},

	{0x3103, 0x01},
	{0x3104, 0x00},

	{0x3503, 0x14},

	{0x3602, 0xc0},
	{0x3611, 0x10},
	{0x3613, 0x83},
	{0x3620, 0x24},
	{0x3622, 0x2c},
	{0x3631, 0xc2},
	{0x3634, 0x04},

	{0x3708, 0x24},
	{0x3709, 0x10},
	{0x370e, 0x00},
	{0x371b, 0x60},
	{0x3724, 0x1c},
	{0x372c, 0x00},
	{0x372d, 0x00},
	{0x3745, 0x00},
	{0x3746, 0x18},

	{0x3811, 0x0e},
	{0x3813, 0x08},

	{0x3a0c, 0x20},

	{0x3b01, 0x32},
	{0x3b02, 0xa4},

	{0x3c00, 0x00},

	{0x3f00, 0x2a},
	{0x3f01, 0x8c},
	{0x3f0f, 0xf5},

	{0x4000, 0x07},

	{0x4001, 0x02},

	{0x460e, 0xb1},

	{0x4800, 0x44},
	{0x4801, 0x0f},
	{0x4805, 0x10},
	{0x4815, 0x00},
	{0x4837, 0x36},

	{0x5000, 0x06},
	{0x5001, 0x31},
	{0x5005, 0x08},

	{0x5100, 0x00},

	{0x5310, 0x01},
	{0x5311, 0xff},
	{0x53b9, 0x0f},
	{0x53ba, 0x04},
	{0x53bb, 0x4a},
	{0x53bc, 0xd3},
	{0x53bd, 0x41},
	{0x53be, 0x00},
	{0x53c4, 0x03},

	/* Start Streaming */
	{0x0100, 0x01},

	{OV9772_TABLE_WAIT_MS, OV9772_WAIT_MS_100},
	{OV9772_TABLE_END, 0x00}
};

enum {
	ov9772_MODE_1284x724,
};

static struct ov9772_reg *mode_table[] = {
	[ov9772_MODE_1284x724] = mode_1284x724,
};

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void ov9772_get_frame_length_regs(struct ov9772_reg *regs,
						u32 frame_length)
{
	regs->addr = OV9772_REG_FRAME_LENGTH_HI;
	regs->val = (frame_length >> 8) & 0xff;
	regs++;
	regs->addr = OV9772_REG_FRAME_LENGTH_LO;
	regs->val = frame_length & 0xff;
}

static inline void ov9772_get_coarse_time_regs(struct ov9772_reg *regs,
					       u32 coarse_time)
{
	regs->addr = OV9772_REG_COARSE_TIME_HI;
	regs->val = (coarse_time >> 8) & 0xff;
	regs++;
	regs->addr = OV9772_REG_COARSE_TIME_LO;
	regs->val = coarse_time & 0xff;
}

static inline void ov9772_get_gain_reg(struct ov9772_reg *regs, u16 gain)
{
	regs->addr = OV9772_REG_GAIN_HI;
	regs->val  = gain;
}

static int ov9772_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];
	return 0;
}

static int ov9772_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("[ov9772]:%s:i2c transfer failed, retrying %x %x\n",
		       __func__, addr, val);
		msleep_range(3);
	} while (retry <= OV9772_MAX_RETRIES);

	return err;
}

static int ov9772_write_table(struct i2c_client *client,
				 const struct ov9772_reg table[],
				 const struct ov9772_reg override_list[],
				 int num_override_regs)
{
	int err;
	const struct ov9772_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != OV9772_TABLE_END; next++) {
		if (next->addr == OV9772_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = ov9772_write_reg(client, next->addr, val);
		if (err) {
			pr_err("[ov9772]:%s:ov9772_write_table:%d",
			       __func__, err);
			return err;
		}
	}
	return 0;
}

static int ov9772_set_mode(struct ov9772_info *info, struct ov9772_mode *mode)
{
	int sensor_mode;
	int err;
	struct ov9772_reg reg_list[5];

	pr_info("[ov9772]:%s: xres %u yres %u framelength %u coarsetime %u \
		gain %u\n", __func__, mode->xres, mode->yres, mode->frame_length,
			 mode->coarse_time, mode->gain);

	if (mode->xres == 1284 && mode->yres == 724)
		sensor_mode = ov9772_MODE_1284x724;
	else {
		pr_err("[ov9772]:%s: invalid resolution supplied to set \
		mode %d %d\n", __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	ov9772_get_frame_length_regs(reg_list, mode->frame_length);
	ov9772_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	ov9772_get_gain_reg(reg_list + 4, mode->gain);

	err = ov9772_write_table(info->i2c_client, mode_table[sensor_mode],
							 reg_list, 5);
	if (err)
		return err;

	info->mode = sensor_mode;
	pr_info("[ov9772]: stream on.\n");
	return 0;
}

static int ov9772_get_status(struct ov9772_info *info, u8 *dev_status)
{
	*dev_status = 0;
	return 0;
}

static int ov9772_set_frame_length(struct ov9772_info *info, u32 frame_length,
						 bool group_hold)
{
	struct ov9772_reg reg_list[2];
	int i = 0;
	int ret;

	ov9772_get_frame_length_regs(reg_list, frame_length);

	if (group_hold) {
		ret = ov9772_write_reg(info->i2c_client, 0x0104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < 2; i++) {
		ret = ov9772_write_reg(info->i2c_client, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ov9772_write_reg(info->i2c_client, 0x0104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int ov9772_set_coarse_time(struct ov9772_info *info, u32 coarse_time,
						 bool group_hold)
{
	int ret;

	struct ov9772_reg reg_list[2];
	int i = 0;

	ov9772_get_coarse_time_regs(reg_list, coarse_time);

	if (group_hold) {
		ret = ov9772_write_reg(info->i2c_client, 0x104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < 2; i++) {
		ret = ov9772_write_reg(info->i2c_client, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ov9772_write_reg(info->i2c_client, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int ov9772_set_gain(struct ov9772_info *info, u16 gain, bool group_hold)
{
	int ret;
	struct ov9772_reg reg_list;

	ov9772_get_gain_reg(&reg_list, gain);

	if (group_hold) {
		ret = ov9772_write_reg(info->i2c_client, 0x104, 0x1);
		if (ret)
			return ret;
	}

	ret = ov9772_write_reg(info->i2c_client, reg_list.addr, reg_list.val);
	if (ret)
		return ret;

	if (group_hold) {
		ret = ov9772_write_reg(info->i2c_client, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int ov9772_set_group_hold(struct ov9772_info *info, struct ov9772_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = ov9772_write_reg(info->i2c_client, 0x104, 0x1);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ov9772_set_gain(info, ae->gain, false);
	if (ae->coarse_time_enable)
		ov9772_set_coarse_time(info, ae->coarse_time, false);
	if (ae->frame_length_enable)
		ov9772_set_frame_length(info, ae->frame_length, false);

	if (groupHoldEnabled) {
		ret = ov9772_write_reg(info->i2c_client, 0x104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int ov9772_get_sensor_id(struct ov9772_info *info)
{
	int ret = 0;
	int i;
	u8 bak = 0;

	pr_info("%s\n", __func__);
	if (info->sensor_data.fuse_id_size)
		return 0;

	/* Note 1: If the sensor does not have power at this point
	Need to supply the power, e.g. by calling power on function */

	for (i = 0; i < 5; i++) {
		ret |= ov9772_write_reg(info->i2c_client, 0x3d00, i);
		ret |= ov9772_read_reg(info->i2c_client, 0x3d04,
				&bak);
		info->sensor_data.fuse_id[i] = bak;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = i;

	/* Note 2: Need to clean up any action carried out in Note 1 */

	return ret;
}

static long ov9772_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct ov9772_info *info = file->private_data;

	switch (cmd) {
	case OV9772_IOCTL_SET_MODE:
	{
		struct ov9772_mode mode;
		if (copy_from_user(&mode,
			 (const void __user *)arg,
			 sizeof(struct ov9772_mode))) {
			pr_err("[ov9772]:%s:Failed to get mode from user.\n",
			       __func__);
			return -EFAULT;
		}
		return ov9772_set_mode(info, &mode);
	}
	case OV9772_IOCTL_SET_FRAME_LENGTH:
		return ov9772_set_frame_length(info, (u32)arg, true);
	case OV9772_IOCTL_SET_COARSE_TIME:
		return ov9772_set_coarse_time(info, (u32)arg, true);
	case OV9772_IOCTL_SET_GAIN:
		return ov9772_set_gain(info, (u16)arg, true);
	case OV9772_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ov9772_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			pr_err("[ov9772]:%s:Failed to copy status to user.\n",
			       __func__);
			return -EFAULT;
			}
		return 0;
	}
	case OV9772_IOCTL_GET_SENSORDATA:
	{
		err = ov9772_get_sensor_id(info);

		if (err) {
			pr_err("[ov9772]:%s:Failed to get fuse id info.\n",
			       __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg,
				 &info->sensor_data,
				 sizeof(struct ov9772_sensordata))) {
			pr_info("[ov9772]:%s:Failed to copy fuse id to user \
				space\n",__func__);
			return -EFAULT;
		}
		return 0;
	}
	case OV9772_IOCTL_SET_GROUP_HOLD:
	{
	  struct ov9772_ae ae;
	  if (copy_from_user(&ae, (const void __user *)arg,
			sizeof(struct ov9772_ae))) {
		pr_info("[ov9772]:%s:fail group hold\n", __func__);
		return -EFAULT;
	  }
	  return ov9772_set_group_hold(info, &ae);
	}
	default:
	  pr_err("[ov9772]:%s:unknown cmd.\n", __func__);
	  return -EINVAL;
	}
	return 0;
}

static struct ov9772_info *info;

static int ov9772_open(struct inode *inode, struct file *file)
{
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		pr_info("[ov9772]:%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on(&info->i2c_client->dev);
	else{
		pr_err("[ov9772]:%s:no valid power_on function.\n",
		       __func__);
		return -EEXIST;
	}

	return 0;
}

static int ov9772_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off(&info->i2c_client->dev);
	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}


static const struct file_operations ov9772_fileops = {
	.owner = THIS_MODULE,
	.open = ov9772_open,
	.unlocked_ioctl = ov9772_ioctl,
	.release = ov9772_release,
};

static struct miscdevice ov9772_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov9772",
	.fops = &ov9772_fileops,
};

static int ov9772_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("[ov9772]: probing sensor.\n");

	info = kzalloc(sizeof(struct ov9772_info), GFP_KERNEL);
	if (!info) {
		pr_err("[ov9772]:%s:Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	err = misc_register(&ov9772_device);
	if (err) {
		pr_err("[ov9772]:%s:Unable to register misc device!\n",
		       __func__);
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	atomic_set(&info->in_use, 0);
	info->mode = -1;

	i2c_set_clientdata(client, info);

	return 0;
}

static int ov9772_remove(struct i2c_client *client)
{
	struct ov9772_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov9772_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ov9772_id[] = {
	{ "ov9772", 0 },
};

MODULE_DEVICE_TABLE(i2c, ov9772_id);

static struct i2c_driver ov9772_i2c_driver = {
	.driver = {
		.name = "ov9772",
		.owner = THIS_MODULE,
	},
	.probe = ov9772_probe,
	.remove = ov9772_remove,
	.id_table = ov9772_id,
};

static int __init ov9772_init(void)
{
	pr_info("[ov9772] sensor driver loading\n");
	return i2c_add_driver(&ov9772_i2c_driver);
}

static void __exit ov9772_exit(void)
{
	i2c_del_driver(&ov9772_i2c_driver);
}

module_init(ov9772_init);
module_exit(ov9772_exit);
