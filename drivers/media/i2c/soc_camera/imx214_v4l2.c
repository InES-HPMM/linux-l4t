/*
 * imx214.c - imx214 sensor driver
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/videodev2.h>

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <mach/io_dpd.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <media/imx214.h>

#define CAM_RSTN	148	/* TEGRA_GPIO_PS4 */
#define CAM1_PWDN	151	/* TEGRA_GPIO_PS7 */
#define CAM_AF_PWDN	149	/* TEGRA_GPIO_PS5 */

static struct tegra_io_dpd csia_io = {
	.name			= "CSIA",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 0,
};

static struct tegra_io_dpd csib_io = {
	.name			= "CSIB",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 1,
};

struct imx214_reg {
	u16 addr;
	u8 val;
};

struct imx214_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct imx214 {
	struct v4l2_subdev		subdev;
	const struct imx214_datafmt	*fmt;

	int				mode;
	struct imx214_power_rail	power;
	struct imx214_sensordata	sensor_data;
	struct i2c_client		*i2c_client;
	struct imx214_platform_data	*pdata;
	struct clk			*mclk;
	struct dentry			*debugdir;
};

static const struct imx214_datafmt imx214_colour_fmts[] = {
	{V4L2_MBUS_FMT_SRGGB10_1X10, V4L2_COLORSPACE_SRGB},
	{V4L2_MBUS_FMT_SRGGB8_1X8, V4L2_COLORSPACE_SRGB},
};

static inline struct imx214 *to_imx214(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx214, subdev);
}

#define IMX214_TABLE_WAIT_MS 0
#define IMX214_TABLE_END 1
#define IMX214_MAX_RETRIES 3
#define IMX214_WAIT_MS 10

#define MAX_BUFFER_SIZE 32
#define IMX214_FRAME_LENGTH_ADDR_MSB 0x0340
#define IMX214_FRAME_LENGTH_ADDR_LSB 0x0341
#define IMX214_COARSE_TIME_ADDR_MSB 0x0202
#define IMX214_COARSE_TIME_ADDR_LSB 0x0203
#define IMX214_COARSE_TIME_SHORT_ADDR_MSB 0x0230
#define IMX214_COARSE_TIME_SHORT_ADDR_LSB 0x0231
#define IMX214_GAIN_ADDR 0x0205
#define IMX214_GAIN_SHORT_ADDR 0x0233

static struct imx214_reg mode_4096x3072[] = {
	{0x0114, 0x03},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0222, 0x01},
	{0x0340, 0x0C},
	{0x0341, 0x7A},
	{0x0342, 0x13},
	{0x0343, 0x90},
	{0x0344, 0x00},
	{0x0345, 0x38},
	{0x0346, 0x00},
	{0x0347, 0x18},
	{0x0348, 0x10},
	{0x0349, 0x37},
	{0x034A, 0x0C},
	{0x034B, 0x17},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0902, 0x00},
	{0x3000, 0x35},
	{0x3054, 0x01},
	{0x305C, 0x11},

	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x034C, 0x10},
	{0x034D, 0x00},
	{0x034E, 0x0C},
	{0x034F, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},
	{0x040D, 0x00},
	{0x040E, 0x0C},
	{0x040F, 0x00},

	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x96},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x0310, 0x00},

	{0x0820, 0x12},
	{0x0821, 0xC0},
	{0x0822, 0x00},
	{0x0823, 0x00},

	{0x3A03, 0x09},
	{0x3A04, 0x60},
	{0x3A05, 0x01},

	{0x0B06, 0x01},
	{0x30A2, 0x00},

	{0x30B4, 0x00},

	{0x3A02, 0xFF},

	{0x3011, 0x00},
	{0x3013, 0x01},

	{0x0202, 0x0C},
	{0x0203, 0x70},
	{0x0224, 0x01},
	{0x0225, 0xF4},

	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},

	{0x4170, 0x00},
	{0x4171, 0x10},
	{0x4176, 0x00},
	{0x4177, 0x3C},
	{0xAE20, 0x04},
	{0xAE21, 0x5C},

	{IMX214_TABLE_WAIT_MS, IMX214_WAIT_MS},
	{0x0138, 0x01},

	/* stream on */
	{0x0100, 0x01},
	{IMX214_TABLE_END, 0x00}
};

static struct imx214_reg mode_1920x1080[] = {
	{0x0114, 0x03},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0222, 0x01},
	{0x0340, 0x0C},
	{0x0341, 0x7A},
	{0x0342, 0x13},
	{0x0343, 0x90},
	{0x0344, 0x04},
	{0x0345, 0x78},
	{0x0346, 0x03},
	{0x0347, 0xFC},
	{0x0348, 0x0B},
	{0x0349, 0xF7},
	{0x034A, 0x08},
	{0x034B, 0x33},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0902, 0x00},
	{0x3000, 0x35},
	{0x3054, 0x01},
	{0x305C, 0x11},

	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x034C, 0x07},
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x07},
	{0x040D, 0x80},
	{0x040E, 0x04},
	{0x040F, 0x38},

	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x96},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x0310, 0x00},

	{0x0820, 0x12},
	{0x0821, 0xC0},
	{0x0822, 0x00},
	{0x0823, 0x00},

	{0x3A03, 0x04},
	{0x3A04, 0xF8},
	{0x3A05, 0x02},

	{0x0B06, 0x01},
	{0x30A2, 0x00},

	{0x30B4, 0x00},

	{0x3A02, 0xFF},

	{0x3011, 0x00},
	{0x3013, 0x01},

	{0x0202, 0x0C},
	{0x0203, 0x70},
	{0x0224, 0x01},
	{0x0225, 0xF4},

	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},

	{0x4170, 0x00},
	{0x4171, 0x10},
	{0x4176, 0x00},
	{0x4177, 0x3C},
	{0xAE20, 0x04},
	{0xAE21, 0x5C},

	{IMX214_TABLE_WAIT_MS, IMX214_WAIT_MS},
	{0x0138, 0x01},

	/* stream on */
	{0x0100, 0x01},
	{IMX214_TABLE_END, 0x00}
};


static struct imx214_reg tp_colorbars[] = {
	{0x0600, 0x00},
	{0x0601, 0x02},

	{IMX214_TABLE_WAIT_MS, IMX214_WAIT_MS},
	{IMX214_TABLE_END, 0x00}
};

enum {
	IMX214_MODE_4096X3072,
	IMX214_MODE_1920X1080,
	IMX214_MODE_INVALID
};

static struct imx214_reg *mode_table[] = {
	[IMX214_MODE_4096X3072] = mode_4096x3072,
	[IMX214_MODE_1920X1080] = mode_1920x1080,
};

static int test_mode;
module_param(test_mode, int, 0644);

static const struct v4l2_frmsize_discrete imx214_frmsizes[] = {
	{4096, 3072},
	{1920, 1080},
};

/* Find a data format by a pixel code in an array */
static const struct imx214_datafmt *imx214_find_datafmt(
		enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(imx214_colour_fmts); i++)
		if (imx214_colour_fmts[i].code == code)
			return imx214_colour_fmts + i;

	return NULL;
}

#define IMX214_MODE	IMX214_MODE_4096X3072
#define IMX214_WIDTH	4096
#define IMX214_HEIGHT	3072

static int imx214_find_mode(struct v4l2_subdev *sd,
			    u32 width, u32 height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(imx214_frmsizes); i++) {
		if (width == imx214_frmsizes[i].width &&
		    height == imx214_frmsizes[i].height)
			return i;
	}

	dev_err(sd->v4l2_dev->dev, "%dx%d is not supported\n", width, height);
	return IMX214_MODE_4096X3072;
}

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static int imx214_read_reg(struct i2c_client *client, u16 addr, u8 *val)
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

static int imx214_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx214_write_table(struct i2c_client *client,
			      const struct imx214_reg table[])
{
	int err;
	const struct imx214_reg *next;
	u16 val;

	for (next = table; next->addr != IMX214_TABLE_END; next++) {
		if (next->addr == IMX214_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		err = imx214_write_reg(client, next->addr, val);
		if (err) {
			pr_err("%s:imx214_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static void imx214_mclk_disable(struct imx214 *priv)
{
	return;
	dev_dbg(&priv->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(priv->mclk);
}

static int imx214_mclk_enable(struct imx214 *priv)
{
	int err;
	unsigned long mclk_init_rate = 24000000;

	dev_dbg(&priv->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(priv->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(priv->mclk);
	return err;
}


static int imx214_debugfs_show(struct seq_file *s, void *unused)
{
	struct imx214 *dev = s->private;

	dev_dbg(&dev->i2c_client->dev, "%s: ++\n", __func__);

	return 0;
}

static ssize_t imx214_debugfs_write(
	struct file *file,
	char const __user *buf,
	size_t count,
	loff_t *offset)
{
	struct imx214 *dev =
			((struct seq_file *)file->private_data)->private;
	struct i2c_client *i2c_client = dev->i2c_client;
	int ret = 0;
	char buffer[MAX_BUFFER_SIZE];
	u32 address;
	u32 data;
	u8 readback;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	if (copy_from_user(&buffer, buf, sizeof(buffer)))
		goto debugfs_write_fail;

	if (sscanf(buf, "0x%x 0x%x", &address, &data) == 2)
		goto set_attr;
	if (sscanf(buf, "0X%x 0X%x", &address, &data) == 2)
		goto set_attr;
	if (sscanf(buf, "%d %d", &address, &data) == 2)
		goto set_attr;

	if (sscanf(buf, "0x%x 0x%x", &address, &data) == 1)
		goto read;
	if (sscanf(buf, "0X%x 0X%x", &address, &data) == 1)
		goto read;
	if (sscanf(buf, "%d %d", &address, &data) == 1)
		goto read;

	dev_err(&i2c_client->dev, "SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	dev_info(&i2c_client->dev,
			"new address = %x, data = %x\n", address, data);
	ret |= imx214_write_reg(i2c_client, address, data);
read:
	ret |= imx214_read_reg(i2c_client, address, &readback);
	dev_info(&i2c_client->dev,
			"wrote to address 0x%x with value 0x%x\n",
			address, readback);

	if (ret)
		goto debugfs_write_fail;

	return count;

debugfs_write_fail:
	dev_err(&i2c_client->dev,
			"%s: test pattern write failed\n", __func__);
	return -EFAULT;
}

static int imx214_debugfs_open(struct inode *inode, struct file *file)
{
	struct imx214 *dev = inode->i_private;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	return single_open(file, imx214_debugfs_show, inode->i_private);
}

static const struct file_operations imx214_debugfs_fops = {
	.open		= imx214_debugfs_open,
	.read		= seq_read,
	.write		= imx214_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void imx214_remove_debugfs(struct imx214 *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	debugfs_remove_recursive(dev->debugdir);
	dev->debugdir = NULL;
}

static void imx214_create_debugfs(struct imx214 *dev)
{
	struct dentry *ret;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s\n", __func__);

	dev->debugdir =
		debugfs_create_dir("imx214", NULL);
	if (!dev->debugdir)
		goto remove_debugfs;

	ret = debugfs_create_file("d",
				S_IWUSR | S_IRUGO,
				dev->debugdir, dev,
				&imx214_debugfs_fops);
	if (!ret)
		goto remove_debugfs;

	return;
remove_debugfs:
	dev_err(&i2c_client->dev, "couldn't create debugfs\n");
	imx214_remove_debugfs(dev);
}

static int imx214_power_on(struct imx214_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd || !pw->dvdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for ardbeg */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 0);
	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx214_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx214_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM_RSTN, 1);
	gpio_set_value(CAM1_PWDN, 1);
	gpio_set_value(CAM_AF_PWDN, 1);

	usleep_range(300, 310);

	return 0;

imx214_iovdd_fail:
	regulator_disable(pw->avdd);

imx214_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int imx214_power_off(struct imx214_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd))) {
		tegra_io_dpd_enable(&csia_io);
		tegra_io_dpd_enable(&csib_io);
		return -EFAULT;
	}

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 0);
	gpio_set_value(CAM1_PWDN, 0);

	/* put CSIA/B IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	return 0;
}

static int imx214_power_put(struct imx214_power_rail *pw)
{
	if (unlikely(!pw))
		return -EFAULT;

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;
	pw->ext_reg1 = NULL;
	pw->ext_reg2 = NULL;

	return 0;
}

static int imx214_power_get(struct imx214 *priv)
{
	struct imx214_power_rail *pw = &priv->power;
	int err;

	/* ananlog 2.7v */
	pw->avdd = devm_regulator_get(&priv->i2c_client->dev, "vana");
	if (IS_ERR(pw->avdd)) {
		err = PTR_ERR(pw->avdd);
		pw->avdd = NULL;
		dev_err(&priv->i2c_client->dev, "Failed to get regulator vana\n");
		return err;
	}

	/* digital 1.2v */
	pw->dvdd = devm_regulator_get(&priv->i2c_client->dev, "vdig");
	if (IS_ERR(pw->dvdd)) {
		err = PTR_ERR(pw->dvdd);
		pw->dvdd = NULL;
		dev_err(&priv->i2c_client->dev, "Failed to get regulator vdig\n");
		return err;
	}

	/* IO 1.8v */
	pw->iovdd = devm_regulator_get(&priv->i2c_client->dev, "vif");
	if (IS_ERR(pw->iovdd)) {
		err = PTR_ERR(pw->iovdd);
		pw->iovdd = NULL;
		dev_err(&priv->i2c_client->dev, "Failed to get regulator vif\n");
		return err;
	}

	return 0;
}

static int imx214_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx214 *priv = to_imx214(client);
	int mode = imx214_find_mode(sd, mf->width, mf->height);

	mf->width = imx214_frmsizes[mode].width;
	mf->height = imx214_frmsizes[mode].height;

	if (mf->code != V4L2_MBUS_FMT_SRGGB8_1X8 &&
	    mf->code != V4L2_MBUS_FMT_SRGGB10_1X10)
		mf->code = V4L2_MBUS_FMT_SRGGB10_1X10;

	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	priv->mode = mode;

	return 0;
}

static int imx214_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx214 *priv = to_imx214(client);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	/* MIPI CSI could have changed the format, double-check */
	if (!imx214_find_datafmt(mf->code))
		return -EINVAL;

	imx214_try_fmt(sd, mf);

	priv->fmt = imx214_find_datafmt(mf->code);

	return 0;
}

static int imx214_g_fmt(struct v4l2_subdev *sd,	struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx214 *priv = to_imx214(client);

	const struct imx214_datafmt *fmt = priv->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= IMX214_WIDTH;
	mf->height	= IMX214_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int imx214_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rect->top	= 0;
	rect->left	= 0;
	rect->width	= IMX214_WIDTH;
	rect->height	= IMX214_HEIGHT;

	return 0;
}

static int imx214_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= IMX214_WIDTH;
	a->bounds.height		= IMX214_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int imx214_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if ((unsigned int)index >= ARRAY_SIZE(imx214_colour_fmts))
		return -EINVAL;

	*code = imx214_colour_fmts[index].code;
	return 0;
}

static int imx214_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx214 *priv = to_imx214(client);
	int err = 0;

	if (!enable)
		return 0;

	err = imx214_write_table(priv->i2c_client, mode_table[priv->mode]);
	if (err)
		return err;

	if (test_mode)
		imx214_write_table(priv->i2c_client, tp_colorbars);

	return err;
}

static int imx214_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= V4L2_IDENT_IMX214;
	id->revision	= 0;

	return 0;
}

static int imx214_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx214 *priv = to_imx214(client);
	int err;

	if (on) {
		err = imx214_mclk_enable(priv);
		if (!err)
			err = imx214_power_on(&priv->power);
		if (err < 0)
			imx214_mclk_disable(priv);
		return err;
	} else if (!on) {
		imx214_power_off(&priv->power);
		imx214_mclk_disable(priv);
		return 0;
	} else
		return -EINVAL;
}

static int imx214_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static struct v4l2_subdev_video_ops imx214_subdev_video_ops = {
	.s_stream	= imx214_s_stream,
	.s_mbus_fmt	= imx214_s_fmt,
	.g_mbus_fmt	= imx214_g_fmt,
	.try_mbus_fmt	= imx214_try_fmt,
	.enum_mbus_fmt	= imx214_enum_fmt,
	.g_crop		= imx214_g_crop,
	.cropcap	= imx214_cropcap,
	.g_mbus_config	= imx214_g_mbus_config,
};

static struct v4l2_subdev_core_ops imx214_subdev_core_ops = {
	.g_chip_ident	= imx214_g_chip_ident,
	.s_power	= imx214_s_power,
};

static struct v4l2_subdev_ops imx214_subdev_ops = {
	.core	= &imx214_subdev_core_ops,
	.video	= &imx214_subdev_video_ops,
};

static struct of_device_id imx214_of_match[] = {
	{ .compatible = "nvidia,imx214", },
	{ },
};

MODULE_DEVICE_TABLE(of, imx214_of_match);

static struct imx214_platform_data *imx214_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct imx214_platform_data *board_priv_pdata;
	const struct of_device_id *match;

	match = of_match_device(imx214_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev, sizeof(*board_priv_pdata),
			GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_priv_pdata->cam1_gpio = of_get_named_gpio(np, "cam1-gpios", 0);
	board_priv_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	board_priv_pdata->af_gpio = of_get_named_gpio(np, "af-gpios", 0);

	board_priv_pdata->ext_reg = of_property_read_bool(np, "nvidia,ext_reg");

	board_priv_pdata->power_on = imx214_power_on;
	board_priv_pdata->power_off = imx214_power_off;

	return board_priv_pdata;
}

static int imx214_get_sensor_id(struct imx214 *priv)
{
	int i;
	u8 bak = 0, fuse_id[16];

	imx214_mclk_enable(priv);
	imx214_power_on(&priv->power);

	imx214_write_reg(priv->i2c_client, 0x3B02, 0x00);
	imx214_write_reg(priv->i2c_client, 0x3B00, 0x01);

	for (i = 0; i < 9; i++) {
		imx214_read_reg(priv->i2c_client, 0x3B24 + i, &bak);
		fuse_id[i] = bak;
	}
	imx214_power_off(&priv->power);
	imx214_mclk_disable(priv);

	return 0;
}

static int imx214_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct imx214 *priv;
	const char *mclk_name;
	int err;

	priv = devm_kzalloc(&client->dev,
			sizeof(struct imx214), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	if (client->dev.of_node)
		priv->pdata = imx214_parse_dt(client);
	else
		priv->pdata = client->dev.platform_data;

	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	priv->i2c_client = client;
	priv->mode = IMX214_MODE;
	priv->fmt = &imx214_colour_fmts[0];

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	priv->mclk = devm_clk_get(&client->dev, mclk_name);
	if (IS_ERR(priv->mclk)) {
		dev_err(&client->dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(priv->mclk);
	}

	err = imx214_power_get(priv);
	if (err)
		return err;

	i2c_set_clientdata(client, priv);

	err = imx214_get_sensor_id(priv);
	if (err) {
		dev_err(&client->dev, "unable to get sensor id\n");
		return err;
	}

	imx214_create_debugfs(priv);

	v4l2_i2c_subdev_init(&priv->subdev, client, &imx214_subdev_ops);

	return 0;
}

static int
imx214_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd;
	struct imx214 *priv;

	ssdd = soc_camera_i2c_to_desc(client);
	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	priv = i2c_get_clientdata(client);
	imx214_power_put(&priv->power);
	imx214_remove_debugfs(priv);

	return 0;
}

static const struct i2c_device_id imx214_id[] = {
	{ "imx214_v4l2", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx214_id);

static struct i2c_driver imx214_i2c_driver = {
	.driver = {
		.name = "imx214_v4l2",
		.owner = THIS_MODULE,
	},
	.probe = imx214_probe,
	.remove = imx214_remove,
	.id_table = imx214_id,
};

module_i2c_driver(imx214_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Sony IMX214");
MODULE_AUTHOR("Bryan Wu <pengw@nvidia.com>");
MODULE_LICENSE("GPL v2");
