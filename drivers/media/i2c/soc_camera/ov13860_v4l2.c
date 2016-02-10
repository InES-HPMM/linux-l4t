/*
 * ov13860.c - ov13860 sensor driver
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/camera_common.h>
#include <media/ov13860.h>

#include "ov13860_mode_tbls.h"

#define OV13860_MAX_COARSE_DIFF		8

#define OV13860_GAIN_SHIFT		8
#define OV13860_MIN_GAIN		(1 << OV13860_GAIN_SHIFT)
#define OV13860_MAX_GAIN \
	((15 << OV13860_GAIN_SHIFT) | (1 << (OV13860_GAIN_SHIFT - 1)))
#define OV13860_MIN_FRAME_LENGTH		(0x0DA8)
#define OV13860_MAX_FRAME_LENGTH		(0x7fff)
#define OV13860_MIN_EXPOSURE_COARSE		(0x8)
#define OV13860_MAX_EXPOSURE_COARSE	\
	(OV13860_MAX_FRAME_LENGTH-OV13860_MAX_COARSE_DIFF)

#define OV13860_DEFAULT_GAIN		OV13860_MIN_GAIN
#define OV13860_DEFAULT_FRAME_LENGTH		OV13860_MIN_FRAME_LENGTH
#define OV13860_DEFAULT_EXPOSURE_COARSE	\
	(OV13860_DEFAULT_FRAME_LENGTH-OV13860_MAX_COARSE_DIFF)

#define OV13860_DEFAULT_MODE		OV13860_MODE_4224X3120
#define OV13860_DEFAULT_HDR_MODE		OV13860_MODE_4224X3120_HDR
#define OV13860_DEFAULT_WIDTH		4224
#define OV13860_DEFAULT_HEIGHT		3120
#define OV13860_DEFAULT_DATAFMT		V4L2_MBUS_FMT_SRGGB10_1X10
#define OV13860_DEFAULT_CLK_FREQ		24000000

struct ov13860 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct camera_common_eeprom_data eeprom[OV13860_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[OV13860_EEPROM_SIZE];
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;

	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

u16 ov13860_to_gain(u32 rep, int shift)
{
	u16 gain;
	int gain_int;
	int gain_dec;
	int min_int = (1 << shift);

	if (rep < OV13860_MIN_GAIN)
		rep = OV13860_MIN_GAIN;
	else if (rep > OV13860_MAX_GAIN)
		rep = OV13860_MAX_GAIN;

	/* shift indicates number of least significant bits
	 * used for decimal representation of gain */
	gain_int = (int)(rep >> shift);
	gain_dec = (int)(rep & ~(0xffff << shift));

	/* derived from formulat gain = (x * 16 + 0.5) */
	gain = ((gain_int * min_int + gain_dec) * 32 + min_int) / (2 * min_int);

	return gain;
}

static int ov13860_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int ov13860_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops ov13860_ctrl_ops = {
	.g_volatile_ctrl = ov13860_g_volatile_ctrl,
	.s_ctrl		= ov13860_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV13860_MIN_GAIN,
		.max = OV13860_MAX_GAIN,
		.def = OV13860_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV13860_MIN_FRAME_LENGTH,
		.max = OV13860_MAX_FRAME_LENGTH,
		.def = OV13860_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV13860_MIN_EXPOSURE_COARSE,
		.max = OV13860_MAX_EXPOSURE_COARSE,
		.def = OV13860_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = OV13860_MIN_EXPOSURE_COARSE,
		.max = OV13860_MAX_EXPOSURE_COARSE,
		.def = OV13860_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_EEPROM_DATA,
		.name = "EEPROM Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.min = 0,
		.max = OV13860_EEPROM_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV13860_OTP_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &ov13860_ctrl_ops,
		.id = V4L2_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = OV13860_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};

static inline void ov13860_get_frame_length_regs(ov13860_reg *regs,
				u16 frame_length)
{
	regs->addr = OV13860_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = OV13860_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
	(regs + 2)->addr = OV13860_TABLE_END;
	(regs + 2)->val = 0;
}

static inline void ov13860_get_coarse_time_regs(ov13860_reg *regs,
				u16 coarse_time)
{
	regs->addr = OV13860_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = OV13860_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
	(regs + 2)->addr = OV13860_TABLE_END;
	(regs + 2)->val = 0;
}

static inline void ov13860_get_coarse_time_short_regs(ov13860_reg *regs,
				u16 coarse_time)
{
	regs->addr = OV13860_COARSE_TIME_SHORT_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = OV13860_COARSE_TIME_SHORT_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
	(regs + 2)->addr = OV13860_TABLE_END;
	(regs + 2)->val = 0;
}

static inline void ov13860_get_gain_reg(ov13860_reg *regs,
				u16 gain)
{
	regs->addr = OV13860_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = OV13860_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
	(regs + 2)->addr = OV13860_TABLE_END;
	(regs + 2)->val = 0;
}

static inline void ov13860_get_gain_short_reg(ov13860_reg *regs,
				u16 gain)
{
	regs->addr = OV13860_GAIN_SHORT_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = OV13860_GAIN_SHORT_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int ov13860_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct ov13860 *priv = (struct ov13860 *)s_data->priv;
	return regmap_read(priv->regmap, addr, (unsigned int *) val);
}

static int ov13860_write_reg(struct camera_common_data *s_data,
		u16 addr, u8 val)
{
	int err;
	struct ov13860 *priv = (struct ov13860 *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int ov13860_write_table(struct ov13860 *priv,
				const ov13860_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 OV13860_TABLE_WAIT_MS,
					 OV13860_TABLE_END);
}

static int ov13860_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct ov13860 *priv = (struct ov13860 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/* sleep calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	usleep_range(10, 20);

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto ov13860_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto ov13860_iovdd_fail;

	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);
	if (err)
		goto ov13860_dvdd_fail;

	usleep_range(10, 20);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 1);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);

	usleep_range(2000, 3000);
	pw->state = SWITCH_ON;
	return 0;

ov13860_dvdd_fail:
	regulator_disable(pw->iovdd);

ov13860_iovdd_fail:
	regulator_disable(pw->avdd);

ov13860_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int ov13860_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct ov13860 *priv = (struct ov13860 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (err) {
			pr_err("%s failed.\n", __func__);
			return err;
		} else {
			goto power_off_done;
		}
	}

	/* sleep calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */
	usleep_range(20, 25);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);

	if (pw->dvdd)
		regulator_disable(pw->dvdd);
	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->avdd)
		regulator_disable(pw->avdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int ov13860_power_put(struct ov13860 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;

	return 0;
}

static int ov13860_power_get(struct ov13860 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	/* ananlog 2.7v */
	err |= camera_common_regulator_get(priv->i2c_client,
			&pw->avdd, pdata->regulators.avdd);
	/* digital 1.2v */
	err |= camera_common_regulator_get(priv->i2c_client,
			&pw->dvdd, pdata->regulators.dvdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(priv->i2c_client,
			&pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	pw->state = SWITCH_OFF;
	return err;
}

static int ov13860_set_gain(struct ov13860 *priv, s32 val);
static int ov13860_set_frame_length(struct ov13860 *priv, s32 val);
static int ov13860_set_coarse_time(struct ov13860 *priv, s32 val);
static int ov13860_set_coarse_time_short(struct ov13860 *priv, s32 val);

static int ov13860_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct ov13860 *priv = (struct ov13860 *)s_data->priv;
	struct v4l2_control control;
	int err;

	if (!enable)
		return ov13860_write_table(priv,
			mode_table[OV13860_MODE_STOP_STREAM]);

	err = ov13860_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;

	/* write list of override regs for the asking frame length,
	 * coarse integration time, and gain. Failures to write
	 * overrides are non-fatal. */
	control.id = V4L2_CID_GAIN;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= ov13860_set_gain(priv, control.value);
	if (err)
		dev_dbg(&client->dev, "%s: error gain override\n", __func__);

	control.id = V4L2_CID_FRAME_LENGTH;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= ov13860_set_frame_length(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: error frame length override\n", __func__);

	control.id = V4L2_CID_COARSE_TIME;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= ov13860_set_coarse_time(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: error coarse time override\n", __func__);

	control.id = V4L2_CID_COARSE_TIME_SHORT;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= ov13860_set_coarse_time_short(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: error coarse time short override\n", __func__);

	err = ov13860_write_table(priv, mode_table[OV13860_MODE_START_STREAM]);
	if (err)
		goto exit;

	if (test_mode)
		err = ov13860_write_table(priv,
			mode_table[OV13860_MODE_TEST_PATTERN]);

	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static struct v4l2_subdev_video_ops ov13860_subdev_video_ops = {
	.s_stream	= ov13860_s_stream,
	.s_mbus_fmt	= camera_common_s_fmt,
	.g_mbus_fmt	= camera_common_g_fmt,
	.try_mbus_fmt	= camera_common_try_fmt,
	.enum_mbus_fmt	= camera_common_enum_fmt,
	.g_mbus_config	= camera_common_g_mbus_config,
};

static struct v4l2_subdev_core_ops ov13860_subdev_core_ops = {
	.g_chip_ident	= camera_common_g_chip_ident,
	.s_power	= camera_common_s_power,
};

static struct v4l2_subdev_ops ov13860_subdev_ops = {
	.core	= &ov13860_subdev_core_ops,
	.video	= &ov13860_subdev_video_ops,
};

static struct of_device_id ov13860_of_match[] = {
	{ .compatible = "nvidia,ov13860", },
	{ },
};

static struct camera_common_sensor_ops ov13860_common_ops = {
	.power_on = ov13860_power_on,
	.power_off = ov13860_power_off,
	.write_reg = ov13860_write_reg,
	.read_reg = ov13860_read_reg,
};

static int ov13860_set_group_hold(struct ov13860 *priv, s32 val)
{
	int err;
	int gh_en = switch_ctrl_qmenu[val];

	if (gh_en == SWITCH_ON) {
		/* group hold start */
		err = ov13860_write_reg(priv->s_data,
				       OV13860_GROUP_HOLD_ADDR, 0x00);
		if (err)
			goto fail;
	} else if (gh_en == SWITCH_OFF) {
		/* group hold end */
		err = ov13860_write_reg(priv->s_data,
				       OV13860_GROUP_HOLD_ADDR, 0x10);
		if (err)
			goto fail;
		/* enable manual launch */
		err = ov13860_write_reg(priv->s_data,
				       OV13860_GROUP_HOLD_LAUNCH_ADDR, 0x00);
		if (err)
			goto fail;
		/* quick launch */
		err = ov13860_write_reg(priv->s_data,
				       OV13860_GROUP_HOLD_ADDR, 0xE0);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

static int ov13860_set_gain(struct ov13860 *priv, s32 val)
{
	ov13860_reg reg_list[3];
	int err;
	u16 gain;
	int i = 0;

	/* max_gain 15.5x ---> 0x350A=0x00, 0x350B=0xF8 */
	/* min_gain 1.0x  ---> 0x350A=0x00, 0x350B=0x10 */
	/* translate value */
	gain = ov13860_to_gain((u32)val, OV13860_GAIN_SHIFT);

	dev_dbg(&priv->i2c_client->dev,
		 "%s: gain: %d\n", __func__, gain);

	ov13860_get_gain_reg(reg_list, gain);
	err = ov13860_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: GAIN control error\n", __func__);
	return err;
}

static int ov13860_set_frame_length(struct ov13860 *priv, s32 val)
{
	ov13860_reg reg_list[3];
	int err;
	u16 frame_length;
	int i = 0;

	frame_length = (u16)val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: frame_length: %d\n", __func__, frame_length);

	ov13860_get_frame_length_regs(reg_list, frame_length);
	err = ov13860_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int ov13860_set_coarse_time(struct ov13860 *priv, s32 val)
{
	ov13860_reg reg_list[3];
	int err;
	u16 coarse_time;
	int i = 0;

	coarse_time = (u16)val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: coarse_time: %d\n", __func__, coarse_time);

	ov13860_get_coarse_time_regs(reg_list, coarse_time);
	err = ov13860_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static int ov13860_set_coarse_time_short(struct ov13860 *priv, s32 val)
{
	ov13860_reg reg_list[3];
	int err;
	struct v4l2_control hdr_control;
	int hdr_en;
	u16 coarse_time_short;
	int i = 0;

	/* check hdr enable ctrl */
	hdr_control.id = V4L2_CID_HDR_EN;

	err = v4l2_g_ctrl(&priv->ctrl_handler, &hdr_control);
	if (err < 0) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[hdr_control.value];
	if (hdr_en == SWITCH_OFF)
		return 0;

	coarse_time_short = (u16)val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: coarse_time_short: %d\n", __func__, coarse_time_short);

	ov13860_get_coarse_time_short_regs(reg_list, coarse_time_short);
	err = ov13860_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME_SHORT control error\n", __func__);
	return err;
}

static int ov13860_eeprom_device_release(struct ov13860 *priv)
{
	int i;

	for (i = 0; i < OV13860_EEPROM_NUM_BLOCKS; i++) {
		if (priv->eeprom[i].i2c_client != NULL) {
			i2c_unregister_device(priv->eeprom[i].i2c_client);
			priv->eeprom[i].i2c_client = NULL;
		}
	}

	return 0;
}

static int ov13860_eeprom_device_init(struct ov13860 *priv)
{
	char *dev_name = "eeprom_ov13860";
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, V4L2_CID_EEPROM_DATA);
	if (!priv->pdata->has_eeprom) {
		ctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}

	for (i = 0; i < OV13860_EEPROM_NUM_BLOCKS; i++) {
		priv->eeprom[i].adap = i2c_get_adapter(
				priv->i2c_client->adapter->nr);
		memset(&priv->eeprom[i].brd, 0, sizeof(priv->eeprom[i].brd));
		strncpy(priv->eeprom[i].brd.type, dev_name,
				sizeof(priv->eeprom[i].brd.type));
		priv->eeprom[i].brd.addr = priv->pdata->eeprom_base_addr + i;
		priv->eeprom[i].i2c_client = i2c_new_device(
				priv->eeprom[i].adap, &priv->eeprom[i].brd);

		priv->eeprom[i].regmap = devm_regmap_init_i2c(
			priv->eeprom[i].i2c_client, &eeprom_regmap_config);
		if (IS_ERR(priv->eeprom[i].regmap)) {
			err = PTR_ERR(priv->eeprom[i].regmap);
			ov13860_eeprom_device_release(priv);
			ctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return err;
		}
	}

	return 0;
}

static int ov13860_read_eeprom(struct ov13860 *priv,
				struct v4l2_ctrl *ctrl)
{
	int err, i;

	for (i = 0; i < OV13860_EEPROM_NUM_BLOCKS; i++) {
		err = regmap_bulk_read(priv->eeprom[i].regmap, 0,
			&priv->eeprom_buf[i * OV13860_EEPROM_BLOCK_SIZE],
			OV13860_EEPROM_BLOCK_SIZE);
		if (err)
			return err;
	}

	for (i = 0; i < OV13860_EEPROM_SIZE; i++)
		sprintf(&ctrl->string[i*2], "%02x",
			priv->eeprom_buf[i]);
	return 0;
}

static int ov13860_write_eeprom(struct ov13860 *priv,
				char *string)
{
	int err;
	int i;
	u8 curr[3];
	unsigned long data;

	for (i = 0; i < OV13860_EEPROM_SIZE; i++) {
		curr[0] = string[i*2];
		curr[1] = string[i*2+1];
		curr[2] = '\0';

		err = kstrtol(curr, 16, &data);
		if (err) {
			dev_err(&priv->i2c_client->dev,
				"invalid eeprom string\n");
			return -EINVAL;
		}

		priv->eeprom_buf[i] = (u8)data;
		err = regmap_write(priv->eeprom[i >> 8].regmap,
				   i & 0xFF, (u8)data);
		if (err)
			return err;
		msleep(20);
	}
	return 0;
}

static int ov13860_read_otp(struct ov13860 *priv, u8 *buf,
		u16 addr, int size)
{
	int err;

	/* PLL setting */
	err = ov13860_write_reg(priv->s_data, OV13860_ISP_CTRL_ADDR, 0x01);
	if (err)
		return err;
	/* Start streaming before write or read */
	err = ov13860_write_reg(priv->s_data, 0x0100, 0x01);
	if (err)
		return err;
	msleep(20);

	/* By default otp loading works in auto mode, but we can switch to
	   manual mode through OV13860_OTP_MODE_CTRL_ADDR[6] and the start
	   addr and end addr of manual mode can be configured by registers
	   accordingly
	 */

	/* Loading enable */
	err = ov13860_write_reg(priv->s_data, OV13860_OTP_LOAD_CTRL_ADDR, 0x01);
	if (err)
		return err;

	msleep(20);
	err = regmap_bulk_read(priv->regmap, addr, buf, size);
	if (err)
		return err;

	return 0;
}

static int ov13860_otp_setup(struct ov13860 *priv)
{
	int err;
	int i;
	struct v4l2_ctrl *ctrl;
	u8 otp_buf[OV13860_OTP_SIZE];

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	ov13860_read_otp(priv, &otp_buf[0],
				   OV13860_OTP_SRAM_START_ADDR,
				   OV13860_OTP_SIZE);

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, V4L2_CID_OTP_DATA);
	if (!ctrl) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < OV13860_OTP_SIZE; i++)
		sprintf(&ctrl->string[i*2], "%02x",
			otp_buf[i]);
	ctrl->cur.string = ctrl->string;

	err = camera_common_s_power(priv->subdev, false);
	if (err)
		return -ENODEV;

	return 0;
}

static int ov13860_fuse_id_setup(struct ov13860 *priv)
{
	int err;
	int i;
	struct v4l2_ctrl *ctrl;
	u8 fuse_id[OV13860_FUSE_ID_SIZE];

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	/* Here we read ID from OTP */
	/* Actually, it can also be read from registers */
	/* Bit[23:0] = [0x380A] | [0x380B] | [0x380C] */
	ov13860_read_otp(priv, &fuse_id[0],
				   OV13860_FUSE_ID_OTP_BASE_ADDR,
				   OV13860_FUSE_ID_SIZE);

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, V4L2_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < OV13860_FUSE_ID_SIZE; i++)
		sprintf(&ctrl->string[i*2], "%02x",
			fuse_id[i]);
	ctrl->cur.string = ctrl->string;

	err = camera_common_s_power(priv->subdev, false);
	if (err)
		return -ENODEV;

	return 0;

	return 0;
}

static int ov13860_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov13860 *priv =
		container_of(ctrl->handler, struct ov13860, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EEPROM_DATA:
		err = ov13860_read_eeprom(priv, ctrl);
		if (err)
			return err;
		break;
	default:
			pr_err("%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int ov13860_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov13860 *priv =
		container_of(ctrl->handler, struct ov13860, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = ov13860_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_FRAME_LENGTH:
		err = ov13860_set_frame_length(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME:
		err = ov13860_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME_SHORT:
		err = ov13860_set_coarse_time_short(priv, ctrl->val);
		break;
	case V4L2_CID_GROUP_HOLD:
		err = ov13860_set_group_hold(priv, ctrl->val);
		break;
	case V4L2_CID_EEPROM_DATA:
		if (!ctrl->string[0])
			break;
		err = ov13860_write_eeprom(priv, ctrl->string);
		if (err)
			return err;
		break;
	case V4L2_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int ov13860_ctrls_init(struct ov13860 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->string = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
			if (!ctrl->string) {
				dev_err(&client->dev,
					"Failed to allocate otp data\n");
				return -ENOMEM;
			}
		}
		priv->ctrls[i] = ctrl;
	}

	priv->num_ctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	err = ov13860_otp_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading otp data\n", err);
		goto error;
	}

	err = ov13860_fuse_id_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading fuse id data\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, ov13860_of_match);

static struct camera_common_pdata *ov13860_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int gpio;
	int err;

	match = of_match_device(ov13860_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	err = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err) {
		dev_err(&client->dev, "mclk not in DT\n");
		goto error;
	}

	gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (gpio < 0) {
		dev_dbg(&client->dev, "pwdn gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		dev_dbg(&client->dev, "reset gpios not in DT\n");
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	of_property_read_string(np, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	of_property_read_string(np, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	of_property_read_u32(np, "eeprom-addr",
			&board_priv_pdata->eeprom_base_addr);

	board_priv_pdata->has_eeprom = of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static int ov13860_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct ov13860 *priv;
	struct soc_camera_subdev_desc *ssdd;
	struct tegra_camera_platform_data *ov13860_camera_data;
	char node_name[16];
	int err;

	pr_info("[OV13860]: probing v4l2 sensor.\n");

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			sizeof(struct ov13860) + sizeof(struct v4l2_ctrl *) *
			ARRAY_SIZE(ctrl_config_list),
			GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	ssdd = soc_camera_i2c_to_desc(client);
	ov13860_camera_data = (struct tegra_camera_platform_data *)
			     ssdd->drv_priv;
	if (!ov13860_camera_data) {
		dev_err(&client->dev, "unable to find iclink module name\n");
		return -EFAULT;
	}

	snprintf(node_name, sizeof(node_name), "ov13860_%02x", client->addr);
	dev_dbg(&client->dev, "%s: dt node name %s\n", __func__, node_name);
	client->dev.of_node = of_find_node_by_name(NULL, node_name);

	if (client->dev.of_node)
		priv->pdata = ov13860_parse_dt(client);
	else
		priv->pdata = ssdd->dev_priv;

	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &ov13860_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= &ov13860_frmfmt[0];
	common_data->colorfmt		= camera_common_find_datafmt(
					  OV13860_DEFAULT_DATAFMT);
	common_data->power		= &priv->power;
	common_data->priv		= (void *)priv;
	common_data->numfmts		= ARRAY_SIZE(ov13860_frmfmt);
	common_data->def_mode		= OV13860_DEFAULT_MODE;
	common_data->def_width		= OV13860_DEFAULT_WIDTH;
	common_data->def_height		= OV13860_DEFAULT_HEIGHT;
	common_data->def_clk_freq	= OV13860_DEFAULT_CLK_FREQ;
	common_data->csi_port		= (int)ov13860_camera_data->port;
	common_data->numlanes		= ov13860_camera_data->lanes;

	priv->i2c_client		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;

	err = ov13860_power_get(priv);
	if (err)
		return err;

	camera_common_create_debugfs(common_data, node_name);

	v4l2_i2c_subdev_init(&common_data->subdev, client, &ov13860_subdev_ops);

	err = ov13860_ctrls_init(priv);
	if (err)
		return err;

	/* eeprom interface */
	err = ov13860_eeprom_device_init(priv);
	if (err)
		dev_err(&client->dev,
			"Failed to allocate eeprom register map: %d\n", err);

	return 0;
}

static int
ov13860_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd;
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct ov13860 *priv = (struct ov13860 *)s_data->priv;

	ssdd = soc_camera_i2c_to_desc(client);
	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	ov13860_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id ov13860_id[] = {
	{ "ov13860_v4l2", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov13860_id);

static struct i2c_driver ov13860_i2c_driver = {
	.driver = {
		.name = "ov13860_v4l2",
		.owner = THIS_MODULE,
	},
	.probe = ov13860_probe,
	.remove = ov13860_remove,
	.id_table = ov13860_id,
};

module_i2c_driver(ov13860_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omnivison OV13860");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
