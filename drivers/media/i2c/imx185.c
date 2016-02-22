/*
 * imx185.c - imx185 sensor driver
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <media/camera_common.h>
#include <media/imx185.h>
#include "imx185_mode_tbls.h"


#define TESTING_10BIT  1

#ifdef TESTING_10BIT
#define IMX185_DEFAULT_MODE	IMX185_MODE_1920X1080_CROP_10BIT
#define IMX185_DEFAULT_DATAFMT	V4L2_MBUS_FMT_SRGGB10_1X10
#else
#define IMX185_DEFAULT_MODE	IMX185_MODE_1920X1080_CROP
#define IMX185_DEFAULT_DATAFMT	V4L2_MBUS_FMT_SRGGB12_1X12
#endif

#define IMX185_MAX_COARSE_DIFF 2
#define IMX185_MAX_COARSE_DIFF_HDR 5
#define IMX185_GAIN_SHIFT		0
#define IMX185_MIN_GAIN		(1)
#define IMX185_MAX_GAIN		(255)
#define IMX185_MIN_FRAME_LENGTH	(1125)
#define IMX185_MAX_FRAME_LENGTH	(0xFFFF)
#define IMX185_MIN_EXPOSURE_COARSE	(0x0001)
#define IMX185_MAX_EXPOSURE_COARSE	\
	(IMX185_MAX_FRAME_LENGTH-IMX185_MAX_COARSE_DIFF)

#define IMX185_MIN_FRAME_LENGTH_1080P_HDR	(1125)
#define IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS1	(5)
#define IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS1	(70)
#define IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS2	(80)
#define IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS2	(1120)


#define IMX185_DEFAULT_WIDTH	1920
#define IMX185_DEFAULT_HEIGHT	1080
#define IMX185_DEFAULT_GAIN		IMX185_MIN_GAIN
#define IMX185_DEFAULT_FRAME_LENGTH	(1125)
#define IMX185_DEFAULT_EXPOSURE_COARSE	\
	(IMX185_DEFAULT_FRAME_LENGTH-IMX185_MAX_COARSE_DIFF)
#define IMX185_DEFAULT_EXPOSURE_COARSE_SHORT_HDR	\
	(IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS1- \
	IMX185_MAX_COARSE_DIFF_HDR)


#define IMX185_DEFAULT_CLK_FREQ	37125000


struct imx185 {
	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	s32 group_hold_prev;
	bool group_hold_en;
	u32	frame_length;

	u32 i2c_channel;
	struct i2c_client *pca954x_i2c_client;
	struct i2c_adapter *pca954x_adap;
	struct i2c_board_info pca954x_brd;
	struct regmap *pca954x_regmap;

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

static int imx185_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx185_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx185_ctrl_ops = {
	.g_volatile_ctrl = imx185_g_volatile_ctrl,
	.s_ctrl = imx185_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx185_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX185_MIN_GAIN,
		.max = IMX185_MAX_GAIN,
		.def = IMX185_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX185_MIN_FRAME_LENGTH,
		.max = IMX185_MAX_FRAME_LENGTH,
		.def = IMX185_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX185_MIN_EXPOSURE_COARSE,
		.max = IMX185_MAX_EXPOSURE_COARSE,
		.def = IMX185_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx185_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS1,
		.max = IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS1,
		.def = IMX185_DEFAULT_EXPOSURE_COARSE_SHORT_HDR,
		.step = 1,
	},
	{
		.ops = &imx185_ctrl_ops,
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
		.ops = &imx185_ctrl_ops,
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
		.ops = &imx185_ctrl_ops,
		.id = V4L2_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = IMX185_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};

static inline void imx185_get_frame_length_regs(imx185_reg *regs,
				u16 frame_length)
{
	regs->addr = IMX185_FRAME_LENGTH_ADDR_MID;
	regs->val = (frame_length >> 8) & 0xff;

	(regs + 1)->addr = IMX185_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void imx185_get_coarse_time_regs_shs1(imx185_reg *regs,
				u16 coarse_time)
{
	regs->addr = IMX185_COARSE_TIME_SHS1_ADDR_MID;
	regs->val = (coarse_time >> 8) & 0xff;

	(regs + 1)->addr = IMX185_COARSE_TIME_SHS1_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx185_get_coarse_time_regs_shs2(imx185_reg *regs,
				u16 coarse_time)
{
	regs->addr = IMX185_COARSE_TIME_SHS2_ADDR_MID;
	regs->val = (coarse_time >> 8) & 0xff;

	(regs + 1)->addr = IMX185_COARSE_TIME_SHS2_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx185_get_gain_reg(imx185_reg *regs,
				u8 gain)
{
	regs->addr = IMX185_GAIN_ADDR;
	regs->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx185_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	return regmap_read(priv->regmap, addr, (unsigned int *) val);
}

static int imx185_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct imx185 *priv = (struct imx185 *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}



static int imx185_write_table(struct imx185 *priv,
				const imx185_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX185_TABLE_WAIT_MS,
					 IMX185_TABLE_END);
}

static int pca954x_device_release(struct imx185 *priv)
{
	if (priv->pca954x_i2c_client != NULL) {
		i2c_unregister_device(priv->pca954x_i2c_client);
		priv->pca954x_i2c_client = NULL;
	}
	return 0;
}

static int pca954x_device_init(struct imx185 *priv)
{
	int err;
	char *dev_name = "pca954x";

	static  struct regmap_config pca954x_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
		.cache_type = REGCACHE_RBTREE,
	};

	priv->pca954x_adap = i2c_get_adapter(
			priv->i2c_client->adapter->nr);
	memset(&priv->pca954x_brd, 0, sizeof(priv->pca954x_brd));
	strncpy(priv->pca954x_brd.type, dev_name,
			sizeof(priv->pca954x_brd.type));

	priv->pca954x_brd.addr = IMX185_PCA954X_I2C_ADDR;
	priv->pca954x_i2c_client = i2c_new_device(
			priv->pca954x_adap, &priv->pca954x_brd);

	priv->pca954x_regmap = devm_regmap_init_i2c(
		priv->pca954x_i2c_client, &pca954x_regmap_config);
	if (IS_ERR(priv->pca954x_regmap)) {
		err = PTR_ERR(priv->pca954x_regmap);
		pca954x_device_release(priv);
		return err;
	}
	return 0;
}

static int pca954x_write_reg(struct camera_common_data *s_data,
				u8 addr, u8 val)
{
	int err;
	struct imx185 *priv = (struct imx185 *)s_data->priv;

	err = regmap_write(priv->pca954x_regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx185_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);
	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err) {
			pr_err("%s failed.\n", __func__);
			return err;
		}
	}

	/*exit reset mode: XCLR */
	if (pw->reset_gpio) {
		gpio_set_value(pw->reset_gpio, 0);
		usleep_range(30, 50);
		gpio_set_value(pw->reset_gpio, 1);
		usleep_range(30, 50);
	}

	pw->state = SWITCH_ON;
	return 0;

}

static int imx185_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);
	if (priv->pdata && priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (err) {
			pr_err("%s failed.\n", __func__);
			return err;
		}
	}
	/* enter reset mode: XCLR */
	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

	pw->state = SWITCH_OFF;

	return 0;
}

static int imx185_power_put(struct imx185 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	if (unlikely(!pw))
		return -EFAULT;
	return 0;
}

static int imx185_power_get(struct imx185 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
				priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	if (pdata->regulators.iovdd != NULL)
		camera_common_regulator_get(priv->i2c_client,
				&pw->iovdd, pdata->regulators.iovdd);
	if (!err)
		pw->reset_gpio = pdata->reset_gpio;

	pw->state = SWITCH_OFF;
	return err;
}

static int imx185_set_gain(struct imx185 *priv, s32 val);
static int imx185_set_frame_length(struct imx185 *priv, s32 val);
static int imx185_set_coarse_time(struct imx185 *priv, s32 val);
static int imx185_set_coarse_time_shs1(struct imx185 *priv, s32 val);
static int imx185_set_coarse_time_hdr_shs2(struct imx185 *priv, s32 val);

static int imx185_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx185 *priv = (struct imx185 *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	err =  imx185_write_table(priv, mode_table[IMX185_MODE_STOP_STREAM]);
	if (err)
		goto exit;
	if (!enable)
		return err;

	err = imx185_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;
	/* write list of override regs for the asking frame length, */
	/* coarse integration time, and gain. Failures to write
	 * overrides are non-fatal */
	control.id = V4L2_CID_GAIN;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx185_set_gain(priv, control.value);
	if (err)
		dev_dbg(&client->dev, "%s: warning gain override failed\n",
			__func__);

	control.id = V4L2_CID_FRAME_LENGTH;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx185_set_frame_length(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: warning frame length override failed\n", __func__);

	control.id = V4L2_CID_COARSE_TIME;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx185_set_coarse_time(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: warning coarse time override failed\n", __func__);

	control.id = V4L2_CID_COARSE_TIME_SHORT;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx185_set_coarse_time_shs1(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: warning coarse time short override failed\n",
			__func__);
	if (test_mode)
		err = imx185_write_table(priv,
			mode_table[IMX185_MODE_TEST_PATTERN]);

	err = imx185_write_table(priv, mode_table[IMX185_MODE_START_STREAM]);
	if (err)
		goto exit;

	return 0;
exit:
	dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static struct v4l2_subdev_video_ops imx185_subdev_video_ops = {
	.s_stream	= imx185_s_stream,
	.s_mbus_fmt	= camera_common_s_fmt,
	.g_mbus_fmt	= camera_common_g_fmt,
	.try_mbus_fmt	= camera_common_try_fmt,
	.enum_mbus_fmt	= camera_common_enum_fmt,
	.g_mbus_config	= camera_common_g_mbus_config,
};

static struct v4l2_subdev_core_ops imx185_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static struct v4l2_subdev_ops imx185_subdev_ops = {
	.core	= &imx185_subdev_core_ops,
	.video	= &imx185_subdev_video_ops,
};

static struct of_device_id imx185_of_match[] = {
	{ .compatible = "nvidia,imx185", },
	{ },
};

static struct camera_common_sensor_ops imx185_common_ops = {
	.power_on = imx185_power_on,
	.power_off = imx185_power_off,
	.write_reg = imx185_write_reg,
	.read_reg = imx185_read_reg,
};

static int imx185_set_group_hold(struct imx185 *priv, s32 val)
{
	int err;
	int gh_en = switch_ctrl_qmenu[val];

	priv->group_hold_prev = val;
	if (gh_en == SWITCH_ON) {

		err = imx185_write_reg(priv->s_data,
				       IMX185_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
	} else if (gh_en == SWITCH_OFF) {
		err = imx185_write_reg(priv->s_data,
				       IMX185_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
	}
	return 0;
fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

static int imx185_calculate_gain(u32 rep, int shift)
{
	u8 gain;
	int i;

	if (rep < IMX185_MIN_GAIN || rep > IMX185_MAX_GAIN) {
		pr_err("%s: %d is not a valid gain\n", __func__, rep);
		return -ENODEV;
	}

	if (rep == (imx185_gain_lookup_table[rep-1].gain_x))
		gain = imx185_gain_lookup_table[rep-1].reg_val;
	else
		pr_err("%s: invalid gain set, rep=%d\n", __func__, rep);

	return gain;
}

static int imx185_set_gain(struct imx185 *priv, s32 val)
{
	imx185_reg reg_list[1];
	int err;
	u8 gain;

	/* translate value */
	gain = (u8)imx185_calculate_gain(val, IMX185_GAIN_SHIFT);

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d gain: %d\n", __func__, val, gain);

	imx185_get_gain_reg(reg_list, gain);

	err = imx185_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: GAIN control error\n", __func__);
	return err;
}

static int imx185_set_frame_length(struct imx185 *priv, s32 val)
{
	imx185_reg reg_list[2];
	int err;
	u16 frame_length;

	frame_length = val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d\n", __func__, frame_length);

	priv->frame_length = frame_length;
	imx185_get_frame_length_regs(reg_list, frame_length);
	err = imx185_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx185_set_coarse_time(struct imx185 *priv, s32 val)
{
	int err;
	struct v4l2_control control;
	int hdr_en;

	/* check hdr enable ctrl */
	control.id = V4L2_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if (hdr_en == SWITCH_OFF) {
		/*no WDR, update SHS1 as ET*/
		err = imx185_set_coarse_time_shs1(priv, val);
		if (err)
			dev_dbg(&priv->i2c_client->dev,
			"%s: error coarse time SHS1 override\n", __func__);
	} else if (hdr_en == SWITCH_ON) {
		/*WDR, update SHS2 as long ET*/
		err = imx185_set_coarse_time_hdr_shs2(priv, val);
		if (err)
			dev_dbg(&priv->i2c_client->dev,
			"%s: error coarse time SHS2 override\n", __func__);
	}
	return err;
fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME_SHORT control error\n", __func__);
	return err;
}

static int imx185_set_coarse_time_shs1(struct imx185 *priv, s32 val)
{
	imx185_reg reg_list[2];
	int err;
	u16 coarse_shs1;
	struct v4l2_control control;
	int hdr_en;

	coarse_shs1 = val;

	if (priv->frame_length == 0)
		priv->frame_length = IMX185_MIN_FRAME_LENGTH;

	/* check hdr enable ctrl */
	control.id = V4L2_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if (hdr_en == SWITCH_ON) {
		if (coarse_shs1 < IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS1)
			coarse_shs1 = IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS1;

		if (coarse_shs1 > IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS1)
			coarse_shs1 = IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS1;

		priv->frame_length = IMX185_MIN_FRAME_LENGTH;
	}

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d,  shs1=%d, frame_length: %d\n", __func__,
		 coarse_shs1,
		 priv->frame_length - coarse_shs1 - 1,
		 priv->frame_length);

	imx185_get_coarse_time_regs_shs1(reg_list,
			priv->frame_length - coarse_shs1 - 1);

	err = imx185_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static int imx185_set_coarse_time_hdr_shs2(struct imx185 *priv, s32 val)
{
	imx185_reg reg_list[2];
	int err;
	u16 coarse_shs2;

	coarse_shs2 = val;
	if (coarse_shs2 < IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS2)
		coarse_shs2 = IMX185_MIN_EXPOSURE_COARSE_1080P_HDR_SHS2;

	if (coarse_shs2 > IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS2)
		coarse_shs2 = IMX185_MAX_EXPOSURE_COARSE_1080P_HDR_SHS2;

	priv->frame_length = IMX185_MIN_FRAME_LENGTH;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d,  shs2=%d, frame_length: %d\n", __func__,
		 coarse_shs2,
		 priv->frame_length - coarse_shs2 - 1,
		 priv->frame_length);


	imx185_get_coarse_time_regs_shs2(reg_list,
			priv->frame_length - coarse_shs2 - 1);

	err = imx185_write_table(priv, reg_list);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME_SHORT control error\n", __func__);
	return err;
}

static int imx185_fuse_id_setup(struct imx185 *priv)
{
	int err;
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(priv->subdev);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct camera_common_power_rail *pw = &priv->power;

	struct v4l2_ctrl *ctrl;
	u8 fuse_id[IMX185_FUSE_ID_SIZE];
	u8 bak = 0;

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;
	if (pw->iovdd) {
		err = pca954x_device_init(priv);
		if (err)
			dev_err(&client->dev,
				"Fail to allocate pca954x regmap: %d\n", err);
		err = regulator_enable(pw->iovdd);
		if (!err) {
			usleep_range(500, 510);
			pca954x_write_reg(priv->s_data, IMX185_PCA954X_I2C_ADDR,
				priv->i2c_channel);
			usleep_range(500, 510);
		} else {
			pr_err("%s: iovdd regulator_enable faill\n", __func__);
		}
	}

	for (i = 0; i < IMX185_FUSE_ID_SIZE; i++) {
		err |= imx185_read_reg(s_data,
			IMX185_FUSE_ID_ADDR + i, (unsigned int *) &bak);
		if (!err)
			fuse_id[i] = bak;
		else
			pr_err("%s: can not read fuse id\n", __func__);
	}

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, V4L2_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < IMX185_FUSE_ID_SIZE; i++)
		sprintf(&ctrl->string[i*2], "%02x",
			fuse_id[i]);
	ctrl->cur.string = ctrl->string;
	pr_info("%s,  fuse id: %s\n", __func__, ctrl->cur.string);

	err = camera_common_s_power(priv->subdev, false);
	if (err)
		return -ENODEV;

	return 0;
}

static int imx185_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx185 *priv =
		container_of(ctrl->handler, struct imx185, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {

	default:
			pr_err("%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int imx185_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx185 *priv =
		container_of(ctrl->handler, struct imx185, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = imx185_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_FRAME_LENGTH:
		err = imx185_set_frame_length(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME:
		err = imx185_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME_SHORT:
		err = imx185_set_coarse_time_shs1(priv, ctrl->val);
		break;
	case V4L2_CID_GROUP_HOLD:
		err = imx185_set_group_hold(priv, ctrl->val);
		break;
	case V4L2_CID_HDR_EN:

		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx185_ctrls_init(struct imx185 *priv)
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

	err = imx185_fuse_id_setup(priv);
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

MODULE_DEVICE_TABLE(of, imx185_of_match);

static struct camera_common_pdata *imx185_parse_dt(struct imx185 *priv,
				struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int sts;

	if (!np)
		return NULL;

	match = of_match_device(imx185_of_match, &client->dev);
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

	sts = camera_common_parse_clocks(client, board_priv_pdata);
	if (sts)
		dev_err(&client->dev, "Failed to find clocks\n");

	sts = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (sts)
		dev_err(&client->dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (sts) {
		dev_err(&client->dev, "reset-gpios not found %d\n", sts);
		board_priv_pdata->reset_gpio = 0;
	}

	sts = of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (sts) {
		board_priv_pdata->regulators.iovdd = NULL;
		dev_err(&client->dev, "iovdd-reg not found %d\n", sts);
	}

	sts = of_property_read_u32(np, "i2c-channel", &priv->i2c_channel);
	if (sts) {
		board_priv_pdata->regulators.iovdd = NULL;
		dev_err(&client->dev, "i2c-channel not found %d\n", sts);
	}

	return board_priv_pdata;
}

static int imx185_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx185_subdev_internal_ops = {
	.open = imx185_open,
};

static const struct media_entity_operations imx185_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx185_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct imx185 *priv;
	char dt_name[16];
	char debugfs_name[10];
	int err;

	pr_info("[IMX185]: probing v4l2 sensor at addr 0x%0x.\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx185) + sizeof(struct v4l2_ctrl *) *
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

	if (client->dev.of_node)
		priv->pdata = imx185_parse_dt(priv, client);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops = &imx185_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->i2c_client = client;
	common_data->frmfmt = &imx185_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
					  IMX185_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(imx185_frmfmt);
	common_data->def_mode = IMX185_DEFAULT_MODE;
	common_data->def_width = IMX185_DEFAULT_WIDTH;
	common_data->def_height = IMX185_DEFAULT_HEIGHT;
	common_data->def_clk_freq = IMX185_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;

	err = imx185_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(client, common_data);
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}
	sprintf(debugfs_name, "imx185_%c", common_data->csi_port + 'a');
	dev_dbg(&client->dev, "%s: name %s\n", __func__, debugfs_name);

	camera_common_create_debugfs(common_data, debugfs_name);

	v4l2_i2c_subdev_init(priv->subdev, client, &imx185_subdev_ops);

	err = imx185_ctrls_init(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx185_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &imx185_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_info(&client->dev, "Detected IMX185 sensor\n");

	return 0;
}

static int
imx185_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx185 *priv = (struct imx185 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx185_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id imx185_id[] = {
	{ "imx185", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx185_id);

static struct i2c_driver imx185_i2c_driver = {
	.driver = {
		.name = "imx185",
		.owner = THIS_MODULE,
	},
	.probe = imx185_probe,
	.remove = imx185_remove,
	.id_table = imx185_id,
};

module_i2c_driver(imx185_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX185");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
