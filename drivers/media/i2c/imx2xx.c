/*
 * imx2xx.c - imx2xx sensor driver
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
#include <media/imx2xx.h>
#include "imx214_mode_tbls.h"
#include "imx230_mode_tbls.h"


/* Sensor-specific const data */
struct imx_sensor_data {
	char imx_name[10];
	/* Mode table array */
	imx2xx_reg **imx_mode_table;
	/* Mode data structure for common layer */
	const struct camera_common_frmfmt *imx_frmfmt;
	/* Color fmt data structure for common layer */
	const struct camera_common_colorfmt *imx_color_fmts;
	/* Number of sensor modes */
	int imx_frmcnt;
	/* Number of color fmts */
	int imx_num_color_fmts;
	/* OTP size */
	int imx_otp_num_pages;
	/* Fuse ID location */
	int imx_fuse_id_pg_num;
	u16 imx_fuse_id_reg_start;
	int imx_fuse_id_size;
	int imx_sensor_max_width;
	int imx_sensor_max_height;
};

const struct imx_sensor_data imx214_sensor_data = {
	.imx_name = "imx214",
	.imx_mode_table = imx214_mode_table,
	.imx_frmfmt = imx214_frmfmt,
	.imx_color_fmts = imx214_color_fmts,
	.imx_frmcnt = ARRAY_SIZE(imx214_frmfmt),
	.imx_num_color_fmts = ARRAY_SIZE(imx214_color_fmts),
	.imx_otp_num_pages = 16,
	.imx_fuse_id_pg_num = 19,
	.imx_fuse_id_reg_start = 0x0A36,
	.imx_fuse_id_size = 11,
	.imx_sensor_max_width = 4096,
	.imx_sensor_max_height = 3072,
};

const struct imx_sensor_data imx230_sensor_data = {
	.imx_name = "imx230",
	.imx_mode_table = imx230_mode_table,
	.imx_frmfmt = imx230_frmfmt,
	.imx_color_fmts = imx230_color_fmts,
	.imx_frmcnt = ARRAY_SIZE(imx230_frmfmt),
	.imx_num_color_fmts = ARRAY_SIZE(imx230_color_fmts),
	.imx_otp_num_pages = 18,
	.imx_fuse_id_pg_num = 31,
	.imx_fuse_id_reg_start = 0x0A36,
	.imx_fuse_id_size = 11,
	.imx_sensor_max_width = 5344,
	.imx_sensor_max_height = 4016,
};

/* Useful macros to dereference sensor-type constants */
#define IMX2XX_OTP_NUM_PAGES		(priv->sensor_data->imx_otp_num_pages)
#define IMX2XX_OTP_SIZE			(IMX2XX_OTP_NUM_PAGES * \
					 IMX2XX_OTP_PAGE_SIZE)
#define IMX2XX_OTP_STR_SIZE		(IMX2XX_OTP_SIZE * 2)

#define IMX2XX_FUSE_ID_OTP_PAGE		(priv->sensor_data->imx_fuse_id_pg_num)
#define IMX2XX_FUSE_ID_OTP_ROW_ADDR	\
				(priv->sensor_data->imx_fuse_id_reg_start)
#define IMX2XX_FUSE_ID_SIZE		(priv->sensor_data->imx_fuse_id_size)
#define IMX2XX_FUSE_ID_STR_SIZE		(IMX2XX_FUSE_ID_SIZE * 2)

/* Defaults */
#define IMX2XX_DEFAULT_GAIN		(priv->default_gain)
#define IMX2XX_DEFAULT_FRAME_LENGTH	(priv->default_fl)
#define IMX2XX_DEFAULT_EXPOSURE_COARSE	\
	(IMX2XX_DEFAULT_FRAME_LENGTH-IMX2XX_MAX_COARSE_DIFF)

#define IMX2XX_EEPROM_ADDRESS		(priv->eeprom_addr)
#define IMX2XX_EEPROM_SIZE		(priv->eeprom_size)
#define IMX2XX_EEPROM_BLOCK_SIZE	(priv->eeprom_blk_size)
#define IMX2XX_EEPROM_STR_SIZE		(IMX2XX_EEPROM_SIZE * 2)
#define IMX2XX_EEPROM_NUM_BLOCKS \
	(priv->eeprom_size/priv->eeprom_blk_size)

struct imx2xx {
	struct camera_common_power_rail	power;
	int				numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct camera_common_eeprom_data *eeprom;
	u8				*eeprom_buf;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;

	/* Sensor-specific constants */
	const struct imx_sensor_data	*sensor_data;

	/* Default values from DT */
	u32 default_gain;
	u32 default_fl;
	u32 default_ct;

	/* EEPROM data from DT */
	u32 eeprom_addr;
	u32 eeprom_size;
	u32 eeprom_blk_size;
	u32 eeprom_regaddr_width;
	u32 eeprom_regval_width;

	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int imx2xx_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx2xx_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx2xx_ctrl_ops = {
	.g_volatile_ctrl = imx2xx_g_volatile_ctrl,
	.s_ctrl		= imx2xx_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX2XX_MIN_GAIN,
		.max = IMX2XX_MAX_GAIN,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX2XX_MIN_FRAME_LENGTH,
		.max = IMX2XX_MAX_FRAME_LENGTH,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX2XX_MIN_EXPOSURE_COARSE,
		.max = IMX2XX_MAX_EXPOSURE_COARSE,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX2XX_MIN_EXPOSURE_COARSE,
		.max = IMX2XX_MAX_EXPOSURE_COARSE,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &imx2xx_ctrl_ops,
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
		.ops = &imx2xx_ctrl_ops,
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
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_EEPROM_DATA,
		.name = "EEPROM Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.min = 0,
		.max = 0,
		.step = 2,
	},
	{
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = 0,
		.step = 2,
	},
	{
		.ops = &imx2xx_ctrl_ops,
		.id = V4L2_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = 0,
		.step = 2,
	},
};

static inline void imx2xx_get_frame_length_regs(imx2xx_reg *regs,
				u16 frame_length)
{
	regs->addr = IMX2XX_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX2XX_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void imx2xx_get_coarse_time_regs(imx2xx_reg *regs,
				u16 coarse_time)
{
	regs->addr = IMX2XX_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX2XX_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx2xx_get_coarse_time_short_regs(imx2xx_reg *regs,
				u16 coarse_time)
{
	regs->addr = IMX2XX_COARSE_TIME_SHORT_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX2XX_COARSE_TIME_SHORT_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx2xx_get_gain_regs(imx2xx_reg *regs,
				u16 gain)
{
	regs->addr = IMX2XX_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = IMX2XX_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static inline void imx2xx_get_gain_short_reg(imx2xx_reg *regs,
				u16 gain)
{
	regs->addr = IMX2XX_GAIN_SHORT_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = IMX2XX_GAIN_SHORT_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static void imx2xx_get_crop_regs(imx2xx_reg *regs,
				const struct v4l2_rect *rect)
{
	u32 x_start, y_start;
	u32 x_end, y_end;
	x_start = rect->left;
	y_start = rect->top;
	x_end = x_start + rect->width - 1;
	y_end = y_start + rect->height - 1;

	regs->addr = IMX2XX_CROP_X_START_ADDR_MSB;
	regs->val = (x_start >> 8) & 0xff;
	(regs + 1)->addr = IMX2XX_CROP_X_START_ADDR_LSB;
	(regs + 1)->val = (x_start) & 0xff;

	(regs + 2)->addr = IMX2XX_CROP_Y_START_ADDR_MSB;
	(regs + 2)->val = (y_start >> 8) & 0xff;
	(regs + 3)->addr = IMX2XX_CROP_Y_START_ADDR_LSB;
	(regs + 3)->val = (y_start) & 0xff;

	(regs + 4)->addr = IMX2XX_CROP_X_END_ADDR_MSB;
	(regs + 4)->val = (x_end >> 8) & 0xff;
	(regs + 5)->addr = IMX2XX_CROP_X_END_ADDR_LSB;
	(regs + 5)->val = (x_end) & 0xff;

	(regs + 6)->addr = IMX2XX_CROP_Y_END_ADDR_MSB;
	(regs + 6)->val = (y_end >> 8) & 0xff;
	(regs + 7)->addr = IMX2XX_CROP_Y_END_ADDR_LSB;
	(regs + 7)->val = (y_end) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx2xx_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;

	return regmap_read(priv->regmap, addr, (unsigned int *) val);
}

static int imx2xx_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx2xx_write_table(struct imx2xx *priv,
				const imx2xx_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX2XX_TABLE_WAIT_MS,
					 IMX2XX_TABLE_END);
}

static int imx2xx_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/* sleep calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	usleep_range(10, 20);

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto imx2xx_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto imx2xx_iovdd_fail;

	udelay(1);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 1);

	usleep_range(1000, 1010);

	pw->state = SWITCH_ON;
	return 0;

imx2xx_iovdd_fail:
	regulator_disable(pw->avdd);

imx2xx_avdd_fail:
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 0);

	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int imx2xx_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_off(pw);
		if (err) {
			pr_err("%s failed.\n", __func__);
			return err;
		} else {
			goto power_off_done;
		}
	}

	/* sleeps calls in the sequence below are for internal device
	 * signal propagation as specified by sensor vendor */

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 0);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	usleep_range(1, 2);

	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->avdd)
		regulator_disable(pw->avdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int imx2xx_power_put(struct imx2xx *priv)
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

static int imx2xx_power_get(struct imx2xx *priv)
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

	/* analog 2.7v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(priv->i2c_client,
				&pw->avdd, pdata->regulators.avdd);
	/* digital 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(priv->i2c_client,
				&pw->dvdd, pdata->regulators.dvdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(priv->i2c_client,
				&pw->iovdd, pdata->regulators.iovdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->af_gpio = pdata->af_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	pw->state = SWITCH_OFF;
	return err;
}

static int imx2xx_set_gain(struct imx2xx *priv, s32 val);
static int imx2xx_set_frame_length(struct imx2xx *priv, s32 val);
static int imx2xx_set_coarse_time(struct imx2xx *priv, s32 val);
static int imx2xx_set_coarse_time_short(struct imx2xx *priv, s32 val);
static int imx2xx_set_crop_data(struct imx2xx *priv, const struct v4l2_rect *rect);
static int imx2xx_get_crop_data(struct imx2xx *priv, struct v4l2_rect *rect);

static int imx2xx_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++ enable %d mode %d\n",
		__func__, enable, s_data->mode);
	if (!enable)
		return imx2xx_write_table(priv,
		priv->sensor_data->imx_mode_table[IMX2XX_MODE_STOP_STREAM]);

	err = imx2xx_write_table(priv,
			 priv->sensor_data->imx_mode_table[IMX2XX_MODE_COMMON]);
	if (err)
		goto exit;
	err = imx2xx_write_table(priv,
			 priv->sensor_data->imx_mode_table[s_data->mode]);
	if (err)
		goto exit;

	/* write list of override regs for the asking frame length, */
	/* coarse integration time, and gain. Failures to write
	 * overrides are non-fatal */
	control.id = V4L2_CID_GAIN;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx2xx_set_gain(priv, control.value);
	if (err)
		dev_dbg(&client->dev, "%s: warning gain override failed\n",
			__func__);

	control.id = V4L2_CID_FRAME_LENGTH;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx2xx_set_frame_length(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: warning frame length override failed\n", __func__);

	control.id = V4L2_CID_COARSE_TIME;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx2xx_set_coarse_time(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: warning coarse time override failed\n", __func__);

	control.id = V4L2_CID_COARSE_TIME_SHORT;
	err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
	err |= imx2xx_set_coarse_time_short(priv, control.value);
	if (err)
		dev_dbg(&client->dev,
			"%s: warning coarse time short override failed\n",
			__func__);

	err = imx2xx_write_table(priv,
		 priv->sensor_data->imx_mode_table[IMX2XX_MODE_START_STREAM]);
	if (err)
		goto exit;

	if (test_mode)
		err = imx2xx_write_table(priv,
		priv->sensor_data->imx_mode_table[IMX2XX_MODE_TEST_PATTERN]);

	dev_dbg(&client->dev, "%s: success setting stream\n", __func__);
	return 0;
exit:
	dev_info(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx2xx_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *crop)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	const struct v4l2_rect *rect = &crop->c;
	int err;

	u32 width, height;
	u32 bottom, right;

	width = s_data->fmt_width;
	height = s_data->fmt_height;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if ((width != rect->width) || (height != rect->height)) {
		dev_err(&client->dev,
			"%s: CROP Resolution Mismatch: %dx%d\n",
			__func__, rect->width, rect->height);
		return -EINVAL;
	}
	if (rect->top < 0 || rect->left < 0) {
		dev_err(&client->dev,
			"%s: CROP Bound Error: left:%d, top:%d\n",
			__func__, rect->left, rect->top);
		return -EINVAL;
	}
	right = rect->left + width - 1;
	bottom = rect->top + height - 1;
	if ((right > priv->sensor_data->imx_sensor_max_width) ||
		(bottom > priv->sensor_data->imx_sensor_max_height)) {
		dev_err(&client->dev,
			"%s: CROP Bound Error: right:%d, bottom:%d)\n",
			__func__, right, bottom);
		return -EINVAL;
	}
	err = imx2xx_set_crop_data(priv, rect);

	return err;
}

static int imx2xx_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	struct v4l2_rect *rect = &crop->c;
	int err;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	err = imx2xx_get_crop_data(priv, rect);

	return err;
}

static int imx2xx_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int imx2xx_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static int imx2xx_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx2xx_subdev_video_ops = {
	.s_stream	= imx2xx_s_stream,
	.s_crop		= imx2xx_s_crop,
	.g_crop		= imx2xx_g_crop,
	.s_mbus_fmt	= camera_common_s_fmt,
	.g_mbus_fmt	= camera_common_g_fmt,
	.try_mbus_fmt	= camera_common_try_fmt,
	.enum_mbus_fmt	= camera_common_enum_fmt,
	.g_mbus_config	= camera_common_g_mbus_config,
	.enum_framesizes	= camera_common_enum_framesizes,
	.enum_frameintervals	= camera_common_enum_frameintervals,
	.g_input_status		= imx2xx_g_input_status,
};

static struct v4l2_subdev_core_ops imx2xx_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static struct v4l2_subdev_pad_ops imx2xx_subdev_pad_ops = {
	.enum_mbus_code = camera_common_enum_mbus_code,
	.set_fmt	= imx2xx_set_fmt,
	.get_fmt	= imx2xx_get_fmt,
};

static struct v4l2_subdev_ops imx2xx_subdev_ops = {
	.core	= &imx2xx_subdev_core_ops,
	.video	= &imx2xx_subdev_video_ops,
	.pad	= &imx2xx_subdev_pad_ops,
};

static struct of_device_id imx2xx_of_match[] = {
	{
		.compatible = "nvidia,imx214",
		.data = &imx214_sensor_data,
	},
	{
		.compatible = "nvidia,imx230",
		.data = &imx230_sensor_data,
	},
	{ },
};

static struct camera_common_sensor_ops imx2xx_common_ops = {
	.power_on = imx2xx_power_on,
	.power_off = imx2xx_power_off,
	.write_reg = imx2xx_write_reg,
	.read_reg = imx2xx_read_reg,
};

static int imx2xx_set_group_hold(struct imx2xx *priv)
{
	int err;
	int gh_prev = switch_ctrl_qmenu[priv->group_hold_prev];

	if (priv->group_hold_en == true && gh_prev == SWITCH_OFF) {
		err = imx2xx_write_reg(priv->s_data,
				       IMX2XX_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
		priv->group_hold_prev = 1;
	} else if (priv->group_hold_en == false && gh_prev == SWITCH_ON) {
		err = imx2xx_write_reg(priv->s_data,
				       IMX2XX_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
		priv->group_hold_prev = 0;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

static int imx2xx_calculate_gain(u32 rep, int shift)
{
	int gain;
	int gain_int;
	int gain_dec;
	int min_int = (1 << shift);
	int denom;

	/* shift indicates number of least significant bits
	 * used for decimal representation of gain */
	gain_int = (int)(rep >> shift);
	gain_dec = (int)(rep & ~(0xffff << shift));

	denom = gain_int * min_int + gain_dec;
	gain = 512 - ((512 * min_int + (denom - 1)) / denom);

	return gain;
}

static int imx2xx_set_gain(struct imx2xx *priv, s32 val)
{
	imx2xx_reg reg_list[2];
	imx2xx_reg reg_list_short[2];
	int err;
	u16 gain;
	int i = 0;

	/* translate value */
	gain = (u16)imx2xx_calculate_gain(val, IMX2XX_GAIN_SHIFT);

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d\n", __func__, gain);

	imx2xx_get_gain_regs(reg_list, gain);
	imx2xx_get_gain_short_reg(reg_list_short, gain);
	imx2xx_set_group_hold(priv);

	/* writing long gain */
	for (i = 0; i < 2; i++) {
		err = imx2xx_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}
	/* writing short gain */
	for (i = 0; i < 2; i++) {
		err = imx2xx_write_reg(priv->s_data, reg_list_short[i].addr,
			 reg_list_short[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: GAIN control error\n", __func__);
	return err;
}

static int imx2xx_set_frame_length(struct imx2xx *priv, s32 val)
{
	imx2xx_reg reg_list[2];
	int err;
	u16 frame_length;
	int i = 0;

	frame_length = (u16)val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d\n", __func__, frame_length);

	imx2xx_get_frame_length_regs(reg_list, frame_length);
	imx2xx_set_group_hold(priv);

	for (i = 0; i < 2; i++) {
		err = imx2xx_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx2xx_set_coarse_time(struct imx2xx *priv, s32 val)
{
	imx2xx_reg reg_list[2];
	int err;
	u16 coarse_time;
	int i = 0;

	coarse_time = (u16)val;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %d\n", __func__, coarse_time);

	imx2xx_get_coarse_time_regs(reg_list, coarse_time);
	imx2xx_set_group_hold(priv);

	for (i = 0; i < 2; i++) {
		err = imx2xx_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME control error\n", __func__);
	return err;
}

static int imx2xx_set_coarse_time_short(struct imx2xx *priv, s32 val)
{
	imx2xx_reg reg_list[2];
	int err;
	struct v4l2_control hdr_control;
	int hdr_en;
	u16 coarse_time_short;
	int i = 0;

	/* check hdr enable ctrl */
	hdr_control.id = V4L2_CID_HDR_EN;

	err = camera_common_g_ctrl(priv->s_data, &hdr_control);
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
		 "%s: val: %d\n", __func__, coarse_time_short);

	imx2xx_get_coarse_time_short_regs(reg_list, coarse_time_short);
	imx2xx_set_group_hold(priv);

	for (i = 0; i < 2; i++) {
		err  = imx2xx_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: COARSE_TIME_SHORT control error\n", __func__);
	return err;
}

static int imx2xx_set_crop_data(struct imx2xx *priv, const struct v4l2_rect *rect)
{
	imx2xx_reg reg_list_crop[IMX2XX_NUM_CROP_REGS];
	int err;
	int i = 0;

	dev_dbg(&priv->i2c_client->dev,
		 "%s:  crop->left:%d, crop->top:%d, crop->res: %dx%d\n",
		 __func__, rect->left, rect->top, rect->width, rect->height);

	imx2xx_get_crop_regs(reg_list_crop, rect);
	imx2xx_set_group_hold(priv);

	err = imx2xx_write_reg(priv->s_data, IMX2XX_MASK_CORRUPT_FRAME_ADDR,
		IMX2XX_MASK_CORRUPT_FRAME_DISABLE);
	if (err)
		dev_err(&priv->i2c_client->dev,
			"%s: SENSOR_CROP: MASK_CORR_FRAME error\n", __func__);

	for (i = 0; i < IMX2XX_NUM_CROP_REGS; i++) {
		err = imx2xx_write_reg(priv->s_data, reg_list_crop[i].addr,
			 reg_list_crop[i].val);
		if (err) {
			dev_dbg(&priv->i2c_client->dev,
				"%s: SENSOR_CROP control error\n", __func__);
			return err;
		}
	}

	return 0;
}

static int imx2xx_get_crop_data(struct imx2xx *priv, struct v4l2_rect *rect)
{
	imx2xx_reg reg_list_crop[IMX2XX_NUM_CROP_REGS];
	int i, err;
	int a, b;
	int right, bottom;

	for (i = 0; i < IMX2XX_NUM_CROP_REGS; i++) {
		reg_list_crop[i].addr = (IMX2XX_CROP_X_START_ADDR_MSB + i);
		err = imx2xx_read_reg(priv->s_data, reg_list_crop[i].addr,
			&reg_list_crop[i].val);
		if (err) {
			dev_dbg(&priv->i2c_client->dev,
				"%s: SENSOR_CROP control error\n", __func__);
			return err;
		}
	}

	a = reg_list_crop[0].val & 0x00ff;
	b = reg_list_crop[1].val & 0x00ff;
	rect->left = (a << 8) | b;

	a = reg_list_crop[2].val & 0x00ff;
	b = reg_list_crop[3].val & 0x00ff;
	rect->top = (a << 8) | b;

	a = reg_list_crop[4].val & 0x00ff;
	b = reg_list_crop[5].val & 0x00ff;
	right = (a << 8) | b;
	rect->width = right - rect->left + 1;

	a = reg_list_crop[6].val & 0x00ff;
	b = reg_list_crop[7].val & 0x00ff;
	bottom = (a << 8) | b;
	rect->height = bottom - rect->top + 1;

	return 0;
}

static int imx2xx_eeprom_device_release(struct imx2xx *priv)
{
	/* Unregister eeprom device */
	if (priv->eeprom[0].i2c_client != NULL) {
		i2c_unregister_device(priv->eeprom[0].i2c_client);
		priv->eeprom[0].i2c_client = NULL;
	}

	return 0;
}

static int imx2xx_eeprom_device_init(struct imx2xx *priv)
{
	char *dev_name = "eeprom_imx2xx";
	static struct regmap_config eeprom_regmap_config;
	int i;
	int err;
	struct i2c_client *client;

	/* Init regmap config */
	eeprom_regmap_config.reg_bits = priv->eeprom_regaddr_width;
	eeprom_regmap_config.val_bits = priv->eeprom_regval_width;

	for (i = 0; i < IMX2XX_EEPROM_NUM_BLOCKS; i++) {
		priv->eeprom[i].adap = i2c_get_adapter(
				priv->i2c_client->adapter->nr);

		memset(&priv->eeprom[i].brd, 0, sizeof(priv->eeprom[i].brd));

		strncpy(priv->eeprom[i].brd.type, dev_name,
				sizeof(priv->eeprom[i].brd.type));
		priv->eeprom[i].brd.addr = IMX2XX_EEPROM_ADDRESS;

		/* Create new eeprom device and regmap for first block.
		 * Other blocks point to same thing */
		if (i == 0) {
			client = i2c_new_device(priv->eeprom[i].adap,
					&priv->eeprom[i].brd);
			if (client) {
				priv->eeprom[i].i2c_client = client;
				priv->eeprom[i].regmap = devm_regmap_init_i2c(
					priv->eeprom[i].i2c_client,
					&eeprom_regmap_config);
				if (IS_ERR(priv->eeprom[i].regmap)) {
					dev_err(&priv->i2c_client->dev,
						"eeprom regmap failed\n");
					err = PTR_ERR(priv->eeprom[i].regmap);
					imx2xx_eeprom_device_release(priv);
					return err;
				}
			} else {
				dev_err(&priv->i2c_client->dev,
					"can't add eeprom i2c device 0x%x\n",
					IMX2XX_EEPROM_ADDRESS);
				return -ENODEV;
			}
		} else {
			priv->eeprom[i].i2c_client = priv->eeprom[0].i2c_client;
			priv->eeprom[i].regmap = priv->eeprom[0].regmap;
		}
	}

	return 0;
}

static int imx2xx_read_eeprom(struct imx2xx *priv,
				struct v4l2_ctrl *ctrl)
{
	int err, i;

	for (i = 0; i < IMX2XX_EEPROM_NUM_BLOCKS; i++) {
		err = regmap_bulk_read(priv->eeprom[i].regmap,
			(i * IMX2XX_EEPROM_BLOCK_SIZE),
			&priv->eeprom_buf[i * IMX2XX_EEPROM_BLOCK_SIZE],
			IMX2XX_EEPROM_BLOCK_SIZE);
		if (err)
			return err;
	}

	for (i = 0; i < IMX2XX_EEPROM_SIZE; i++) {
		sprintf(&ctrl->string[i*2], "%02x",
			priv->eeprom_buf[i]);

		/* Print the ASCII string in first block */
		if (i == 0) {
			dev_dbg(&priv->i2c_client->dev, "eeprom:%c%c%c%c%c%c\n",
				priv->eeprom_buf[0], priv->eeprom_buf[1],
				priv->eeprom_buf[2], priv->eeprom_buf[3],
				priv->eeprom_buf[4], priv->eeprom_buf[5]);
		}
	}

	return 0;
}

static int imx2xx_write_eeprom(struct imx2xx *priv,
				char *string)
{
	int err;
	int i;
	u8 curr[3];
	unsigned long data;

	for (i = 0; i < IMX2XX_EEPROM_SIZE; i++) {
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
				   i & (IMX2XX_EEPROM_BLOCK_SIZE-1), (u8)data);
		if (err)
			return err;
		msleep(20);
	}
	return 0;
}

static int imx2xx_read_otp_page(struct imx2xx *priv,
				u8 *buf, int page, u16 addr, int size)
{
	u8 status;
	int err;

	err = imx2xx_write_reg(priv->s_data, IMX2XX_OTP_PAGE_NUM_ADDR, page);
	if (err)
		return err;
	err = imx2xx_write_reg(priv->s_data, IMX2XX_OTP_CTRL_ADDR, 0x01);
	if (err)
		return err;
	err = imx2xx_read_reg(priv->s_data, IMX2XX_OTP_STATUS_ADDR, &status);
	if (err)
		return err;
	if (status == IMX2XX_OTP_STATUS_IN_PROGRESS) {
		dev_err(&priv->i2c_client->dev,
			"another OTP read in progress\n");
		return err;
	}

	err = regmap_bulk_read(priv->regmap, addr, buf, size);
	if (err)
		return err;

	err = imx2xx_read_reg(priv->s_data, IMX2XX_OTP_STATUS_ADDR, &status);
	if (err)
		return err;
	if (status == IMX2XX_OTP_STATUS_READ_FAIL) {
		dev_err(&priv->i2c_client->dev, "fuse id read error\n");
		return err;
	}

	return 0;
}

static int imx2xx_otp_setup(struct imx2xx *priv)
{
	int err = 0;
	int i;
	struct v4l2_ctrl *ctrl;
	u8 otp_buf[IMX2XX_OTP_SIZE];

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	for (i = 0; i < IMX2XX_OTP_NUM_PAGES; i++) {
		err = imx2xx_read_otp_page(priv,
				   &otp_buf[i * IMX2XX_OTP_PAGE_SIZE],
				   i,
				   IMX2XX_OTP_PAGE_START_ADDR,
				   IMX2XX_OTP_PAGE_SIZE);
		if (err) {
			dev_err(&priv->i2c_client->dev,
				"otp page read error.\n");
			goto setup_error;
		}
	}

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, V4L2_CID_OTP_DATA);
	if (!ctrl) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		err = -EINVAL;
		goto setup_error;
	}

	for (i = 0; i < IMX2XX_OTP_SIZE; i++)
		sprintf(&ctrl->string[i*2], "%02x",
			otp_buf[i]);
	ctrl->cur.string = ctrl->string;

setup_error:
	camera_common_s_power(priv->subdev, false);

	return err;
}

static int imx2xx_fuse_id_setup(struct imx2xx *priv)
{
	int err;
	int i;
	struct v4l2_ctrl *ctrl;
	u8 fuse_id[30];

	if (IMX2XX_FUSE_ID_SIZE > sizeof(fuse_id)) {
		dev_err(&priv->i2c_client->dev,
			"fuse id value too big\n");
		return -EINVAL;
	}

	err = camera_common_s_power(priv->subdev, true);
	if (err)
		return -ENODEV;

	err = imx2xx_read_otp_page(priv,
			   &fuse_id[0],
			   IMX2XX_FUSE_ID_OTP_PAGE,
			   IMX2XX_FUSE_ID_OTP_ROW_ADDR,
			   IMX2XX_FUSE_ID_SIZE);
	if (err) {
		dev_err(&priv->i2c_client->dev,
			"fuse id otp page read error.\n");
		goto setup_error;
	}

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, V4L2_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		err = -EINVAL;
		goto setup_error;
	}

	for (i = 0; i < IMX2XX_FUSE_ID_SIZE; i++) {
		sprintf(&ctrl->string[i*2], "%02x",
			fuse_id[i]);
		dev_dbg(&priv->i2c_client->dev, "fuse_id[%d] %02x\n",
			i, fuse_id[i]);
	}
	ctrl->cur.string = ctrl->string;

setup_error:
	camera_common_s_power(priv->subdev, false);

	return err;
}

static int imx2xx_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx2xx *priv =
		container_of(ctrl->handler, struct imx2xx, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EEPROM_DATA:
		err = imx2xx_read_eeprom(priv, ctrl);
		if (err)
			return err;
		break;
	default:
			pr_err("%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int imx2xx_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx2xx *priv =
		container_of(ctrl->handler, struct imx2xx, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		err = imx2xx_set_gain(priv, ctrl->val);
		break;
	case V4L2_CID_FRAME_LENGTH:
		err = imx2xx_set_frame_length(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME:
		err = imx2xx_set_coarse_time(priv, ctrl->val);
		break;
	case V4L2_CID_COARSE_TIME_SHORT:
		err = imx2xx_set_coarse_time_short(priv, ctrl->val);
		break;
	case V4L2_CID_GROUP_HOLD:
		if (switch_ctrl_qmenu[ctrl->val] == SWITCH_ON) {
			priv->group_hold_en = true;
		} else {
			priv->group_hold_en = false;
			err = imx2xx_set_group_hold(priv);
		}
		break;
	case V4L2_CID_EEPROM_DATA:
		if (!ctrl->string[0])
			break;
		err = imx2xx_write_eeprom(priv, ctrl->string);
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

static int imx2xx_ctrls_init(struct imx2xx *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	numctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

	/* Fix-up otp, fuse-id sizes, defaults */
	for (i = 0; i < numctrls; i++) {
		switch (ctrl_config_list[i].id) {
		case V4L2_CID_OTP_DATA:
			ctrl_config_list[i].max = IMX2XX_OTP_STR_SIZE;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].max);
		      break;
		case V4L2_CID_FUSE_ID:
			ctrl_config_list[i].max = IMX2XX_FUSE_ID_STR_SIZE;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].max);
			break;
		case V4L2_CID_GAIN:
			ctrl_config_list[i].def = IMX2XX_DEFAULT_GAIN;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].def);
			break;
		case V4L2_CID_FRAME_LENGTH:
			ctrl_config_list[i].def = IMX2XX_DEFAULT_FRAME_LENGTH;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].def);
			break;
		case V4L2_CID_COARSE_TIME:
			ctrl_config_list[i].def =
			    IMX2XX_DEFAULT_EXPOSURE_COARSE;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].def);
			break;
		case V4L2_CID_COARSE_TIME_SHORT:
			ctrl_config_list[i].def =
			    IMX2XX_DEFAULT_EXPOSURE_COARSE;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].def);
			break;
		case V4L2_CID_EEPROM_DATA:
			ctrl_config_list[i].max = IMX2XX_EEPROM_STR_SIZE;
			pr_debug("v4l2 ctrl update %X=%d\n",
			ctrl_config_list[i].id, ctrl_config_list[i].def);
			break;
		}
	}

	for (i = 0; i < numctrls; i++) {
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

	priv->numctrls = numctrls;
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

	err = imx2xx_otp_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading otp data\n", err);
		goto error;
	}

	err = imx2xx_fuse_id_setup(priv);
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

MODULE_DEVICE_TABLE(of, imx2xx_of_match);

static int imx2xx_calculate_default_mode(u32 width, u32 height,
			  const struct camera_common_frmfmt *frmfmt, int cnt)
{
	int i = 0;

	for (i = 0; i < cnt; i++) {
		if ((frmfmt[i].size.width == width) &&
			(frmfmt[i].size.height == height)) {
			pr_debug("[IMX2XX]: default mode :%d\n",
				 frmfmt[i].mode);
			return frmfmt[i].mode;
		}
	}

	pr_info("[IMX2XX]: no matching mode found for default w & h\n");
	return frmfmt[0].mode;
}

static struct camera_common_pdata *imx2xx_parse_dt(struct i2c_client *client,
						   struct imx2xx *priv)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	struct camera_common_data *s_data = priv->s_data;
	const struct of_device_id *match;
	int sts;

	match = of_match_device(imx2xx_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	/* Assign sensor-specific data */
	priv->sensor_data = match->data;

	dev_info(&client->dev, "imx sensor name %s\n",
		 priv->sensor_data->imx_name);

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	sts = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (sts)
		dev_err(&client->dev, "mclk not found %d\n", sts);

	sts = of_get_named_gpio(np, "pwdn-gpios", 0);
	if (sts >= 0)
		board_priv_pdata->pwdn_gpio = sts;
	else
		dev_dbg(&client->dev, "pwdn-gpio not found %d\n", sts);
	sts = of_get_named_gpio(np, "reset-gpios", 0);
	if (sts >= 0)
		board_priv_pdata->reset_gpio = sts;
	else
		dev_dbg(&client->dev, "reset-gpio not found %d\n", sts);
	sts = of_get_named_gpio(np, "af-gpios", 0);
	if (sts >= 0)
		board_priv_pdata->af_gpio = sts;
	else
		dev_dbg(&client->dev, "af-gpio not found %d\n", sts);

	sts = of_property_read_string(np, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	if (sts)
		dev_dbg(&client->dev, "avdd-reg not found %d\n", sts);
	sts = of_property_read_string(np, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	if (sts)
		dev_dbg(&client->dev, "dvdd-reg not found %d\n", sts);
	sts = of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (sts)
		dev_dbg(&client->dev, "iovdd-reg not found %d\n", sts);

	/* Read the defaults */
	sts = of_property_read_u32(np, "default-clk", &s_data->def_clk_freq);
	if (sts)
		dev_info(&client->dev, "default-clk not found %d\n", sts);
	sts = of_property_read_u32(np, "default-width", &s_data->def_width);
	if (sts)
		dev_info(&client->dev, "default-width not found %d\n", sts);
	sts = of_property_read_u32(np, "default-height", &s_data->def_height);
	if (sts)
		dev_info(&client->dev, "default-height not found %d\n", sts);
	s_data->fmt_width		= s_data->def_width;
	s_data->fmt_height		= s_data->def_height;

	s_data->def_mode = imx2xx_calculate_default_mode(
			s_data->def_width, s_data->def_height,
			priv->sensor_data->imx_frmfmt,
			priv->sensor_data->imx_frmcnt);

	sts = of_property_read_u32(np, "default-gain", &priv->default_gain);
	if (sts)
		dev_info(&client->dev, "default-gain not found %d\n", sts);
	sts = of_property_read_u32(np, "default-fl", &priv->default_fl);
	if (sts)
		dev_info(&client->dev, "default-fl not found %d\n", sts);
	sts = of_property_read_u32(np, "default-ct", &priv->default_ct);
	if (sts)
		dev_info(&client->dev, "default-ct not found %d\n", sts);

	/* Read the eeprom info */
	board_priv_pdata->has_eeprom = of_property_read_bool(np, "has-eeprom");

	if (board_priv_pdata->has_eeprom) {
		sts = of_property_read_u32(np, "eeprom-addr",
						&priv->eeprom_addr);
		if (sts)
			dev_err(&client->dev, "eeprom-addr not found %d\n",
				sts);

		sts = of_property_read_u32(np, "eeprom-size",
						&priv->eeprom_size);
		if (sts)
			dev_err(&client->dev, "eeprom-size not found %d\n",
				sts);

		sts = of_property_read_u32(np, "eeprom-blk-size",
						&priv->eeprom_blk_size);
		if (sts)
			dev_err(&client->dev, "eeprom-blk-size not found %d\n",
				sts);

		sts = of_property_read_u32(np, "eeprom-regaddr-width",
						&priv->eeprom_regaddr_width);
		if (sts)
			dev_err(&client->dev,
				"eeprom-regaddr-width not found %d\n", sts);

		sts = of_property_read_u32(np, "eeprom-regval-width",
						&priv->eeprom_regval_width);
		if (sts)
			dev_err(&client->dev,
				"eeprom-regval-width not found %d\n", sts);
	}

	return board_priv_pdata;
}

static int imx2xx_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_dbg(&client->dev, "%s:\n", __func__);


	return 0;
}

static const struct v4l2_subdev_internal_ops imx2xx_subdev_internal_ops = {
	.open = imx2xx_open,
};

static const struct media_entity_operations imx2xx_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx2xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx2xx *priv;
	char debugfs_name[10];
	int err;

	pr_info("[IMX2XX]: probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx2xx) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}
	priv->s_data	= common_data;

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = imx2xx_parse_dt(client, priv);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	priv->eeprom_buf = devm_kzalloc(&client->dev,
			    sizeof(u8) * IMX2XX_EEPROM_SIZE,
			    GFP_KERNEL);
	if (!priv->eeprom_buf) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->eeprom = devm_kzalloc(&client->dev,
				sizeof(struct camera_common_eeprom_data)
				* IMX2XX_EEPROM_NUM_BLOCKS,
				GFP_KERNEL);
	if (!priv->eeprom) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	common_data->ops		= &imx2xx_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= priv->sensor_data->imx_frmfmt;
	common_data->color_fmts		= priv->sensor_data->imx_color_fmts;
	common_data->colorfmt		= camera_common_find_datafmt(
					  V4L2_MBUS_FMT_SRGGB10_1X10);
	common_data->ctrls		= priv->ctrls;
	common_data->power		= &priv->power;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= priv->sensor_data->imx_frmcnt;
	common_data->num_color_fmts	= priv->sensor_data->imx_num_color_fmts;

	priv->i2c_client		= client;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;
	priv->s_data->dev		= &client->dev;

	err = imx2xx_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(client, common_data);
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}
	sprintf(debugfs_name, "imx2xx_%c", common_data->csi_port + 'a');
	dev_dbg(&client->dev, "%s: name %s\n", __func__, debugfs_name);
	camera_common_create_debugfs(common_data, debugfs_name);

	v4l2_i2c_subdev_init(priv->subdev, client, &imx2xx_subdev_ops);

	err = imx2xx_ctrls_init(priv);
	if (err)
		return err;

	/* eeprom interface */
	err = imx2xx_eeprom_device_init(priv);
	if (err)
		dev_err(&client->dev,
			"Failed to allocate eeprom register map: %d\n", err);

	priv->subdev->internal_ops = &imx2xx_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops = &imx2xx_media_ops;
	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_dbg(&client->dev, "Detected IMX2XX sensor\n");

	return 0;
}

static int
imx2xx_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct imx2xx *priv = (struct imx2xx *)s_data->priv;
	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx2xx_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id imx2xx_id[] = {
	{ "imx2xx", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx2xx_id);

static struct i2c_driver imx2xx_i2c_driver = {
	.driver = {
		.name = "imx2xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx2xx_of_match),
	},
	.probe = imx2xx_probe,
	.remove = imx2xx_remove,
	.id_table = imx2xx_id,
};

module_i2c_driver(imx2xx_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Sony IMX2XX");
MODULE_AUTHOR("David Wang <davidw@nvidia.com>");
MODULE_LICENSE("GPL v2");
