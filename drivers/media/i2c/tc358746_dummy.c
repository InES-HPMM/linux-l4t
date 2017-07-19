/*
 * tc358746_dummy.c - Dummy driver for Toshiba parallel to CSI bridge
 *
 * Copyright (c) 2017, Armin Weiss <weii@zhaw.ch>
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


#include <linux/module.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/interrupt.h>

#include <mach/io_dpd.h>

#include <linux/videodev2.h>

#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>
#include <media/soc_camera.h>


static int debug = 3;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");


/*______________________________________________________________________________
 *  Registers
 */

#define CHIPID_ADDR				0x0000
#define MASK_CHIPID				0xFF00
#define MASK_REVID				0x00FF
#define TC358746_CHIPID				0x4401

/*______________________________________________________________________________
 *  Structs
 */

struct tc358746_platform_data {
	uint32_t csi_lanes;
};

struct tc358746_state {
	/* Driver */
	struct tc358746_platform_data *pdata;
	struct i2c_client *i2c_client;

	/* V4L2 */
	struct v4l2_subdev sd;
	struct v4l2_subdev_format format;
};

/*______________________________________________________________________________
 *  Helper Functions
 */

static inline struct tc358746_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358746_state, sd);
}

/*______________________________________________________________________________
 *  Pad Ops
 */

static int tc358746_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *format)
{
	struct tc358746_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *fmt;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	memcpy(format, &state->format, sizeof(struct v4l2_subdev_format));

	fmt = &format->format;
	v4l2_dbg(3, debug, sd,
		"%s(): width=%d, height=%d, code=0x%04X, field=%d, colorspace=%d\n",
		__func__, fmt->width, fmt->height, fmt->code, fmt->field, fmt->colorspace);

	return 0;
}

static int tc358746_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_format *format)
{
	struct tc358746_state *state = to_state(sd);
	u32 code = format->format.code;
	u32 width = format->format.width;
	u32 height = format->format.height;
	u32 colorspace = format->format.colorspace;
	u32 field = format->format.field;
	bool isTry;
	int ret = 0;

	isTry = (format->which == V4L2_SUBDEV_FORMAT_TRY);

	v4l2_dbg(3, debug, sd, "%s(): %s\n", __func__, isTry ? "TRY" : "ACTIVE");
	v4l2_dbg(3, debug, sd,
		"    code: %d, width: %d, height: %d, colorspace: %d, field: %d\n",
		code, width, height, colorspace, field);

	format->format.code = code;

	/* Check data format code */
	switch (code) {
	case V4L2_MBUS_FMT_UYVY8_1X16:
		break;
	default:
		ret = -EINVAL;
	}
	if (ret) {
		v4l2_dbg(3, debug, sd, "%s(): Invalid format code: %d\n",
			__func__, code);
		return ret;
	}

	/*
	 * Check image size
	 * When Gstreamer calls try_fmt_vid_cap IOCTL it sets width/heigt to
	 * either 1 or 32768. This is ok in "try" but not in "active" mode.
	 */
	switch(height) {
	case 32768:
		if (!isTry || width != 32768)
			ret = -EINVAL;
		break;
	case 1080:
		if (width != 1920)
			ret = -EINVAL;
		break;
	case 720:
		if (width != 1280)
			ret = -EINVAL;
		break;
	case 1:
		if (!isTry || width != 1)
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}
	if (ret) {
		v4l2_dbg(3, debug, sd, "%s(): Invalid image size: %dx%d\n",
			__func__, width, height);
		return ret;
	}

	/* Check field (bottom/top) */
	switch(field) {
	case V4L2_FIELD_NONE:	/* Progressive, non-interlaced mode */
		break;
	default:
		ret = -EINVAL;
	}
	if (ret) {
		v4l2_dbg(3, debug, sd, "%s(): Invalid field: %d\n",
			__func__, field);
		return ret;
	}

	/* Check colorspace */
	switch(colorspace) {
	case V4L2_COLORSPACE_DEFAULT:	/* Driver can choose colorspace */
		break;
	case V4L2_COLORSPACE_SMPTE170M:	/* BT.601 */
		break;
	default:
		ret = -EINVAL;
	}
	if (ret) {
		v4l2_dbg(3, debug, sd, "%s(): Invalid colorspace: %d\n",
			__func__, colorspace);
		return ret;
	}

	if (isTry)
		return 0;


	v4l2_dbg(3, debug, sd, "%s(): format->which=%d\n",
		__func__, format->which);

	/* Save format */
	memcpy(&state->format, format, sizeof(struct v4l2_subdev_format));

	return 0;
}


/*______________________________________________________________________________
 *  Subdev Video Ops
 */

static int tc358746_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	v4l2_dbg(3, debug, sd, "%s(): Always ok\n", __func__);

	return 0;
}

static int tc358746_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *cfg)
{
	struct tc358746_state *state = to_state(sd);
	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	cfg->type = V4L2_MBUS_CSI2;

	/* Support for non-continuous CSI-2 clock is missing in the driver */
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK | V4L2_MBUS_CSI2_CHANNEL_0;

	switch (state->pdata->csi_lanes) {
	case 1:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		break;
	case 2:
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
		break;
	case 3:
		cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
		break;
	case 4:
		cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
		break;
	default:
		return -EINVAL;
	}

	v4l2_dbg(2, debug, sd, "%s: Lanes: 0x%02X\n",
		__func__, cfg->flags & 0x0F);

	return 0;
}

static int tc358746_s_stream(struct v4l2_subdev *sd, int enable)
{
	v4l2_dbg(3, debug, sd, "%s(): Always on\n", __func__);

	return 0;
}

static int tc358746_s_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format format;

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	memcpy(&format.format, fmt, sizeof(struct v4l2_mbus_framefmt));
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.pad = 0;

	return tc358746_set_fmt(sd, NULL, &format);
}

static int tc358746_g_mbus_fmt(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format format;
	int ret;

	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	format.pad = 0;
	ret = tc358746_get_fmt(sd, NULL, &format);
	if (ret)
		return ret;

	memcpy(fmt, &format.format, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tc358746_try_mbus_fmt(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_subdev_format format;

	v4l2_dbg(2, debug, sd,
		"%s(): width=%d, height=%d, code=0x%04X, field=%d, colorspace=%d\n",
		__func__, fmt->width, fmt->height, fmt->code, fmt->field,
		fmt->colorspace);

	memcpy(&format.format, fmt, sizeof(struct v4l2_mbus_framefmt));
	format.which = V4L2_SUBDEV_FORMAT_TRY;
	format.pad = 0;

	return tc358746_set_fmt(sd, NULL, &format);
}

// In newer V4L2 patches, enum_mbus_fmt was replaced by enum_mbus_code
static int tc358746_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	if (index >= 1)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_UYVY8_1X16;

	return 0;
}


/*______________________________________________________________________________
 *  Subdev Core Ops
 */

static int tc358746_g_chip_ident(struct v4l2_subdev *sd,
				 struct v4l2_dbg_chip_ident *id)
{
	v4l2_dbg(2, debug, sd, "%s()\n", __func__);

	id->ident = (TC358746_CHIPID >> 8) & 0xff;
	id->revision = TC358746_CHIPID & 0xff;

	return 0;
}

static int tc358746_s_power(struct v4l2_subdev *sd, int on)
{
	v4l2_dbg(2, debug, sd, "%s: Always on\n", __func__);

	return 0;
}


/*______________________________________________________________________________
 *  Ops Struct
 */

static struct v4l2_subdev_video_ops tc358746_subdev_video_ops = {
	.g_input_status = tc358746_g_input_status,
	.s_stream = tc358746_s_stream,
	.g_mbus_config = tc358746_g_mbus_config,
	.s_mbus_fmt = tc358746_s_mbus_fmt,
	.g_mbus_fmt = tc358746_g_mbus_fmt,
	.try_mbus_fmt = tc358746_try_mbus_fmt,
	.enum_mbus_fmt = tc358746_enum_mbus_fmt,
};

static struct v4l2_subdev_core_ops tc358746_subdev_core_ops = {
	.g_chip_ident = tc358746_g_chip_ident,
	.s_power = tc358746_s_power,
};

static const struct v4l2_subdev_pad_ops tc358746_pad_ops = {
	.set_fmt = tc358746_set_fmt,
	.get_fmt = tc358746_get_fmt,
};

static struct v4l2_subdev_ops tc358746_ops = {
	.core = &tc358746_subdev_core_ops,
	.video = &tc358746_subdev_video_ops,
	.pad = &tc358746_pad_ops,
};


/*______________________________________________________________________________
 *  Probe / Remove
 */

static void tc358746_initial_setup(struct v4l2_subdev *sd)
{
	struct v4l2_subdev_format format;

	v4l2_dbg(3, debug, sd, "%s():\n", __func__);

	/* Set default format */
	format.format.code = V4L2_MBUS_FMT_UYVY8_1X16;
	format.format.width = 1920;
	format.format.height = 1080;
	format.format.colorspace = V4L2_COLORSPACE_SMPTE170M;
	format.format.field = V4L2_FIELD_NONE;
	tc358746_set_fmt(sd, NULL, &format);
}

#ifdef CONFIG_OF
static struct tc358746_platform_data *of_tc358746(struct i2c_client *client,
						  struct device_node *node)
{
	struct device *dev = &client->dev;
	struct tc358746_platform_data *pdata;
	u32 ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	v4l_dbg(1, debug, client, "Device Tree Parameters:\n");

	ret = of_property_read_u32(node, "csi-lanes", &pdata->csi_lanes);
	if (ret)
		goto not_found;
	v4l_dbg(1, debug, client, "csi-lanes = %d\n", pdata->csi_lanes);

	return pdata;


not_found:
	devm_kfree(dev, pdata);
	return NULL;
}
#endif

static int tc358746_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tc358746_state *state;
	struct tc358746_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;

#ifdef CONFIG_OF
	struct device_node *node = client->dev.of_node;
#endif

	state = devm_kzalloc(&client->dev, sizeof(struct tc358746_state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

/* platform data */
#ifdef CONFIG_OF
	if (!pdata && node)
		pdata = of_tc358746(client, node);
#endif

	if (!pdata) {
		v4l_err(client, "No platform data!\n");
		return -ENODEV;
	}

	state->pdata = pdata;
	state->i2c_client = client;
	sd = &state->sd;
	sd->of_node = node;

	i2c_set_clientdata(client, state);

	v4l2_i2c_subdev_init(sd, client, &tc358746_ops);
	// sd->flags |= V4L2_SUBDEV_FL_IS_I2C;

	/* Initial Setup */
	tc358746_initial_setup(sd);

	return v4l2_async_register_subdev(sd);
}

static int tc358746_remove(struct i2c_client *client)
{
	v4l_dbg(1, debug, client, "%s()\n", __func__);

	return 0;
}

static const struct i2c_device_id tc358746_id[] = {
	{ "tc358746", 0 },
	{ "tc358748", 1 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tc358746_id);

#ifdef CONFIG_OF
static const struct of_device_id tc358746_of_table[] = {
	{ .compatible = "toshiba,tc358746xbg" },
	{ .compatible = "toshiba,tc358748xbg" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358746_of_table);
#endif

static struct i2c_driver tc358746_driver = {
	.driver = {
		.of_match_table = of_match_ptr(tc358746_of_table),
		.name = "tc358746",
		.owner = THIS_MODULE,
	},
	.probe = tc358746_probe,
	.remove = tc358746_remove,
	.id_table = tc358746_id,
};
module_i2c_driver(tc358746_driver);

MODULE_DESCRIPTION("Driver for Toshiba TC358746 and TC358748 Parallel CSI-2 Bridge");
MODULE_AUTHOR("Armin Weiss (weii@zhaw.ch)");
MODULE_LICENSE("GPL v2");
