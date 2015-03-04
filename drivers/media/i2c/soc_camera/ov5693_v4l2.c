/*
 * ov5693.c - ov5693 sensor driver
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

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <media/ov5693.h>

struct ov5693_reg {
	u16 addr;
	u8 val;
};

struct ov5693_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct ov5693 {
	struct v4l2_subdev		subdev;
	const struct ov5693_datafmt	*fmt;

	int				mode;
	int				gpio_pwdn;
	struct ov5693_power_rail	power;
	struct i2c_client		*i2c_client;
	struct ov5693_v4l2_platform_data	*pdata;
	struct clk			*mclk;
	struct dentry			*debugdir;
};

static const struct ov5693_datafmt ov5693_colour_fmts[] = {
	{V4L2_MBUS_FMT_SRGGB10_1X10, V4L2_COLORSPACE_SRGB},
	{V4L2_MBUS_FMT_SRGGB8_1X8, V4L2_COLORSPACE_SRGB},
};

static inline struct ov5693 *to_ov5693(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov5693, subdev);
}

#define OV5693_TABLE_WAIT_MS 0
#define OV5693_TABLE_END 1
#define OV5693_MAX_RETRIES 3
#define OV5693_WAIT_MS 10

#define MAX_BUFFER_SIZE 32
#define OV5693_FRAME_LENGTH_ADDR_MSB 0x0340
#define OV5693_FRAME_LENGTH_ADDR_LSB 0x0341
#define OV5693_COARSE_TIME_ADDR_MSB 0x0202
#define OV5693_COARSE_TIME_ADDR_LSB 0x0203
#define OV5693_COARSE_TIME_SHORT_ADDR_MSB 0x0230
#define OV5693_COARSE_TIME_SHORT_ADDR_LSB 0x0231
#define OV5693_GAIN_ADDR 0x0205
#define OV5693_GAIN_SHORT_ADDR 0x0233

static struct ov5693_reg mode_2592x1944[] = {
	{0x0103, 0x01},
	{0x3001, 0x0a},
	{0x3002, 0x80},
	{0x3006, 0x00},
	{0x3011, 0x21},
	{0x3012, 0x09},
	{0x3013, 0x10},
	{0x3014, 0x00},
	{0x3015, 0x08},
	{0x3016, 0xf0},
	{0x3017, 0xf0},
	{0x3018, 0xf0},
	{0x301b, 0xb4},
	{0x301d, 0x02},
	{0x3021, 0x00},
	{0x3022, 0x01},
	{0x3028, 0x44},
	{0x3090, 0x02},
	{0x3091, 0x0e},
	{0x3092, 0x00},
	{0x3093, 0x00},
	{0x3098, 0x03},
	{0x3099, 0x1e},
	{0x309a, 0x02},
	{0x309b, 0x01},
	{0x309c, 0x00},
	{0x30a0, 0xd2},
	{0x30a2, 0x01},
	{0x30b2, 0x00},
	{0x30b3, 0x68},
	{0x30b4, 0x03},
	{0x30b5, 0x04},
	{0x30b6, 0x01},
	{0x3104, 0x21},
	{0x3106, 0x00},
	{0x3400, 0x04},
	{0x3401, 0x00},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0x00},
	{0x3406, 0x01},
	{0x3500, 0x00},
	{0x3501, 0x7b},
	{0x3502, 0x00},
	{0x3503, 0x07},
	{0x3504, 0x00},
	{0x3505, 0x00},
	{0x3506, 0x00},
	{0x3507, 0x02},
	{0x3508, 0x00},
	{0x3509, 0x10},
	{0x350a, 0x00},
	{0x350b, 0x40},
	{0x3601, 0x0a},
	{0x3602, 0x18},
	{0x3612, 0x80},
	{0x3620, 0x54},
	{0x3621, 0xc7},
	{0x3622, 0x0f},
	{0x3625, 0x10},
	{0x3630, 0x55},
	{0x3631, 0xf4},
	{0x3632, 0x00},
	{0x3633, 0x34},
	{0x3634, 0x02},
	{0x364d, 0x0d},
	{0x364f, 0xdd},
	{0x3660, 0x04},
	{0x3662, 0x10},
	{0x3663, 0xf1},
	{0x3665, 0x00},
	{0x3666, 0x20},
	{0x3667, 0x00},
	{0x366a, 0x80},
	{0x3680, 0xe0},
	{0x3681, 0x00},
	{0x3700, 0x42},
	{0x3701, 0x14},
	{0x3702, 0xa0},
	{0x3703, 0xd8},
	{0x3704, 0x78},
	{0x3705, 0x02},
	{0x3708, 0xe2},
	{0x3709, 0xc3},
	{0x370a, 0x00},
	{0x370b, 0x20},
	{0x370c, 0x0c},
	{0x370d, 0x11},
	{0x370e, 0x00},
	{0x370f, 0x40},
	{0x3710, 0x00},
	{0x371a, 0x1c},
	{0x371b, 0x05},
	{0x371c, 0x01},
	{0x371e, 0xa1},
	{0x371f, 0x0c},
	{0x3721, 0x00},
	{0x3726, 0x00},
	{0x372a, 0x01},
	{0x3730, 0x10},
	{0x3738, 0x22},
	{0x3739, 0xe5},
	{0x373a, 0x50},
	{0x373b, 0x02},
	{0x373c, 0x41},
	{0x373f, 0x02},
	{0x3740, 0x42},
	{0x3741, 0x02},
	{0x3742, 0x18},
	{0x3743, 0x01},
	{0x3744, 0x02},
	{0x3747, 0x10},
	{0x374c, 0x04},
	{0x3751, 0xf0},
	{0x3752, 0x00},
	{0x3753, 0x00},
	{0x3754, 0xc0},
	{0x3755, 0x00},
	{0x3756, 0x1a},
	{0x3758, 0x00},
	{0x3759, 0x0f},
	{0x376b, 0x44},
	{0x375c, 0x04},
	{0x3776, 0x00},
	{0x377f, 0x08},
	{0x3780, 0x22},
	{0x3781, 0x0c},
	{0x3784, 0x2c},
	{0x3785, 0x1e},
	{0x378f, 0xf5},
	{0x3791, 0xb0},
	{0x3795, 0x00},
	{0x3796, 0x64},
	{0x3797, 0x11},
	{0x3798, 0x30},
	{0x3799, 0x41},
	{0x379a, 0x07},
	{0x379b, 0xb0},
	{0x379c, 0x0c},
	{0x37c5, 0x00},
	{0x37c6, 0x00},
	{0x37c7, 0x00},
	{0x37c9, 0x00},
	{0x37ca, 0x00},
	{0x37cb, 0x00},
	{0x37de, 0x00},
	{0x37df, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x0a},
	{0x380d, 0x80},
	{0x380e, 0x07},
	{0x380f, 0xc0},
	{0x3810, 0x00},
	{0x3811, 0x02},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x1e},
	{0x3823, 0x00},
	{0x3824, 0x00},
	{0x3825, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x00},
	{0x382a, 0x04},
	{0x3a04, 0x06},
	{0x3a05, 0x14},
	{0x3a06, 0x00},
	{0x3a07, 0xfe},
	{0x3b00, 0x00},
	{0x3b02, 0x00},
	{0x3b03, 0x00},
	{0x3b04, 0x00},
	{0x3b05, 0x00},
	{0x3d00, 0x00},
	{0x3d01, 0x00},
	{0x3d02, 0x00},
	{0x3d03, 0x00},
	{0x3d04, 0x00},
	{0x3d05, 0x00},
	{0x3d06, 0x00},
	{0x3d07, 0x00},
	{0x3d08, 0x00},
	{0x3d09, 0x00},
	{0x3d0a, 0x00},
	{0x3d0b, 0x00},
	{0x3d0c, 0x00},
	{0x3d0d, 0x00},
	{0x3d0e, 0x00},
	{0x3d0f, 0x00},
	{0x3d80, 0x00},
	{0x3d81, 0x00},
	{0x3d84, 0x00},
	{0x3e07, 0x20},
	{0x4000, 0x08},
	{0x4001, 0x04},
	{0x4002, 0x45},
	{0x4004, 0x08},
	{0x4005, 0x18},
	{0x4006, 0x20},
	{0x4008, 0x24},
	{0x4009, 0x10},
	{0x400c, 0x00},
	{0x400d, 0x00},
	{0x4058, 0x00},
	{0x4101, 0xb2},
	{0x4303, 0x00},
	{0x4304, 0x08},
	{0x4307, 0x30},
	{0x4311, 0x04},
	{0x4315, 0x01},
	{0x4511, 0x05},
	{0x4512, 0x01},
	{0x4806, 0x00},
	{0x4816, 0x52},
	{0x481f, 0x30},
	{0x4826, 0x2c},
	{0x4831, 0x64},
	{0x4d00, 0x04},
	{0x4d01, 0x71},
	{0x4d02, 0xfd},
	{0x4d03, 0xf5},
	{0x4d04, 0x0c},
	{0x4d05, 0xcc},
	{0x4837, 0x09},
	{0x5000, 0x06},
	{0x5001, 0x01},
	{0x5002, 0x00},
	{0x5003, 0x20},
	{0x5046, 0x0a},
	{0x5013, 0x00},
	{0x5046, 0x0a},
	{0x5780, 0x1c},
	{0x5786, 0x20},
	{0x5787, 0x10},
	{0x5788, 0x18},
	{0x578a, 0x04},
	{0x578b, 0x02},
	{0x578c, 0x02},
	{0x578e, 0x06},
	{0x578f, 0x02},
	{0x5790, 0x02},
	{0x5791, 0xff},
	{0x5842, 0x01},
	{0x5843, 0x2b},
	{0x5844, 0x01},
	{0x5845, 0x92},
	{0x5846, 0x01},
	{0x5847, 0x8f},
	{0x5848, 0x01},
	{0x5849, 0x0c},
	{0x5e00, 0x00},
	{0x5e10, 0x0c},
	{OV5693_TABLE_WAIT_MS, OV5693_WAIT_MS},
	/* stream on */
	{0x0100, 0x01},
	{OV5693_TABLE_END, 0x00}
};

static struct ov5693_reg mode_1920x1080[] = {
	{0x0103, 0x01},
	{0x3001, 0x0a},
	{0x3002, 0x80},
	{0x3006, 0x00},
	{0x3011, 0x21},
	{0x3012, 0x09},
	{0x3013, 0x10},
	{0x3014, 0x00},
	{0x3015, 0x08},
	{0x3016, 0xf0},
	{0x3017, 0xf0},
	{0x3018, 0xf0},
	{0x301b, 0xb4},
	{0x301d, 0x02},
	{0x3021, 0x00},
	{0x3022, 0x01},
	{0x3028, 0x44},
	{0x3098, 0x03},
	{0x3099, 0x1e},
	{0x309a, 0x02},
	{0x309b, 0x01},
	{0x309c, 0x00},
	{0x30a0, 0xd2},
	{0x30a2, 0x01},
	{0x30b2, 0x00},
	{0x30b3, 0x64},
	{0x30b4, 0x03},
	{0x30b5, 0x04},
	{0x30b6, 0x01},
	{0x3104, 0x21},
	{0x3106, 0x00},
	{0x3406, 0x01},
	{0x3500, 0x00},
	{0x3501, 0x7b},
	{0x3502, 0x00},
	{0x3503, 0x07},
	{0x3504, 0x00},
	{0x3505, 0x00},
	{0x3506, 0x00},
	{0x3507, 0x02},
	{0x3508, 0x00},
	{0x3509, 0x10},
	{0x350a, 0x00},
	{0x350b, 0x40},
	{0x3601, 0x0a},
	{0x3602, 0x38},
	{0x3612, 0x80},
	{0x3620, 0x54},
	{0x3621, 0xc7},
	{0x3622, 0x0f},
	{0x3625, 0x10},
	{0x3630, 0x55},
	{0x3631, 0xf4},
	{0x3632, 0x00},
	{0x3633, 0x34},
	{0x3634, 0x02},
	{0x364d, 0x0d},
	{0x364f, 0xdd},
	{0x3660, 0x04},
	{0x3662, 0x10},
	{0x3663, 0xf1},
	{0x3665, 0x00},
	{0x3666, 0x20},
	{0x3667, 0x00},
	{0x366a, 0x80},
	{0x3680, 0xe0},
	{0x3681, 0x00},
	{0x3700, 0x42},
	{0x3701, 0x14},
	{0x3702, 0xa0},
	{0x3703, 0xd8},
	{0x3704, 0x78},
	{0x3705, 0x02},
	{0x3708, 0xe2},
	{0x3709, 0xc3},
	{0x370a, 0x00},
	{0x370b, 0x20},
	{0x370c, 0x0c},
	{0x370d, 0x11},
	{0x370e, 0x00},
	{0x370f, 0x40},
	{0x3710, 0x00},
	{0x371a, 0x1c},
	{0x371b, 0x05},
	{0x371c, 0x01},
	{0x371e, 0xa1},
	{0x371f, 0x0c},
	{0x3721, 0x00},
	{0x3724, 0x10},
	{0x3726, 0x00},
	{0x372a, 0x01},
	{0x3730, 0x10},
	{0x3738, 0x22},
	{0x3739, 0xe5},
	{0x373a, 0x50},
	{0x373b, 0x02},
	{0x373c, 0x41},
	{0x373f, 0x02},
	{0x3740, 0x42},
	{0x3741, 0x02},
	{0x3742, 0x18},
	{0x3743, 0x01},
	{0x3744, 0x02},
	{0x3747, 0x10},
	{0x374c, 0x04},
	{0x3751, 0xf0},
	{0x3752, 0x00},
	{0x3753, 0x00},
	{0x3754, 0xc0},
	{0x3755, 0x00},
	{0x3756, 0x1a},
	{0x3758, 0x00},
	{0x3759, 0x0f},
	{0x376b, 0x44},
	{0x375c, 0x04},
	{0x3774, 0x10},
	{0x3776, 0x00},
	{0x377f, 0x08},
	{0x3780, 0x22},
	{0x3781, 0x0c},
	{0x3784, 0x2c},
	{0x3785, 0x1e},
	{0x378f, 0xf5},
	{0x3791, 0xb0},
	{0x3795, 0x00},
	{0x3796, 0x64},
	{0x3797, 0x11},
	{0x3798, 0x30},
	{0x3799, 0x41},
	{0x379a, 0x07},
	{0x379b, 0xb0},
	{0x379c, 0x0c},
	{0x37c5, 0x00},
	{0x37c6, 0x00},
	{0x37c7, 0x00},
	{0x37c9, 0x00},
	{0x37ca, 0x00},
	{0x37cb, 0x00},
	{0x37de, 0x00},
	{0x37df, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0xf8},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x06},
	{0x3807, 0xab},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x0a},
	{0x380d, 0x80},
	{0x380e, 0x07},
	{0x380f, 0xc0},
	{0x3810, 0x00},
	{0x3811, 0x02},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x1e},
	{0x3823, 0x00},
	{0x3824, 0x00},
	{0x3825, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x00},
	{0x382a, 0x04},
	{0x3a04, 0x06},
	{0x3a05, 0x14},
	{0x3a06, 0x00},
	{0x3a07, 0xfe},
	{0x3b00, 0x00},
	{0x3b02, 0x00},
	{0x3b03, 0x00},
	{0x3b04, 0x00},
	{0x3b05, 0x00},
	{0x3e07, 0x20},
	{0x4000, 0x08},
	{0x4001, 0x04},
	{0x4002, 0x45},
	{0x4004, 0x08},
	{0x4005, 0x18},
	{0x4006, 0x20},
	{0x4008, 0x24},
	{0x4009, 0x10},
	{0x400c, 0x00},
	{0x400d, 0x00},
	{0x4058, 0x00},
	{0x404e, 0x37},
	{0x404f, 0x8f},
	{0x4058, 0x00},
	{0x4101, 0xb2},
	{0x4303, 0x00},
	{0x4304, 0x08},
	{0x4307, 0x30},
	{0x4311, 0x04},
	{0x4315, 0x01},
	{0x4511, 0x05},
	{0x4512, 0x01},
	{0x4800, 0x20},
	{0x4806, 0x00},
	{0x4816, 0x52},
	{0x481f, 0x30},
	{0x4826, 0x2c},
	{0x4831, 0x64},
	{0x4d00, 0x04},
	{0x4d01, 0x71},
	{0x4d02, 0xfd},
	{0x4d03, 0xf5},
	{0x4d04, 0x0c},
	{0x4d05, 0xcc},
	{0x4837, 0x0a},
	{0x5000, 0x06},
	{0x5001, 0x01},
	{0x5002, 0x80},
	{0x5003, 0x20},
	{0x5046, 0x0a},
	{0x5013, 0x00},
	{0x5046, 0x0a},
	{0x5780, 0x1c},
	{0x5786, 0x20},
	{0x5787, 0x10},
	{0x5788, 0x18},
	{0x578a, 0x04},
	{0x578b, 0x02},
	{0x578c, 0x02},
	{0x578e, 0x06},
	{0x578f, 0x02},
	{0x5790, 0x02},
	{0x5791, 0xff},
	{0x5842, 0x01},
	{0x5843, 0x2b},
	{0x5844, 0x01},
	{0x5845, 0x92},
	{0x5846, 0x01},
	{0x5847, 0x8f},
	{0x5848, 0x01},
	{0x5849, 0x0c},
	{0x5e00, 0x00},
	{0x5e10, 0x0c},
	{OV5693_TABLE_WAIT_MS, OV5693_WAIT_MS},
	/* stream on */
	{0x0100, 0x01},
	{OV5693_TABLE_END, 0x00}
};

static struct ov5693_reg tp_colorbars[] = {
	{0x0600, 0x00},
	{0x0601, 0x02},

	{OV5693_TABLE_WAIT_MS, OV5693_WAIT_MS},
	{OV5693_TABLE_END, 0x00}
};

enum {
	OV5693_MODE_2592X1944,
	OV5693_MODE_1920X1080,
	OV5693_MODE_INVALID
};

static struct ov5693_reg *mode_table[] = {
	[OV5693_MODE_2592X1944] = mode_2592x1944,
	[OV5693_MODE_1920X1080] = mode_1920x1080,
};

static int test_mode;
module_param(test_mode, int, 0644);

static const struct v4l2_frmsize_discrete ov5693_frmsizes[] = {
	{2592, 1944},
	{1920, 1080},
};

/* Find a data format by a pixel code in an array */
static const struct ov5693_datafmt *ov5693_find_datafmt(
		enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5693_colour_fmts); i++)
		if (ov5693_colour_fmts[i].code == code)
			return ov5693_colour_fmts + i;

	return NULL;
}

#define OV5693_MODE	OV5693_MODE_2592X1944
#define OV5693_WIDTH	2592
#define OV5693_HEIGHT	1944

static int ov5693_find_mode(struct v4l2_subdev *sd,
			    u32 width, u32 height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5693_frmsizes); i++) {
		if (width == ov5693_frmsizes[i].width &&
		    height == ov5693_frmsizes[i].height)
			return i;
	}

	dev_err(sd->v4l2_dev->dev, "%dx%d is not supported\n", width, height);
	return OV5693_MODE_2592X1944;
}

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static int ov5693_read_reg(struct i2c_client *client, u16 addr, u8 *val)
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

static int ov5693_write_reg(struct i2c_client *client, u16 addr, u8 val)
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

static int ov5693_write_table(struct i2c_client *client,
			      const struct ov5693_reg table[])
{
	int err;
	const struct ov5693_reg *next;
	u16 val;

	for (next = table; next->addr != OV5693_TABLE_END; next++) {
		if (next->addr == OV5693_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		err = ov5693_write_reg(client, next->addr, val);
		if (err) {
			pr_err("%s:ov5693_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static void ov5693_mclk_disable(struct ov5693 *priv)
{
	return;
	dev_dbg(&priv->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(priv->mclk);
}

static int ov5693_mclk_enable(struct ov5693 *priv)
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


static int ov5693_debugfs_show(struct seq_file *s, void *unused)
{
	struct ov5693 *dev = s->private;

	dev_dbg(&dev->i2c_client->dev, "%s: ++\n", __func__);

	return 0;
}

static ssize_t ov5693_debugfs_write(
	struct file *file,
	char const __user *buf,
	size_t count,
	loff_t *offset)
{
	struct ov5693 *dev =
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
	ret |= ov5693_write_reg(i2c_client, address, data);
read:
	ret |= ov5693_read_reg(i2c_client, address, &readback);
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

static int ov5693_debugfs_open(struct inode *inode, struct file *file)
{
	struct ov5693 *dev = inode->i_private;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	return single_open(file, ov5693_debugfs_show, inode->i_private);
}

static const struct file_operations ov5693_debugfs_fops = {
	.open		= ov5693_debugfs_open,
	.read		= seq_read,
	.write		= ov5693_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void ov5693_remove_debugfs(struct ov5693 *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	debugfs_remove_recursive(dev->debugdir);
	dev->debugdir = NULL;
}

static void ov5693_create_debugfs(struct ov5693 *dev)
{
	struct dentry *ret;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s\n", __func__);

	dev->debugdir =
		debugfs_create_dir("ov5693", NULL);
	if (!dev->debugdir)
		goto remove_debugfs;

	ret = debugfs_create_file("d",
				S_IWUSR | S_IRUGO,
				dev->debugdir, dev,
				&ov5693_debugfs_fops);
	if (!ret)
		goto remove_debugfs;

	return;
remove_debugfs:
	dev_err(&i2c_client->dev, "couldn't create debugfs\n");
	ov5693_remove_debugfs(dev);
}

static int ov5693_power_on(struct ov5693 *priv)
{
	struct ov5693_power_rail *pw = &priv->power;
	int err;

	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd || !pw->dvdd)))
		return -EFAULT;

	gpio_set_value(priv->gpio_pwdn, 1);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ov5693_avdd_fail;

	err = regulator_enable(pw->dovdd);
	if (err)
		goto ov5693_dovdd_fail;

	usleep_range(300, 310);

	return 0;

ov5693_dovdd_fail:
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int ov5693_power_off(struct ov5693 *priv)
{
	struct ov5693_power_rail *pw = &priv->power;

	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd)))
		return -EFAULT;

	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	gpio_set_value(priv->gpio_pwdn, 0);

	return 0;
}

static int ov5693_power_put(struct ov5693_power_rail *pw)
{
	if (unlikely(!pw))
		return -EFAULT;

	pw->avdd = NULL;
	pw->dovdd = NULL;
	pw->dvdd = NULL;

	return 0;
}

static int ov5693_power_get(struct ov5693 *priv)
{
	struct ov5693_power_rail *pw = &priv->power;
	struct ov5693_regulators *regs = priv->pdata->regulators;
	int err;

	/* analog 2.8v */
	pw->avdd = devm_regulator_get(&priv->i2c_client->dev, regs->avdd);
	if (IS_ERR(pw->avdd)) {
		err = PTR_ERR(pw->avdd);
		pw->avdd = NULL;
		dev_err(&priv->i2c_client->dev, "Failed to get regulator %s\n",
			regs->avdd);
		return err;
	}

	/* digital 1.2v */
	pw->dvdd = devm_regulator_get(&priv->i2c_client->dev, regs->dvdd);
	if (IS_ERR(pw->dvdd)) {
		err = PTR_ERR(pw->dvdd);
		pw->dvdd = NULL;
		dev_err(&priv->i2c_client->dev, "Failed to get regulator %s\n",
			regs->dvdd);
		return err;
	}

	/* IO 1.8v */
	pw->dovdd = devm_regulator_get(&priv->i2c_client->dev, regs->dovdd);
	if (IS_ERR(pw->dovdd)) {
		err = PTR_ERR(pw->dovdd);
		pw->dovdd = NULL;
		dev_err(&priv->i2c_client->dev, "Failed to get regulator %s\n",
			regs->dovdd);
		return err;
	}

	return 0;
}

static int ov5693_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693 *priv = to_ov5693(client);
	int mode = ov5693_find_mode(sd, mf->width, mf->height);

	mf->width = ov5693_frmsizes[mode].width;
	mf->height = ov5693_frmsizes[mode].height;

	if (mf->code != V4L2_MBUS_FMT_SRGGB8_1X8 &&
	    mf->code != V4L2_MBUS_FMT_SRGGB10_1X10)
		mf->code = V4L2_MBUS_FMT_SRGGB10_1X10;

	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	priv->mode = mode;

	return 0;
}

static int ov5693_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693 *priv = to_ov5693(client);

	dev_dbg(sd->v4l2_dev->dev, "%s(%u)\n", __func__, mf->code);

	/* MIPI CSI could have changed the format, double-check */
	if (!ov5693_find_datafmt(mf->code))
		return -EINVAL;

	ov5693_try_fmt(sd, mf);

	priv->fmt = ov5693_find_datafmt(mf->code);

	return 0;
}

static int ov5693_g_fmt(struct v4l2_subdev *sd,	struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693 *priv = to_ov5693(client);

	const struct ov5693_datafmt *fmt = priv->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= OV5693_WIDTH;
	mf->height	= OV5693_HEIGHT;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov5693_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	rect->top	= 0;
	rect->left	= 0;
	rect->width	= OV5693_WIDTH;
	rect->height	= OV5693_HEIGHT;

	return 0;
}

static int ov5693_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= OV5693_WIDTH;
	a->bounds.height		= OV5693_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov5693_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if ((unsigned int)index >= ARRAY_SIZE(ov5693_colour_fmts))
		return -EINVAL;

	*code = ov5693_colour_fmts[index].code;
	return 0;
}

static int ov5693_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693 *priv = to_ov5693(client);
	int err = 0;

	if (!enable)
		return 0;

	err = ov5693_write_table(priv->i2c_client, mode_table[priv->mode]);
	if (err)
		return err;

	if (test_mode)
		ov5693_write_table(priv->i2c_client, tp_colorbars);

	return err;
}

static int ov5693_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5693 *priv = to_ov5693(client);
	int err;

	if (on) {
		err = ov5693_mclk_enable(priv);
		if (!err)
			err = ov5693_power_on(priv);
		if (err < 0)
			ov5693_mclk_disable(priv);
		return err;
	} else if (!on) {
		ov5693_power_off(priv);
		ov5693_mclk_disable(priv);
		return 0;
	} else
		return -EINVAL;
}

static int ov5693_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static struct v4l2_subdev_video_ops ov5693_subdev_video_ops = {
	.s_stream	= ov5693_s_stream,
	.s_mbus_fmt	= ov5693_s_fmt,
	.g_mbus_fmt	= ov5693_g_fmt,
	.try_mbus_fmt	= ov5693_try_fmt,
	.enum_mbus_fmt	= ov5693_enum_fmt,
	.g_crop		= ov5693_g_crop,
	.cropcap	= ov5693_cropcap,
	.g_mbus_config	= ov5693_g_mbus_config,
};

static struct v4l2_subdev_core_ops ov5693_subdev_core_ops = {
	.s_power	= ov5693_s_power,
};

static struct v4l2_subdev_ops ov5693_subdev_ops = {
	.core	= &ov5693_subdev_core_ops,
	.video	= &ov5693_subdev_video_ops,
};

static int ov5693_get_sensor_id(struct ov5693 *priv)
{
	int i;
	u8 bak = 0, fuse_id[16];

	if (!strcmp(priv->i2c_client->name, "ov5693_v4l2.1")) {
		dev_info(&priv->i2c_client->dev, "Skip get sensor id\n");
		return 0;
	}

	ov5693_mclk_enable(priv);
	ov5693_power_on(priv);

	ov5693_write_reg(priv->i2c_client, 0x3B02, 0x00);
	ov5693_write_reg(priv->i2c_client, 0x3B00, 0x01);

	for (i = 0; i < 9; i++) {
		ov5693_read_reg(priv->i2c_client, 0x3B24 + i, &bak);
		fuse_id[i] = bak;
	}
	ov5693_power_off(priv);
	ov5693_mclk_disable(priv);

	return 0;
}

static int ov5693_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov5693 *priv;
	struct soc_camera_link *ov5693_iclink;
	const char *mclk_name;
	int err;

	priv = devm_kzalloc(&client->dev,
			sizeof(struct ov5693), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	ov5693_iclink = client->dev.platform_data;
	priv->pdata = ov5693_iclink->dev_priv;
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	priv->i2c_client = client;
	priv->mode = OV5693_MODE;
	priv->fmt = &ov5693_colour_fmts[0];

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	priv->mclk = devm_clk_get(&client->dev, mclk_name);
	if (IS_ERR(priv->mclk)) {
		dev_err(&client->dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(priv->mclk);
	}

	priv->gpio_pwdn = priv->pdata->gpio_pwdn;
	gpio_request(priv->gpio_pwdn, "cam_gpio_pwdn");

	err = ov5693_power_get(priv);
	if (err)
		return err;

	i2c_set_clientdata(client, priv);

	err = ov5693_get_sensor_id(priv);
	if (err) {
		dev_err(&client->dev, "unable to get sensor id\n");
		return err;
	}

	ov5693_create_debugfs(priv);

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5693_subdev_ops);

	dev_info(&client->dev, "Detected a OV5693 chip\n");

	return 0;
}

static int
ov5693_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd;
	struct ov5693 *priv;

	ssdd = soc_camera_i2c_to_desc(client);
	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	priv = i2c_get_clientdata(client);
	ov5693_power_put(&priv->power);
	gpio_free(priv->gpio_pwdn);
	ov5693_remove_debugfs(priv);

	return 0;
}

static const struct i2c_device_id ov5693_id[] = {
	{ "ov5693_v4l2", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov5693_id);

static struct i2c_driver ov5693_i2c_driver = {
	.driver = {
		.name = "ov5693_v4l2",
		.owner = THIS_MODULE,
	},
	.probe = ov5693_probe,
	.remove = ov5693_remove,
	.id_table = ov5693_id,
};

module_i2c_driver(ov5693_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Sony OV5693");
MODULE_AUTHOR("Bryan Wu <pengw@nvidia.com>");
MODULE_LICENSE("GPL v2");
