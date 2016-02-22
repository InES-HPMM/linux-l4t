/**
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

#ifndef __IMX185_H__
#define __IMX185_H__

#include <linux/ioctl.h>
#include <media/nvc.h>
#include <media/nvc_image.h>

#define IMX185_EEPROM_STR_SIZE	80
#define IMX185_OTP_STR_SIZE	60
#define IMX185_FUSE_ID_ADDR	0x3382
#define IMX185_FUSE_ID_SIZE	6
#define IMX185_FUSE_ID_STR_SIZE	(IMX185_FUSE_ID_SIZE * 2)

#define IMX185_FRAME_LENGTH_ADDR_MSB	0x301A
#define IMX185_FRAME_LENGTH_ADDR_MID	0x3019
#define IMX185_FRAME_LENGTH_ADDR_LSB	0x3018
#define IMX185_COARSE_TIME_SHS1_ADDR_MSB	0x3022
#define IMX185_COARSE_TIME_SHS1_ADDR_MID	0x3021
#define IMX185_COARSE_TIME_SHS1_ADDR_LSB	0x3020
#define IMX185_COARSE_TIME_SHS2_ADDR_MSB	0x3025
#define IMX185_COARSE_TIME_SHS2_ADDR_MID	0x3024
#define IMX185_COARSE_TIME_SHS2_ADDR_LSB	0x3023
#define IMX185_GAIN_ADDR	0x3014
#define IMX185_GROUP_HOLD_ADDR	0x3001

#define IMX185_PCA954X_I2C_ADDR (0x70)

struct imx185_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct imx185_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct imx185_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct imx185_sensordata {
	__u32 fuse_id_size;
	__u8  fuse_id[IMX185_FUSE_ID_SIZE];
};

#ifdef __KERNEL__
struct imx185_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *ext_reg1;
	struct regulator *ext_reg2;
	struct clk *mclk;
	unsigned int pwdn_gpio;
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
};

struct imx185_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct imx185_power_rail *pw);
	int (*power_off)(struct imx185_power_rail *pw);
};
#endif /* __KERNEL__ */
#endif  /* __IMX185_H__ */
