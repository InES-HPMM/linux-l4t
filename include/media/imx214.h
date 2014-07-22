/**
 * Copyright (c) 2014, NVIDIA Corporation.  All rights reserved.
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

#ifndef __IMX214_H__
#define __IMX214_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>

#define IMX214_IOCTL_SET_MODE		_IOW('o', 1, struct imx214_mode)
#define IMX214_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define IMX214_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX214_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX214_IOCTL_SET_GAIN		_IOW('o', 5, __u16)
#define IMX214_IOCTL_GET_SENSORDATA	_IOR('o', 6, struct imx214_sensordata)
#define IMX214_IOCTL_SET_GROUP_HOLD	_IOW('o', 7, struct imx214_ae)
#define IMX214_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, struct imx214_hdr)
#define IMX214_IOCTL_SET_POWER		_IOW('o', 20, __u32)

#define IMX214_EEPROM_ADDRESS	0x50
#define IMX214_EEPROM_SIZE	1024
#define IMX214_EEPROM_BLOCK_SIZE	(1 << 8)
#define IMX214_EEPROM_NUM_BLOCKS \
	(IMX214_EEPROM_SIZE / IMX214_EEPROM_BLOCK_SIZE)

struct imx214_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct imx214_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct imx214_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct imx214_sensordata {
	__u32 fuse_id_size;
	__u8  fuse_id[16];
};

struct imx214_eeprom_data {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
};

#ifdef __KERNEL__
struct imx214_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *ext_reg1;
	struct regulator *ext_reg2;
};

struct imx214_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct imx214_power_rail *pw);
	int (*power_off)(struct imx214_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __IMX214_H__ */
