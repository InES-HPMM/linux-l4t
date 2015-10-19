/**
 * Copyright (c) 2014-2015, NVIDIA Corporation.  All rights reserved.
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

#ifndef __OV13860_H__
#define __OV13860_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>

#define OV13860_IOCTL_SET_MODE			_IOW('o', 1, \
		struct ov13860_mode)
#define OV13860_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define OV13860_IOCTL_SET_FRAME_LENGTH		_IOW('o', 3, __u32)
#define OV13860_IOCTL_SET_COARSE_TIME		_IOW('o', 4, __u32)
#define OV13860_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define OV13860_IOCTL_GET_SENSORDATA		_IOR('o', 6, \
	 struct ov13860_sensordata)
#define OV13860_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct ov13860_ae)
#define OV13860_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, struct ov13860_hdr)
#define OV13860_IOCTL_SET_POWER			_IOW('o', 20, __u32)

#define OV13860_EEPROM_SIZE		1024
#define OV13860_EEPROM_STR_SIZE		(OV13860_EEPROM_SIZE * 2)
#define OV13860_EEPROM_BLOCK_SIZE	(1 << 8)
#define OV13860_EEPROM_NUM_BLOCKS \
	 (OV13860_EEPROM_SIZE / OV13860_EEPROM_BLOCK_SIZE)

#define OV13860_ISP_CTRL_ADDR		0x5000
#define OV13860_OTP_PROGRAME_CTRL_ADDR		0x3D80
#define OV13860_OTP_LOAD_CTRL_ADDR		0x3D81
#define OV13860_OTP_MODE_CTRL_ADDR		0x3D84
#define OV13860_OTP_PROGRAME_START_ADDR_MSB	0x3D88
#define OV13860_OTP_PROGRAME_START_ADDR_LSB	0x3D89
#define OV13860_OTP_PROGRAME_END_ADDR_MSB	0x3D8A
#define OV13860_OTP_PROGRAME_END_ADDR_LSB	0x3D8B
#define OV13860_OTP_SIZE 0xA00
#define OV13860_OTP_SRAM_START_ADDR			0x7000
#define OV13860_OTP_STR_SIZE (OV13860_OTP_SIZE * 2)

#define OV13860_FUSE_ID_OTP_BASE_ADDR	0x7000
#define OV13860_FUSE_ID_SIZE		3
#define OV13860_FUSE_ID_STR_SIZE	(OV13860_FUSE_ID_SIZE * 2)

#define OV13860_FRAME_LENGTH_ADDR_MSB		0x380E
#define OV13860_FRAME_LENGTH_ADDR_LSB		0x380F
#define OV13860_COARSE_TIME_ADDR_MSB		0x3501
#define OV13860_COARSE_TIME_ADDR_LSB		0x3502
#define OV13860_COARSE_TIME_SHORT_ADDR_MSB	0x3507
#define OV13860_COARSE_TIME_SHORT_ADDR_LSB	0x3508
#define OV13860_GAIN_ADDR_MSB			0x350A
#define OV13860_GAIN_ADDR_LSB			0x350B
#define OV13860_GAIN_SHORT_ADDR_MSB		0x350E
#define OV13860_GAIN_SHORT_ADDR_LSB		0x350F
#define OV13860_GROUP_HOLD_ADDR			0x3208
#define OV13860_GROUP_HOLD_LAUNCH_ADDR	0x320B

struct ov13860_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct ov13860_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct ov13860_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct ov13860_sensordata {
	__u32 fuse_id_size;
	__u8  fuse_id[OV13860_FUSE_ID_SIZE];
};

#ifdef __KERNEL__
struct ov13860_power_rail {
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

struct ov13860_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct ov13860_power_rail *pw);
	int (*power_off)(struct ov13860_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __OV13860_H__ */
