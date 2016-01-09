/**
 * Copyright (c) 2015-2016, NVIDIA Corporation.  All rights reserved.
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

#ifndef __IMX230_H__
#define __IMX230_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>


#define IMX230_IOCTL_SET_MODE			_IOW('o', 1, struct imx230_mode)
#define IMX230_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define IMX230_IOCTL_SET_FRAME_LENGTH		_IOW('o', 3, __u32)
#define IMX230_IOCTL_SET_COARSE_TIME		_IOW('o', 4, __u32)
#define IMX230_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define IMX230_IOCTL_GET_SENSORDATA		_IOR('o', 6, \
	 struct imx230_sensordata)
#define IMX230_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct imx230_ae)
#define IMX230_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, struct imx230_hdr)
#define IMX230_IOCTL_SET_POWER			_IOW('o', 20, __u32)

#ifdef IMX230_EEPROM_PRESENT
#define IMX230_EEPROM_ADDRESS		0x50
#define IMX230_EEPROM_SIZE		1024
#define IMX230_EEPROM_STR_SIZE		(IMX230_EEPROM_SIZE * 2)
#define IMX230_EEPROM_BLOCK_SIZE	(1 << 8)
#define IMX230_EEPROM_NUM_BLOCKS \
	 (IMX230_EEPROM_SIZE / IMX230_EEPROM_BLOCK_SIZE)
#endif /* if IMX230_EEPROM_PRESENT */

#define IMX230_OTP_CTRL_ADDR		0x0A00
#define IMX230_OTP_STATUS_ADDR		0x0A01
#define IMX230_OTP_PAGE_NUM_ADDR	0x0A02
#define IMX230_OTP_PAGE_START_ADDR	0x0A04
#define IMX230_OTP_PAGE_END_ADDR	0x0A43
#define IMX230_OTP_NUM_PAGES		(18)
#define IMX230_OTP_PAGE_SIZE \
	 (IMX230_OTP_PAGE_END_ADDR - IMX230_OTP_PAGE_START_ADDR + 1)
#define IMX230_OTP_SIZE \
	 (IMX230_OTP_PAGE_SIZE * IMX230_OTP_NUM_PAGES)
#define IMX230_OTP_STR_SIZE (IMX230_OTP_SIZE * 2)
#define IMX230_OTP_STATUS_IN_PROGRESS		0
#define IMX230_OTP_STATUS_READ_COMPLETE	1
#define IMX230_OTP_STATUS_READ_FAIL		5

#define IMX230_FUSE_ID_OTP_ROW_ADDR	0x0A36
#define IMX230_FUSE_ID_OTP_PAGE	31 /*0x1F*/
#define IMX230_FUSE_ID_SIZE	11
#define IMX230_FUSE_ID_STR_SIZE	(IMX230_FUSE_ID_SIZE * 2)

#define IMX230_FRAME_LENGTH_ADDR_MSB		0x0340
#define IMX230_FRAME_LENGTH_ADDR_LSB		0x0341
#define IMX230_COARSE_TIME_ADDR_MSB		0x0202
#define IMX230_COARSE_TIME_ADDR_LSB		0x0203
#define IMX230_COARSE_TIME_SHORT_ADDR_MSB	0x0224
#define IMX230_COARSE_TIME_SHORT_ADDR_LSB	0x0225
#define IMX230_GAIN_ADDR_MSB			0x0204
#define IMX230_GAIN_ADDR_LSB			0x0205
#define IMX230_GAIN_SHORT_ADDR_MSB		0x0216
#define IMX230_GAIN_SHORT_ADDR_LSB		0x0217
#define IMX230_GROUP_HOLD_ADDR			0x0104

#define IMX230_CROP_X_START_ADDR_MSB	0x0344
#define IMX230_CROP_X_START_ADDR_LSB	0x0345
#define IMX230_CROP_Y_START_ADDR_MSB	0x0346
#define IMX230_CROP_Y_START_ADDR_LSB	0x0347
#define IMX230_CROP_X_END_ADDR_MSB	0x0348
#define IMX230_CROP_X_END_ADDR_LSB	0x0349
#define IMX230_CROP_Y_END_ADDR_MSB	0x034A
#define IMX230_CROP_Y_END_ADDR_LSB	0x034B
#define IMX230_CROP_X_OUTPUT_SIZE_MSB	0x034C
#define IMX230_CROP_X_OUTPUT_SIZE_LSB	0x034D
#define IMX230_CROP_Y_OUTPUT_SIZE_MSB	0x034E
#define IMX230_CROP_Y_OUTPUT_SIZE_LSB	0x034F

struct imx230_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct imx230_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct imx230_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct imx230_sensordata {
	__u32 fuse_id_size;
	__u8  fuse_id[IMX230_FUSE_ID_SIZE];
};

#ifdef __KERNEL__
struct imx230_power_rail {
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

struct imx230_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct imx230_power_rail *pw);
	int (*power_off)(struct imx230_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __IMX230_H__ */
