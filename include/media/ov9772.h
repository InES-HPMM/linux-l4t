/**
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __OV9772_H__
#define __OV9772_H__

#include <linux/ioctl.h>

#define OV9772_IOCTL_SET_MODE			_IOW('o', 1, struct ov9772_mode)
#define OV9772_IOCTL_GET_STATUS			_IOR('o', 2, __u8)
#define OV9772_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define OV9772_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define OV9772_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define OV9772_IOCTL_GET_SENSORDATA		_IOR('o', 6, struct ov9772_sensordata)
#define OV9772_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct ov9772_ae)

struct ov9772_mode {
	int		xres;
	int		yres;
	__u32	frame_length;
	__u32	coarse_time;
	__u16	gain;
};

struct ov9772_ae {
	__u32	frame_length;
	__u8	frame_length_enable;
	__u32	coarse_time;
	__u8	coarse_time_enable;
	__s32	gain;
	__u8	gain_enable;
};

struct ov9772_sensordata {
	__u32	fuse_id_size;
	__u8	fuse_id[16];
};

#ifdef __KERNEL__

#define OV9772_REG_FRAME_LENGTH_HI	0x340
#define OV9772_REG_FRAME_LENGTH_LO	0x341
#define OV9772_REG_COARSE_TIME_HI	0x202
#define OV9772_REG_COARSE_TIME_LO	0x203
#define OV9772_REG_GAIN_HI			0x204
#define OV9772_REG_GAIN_LO			0x205

struct ov9772_platform_data {
	int	(*power_on)(struct device *);
	int	(*power_off)(struct device *);
};
#endif /* __KERNEL__ */

#endif  /* __OV9772_H__ */

