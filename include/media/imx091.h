/**
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX091_H__
#define __IMX091_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define IMX091_IOCTL_SET_MODE			_IOW('o', 1, struct imx091_mode)
#define IMX091_IOCTL_GET_STATUS			_IOR('o', 2, __u8)
#define IMX091_IOCTL_SET_FRAME_LENGTH		_IOW('o', 3, __u32)
#define IMX091_IOCTL_SET_COARSE_TIME		_IOW('o', 4, __u32)
#define IMX091_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define IMX091_IOCTL_GET_SENSORDATA		_IOR('o', 6, \
						struct imx091_sensordata)
#define IMX091_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct imx091_ae)

struct imx091_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct imx091_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct imx091_sensordata {
	__u32 fuse_id_size;
	__u8  fuse_id[16];
};

#ifdef __KERNEL__
struct imx091_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
};

struct imx091_platform_data {
	int (*power_on)(struct imx091_power_rail *pw);
	int (*power_off)(struct imx091_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __IMX091_H__ */
