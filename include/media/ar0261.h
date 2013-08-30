/**
 * Copyright (c) 2013 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __AR0261_H__
#define __AR0261_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define AR0261_IOCTL_SET_MODE		_IOW('o', 1, struct ar0261_mode)
#define AR0261_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define AR0261_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define AR0261_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define AR0261_IOCTL_SET_GAIN		_IOW('o', 5, __u16)
#define AR0261_IOCTL_GET_SENSORDATA _IOR('o', 6, struct ar0261_sensordata)
#define AR0261_IOCTL_SET_GROUP_HOLD	_IOW('o', 7, struct ar0261_ae)

/* AR0261 registers */
#define AR0261_GROUP_PARAM_HOLD			(0x0104)
#define AR0261_COARSE_INTEGRATION_TIME_15_8	(0x0202)
#define AR0261_COARSE_INTEGRATION_TIME_7_0	(0x0203)
#define AR0261_ANA_GAIN_GLOBAL			(0x305F)
#define AR0261_FRAME_LEN_LINES_15_8		(0x0340)
#define AR0261_FRAME_LEN_LINES_7_0		(0x0341)

#define NUM_OF_FRAME_LEN_REG		2
#define NUM_OF_COARSE_TIME_REG		2
#define NUM_OF_SENSOR_ID_SPECIFIC_REG	8
struct ar0261_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct ar0261_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct ar0261_sensordata {
	__u32 fuse_id_size;
	__u8 fuse_id[16];
};

#ifdef __KERNEL__
struct ar0261_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
};

struct ar0261_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	int (*power_on)(struct ar0261_power_rail *pw);
	int (*power_off)(struct ar0261_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __AR0261_H__ */
