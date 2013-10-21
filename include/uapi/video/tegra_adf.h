/*
 * Copyright (C) 2013 Google, Inc.
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
 *
 * modified from include/video/tegra_dc_ext.h
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _UAPI_VIDEO_TEGRA_ADF_H_
#define _UAPI_VIDEO_TEGRA_ADF_H_

#include <linux/types.h>
#include <video/adf.h>
#include <drm/drm_fourcc.h>

#define TEGRA_ADF_FORMAT_P1		fourcc_code('P', '1', ' ', ' ')
#define TEGRA_ADF_FORMAT_P2		fourcc_code('P', '2', ' ', ' ')
#define TEGRA_ADF_FORMAT_P4		fourcc_code('P', '4', ' ', ' ')
#define TEGRA_ADF_FORMAT_P8		fourcc_code('P', '8', ' ', ' ')
#define TEGRA_ADF_FORMAT_B6x2G6x2R6x2A8	fourcc_code('B', 'A', '6', '2')
#define TEGRA_ADF_FORMAT_R6x2G6x2B6x2A8	fourcc_code('R', 'A', '6', '2')
#define TEGRA_ADF_FORMAT_R6x2G6x2B6x2A8	fourcc_code('R', 'A', '6', '2')
#define TEGRA_ADF_FORMAT_YCbCr422R	fourcc_code('Y', 'U', '2', 'R')

enum tegra_adf_interface_type {
	TEGRA_ADF_INTF_RGB = ADF_INTF_TYPE_DEVICE_CUSTOM,
	TEGRA_ADF_INTF_LVDS = ADF_INTF_TYPE_DEVICE_CUSTOM + 1,
};

#define TEGRA_ADF_EVENT_BANDWIDTH_RENEGOTIATE	ADF_EVENT_DEVICE_CUSTOM
struct tegra_adf_event_bandwidth {
	struct adf_event base;
	__u32 total_bw;
	__u32 avail_bw;
	__u32 resvd_bw;
};

#define TEGRA_ADF_CAPABILITIES_CURSOR_MODE	(1 << 0)
#define TEGRA_ADF_CAPABILITIES_BLOCKLINEAR	(1 << 1)
struct tegra_adf_capabilities {
	__u32 caps;
	/* Leave some wiggle room for future expansion */
	__u32 pad[3];
};

#define TEGRA_ADF_BLEND_NONE		0
#define TEGRA_ADF_BLEND_PREMULT		1
#define TEGRA_ADF_BLEND_COVERAGE	2

#define TEGRA_ADF_FLIP_FLAG_INVERT_H		(1 << 0)
#define TEGRA_ADF_FLIP_FLAG_INVERT_V		(1 << 1)
#define TEGRA_ADF_FLIP_FLAG_TILED		(1 << 2)
#define TEGRA_ADF_FLIP_FLAG_CURSOR		(1 << 3)
#define TEGRA_ADF_FLIP_FLAG_GLOBAL_ALPHA	(1 << 4)
#define TEGRA_ADF_FLIP_FLAG_BLOCKLINEAR		(1 << 5)
#define TEGRA_ADF_FLIP_FLAG_SCAN_COLUMN		(1 << 6)
#define TEGRA_ADF_FLIP_FLAG_INTERLACE		(1 << 7)

struct tegra_adf_flip_windowattr {
	__s32	win_index;
	__s32	buf_index;
	__u32	blend;
	/*
	 * x and y are fixed-point: 20 bits of integer (MSB) and 12 bits of
	 * fractional (LSB)
	 */
	__u32	x;
	__u32	y;
	__u32	out_x;
	__u32	out_y;
	__u32	out_w;
	__u32	out_h;
	__u32	z;
	__u32	swap_interval;
	struct timespec timestamp;
	__u32	flags;
	__u8	global_alpha; /* requires TEGRA_ADF_FLIP_FLAG_GLOBAL_ALPHA */
	/* log2(blockheight) for blocklinear format */
	__u8	block_height_log2;
	__u8	pad1[2];
	__u32	offset2;
	__u32	offset_u2;
	__u32	offset_v2;
	/* Leave some wiggle room for future expansion */
	__u32   pad2[1];
};

struct tegra_adf_flip {
	__u8 win_num;
	__u8 reserved1; /* unused - must be 0 */
	__u16 reserved2; /* unused - must be 0 */
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
	struct tegra_adf_flip_windowattr win[0];
};

enum {
	TEGRA_DC_Y,
	TEGRA_DC_U,
	TEGRA_DC_V,
	TEGRA_DC_NUM_PLANES,
};

#endif /* _UAPI_VIDEO_TEGRA_ADF_H_ */
