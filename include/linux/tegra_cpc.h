/*
 * tegra_cpc.c - Access CPC storage blocks through i2c bus
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef _TEGRA_CPC_H
#define _TEGRA_CPC_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* sizes of various CPC frame components */
#define CPC_DATA_SIZE		64
#define CPC_HMAC_SIZE		32

enum req_resp_t {
	CPC_PROGRAM_KEY = 0x1,
	CPC_READ_M_COUNT,
	CPC_READ_FRAME,
	CPC_WRITE_FRAME,
	CPC_GET_RESULT,
};

enum result_t {
	CPC_GEN_FAIL = 0x1,
	CPC_MAC_FAIL,
	CPC_CNT_FAIL,
	CPC_ADR_FAIL,
	CPC_WR_FAIL,
	CPC_RD_FAIL,
	CPC_KEY_FAIL,
};

/* CPC frame layout */
struct cpc_frame_t {
	__u8 hmac[CPC_HMAC_SIZE];
	__u8 data[CPC_DATA_SIZE];
	__u16 len;
	__u16 result;
	__u32 write_counter;
	enum req_resp_t req_or_resp;
};

#define NVCPC_IOC_MAGIC 'C'
#define NVCPC_IOCTL_DO_IO	_IOWR(NVCPC_IOC_MAGIC, 2, struct cpc_frame_t)

#endif
