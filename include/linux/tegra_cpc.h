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

enum req_resp_t {
	CPC_READ_M_COUNT = 0x2,
	CPC_WRITE_FRAME,
	CPC_READ_FRAME,
	CPC_GET_RESULT = 0x5
};

enum result_t {
	CPC_RESULT_OK = 0x0,
	CPC_RESULT_GENERAL_FAILURE,
	CPC_RESULT_AUTH_FAILURE,
	CPC_RESULT_COUNTER_FAILURE,
	CPC_RESULT_ADDR_FAILURE,
	CPC_RESULT_WRITE_FAILURE = 0x5,
	CPC_RESULT_READ_FAILURE,
	CPC_RESULT_UNKNOWN_REQUEST,
	CPC_RESULT_KEY_PROG_SEQ_FAILURE,
	CPC_RESULT_KEY_PROG_PG_CORRUPT,
	CPC_RESULT_KEY_PROG_DONE_PRIOR = 0xA,
	CPC_RESULT_DATA_LENGTH_MISMATCH,
	CPC_RESULT_UNEXPECTED_CONDITION,
	CPC_RESULT_CONTROLLER_BUSY
};

/*
   Packets to uC will have the format below
	CMD_ID		Type of command. req_resp_t
	DATA_LEN	indicates the length of valid data. For a read type,
			this would be the number of bytes the host would be
			reading from the CPC. For write, this would be the
			number of bytes in the data field which actually
			represent data. Does not include the number of bytes
			occupied by HMAC, whether HMAC is present or not
	DATA		Empty for read. Contains CPC_DATA + CPC_HMAC for write
 */

#define CPC_MAX_DATA_SIZE	4
#define CPC_KEY_SIZE		32
#define CPC_HMAC_SIZE		32
#define CPC_RESULT_SIZE		1
#define CPC_DATALEN_SIZE	1
#define CPC_CMDID_SIZE		1
#define CPC_COUNTER_SIZE	4
#define CPC_HEADER_SIZE		(CPC_DATALEN_SIZE + CPC_CMDID_SIZE)

/*
 * Size of various CPC frame components in bytes
 * The header is everything up to, but not including, the data
 */
#define CPC_CMD_ID_INDEX	0
#define CPC_DATALEN_INDEX	1
#define CPC_DATA_INDEX		2

typedef	u8		cpc_data_len;

/* CPC frame layout */
struct cpc_frame_t {
	__u8 hmac[CPC_HMAC_SIZE];
	__u8 data[CPC_MAX_DATA_SIZE];
	__u32 write_counter;
	__u8 len;
	__u8 result;
	__u8 req_or_resp;
};

#define NVCPC_IOC_MAGIC 'C'
#define NVCPC_IOCTL_DO_IO	_IOWR(NVCPC_IOC_MAGIC, 2, struct cpc_frame_t)

#endif
