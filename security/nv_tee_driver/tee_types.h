/*
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __NV_TEE_TYPES_H__
#define __NV_TEE_TYPES_H__

/*
 * Return Codes
 */
enum {
	TEEC_SUCCESS			= 0,
	/* Non-specific cause */
	TEEC_ERROR_GENERIC		= 0xFFFF0000,
	/* Access priviledge not sufficient */
	TEEC_ERROR_ACCESS_DENIED	= 0xFFFF0001,
	/* The operation was cancelled */
	TEEC_ERROR_CANCEL		= 0xFFFF0002,
	/* Concurrent accesses conflict */
	TEEC_ERROR_ACCESS_CONFLICT	= 0xFFFF0003,
	/* Too much data for req was passed */
	TEEC_ERROR_EXCESS_DATA		= 0xFFFF0004,
	/* Input data was of invalid format */
	TEEC_ERROR_BAD_FORMAT		= 0xFFFF0005,
	/* Input parameters were invalid */
	TEEC_ERROR_BAD_PARAMETERS	= 0xFFFF0006,
	/* Oper invalid in current state */
	TEEC_ERROR_BAD_STATE		= 0xFFFF0007,
	/* The req data item not found */
	TEEC_ERROR_ITEM_NOT_FOUND	= 0xFFFF0008,
	/* The req oper not implemented */
	TEEC_ERROR_NOT_IMPLEMENTED	= 0xFFFF0009,
	/* The req oper not supported */
	TEEC_ERROR_NOT_SUPPORTED	= 0xFFFF000A,
	/* Expected data was missing */
	TEEC_ERROR_NO_DATA		= 0xFFFF000B,
	/* System ran out of resources */
	TEEC_ERROR_OUT_OF_MEMORY	= 0xFFFF000C,
	/* The system is busy */
	TEEC_ERROR_BUSY			= 0xFFFF000D,
	/* Communication failed */
	TEEC_ERROR_COMMUNICATION	= 0xFFFF000E,
	/* A security fault was detected */
	TEEC_ERROR_SECURITY		= 0xFFFF000F,
	/* The supplied buffer is too short */
	TEEC_ERROR_SHORT_BUFFER		= 0xFFFF0010,
};

/*
 * Return Code origins
 */
enum {
	TEEC_ORIGIN_API		= 1,
	TEEC_ORIGIN_COMMS	= 2,
	TEEC_ORIGIN_TEE		= 3,
	TEEC_ORIGIN_TRUSTED_APP	= 4,
};

/*
 * Shared Memory control flags
 */
enum {
	TEEC_MEM_INPUT	= 1,
	TEEC_MEM_OUTPUT	= 2,
};

/*
 * Parameter types
 */
enum {
	TEEC_PARAM_TYPE_NONE		= 0x0,
	TEEC_PARAM_TYPE_VALUE_INPUT	= 0x1,
	TEEC_PARAM_TYPE_VALUE_OUTPUT	= 0x2,
	TEEC_PARAM_TYPE_VALUE_INOUT	= 0x3,
	TEEC_PARAM_TYPE_MEMREF_INPUT	= 0x5,
	TEEC_PARAM_TYPE_MEMREF_OUTPUT	= 0x6,
	TEEC_PARAM_TYPE_MEMREF_INOUT	= 0x7,
	TEEC_MEMREF_WHOLE		= 0xC,
	TEEC_MEMREF_PARTIAL_INPUT	= 0xD,
	TEEC_MEMREF_PARTIAL_OUTPUT	= 0xE,
	TEEC_MEMREF_PARTIAL_INOUT	= 0xF,
};

/*
 * Session Login Methods
 */
enum {
	TEEC_LOGIN_PUBLIC		= 0x0,
	TEEC_LOGIN_USER			= 0x1,
	TEEC_LOGIC_GROUP		= 0x2,
	TEEC_LOGIC_APPLICATION		= 0x4,
	TEEC_LOGIC_USER_APPLICATION	= 0x5,
	TEEC_LOGIC_GROUP_APPLICATION	= 0x6,
};
#endif
