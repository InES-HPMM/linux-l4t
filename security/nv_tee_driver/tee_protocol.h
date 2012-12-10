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

#ifndef __NV_TEE_PROTOCOL_H
#define __NV_TEE_PROTOCOL_H

#include "tee_types.h"
#include "tee_client_api.h"

#define TEE_IOCTL_MAGIC_NUMBER ('t')
#define TEE_IOCTL_OPEN_CLIENT_SESSION \
	_IOWR(TEE_IOCTL_MAGIC_NUMBER, 0x10, union tee_cmd)
#define TEE_IOCTL_CLOSE_CLIENT_SESSION \
	_IOWR(TEE_IOCTL_MAGIC_NUMBER, 0x11, union tee_cmd)
#define TEE_IOCTL_REGISTER_MEMORY \
	_IOWR(TEE_IOCTL_MAGIC_NUMBER, 0x12, union tee_cmd)
#define TEE_IOCTL_RELEASE_SHARED_MEM \
	_IOWR(TEE_IOCTL_MAGIC_NUMBER, 0x13, struct TEEC_SharedMemory)
#define TEE_IOCTL_INVOKE_COMMAND \
	_IOWR(TEE_IOCTL_MAGIC_NUMBER, 0x14, union tee_cmd)
#define TEE_IOCTL_REQ_CANCELLATION \
	_IOR(TEE_IOCTL_MAGIC_NUMBER, 0x15, union tee_cmd)

#define TEE_IOCTL_MIN_NR	_IOC_NR(TEE_IOCTL_OPEN_CLIENT_SESSION)
#define TEE_IOCTL_MAX_NR	_IOC_NR(TEE_IOCTL_REQ_CANCELLATION)

#define NV_CMD_DESC_MAX	120

extern void nv_tee_irq_handler(void);

struct nv_device {
	unsigned long param_addr;
	struct list_head used_cmd_list;
	struct list_head free_cmd_list;
};

struct nv_cmd_param_desc {
	unsigned long param_addr;
	struct list_head list;
};

struct nv_shmem_desc {
	struct list_head list;
	void *buffer;
	size_t size;
	unsigned int mem_type;
	struct page **pages;
	unsigned int nr_pages;
};

struct nv_tee_context {
	struct nv_device *dev;
	struct list_head shmem_alloc_list;
};

enum {
	/* Do a tee invoke */
	TMK_SMC_INVOKE_CMD = 0xFFFF1000,
	/* Get a pending answer without making new invokes */
	TMK_SMC_GET_MORE = 0xFFFF1001,
	/* Answer from secure side */
	TMK_SMC_ANSWER = 0xFFFF1002,
	/* No answers for now (secure side idle) */
	TMK_SMC_NO_ANSWER = 0xFFFF1003,
	/* Open Session */
	TMK_SMC_OPEN_SESSION = 0xFFFF1004,
	/* Close Session */
	TMK_SMC_CLOSE_SESSION = 0xFFFF1005,
	/* Alloc Shared Memory*/
	TMK_SMC_ALLOC_SHARED_MEM = 0xFFFF1006,
	/* Register Shared Memory*/
	TMK_SMC_REG_SHARED_MEM = 0xFFFF1007,
	/* Release Shared Memory*/
	TMK_SMC_RELEASE_SHARED_MEM = 0xFFFF1008,
};

union tee_param {
	struct {
		void	*buffer;
		size_t	size;
	} memref;
	struct {
		uint32_t	a;
		uint32_t	b;
	} value;
};

struct tee_request {
	uint32_t	type;
	uint32_t	session_id;
	uint32_t	command_id;
	phys_addr_t	cmd_param;
};

struct tee_answer {
	uint32_t	type;
	uint32_t	result;
	uint32_t	return_origin;
	uint32_t	session_id;
	union TEEC_Param	params[4];
};

/*
 * structures for user app communication
 */

/*
 * OpenSession
 */

struct tee_opensession {
	struct TEEC_UUID dest_uuid;
	uint32_t login_types;
	uint32_t login_data;
	struct TEEC_Operation operation;
	uint32_t answer;
};

/*
 * CloseSession
 */
struct tee_closesession {
	uint32_t	session_id;
	uint32_t	answer;
};

/*
 * Shared Memory request
 */
struct tee_sharedmem {
	uint32_t		session_id;
	uint32_t		command_id;
	struct TEEC_SharedMemory	memref;
	uint32_t		answer;
};

/*
 * Invoke Command request
 */
struct tee_invokecmd {
	uint32_t	session_id;
	uint32_t	command_id;
	struct TEEC_Operation	operation;
	uint32_t	answer;
};

/*
 * Request Cancellation request
 */
struct tee_req_cancellation {
	uint32_t	session_id;
	uint32_t	command_id;
	struct TEEC_Operation	operation;
	uint32_t	answer;
};

union tee_cmd {
	struct tee_opensession		opensession;
	struct tee_closesession		closesession;
	struct tee_sharedmem		sharedmem;
	struct TEEC_SharedMemory		release_shared_mem;
	struct tee_invokecmd		invokecmd;
	struct tee_req_cancellation	cancellation;
};

struct tee_cmd_param {
	uint32_t	param_types;
	union tee_param	params[4];
	uint32_t	dest_uuid[4];
};

/*
 * SMC protocol union
 */
union smc_args_t {
	struct tee_request	request;
	struct tee_answer	answer;
	unsigned int		smc[8];
};

int tee_open_session(struct tee_opensession *cmd,
	phys_addr_t phy_cmd_page,
	struct tee_answer *answer);

int tee_close_session(uint32_t session_id);

int tee_register_memory(struct tee_sharedmem *cmd,
	phys_addr_t phy_cmd_page,
	struct tee_answer *answer,
	struct nv_tee_context *context);

void tee_unregister_memory(void *buffer,
	struct nv_tee_context *context);

int tee_invoke_command(struct tee_invokecmd *cmd,
	phys_addr_t phy_cmd_page,
	struct tee_answer *answer);

int tee_pin_mem_buffers(void *buffer, size_t size,
	struct nv_tee_context *context);
#endif
