/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 */

/*
 * This header file corresponds to V1.0_c of the GlobalPlatform
 * TEE Client API Specification
 */

#ifndef __TEE_CLIENT_API_H__
#define __TEE_CLIENT_API_H__

#define TEEC_Result		uint32_t;

#define TEEC_PARAM_TYPES(t0, t1, t2, t3) \
	 ((t0) | ((t1) << 4) | ((t2) << 8) | ((t3) << 12))
#define TEEC_PARAM_TYPE_GET(t, i) (((t) >> (i*4)) & 0xF)

/*
 * Implementation dependent data types
 */

struct TEEC_Context_Imp {
	uint32_t context_id;
};

struct TEEC_Session_Imp {
	uint32_t context_id;
	uint32_t session_id;
};

struct TEEC_Operation_Imp {
	uint32_t context_id;
	uint32_t session_id;
};

struct TEEC_SharedMemory_Imp {
	uint32_t context_id;
	uint32_t session_id;
	uint32_t alloc_state;
};

/*
 * Type definitions
 */

struct TEEC_Context {
	struct TEEC_Context_Imp imp;
};

struct TEEC_Session {
	struct TEEC_Session_Imp imp;
};

struct TEEC_SharedMemory {
	void *buffer;
	size_t size;
	uint32_t flags;
	struct TEEC_SharedMemory_Imp imp;
};

union TEEC_Param {
	struct {
		void *buffer;
		size_t size;
	} tmpref;
	struct {
		struct TEEC_SharedMemory *parent;
		size_t size;
		size_t offset;
	} memref;
	struct {
		uint32_t a;
		uint32_t b;
	} value;
};

struct TEEC_Operation {
	uint32_t started;
	uint32_t paramTypes;
	union TEEC_Param params[4];
};

struct TEEC_UUID {
	uint32_t time_low;
	uint16_t time_mid;
	uint16_t time_hi_and_version;
	uint8_t clock_seq_and_node[8];
};

#define TEEC_MAX_FILE_NAME_LEN		64

typedef enum {
	TEEC_FILE_REQ_READ = 0,
	TEEC_FILE_REQ_WRITE = 1,
	TEEC_FILE_REQ_DELETE = 2,
	TEEC_FILE_REQ_SIZE = 3,
} TEEC_FileReqType;

typedef struct {
	char name[TEEC_MAX_FILE_NAME_LEN];
	TEEC_FileReqType type;
	void *user_data_buf;
	void *kern_data_buf;
	unsigned long data_len;
	unsigned long result;
	int error;
} TEEC_FileReq;

struct tee_file_req_node {
	struct list_head node;
	TEEC_FileReq *req;
};

#endif
