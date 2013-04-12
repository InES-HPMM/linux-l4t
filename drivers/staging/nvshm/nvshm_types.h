/*
 * Copyright (C) 2012-2013 NVIDIA Corporation.
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

#ifndef _NVSHM_TYPES_H
#define _NVSHM_TYPES_H

#include <linux/workqueue.h>
#include <linux/platform_data/nvshm.h> /* NVSHM_SERIAL_BYTE_SIZE */

/* NVSHM common types */

/* Shared memory fixed offsets */

#define NVSHM_IPC_BASE (0x0)              /* IPC mailbox base offset */
#define NVSHM_IPC_MAILBOX (0x0)           /* IPC mailbox offset */
#define NVSHM_IPC_RETCODE (0x4)           /* IPC mailbox return code offset */
#define NVSHM_IPC_SIZE (4096)             /* IPC mailbox region size */
#define NVSHM_CONFIG_OFFSET (8)           /* shared memory config offset */
#define NVSHM_MAX_CHANNELS (12)           /* Maximum number of channels */
#define NVSHM_CHAN_NAME_SIZE (27)         /* max channel name size in chars */

/* Versions: */
/* Original version */
#define NVSHM_CONFIG_ORIGINAL_VERSION (0x00010001)
/** Serial version: support for SHM serial version in SHM
 *  config: SHM serial version is PCID */
#define NVSHM_CONFIG_SERIAL_VERSION (0x00010002)
#define NVSHM_CONFIG_V3_VERSION (0x00010003)
/** Current configuration version (major/minor) */
#define NVSHM_CONFIG_VERSION NVSHM_CONFIG_V3_VERSION


#define NVSHM_AP_POOL_ID (128) /* IOPOOL ID - use 128-255 for AP */

#define NVSHM_RATE_LIMIT_TTY (256)
#define NVSHM_RATE_LIMIT_LOG (512)
#define NVSHM_RATE_LIMIT_NET (2048)
#define NVSHM_RATE_LIMIT_RPC (256)
#define NVSHM_RATE_LIMIT_TRESHOLD (8)

/* NVSHM_IPC mailbox messages ids */
enum nvshm_ipc_mailbox {
	/* Boot status */
	NVSHM_IPC_BOOT_COLD_BOOT_IND = 0x01,
	NVSHM_IPC_BOOT_FW_REQ,
	NVSHM_IPC_BOOT_RESTART_FW_REQ,
	NVSHM_IPC_BOOT_FW_CONF,
	NVSHM_IPC_READY,

	/* Boot errors */
	NVSHM_IPC_BOOT_ERROR_BT2_HDR = 0x1000,
	NVSHM_IPC_BOOT_ERROR_BT2_SIGN,
	NVSHM_IPC_BOOT_ERROR_HWID,
	NVSHM_IPC_BOOT_ERROR_APP_HDR,
	NVSHM_IPC_BOOT_ERROR_APP_SIGN,
	NVSHM_IPC_BOOT_ERROR_UNLOCK_HEADER,
	NVSHM_IPC_BOOT_ERROR_UNLOCK_SIGN,
	NVSHM_IPC_BOOT_ERROR_UNLOCK_PCID,

	NVSHM_IPC_MAX_MSG = 0xFFFF
};

/* NVSHM Config */

/* Channel type */
enum nvshm_chan_type {
	NVSHM_CHAN_UNMAP = 0,
	NVSHM_CHAN_TTY,
	NVSHM_CHAN_LOG,
	NVSHM_CHAN_NET,
	NVSHM_CHAN_RPC,
};

/* Channel mapping structure */
struct nvshm_chan_map {
	/* tty/net/log */
	int type;
	/* Name of device - reflected in sysfs */
	char name[NVSHM_CHAN_NAME_SIZE+1];
};

/*
 * This structure is set by BB after boot to give AP its current shmem mapping
 * BB initialize all descriptor content and give initial empty element
 * for each queue
 * BB enqueue free AP descriptor element into AP queue
 * AP initialize its queues pointer with empty descriptors offset
 * and retreive its decriptors
 * from ap queue.
 */
struct nvshm_config {
	int version;
	int shmem_size;
	int region_ap_desc_offset;
	int region_ap_desc_size;
	int region_bb_desc_offset;
	int region_bb_desc_size;
	int region_ap_data_offset;
	int region_ap_data_size;
	int region_bb_data_offset;
	int region_bb_data_size;
	int queue_ap_offset;
	int queue_bb_offset;
	struct nvshm_chan_map chan_map[NVSHM_MAX_CHANNELS];
	char serial[NVSHM_SERIAL_BYTE_SIZE];
};

/*
 * This structure holds data fragments reference
 * WARNING: ALL POINTERS ARE IN BASEBAND MAPPING
 * NO POINTER SHOULD BE USED WITHOUT PROPER MACRO
 * see nvshm_iobuf.h for reference
 */
struct nvshm_iobuf {
	/* Standard iobuf part - This part is fixed and cannot be changed */
	unsigned char   *npduData;
	unsigned short   length;
	unsigned short   dataOffset;
	unsigned short   totalLength;
	unsigned char ref;
	unsigned char pool_id;
	struct nvshm_iobuf *next;
	struct nvshm_iobuf *sg_next;
	unsigned short flags;
	unsigned short _size;
	void *_handle;
	unsigned int _reserved;

	/* Extended iobuf - This part is not yet fixed (under spec/review) */
	struct nvshm_iobuf *qnext;
	int chan;
	int qflags;
	int _reserved1;
	int _reserved2;
	int _reserved3;
	int _reserved4;
	int _reserved5;
};

/* channel structure */
struct nvshm_channel {
	int index;
	struct nvshm_chan_map map;
	struct nvshm_if_operations *ops;
	void *data;
	int rate_counter;
	struct work_struct start_tx_work;
};


#endif /* _NVSHM_TYPES_H */
