/*
 * include/linux/tegra_adsp.h
 *
 * A Header file for managing ADSP/APE
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
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
#ifndef __LINUX_TEGRA_NVADSP_H
#define __LINUX_TEGRA_NVADSP_H

#include <linux/types.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>

typedef int status_t;

/*
 * Shared Semaphores
 */
typedef struct {
	int magic; /* 'ssem' */
	uint8_t id;
	wait_queue_head_t wait;
	struct timer_list timer;
} nvadsp_shared_sema_t;

nvadsp_shared_sema_t *
nvadsp_shared_sema_init(uint8_t nvadsp_shared_sema_id);
status_t nvadsp_shared_sema_destroy(nvadsp_shared_sema_t *);
status_t nvadsp_shared_sema_acquire(nvadsp_shared_sema_t *);
status_t nvadsp_shared_sema_release(nvadsp_shared_sema_t *);

/*
 * Arbitrated Semaphores
 */
typedef struct {
	int magic; /* 'asem' */
	uint8_t  id;
	wait_queue_head_t wait;
	struct completion comp;
} nvadsp_arb_sema_t;

nvadsp_arb_sema_t *nvadsp_arb_sema_init(uint8_t nvadsp_arb_sema_id);
status_t nvadsp_arb_sema_destroy(nvadsp_arb_sema_t *);
status_t nvadsp_arb_sema_acquire(nvadsp_arb_sema_t *);
status_t nvadsp_arb_sema_release(nvadsp_arb_sema_t *);

/*
 * Mailbox messages
 */
typedef struct {
	int magic; /* 'mbmg' */
	struct list_head node;
	uint16_t src_serviceid;
	uint16_t src_mbid;
	uint16_t dst_serviceid;
	uint16_t dst_mbid;
	void *data;
	uint32_t dlen;
} nvadsp_mbmsg_t;

nvadsp_mbmsg_t *nvadsp_mbox_get_msg(void);
void nvadsp_mbox_put_msg(nvadsp_mbmsg_t *);

/*
 * Mailbox Message Queue
 */
typedef struct {
	int magic; /* 'mbmq' */
	struct list_head list;
	uint16_t count;
	uint16_t depth;
} nvadsp_mbqueue_t;

/*
 * Mailbox
 */
typedef struct {
	int magic; /* 'mbox' */
	struct list_head node;
	uint16_t src_serviceid;
	uint16_t src_mbid;
	uint16_t dst_serviceid;
	uint16_t dst_mbid;
	nvadsp_mbqueue_t mbqueues[2];
} nvadsp_mbox_t;

#define NVADSP_MBQ_INBOUND	0
#define NVADSP_MBQ_OUTBOUND	1

nvadsp_mbox_t *nvadsp_mbox_init(uint16_t serviceid, uint16_t mbid);
status_t nvadsp_mbox_connect(nvadsp_mbox_t *, uint16_t dst_serviceid,
			     uint16_t dst_mbid);
status_t nvadsp_mbox_send(nvadsp_mbox_t *, nvadsp_mbmsg_t *);
nvadsp_mbmsg_t *nvadsp_mbox_recv(nvadsp_mbox_t *);
status_t nvadsp_mbox_destroy(nvadsp_mbox_t *);

/*
 * Circular Message Queue
 */
typedef struct {
	uint32_t type;
	void *data;
	size_t dlen;
} nvadsp_cmsg_queue_msg_t;

typedef struct {
	int magic;	/* 'cmgq' */
	int key;
	uint32_t head;
	uint32_t tail;
	uint32_t num;
	uint32_t state;
	nvadsp_cmsg_queue_msg_t *queue[]; /* XXX */
} nvadsp_cmsg_queue_t;

status_t nvadsp_cmsg_queue_init(int mq_key, nvadsp_cmsg_queue_t *);
nvadsp_cmsg_queue_t *nvadsp_cmsg_queue_connect(int mq_key);
status_t nvadsp_cmsg_queue_enqueue(nvadsp_cmsg_queue_t *,
				   nvadsp_cmsg_queue_msg_t *);
nvadsp_cmsg_queue_msg_t *nvadsp_cmsg_queue_dequeue(nvadsp_cmsg_queue_t *);
bool nvadsp_cmsg_queue_empty(nvadsp_cmsg_queue_t *);
bool nvadsp_cmsg_queue_full(nvadsp_cmsg_queue_t *);
status_t nvadsp_cmsg_queue_deinit(nvadsp_cmsg_queue_t *);

/*
 * DRAM Sharing
 */
typedef dma_addr_t nvadsp_iova_addr_t;
typedef enum dma_data_direction nvadsp_data_direction_t;

nvadsp_iova_addr_t
nvadsp_dram_map_single(struct device *nvadsp_dev,
		       void *cpu_addr, size_t size,
		       nvadsp_data_direction_t direction);
void
nvadsp_dram_unmap_single(struct device *nvadsp_dev,
			 nvadsp_iova_addr_t iova_addr, size_t size,
			 nvadsp_data_direction_t direction);

nvadsp_iova_addr_t
nvadsp_dram_map_page(struct device *nvadsp_dev,
		     struct page *page, unsigned long offset, size_t size,
		     nvadsp_data_direction_t direction);
void
nvadsp_dram_unmap_page(struct device *nvadsp_dev,
		       nvadsp_iova_addr_t iova_addr, size_t size,
		       nvadsp_data_direction_t direction);

void
nvadsp_dram_sync_single_for_cpu(struct device *nvadsp_dev,
				nvadsp_iova_addr_t iova_addr, size_t size,
				nvadsp_data_direction_t direction);
void
nvadsp_dram_sync_single_for_device(struct device *nvadsp_dev,
				   nvadsp_iova_addr_t iova_addr, size_t size,
				   nvadsp_data_direction_t direction);

/*
 * ARAM Bookkeeping
 */
bool nvadsp_aram_request(char *start, size_t size, char *id);
void nvadsp_aram_release(char *start, size_t size);

/*
 * ADSP OS
 */
void nvadsp_adsp_init(void);
int nvadsp_os_load(void);
void nvadsp_os_start(void);
void nvadsp_os_stop(void);

/*
 * ADSP OS App
 */
#define NVADSP_MAX_APP_NAME (64+1)

typedef struct {
	char name[NVADSP_MAX_APP_NAME];
} nvadsp_app_info_t;

nvadsp_app_info_t *nvadsp_app_load(char *appname, char *appfile);
void nvadsp_app_init(nvadsp_app_info_t *);
void nvadsp_app_exit(nvadsp_app_info_t *);
void nvadsp_app_unload(nvadsp_app_info_t *);
int nvadsp_app_start(char *name, void *arg);
int nvadsp_app_stop(char *name);

#endif /* __LINUX_TEGRA_NVADSP_H */
