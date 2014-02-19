/*
 * A Header file for managing ADSP/APE
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __LINUX_TEGRA_NVADSP_H
#define __LINUX_TEGRA_NVADSP_H

#include <linux/types.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
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
 * Mailbox Queue
 */
#define NVADSP_MBOX_QUEUE_SIZE		32
#define NVADSP_MBOX_QUEUE_SIZE_MASK	(NVADSP_MBOX_QUEUE_SIZE - 1)
struct nvadsp_mbox_queue {
	uint32_t array[NVADSP_MBOX_QUEUE_SIZE];
	uint16_t head;
	uint16_t tail;
	uint16_t count;
	struct completion comp;
	spinlock_t lock;
};

status_t nvadsp_mboxq_enqueue(struct nvadsp_mbox_queue *, uint32_t);

/*
 * Mailbox
 */
#define NVADSP_MBOX_NAME_MAX (16 + 1)

typedef status_t (*nvadsp_mbox_handler_t)(uint32_t, void *);

struct nvadsp_mbox {
	uint16_t id;
	char name[NVADSP_MBOX_NAME_MAX];
	struct nvadsp_mbox_queue recv_queue;
	nvadsp_mbox_handler_t handler;
	void *hdata;
};

#define NVADSP_MBOX_SMSG       0x1
#define NVADSP_MBOX_LMSG       0x2

status_t nvadsp_mbox_open(struct nvadsp_mbox *mbox, uint16_t *mid, char *name,
			  nvadsp_mbox_handler_t handler, void *hdata);
status_t nvadsp_mbox_send(struct nvadsp_mbox *mbox, uint32_t data,
			  uint32_t flags, bool block, unsigned int timeout);
status_t nvadsp_mbox_recv(struct nvadsp_mbox *mbox, uint32_t *data, bool block,
			  unsigned int timeout);
status_t nvadsp_mbox_close(struct nvadsp_mbox *mbox);

status_t nvadsp_hwmbox_send_data(uint16_t, uint32_t, uint32_t);

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
