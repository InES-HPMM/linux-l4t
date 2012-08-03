/*
 * Copyright (C) 2012 NVIDIA Corporation.
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

#ifndef _NVSHM_PRIV_H
#define _NVSHM_PRIV_H

#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/semaphore.h>
#include <linux/module.h>

/*
 * Test stub is used to implement nvshm on private memory for testing purpose.
 * Data are allocated into this private memory but queues loop on themselves
 */
#define NVSHM_TEST_STUB

/* Generate NVSHM_IPC MSG */
#define NVSHM_IPC_MESSAGE(id) (((~id & 0xFFFF) << 16) | (id & 0xFFFF))

/* Flags for descriptors */
#define NVSHM_DESC_AP    (0x01)  /* AP descriptor ownership */
#define NVSHM_DESC_BB    (0x02)  /* BB descriptor ownership */
#define NVSHM_DESC_OPEN  (0x04)  /* OOB channel open */
#define NVSHM_DESC_CLOSE (0x08)  /* OOB channel close */
#define NVSHM_DESC_XOFF  (0x10)  /* OOB channel Tx off */
#define NVSHM_DESC_XON   (0x20)  /* OOB channel Tx on */

struct nvshm_handle {
	spinlock_t lock;
	int instance;
	int old_status;
	struct nvshm_config *conf;
	void *ipc_base_virt;
	void *mb_base_virt;
	void *desc_base_virt; /* AP desc region */
	void *data_base_virt; /* AP data region */
	unsigned long ipc_size;
	unsigned long mb_size;
	unsigned long desc_size;
	unsigned long data_size;
	struct nvshm_iobuf *shared_queue_head; /* shared desc list */
	struct nvshm_iobuf *shared_queue_tail; /* shared desc list */
	struct nvshm_iobuf *free_pool_head;    /* free desc list */
	struct nvshm_channel chan[NVSHM_MAX_CHANNELS];
	struct work_struct nvshm_work;
	struct workqueue_struct *nvshm_wq;
	char wq_name[16];
	struct device *dev;
	struct hrtimer queue_timer;
	void *ipc_data;
	void (*generate_ipc)(void *ipc_data);
	struct platform_device *tegra_bb;
};

extern inline struct nvshm_handle *nvshm_get_handle(void);

extern int nvshm_tty_init(struct nvshm_handle *handle);
extern void nvshm_tty_cleanup(void);

#endif /* _NVSHM_PRIV_H */
