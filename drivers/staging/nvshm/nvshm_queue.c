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

#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_ipc.h"
#include "nvshm_queue.h"
#include "nvshm_iobuf.h"

#include <mach/tegra_bb.h>

/* Flush cache lines associated with iobuf list */
static void flush_iob_list(struct nvshm_handle *handle, struct nvshm_iobuf *iob)
{
	struct nvshm_iobuf *phy_list, *leaf, *next, *sg_next;
	phy_list = iob;
	while (phy_list) {
		leaf = phy_list;
		next = phy_list->next;
		while (leaf) {
			sg_next = leaf->sg_next;
			/* Flush associated data */
			if (leaf->length) {
				FLUSH_CPU_DCACHE(NVSHM_B2A(handle,
							   (int)leaf->npduData
							   + leaf->dataOffset),
						 leaf->length);
			}
			/* Flush iobuf */
			FLUSH_CPU_DCACHE(leaf, sizeof(struct nvshm_iobuf));
			if (sg_next)
				leaf = NVSHM_B2A(handle, sg_next);
			else
				leaf = NULL;
		}
		if (next)
			phy_list = NVSHM_B2A(handle, next);
		else
			phy_list = NULL;
	}
}

/* Invalidate cache lines associated with iobuf list */
static void inv_iob_list(struct nvshm_handle *handle, struct nvshm_iobuf *iob)
{
	struct nvshm_iobuf *phy_list, *leaf;

	phy_list = iob;
	while (phy_list) {
		leaf = phy_list;
		while (leaf) {
			/* Invalidate first iobuf */
			INV_CPU_DCACHE(leaf, sizeof(struct nvshm_iobuf));
			/* Invalidate associated data */
			if (leaf->length) {
				INV_CPU_DCACHE(NVSHM_B2A(handle,
							   (int)leaf->npduData
							   + leaf->dataOffset),
							   leaf->length);
			}
			if (leaf->sg_next)
				leaf = NVSHM_B2A(handle, leaf->sg_next);
			else
				leaf = NULL;
		}
		if (phy_list->next)
			phy_list = NVSHM_B2A(handle, phy_list->next);
		else
			phy_list = NULL;
	}
}

struct nvshm_iobuf *nvshm_queue_get(struct nvshm_handle *handle)
{
	struct nvshm_iobuf *dummy, *ret;

	if (!handle->shared_queue_head) {
		pr_err("%s: Queue not init!\n", __func__);
		return NULL;
	}

	dummy = handle->shared_queue_head;
	/* Invalidate lower part of iobuf - upper part can be written by AP */
	INV_CPU_DCACHE(&dummy->qnext,
		       sizeof(struct nvshm_iobuf) / 2);
	ret = NVSHM_B2A(handle, dummy->qnext);

	if (dummy->qnext == NULL)
		return NULL;

	inv_iob_list(handle, ret);
	handle->shared_queue_head = ret;

	/* Update queue_bb_offset for debug purpose */
	handle->conf->queue_bb_offset = (int)ret
		- (int)handle->ipc_base_virt;

	if ((handle->conf->queue_bb_offset < 0) ||
	    (handle->conf->queue_bb_offset > handle->conf->shmem_size))
		pr_err("%s: out of bound descriptor offset %d addr 0x%p/0x%p\n",
		       __func__,
		       handle->conf->queue_bb_offset,
		       ret,
		       NVSHM_A2B(handle, ret));

	pr_debug("%s (%p)->%p->(%p)\n", __func__,
		 dummy, ret, ret->qnext);

	nvshm_iobuf_free(dummy);
	return ret;
}

int nvshm_queue_put(struct nvshm_handle *handle, struct nvshm_iobuf *iob)
{
	unsigned long f;

	spin_lock_irqsave(&handle->qlock, f);
	if (!handle->shared_queue_tail) {
		spin_unlock_irq(&handle->qlock);
		pr_err("%s: Queue not init!\n", __func__);
		return -EINVAL;
	}

	if (!iob) {
		pr_err("%s: Queueing null pointer!\n", __func__);
		spin_unlock_irqrestore(&handle->qlock, f);
		return -EINVAL;
	}

	/* Sanity check */
	if (handle->shared_queue_tail->qnext) {
		pr_err("%s: illegal queue pointer detected!\n", __func__);
		spin_unlock_irqrestore(&handle->qlock, f);
		return -EINVAL;
	}

	pr_debug("%s (%p)->%p/%d/%d->%p\n", __func__,
		handle->shared_queue_tail,
		iob, iob->chan, iob->length,
		iob->next);

	/* Take a reference on queued iobuf */
	nvshm_iobuf_ref(iob);
	/* Flush iobuf(s) in cache */
	flush_iob_list(handle, iob);
	handle->shared_queue_tail->qnext = NVSHM_A2B(handle, iob);
	/* Flush guard element from cache */
	FLUSH_CPU_DCACHE(handle->shared_queue_tail, sizeof(struct nvshm_iobuf));
	handle->shared_queue_tail = iob;

	spin_unlock_irqrestore(&handle->qlock, f);
	return 0;
}

int nvshm_init_queue(struct nvshm_handle *handle)
{

	pr_debug("%s instance %d\n", __func__, handle->instance);
	/* Catch config issues */
	if ((!handle->ipc_base_virt) || (!handle->desc_base_virt)) {
		pr_err("%s IPC or DESC base not defined!", __func__);
		return -ENOMEM;
	}

	if ((handle->desc_size % sizeof(struct nvshm_iobuf))) {
		pr_err("%s DESC zone illegal size!", __func__);
		return -EINVAL;
	}
	return 0;
}
/*
 * Called from IPC workqueue
 */
void nvshm_process_queue(struct nvshm_handle *handle)
{
	struct nvshm_iobuf *iob;
	struct nvshm_if_operations *ops;
	int chan;

	spin_lock_bh(&handle->lock);
	iob = nvshm_queue_get(handle);
	while (iob) {
		pr_debug("%s %p/%d/%d/%d->%p\n", __func__,
			iob, iob->chan, iob->length, iob->ref, iob->next);
		tegra_bb_clear_ipc(handle->tegra_bb);
		chan = iob->chan;
		if (iob->pool_id < NVSHM_AP_POOL_ID) {
			ops = handle->chan[chan].ops;
			if (ops) {
				spin_unlock_bh(&handle->lock);
				ops->rx_event(
					&handle->chan[chan],
					iob);
				spin_lock_bh(&handle->lock);
			} else {
				nvshm_iobuf_free_cluster(
					iob);
			}
		}
		iob = nvshm_queue_get(handle);
	}
	spin_unlock_bh(&handle->lock);
}

void nvshm_abort_queue(struct nvshm_handle *handle)
{
	pr_debug("%s:abort queue\n", __func__);
}

