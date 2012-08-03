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

#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_ipc.h"
#include "nvshm_queue.h"
#include "nvshm_iobuf.h"

struct nvshm_iobuf *nvshm_queue_get(struct nvshm_handle *handle)
{
	struct nvshm_iobuf *dummy, *ret;

	if (!handle->shared_queue_head) {
		pr_err("%s: Queue not init!\n", __func__);
		return NULL;
	}
	dummy = handle->shared_queue_head;
	ret = NVSHM_B2A(handle, handle->shared_queue_head->qnext);

	if (dummy->qnext == NULL)
		return NULL;

	dummy->qnext = NULL;
	handle->shared_queue_head = ret;
	nvshm_iobuf_free_cluster(&handle->chan[dummy->chan], dummy);
	return ret;
}

int nvshm_queue_put(struct nvshm_handle *handle, struct nvshm_iobuf *iob)
{
	if (!handle->shared_queue_tail) {
		pr_err("%s: Queue not init!\n", __func__);
		return -EINVAL;
	}
	/* Sanity check */
	if (handle->shared_queue_tail->qnext) {
		pr_err("%s: illegal queue pointer detected!\n", __func__);
		return -EINVAL;
	}
	/* Take a reference on queued iobufs (all of them!) */
	nvshm_iobuf_ref_cluster(iob);
	handle->shared_queue_tail->qnext = NVSHM_A2B(handle, iob);
	handle->shared_queue_tail = iob;
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
	int chan;

	spin_lock_bh(&handle->lock);
	iob = nvshm_queue_get(handle);
	while (iob) {
		if (iob->pool_id >= NVSHM_AP_POOL_ID) {
			chan = iob->chan;
			if (handle->chan[chan].ops) {
				spin_unlock_bh(&handle->lock);
				handle->chan[chan].ops->rx_event(
					&handle->chan[chan],
					iob);
				spin_lock_bh(&handle->lock);
			} else {
				nvshm_iobuf_free_cluster(
					&handle->chan[chan],
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

