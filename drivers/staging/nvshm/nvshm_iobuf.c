/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_ipc.h"
#include "nvshm_iobuf.h"
#include "nvshm_queue.h"
/*
 * really simple allocator: data is divided of chunk of equal size
 *  since iobuf are mainly for tty/net traffic which is well below 8k
 */

#define NVSHM_DEFAULT_OFFSET 32

struct nvshm_allocator {
	spinlock_t lock;
	/* AP free iobufs */
	struct nvshm_iobuf *free_pool_head;
	struct nvshm_iobuf *free_pool_tail;
	/* Freed BBC iobuf to be returned */
	struct nvshm_iobuf *bbc_pool_head;
	struct nvshm_iobuf *bbc_pool_tail;
	int nbuf;
};

static struct nvshm_allocator alloc;

/* Accumulate BBC freed iobuf to return them later at end of rx processing */
/* This saves a lot of CPU/memory cycles on both sides */
static void bbc_free(struct nvshm_handle *handle, struct nvshm_iobuf *iob)
{
	if (alloc.bbc_pool_head) {
		alloc.bbc_pool_tail->next = NVSHM_A2B(handle, iob);
		alloc.bbc_pool_tail = iob;
	} else {
		alloc.bbc_pool_head = alloc.bbc_pool_tail = iob;
	}
}

/* Effectively free all iobufs accumulated */
void nvshm_iobuf_bbc_free(struct nvshm_handle *handle)
{
	struct nvshm_iobuf *iob = NULL;
	unsigned long f;

	spin_lock_irqsave(&alloc.lock, f);
	if (alloc.bbc_pool_head) {
		iob = alloc.bbc_pool_head;
		alloc.bbc_pool_head = alloc.bbc_pool_tail = NULL;
	}
	spin_unlock_irqrestore(&alloc.lock, f);
	if (iob) {
		nvshm_queue_put(handle, iob);
		nvshm_generate_ipc(handle);
	}
}

struct nvshm_iobuf *nvshm_iobuf_alloc(struct nvshm_channel *chan, int size)
{
	struct nvshm_handle *handle = nvshm_get_handle();
	struct nvshm_iobuf *desc = NULL;
	unsigned long f;

	spin_lock_irqsave(&alloc.lock, f);
	if (alloc.free_pool_head) {
		int check = nvshm_iobuf_check(alloc.free_pool_head);

		if (check) {
			pr_err("%s: iobuf check ret %d\n", __func__, check);
			return NULL;
		}
		if (size > (alloc.free_pool_head->total_length -
			    NVSHM_DEFAULT_OFFSET)) {
			spin_unlock_irqrestore(&alloc.lock, f);
			pr_err("%s: requested size (%d > %d) too big\n",
			       __func__,
			       size,
			       alloc.free_pool_head->total_length -
			       NVSHM_DEFAULT_OFFSET);
			if (chan->ops) {
				chan->ops->error_event(chan,
						       NVSHM_IOBUF_ERROR);
			}
			return desc;
		}
		desc = alloc.free_pool_head;
		alloc.free_pool_head = desc->next;
		if (alloc.free_pool_head) {
			alloc.free_pool_head = NVSHM_B2A(handle,
							 alloc.free_pool_head);
		} else {
			pr_debug("%s end of alloc queue - clearing tail\n",
				__func__);
			alloc.free_pool_tail = NULL;
		}
		desc->length = 0;
		desc->flags = 0;
		desc->data_offset = NVSHM_DEFAULT_OFFSET;
		desc->sg_next = NULL;
		desc->next = NULL;
		desc->ref = 1;

	} else {
		spin_unlock_irqrestore(&alloc.lock, f);
		pr_err("%s: no more alloc space\n", __func__);
		/* No error since it's only Xoff situation */
		return desc;
	}

	spin_unlock_irqrestore(&alloc.lock, f);

	return desc;
}

/** Returned iobuf are already freed - just process them */
void nvshm_iobuf_process_freed(struct nvshm_iobuf *desc)
{
	struct nvshm_handle *priv = nvshm_get_handle();
	unsigned long f;

	while (desc) {
		int callback = 0, chan;
		struct nvshm_iobuf *next = desc->next;

		if (desc->ref != 0) {
			pr_err("%s: BBC returned an non freed iobuf (0x%x)\n",
			       __func__,
			       (unsigned int)desc);
			return;
		}

		chan = desc->chan;
		spin_lock_irqsave(&alloc.lock, f);
		/* update rate counter */
		if ((chan >= 0) &&
		    (chan < NVSHM_MAX_CHANNELS)) {
			if (priv->chan[chan].rate_counter++ ==
			    NVSHM_RATE_LIMIT_TRESHOLD)
				callback = 1;
		}
		desc->sg_next = NULL;
		desc->next = NULL;
		desc->length = 0;
		desc->flags = 0;
		desc->data_offset = 0;
		desc->chan = 0;
		if (alloc.free_pool_tail) {
			alloc.free_pool_tail->next = NVSHM_A2B(priv,
							       desc);
			alloc.free_pool_tail = desc;
		} else {
			alloc.free_pool_head = desc;
				alloc.free_pool_tail = desc;
		}
		spin_unlock_irqrestore(&alloc.lock, f);
		if (callback)
			nvshm_start_tx(&priv->chan[chan]);
		if (next) {
			desc = NVSHM_B2A(priv, next);
		} else {
			desc = next;
		}
	}
}

/** Single iobuf free - do not follow iobuf links */
void nvshm_iobuf_free(struct nvshm_iobuf *desc)
{
	struct nvshm_handle *priv = nvshm_get_handle();
	int callback = 0, chan;
	unsigned long f;

	if (desc->ref == 0) {
		pr_err("%s: freeing an already freed iobuf (0x%x)\n",
		       __func__,
		       (unsigned int)desc);
		return;
	}
	spin_lock_irqsave(&alloc.lock, f);
	pr_debug("%s: free 0x%p ref %d pool %x\n", __func__,
		 desc, desc->ref, desc->pool_id);
	desc->ref--;
	chan = desc->chan;
	if (desc->ref == 0) {
		if (desc->pool_id >= NVSHM_AP_POOL_ID) {
			/* update rate counter */
			if ((chan >= 0) &&
			    (chan < NVSHM_MAX_CHANNELS)) {
				if (priv->chan[chan].rate_counter++ ==
				    NVSHM_RATE_LIMIT_TRESHOLD)
					callback = 1;
			}
			desc->sg_next = NULL;
			desc->next = NULL;
			desc->length = 0;
			desc->flags = 0;
			desc->data_offset = 0;
			desc->chan = 0;
			if (alloc.free_pool_tail) {
				alloc.free_pool_tail->next = NVSHM_A2B(priv,
								       desc);
				alloc.free_pool_tail = desc;
			} else {
				alloc.free_pool_head = desc;
				alloc.free_pool_tail = desc;
			}
		} else {
			/* iobuf belongs to other side */
			pr_debug("%s: re-queue freed buffer\n", __func__);
			desc->sg_next = NULL;
			desc->next = NULL;
			desc->length = 0;
			desc->data_offset = 0;
			bbc_free(priv, desc);
			spin_unlock_irqrestore(&alloc.lock, f);
			return;
		}
	}
	spin_unlock_irqrestore(&alloc.lock, f);
	if (callback)
		nvshm_start_tx(&priv->chan[chan]);
}

void nvshm_iobuf_free_cluster(struct nvshm_iobuf *list)
{
	struct nvshm_handle *priv = nvshm_get_handle();
	struct nvshm_iobuf *_phy_list, *_to_free, *leaf;
	int n = 0;

	_phy_list = list;
	while (_phy_list) {
		_to_free = list;
		if (list->sg_next) {
			_phy_list = list->sg_next;
			if (_phy_list) {
				leaf = NVSHM_B2A(priv, _phy_list);
				leaf->next = list->next;
			}
		} else {
			_phy_list = list->next;
		}
		list = NVSHM_B2A(priv, _phy_list);
		n++;
		nvshm_iobuf_free(_to_free);
	}
}

int nvshm_iobuf_ref(struct nvshm_iobuf *iob)
{
	int ref;
	unsigned long f;

	spin_lock_irqsave(&alloc.lock, f);
	ref = iob->ref++;
	spin_unlock_irqrestore(&alloc.lock, f);
	return ref;
}

int nvshm_iobuf_unref(struct nvshm_iobuf *iob)
{
	int ref;
	unsigned long f;

	spin_lock_irqsave(&alloc.lock, f);
	ref = iob->ref--;
	spin_unlock_irqrestore(&alloc.lock, f);
	return ref;
}

int nvshm_iobuf_ref_cluster(struct nvshm_iobuf *iob)
{
	int ref, ret = 0;
	struct nvshm_iobuf *_phy_list, *_phy_leaf;
	struct nvshm_handle *handle = nvshm_get_handle();

	_phy_list = iob;
	while (_phy_list) {
		_phy_leaf = _phy_list;
		while (_phy_leaf) {
			ref = nvshm_iobuf_ref(_phy_leaf);
			ret = (ref > ret) ? ref : ret;
			if (_phy_leaf->sg_next) {
				_phy_leaf = NVSHM_B2A(handle,
						      _phy_leaf->sg_next);
			} else {
				_phy_leaf = NULL;
			}
		}
		if (_phy_list->next)
			_phy_list = NVSHM_B2A(handle, _phy_list->next);
		else
			_phy_list = NULL;
	}
	return ret;
}

int nvshm_iobuf_unref_cluster(struct nvshm_iobuf *iob)
{
	int ref, ret = 0;
	struct nvshm_iobuf *_phy_list, *_phy_leaf;
	struct nvshm_handle *handle = nvshm_get_handle();

	_phy_list = iob;
	while (_phy_list) {
		_phy_leaf = _phy_list;
		while (_phy_leaf) {
			ref = nvshm_iobuf_unref(_phy_leaf);
			ret = (ref > ret) ? ref : ret;
			if (_phy_leaf->sg_next) {
				_phy_leaf = NVSHM_B2A(handle,
						      _phy_leaf->sg_next);
			} else {
				_phy_leaf = NULL;
			}
		}
		if (_phy_list->next)
			_phy_list = NVSHM_B2A(handle, _phy_list->next);
		else
			_phy_list = NULL;
	}

	return ret;
}

int nvshm_iobuf_flags(struct nvshm_iobuf *iob,
		      unsigned int set,
		      unsigned int clear)
{
	iob->flags &= ~(clear & 0xFFFF);
	iob->flags |= set & 0xFFFF;
	return 0;
}

int nvshm_iobuf_check(struct nvshm_iobuf *iob)
{
	struct nvshm_handle *priv = nvshm_get_handle();

	/* Check iobuf is in IPC space */
	if (ADDR_OUTSIDE(iob, priv->ipc_base_virt, priv->ipc_size)) {
		pr_err("%s: iob @ check failed 0x%lx\n",
		       __func__,
		       (long)iob);
		return -1;
	}

	if (ADDR_OUTSIDE(iob->npdu_data, NVSHM_IPC_BB_BASE, priv->ipc_size)) {
		pr_err("%s: npdu_data @ check failed 0x%lx\n",
		       __func__,
		       (long)iob->npdu_data);
		return -2;
	}
	if (ADDR_OUTSIDE(iob->npdu_data + iob->data_offset,
			NVSHM_IPC_BB_BASE, priv->ipc_size)) {
		pr_err("%s: npdu_data + offset @ check failed 0x%lx/0x%lx\n",
		       __func__, (long)iob->npdu_data, (long)iob->data_offset);
		return -3;
	}
	if (iob->next) {
		if (ADDR_OUTSIDE(iob->next,
				NVSHM_IPC_BB_BASE, priv->ipc_size)) {
			pr_err("%s: next @ check failed 0x%lx\n",
			       __func__,
			       (long)iob->next);
			return -4;
		}
	}
	if (iob->sg_next) {
		if (ADDR_OUTSIDE(iob->sg_next,
				NVSHM_IPC_BB_BASE, priv->ipc_size)) {
			pr_err("%s:sg_next @ check failed 0x%lx\n",
			       __func__, (long)iob->sg_next);
			return -5;
		}
	}
	if (iob->qnext) {
		if (ADDR_OUTSIDE(iob->qnext,
				NVSHM_IPC_BB_BASE, priv->ipc_size)) {
			pr_err("%s:qnext @ check failed 0x%lx\n",
			       __func__, (long)iob->qnext);
			return -6;
		}
	}
	return 0;
}

#ifdef IOBUF_CHECK
static int iobuf_sanity_check(struct nvshm_handle *handle)
{
	struct nvshm_iobuf *list = NULL, *iob = NULL;
	int n = 0;
	int pass, lastpass;

	/* Test size limit */
	pr_debug("%s Alloc request too big. Expect: failure\n", __func__);
	iob = nvshm_iobuf_alloc(&handle->chan[0], 1000000);
	if (iob != NULL) {
		pr_err("%s: alloc size limit test failed\n", __func__);
		return -1;
	}
	pr_debug("%s Alloc request too big. Passed\n", __func__);

	list = iob = NULL;

	pr_debug("%s Alloc test. Expect: success\n", __func__);
	iob = nvshm_iobuf_alloc(&handle->chan[0], 10);
	if (iob == NULL) {
		pr_err("%s: alloc test failed\n", __func__);
		return -1;
	}
	pr_debug("%s Alloc test. passed\n", __func__);

	pr_debug("%s free test. Expect: success\n", __func__);
	nvshm_iobuf_free(iob);
	pr_debug("%s free test. passed\n", __func__);

	pr_debug("%s double free test. Expect: failure\n", __func__);
	/* Test double free (harmless but error msg in dmesg */
	nvshm_iobuf_free(iob);
	pr_debug("%s double free test. passed\n", __func__);

	lastpass = 0;
	for (pass = 0; pass < 2; pass++) {
		list = iob = NULL;
		n = 0;
		pr_debug("%s Mass alloc #%d. Exp: failure after #%ld alloc\n",
			__func__,
			pass,
			handle->desc_size / sizeof(struct nvshm_iobuf));
		/* Test alloc limit */
		do {
			iob = nvshm_iobuf_alloc(&handle->chan[0], 100);
			if (iob)
				n++;
			if (list) {
				if (iob) {
					iob->next = NVSHM_A2B(handle, list);
					list = iob;
				}
			} else {
				list = iob;
			}
		} while (iob);
		if (lastpass) {
			if (n < lastpass) {
				pr_err("%s Mass alloc #%d. last %d curr %d\n",
				       __func__, pass, lastpass, n);
				return -1;
			}
		}
		lastpass = n;
		pr_debug("%s Mass alloc #%d. Num of alloc=%d\n",
			__func__, pass, n);

		pr_debug("%s free cluster test #%d. Expect: success\n",
			__func__, pass);
		nvshm_iobuf_free_cluster(list);
		pr_debug("%s free cluster test #%d. passed\n", __func__, pass);
	}

	lastpass = 0;
	for (pass = 0; pass < 2; pass++) {
		list = iob = NULL;
		n = 0;
		pr_debug("%s Mass alloc (trans) #%d. Exp:  fail after #%ld\n",
			__func__,
			pass,
			handle->desc_size / sizeof(struct nvshm_iobuf));
		/* Test alloc limit */
		do {
			iob = nvshm_iobuf_alloc(&handle->chan[0], 100);
			if (iob)
				n++;

			if (list) {
				if (iob) {
					iob->sg_next = NVSHM_A2B(handle, list);
					list = iob;
				}
			} else {
				list = iob;
			}
		} while (iob);
		if (lastpass)
			if (n < lastpass) {
				pr_err("%s Mass alloc (trans) #%d. %d/%d\n",
				       __func__, pass, lastpass, n);
				return -1;
			}
		lastpass = n;
		pr_debug("%s Mass alloc (trans) #%d. Num=%d\n",
			__func__, pass, n);

		pr_debug("%s free cluster test (trans) #%d. Expect: success\n",
			__func__, pass);
		nvshm_iobuf_free_cluster(list);
		pr_debug("%s free cluster test (trans) #%d. passed\n",
			__func__, pass);
	}
	return 0;
}
#endif

int nvshm_iobuf_init(struct nvshm_handle *handle)
{
	struct nvshm_iobuf *iob;
	int ndesc, desc, datasize;
	unsigned char *dataptr;

	pr_debug("%s instance %d\n", __func__, handle->instance);

	spin_lock_init(&alloc.lock);
	ndesc = handle->desc_size / sizeof(struct nvshm_iobuf) ;
	if (ndesc < 64) {
		pr_err("%s: no enougth space for serious tests ndesc=%d\n",
		       __func__, ndesc);
		return -1;
	}
	alloc.nbuf = ndesc;
	datasize =  handle->data_size / ndesc;
	if (datasize < 2048) {
		pr_err("%s: no enougth space for serious tests data size=%d\n",
		       __func__, datasize);
		return -1;
	}
	spin_lock(&alloc.lock);
	if (handle->shared_queue_tail != handle->desc_base_virt) {
		pr_err("%s initial tail != desc_base_virt not supported yet\n",
		       __func__);
	}
	iob = (struct nvshm_iobuf *)handle->desc_base_virt;

	dataptr = handle->data_base_virt;
	/* Dummy queue element */
	memset(handle->desc_base_virt, 0, handle->desc_size);
	iob->npdu_data = NVSHM_A2B(handle, dataptr);
	dataptr += datasize;
	iob->data_offset = NVSHM_DEFAULT_OFFSET;
	iob->total_length = datasize;
	iob->next = NULL;
	iob->pool_id = NVSHM_AP_POOL_ID;
	iob->ref = 1;
	alloc.free_pool_head = ++iob;
	for (desc = 1; desc < (ndesc-1); desc++) {
		iob->npdu_data = NVSHM_A2B(handle, dataptr);
		dataptr += datasize;
		iob->data_offset = NVSHM_DEFAULT_OFFSET;
		iob->total_length = datasize;
		iob->next = NVSHM_A2B(handle, (void *)iob +
				      sizeof(struct nvshm_iobuf));
		iob->pool_id = NVSHM_AP_POOL_ID;
		iob++;
	}
	/* Untied last */
	iob->npdu_data = NVSHM_A2B(handle, dataptr);
	iob->data_offset = NVSHM_DEFAULT_OFFSET;
	iob->total_length = datasize;
	iob->pool_id = NVSHM_AP_POOL_ID;
	iob->next = NULL;

	alloc.free_pool_tail = iob;
	FLUSH_CPU_DCACHE(alloc.free_pool_head,
			 (long)alloc.free_pool_tail
			 - (long)alloc.free_pool_head);
	spin_unlock(&alloc.lock);
#ifdef IOBUF_CHECK
	if (iobuf_sanity_check(handle))
		pr_err("%s iobuf sanity check failure!\n", __func__);
#endif
	return 0;
}
