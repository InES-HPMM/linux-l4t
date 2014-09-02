/*
 * Copyright (c) 2013-2014 NVIDIA Corporation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>

#include "ote_protocol.h"

static DECLARE_COMPLETION(req_ready);
static DECLARE_COMPLETION(req_complete);

static struct te_ss_op *ss_op_shmem;

int te_handle_ss_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	int ss_cmd;

	if (ioctl_num != TE_IOCTL_SS_CMD)
		return -EINVAL;

	if (copy_from_user(&ss_cmd, (void __user *)ioctl_param,
				sizeof(ss_cmd))) {
		pr_err("%s: copy from user space failed\n", __func__);
		return -EFAULT;
	}

	switch (ss_cmd) {
	case TE_IOCTL_SS_CMD_GET_NEW_REQ:
		/* wait for a new request */
		if (wait_for_completion_interruptible(&req_ready))
			return -ENODATA;
		break;
	case TE_IOCTL_SS_CMD_REQ_COMPLETE:
		/* signal the producer */
		complete(&req_complete);
		break;
	default:
		pr_err("%s: unknown ss_cmd 0x%x\n", __func__, ss_cmd);
		return -EINVAL;
	}

	return 0;
}

void tlk_ss_op(void)
{
	/* signal consumer */
	complete(&req_ready);

	/* wait for the consumer's signal */
	wait_for_completion(&req_complete);
}

static int __init tlk_ss_init(void)
{
	dma_addr_t ss_op_shmem_dma;
	int32_t ret;

	/* allocate shared memory buffer */
	ss_op_shmem = dma_alloc_coherent(NULL, sizeof(struct te_ss_op),
			&ss_op_shmem_dma, GFP_KERNEL);
	if (!ss_op_shmem) {
		pr_err("%s: no memory available for fs operations\n", __func__);
		return -ENOMEM;
	}

	ret = tlk_generic_smc(TE_SMC_SS_REGISTER_HANDLER,
			(uintptr_t)ss_op_shmem, 0);
	if (ret != 0) {
		dma_free_coherent(NULL, sizeof(struct te_ss_op),
			(void *)ss_op_shmem, ss_op_shmem_dma);
		ss_op_shmem = NULL;
		return -ENOTSUPP;
	}

	return 0;
}

arch_initcall(tlk_ss_init);
