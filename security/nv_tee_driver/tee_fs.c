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

#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/bitops.h>

#include <asm/uaccess.h>

#include "tee_protocol.h"

#define TEE_SHMEM_FNAME_SZ	SZ_64
#define TEE_SHMEM_DATA_SZ	SZ_128K

#define TEE_FS_READY_BIT	1

struct tee_shmem {
	char	file_name[TEE_SHMEM_FNAME_SZ];
	char	file_data[TEE_SHMEM_DATA_SZ];
};

struct list_head req_list;
DECLARE_COMPLETION(req_ready);
DECLARE_COMPLETION(req_complete);
static unsigned long secure_error;
static unsigned long fs_ready;

static void indicate_complete(unsigned long ret)
{
	asm volatile (
		"mov	r1, %0			\n"
		"movw	r0, #0x1FFF		\n"
		"movt	r0, #0xFFFF		\n"
#ifdef REQUIRES_SEC
		".arch_extension sec		\n"
#endif
		"smc	#0			\n"
		: : "r" (ret)
		: "r0", "r1"
	);
}

int tee_handle_fs_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	TEEC_FileReq new_req, *ptr_user_req = NULL;
	struct tee_file_req_node *req_node;

	switch (ioctl_num) {
	case TEE_IOCTL_FILE_NEW_REQ: /* new request */

		ptr_user_req = (TEEC_FileReq *)ioctl_param;

		set_freezable();

		set_bit(TEE_FS_READY_BIT, &fs_ready);

		/* wait for a new request */
		while (wait_for_completion_interruptible(&req_ready))
			try_to_freeze();

		/* dequeue new request from the secure world */
		req_node = list_first_entry(&req_list, struct tee_file_req_node,
				node);

		/* populate request for the non-secure client */
		if (req_node) {
			if (copy_to_user(ptr_user_req, req_node->req,
				sizeof(TEEC_FileReq))) {
				pr_err("copy_to_user failed for new request\n");
				return -EFAULT;
			}

			list_del(&req_node->node);
			kfree(req_node);
		} else {
			pr_err("no request available\n");
			return -ENOMEM;
		}

		break;

	case TEE_IOCTL_FILE_FILL_BUF: /* pass data to be written to the file */

		if (copy_from_user(&new_req, (void __user *)ioctl_param,
			sizeof(TEEC_FileReq))) {
			pr_err("copy_from_user failed for request\n");
			return -EFAULT;
		}

		if (new_req.type != TEEC_FILE_REQ_WRITE)
			return -EINVAL;

		if (!new_req.kern_data_buf || !new_req.user_data_buf)
			return -EINVAL;

		if (copy_to_user(new_req.user_data_buf, new_req.kern_data_buf,
			new_req.data_len)) {
			pr_err("copy_to_user failed for fill buffer\n");
			return -EFAULT;
		}
		break;

	case TEE_IOCTL_FILE_REQ_COMPLETE: /* request complete */

		if (copy_from_user(&new_req, (void __user *)ioctl_param,
			sizeof(TEEC_FileReq))) {
			pr_err("copy_from_user failed for request\n");
			return -EFAULT;
		}

		if (new_req.type == TEEC_FILE_REQ_READ && !new_req.error) {
			if (copy_from_user(new_req.kern_data_buf,
				(void __user *)new_req.user_data_buf,
				new_req.data_len)) {
				pr_err("copy_from_user failed for request\n");
				return -EFAULT;
			}
		}

		/* get error code */
		secure_error = (new_req.error) ? TEEC_ERROR_NO_DATA : new_req.result;

		/* signal the producer */
		complete(&req_complete);
		break;
	}

	return 0;
}

static void _tee_fs_file_operation(const char *name, void *buf, int len,
		TEEC_FileReqType type)
{
	TEEC_FileReq *new_req;
	struct tee_file_req_node *req_node;

	if (!test_and_clear_bit(TEE_FS_READY_BIT, &fs_ready)) {
		pr_err("%s: daemon not loaded yet\n", __func__);
		secure_error = TEEC_ERROR_NO_DATA;
		goto fail;
	}

	BUG_ON(!name);

	if (type == TEEC_FILE_REQ_READ || type == TEEC_FILE_REQ_WRITE)
		BUG_ON(!buf);

	/* allocate TEEC_FileReq structure */
	new_req = kzalloc(sizeof(TEEC_FileReq), GFP_KERNEL);
	BUG_ON(!new_req);

	/* prepare a new request */
	strncpy(new_req->name, name, strlen(name));
	new_req->type = type;
	new_req->data_len = len;
	new_req->result = 0;
	new_req->kern_data_buf = buf;
	new_req->error = 0;

	req_node = kzalloc(sizeof(struct tee_file_req_node), GFP_KERNEL);
	BUG_ON(!req_node);

	req_node->req = new_req;
	INIT_LIST_HEAD(&req_node->node);

	/* add it to the pending queue and signal the consumer */
	list_add_tail(&req_list, &req_node->node);
	complete(&req_ready);

	set_freezable();

	/* wait for the consumer's signal */
	while (wait_for_completion_interruptible(&req_complete))
		try_to_freeze();

	kfree(new_req);

fail:
	/* signal completion to the secure world */
	indicate_complete(secure_error);
}

void nv_tee_fread(const char *name, void *buf, int len)
{
	if (!buf)
		_tee_fs_file_operation(name, buf, len, TEEC_FILE_REQ_SIZE);
	else
		_tee_fs_file_operation(name, buf, len, TEEC_FILE_REQ_READ);
}

void nv_tee_fwrite(const char *name, void *buf, int len)
{
	_tee_fs_file_operation(name, buf, len, TEEC_FILE_REQ_WRITE);
}

void nv_tee_fdelete(const char *name)
{
	_tee_fs_file_operation(name, NULL, 0, TEEC_FILE_REQ_DELETE);
}

static int __init nv_tee_fs_register_handlers(void)
{
	struct tee_shmem *shmem_ptr;

	shmem_ptr = kzalloc(sizeof(struct tee_shmem), GFP_KERNEL);
	if (!shmem_ptr) {
		pr_err("%s: no memory available for fs operations\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&req_list);
	init_completion(&req_ready);
	init_completion(&req_complete);

	asm volatile (
		"movw	r0, #0x1FF2		\n"
		"movt	r0, #0xFFFF		\n"
		"mov	r1, %0			\n"
		"mov	r2, %1			\n"
		"mov	r3, %2			\n"
		"mov	r4, %3			\n"
		"mov	r5, %4			\n"
#ifdef REQUIRES_SEC
		".arch_extension sec		\n"
#endif
		"smc	#0			\n"
		: : "r" (nv_tee_fread), "r" (nv_tee_fwrite), "r" (nv_tee_fdelete),
		    "r" (shmem_ptr->file_name), "r" (shmem_ptr->file_data)
		: "r0", "r1", "r2", "r3", "r4", "r13", "r14"
	);

	return 0;
}

arch_initcall(nv_tee_fs_register_handlers);
