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

#include <linux/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <linux/list.h>

#include "tee_protocol.h"
#include "tee_types.h"

#define SET_ANSWER(a, r, ro)	{ a.result = r; a.return_origin = ro; }

#define CREATE_TRACE_POINTS
#include <trace/events/nvsecurity.h>

u32 notrace tegra_read_cycle(void)
{
	u32 cycle_count;

	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(cycle_count));

	return cycle_count;
}

struct nv_device tee_nv_dev;

/*
 * The maximum number of outstanding command requests.
 */
#define NV_CMD_DESC_MAX		(PAGE_SIZE / sizeof(struct tee_request))

static int tee_create_free_cmd_list(struct nv_device *dev)
{
	struct page *tee_cmd_page;
	int cmd_desc_count = 0, ret = 0;
	struct nv_cmd_param_desc *param_desc;

	tee_cmd_page =  alloc_pages(GFP_KERNEL, 1);
	if (!tee_cmd_page) {
		ret = -ENOMEM;
		goto error;
	}
	set_pages_array_uc(&tee_cmd_page, 1);
	dev->param_addr = (unsigned long) page_address(tee_cmd_page);

	for (cmd_desc_count = 0;
		cmd_desc_count < NV_CMD_DESC_MAX; cmd_desc_count++) {

		param_desc = kzalloc(
					sizeof(struct nv_cmd_param_desc),
					GFP_KERNEL);
		if (param_desc == NULL) {
			pr_err("Failed to allocate cmd param descriptor\n");
			ret = -ENOMEM;
			goto error;
		}
		param_desc->param_addr =
			dev->param_addr +
			sizeof(struct tee_request) * cmd_desc_count;
		INIT_LIST_HEAD(&(param_desc->list));

		/*Add the cmd param descriptor to free list*/
		list_add_tail(&param_desc->list, &(dev->free_cmd_list));
	}
error:
	return ret;

}

static struct nv_cmd_param_desc *tee_get_free_cmd_desc(struct nv_device *nv_dev)
{
	struct nv_cmd_param_desc *cmd_desc = NULL;

	if (!(list_empty(&(nv_dev->free_cmd_list)))) {
		cmd_desc = list_first_entry(&(nv_dev->free_cmd_list),
				struct nv_cmd_param_desc, list);
		list_del(&(cmd_desc->list));
		list_add_tail(&cmd_desc->list, &(nv_dev->used_cmd_list));
	}
	return cmd_desc;
}

static void tee_put_used_cmd_desc(struct nv_device *nv_dev,
	struct nv_cmd_param_desc *cmd_desc)
{
	struct nv_cmd_param_desc *param_desc, *tmp_param_desc;

	if (cmd_desc) {
		list_for_each_entry_safe(param_desc, tmp_param_desc,
				&(nv_dev->used_cmd_list), list) {
			if (cmd_desc->param_addr ==
					param_desc->param_addr) {
				list_del(&param_desc->list);
				list_add_tail(&param_desc->list,
					&(nv_dev->free_cmd_list));
			}
		}
	}
}

static void tee_print_cmd_list(struct nv_device *dev, int used_list)
{
	struct nv_cmd_param_desc *param_desc;

	if (!used_list) {
		pr_info("Printing free cmd list\n");
		if (!(list_empty(&(dev->free_cmd_list)))) {
			list_for_each_entry(param_desc, &(dev->free_cmd_list),
					list)
				pr_info("Phys addr for cmd param desc (%X)\n",
					param_desc->param_addr);
		}
	} else {
		pr_info("Printing used cmd list\n");
		if (!(list_empty(&(dev->used_cmd_list)))) {
			list_for_each_entry(param_desc, &(dev->used_cmd_list),
					list)
				pr_info("Phys addr for cmd param desc (%X)\n",
					param_desc->param_addr);
		}
	}
}

static int tee_device_open(struct inode *inode, struct file *file)
{
	struct nv_tee_context *context;
	int ret = 0;

	context = kzalloc(
			sizeof(struct nv_tee_context), GFP_KERNEL);
	if (!context) {
		ret = -ENOMEM;
		goto error;
	}
	context->dev = &tee_nv_dev;
	INIT_LIST_HEAD(&(context->shmem_alloc_list));

	file->private_data = context;
	return 0;
error:
	return ret;
}

static int tee_device_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	file->private_data = NULL;
	return 0;
}

static long tee_handle_trustedapp_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	long err = 0;
	union tee_cmd cmd;
	void *ptr_user_answer = NULL;
	struct tee_answer answer;
	struct tee_request *request;
	struct nv_cmd_param_desc *cmd_desc = NULL;
	struct nv_tee_context *context = file->private_data;
	struct nv_device *nv_dev = context->dev;

	if (copy_from_user(&cmd, (void __user *)ioctl_param,
				sizeof(union tee_cmd))) {
		pr_err("Failed to copy command request\n");
		err = -EFAULT;
		goto error;
	}

	memset(&answer, 0, sizeof(struct tee_answer));
	switch (ioctl_num) {
	case TEE_IOCTL_OPEN_CLIENT_SESSION:
		ptr_user_answer = (void *)cmd.opensession.answer;

		cmd_desc = tee_get_free_cmd_desc(nv_dev);
		if (!cmd_desc) {
			SET_ANSWER(answer,
				   TEEC_ERROR_OUT_OF_MEMORY,
				   TEEC_ORIGIN_COMMS);
			pr_err("failed to get cmd_desc\n");
			goto error;
		}

		request = (struct tee_request *)cmd_desc->param_addr;
		memset(request, 0, sizeof(struct tee_request));

		tee_open_session(&cmd.opensession, request, context);

		memcpy(answer.params, cmd.opensession.operation.params,
		       sizeof(answer.params));

		SET_ANSWER(answer, request->result, request->result_origin);
		answer.session_id = request->session_id;
		break;

	case TEE_IOCTL_CLOSE_CLIENT_SESSION:
		ptr_user_answer = (void *)cmd.closesession.answer;
		cmd_desc = tee_get_free_cmd_desc(nv_dev);
		if (!cmd_desc) {
			SET_ANSWER(answer,
				   TEEC_ERROR_OUT_OF_MEMORY,
				   TEEC_ORIGIN_COMMS);
			pr_err("failed to get cmd_desc\n");
			goto error;
		}

		request = (struct tee_request *)cmd_desc->param_addr;
		memset(request, 0, sizeof(struct tee_request));

		/* close session cannot fail */
		tee_close_session(&cmd.closesession, request);
		break;

	case TEE_IOCTL_REGISTER_MEMORY:
		ptr_user_answer = (void *)cmd.sharedmem.answer;
		cmd_desc = tee_get_free_cmd_desc(nv_dev);
		if (!cmd_desc) {
			SET_ANSWER(answer,
				   TEEC_ERROR_OUT_OF_MEMORY,
				   TEEC_ORIGIN_COMMS);
			pr_err("failed to get cmd_desc\n");
			goto error;
		}

		request = (struct tee_request *)cmd_desc->param_addr;
		memset(request, 0, sizeof(request));

		tee_register_memory(&cmd.sharedmem, request, context);

		SET_ANSWER(answer, request->result, request->result_origin);

		break;

	case TEE_IOCTL_RELEASE_SHARED_MEM:
		tee_unregister_memory(cmd.release_shared_mem.buffer,
			context);
		break;

	case TEE_IOCTL_INVOKE_COMMAND:
		ptr_user_answer = (void *)cmd.invokecmd.answer;
		cmd_desc = tee_get_free_cmd_desc(nv_dev);

		if (!cmd_desc) {
			SET_ANSWER(answer,
				   TEEC_ERROR_OUT_OF_MEMORY,
				   TEEC_ORIGIN_COMMS);
			pr_err("failed to get cmd_desc\n");
			goto error;
		}

		request = (struct tee_request *)cmd_desc->param_addr;
		memset(request, 0, sizeof(struct tee_request));

		tee_invoke_command(&cmd.invokecmd, request, context);

		memcpy(answer.params, cmd.invokecmd.operation.params,
		       sizeof(answer.params));

		SET_ANSWER(answer, request->result, request->result_origin);
		break;

	default:
		pr_err("Invalid IOCTL Cmd\n");
		err = -EINVAL;
		goto error;
	}
	if (ptr_user_answer && !err)
		if (copy_to_user(ptr_user_answer, &answer,
			sizeof(struct tee_answer))) {
			pr_err("Failed to copy answer\n");
			err = -EFAULT;
		}

error:
	if (cmd_desc)
		tee_put_used_cmd_desc(nv_dev, cmd_desc);
	return err;
}

static long tee_device_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	int err;

	switch (ioctl_num) {
	case TEE_IOCTL_OPEN_CLIENT_SESSION:
	case TEE_IOCTL_CLOSE_CLIENT_SESSION:
	case TEE_IOCTL_REGISTER_MEMORY:
	case TEE_IOCTL_RELEASE_SHARED_MEM:
	case TEE_IOCTL_INVOKE_COMMAND:
		err = tee_handle_trustedapp_ioctl(file, ioctl_num, ioctl_param);
		break;

	case TEE_IOCTL_FILE_NEW_REQ:
	case TEE_IOCTL_FILE_FILL_BUF:
	case TEE_IOCTL_FILE_REQ_COMPLETE:
		err = tee_handle_fs_ioctl(file, ioctl_num, ioctl_param);
		break;

	default:
		pr_err("%s: Invalid IOCTL (0x%lx) 0x%lx, %d\n", __func__,
			ioctl_num, TEE_IOCTL_FILE_NEW_REQ,
			sizeof(TEEC_FileReq));
		err = -EINVAL;
	}

	return err;
}

/*
 * tee_driver function definitions.
 */
static const struct file_operations tegra_tee_device_fops = {
	.owner = THIS_MODULE,
	.open = tee_device_open,
	.release = tee_device_release,
	.unlocked_ioctl = tee_device_ioctl,
};

struct miscdevice tegra_tee_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tee_device",
	.fops = &tegra_tee_device_fops,
};

static int __init nv_tee_init(void)
{
	int ret;

	INIT_LIST_HEAD(&(tee_nv_dev.used_cmd_list));
	INIT_LIST_HEAD(&(tee_nv_dev.free_cmd_list));

	ret = tee_create_free_cmd_list(&tee_nv_dev);
	if (ret != 0)
		return ret;

	return misc_register(&tegra_tee_device);
}

module_init(nv_tee_init);
