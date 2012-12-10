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

#define CREATE_TRACE_POINTS
#include <trace/events/nvsecurity.h>

u32 notrace tegra_read_cycle(void)
{
	u32 cycle_count;

	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(cycle_count));

	return cycle_count;
}

struct nv_device tee_nv_dev;

static int tee_device_setup_operation_params(struct tee_cmd_param *param,
	struct TEEC_Operation *operation)
{
	uint32_t i, type;
	int ret = TEEC_SUCCESS;

	param->param_types = operation->paramTypes;
	for (i = 0; i < 4; i++) {
		type = TEEC_PARAM_TYPE_GET(operation->paramTypes, i);
		switch (type) {
		case TEEC_PARAM_TYPE_NONE:
			break;
		case TEEC_PARAM_TYPE_VALUE_INPUT:
		case TEEC_PARAM_TYPE_VALUE_OUTPUT:
		case TEEC_PARAM_TYPE_VALUE_INOUT:
			memcpy(&param->params[i].value,
				&operation->params[i].value,
				sizeof(union tee_param));
		break;
		case TEEC_PARAM_TYPE_MEMREF_INPUT:
		case TEEC_PARAM_TYPE_MEMREF_OUTPUT:
		case TEEC_PARAM_TYPE_MEMREF_INOUT:
			memcpy(&param->params[i].memref,
				&operation->params[i].tmpref,
				sizeof(union tee_param));
		break;
#if 0
		/* XXX support for these requires some work */
		case TEEC_PARAM_TYPE_MEMREF_WHOLE:
		case TEEC_PARAM_TYPE_MEMREF_PARTIAL_INPUT:
		case TEEC_PARAM_TYPE_MEMREF_PARTIAL_OUTPUT:
		case TEEC_PARAM_TYPE_MEMREF_PARTIAL_INOUT:
		break;
#endif
		default:
			return TEEC_ERROR_BAD_PARAMETERS;
		}
	}
	return 0;
}

static int tee_setup_temp_buffers(struct TEEC_Operation *oper,
	struct nv_tee_context *context)
{
	uint32_t i, type;
	int ret = TEEC_SUCCESS;

	for (i = 0; i < 4; i++) {
		type = TEEC_PARAM_TYPE_GET(oper->paramTypes, i);
		switch (type) {
		case TEEC_PARAM_TYPE_NONE:
		case TEEC_PARAM_TYPE_VALUE_INPUT:
		case TEEC_PARAM_TYPE_VALUE_OUTPUT:
		case TEEC_PARAM_TYPE_VALUE_INOUT:
			break;
		case TEEC_PARAM_TYPE_MEMREF_INPUT:
		case TEEC_PARAM_TYPE_MEMREF_OUTPUT:
		case TEEC_PARAM_TYPE_MEMREF_INOUT:
			ret = tee_pin_mem_buffers(
				oper->params[i].tmpref.buffer,
				oper->params[i].tmpref.size,
				context);
			if (ret < 0) {
				pr_err("Pin buffer failed err (%d)\n", ret);
				break;
			}
			break;
		default:
			pr_err("Pin buffer TEEC_ERROR_BAD_PARAMETERS\n");
			ret = TEEC_ERROR_BAD_PARAMETERS;
		}
	}
	return ret;
}

static int tee_unpin_temp_buffers(struct TEEC_Operation *oper,
	struct nv_tee_context *context)
{
	uint32_t i, type;
	int ret = TEEC_SUCCESS;

	for (i = 0; i < 4; i++) {
		type = TEEC_PARAM_TYPE_GET(oper->paramTypes, i);
		switch (type) {
		case TEEC_PARAM_TYPE_NONE:
		case TEEC_PARAM_TYPE_VALUE_INPUT:
		case TEEC_PARAM_TYPE_VALUE_OUTPUT:
		case TEEC_PARAM_TYPE_VALUE_INOUT:
			break;
		case TEEC_PARAM_TYPE_MEMREF_INPUT:
		case TEEC_PARAM_TYPE_MEMREF_OUTPUT:
		case TEEC_PARAM_TYPE_MEMREF_INOUT:
			tee_unregister_memory(oper->params[i].tmpref.buffer,
				context);
			break;
		default:
			pr_err("tee_pin_mem_buffers: TEEC_ERROR_BAD_PARAMETERS\n");
			ret = TEEC_ERROR_BAD_PARAMETERS;
		}
	}
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

static void tee_print_opensession_cmd(struct tee_opensession *opensession)
{
	int i;
	pr_info("UUID time low (%x)\n", opensession->dest_uuid.time_low);
	pr_info("UUID time low (%x)\n", opensession->dest_uuid.time_mid);
	pr_info("UUID time_hi_and_version (%x)\n",
		opensession->dest_uuid.time_hi_and_version);
	pr_info("login type (%d)\n", opensession->login_types);
	if (opensession->login_data)
		pr_info("login data (%d)\n", opensession->login_data);
	for (i = 0; i < 4; i++)
			pr_info("Param#(%d) : param_type (%d)\n", i,
				TEEC_PARAM_TYPE_GET(
					(opensession->operation.paramTypes),
					i));
}

static int tee_device_open(struct inode *inode, struct file *file)
{
	struct nv_tee_context *context;
	int ret = 0;

	pr_info("tee_device_open\n");
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
	pr_info("tee_device_release\n");
	kfree(file->private_data);
	file->private_data = NULL;
	return 0;
}

static long tee_device_ioctl(struct file *file, unsigned int ioctl_num,
	unsigned long ioctl_param)
{
	int err = TEEC_SUCCESS;
	union tee_cmd command;
	void *ptr_user_answer = NULL;
	struct tee_answer answer;
	struct tee_cmd_param *param;
	struct nv_cmd_param_desc *cmd_desc = NULL;

	struct nv_tee_context *context = file->private_data;
	struct nv_device *nv_dev = context->dev;

	if (copy_from_user(&command, (void __user *)ioctl_param,
				sizeof(union tee_cmd))) {
		pr_err("Failed to copy command request\n");
		goto error;
	}

	memset(&answer, 0, sizeof(struct tee_answer));
	switch (ioctl_num) {
	case TEE_IOCTL_OPEN_CLIENT_SESSION:
		ptr_user_answer = (void *)command.opensession.answer;

		cmd_desc = tee_get_free_cmd_desc(nv_dev);
		if (cmd_desc) {
			param = (struct tee_cmd_param *)
				cmd_desc->param_addr;

			err = tee_device_setup_operation_params(param,
				&command.opensession.operation);
			if (err != TEEC_SUCCESS) {
				pr_err("setup operation params failed\n");
				goto error;
			}

			err = tee_setup_temp_buffers(
				&command.opensession.operation, context);
			if (err != TEEC_SUCCESS) {
				pr_err("setup temp buf failed err (%d)\n", err);
				goto error;
			}

			memcpy(&param->dest_uuid,
				&command.opensession.dest_uuid,
				sizeof(struct TEEC_UUID));

			err = tee_open_session(&command.opensession,
				virt_to_phys((void *)cmd_desc->param_addr),
				&answer);
			if (err)
				pr_err("open session failed with err (%d)\n",
						err);
			err = tee_unpin_temp_buffers(
				&command.opensession.operation,	context);
		} else {
			pr_err("failed to get cmd_desc\n");
			goto error;
		}
		break;

	case TEE_IOCTL_CLOSE_CLIENT_SESSION:
		ptr_user_answer = (void *)command.closesession.answer;
		err = tee_close_session(command.closesession.session_id);
		if (err)
			pr_err("close session failed with error %d\n", err);
		break;

	case TEE_IOCTL_REGISTER_MEMORY:
		ptr_user_answer = (void *)command.sharedmem.answer;
		cmd_desc = tee_get_free_cmd_desc(nv_dev);
		if (cmd_desc) {
			param = (struct tee_cmd_param *)
				 cmd_desc->param_addr;
			param->param_types = command.sharedmem.memref.flags;
			param->params[0].memref.buffer =
				command.sharedmem.memref.buffer;
			param->params[0].memref.size =
				command.sharedmem.memref.size;
		}

		err = tee_register_memory(&(command.sharedmem),
			virt_to_phys((void *)cmd_desc->param_addr),
			&answer,
			context);
		if (err)
			pr_err("memory registor failed with error %d\n", err);
		break;

	case TEE_IOCTL_RELEASE_SHARED_MEM:
		tee_unregister_memory(command.release_shared_mem.buffer,
			context);
		break;

	case TEE_IOCTL_INVOKE_COMMAND:
		ptr_user_answer = (void *)command.invokecmd.answer;
		cmd_desc = tee_get_free_cmd_desc(nv_dev);
		if (cmd_desc) {
			param = (struct tee_cmd_param *)
				cmd_desc->param_addr;
			err = tee_device_setup_operation_params(param,
				&command.invokecmd.operation);
			if (err != TEEC_SUCCESS) {
				pr_err("tee_device_setup_operation_params failed\n");
				goto error;
			}

			err = tee_setup_temp_buffers(
				&command.invokecmd.operation, context);
			if (err != TEEC_SUCCESS) {
				pr_err("setup temp buffers failed err (%d)\n",
						err);
				goto error;
			}

			err = tee_invoke_command(&(command.invokecmd),
				virt_to_phys((void *)cmd_desc->param_addr),
				 &answer);
			if (err) {
				pr_err("Invoke cmd id (%u) failed err (%d)\n",
					command.invokecmd.command_id, err);
			}
			err = tee_unpin_temp_buffers(
				&command.invokecmd.operation, context);
		} else {
			pr_err("failed to get cmd_desc\n");
		}
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
			sizeof(struct nv_cmd_param_desc) * cmd_desc_count;
		INIT_LIST_HEAD(&(param_desc->list));

		/*Add the cmd param descriptor to free list*/
		list_add_tail(&param_desc->list, &(dev->free_cmd_list));
	}
error:
	return ret;

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
