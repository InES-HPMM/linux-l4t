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
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagemap.h>

#include "tee_types.h"
#include "tee_protocol.h"

bool verbose_smc;
core_param(verbose_smc, verbose_smc, bool, 0644);

#define SET_RESULT(req, r, ro)	{ req->result = r; req->result_origin = ro; }

#define TEE_PARAM_COUNT	4

static int tee_device_set_request_params(struct tee_request *request,
	struct TEEC_Operation *operation)
{
	struct tee_cmd_param *param = &request->cmd_param;
	uint32_t i, type;

	param->param_types = operation->paramTypes;
	for (i = 0; i < TEE_PARAM_COUNT; i++) {
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
		default:
			return TEEC_ERROR_BAD_PARAMETERS;
		}
	}
	return TEEC_SUCCESS;
}

static int tee_device_get_answer_params(struct TEEC_Operation *operation,
	struct tee_request *request)
{
	struct tee_cmd_param *param = &request->cmd_param;
	uint32_t i, type;

	param->param_types = operation->paramTypes;
	for (i = 0; i < TEE_PARAM_COUNT; i++) {
		type = TEEC_PARAM_TYPE_GET(operation->paramTypes, i);
		switch (type) {
		case TEEC_PARAM_TYPE_NONE:
			break;
		case TEEC_PARAM_TYPE_VALUE_INPUT:
		case TEEC_PARAM_TYPE_VALUE_OUTPUT:
		case TEEC_PARAM_TYPE_VALUE_INOUT:
			memcpy(&operation->params[i].value,
				&param->params[i].value,
				sizeof(union tee_param));
		break;
		case TEEC_PARAM_TYPE_MEMREF_INPUT:
		case TEEC_PARAM_TYPE_MEMREF_OUTPUT:
		case TEEC_PARAM_TYPE_MEMREF_INOUT:
			memcpy(&operation->params[i].tmpref,
			       &param->params[i].memref,
			       sizeof(union tee_param));
		break;
		default:
			return TEEC_ERROR_BAD_PARAMETERS;
		}
	}
	return TEEC_SUCCESS;
}

static int tee_pin_user_pages(void *buffer, size_t size,
	unsigned long *pages_ptr)
{
	int ret = 0;
	unsigned int nr_pages;
	struct page **pages = NULL;

	nr_pages = (((unsigned int)buffer & (PAGE_SIZE - 1)) +
			(size + PAGE_SIZE - 1)) >> PAGE_SHIFT;

	pages = kzalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	down_read(&current->mm->mmap_sem);
	ret = get_user_pages(current, current->mm, (unsigned long)buffer,
				nr_pages, WRITE, 0, pages, NULL);
	up_read(&current->mm->mmap_sem);

	*pages_ptr = (unsigned long) pages;

	return ret;
}

static struct nv_shmem_desc *tee_add_shmem_desc(void *buffer, size_t size,
			unsigned int nr_pages, struct page **pages,
			struct nv_tee_context *context)
{
	struct nv_shmem_desc *shmem_desc = NULL;
	shmem_desc = kzalloc(
						sizeof(struct nv_shmem_desc),
						GFP_KERNEL);
	if (shmem_desc) {
		INIT_LIST_HEAD(&(shmem_desc->list));
		shmem_desc->buffer = buffer;
		shmem_desc->size = size;
		shmem_desc->nr_pages = nr_pages;
		shmem_desc->pages = pages;
		list_add_tail(&shmem_desc->list, &(context->shmem_alloc_list));
	}

	return shmem_desc;
}

static int tee_pin_mem_buffers(void *buffer, size_t size,
	struct nv_tee_context *context)
{

	unsigned long pages = 0;
	struct nv_shmem_desc *shmem_desc = NULL;
	int ret = 0, nr_pages = 0;

	nr_pages = tee_pin_user_pages(buffer, size, &pages);
	if (nr_pages <= 0) {
		pr_err("tee_pin_mem_buffers: tee_pin_user_pages Failed\n");
		ret = TEEC_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	shmem_desc = tee_add_shmem_desc(buffer, size,
				nr_pages, (struct page **)pages, context);
	if (!shmem_desc) {
		pr_err("tee_pin_mem_buffers: tee_add_shmem_desc Failed\n");
		ret = TEEC_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	return TEEC_SUCCESS;
error:
	return ret;
}

static int tee_setup_temp_buffers(struct TEEC_Operation *oper,
	struct nv_tee_context *context)
{
	uint32_t i, type;
	int ret = TEEC_SUCCESS;

	for (i = 0; i < TEE_PARAM_COUNT; i++) {
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
				pr_err("tee_pin_mem_buffers failed with err (%d)\n",
						ret);
				ret = TEEC_ERROR_BAD_PARAMETERS;
				break;
			}
			break;
		default:
			pr_err("tee_pin_mem_buffers: TEEC_ERROR_BAD_PARAMETERS\n");
			ret = TEEC_ERROR_BAD_PARAMETERS;
			break;
		}
	}
	return ret;
}

static void tee_del_shmem_desc(void *buffer, struct nv_tee_context *context)
{
	struct nv_shmem_desc *shmem_desc, *tmp_shmem_desc;
	int i;

	list_for_each_entry_safe(shmem_desc, tmp_shmem_desc,
		&(context->shmem_alloc_list), list) {
		if (shmem_desc->buffer == buffer) {
			list_del(&shmem_desc->list);
			for (i = 0; i < shmem_desc->nr_pages; i++)
				page_cache_release(shmem_desc->pages[i]);
			kfree(shmem_desc->pages);
			kfree(shmem_desc);
		}
	}
}

/*
 * Deregister previously initialized shared memory
 */
void tee_unregister_memory(void *buffer,
	struct nv_tee_context *context)
{
	if (!(list_empty(&(context->shmem_alloc_list))))
		tee_del_shmem_desc(buffer, context);
	else
		pr_err("No buffers to unpin\n");
}

static void tee_unpin_temp_buffers(struct TEEC_Operation *oper,
	struct nv_tee_context *context)
{
	uint32_t i, type;

	for (i = 0; i < TEE_PARAM_COUNT; i++) {
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
			pr_err("tee_unpin_mem_buffers: TEEC_ERROR_BAD_PARAMETERS\n");
			break;
		}
	}
}

/*
 * Do an SMC call
 */
static void do_smc(struct tee_request *request)
{
	phys_addr_t smc_args = virt_to_phys(request);

#ifdef CONFIG_SMP
	long ret;
	cpumask_t saved_cpu_mask;
	cpumask_t local_cpu_mask = CPU_MASK_NONE;

	cpu_set(0, local_cpu_mask);
	sched_getaffinity(0, &saved_cpu_mask);
	ret = sched_setaffinity(0, &local_cpu_mask);
	if (ret != 0)
		pr_err("sched_setaffinity #1 -> 0x%lX", ret);
#endif

	asm volatile (
		"stmdb	sp!, {r4-r12}\n"
		"mov    r0,  %0\n"
		"mov    r1,  %1\n"
#ifdef REQUIRES_SEC
		".arch_extension sec\n"
#endif
		"smc	#0\n"
		"ldmia	sp!, {r4-r12}\n"
		: : "r" (request->type), "r" (smc_args)
		: "r0", "r1"
	);

#ifdef CONFIG_SMP
	ret = sched_setaffinity(0, &saved_cpu_mask);
	if (ret != 0)
		pr_err("sched_setaffinity #2 -> 0x%lX", ret);
#endif
}

/*
 * Do an 'empty' request just to get more pending answers.
 */
static void get_more_answers(struct tee_request *request)
{
	request->type = TMK_SMC_GET_MORE;
	/* rest of request ignored */
	do_smc(request);
}

/*
 * Open session SMC (TEEC_OpenSession)
 */
void tee_open_session(struct tee_opensession *cmd,
		      struct tee_request *request,
		      struct nv_tee_context *context)
{
	int ret;

	ret = tee_device_set_request_params(request, &cmd->operation);
	if (ret != TEEC_SUCCESS) {
		pr_err("tee_device_set_request_params failed\n");
		SET_RESULT(request, ret, TEEC_ORIGIN_API);
		return;
	}

	ret = tee_setup_temp_buffers(&cmd->operation, context);
	if (ret != TEEC_SUCCESS) {
		pr_err("tee_setup_temp_buffers failed err (0x%x)\n", ret);
		SET_RESULT(request, ret, TEEC_ORIGIN_API);
		return;
	}

	memcpy(&request->cmd_param.dest_uuid,
	       &cmd->dest_uuid,
	       sizeof(struct TEEC_UUID));

	pr_info("OPEN_CLIENT_SESSION: 0x%x 0x%x 0x%x 0x%x\n",
		request->cmd_param.dest_uuid[0],
		request->cmd_param.dest_uuid[1],
		request->cmd_param.dest_uuid[2],
		request->cmd_param.dest_uuid[3]);

	request->type = TMK_SMC_OPEN_SESSION;

	do_smc(request);

	tee_device_get_answer_params(&cmd->operation, request);

	tee_unpin_temp_buffers(&cmd->operation, context);
}

/*
 * Close session SMC (TEEC_CloseSession)
 */
void tee_close_session(struct tee_closesession *cmd,
		       struct tee_request *request)
{
	request->type = TMK_SMC_CLOSE_SESSION;
	request->session_id = cmd->session_id;

	do_smc(request);
	if (request->result)
		pr_info("Error closing session: %08x\n", request->result);
}

/*
 * Register Shared Memory SMC (TEEC_RegisterSharedMemory)
 */
void tee_register_memory(struct tee_sharedmem *cmd, struct tee_request *request,
			struct nv_tee_context *context)
{
	int ret = 0;

	request->type = TMK_SMC_REG_SHARED_MEM;
	request->session_id = cmd->session_id;

	request->cmd_param.param_types = cmd->memref.flags;
	request->cmd_param.params[0].memref.buffer = cmd->memref.buffer;
	request->cmd_param.params[0].memref.size = cmd->memref.size;

	ret = tee_pin_mem_buffers(cmd->memref.buffer, cmd->memref.size,
		context);
	if (ret != TEEC_SUCCESS) {
		SET_RESULT(request, ret, TEEC_ORIGIN_API);
		return;
	}

	do_smc(request);
}

/*
 * Invoke Command SMC (TEEC_InvokeCommand)
 */
void tee_invoke_command(struct tee_invokecmd *cmd,
			struct tee_request *request,
			struct nv_tee_context *context)
{
	int ret = TEEC_SUCCESS;

	ret = tee_device_set_request_params(request, &cmd->operation);
	if (ret != TEEC_SUCCESS) {
		pr_err("tee_device_set_request_params failed\n");
		SET_RESULT(request, ret, TEEC_ORIGIN_API);
		return;
	}

	ret = tee_setup_temp_buffers(&cmd->operation, context);
	if (ret != TEEC_SUCCESS) {
		pr_err("tee_setup_temp_buffers failed err (0x%x)\n", ret);
		SET_RESULT(request, ret, TEEC_ORIGIN_API);
		return;
	}

	request->type = TMK_SMC_INVOKE_CMD;
	request->session_id = cmd->session_id;
	request->command_id = cmd->command_id;

	do_smc(request);

	tee_device_get_answer_params(&cmd->operation, request);

	tee_unpin_temp_buffers(&cmd->operation, context);
}

static int __init nv_tee_register_irq_handler(void)
{
	asm volatile (
		"mov	r1, %0\n"
		"movw	r0, #0x1FF0\n"
		"movt	r0, #0xFFFF\n"
#ifdef REQUIRES_SEC
		".arch_extension sec\n"
#endif
		"smc	#0\n"
		"cpsie	i\n"
		: : "r" (nv_tee_irq_handler)
		: "r0", "r1", "r13", "r14"
	);

	return 0;
}

arch_initcall(nv_tee_register_irq_handler);
