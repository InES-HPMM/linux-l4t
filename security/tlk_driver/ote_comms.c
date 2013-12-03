/*
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
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

#include "ote_protocol.h"

bool verbose_smc;
core_param(verbose_smc, verbose_smc, bool, 0644);

#define SET_RESULT(req, r, ro)	{ req->result = r; req->result_origin = ro; }

static int te_pin_user_pages(void *buffer, size_t size,
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
	if (ret < 0)
		ret = get_user_pages(current, current->mm, (unsigned long)buffer,
				nr_pages, WRITE, 1/*force*/, pages, NULL);
	up_read(&current->mm->mmap_sem);

	*pages_ptr = (unsigned long) pages;

	return ret;
}

static struct te_shmem_desc *te_add_shmem_desc(void *buffer, size_t size,
		unsigned int nr_pages, struct page **pages,
		struct tlk_context *context)
{
	struct te_shmem_desc *shmem_desc = NULL;
	shmem_desc = kzalloc(sizeof(struct te_shmem_desc), GFP_KERNEL);
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

static int te_pin_mem_buffers(void *buffer, size_t size,
		struct tlk_context *context)
{

	unsigned long pages = 0;
	struct te_shmem_desc *shmem_desc = NULL;
	int ret = 0, nr_pages = 0;

	nr_pages = te_pin_user_pages(buffer, size, &pages);
	if (nr_pages <= 0) {
		pr_err("%s: te_pin_user_pages Failed (%d)\n", __func__,
			nr_pages);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	shmem_desc = te_add_shmem_desc(buffer, size,
				nr_pages, (struct page **)pages, context);
	if (!shmem_desc) {
		pr_err("%s: te_add_shmem_desc Failed\n", __func__);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	return OTE_SUCCESS;
error:
	return ret;
}

static int te_setup_temp_buffers(struct te_request *request,
		struct tlk_context *context)
{
	uint32_t i;
	int ret = OTE_SUCCESS;
	struct te_oper_param *params = request->params;

	for (i = 0; i < request->params_size; i++) {
		switch (params[i].type) {
		case TE_PARAM_TYPE_NONE:
		case TE_PARAM_TYPE_INT_RO:
		case TE_PARAM_TYPE_INT_RW:
			break;
		case TE_PARAM_TYPE_MEM_RO:
		case TE_PARAM_TYPE_MEM_RW:
			ret = te_pin_mem_buffers(
				params[i].u.Mem.base,
				params[i].u.Mem.len,
				context);
			if (ret < 0) {
				pr_err("%s failed with err (%d)\n",
					__func__, ret);
				ret = OTE_ERROR_BAD_PARAMETERS;
				break;
			}
			break;
		default:
			pr_err("%s: OTE_ERROR_BAD_PARAMETERS\n", __func__);
			ret = OTE_ERROR_BAD_PARAMETERS;
			break;
		}
	}
	return ret;
}

static void te_del_shmem_desc(void *buffer, struct tlk_context *context)
{
	struct te_shmem_desc *shmem_desc, *tmp_shmem_desc;
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
void te_unregister_memory(void *buffer,
	struct tlk_context *context)
{
	if (!(list_empty(&(context->shmem_alloc_list))))
		te_del_shmem_desc(buffer, context);
	else
		pr_err("No buffers to unpin\n");
}

static void te_unpin_temp_buffers(struct te_request *request,
	struct tlk_context *context)
{
	uint32_t i;
	struct te_oper_param *params = request->params;

	for (i = 0; i < request->params_size; i++) {
		switch (params[i].type) {
		case TE_PARAM_TYPE_NONE:
		case TE_PARAM_TYPE_INT_RO:
		case TE_PARAM_TYPE_INT_RW:
			break;
		case TE_PARAM_TYPE_MEM_RO:
		case TE_PARAM_TYPE_MEM_RW:
			te_unregister_memory(params[i].u.Mem.base, context);
			break;
		default:
			pr_err("%s: OTE_ERROR_BAD_PARAMETERS\n", __func__);
			break;
		}
	}
}

#ifdef CONFIG_SMP
cpumask_t saved_cpu_mask;
void switch_cpumask_to_cpu0(void)
{
	long ret;
	cpumask_t local_cpu_mask = CPU_MASK_NONE;

	cpu_set(0, local_cpu_mask);
	cpumask_copy(&saved_cpu_mask, tsk_cpus_allowed(current));
	ret = sched_setaffinity(0, &local_cpu_mask);
	if (ret)
		pr_err("sched_setaffinity #1 -> 0x%lX", ret);
}

void restore_cpumask(void)
{
	long ret = sched_setaffinity(0, &saved_cpu_mask);
	if (ret)
		pr_err("sched_setaffinity #2 -> 0x%lX", ret);
}
#endif

uint32_t tlk_generic_smc(uint32_t arg0, uint32_t arg1, uint32_t arg2)
{
	register uint32_t r0 asm("r0") = arg0;
	register uint32_t r1 asm("r1") = arg1;
	register uint32_t r2 asm("r2") = arg2;

	asm volatile(
		__asmeq("%0", "r0")
		__asmeq("%1", "r0")
		__asmeq("%2", "r1")
		__asmeq("%3", "r2")
#ifdef REQUIRES_SEC
		".arch_extension sec				\n"
#endif
		"smc	#0		@ switch to secure world\n"
		: "=r" (r0)
		: "r" (r0), "r" (r1), "r" (r2)
	);

	return r0;
}

uint32_t tlk_extended_smc(uint32_t *regs)
{
	register uint32_t r0 asm("r0") = (uint32_t)regs;

	/* allows MAX_EXT_SMC_ARGS (r0-r11) to be passed in registers */
	asm volatile(
		__asmeq("%0", "r0")
		"stmfd	sp!, {r4-r12}	@ save reg state\n"
		"mov	r12, r0		@ reg ptr to r12\n"
		"ldmia	r12, {r0-r11}	@ load arg regs\n"
#ifdef REQUIRES_SEC
		".arch_extension sec\n"
#endif
		"smc	#0		@ switch to secure world\n"
		"ldmfd	sp!, {r4-r12}	@ restore saved regs\n"
		: "=r" (r0)
		: "r" (r0)
	);

	return r0;
}

/*
 * Do an SMC call
 */
static void do_smc(struct te_request *request, struct tlk_device *dev)
{
	uint32_t smc_args;
	uint32_t smc_params = 0;

	if (dev->req_param_buf) {
		smc_args = (char *)request - dev->req_param_buf;
		if (request->params)
			smc_params = (char *)request->params -
						dev->req_param_buf;
	} else {
		smc_args = (uint32_t)virt_to_phys(request);
		if (request->params)
			smc_params = (uint32_t)virt_to_phys(request->params);
	}

	TLK_GENERIC_SMC(request->type, smc_args, smc_params);

	/*
	 * Check to see if there are any logs in written by TLK.
	 * If there are, print them out.
	 */
	ote_print_logs();
}

/*
 * VPR programming SMC
 */
int te_set_vpr_params(void *vpr_base, size_t vpr_size)
{
	uint32_t retval;

	/* Share the same lock used when request is send from user side */
	mutex_lock(&smc_lock);

	retval = TLK_GENERIC_SMC(TE_SMC_PROGRAM_VPR, (uint32_t)vpr_base, vpr_size);

	mutex_unlock(&smc_lock);

	if (retval != OTE_SUCCESS) {
		pr_err("te_set_vpr_params failed err (0x%x)\n", retval);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(te_set_vpr_params);

/*
 * Open session SMC (supporting client-based te_open_session() calls)
 */
void te_open_session(struct te_opensession *cmd,
		     struct te_request *request,
		     struct tlk_context *context)
{
	int ret;

	ret = te_setup_temp_buffers(request, context);
	if (ret != OTE_SUCCESS) {
		pr_err("te_setup_temp_buffers failed err (0x%x)\n", ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		return;
	}

	memcpy(&request->dest_uuid,
	       &cmd->dest_uuid,
	       sizeof(struct te_service_id));

	pr_info("OPEN_CLIENT_SESSION: 0x%x 0x%x 0x%x 0x%x\n",
		request->dest_uuid[0],
		request->dest_uuid[1],
		request->dest_uuid[2],
		request->dest_uuid[3]);

	request->type = TE_SMC_OPEN_SESSION;

	do_smc(request, context->dev);

	te_unpin_temp_buffers(request, context);
}

/*
 * Close session SMC (supporting client-based te_close_session() calls)
 */
void te_close_session(struct te_closesession *cmd,
		      struct te_request *request,
		      struct tlk_context *context)
{
	request->session_id = cmd->session_id;
	request->type = TE_SMC_CLOSE_SESSION;

	do_smc(request, context->dev);
	if (request->result)
		pr_info("Error closing session: %08x\n", request->result);
}

/*
 * Launch operation SMC (supporting client-based te_launch_operation() calls)
 */
void te_launch_operation(struct te_launchop *cmd,
			 struct te_request *request,
			 struct tlk_context *context)
{
	int ret;

	ret = te_setup_temp_buffers(request, context);
	if (ret != OTE_SUCCESS) {
		pr_err("te_setup_temp_buffers failed err (0x%x)\n", ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		return;
	}

	request->session_id = cmd->session_id;
	request->command_id = cmd->operation.command;
	request->type = TE_SMC_LAUNCH_OPERATION;

	do_smc(request, context->dev);

	te_unpin_temp_buffers(request, context);
}

static int __init tlk_register_irq_handler(void)
{
	TLK_GENERIC_SMC(TE_SMC_REGISTER_IRQ_HANDLER,
		(unsigned int)tlk_irq_handler, 0);
	return 0;
}

arch_initcall(tlk_register_irq_handler);
