/*
 * Copyright (c) 2012-2014 NVIDIA Corporation. All rights reserved.
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
#include <asm/smp_plat.h>

#include "ote_protocol.h"

bool verbose_smc;
core_param(verbose_smc, verbose_smc, bool, 0644);

#define SET_RESULT(req, r, ro)	{ req->result = r; req->result_origin = ro; }

static int te_pin_user_pages(void *buffer, size_t size,
		unsigned long *pages_ptr, uint32_t buf_type)
{
	int ret = 0;
	unsigned int nr_pages;
	struct page **pages = NULL;
	bool writable;

	nr_pages = (((uintptr_t)buffer & (PAGE_SIZE - 1)) +
			(size + PAGE_SIZE - 1)) >> PAGE_SHIFT;

	pages = kzalloc(nr_pages * sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	writable = (buf_type == TE_PARAM_TYPE_MEM_RW ||
		buf_type == TE_PARAM_TYPE_PERSIST_MEM_RW);

	down_read(&current->mm->mmap_sem);
	ret = get_user_pages(current, current->mm, (unsigned long)buffer,
			nr_pages, writable,
			0, pages, NULL);

	up_read(&current->mm->mmap_sem);

	*pages_ptr = (unsigned long) pages;

	return ret;
}

static int te_prep_mem_buffer(uint32_t session_id,
		void *buffer, size_t size, uint32_t buf_type,
		struct tlk_context *context)
{
	unsigned long pages = 0;
	struct te_shmem_desc *shmem_desc = NULL;
	int ret = 0, nr_pages = 0;

	/* allocate new shmem descriptor */
	shmem_desc = kzalloc(sizeof(struct te_shmem_desc), GFP_KERNEL);
	if (!shmem_desc) {
		pr_err("%s: te_add_shmem_desc failed\n", __func__);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	/* pin pages */
	nr_pages = te_pin_user_pages(buffer, size, &pages, buf_type);
	if (nr_pages <= 0) {
		pr_err("%s: te_pin_user_pages failed (%d)\n", __func__,
			nr_pages);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		kfree(shmem_desc);
		goto error;
	}

	/* initialize shmem descriptor */
	INIT_LIST_HEAD(&(shmem_desc->list));
	shmem_desc->active = false;
	shmem_desc->buffer = buffer;
	shmem_desc->size = size;
	shmem_desc->nr_pages = nr_pages;
	shmem_desc->pages = (struct page **)(uintptr_t)pages;

	/* add shmem descriptor to proper list */
	if ((buf_type == TE_PARAM_TYPE_MEM_RO) ||
		(buf_type == TE_PARAM_TYPE_MEM_RW))
		list_add_tail(&shmem_desc->list, &context->temp_shmem_list);
	else {
		list_add_tail(&shmem_desc->list, &context->persist_shmem_list);
	}

	return OTE_SUCCESS;
error:
	return ret;
}

static int te_prep_mem_buffers(struct te_request *request,
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
		case TE_PARAM_TYPE_PERSIST_MEM_RO:
		case TE_PARAM_TYPE_PERSIST_MEM_RW:
			ret = te_prep_mem_buffer(request->session_id,
				params[i].u.Mem.base,
				params[i].u.Mem.len,
				params[i].type,
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

static int te_prep_mem_buffers_compat(struct te_request_compat *request,
			struct tlk_context *context)
{
	uint32_t i;
	int ret = OTE_SUCCESS;
	struct te_oper_param_compat *params;

	params = (struct te_oper_param_compat *)(uintptr_t)request->params;
	for (i = 0; i < request->params_size; i++) {
		switch (params[i].type) {
		case TE_PARAM_TYPE_NONE:
		case TE_PARAM_TYPE_INT_RO:
		case TE_PARAM_TYPE_INT_RW:
			break;
		case TE_PARAM_TYPE_MEM_RO:
		case TE_PARAM_TYPE_MEM_RW:
		case TE_PARAM_TYPE_PERSIST_MEM_RO:
		case TE_PARAM_TYPE_PERSIST_MEM_RW:
			ret = te_prep_mem_buffer(request->session_id,
				(void *)(uintptr_t)params[i].u.Mem.base,
				params[i].u.Mem.len,
				params[i].type,
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

static void te_release_mem_buffer(struct te_shmem_desc *shmem_desc)
{
	uint32_t i;

	list_del(&shmem_desc->list);
	for (i = 0; i < shmem_desc->nr_pages; i++) {
		if ((shmem_desc->type == TE_PARAM_TYPE_MEM_RW) ||
			(shmem_desc->type == TE_PARAM_TYPE_PERSIST_MEM_RW))
			set_page_dirty_lock(shmem_desc->pages[i]);
		page_cache_release(shmem_desc->pages[i]);
	}
	kfree(shmem_desc->pages);
	kfree(shmem_desc);
}

static void te_release_temp_mem_buffers(struct tlk_context *context)
{
	struct te_shmem_desc *shmem_desc, *tmp_shmem_desc;

	if (list_empty(&context->temp_shmem_list))
		return;

	list_for_each_entry_safe(shmem_desc, tmp_shmem_desc,
		&context->temp_shmem_list, list) {
		te_release_mem_buffer(shmem_desc);
	}
}

static void te_release_persist_mem_buffers(uint32_t session_id,
	struct tlk_context *context)
{
	struct te_shmem_desc *shmem_desc, *tmp_shmem_desc;

	if (list_empty(&context->persist_shmem_list))
		return;

	/*
	 * Release any persistent mem buffers that either belong to
	 * the specified session_id or are not currently marked active
	 * (i.e. because the associated open_session or launch_operation
	 * failed).
	 */
	list_for_each_entry_safe(shmem_desc, tmp_shmem_desc,
		&context->persist_shmem_list, list) {
		if ((shmem_desc->session_id == session_id) ||
			(!shmem_desc->active))
			te_release_mem_buffer(shmem_desc);
	}
}

static void te_update_persist_mem_buffers(uint32_t session_id,
	struct tlk_context *context)
{
	struct te_shmem_desc *shmem_desc, *tmp_shmem_desc;

	/*
	 * Assumes any entries that have yet to be marked active belong
	 * to the session associated with the session_id that has been
	 * passed in.
	 */
	list_for_each_entry_safe(shmem_desc, tmp_shmem_desc,
		&context->persist_shmem_list, list) {

		if (!shmem_desc->active) {
			shmem_desc->session_id = session_id;
			shmem_desc->active = true;
		}
	}
}

#ifdef CONFIG_SMP
cpumask_t saved_cpu_mask;
static void switch_cpumask_to_cpu0(void)
{
	long ret;
	cpumask_t local_cpu_mask = CPU_MASK_NONE;

	cpu_set(0, local_cpu_mask);
	cpumask_copy(&saved_cpu_mask, tsk_cpus_allowed(current));
	ret = sched_setaffinity(0, &local_cpu_mask);
	if (ret)
		pr_err("%s: sched_setaffinity #1 -> 0x%lX", __func__, ret);
}

static void restore_cpumask(void)
{
	long ret = sched_setaffinity(0, &saved_cpu_mask);
	if (ret)
		pr_err("%s: sched_setaffinity #2 -> 0x%lX", __func__, ret);
}
#else
static inline void switch_cpumask_to_cpu0(void) {};
static inline void restore_cpumask(void) {};
#endif

uint32_t tlk_generic_smc(uint32_t arg0, uintptr_t arg1, uintptr_t arg2)
{
	uint32_t retval;

	switch_cpumask_to_cpu0();

	retval = _tlk_generic_smc(arg0, arg1, arg2);
	while (retval == TE_ERROR_PREEMPT_BY_IRQ ||
	       retval == TE_ERROR_PREEMPT_BY_FS) {
		if (retval == TE_ERROR_PREEMPT_BY_FS)
			tlk_ss_op();
		retval = _tlk_generic_smc(TE_SMC_RESTART, 0, 0);
	}

	restore_cpumask();

	/* Print TLK logs if any */
	ote_print_logs();

	return retval;
}

uint32_t tlk_extended_smc(uintptr_t *regs)
{
	uint32_t retval;

	switch_cpumask_to_cpu0();

	retval = _tlk_extended_smc(regs);
	while (retval == TE_ERROR_PREEMPT_BY_IRQ)
		retval = _tlk_generic_smc(TE_SMC_RESTART, 0, 0);

	restore_cpumask();

	/* Print TLK logs if any */
	ote_print_logs();

	return retval;
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

	tlk_generic_smc(request->type, smc_args, smc_params);
}

/*
 * Do an SMC call
 */
static void do_smc_compat(struct te_request_compat *request,
			  struct tlk_device *dev)
{
	uint32_t smc_args;
	uint32_t smc_params = 0;

	smc_args = (char *)request - dev->req_param_buf;
	if (request->params) {
		smc_params =
			(char *)(uintptr_t)request->params - dev->req_param_buf;
	}

	tlk_generic_smc(request->type, smc_args, smc_params);
}

struct tlk_smc_work_args {
	uint32_t arg0;
	uintptr_t arg1;
	uint32_t arg2;
};

static long tlk_generic_smc_on_cpu0(void *args)
{
	struct tlk_smc_work_args *work;
	int cpu = cpu_logical_map(smp_processor_id());
	uint32_t retval;

	BUG_ON(cpu != 0);

	work = (struct tlk_smc_work_args *)args;
	retval = _tlk_generic_smc(work->arg0, work->arg1, work->arg2);
	while (retval == TE_ERROR_PREEMPT_BY_IRQ)
		retval = _tlk_generic_smc(TE_SMC_RESTART, 0, 0);
	return retval;
}

/*
 * VPR programming SMC
 *
 * This routine is called both from normal threads and worker threads.
 * The worker threads are per-cpu and have PF_NO_SETAFFINITY set, so
 * any calls to sched_setaffinity will fail.
 *
 * If it's a worker thread on CPU0, just invoke the SMC directly. If
 * it's running on a non-CPU0, use work_on_cpu() to schedule the SMC
 * on CPU0.
 */
int te_set_vpr_params(void *vpr_base, size_t vpr_size)
{
	uint32_t retval;

	/* Share the same lock used when request is send from user side */
	mutex_lock(&smc_lock);

	if (current->flags &
	    (PF_WQ_WORKER | PF_NO_SETAFFINITY | PF_KTHREAD)) {
		struct tlk_smc_work_args work_args;
		int cpu = cpu_logical_map(smp_processor_id());

		work_args.arg0 = TE_SMC_PROGRAM_VPR;
		work_args.arg1 = (uintptr_t)vpr_base;
		work_args.arg2 = vpr_size;

		/* workers don't change CPU. depending on the CPU, execute
		 * directly or sched work */
		if (cpu == 0 && (current->flags & PF_WQ_WORKER))
			retval = tlk_generic_smc_on_cpu0(&work_args);
		else
			retval = work_on_cpu(0,
					tlk_generic_smc_on_cpu0, &work_args);
	} else {
		retval = tlk_generic_smc(TE_SMC_PROGRAM_VPR,
					(uintptr_t)vpr_base, vpr_size);
	}

	mutex_unlock(&smc_lock);

	if (retval != OTE_SUCCESS) {
		pr_err("%s: smc failed err (0x%x)\n", __func__, retval);
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

	request->type = TE_SMC_OPEN_SESSION;

	ret = te_prep_mem_buffers(request, context);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_prep_mem_buffers failed err (0x%x)\n",
			__func__, ret);
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

	do_smc(request, context->dev);

	if (request->result) {
		/* release any persistent mem buffers if we failed */
		te_release_persist_mem_buffers(request->session_id, context);
	} else {
		/* mark active any persistent mem buffers */
		te_update_persist_mem_buffers(request->session_id, context);
	}

	te_release_temp_mem_buffers(context);
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
		pr_info("%s: error closing session: %08x\n",
			__func__, request->result);

	/* release any peristent mem buffers */
	te_release_persist_mem_buffers(request->session_id, context);
}

/*
 * Launch operation SMC (supporting client-based te_launch_operation() calls)
 */
void te_launch_operation(struct te_launchop *cmd,
			 struct te_request *request,
			 struct tlk_context *context)
{
	int ret;

	request->session_id = cmd->session_id;
	request->command_id = cmd->operation.command;
	request->type = TE_SMC_LAUNCH_OPERATION;

	ret = te_prep_mem_buffers(request, context);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_prep_mem_buffers failed err (0x%x)\n",
			__func__, ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		return;
	}

	do_smc(request, context->dev);

	if (request->result) {
		/* release any persistent mem buffers if we failed */
		te_release_persist_mem_buffers(request->session_id, context);
	} else {
		/* mark active any persistent mem buffers */
		te_update_persist_mem_buffers(request->session_id, context);
	}

	te_release_temp_mem_buffers(context);
}

/*
 * Open session SMC (supporting client-based te_open_session() calls)
 */
void te_open_session_compat(struct te_opensession_compat *cmd,
			    struct te_request_compat *request,
			    struct tlk_context *context)
{
	int ret;

	request->type = TE_SMC_OPEN_SESSION;

	ret = te_prep_mem_buffers_compat(request, context);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_prep_mem_buffers failed err (0x%x)\n",
			__func__, ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		return;
	}

	memcpy(&request->dest_uuid,
	       &cmd->dest_uuid,
	       sizeof(struct te_service_id));

	pr_info("OPEN_CLIENT_SESSION_COMPAT: 0x%x 0x%x 0x%x 0x%x\n",
		request->dest_uuid[0],
		request->dest_uuid[1],
		request->dest_uuid[2],
		request->dest_uuid[3]);

	do_smc_compat(request, context->dev);

	if (request->result) {
		/* release any persistent mem buffers if we failed */
		te_release_persist_mem_buffers(request->session_id, context);
	} else {
		/* mark active any persistent mem buffers */
		te_update_persist_mem_buffers(request->session_id, context);
	}

	te_release_temp_mem_buffers(context);
}

/*
 * Close session SMC (supporting client-based te_close_session() calls)
 */
void te_close_session_compat(struct te_closesession_compat *cmd,
			     struct te_request_compat *request,
			     struct tlk_context *context)
{
	request->session_id = cmd->session_id;
	request->type = TE_SMC_CLOSE_SESSION;

	do_smc_compat(request, context->dev);
	if (request->result)
		pr_info("%s: error closing session: %08x\n",
			__func__, request->result);

	/* release any peristent mem buffers */
	te_release_persist_mem_buffers(request->session_id, context);
}

/*
 * Launch operation SMC (supporting client-based te_launch_operation() calls)
 */
void te_launch_operation_compat(struct te_launchop_compat *cmd,
				struct te_request_compat *request,
				struct tlk_context *context)
{
	int ret;

	request->session_id = cmd->session_id;
	request->command_id = cmd->operation.command;
	request->type = TE_SMC_LAUNCH_OPERATION;

	ret = te_prep_mem_buffers_compat(request, context);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_prep_mem_buffers failed err (0x%x)\n",
			__func__, ret);
		SET_RESULT(request, ret, OTE_RESULT_ORIGIN_API);
		return;
	}

	do_smc_compat(request, context->dev);

	if (request->result) {
		/* release any persistent mem buffers if we failed */
		te_release_persist_mem_buffers(request->session_id, context);
	} else {
		/* mark active any persistent mem buffers */
		te_update_persist_mem_buffers(request->session_id, context);
	}

	te_release_temp_mem_buffers(context);
}
