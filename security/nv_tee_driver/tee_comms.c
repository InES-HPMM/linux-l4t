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

static int tee_pin_user_pages(void *buffer, size_t size,
	unsigned long *pages_ptr)
{
	int ret = 0, i;
	unsigned int nr_pages;
	struct page **pages = NULL;

	nr_pages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;

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
 * Do an SMC call
 */
static void do_smc(union smc_args_t *smc_args, void *data)
{
	unsigned int *args = smc_args->smc;

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

	if (verbose_smc) {
		pr_info("SMC call:\n");
		pr_info(" %08x %08x\n", args[0], args[1]);
		pr_info(" %08x %08x\n", args[2], args[3]);
		pr_info(" %08x %08x\n", args[4], args[5]);
		pr_info(" %08x %08x\n", args[6], args[7]);
	}

	asm volatile (
		"stmdb	sp!, {r4-r12}\n"
		"mov	r12, %2\n"
		"ldmia	r12, {r0-r7}\n"
#ifdef REQUIRES_SEC
		".arch_extension sec\n"
#endif
		"smc	#0\n"
		"ldmia	sp!, {r4-r12}\n"
		"mov	r2, %0\n"
		"str	r0, [r2]\n"
		"mov	r2, %1\n"
		"cmp	r2, #0\n"
		"strne	r1, [r2]\n"
		: : "r" (&smc_args->answer.result), "r" (data), "r" (args)
		: "r0", "r1", "r2", "r3"
	);

	if (verbose_smc) {
		pr_info("SMC result:\n");
		pr_info(" %08x %08x\n", args[0], args[1]);
		pr_info(" %08x %08x\n", args[2], args[3]);
		pr_info(" %08x %08x\n", args[4], args[5]);
		pr_info(" %08x %08x\n", args[6], args[7]);
	}

#ifdef CONFIG_SMP
	ret = sched_setaffinity(0, &saved_cpu_mask);
	if (ret != 0)
		pr_err("sched_setaffinity #2 -> 0x%lX", ret);
#endif
}

/*
 * Do an 'empty' request just to get more pending answers.
 */
static void get_more_answers(union smc_args_t *smc_buf)
{
	smc_buf->request.type = TMK_SMC_GET_MORE;
	/* rest of smc_buf ignored */
	do_smc(smc_buf, NULL);
}

/*
 * Handle an answer from the secure side.
 * This should unblock the waiting client (according to session_id number)
 * and give it back results from smc_buf->answer union field.
 */
static void interpret_single_answer(union smc_args_t *smc_buf)
{
	pr_info("UNBLOCK client of session %d\n", smc_buf->answer.session_id);
	pr_info(" result=%08x origin=%08x\n",
		smc_buf->answer.result, smc_buf->answer.return_origin);
}

/*
 * Fetch all pending answers from the secure side.
 */
static void interpret_answers(union smc_args_t *smc_buf)
{
	/*
	 * Future improvement would be to signal the number
	 * of pending answers from secure side
	 */
	while (smc_buf->answer.type == TMK_SMC_ANSWER) {
		interpret_single_answer(smc_buf);
		get_more_answers(smc_buf);
	}

	if (smc_buf->answer.type != TMK_SMC_NO_ANSWER)
		pr_info("Protocol error: expected NO_ANSWER\n");
}

static void tee_setup_smc_buf(union smc_args_t *smc_buf,
	uint32_t type, uint32_t session_id,
	uint32_t command_id, phys_addr_t phy_cmd_page)
{
	smc_buf->request.type = type;
	smc_buf->request.session_id = session_id;
	smc_buf->request.command_id = command_id;
	smc_buf->request.cmd_param = phy_cmd_page;
}


/*
 * Open session SMC (TEEC_OpenSession)
 */
int tee_open_session(struct tee_opensession *cmd,
		phys_addr_t phy_cmd_page,
		struct tee_answer *answer)
{
	union smc_args_t smc_buf;
	unsigned int id;

	tee_setup_smc_buf(&smc_buf, TMK_SMC_OPEN_SESSION, cmd->login_types,
		cmd->login_data, phy_cmd_page);

	do_smc(&smc_buf, &id);
	if (smc_buf.answer.result) {
		pr_err("Error opening session: %08x %08x\n",
			smc_buf.answer.result,
			smc_buf.answer.return_origin);
		return -1;
	}

	answer->session_id = id;
	return 0;
}

/*
 * Close session SMC (TEEC_CloseSession)
 */
int tee_close_session(uint32_t session_id)
{
	union smc_args_t smc_buf;

	tee_setup_smc_buf(&smc_buf, TMK_SMC_CLOSE_SESSION, session_id,
		0, 0);
	do_smc(&smc_buf, NULL);
	if (smc_buf.answer.result) {
		pr_info("Error closing session: %08x\n", smc_buf.answer.result);
		return -1;
	}

	return 0;
}

int tee_pin_mem_buffers(void *buffer, size_t size,
	struct nv_tee_context *context)
{

	unsigned long pages = NULL;
	struct nv_shmem_desc *shmem_desc = NULL;
	int ret = 0, nr_pages = 0;

	nr_pages = tee_pin_user_pages(buffer, size, &pages);
	if (nr_pages <= 0) {
		pr_err("tee_pin_mem_buffers: tee_pin_user_pages Failed\n");
		ret = -EFAULT;
		goto error;
	}

	shmem_desc = tee_add_shmem_desc(buffer, size,
				nr_pages, (struct page **)pages, context);
	if (!shmem_desc) {
		pr_err("tee_pin_mem_buffers: tee_add_shmem_desc Failed\n");
		ret = -EFAULT;
		goto error;
	}

	return 0;
error:
	return ret;
}

/*
 * Register Shared Memory SMC (TEEC_RegisterSharedMemory)
 */
int tee_register_memory(struct tee_sharedmem *cmd, phys_addr_t phy_cmd_page,
	struct tee_answer *answer, struct nv_tee_context *context)
{
	int ret = 0;
	union smc_args_t smc_buf;

	ret = tee_pin_mem_buffers(cmd->memref.buffer, cmd->memref.size,
		context);
	if (ret < 0) {
		ret = -EFAULT;
		goto error;
	}

	tee_setup_smc_buf(&smc_buf, TMK_SMC_REG_SHARED_MEM, cmd->session_id,
		0, phy_cmd_page);

	do_smc(&smc_buf, NULL);

	return 0;
error:
	return ret;
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

/*
 * Invoke Command SMC (TEEC_InvokeCommand)
 */

int tee_invoke_command(struct tee_invokecmd *cmd, phys_addr_t phy_cmd_page,
	struct tee_answer *answer)
{
	union smc_args_t smc_buf;

	pr_info("tee_invoke_command: session_id (%d) command_id(%d)\n",
		cmd->session_id, cmd->command_id);

	tee_setup_smc_buf(&smc_buf, TMK_SMC_INVOKE_CMD, cmd->session_id,
		cmd->command_id, phy_cmd_page);

	do_smc(&smc_buf, NULL);

	if (smc_buf.answer.result) {
		pr_err("Error opening session: %08x %08x\n",
			smc_buf.answer.result,
			smc_buf.answer.return_origin);
		return -1;
	}
	return 0;
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
