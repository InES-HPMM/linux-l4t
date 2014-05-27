/*
 * arch/arm64/mach-tegra/nvdumper_regdump.c
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/kdebug.h>
#include <linux/notifier.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <mach/nvdumper.h>

#define THREAD_INFO(sp) ((struct thread_info *) \
		((unsigned long)(sp) & ~(THREAD_SIZE - 1)))

#define DEBUG_REGDUMP 0

static int max_cpus;
static struct nvdumper_cpu_data_t *nvdumper_cpu_data;
static dma_addr_t nvdumper_p;

void save_aar64_sys_regs(struct aar64_system_regs_t *aar64_sys_regs)
{
	asm("mrs    x1, ACTLR_EL1\n\t"
	    "str    x1, [%0, #8 * 0]\n\t"
	    "mrs    x1, AFSR0_EL1\n\t"
	    "str    x1, [%0, #8 * 1]\n\t"
	    "mrs    x1, AFSR1_EL1\n\t"
	    "str    x1, [%0, #8 * 2]\n\t"
	    "mrs    x1, AMAIR_EL1\n\t"
	    "str    x1, [%0, #8 * 3]\n\t"
	    "mrs    x1, CONTEXTIDR_EL1\n\t"
	    "str    x1, [%0, #8 * 4]\n\t"
	    "mrs    x1, CPACR_EL1\n\t"
	    "str    x1, [%0, #8 * 5]\n\t"
	    "mrs    x1, CSSELR_EL1\n\t"
	    "str    x1, [%0, #8 * 6]\n\t"
	    "mrs    x1, FAR_EL1\n\t"
	    "str    x1, [%0, #8 * 7]\n\t"
	    "mrs    x1, ESR_EL1\n\t"
	    "str    x1, [%0, #8 * 8]\n\t"
	    "mrs    x1, PAR_EL1\n\t"
	    "str    x1, [%0, #8 * 9]\n\t"
	    :		                        /* output */
	    : "r"(aar64_sys_regs)			/* input */
	    : "%x1", "memory"			    /* clobbered register */
	);
}

void nvdumper_save_regs(void *data)
{
	int id = smp_processor_id();

	if (!nvdumper_cpu_data)	{
		pr_info("nvdumper_cpu_data is not initialized!\n");
		return;
	}

	if (current_thread_info())
		nvdumper_cpu_data[id].current_task =
				current_thread_info()->task;
	nvdumper_cpu_data[id].is_online = true;

	__asm__ __volatile__ (
	    "mov    %[_arm_sp], sp\n\t"
	    "adr    %[_arm_pc], 1f\n\t"
	    "mov    %[regx0], x0\n\t"
	    "mov    %[regx1], x1\n\t"
	    "mov    %[regx2], x2\n\t"
	    "mov    %[regx3], x3\n\t"
	    "mov    %[regx4], x4\n\t"
	    "mov    %[regx5], x5\n\t"
	    "mov    %[regx6], x6\n\t"
	    "mov    %[regx7], x7\n\t"
	    "mov    %[regx8], x8\n\t"
	    "mov    %[regx9], x9\n\t"
	    "mov    %[regx10], x10\n\t"
	    "mov    %[regx11], x11\n\t"
	    "mov    %[regx12], x12\n\t"
	    "mov    %[regx13], x13\n\t"
	    "mov    %[regx14], x14\n\t"
	    "mov    %[regx15], x15\n\t"
	    "1:" :
	    [_arm_pc] "=r" (nvdumper_cpu_data[id].pt_regs.pc),
		[_arm_sp] "=r" (nvdumper_cpu_data[id].pt_regs.sp),
		[regx0] "=r" (nvdumper_cpu_data[id].pt_regs.regs[0]),
		[regx1] "=r" (nvdumper_cpu_data[id].pt_regs.regs[1]),
		[regx2] "=r" (nvdumper_cpu_data[id].pt_regs.regs[2]),
		[regx3] "=r" (nvdumper_cpu_data[id].pt_regs.regs[3]),
		[regx4] "=r" (nvdumper_cpu_data[id].pt_regs.regs[4]),
		[regx5] "=r" (nvdumper_cpu_data[id].pt_regs.regs[5]),
		[regx6] "=r" (nvdumper_cpu_data[id].pt_regs.regs[6]),
		[regx7] "=r" (nvdumper_cpu_data[id].pt_regs.regs[7]),
		[regx8] "=r" (nvdumper_cpu_data[id].pt_regs.regs[8]),
		[regx9] "=r" (nvdumper_cpu_data[id].pt_regs.regs[9]),
		[regx10] "=r" (nvdumper_cpu_data[id].pt_regs.regs[10]),
		[regx11] "=r" (nvdumper_cpu_data[id].pt_regs.regs[11]),
		[regx12] "=r" (nvdumper_cpu_data[id].pt_regs.regs[12]),
		[regx13] "=r" (nvdumper_cpu_data[id].pt_regs.regs[13]),
		[regx14] "=r" (nvdumper_cpu_data[id].pt_regs.regs[14]),
		[regx15] "=r" (nvdumper_cpu_data[id].pt_regs.regs[15])
	);

	__asm__ __volatile__ (
	    "mov    %[regx16], x16\n\t"
	    "mov    %[regx17], x17\n\t"
	    "mov    %[regx18], x18\n\t"
	    "mov    %[regx19], x19\n\t"
	    "mov    %[regx20], x20\n\t"
	    "mov    %[regx21], x21\n\t"
	    "mov    %[regx22], x22\n\t"
	    "mov    %[regx23], x23\n\t"
	    "mov    %[regx24], x24\n\t"
	    "mov    %[regx25], x25\n\t"
	    "mov    %[regx26], x26\n\t"
	    "mov    %[regx27], x27\n\t"
	    "mov    %[regx28], x28\n\t"
	    "mov    %[regx29], x29\n\t"
	    "mov    %[regx30], x30\n\t"
	    "1:" :
		[regx16] "=r" (nvdumper_cpu_data[id].pt_regs.regs[16]),
	    [regx17] "=r" (nvdumper_cpu_data[id].pt_regs.regs[17]),
		[regx18] "=r" (nvdumper_cpu_data[id].pt_regs.regs[18]),
		[regx19] "=r" (nvdumper_cpu_data[id].pt_regs.regs[19]),
		[regx20] "=r" (nvdumper_cpu_data[id].pt_regs.regs[20]),
		[regx21] "=r" (nvdumper_cpu_data[id].pt_regs.regs[21]),
		[regx22] "=r" (nvdumper_cpu_data[id].pt_regs.regs[22]),
		[regx23] "=r" (nvdumper_cpu_data[id].pt_regs.regs[23]),
		[regx24] "=r" (nvdumper_cpu_data[id].pt_regs.regs[24]),
		[regx25] "=r" (nvdumper_cpu_data[id].pt_regs.regs[25]),
		[regx26] "=r" (nvdumper_cpu_data[id].pt_regs.regs[26]),
		[regx27] "=r" (nvdumper_cpu_data[id].pt_regs.regs[27]),
		[regx28] "=r" (nvdumper_cpu_data[id].pt_regs.regs[28]),
		[regx29] "=r" (nvdumper_cpu_data[id].pt_regs.regs[29]),
		[regx30] "=r" (nvdumper_cpu_data[id].pt_regs.regs[30])
	);

	save_aar64_sys_regs(&nvdumper_cpu_data[id].aar64_sys_regs);

	pr_info("nvdumper: all registers are saved.\n");
}

void nvdumper_crash_setup_regs(void)
{
	pr_info("Enter nvdumper_crash_setup_regs\n");
	on_each_cpu(nvdumper_save_regs, NULL, 1);
}

void nvdumper_copy_regs(unsigned int id, struct pt_regs *regs, void *svc_sp)
{
	struct thread_info *thread_info = THREAD_INFO(svc_sp);
	nvdumper_cpu_data[id].current_task = thread_info->task;
	nvdumper_cpu_data[id].is_online = true;
	memcpy(&nvdumper_cpu_data[id].pt_regs, regs, sizeof(struct pt_regs));
	save_aar64_sys_regs(&nvdumper_cpu_data[id].aar64_sys_regs);
}

void print_cpu_data(int id)
{
	struct pt_regs *pt_regs = &nvdumper_cpu_data[id].pt_regs;
	struct aar64_system_regs_t *s_regs =
				&nvdumper_cpu_data[id].aar64_sys_regs;

	pr_info("------------------------------------------------\n");
	pr_info("CPU%d Status: %s\n", id,
		nvdumper_cpu_data[id].is_online ? "online" : "offline");
	pr_info("current task: %p\n", nvdumper_cpu_data[id].current_task);

	if (nvdumper_cpu_data[id].is_online) {
		int i;
		for (i = 0; i < 31; i++) {
			if (i < 10) /* Keep "=" align */
				pr_info("ARM_x%d    = 0x%016llx\n",
					i, pt_regs->regs[i]);
			else
				pr_info("ARM_x%d   = 0x%016llx\n",
					i, pt_regs->regs[i]);
		}
		pr_info("ARM_sp    = 0x%016llx\n", pt_regs->sp);
		pr_info("ARM_pc    = 0x%016llx\n", pt_regs->pc);

		/* Print system register */
		pr_info("ACTLR_EL1      = 0x%016llx\n", s_regs->ACTLR_EL1);
		pr_info("AFSR0_EL1      = 0x%016llx\n", s_regs->AFSR0_EL1);
		pr_info("AFSR1_EL1      = 0x%016llx\n", s_regs->AFSR1_EL1);
		pr_info("AMAIR_EL1      = 0x%016llx\n", s_regs->AMAIR_EL1);
		pr_info("CONTEXTIDR_EL1 = 0x%016llx\n",
			s_regs->CONTEXTIDR_EL1);
		pr_info("CPACR_EL1      = 0x%016llx\n", s_regs->CPACR_EL1);
		pr_info("CSSELR_EL1     = 0x%016llx\n", s_regs->CSSELR_EL1);
		pr_info("FAR_EL1        = 0x%016llx\n", s_regs->FAR_EL1);
		pr_info("ESR_EL1        = 0x%016llx\n", s_regs->ESR_EL1);
		pr_info("PAR_EL1        = 0x%016llx\n", s_regs->PAR_EL1);
		pr_info("MAIR_EL1       = 0x%016llx\n", s_regs->MAIR_EL1);
		pr_info("RMR_EL1        = 0x%016llx\n", s_regs->RMR_EL1);
		pr_info("SCTLR_EL1      = 0x%016llx\n", s_regs->SCTLR_EL1);
		pr_info("TEECR32_EL1    = 0x%016llx\n", s_regs->TEECR32_EL1);
		pr_info("TEEHBR32_EL1   = 0x%016llx\n", s_regs->TEEHBR32_EL1);
		pr_info("TPIDR_EL1      = 0x%016llx\n", s_regs->TPIDR_EL1);
		pr_info("TCR_EL1        = 0x%016llx\n", s_regs->TCR_EL1);
		pr_info("TTBR0_EL1      = 0x%016llx\n", s_regs->TTBR0_EL1);
		pr_info("TTBR1_EL1      = 0x%016llx\n", s_regs->TTBR1_EL1);
		pr_info("VBAR_EL1       = 0x%016llx\n", s_regs->VBAR_EL1);
		pr_info("TPIDRRO_EL0    = 0x%016llx\n", s_regs->TPIDRRO_EL0);
		pr_info("TPIDR_EL0      = 0x%016llx\n", s_regs->TPIDR_EL0);
	}
}

void nvdumper_print_data(void)
{
	int id;

	for_each_present_cpu(id)
		print_cpu_data(id);
}

int nvdumper_die_handler(struct notifier_block *nb, unsigned long reason,
					void *data)
{
	nvdumper_crash_setup_regs();
	return NOTIFY_OK;
}

static int nvdumper_panic_handler(struct notifier_block *this,
					unsigned long event, void *unused)
{
#if DEBUG_REGDUMP
	nvdumper_print_data();
#endif
	flush_cache_all();

	return NOTIFY_OK;
}

struct notifier_block nvdumper_die_notifier = {
	.notifier_call = nvdumper_die_handler,
	.priority      = INT_MAX-1, /* priority: INT_MAX >= x >= 0 */
};

static struct notifier_block nvdumper_panic_notifier = {
	.notifier_call = nvdumper_panic_handler,
	.priority      = INT_MAX-1, /* priority: INT_MAX >= x >= 0 */
};

int nvdumper_regdump_init(void)
{
	int ret;

	max_cpus = num_possible_cpus();

	nvdumper_cpu_data = dma_alloc_coherent(NULL,
		sizeof(struct nvdumper_cpu_data_t) * max_cpus,
		&nvdumper_p, 0);
	if (!nvdumper_cpu_data) {
		pr_err("%s: can not allocate bounce buffer\n", __func__);

		return -ENOMEM;
	}

	ret = register_die_notifier(&nvdumper_die_notifier);
	if (ret != 0) {
		pr_err("%s: registering die notifier failed with err=%d\n",
				__func__, ret);
		goto err_out1;
	}

	ret = atomic_notifier_chain_register(&panic_notifier_list,
		&nvdumper_panic_notifier);
	if (ret != 0) {
		pr_err("%s: unable to register a panic notifier (err=%d)\n",
				__func__, ret);
		goto err_out2;
	}

	return ret;

err_out2:
	unregister_die_notifier(&nvdumper_die_notifier);

err_out1:
	if (nvdumper_cpu_data)
		dma_free_coherent(NULL,
				sizeof(struct nvdumper_cpu_data_t) * max_cpus,
				nvdumper_cpu_data, nvdumper_p);

	return ret;
}

void nvdumper_regdump_exit(void)
{
	dma_free_coherent(NULL, sizeof(struct nvdumper_cpu_data_t) * max_cpus,
			nvdumper_cpu_data, nvdumper_p);
	unregister_die_notifier(&nvdumper_die_notifier);
	atomic_notifier_chain_unregister(&panic_notifier_list,
			&nvdumper_panic_notifier);
}
