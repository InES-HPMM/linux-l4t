/*
 *  Based on arch/arm/kernel/smp.c
 *
 *  Original Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *  Copyright (C) 2013 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/cpu.h>

extern volatile ulong secondary_holding_pen_release;

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void tegra_cpu_die(unsigned int cpu)
{
	static const unsigned long pmstate = 2;

	do {
		/* Enter C6  and wait for secondary_holding_pen_release */
		asm volatile(
		"	msr actlr_el1, %0\n"
		"	wfi\n"
		:
		: "r" (pmstate));
	} while (secondary_holding_pen_release != cpu);
}

noinline void setup_mca(void *info)
{
	unsigned long serr_ctl_enable = 1;
	asm volatile("msr s3_0_c15_c3_2, %0" : : "r" (serr_ctl_enable));
}

noinline int mca_cpu_callback(struct notifier_block *nfb,
				unsigned long action, void *hcpu)
{
	if(action == CPU_ONLINE || action == CPU_ONLINE_FROZEN)
		smp_call_function_single((int) hcpu, setup_mca, NULL, 1);
	return 0;
}

static struct notifier_block mca_cpu_notifier =
{
	.notifier_call = mca_cpu_callback,
};

noinline int __init mca_init(void){
	/* make sure all hotplugged cpus enable mca */
	register_hotcpu_notifier(&mca_cpu_notifier);
	/* make boot cpu enable mca */
	setup_mca(NULL);
	return 0;
}

early_initcall(mca_init);
