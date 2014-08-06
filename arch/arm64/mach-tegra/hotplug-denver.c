/*
 *  arch/arm64/mach-tegra/hotplug-denver.c
 *
 *  Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/cpu_pm.h>
#include <linux/clockchips.h>
#include <linux/completion.h>
#include <linux/cpu.h>

#include <asm/suspend.h>

#include "../../../drivers/platform/tegra/nvdumper/nvdumper-footprint.h"

#include "sleep.h"
#include "pm-soc.h"
#include "pm-tegra132.h"

extern volatile ulong secondary_holding_pen_release;

extern bool tegra_suspend_in_progress(void);

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void tegra_cpu_die(unsigned int cpu)
{
	static unsigned long pmstate;

	if (tegra_suspend_in_progress()) {
		/*
		 * Only secondary cores should be killed here. The
		 * main/boot core should die in pm.c during LP0.
		 */
		BUG_ON(cpu == 0);

		/* 2nd cores must be in C7 for LP0/LP1 */
		tegra_tear_down_cpu();

		/* Synchronize with CPU0 */
		while (secondary_holding_pen_release != cpu)
			cpu_relax();
	} else {
		pmstate = T132_CORE_C6;

		do {
			asm volatile(
			"	msr actlr_el1, %0\n"
			"	wfi\n"
			:
			: "r" (pmstate));
		} while (secondary_holding_pen_release != cpu);
	}

#ifdef CONFIG_TEGRA_NVDUMPER
	/* set debug footprint */
	dbg_set_cpu_hotplug_on(cpu, 0xDEAD);
#endif

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
		smp_call_function_single((int)(uintptr_t) hcpu,
						setup_mca, NULL, 1);
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
