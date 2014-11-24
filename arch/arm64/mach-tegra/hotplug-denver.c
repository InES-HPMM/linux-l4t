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
#include <linux/platform/tegra/common.h>

#include <asm/suspend.h>

#include "../../../drivers/platform/tegra/nvdumper/nvdumper-footprint.h"

#include "sleep.h"
#include "pm-soc.h"
#include "pm-tegra132.h"

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
