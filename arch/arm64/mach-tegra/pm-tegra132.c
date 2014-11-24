/*
 * arch/arm64/mach-tegra/pm-tegra132.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/tegra-pmc.h>
#include <linux/clk/tegra.h>
#include <linux/tegra-powergate.h>

#include <asm/compiler.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>

#include "pm.h"
#include "sleep.h"
#include <linux/platform/tegra/common.h>
#include <linux/platform/tegra/flowctrl.h>
#include "pm-soc.h"
#include <linux/platform/tegra/common.h>
#include "iomap.h"
#include <linux/platform/tegra/flowctrl.h>
#include "denver-knobs.h"

#include "pm-tegra132.h"

#define SMC_CPU_NEXT_PWRDN	0x82000010
#define  ENTER_C6_STATE		0
#define  ENTER_C7_STATE		1

static int cpu_pm_notify(struct notifier_block *self,
					 unsigned long action, void *hcpu)
{
	int cpu = smp_processor_id();

	switch (action) {
	case CPU_CLUSTER_PM_EXIT:
		denver_set_bg_allowed(cpu, true);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_pm_notifier_block = {
	.notifier_call = cpu_pm_notify,
};

static int cpu_notify(struct notifier_block *self,
					 unsigned long action, void *hcpu)
{
	long cpu = (long) hcpu;

	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		denver_set_bg_allowed(cpu, true);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_notifier_block = {
	.notifier_call = cpu_notify,
};

#ifdef CONFIG_SMP
static notrace void send_deep_sleep_smc(u64 function_id, u64 arg0,
					u64 arg1, u64 arg2)
{
	asm volatile(
			"mov	x0, %0		\n"
			"mov	x1, %1		\n"
			"mov	x2, %2		\n"
			"mov	x3, %3		\n"
			"smc	#0\n"
		: : "r" (function_id), "r" (arg0), "r" (arg1), "r" (arg2)
		: "x0", "x1", "x2", "x3");
}

/*
 * LP0 WAR: Bring all CPUs online before LP0 so that they can be put into C7 on
 * subsequent __cpu_downs otherwise we end up hanging the system by leaving a
 * core in C6 and requesting LP0 from CPU0
 */
static int __cpuinit pm_suspend_notifier(struct notifier_block *nb,
						unsigned long event, void *data)
{
	int cpu, ret;

	if ((event != PM_SUSPEND_PREPARE) && (event != PM_POST_SUSPEND))
		return NOTIFY_OK;

	/* EL3 monitor controls LP0 entry/exit */
	if (event == PM_POST_SUSPEND) {
		/* enter C6 state on next power down request */
		send_deep_sleep_smc(SMC_CPU_NEXT_PWRDN, ENTER_C6_STATE, 0, 0);
		return NOTIFY_OK;
	}

	/* enter C7 state on next power down request */
	send_deep_sleep_smc(SMC_CPU_NEXT_PWRDN, ENTER_C7_STATE, 0, 0);

	for_each_present_cpu(cpu) {
		if (!cpu)
			continue;
		ret = cpu_up(cpu);

		/*
		 * Error in getting CPU out of C6. Let -EINVAL through as CPU
		 * could have come online
		 */
		if (ret && ret != -EINVAL) {
			pr_err("%s: Couldn't bring up CPU%d on LP0 entry: %d\n",
					__func__, cpu, ret);
			return NOTIFY_BAD;
		}
	}

	return NOTIFY_OK;
}

/*
 * Note: The priority of this notifier needs to be higher than cpu_hotplug's
 * suspend notifier otherwise the subsequent cpu_up operation in
 * pm_suspend_notifier will fail
 */
static struct notifier_block __cpuinitdata suspend_notifier = {
	.notifier_call = pm_suspend_notifier,
	.priority = 1,
};

static void tegra132_boot_secondary_cpu(int cpu)
{
	/*
	 * Wake up secondaries stuck at WFI
	 */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));
}
#endif

void __init tegra_soc_suspend_init(void)
{
	/* Notifier to disable/enable BGALLOW */
	cpu_pm_register_notifier(&cpu_pm_notifier_block);

	/* Notifier to enable BGALLOW for CPU1-only */
	register_hotcpu_notifier(&cpu_notifier_block);

#ifdef CONFIG_SMP
	tegra_boot_secondary_cpu = tegra132_boot_secondary_cpu;

	/* Notifier to wakeup CPU1 to enter C7 before LP0 */
	if (register_pm_notifier(&suspend_notifier))
		pr_err("%s: Failed to register suspend notifier\n", __func__);
#endif
}
