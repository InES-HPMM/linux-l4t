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

#include <asm/suspend.h>
#include <asm/cacheflush.h>

#include <linux/tegra-pmc.h>

#include "pm.h"
#include "sleep.h"
#include "flowctrl.h"
#include "pm-soc.h"
#include "pm-tegra132.h"
#include "denver-knobs.h"

#define HALT_REG_CORE0 \
	FLOW_CTRL_WAIT_FOR_INTERRUPT | \
	FLOW_CTRL_HALT_LIC_IRQ | \
	FLOW_CTRL_HALT_LIC_FIQ;

#define HALT_REG_CORE1 FLOW_CTRL_WAITEVENT

static int tegra132_enter_sleep(unsigned long pmstate)
{
	u32 reg;
	int cpu = smp_processor_id();

	reg = cpu ? HALT_REG_CORE1 : HALT_REG_CORE0;
	flowctrl_write_cpu_halt(cpu, reg);
	reg = readl(FLOW_CTRL_HALT_CPU(cpu));

	do {
		asm volatile(
		"	isb\n"
		"	msr actlr_el1, %0\n"
		"	wfi\n"
		:
		: "r" (pmstate));
	} while (0);

	return 0;
}

/*
 * tegra132_tear_down_cpu
 *  - core power gating entry finisher
 */
static void tegra132_tear_down_cpu(void)
{
	int cpu = smp_processor_id();
	u32 reg;

	BUG_ON(cpu == 0);

	tegra_psci_suspend_cpu(tegra_resume);

	local_irq_disable();
	local_fiq_disable();

	cpu_pm_enter();

	/* FlowCtrl programming */
	reg = readl(FLOW_CTRL_CPU_CSR(cpu));
	reg &= ~FLOW_CTRL_CSR_WFE_BITMAP;	/* clear wfe bitmap */
	reg &= ~FLOW_CTRL_CSR_WFI_BITMAP;	/* clear wfi bitmap */
	reg |= FLOW_CTRL_CSR_INTR_FLAG;		/* clear intr flag */
	reg |= FLOW_CTRL_CSR_EVENT_FLAG;	/* clear event flag */
	reg |= FLOW_CTRL_CSR_WFI_CPU0 << cpu;	/* enable power gating on wfi */
	reg |= FLOW_CTRL_CSR_ENABLE;		/* enable power gating */
	flowctrl_writel(reg, FLOW_CTRL_CPU_CSR(cpu));
	reg = readl(FLOW_CTRL_CPU_CSR(cpu));

	cpu_suspend(T132_CORE_C7, tegra132_enter_sleep);

	cpu_pm_exit();
}

/*
 * tegra132_sleep_core_finish(unsigned long v2p)
 *  - cluster power state entry finisher
 *  - v2p: ignored
 */
static int tegra132_sleep_core_finish(unsigned long v2p)
{
	tegra132_enter_sleep(T132_SYSTEM_LP0);
	return 0;
}

static int cpu_pm_notify(struct notifier_block *self,
					 unsigned long action, void *hcpu)
{
	int cpu = smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
	case CPU_CLUSTER_PM_ENTER:
		denver_set_bg_allowed(cpu, false);
		break;

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

void tegra_soc_suspend_init(void)
{
	tegra_tear_down_cpu = tegra132_tear_down_cpu;
	tegra_sleep_core_finish = tegra132_sleep_core_finish;

	/* Notifier to disable/enable BGALLOW */
	cpu_pm_register_notifier(&cpu_pm_notifier_block);

	/* Notifier to enable BGALLOW for CPU1-only */
	register_hotcpu_notifier(&cpu_notifier_block);
}
