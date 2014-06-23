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

#include <asm/suspend.h>
#include <asm/cacheflush.h>

#include "pm.h"
#include "sleep.h"
#include "common.h"
#include "flowctrl.h"
#include "pm-soc.h"
#include "common.h"
#include "iomap.h"
#include "flowctrl.h"
#include "denver-knobs.h"

#include "pm-tegra132.h"

#define PMC_SCRATCH41	0x140 // stores AARCH64 reset vector

#define HALT_REG_CORE0 \
	FLOW_CTRL_WAIT_FOR_INTERRUPT | \
	FLOW_CTRL_HALT_LIC_IRQ | \
	FLOW_CTRL_HALT_LIC_FIQ;

#define HALT_REG_CORE1 FLOW_CTRL_WAITEVENT

#define NS_RST_VEC_WR_DIS	0x2

/* AARCH64 reset vector */
extern void tegra_resume(void);

static int tegra132_reset_vector_init(void);

static bool suspend_in_progress;

bool tegra_suspend_in_progress(void)
{
	smp_rmb();
	return suspend_in_progress;
}

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
		tegra132_reset_vector_init();
		break;
	}

	return NOTIFY_OK;
}

static void set_cpu_reset_vector(u32 vec_phys)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

	/* SecureOS controls reset vector if present */
	if (tegra_cpu_is_secure())
		return;
	writel(vec_phys, pmc + PMC_SCRATCH41);
	readl(pmc + PMC_SCRATCH41);
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
		set_cpu_reset_vector(0);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_notifier_block = {
	.notifier_call = cpu_notify,
};

#ifdef CONFIG_SMP
/*
 * LP0 WAR: Bring all CPUs online before LP0 so that they can be put into C7 on
 * subsequent __cpu_downs otherwise we end up hanging the system by leaving a
 * core in C6 and requesting LP0 from CPU0
 */
static int __cpuinit pm_suspend_notifier(struct notifier_block *nb,
						unsigned long event, void *data)
{
	int cpu, ret;

	if (event != PM_SUSPEND_PREPARE)
		return NOTIFY_OK;

	suspend_in_progress = true;

	dsb();

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
	/* CPU1 is taken out of reset by bootloader for cold boot */
	if (tegra_powergate_is_powered(TEGRA_CPU_POWERGATE_ID(cpu)))
		return;

	/* AARCH64 reset vector */
	set_cpu_reset_vector(virt_to_phys(tegra_resume));

	/* Power ungate CPU */
	tegra_unpowergate_partition(TEGRA_CPU_POWERGATE_ID(cpu));

	/* Remove CPU from reset */
	flowctrl_write_cpu_halt(cpu, 0);
	tegra_cpu_car_ops->out_of_reset(cpu);
}
#endif

void __init tegra_soc_suspend_init(void)
{
	tegra_tear_down_cpu = tegra132_tear_down_cpu;
	tegra_sleep_core_finish = tegra132_sleep_core_finish;

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

static int tegra132_reset_vector_init(void)
{
	extern void *__aarch64_tramp;
	void __iomem *evp_cpu_reset;
	void __iomem *sb_ctrl;
	unsigned long reg;

	/* SecureOS controls reset vector if present */
	if (tegra_cpu_is_secure())
		return 0;

	evp_cpu_reset = IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE + 0x100);
	sb_ctrl = IO_ADDRESS(TEGRA_SB_BASE);

	writel(virt_to_phys(&__aarch64_tramp), evp_cpu_reset);
	wmb();
	reg = readl(evp_cpu_reset);

	/* Prevent further modifications to the physical reset vector. */
	reg = readl(sb_ctrl);
	reg |= NS_RST_VEC_WR_DIS;
	writel(reg, sb_ctrl);
	wmb();

	return 0;
}
early_initcall(tegra132_reset_vector_init);
