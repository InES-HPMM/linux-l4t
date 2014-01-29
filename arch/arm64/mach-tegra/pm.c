/*
 * CPU complex suspend & resume functions for Tegra SoCs
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
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
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/serial_reg.h>
#include <linux/syscore_ops.h>
#include <linux/tegra-timer.h>

#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <asm/idmap.h>
#include <asm/proc-fns.h>
#include <asm/tlbflush.h>

#include <mach/hardware.h>

#include "iomap.h"
#include "reset.h"
#include "flowctrl.h"
#include "sleep.h"
#include "board.h"

#include "pmc.h"
#include "pm.h"
#include "pm-tegra132.h"

/* core power request enable */
#define TEGRA_POWER_PWRREQ_OE		(1 << 9)
/* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_EFFECT_LP0		(1 << 14)
/* CPU pwr req enable */
#define TEGRA_POWER_CPU_PWRREQ_OE	(1 << 16)

#define PMC_CTRL		0x0
#define PMC_CPUPWRGOOD_TIMER	0xc8
#define PMC_CPUPWROFF_TIMER	0xcc
#define PMC_DPD_SAMPLE	 0x20

#define PMC_DPD_ENABLE			0x24
#define PMC_DPD_ENABLE_TSC_MULT_ENABLE	(1 << 1)

#define PMC_TSC_MULT			0x2b4
#define PMC_TSC_MULT_FREQ_STS		(1 << 16)

#define TSC_TIMEOUT_US			32

#ifdef CONFIG_PM_SLEEP
static DEFINE_SPINLOCK(tegra_lp2_lock);
static void __iomem *iram_code = IO_ADDRESS(TEGRA_IRAM_CODE_AREA);
static u32 iram_save_size;
static void *iram_save_addr;
struct tegra_lp1_iram *tegra_lp1_iram;

void (*tegra_tear_down_cpu)(void);
void (*tegra_sleep_core_finish)(unsigned long v2p);

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
#define pmc_writel(value, reg) \
		writel(value, (void *)pmc + (reg))
#define pmc_readl(reg) \
		readl((void *)pmc + (reg))

static struct clk *tegra_pclk;

static u32 clk_csite_src;
static void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
#define CLK_RESET_SOURCE_CSITE	0x1d4

enum tegra_suspend_mode current_suspend_mode = TEGRA_SUSPEND_NONE;

static u64 resume_time;
static u64 resume_entry_time;
static u64 suspend_time;
static u64 suspend_entry_time;

static RAW_NOTIFIER_HEAD(tegra_pm_chain_head);

static struct pmc_pm_data *pmc_pm_data;

bool tegra_is_dpd_mode = false;
int tegra_register_pm_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&tegra_pm_chain_head, nb);
}
EXPORT_SYMBOL(tegra_register_pm_notifier);

int tegra_unregister_pm_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&tegra_pm_chain_head, nb);
}
EXPORT_SYMBOL(tegra_unregister_pm_notifier);

static int tegra_pm_notifier_call_chain(unsigned int val)
{
	int ret = raw_notifier_call_chain(&tegra_pm_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

bool tegra_dvfs_is_dfll_bypass(void)
{
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
	return true;
#else
	return false;
#endif
}

#ifdef CONFIG_ARM_ARCH_TIMER
void tegra_tsc_suspend(void)
{
	u32 reg;
	reg = pmc_readl(PMC_DPD_ENABLE);
	reg |= PMC_DPD_ENABLE_TSC_MULT_ENABLE;
	pmc_writel(reg, PMC_DPD_ENABLE);
}

void tegra_tsc_resume(void)
{
	u32 reg;
	reg = pmc_readl(PMC_DPD_ENABLE);
	reg &= ~PMC_DPD_ENABLE_TSC_MULT_ENABLE;
	pmc_writel(reg, PMC_DPD_ENABLE);
}
#endif /* CONFIG_ARM_ARCH_TIMER */

void tegra_log_suspend_time(void)
{
	suspend_entry_time = timer_readl(TIMERUS_CNTR_1US);
}

void tegra_log_resume_time(void)
{
	u64 resume_end_time = timer_readl(TIMERUS_CNTR_1US);

	if (resume_entry_time > resume_end_time)
		resume_end_time |= 1ull<<32;
	resume_time = resume_end_time - resume_entry_time;
}

/*
 * restore_cpu_complex
 *
 * Set up wakeup events in flow contrller.
 *
 * Always called on CPU 0.
 */
static void restore_cpu_complex(void)
{
	int cpu = smp_processor_id();

	BUG_ON(cpu != 0);

#ifdef CONFIG_SMP
	cpu = cpu_logical_map(cpu);
#endif

	/* TODO: flow controller programming if needed */
}

/*
 * suspend_cpu_complex
 *
 * Clear wake events in flow controller.
 *
 * Must always be called on cpu 0.
 */
static void suspend_cpu_complex(void)
{
	int cpu = smp_processor_id();

	BUG_ON(cpu != 0);

#ifdef CONFIG_SMP
	cpu = cpu_logical_map(cpu);
#endif

	/* TODO: flow controller programming if needed */
}

void tegra_clear_cpu_in_lp2(int phy_cpu_id)
{
	ulong *cpu_in_lp2 = (ulong *)tegra_cpu_lp2_mask;

	spin_lock(&tegra_lp2_lock);

	BUG_ON(!(*cpu_in_lp2 & BIT(phy_cpu_id)));
	*cpu_in_lp2 &= ~BIT(phy_cpu_id);

	spin_unlock(&tegra_lp2_lock);
}

bool tegra_set_cpu_in_lp2(int phy_cpu_id)
{
	bool last_cpu = false;
	cpumask_t *cpu_lp2_mask = tegra_cpu_lp2_mask;
	ulong *cpu_in_lp2 = (ulong *)tegra_cpu_lp2_mask;

	spin_lock(&tegra_lp2_lock);

	BUG_ON((*cpu_in_lp2 & BIT(phy_cpu_id)));
	*cpu_in_lp2 |= BIT(phy_cpu_id);

	if ((phy_cpu_id == 0) && cpumask_equal(cpu_lp2_mask, cpu_online_mask))
		last_cpu = true;

	spin_unlock(&tegra_lp2_lock);
	return last_cpu;
}

static void tegra_pmc_pm_set(enum tegra_suspend_mode mode)
{
	u32 reg, boot_flag;

	reg = pmc_readl(PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;

	if (pmc_pm_data->combined_req)
		reg &= ~TEGRA_POWER_PWRREQ_OE;
	else
		reg |= TEGRA_POWER_PWRREQ_OE;
	reg &= ~TEGRA_POWER_EFFECT_LP0;

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		/*
		 * LP0 boots through the AVP, which then resumes the AVP to
		 * the address in scratch 39, and the cpu to the address in
		 * scratch 41 to tegra_resume
		 */
		pmc_writel(0x0, PMC_SCRATCH39);

		/*
		 * Enable DPD sample to trigger sampling pads data and direction
		 * in which pad will be driven during LP0 mode.
		 */
		pmc_writel(0x1, PMC_DPD_SAMPLE);

		/* Set warmboot flag */
		boot_flag = pmc_readl(PMC_SCRATCH0);
		pmc_writel(boot_flag | 1, PMC_SCRATCH0);

		pmc_writel(tegra_lp0_vec_start, PMC_SCRATCH1);
		reg |= TEGRA_POWER_EFFECT_LP0;
		break;

	default:
		break;
	}

	pmc_writel(reg, PMC_CTRL);
}

/* Borrowed from setup_reset in arch/arm64/process.c */
static void setup_sleep(void)
{
	/*
	 * Tell the mm system that we are going to reboot -
	 * we may need it to insert some 1:1 mappings so that
	 * soft boot works.
	 */
	setup_mm_for_reboot();

	/* Clean and invalidate caches */
	flush_cache_all();

	/* Turn D-cache off */
	cpu_cache_off();

	/* Push out any further dirty data, and ensure cache is empty */
	flush_cache_all();
}

static int tegra_sleep_cpu(unsigned long v2p)
{
	setup_sleep();

	tegra_sleep_cpu_finish(v2p);

	/* should never here */
	BUG();

	return 0;
}

static int tegra_sleep_core(unsigned long v2p)
{
	setup_sleep();

	tegra_sleep_core_finish(v2p);

	/* should never here */
	BUG();

	return 0;
}

void tegra_idle_lp2_last(u32 cpu_on_time, u32 cpu_off_time)
{
	tegra_pmc_pm_set(TEGRA_SUSPEND_LP2);

	cpu_cluster_pm_enter();
	suspend_cpu_complex();

	cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, &tegra_sleep_cpu);

	restore_cpu_complex();
	cpu_cluster_pm_exit();
}

/*
 * tegra_lp1_iram_hook
 *
 * Hooking the address of LP1 reset vector and SDRAM self-refresh code in
 * SDRAM. These codes not be copied to IRAM in this function. We need to
 * copy these code to IRAM before LP0/LP1 suspend and restore the content
 * of IRAM after resume.
 */
static bool tegra_lp1_iram_hook(void)
{
	switch (tegra_get_chipid()) {
	case TEGRA_CHIPID_TEGRA13:
		tegra132_lp1_iram_hook();
		break;
	default:
		break;
	}

	if (!tegra_lp1_iram)
		return false;

	iram_save_size = tegra_lp1_iram->end_addr - tegra_lp1_iram->start_addr;
	iram_save_addr = kmalloc(iram_save_size, GFP_KERNEL);
	if (!iram_save_addr)
		return false;

	return true;
}

static bool tegra_sleep_core_init(void)
{
	switch (tegra_get_chipid()) {
	case TEGRA_CHIPID_TEGRA13:
		tegra132_sleep_core_init();
		break;
	default:
		break;
	}

	if (!tegra_sleep_core_finish)
		return false;

	return true;
}

static void tegra_suspend_enter_lp0(void)
{
	tegra_cpu_reset_handler_save();
	tegra_tsc_suspend();
}

static void tegra_suspend_exit_lp0(void)
{
	tegra_tsc_resume();
	tegra_cpu_reset_handler_restore();
}

static void tegra_suspend_enter_lp1(void)
{
	pmc_writel(virt_to_phys(tegra_resume), PMC_SCRATCH41);

	/* copy the reset vector & SDRAM shutdown code into IRAM */
	memcpy(iram_save_addr, iram_code, iram_save_size);
	memcpy(iram_code, tegra_lp1_iram->start_addr, iram_save_size);

	*((ulong *)tegra_cpu_lp1_mask) = 1;
}

static void tegra_suspend_exit_lp1(void)
{
	/* Clear DPD sample */
	pmc_writel(0x0, PMC_DPD_SAMPLE);

	pmc_writel(0x0, PMC_SCRATCH41);
	pmc_writel(0x0, PMC_SCRATCH39);

	/* restore IRAM */
	memcpy(iram_code, iram_save_addr, iram_save_size);

	*((ulong *)tegra_cpu_lp1_mask) = 0;
}

static const char *lp_state[TEGRA_MAX_SUSPEND_MODE] = {
	[TEGRA_SUSPEND_NONE] = "none",
	[TEGRA_SUSPEND_LP2] = "LP2",
	[TEGRA_SUSPEND_LP1] = "LP1",
	[TEGRA_SUSPEND_LP0] = "LP0",
};

static int tegra_suspend_enter(suspend_state_t state)
{
	enum tegra_suspend_mode mode = current_suspend_mode;

	pr_info("Entering suspend state %s\n", lp_state[mode]);

	tegra_pmc_pm_set(mode);

	local_fiq_disable();

	suspend_cpu_complex();
	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		tegra_suspend_enter_lp0();
	case TEGRA_SUSPEND_LP1:
		tegra_suspend_enter_lp1();
		break;
	case TEGRA_SUSPEND_LP2:
		tegra_set_cpu_in_lp2(0);
		break;
	default:
		break;
	}

	if (mode == TEGRA_SUSPEND_LP2)
		cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, &tegra_sleep_cpu);
	else
		cpu_suspend(PHYS_OFFSET - PAGE_OFFSET, &tegra_sleep_core);

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		tegra_suspend_exit_lp0();
	case TEGRA_SUSPEND_LP1:
		tegra_suspend_exit_lp1();
		break;
	case TEGRA_SUSPEND_LP2:
		tegra_clear_cpu_in_lp2(0);
		break;
	default:
		break;
	}

	restore_cpu_complex();

	local_fiq_enable();

	return 0;
}

static const struct platform_suspend_ops tegra_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= tegra_suspend_enter,
};

void __init tegra_init_suspend(struct pmc_pm_data *pdata)
{
	enum tegra_suspend_mode mode;

	if (!pdata || pdata->suspend_mode == TEGRA_SUSPEND_NONE)
		return;

	pmc_pm_data = pdata;
	mode = pdata->suspend_mode;

	if (mode >= TEGRA_SUSPEND_LP1) {
		if (!tegra_lp1_iram_hook() || !tegra_sleep_core_init()) {
			pr_err("%s: unable to allocate memory for SDRMA self-refresh "
			       "-- LP0/LP1 unavailable\n", __func__);
			current_suspend_mode = TEGRA_SUSPEND_LP2;
		}
	}

	tegra_pclk = clk_get_sys(NULL, "pclk");
	BUG_ON(IS_ERR(tegra_pclk));

	current_suspend_mode = pdata->suspend_mode;

	suspend_set_ops(&tegra_suspend_ops);
}

unsigned long debug_uart_port_base;
EXPORT_SYMBOL(debug_uart_port_base);

struct clk *debug_uart_clk;
EXPORT_SYMBOL(debug_uart_clk);

#endif
