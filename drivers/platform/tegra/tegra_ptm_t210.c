/*
 * arch/arm/mach-tegra/tegra_ptm.c
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/sysrq.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <linux/kallsyms.h>
#include <linux/clk.h>
#include <asm/sections.h>
#include <linux/cpu.h>
#include "pm.h"
#include <linux/tegra_ptm.h>
#include <linux/of.h>
#include <linux/delay.h>

/*
_________________________________________________________
|CLUSTER		ATID	Processor	Protocol|
|_______________________________________________________|
|BCCPLEX,Fast Cluster,	0x40	CPU0		ETMv4	|
|Big cluster=Atlas=	0x41	CPU1		ETMv4	|
|Cortex A57		0x42	CPU2		ETMv4	|
|			0x43	CPU3		ETMv4	|
|							|
|LCCPLEX,Slow Cluster,	0x30	CPU0            ETMv4	|
|Little cluster=	0x31	CPU1		ETMv4	|
|Apollo=Cortex A53	0x32	CPU2		ETMv4	|
|			0x33	CPU3		ETMv4	|
|							|
|APE, Cortex A9		0x20	CPU0		PFT1.0	|
|							|
|STM			0x10	NA		MIPI STP|
|_______________________________________________________|

*/

#define CSITE_CLK_HIGH 408000000	/* basically undivided pll_p */
#define CSITE_CLK_LOW  9600000

/* PTM tracer state */
struct tracectx {
	struct device   *dev;
	struct clk	*csite_clk;
	struct mutex    mutex;
	void __iomem    *funnel_major_regs;
	void __iomem    *etf_regs;
	void __iomem    *replicator_regs;
	void __iomem    *etr_regs;
	void __iomem    *funnel_minor_regs;
	void __iomem    *ape_regs;
	void __iomem    *cpu_regs[4];
	void __iomem    *ptm_t210_regs[4];
	void __iomem    *funnel_bccplex_regs;
	int             ptm_t210_regs_count;
	int             cpu_regs_count;
	int             *etf_buf;
	int             etf_size;
	bool            dump_initial_etf;
	unsigned long   start_address_msb;
	unsigned long   start_address_lsb;
	unsigned long   stop_address_msb;
	unsigned long   stop_address_lsb;
	int             enable;
	int             el0_trace;
	int             branch_broadcast;
	int             return_stack;
};

/* initialising some values of the structure */
static struct tracectx tracer = {
	.cpu_regs_count		=	0,
	.ptm_t210_regs_count	=	0,
	.start_address_msb	=	0,
	.start_address_lsb	=	0,
	.stop_address_msb	=	0xFFFFFFFF,
	.stop_address_lsb	=	0xFFFFFFFF,
	.enable			=	0,
	.el0_trace		=	1,
	.branch_broadcast	=	1,
	.return_stack		=	1,
};

static struct clk *pll_p, *clk_m;

static void etf_last_init(struct tracectx *t)
{
	t->dump_initial_etf = true;

	t->etf_buf = devm_kzalloc(t->dev, MAX_ETF_SIZE * sizeof(*t->etf_buf),
					GFP_KERNEL);

	if (NULL == t->etf_buf)
		dev_err(t->dev, "failes to allocate memory to hold ETF\n");
}

static void ape_init(struct tracectx *t)
{
	ape_regs_unlock(t);

	ape_writel(t, 0x500, CORESIGHT_APE_CPU0_ETM_ETMCR_0);

	ape_writel(t, 1, CORESIGHT_APE_CPU0_ETM_ETMTECR1_0);

	ape_writel(t, 0x10, CORESIGHT_APE_CPU0_ETM_ETMTEEVR_0);

	ape_writel(t, 0x80000000, CORESIGHT_APE_CPU0_ETM_ETMACVR1_0);
	ape_writel(t, 0xffffffff, CORESIGHT_APE_CPU0_ETM_ETMACVR2_0);

	ape_writel(t, 0, CORESIGHT_APE_CPU0_ETM_ETMACTR1_0);
	ape_writel(t, 0, CORESIGHT_APE_CPU0_ETM_ETMACTR2_0);

	ape_writel(t, 0xAA, CORESIGHT_APE_CPU0_ETM_ETMATID_0);
}

static void cpu_debug_init(struct tracectx *t, int id)
{
	cpu_debug_regs_unlock(t, id);
	cpu_debug_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_DEBUG_OSLAR_EL1_0);
}

static void ptm_init(struct tracectx *t, int id)
{
	/* Power up the CPU PTM */
	ptm_t210_writel(t, id, 8, CORESIGHT_BCCPLEX_CPU_TRACE_TRCPDCR_0);

	/* Unlock the COU PTM */
	ptm_t210_regs_unlock(t, id);

	/* clear OS lock */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCOSLAR_0);

	/* clear main enable bit to enable register programming */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

	/* Clear uninitialised flopes , else we get event packets */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCEVENTCTL1R_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCEVENTCTL0R_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCTSCTLR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCCCTLR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVISSCTLR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSEQEVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSEQEVR1_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSEQEVR2_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSEQRSTEVR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSEQSTR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCEXTINSELR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCNTRLDVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCNTRLDVR1_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCNTCTLR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCNTCTLR1_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCNTVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCNTVR1_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR2_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR3_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR4_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR5_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR6_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR7_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR8_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR9_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR10_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR11_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR12_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR13_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR14_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR15_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSSCSR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR2_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR3_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR4_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR5_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR6_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR7_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR1_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR2_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR3_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR4_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR5_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR6_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR7_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCIDCVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVMIDCVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCCIDCCTLR0_0);

	/* By default return stack is enabled */
	ptm_t210_writel(t, id, (t->return_stack ? 0x1000 : 0x8),
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCCONFIGR_0);

	/* Enable Periodic Synchronisation after 256 bytes */
	ptm_t210_writel(t, id, 8, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSYNCPR_0);

	/* Branch broadcasting control */
	ptm_t210_writel(t, id, (t->branch_broadcast ? 0x101 : 0x01),
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCBBCTLR_0);

	/* Select Address Range Comparator */
	ptm_t210_writel(t, id, 0x10000,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCSSCCR0_0);

	/* Program Address range comparator. Use 0 and 1 */
	ptm_t210_writel(t, id, t->start_address_msb,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR0_0);
	ptm_t210_writel(t, id, t->start_address_lsb,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR0_0 + 4);
	ptm_t210_writel(t, id, t->stop_address_msb,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR1_0);
	ptm_t210_writel(t, id, t->stop_address_lsb,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR1_0 + 4);

	/* Program the ARC control register */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR0_0);

	/* Program Trace Id register */
	ptm_t210_writel(t, id, (!(is_lp_cluster()) ?
			BCCPLEX_BASE_ID : LCCPLEX_BASE_ID) + id,
			CORESIGHT_BCCPLEX_CPU_TRACE_TRCTRACEIDR_0);

	/* Trace everything with Start/Stop logic started */
	ptm_t210_writel(t, id, (t->el0_trace ? 0xDD0E01 : 0xE01),
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCVICTLR_0);

	/* Enable ViewInst and Include logic for ARC0. Disable the SSC */
	ptm_t210_writel(t, id, 1, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVIIECTLR_0);
}

static void funnel_bccplex_init(struct tracectx *t)
{
	/* unlock BCCPLEX funnel */
	funnel_bccplex_regs_unlock(t);

	/* set up all funnel ports  and set HT as 0x04 */
	funnel_bccplex_writel(t, 0x30F,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
}

static void funnel_major_init(struct tracectx *t)
{
	/* unlock  funnel */
	funnel_major_regs_unlock(t);

	/* enable funnel port 0 and 2  and make HT as 0x04 */
	funnel_major_writel(t, 0x305,
		CORESIGHT_ATBFUNNEL_HUGO_MAJOR_CXATBFUNNEL_REGS_CTRL_REG_0);
}

static void funnel_minor_init(struct tracectx *t)
{
	/* unlock  funnel */
	funnel_minor_regs_unlock(t);

	/* enable funnel port 0 and make HT as 0x04 */
#if 0
	/* this breaks bccplex tracing, need to investigate */
	funnel_minor_writel(t, 0x302,
		CORESIGHT_ATBFUNNEL_MINOR_CXATBFUNNEL_REGS_CTRL_REG_0);
#endif
}

static void etf_init(struct tracectx *t)
{
	/*unlock ETF */
	etf_regs_unlock(t);

	/* Reset RWP and RRP when disabled */
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RWP_0);
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	/* Enabling capturing of trace data and Circular mode */
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_MODE_0);
	etf_writel(t, 0x100, CORESIGHT_ETF_HUGO_CXTMC_REGS_BUFWM_0);

	/* Enabling formatter */
	etf_writel(t, 1, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);

	/* Enable ETF */
	etf_writel(t, 1, CORESIGHT_ETF_HUGO_CXTMC_REGS_CTL_0);
}

static void replicator_init(struct tracectx *t)
{
	/* unlock replicator */
	replicator_regs_unlock(t);

	/* disabling the replicator */
	replicator_writel(t, 0xff ,
	CORESIGHT_ATBREPLICATOR_HUGO_CXATBREPLICATOR_REGISTERS_IDFILTER1_0);
}

static int trace_t210_start(struct tracectx *t)
{
	int id;

	clk_set_parent(t->csite_clk, pll_p);
	clk_set_rate(t->csite_clk, CSITE_CLK_HIGH);

	/* Unlock CPU Debug to monitor PC */
	for (id = 0; id < t->cpu_regs_count; id++)
		cpu_debug_init(t, id);

	/* Programming PTM */
	for (id = 0; id < t->ptm_t210_regs_count; id++)
		ptm_init(t, id);

	/* Programming APE */
	ape_init(t);

	/* Programming BCCPLEX funnel */
	funnel_bccplex_init(t);

	/* Programming major funnel */
	funnel_major_init(t);

	/* Programming minor funnel */
	funnel_minor_init(t);

	/* programming the ETF */
	etf_init(t);

	/* programming the replicator */
	replicator_init(t);

	/* Enabling the ETM */
	for (id = 0; id < t->ptm_t210_regs_count; id++)
		ptm_t210_writel(t, id, 1,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

	ape_writel(t, 0x100, CORESIGHT_APE_CPU0_ETM_ETMCR_0);

	return 0;
}

static int trace_t210_stop(struct tracectx *t)
{
	int id;

	/* Disabling the ETM */
	for (id = 0; id < t->ptm_t210_regs_count; id++)
		ptm_t210_writel(t, id, 0,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

	ape_writel(t, 0x500, CORESIGHT_APE_CPU0_ETM_ETMCR_0);

	clk_set_parent(t->csite_clk, clk_m);
	clk_set_rate(t->csite_clk, CSITE_CLK_LOW);

	return 0;
}

static ssize_t etf_read(struct file *file, char __user *data,
	size_t len, loff_t *ppos)
{
	struct tracectx *t = file->private_data;
	loff_t pos = *ppos;

	if ((pos + len) >= t->etf_size)
		len = t->etf_size - pos;

	if ((len > 0) && copy_to_user(data, (u8 *) t->etf_buf+pos, len))
		return -EFAULT;
	else {
		*ppos = pos + len;
		return len;
	}
}

static ssize_t etf_fill_buf(struct tracectx *t)
{
	int rrp, rwp, rwp32, rrp32, max, count = 0, serial, overflow;

	if (!t->etf_regs)
		return -EINVAL;

	mutex_lock(&t->mutex);

	etf_regs_unlock(t);

	/* Manual flush and stop */
	etf_writel(t, 0x1001, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
	etf_writel(t, 0x1041, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);

	udelay(1000);

	/* Disable ETF */
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_CTL_0);

	/* Get etf data and Check for overflow */
	overflow = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);

	overflow &= 0x01;

	/* Check for data in ETF */
	rwp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RWP_0);
	rrp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	rwp32 = rwp/4;
	rrp32 = rrp/4;

	max = rwp32;
	serial = 0;

	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	trace_t210_stop(t);

	if (!overflow) {
		t->etf_size = max * 4;
		for (count = 0; count < max; count++) {
			t->etf_buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
	} else {
		t->etf_size = MAX_ETF_SIZE * 4;
		/* ETF overflow..the last 4K entries are printed */
		etf_writel(t, rwp, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		/* Read from rwp to end of RAM */
		for (count = rwp32; count < MAX_ETF_SIZE; count++) {
			t->etf_buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
		/* Rollover the RRP and read till rwp-1 */
		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		for (count = 0; count < max; count++) {
			t->etf_buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
	}
	etf_regs_lock(t);
	mutex_unlock(&t->mutex);

	return t->etf_size;
}

static int etf_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tracectx *t = dev_get_drvdata(miscdev->parent);

	if (NULL == t->etf_regs)
		return -ENODEV;

	file->private_data = t;

	etf_fill_buf(t);

	return nonseekable_open(inode, file);
}

static int etf_release(struct inode *inode, struct file *file)
{
	/* there's nothing to do here, actually */
	return 0;
}

static void etf_save_last(struct tracectx *t)
{
	int rrp, rwp, rwp32, rrp32, max, count = 0, serial, overflow;

	BUG_ON(!t->dump_initial_etf);

	etf_regs_unlock(t);

	/* Manual flush and stop */
	etf_writel(t, 0x1001, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
	etf_writel(t, 0x1041, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);

	udelay(1000);

	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_CTL_0);

	/* Get etf data and Check for overflow */
	overflow = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);

	overflow &= 0x01;

	/* Check for data in ETF */
	rwp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RWP_0);
	rrp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	rwp32 = rwp/4;
	rrp32 = rrp/4;

	max = rwp32;
	serial = 0;

	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	pr_info(" RRP32_L %08x ,RWP32_L %08x\n ", rrp32, rwp32);

	if (!overflow) {
		pr_info("ETF not overflown. Reading contents to userspace\n");
		t->etf_size = max * 4;
		for (count = 0; count < max; count++)
			t->etf_buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
	} else {
		pr_info("ETF overflown. Reading contents to userspace\n");
		t->etf_size = MAX_ETF_SIZE * 4;
		/* ETF overflow..the last 4K entries are printed */
		etf_writel(t, rwp, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		/* Read from rwp to end of RAM */
		for (count = rwp32; count < MAX_ETF_SIZE; count++) {
			t->etf_buf[serial] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
		/* Rollover the RRP and read till rwp-1 */
		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		for (count = 0; count < max; count++) {
			t->etf_buf[serial] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
	}
	etf_regs_lock(t);
}

static ssize_t last_etf_read(struct file *file, char __user *data,
	size_t len, loff_t *ppos)
{
	struct tracectx *t = file->private_data;
	size_t ret;
	int i;

	mutex_lock(&t->mutex);

	for (i = 0; i < t->etf_size/4; i++)
		pr_info("COUNT: %08x ETF DATA: %08x\n",
						i, t->etf_buf[i]);

	if (copy_to_user(data, (char *) t->etf_buf , t->etf_size)) {
		ret = -EFAULT;
		goto out;
	}
	*ppos += t->etf_size;
	ret = 0;

out:
	mutex_unlock(&t->mutex);
	return ret;
}

/* use a sysfs file "trace_enable" to start/stop tracing */
static ssize_t trace_enable_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%x\n", tracer.enable);
}

/* use a sysfs file "trace_el0" to start/stop userspace tracing */
static ssize_t trace_el0_trace_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%x\n", tracer.el0_trace);
}

/* use a sysfs file "trace_branch_broadcast" to start/stop branch broadcast */
static ssize_t trace_branch_broadcast_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%x\n", tracer.branch_broadcast);
}

/* use a sysfs file "trace_running" to start/stop return stack mode */
static ssize_t trace_return_stack_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%x\n", tracer.return_stack);
}

/* use a sysfs file "trace_range_address" to allow user to
   specify specific areas of code to be traced */
static ssize_t trace_range_address_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%08lx %08lx %08lx %08lx\n",
			tracer.start_address_msb, tracer.stop_address_lsb,
			tracer.stop_address_msb, tracer.stop_address_lsb);
}

static ssize_t trace_enable_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	unsigned int value;
	int ret;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	if (!tracer.etf_regs)
		return -ENODEV;

	mutex_lock(&tracer.mutex);

	if (value != 0) {
		ret = trace_t210_start(&tracer);
		tracer.enable = 1;
	} else {
		ret = trace_t210_stop(&tracer);
		tracer.enable = 0;
	}

	mutex_unlock(&tracer.mutex);

	return ret ? : n;
}

static ssize_t trace_el0_trace_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	unsigned int el0_trace;

	if (sscanf(buf, "%u", &el0_trace) != 1)
		return -EINVAL;

	mutex_lock(&tracer.mutex);

	if (el0_trace == 0)
		tracer.el0_trace = 0;

	mutex_unlock(&tracer.mutex);

	return n;
}

static ssize_t trace_branch_broadcast_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	unsigned int branch_broadcast;

	if (sscanf(buf, "%u", &branch_broadcast) != 1)
		return -EINVAL;

	mutex_lock(&tracer.mutex);

	if (branch_broadcast == 0)
		tracer.branch_broadcast = 0;

	mutex_unlock(&tracer.mutex);

	return n;
}

static ssize_t trace_return_stack_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	unsigned int return_stack;

	if (sscanf(buf, "%u", &return_stack) != 1)
		return -EINVAL;

	mutex_lock(&tracer.mutex);

	if (return_stack == 0)
		tracer.return_stack = 0;

	mutex_unlock(&tracer.mutex);

	return n;
}

static ssize_t trace_range_address_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	char const *buf, size_t n)
{
	unsigned int start_address_msb, start_address_lsb,
				stop_address_msb, stop_address_lsb;

	if (sscanf(buf, "%ul %ul %ul %ul", &start_address_msb,
			&start_address_lsb, &stop_address_msb,
			&stop_address_lsb) != 4)
		return -EINVAL;

	mutex_lock(&tracer.mutex);

	if (start_address_msb < 0 || start_address_lsb < 0 ||
		start_address_msb > 0xFFFFFFFF || stop_address_msb < 0 ||
		start_address_lsb > 0xFFFFFFFF || stop_address_lsb < 0 ||
		stop_address_msb > 0xFFFFFFFF || stop_address_lsb > 0xFFFFFFFF)
		pr_err("ptm address range invalid\n");
	else {
		tracer.start_address_msb = start_address_msb;
		tracer.start_address_lsb = start_address_lsb;
		tracer.stop_address_msb  = stop_address_msb;
		tracer.stop_address_lsb	 = stop_address_lsb;
	}

	mutex_unlock(&tracer.mutex);

	return n;
}

#define A(a, b, c, d)   __ATTR(trace_##a, b, \
	trace_##c##_show, trace_##d##_store)
static const struct kobj_attribute trace_attr[] = {
	A(enable,		0644,	enable,		enable),
	A(el0_trace,		0644,	el0_trace,	el0_trace),
	A(branch_broadcast,	0644,	branch_broadcast,
						branch_broadcast),
	A(return_stack,		0644,	return_stack,
						return_stack),
	A(range_address,	0644,	range_address,
						range_address)
};

static const struct file_operations etf_fops = {
	.owner = THIS_MODULE,
	.read = etf_read,
	.open = etf_open,
	.release = etf_release,
	.llseek = no_llseek,
};

static const struct file_operations last_etf_fops = {
	.owner = THIS_MODULE,
	.read = last_etf_read,
	.open = etf_open,
	.release = etf_release,
	.llseek = no_llseek,
};

static struct miscdevice etf_miscdev = {
	.name = "etf",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &etf_fops,
};

static struct miscdevice last_etf_miscdev = {
	.name = "last_etf",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &last_etf_fops,
};

static void etf_nodes(struct tracectx *t)
{
	int ret = 0;

	etf_miscdev.parent = t->dev;
	ret = misc_register(&etf_miscdev);
	if (ret)
		dev_err(t->dev, "failes to register /dev/etf\n");

	last_etf_miscdev.parent = t->dev;
	ret = misc_register(&last_etf_miscdev);
	if (ret)
		dev_err(t->dev, "failes to register /dev/last_etf\n");

	dev_info(t->dev, "ETF is initialized.\n");
}

static int ptm_probe(struct platform_device  *dev)
{
	int i, ret = 0;

	struct tracectx *t = &tracer;

	mutex_lock(&t->mutex);
	t->dev = &dev->dev;
	platform_set_drvdata(dev, t);

	for (i = 0; i < dev->num_resources; i++) {
		struct resource *res;
		void __iomem *addr;

		res = platform_get_resource(dev, IORESOURCE_MEM, i);
		if (NULL == res)
			goto out;
		addr = devm_ioremap_nocache(&dev->dev, res->start,
				resource_size(res));
		if (NULL == addr)
			goto out;
		if (dev->dev.of_node) {
			switch (i) {
			case 0:
				t->funnel_major_regs = addr;
				break;
			case 1:
				t->etf_regs = addr;
				break;
			case 2:
				t->replicator_regs = addr;
				break;
			case 3:
				t->etr_regs = addr;
				break;
			case 4:
				t->funnel_bccplex_regs = addr;
				break;
			case 5:
			case 6:
			case 7:
			case 8:
				t->cpu_regs[t->cpu_regs_count] = addr;
				t->cpu_regs_count++;
				break;
			case 9:
			case 10:
			case 11:
			case 12:
				t->ptm_t210_regs[t->ptm_t210_regs_count] = addr;
				t->ptm_t210_regs_count++;
				break;
			case 13:
				t->funnel_minor_regs = addr;
				break;
			case 14:
				t->ape_regs = addr;
				break;
			default:
				dev_err(&dev->dev, "unknown resource for the PTM device\n");
				break;
		}
		} else {
			if (0 == strncmp("funnel_major", res->name, 12))
				t->funnel_major_regs = addr;
			if (0 == strncmp("etf", res->name, 3))
				t->etf_regs = addr;
			if (0 == strncmp("replicator", res->name, 3))
				t->replicator_regs = addr;
			if (0 == strncmp("etr", res->name, 3))
				t->etr_regs = addr;
			if (0 == strncmp("funnel_bccplex", res->name, 14))
				t->funnel_bccplex_regs = addr;
			if (0 == strncmp("cpu", res->name, 3)) {
				t->cpu_regs[t->cpu_regs_count] = addr;
				t->cpu_regs_count++;
			}
			if (0 == strncmp("ptm", res->name, 3)) {
				t->ptm_t210_regs[t->ptm_t210_regs_count] = addr;
				t->ptm_t210_regs_count++;
			}
			if (0 == strncmp("funnel_minor", res->name, 12))
				t->funnel_minor_regs = addr;
			if (0 == strncmp("ape", res->name, 3))
				t->ape_regs = addr;
		}
	}

	/* Configure Coresight Clocks */
	t->csite_clk = clk_get_sys("csite", NULL);
	pll_p = clk_get_sys(NULL, "pll_p");
	clk_m = clk_get_sys(NULL, "clk_m");

	/* Initialise data structures required for saving trace after reset */
	etf_last_init(t);

	/* save ETF contents to DRAM when system is reset */
	etf_save_last(t);

	/* initialising dev nodes */
	etf_nodes(t);

	/* create sysfs */
	for (i = 0; i < ARRAY_SIZE(trace_attr); i++) {
		ret = sysfs_create_file(&dev->dev.kobj, &trace_attr[i].attr);
		if (ret)
			dev_err(&dev->dev, "failed to create %s\n",
				trace_attr[i].attr.name);
	}

	dev_info(&dev->dev, "PTM driver initialized.\n");

out:
	if (ret)
		dev_err(&dev->dev, "Failed to start the PTM device\n");
	mutex_unlock(&t->mutex);
	return ret;
}

static int ptm_remove(struct platform_device *dev)
{
	struct tracectx *t = &tracer;
	int i;

	for (i = 0; i < ARRAY_SIZE(trace_attr); i++)
		sysfs_remove_file(&dev->dev.kobj, &trace_attr[i].attr);

	mutex_lock(&t->mutex);

	for (i = 0; i < t->ptm_t210_regs_count; i++)
		devm_iounmap(&dev->dev, t->ptm_t210_regs[i]);
	for (i = 0; i < t->cpu_regs_count; i++)
		devm_iounmap(&dev->dev, t->cpu_regs[i]);
	devm_iounmap(&dev->dev, t->funnel_major_regs);
	devm_iounmap(&dev->dev, t->funnel_minor_regs);
	devm_iounmap(&dev->dev, t->ape_regs);
	devm_iounmap(&dev->dev, t->etr_regs);
	devm_iounmap(&dev->dev, t->funnel_bccplex_regs);
	devm_iounmap(&dev->dev, t->replicator_regs);
	t->etf_regs = NULL;

	mutex_unlock(&t->mutex);

	return 0;
}

static struct of_device_id ptm_of_match[] = {
	    { .compatible = "nvidia,ptm", },
	    {   },
};

MODULE_DEVICE_TABLE(of, ptm_of_match)

static struct platform_driver ptm_driver = {
	.probe          = ptm_probe,
	.remove         = ptm_remove,
	.driver         = {
		.name   = "ptm",
		.of_match_table = of_match_ptr(ptm_of_match),
		.owner  = THIS_MODULE,
	},
};

static int __init tegra_ptm_driver_init(void)
{
	int retval;

	mutex_init(&tracer.mutex);
	retval = platform_driver_register(&ptm_driver);
	if (retval) {
		pr_err("Failed to probe ptm\n");
		return retval;
	}

	return 0;
}
device_initcall(tegra_ptm_driver_init);

