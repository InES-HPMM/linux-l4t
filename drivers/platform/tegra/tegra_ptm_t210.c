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
#include <linux/dma-mapping.h>

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
	struct device	*dev;
	struct clk	*csite_clk;
	struct mutex	mutex;
	void __iomem	*funnel_major_regs;
	void __iomem	*etf_regs;
	void __iomem	*replicator_regs;
	void __iomem	*etr_regs;
	void __iomem	*funnel_minor_regs;
	void __iomem	*ape_regs;
	void __iomem	*ptm_t210_regs[CONFIG_NR_CPUS];
	void __iomem	*funnel_bccplex_regs;
	uintptr_t	*etr_address;
	int		*trc_buf;
	int		buf_size;
	uintptr_t	start_address;
	uintptr_t	stop_address;

	unsigned long	enable;
	unsigned long	userspace;
	unsigned long	branch_broadcast;
	unsigned long	return_stack;
	unsigned long	formatter;
	unsigned long	timestamp;
	unsigned long	etr;
	unsigned long	ape;

	int		trigger;
	uintptr_t	trigger_address;
	unsigned int	percent_after_trigger;
	unsigned int	cycle_count;
	int		dram_carveout_kb;
};

/* initialising some values of the structure */
static struct tracectx tracer = {
	.start_address			=	0,
	.stop_address			=	0xFFFFFFC100000000,
	.enable				=	0,
	.userspace			=	0,
	.branch_broadcast		=	1,
	.return_stack			=	0,
	.trigger			=	0,
	.percent_after_trigger		=	25,
	.formatter			=	1,
	.timestamp			=	0,
	.cycle_count			=	0,
	.trigger_address		=	0,
	.etr			=	0,
	.ape				=	0,
	.dram_carveout_kb		=	DRAM_CARVEOUT_MB * 1024,
};

static struct clk *csite_clk;

/* Allocate a buffer to hold the ETF traces */
static void trc_buf_init(struct tracectx *t)
{
	t->trc_buf = devm_kzalloc(t->dev, MAX_ETF_SIZE * sizeof(*t->trc_buf),
					GFP_KERNEL);

	if (NULL == t->trc_buf)
		dev_err(t->dev, "failes to allocate memory to hold ETF\n");
}

/* Initialise the APE registers */
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

	ape_writel(t, 0x20, CORESIGHT_APE_CPU0_ETM_ETMATID_0);

	dev_dbg(t->dev, "APE is initialized.\n");
}

/* Initialise the PTM registers */
static void ptm_init(struct tracectx *t, int id)
{
	int trcstat_idle, trccfg;

	/* Power up the CPU PTM */
	ptm_t210_writel(t, id, 8, CORESIGHT_BCCPLEX_CPU_TRACE_TRCPDCR_0);

	/* Unlock the COU PTM */
	ptm_t210_regs_unlock(t, id);

	/* clear OS lock */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCOSLAR_0);

	/* clear main enable bit to enable register programming */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

	trcstat_idle = 0x0;
	while (trcstat_idle == 0x0) { /* wait till IDLE bit is set */
		/*Read the IDLE bit in CORESIGHT_BCCPLEX_CPU0_TRACE_TRC_0 */
		trcstat_idle = ptm_t210_readl(t, id,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCSTAT_0);
		trcstat_idle = trcstat_idle & 0x1;
	}

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
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR1_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR2_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR3_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR4_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR5_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR6_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR7_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR0_0);
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
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVIIECTLR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCBBCTLR_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSTALLCTLR_0);

	/* Configure basic controls RS, BB, TS, VMID, Context tracing */
	trccfg = ptm_t210_readl(t, id,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCCONFIGR_0);

	if (t->branch_broadcast) {
		trccfg = trccfg | (1<<3);
		ptm_t210_writel(t, id, 0x101,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCBBCTLR_0);
	}
	if (t->return_stack)
		trccfg = trccfg | (1<<12);
	if (t->timestamp)
		trccfg = trccfg | (1<<11);
	if (t->cycle_count) {
		trccfg = trccfg | (1<<4);
		ptm_t210_writel(t, id, t->cycle_count,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCCCCTLR_0);
	}
	ptm_t210_writel(t, id, trccfg,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCCONFIGR_0);

	/* Enable Periodic Synchronisation after 1024 bytes */
	ptm_t210_writel(t, id, 0xa, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSYNCPR_0);

	/* Program Trace ID register */
	ptm_t210_writel(t, id, (!(is_lp_cluster()) ?
			BCCPLEX_BASE_ID : LCCPLEX_BASE_ID) + id,
			CORESIGHT_BCCPLEX_CPU_TRACE_TRCTRACEIDR_0);

	/* Program Address range comparator. Use 0 and 1 */
	ptm_t210_writeq(t, id, t->start_address,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR0_0);
	ptm_t210_writeq(t, id, t->stop_address,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR1_0);

	/* control user space tracing */
	ptm_t210_writel(t, id, (t->userspace ? 0x0 : 0x1000),
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR0_0);
	ptm_t210_writel(t, id, (t->userspace ? 0x0 : 0x1000),
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR1_0);

	/* Select Resource 2 to include ARC0 */
	ptm_t210_writel(t, id, 0x50001,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR2_0);

	/* Enable ViewInst and Include logic for ARC0. Disable the SSC */
	ptm_t210_writel(t, id, 1, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVIIECTLR_0);

	/* Select Start/Stop as Started. And select Resource 2 as the Event */
	ptm_t210_writel(t, id, 0xE02, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVICTLR_0);

	/* Trace everything till Address match packet hits. Use Single Shot
	comparator and generate Event Packet */
	if (t->trigger) {
		/* SAC2 for Address Match on which to trigger */
		ptm_t210_writeq(t, id, t->trigger_address,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR2_0);
		ptm_t210_writel(t, id, t->userspace ? 0x0 : 0x1000,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR2_0);
		/* Select SAC2 for SSC0 and enable SSC to re-fire on match */
		ptm_t210_writel(t, id, 0x4,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCSSCCR0_0);
		/* Clear SSC Status bit 31 */
		ptm_t210_writel(t, id, 0x0,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCSSCSR0_0);
		/* Select Resource 3 to include SSC0 */
		ptm_t210_writel(t, id, 0x30001,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCRSCTLR3_0);
		/* Select Resource3 as part of Event0 */
		ptm_t210_writel(t, id, 0x3,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCEVENTCTL0R_0);
		/* Insert ATB packet with ATID=0x7D and Payload=Master ATID.
		ETF can use this to Stop and flush based on this */
		ptm_t210_writel(t, id, 0x800,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCEVENTCTL1R_0);
		if (t->timestamp)
			/* insert global timestamp if enabled */
			ptm_t210_writel(t, id, 0x3,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCTSCTLR_0);
	}
	smp_wmb();
	dev_dbg(t->dev, "PTM%d initialized.\n", id);
}

/* Initialise the funnel bccplex registers */
static void funnel_bccplex_init(struct tracectx *t)
{
	/* unlock BCCPLEX funnel */
	funnel_bccplex_regs_unlock(t);

	/* set up all funnel ports  and set HT as 0x04 */
	funnel_bccplex_writel(t, 0x30F,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);

	dev_info(t->dev, "Funnel bccplex initialized.\n");
}

/* Initialise the funnel major registers */
static void funnel_major_init(struct tracectx *t)
{
	/* unlock  funnel */
	funnel_major_regs_unlock(t);

	/* enable funnel port 0 and 2  and make HT as 0x04 */
	funnel_major_writel(t, 0x305,
		CORESIGHT_ATBFUNNEL_HUGO_MAJOR_CXATBFUNNEL_REGS_CTRL_REG_0);

	dev_info(t->dev, "Funnel Major initialized.\n");
}

/* Initialise the funnel minor registers */
static void funnel_minor_init(struct tracectx *t)
{
	/* unlock  funnel */
	funnel_minor_regs_unlock(t);

	/* enable funnel port 0 and make HT as 0x04 */
	funnel_minor_writel(t, 0x302,
		CORESIGHT_ATBFUNNEL_MINOR_CXATBFUNNEL_REGS_CTRL_REG_0);

	dev_dbg(t->dev, "Funnel Minor initialized.\n");
}

/* Initialize the ETF registers */
static void etf_init(struct tracectx *t)
{
	int ffcr, words_after_trigger;

	/*unlock ETF */
	etf_regs_unlock(t);

	/* Reset RWP and RRP when disabled */
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RWP_0);
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	/* Enabling capturing of trace data and Circular mode */

	if (t->etr) {
		etf_writel(t, 2, CORESIGHT_ETF_HUGO_CXTMC_REGS_MODE_0);
		etf_writel(t, 1, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
	} else {
		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_MODE_0);
		if (t->trigger) {
			/* Capture data for specified cycles after TRIGIN */
			words_after_trigger = (t->percent_after_trigger
					* MAX_ETF_SIZE) / 100;
			etf_writel(t, words_after_trigger,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_TRG_0);
			/* Flush on Trigin, insert trigger and stop on
							Flush Complete */
			etf_writel(t, 0x1120,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
		}
		if ((!t->formatter)) {
			ffcr = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
			ffcr &= ~1;
			etf_writel(t, ffcr,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
		} else {
			ffcr = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
			ffcr |= 1;
			etf_writel(t, ffcr,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
		}
	}
	/* Enable ETF */
	etf_writel(t, 1, CORESIGHT_ETF_HUGO_CXTMC_REGS_CTL_0);
	dev_info(t->dev, "ETF initialized.\n");
}

/* Initialise the replicator registers*/
static void replicator_init(struct tracectx *t)
{
	/* unlock replicator */
	replicator_regs_unlock(t);

	/* disabling the replicator */
	replicator_writel(t, 0xff ,
	CORESIGHT_ATBREPLICATOR_HUGO_CXATBREPLICATOR_REGISTERS_IDFILTER1_0);

	dev_dbg(t->dev, "Replicator initialized.\n");
}

/* Initialise the ETR registers */
static void etr_init(struct tracectx *t)
{
	int size_dwords, words_after_trigger, ffcr;
	dma_addr_t dma_handle;

	etr_regs_unlock(t);

	size_dwords = (t->dram_carveout_kb * 1024)/4;

	etr_writel(t, size_dwords, CORESIGHT_ETR_HUGO_CXTMC_REGS_RSZ_0);
	etr_writel(t, 0, CORESIGHT_ETR_HUGO_CXTMC_REGS_MODE_0);
	etr_writel(t, 0x300, CORESIGHT_ETR_HUGO_CXTMC_REGS_AXICTL_0);

	if (!t->etr_address)
		t->etr_address = dma_zalloc_coherent(t->dev,
				t->dram_carveout_kb, &dma_handle, GFP_KERNEL);

	/*allocate space for ETR */
	etr_writel(t, (uintptr_t)t->etr_address,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_DBALO_0);
	etr_writel(t, (uintptr_t)t->etr_address + t->dram_carveout_kb * 1024,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_DBAHI_0);

	if (t->trigger) {
		/* Capture data for specified cycles after seeing the TRIGIN */
		words_after_trigger = t->percent_after_trigger *
						size_dwords / 100 ;
		etf_writel(t, words_after_trigger,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_TRG_0);
		/* Flush on Trigin, insert trigger and Stop on Flush Complete */
		etf_writel(t, 0x1120, CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
	}
	if ((!t->formatter)) {
		ffcr = etr_readl(t, CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
		ffcr &= ~1;
		etr_writel(t, ffcr, CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
	} else {
		ffcr = etr_readl(t, CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
		ffcr |= 1;
		etr_writel(t, ffcr, CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
	}

	/* Enable ETF */
	etr_writel(t, 1, CORESIGHT_ETR_HUGO_CXTMC_REGS_CTL_0);
	dev_dbg(t->dev, "ETR initialized.\n");
}

/* This function enables PTM traces */
static int trace_t210_start(struct tracectx *t)
{
	int id;

	clk_enable(csite_clk);

	/* Programming PTM */
	for_each_online_cpu(id)
		ptm_init(t, id);

	if (t->ape) {
		/* Programming APE */
		ape_init(t);
		/* Programming minor funnel */
		funnel_minor_init(t);
		/* Enable APE traces */
		ape_writel(t, 0x100, CORESIGHT_APE_CPU0_ETM_ETMCR_0);
	} else {
		/* Programming BCCPLEX funnel */
		funnel_bccplex_init(t);

		/* Programming major funnel */
		funnel_major_init(t);

		/* programming the ETF */
		etf_init(t);

		if (t->etr)
			/* Program the ETR */
			etr_init(t);

		/* programming the replicator */
		replicator_init(t);

		/* Enabling the ETM */
		for_each_online_cpu(id) {
			ptm_t210_writel(t, id, 1,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);
			smp_wmb();
		}
	}

	return 0;
}

/* Disable the traces */
static int trace_t210_stop(struct tracectx *t)
{
	int id;

	if (!t->ape)
		/* Disabling the ETM */
		for_each_online_cpu(id)
			ptm_t210_writel(t, id, 0,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);
	else
		/* Disable APE traces */
		ape_writel(t, 0x500, CORESIGHT_APE_CPU0_ETM_ETMCR_0);

	clk_disable(csite_clk);

	return 0;
}

/* This function transfers the traces from kernel space to user space */
static ssize_t trc_read(struct file *file, char __user *data,
	size_t len, loff_t *ppos)
{
	struct tracectx *t = file->private_data;
	loff_t pos = *ppos;

	if ((pos + len) >= t->buf_size)
		len = t->buf_size - pos;

	if ((len > 0) && copy_to_user(data, (u8 *) t->trc_buf+pos, len))
		return -EFAULT;
	else {
		*ppos = pos + len;
		return len;
	}
}

/* this function copies traces from the ETF to an array */
static ssize_t trc_fill_buf(struct tracectx *t)
{
	int rwp, rwp32, max, count = 0, serial, overflow;
	int trig_stat, tmcready, top_of_buffer;

	if (!t->etf_regs)
		return -EINVAL;

	if (!t->etr_regs)
		return -EINVAL;

	mutex_lock(&t->mutex);
	clk_enable(csite_clk);
	etf_regs_unlock(t);
	etr_regs_unlock(t);

	if (t->trigger_address) {
		if (t->etr == 1) {
			trig_stat = 0;
			while (trig_stat == 0) {
				trig_stat = etr_readl(t,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_STS_0);
				trig_stat = (trig_stat >> 1) & 0x1;
			}
			tmcready = 0;
			while (tmcready == 0) {
				trig_stat = etr_readl(t,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_STS_0);
				tmcready = (trig_stat >> 2) & 0x1;
			}
		} else {
			trig_stat = 0;
			while (trig_stat == 0) {
				trig_stat = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);
				trig_stat = (trig_stat >> 1) & 0x1;
			}
			tmcready = 0;
			while (tmcready == 0) {
				trig_stat = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);
				tmcready = (trig_stat >> 2) & 0x1;
			}
		}
	} else {
		if (t->etr == 1) {
			/* Manual flush and stop */
			etr_writel(t, 0x1001,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
			etr_writel(t, 0x1041,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_FFCR_0);
			tmcready = 0;
			while (tmcready == 0) {
				trig_stat = etr_readl(t,
					CORESIGHT_ETR_HUGO_CXTMC_REGS_STS_0);
				tmcready = (trig_stat >> 2) & 0x1;
			}
		} else {
			/* Manual flush and stop */
			etf_writel(t, 0x1001,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
			etf_writel(t, 0x1041,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
			tmcready = 0;
			while (tmcready == 0) {
				trig_stat = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);
				tmcready = (trig_stat >> 2) & 0x1;
			}
		}
	}

	/* Disable ETF */
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_CTL_0);

	if (t->etr == 1) {
		etr_writel(t, 0, CORESIGHT_ETR_HUGO_CXTMC_REGS_CTL_0);

		rwp = etr_readl(t, CORESIGHT_ETR_HUGO_CXTMC_REGS_RWP_0);
		 /* Get etf data and Check for overflow */
		overflow = etr_readl(t, CORESIGHT_ETR_HUGO_CXTMC_REGS_STS_0);
		overflow &= 0x01;
		rwp32 = rwp/4;
		max = rwp;
		top_of_buffer = (uintptr_t)t->etr_address;
		serial = 0;

		/* Disabling the ETM */
		for_each_online_cpu(count)
			ptm_t210_writel(t, count, 0,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

		ape_writel(t, 0x500, CORESIGHT_APE_CPU0_ETM_ETMCR_0);

		t->buf_size = max - (uintptr_t)t->etr_address;
		if (overflow) {
			t->buf_size = top_of_buffer - rwp;
			for (count = rwp; count < top_of_buffer; count += 4) {
				t->trc_buf[count] = etf_readl(t,
				CORESIGHT_ETR_HUGO_CXTMC_REGS_RWP_0);
			serial += 1;
			}
		}
		for (count = (uintptr_t)t->etr_address;
						count < max; count += 4) {
			t->trc_buf[count] = etf_readl(t,
				CORESIGHT_ETR_HUGO_CXTMC_REGS_RWP_0);
			serial += 1;
		}
	} else {
		/* Check for data in ETF */
		rwp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RWP_0);

		/* Get etf data and Check for overflow */
		overflow = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);
		overflow &= 0x1;

		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

		/* Disabling the ETM */
		for_each_online_cpu(count)
			ptm_t210_writel(t, count, 0,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

		ape_writel(t, 0x500, CORESIGHT_APE_CPU0_ETM_ETMCR_0);

		if (overflow) {
			t->buf_size = MAX_ETF_SIZE * 4;
			/* ETF overflow..the last 4K entries are printed */
			etf_writel(t, rwp, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
			/* Read from rwp to end of RAM */
			rwp >>= 2;
			for (count = rwp; count < MAX_ETF_SIZE - 1; count++) {
				t->trc_buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			}
		}
		/* Rollover the RRP and read till rwp-1 */
		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		for (count = 0; count < rwp-1; count++) {
			t->trc_buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
		}
	}
	etf_regs_lock(t);
	etr_regs_lock(t);
	clk_disable(csite_clk);
	mutex_unlock(&t->mutex);

	return t->buf_size;
}

static int trc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tracectx *t = dev_get_drvdata(miscdev->parent);

	if (NULL == t->etf_regs)
		return -ENODEV;

	if (NULL == t->etr_regs)
		return -ENODEV;

	file->private_data = t;

	trc_fill_buf(t);

	return nonseekable_open(inode, file);
}

static int trc_release(struct inode *inode, struct file *file)
{
	/* there's nothing to do here, actually */
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static int ptm_cpu_hotplug_notifier(struct notifier_block *self,
	unsigned long action, void *hcpu)
{
	long cpu = (long)hcpu;
	switch (action) {
	case CPU_STARTING:
		if (tracer.enable) {
			ptm_init(&tracer, cpu);
			ptm_t210_writel(&tracer, cpu, 1,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);
		}
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

/* configure per-CPU trace unit in hotplug path */
static struct notifier_block ptm_cpu_hotplug_notifier_block = {
	.notifier_call = ptm_cpu_hotplug_notifier,
};
#endif

/* use a sysfs file "trace_range_address" to allow user to
   specify specific areas of code to be traced */
static ssize_t trace_range_address_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "start address : %16lx\n stop address : %16lx\n",
			tracer.start_address, tracer.stop_address);
}

/* use a sysfs file "trace_address_trigger" to allow user to
   specify specific areas of code to be traced */
static ssize_t trace_trigger_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	if (tracer.trigger)
		return sprintf(buf, "Address match is enabled\n"
			"Address trigger at %16lx\n", tracer.trigger_address);
	else
		return sprintf(buf, "Address match is disabled\n");
}

/* use a sysfs file "trace_cycle_count" to allow/disallow cycle count
   to be captured */
static ssize_t trace_cycle_count_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	if (tracer.cycle_count)
		return sprintf(buf, "cycle count threshold: %x",
				tracer.cycle_count);
	else
		return sprintf(buf, "cycle accurate is disabled");
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

	if (value) {
		ret = trace_t210_start(&tracer);
		tracer.enable = 1;
	} else {
		ret = trace_t210_stop(&tracer);
		tracer.enable = 0;
	}

	mutex_unlock(&tracer.mutex);

	return ret ? : n;
}

static ssize_t trace_range_address_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	char const *buf, size_t n)
{
	uintptr_t start_address, stop_address;

	if (sscanf(buf, "%lx %lx", &start_address, &stop_address) != 2)
		return -EINVAL;

	if (start_address >= stop_address)
		return -EINVAL;

	mutex_lock(&tracer.mutex);

	tracer.start_address= start_address;
	tracer.stop_address = stop_address;

	mutex_unlock(&tracer.mutex);

	return n;
}

static ssize_t trace_trigger_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	unsigned int trigger, percent_after_trigger;
	uintptr_t trigger_address;

	if (sscanf(buf, "%u %lx %u", &trigger, &trigger_address,
				&percent_after_trigger) != 3)
		return -EINVAL;

	if ( percent_after_trigger > 100)
		percent_after_trigger = 100;

	mutex_lock(&tracer.mutex);

	if (trigger) {
		tracer.trigger = 1;
		tracer.trigger_address = trigger_address;
		tracer.percent_after_trigger = percent_after_trigger;
	} else
		tracer.trigger = 0;

	mutex_unlock(&tracer.mutex);

	return n;
}

static ssize_t trace_cycle_count_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	unsigned int cycle_count;

	if (sscanf(buf, "%u", &cycle_count) != 1)
		return -EINVAL;

	mutex_lock(&tracer.mutex);

	tracer.cycle_count = cycle_count;

	mutex_unlock(&tracer.mutex);

	return n;
}

#define define_show_state_func(_name) \
static ssize_t trace_##_name##_show(struct kobject *kobj, \
				struct kobj_attribute *attr, \
				char *buf) \
{ \
	return sprintf(buf, "%lu\n", tracer._name); \
}

define_show_state_func(enable)
define_show_state_func(userspace)
define_show_state_func(branch_broadcast)
define_show_state_func(return_stack)
define_show_state_func(timestamp)
define_show_state_func(formatter)
define_show_state_func(etr)
define_show_state_func(ape)

#define define_store_state_func(_name) \
static ssize_t trace_##_name##_store(struct kobject *kboj, \
				struct kobj_attribute *attr, \
				const char *buf, size_t n) \
{ \
	unsigned long value; \
	if (kstrtoul(buf, 0, &value)) \
		return -EINVAL; \
	mutex_lock(&tracer.mutex); \
	tracer._name = (value) ? 1 : 0; \
	mutex_unlock(&tracer.mutex); \
	return n; \
}

define_store_state_func(userspace)
define_store_state_func(branch_broadcast)
define_store_state_func(return_stack)
define_store_state_func(timestamp)
define_store_state_func(formatter)
define_store_state_func(etr)
define_store_state_func(ape)

#define A(a, b, c, d)   __ATTR(trace_##a, b, \
	trace_##c##_show, trace_##d##_store)
static const struct kobj_attribute trace_attr[] = {
	A(enable,		0644,	enable,		enable),
	A(userspace,		0644,	userspace,	userspace),
	A(branch_broadcast,	0644,	branch_broadcast, branch_broadcast),
	A(return_stack,		0644,	return_stack, return_stack),
	A(range_address,	0644,	range_address, range_address),
	A(trigger,		0644,	trigger,	trigger),
	A(timestamp,		0644,	timestamp,	timestamp),
	A(cycle_count,		0644,	cycle_count,	cycle_count),
	A(formatter,		0644,	formatter,	formatter),
	A(etr,			0644,	etr,		etr),
	A(ape,			0644,	ape,		ape)
};

static const struct file_operations trc_fops = {
	.owner = THIS_MODULE,
	.read = trc_read,
	.open = trc_open,
	.release = trc_release,
	.llseek = no_llseek,
};

static struct miscdevice trc_miscdev = {
	.name = "trc",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &trc_fops,
};

static struct miscdevice last_trc_miscdev = {
	.name = "last_trc",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &trc_fops,
};

static void trc_nodes(struct tracectx *t)
{
	int ret = 0;

	trc_miscdev.parent = t->dev;
	ret = misc_register(&trc_miscdev);
	if (ret)
		dev_err(t->dev, "failes to register /dev/trc\n");

	last_trc_miscdev.parent = t->dev;
	ret = misc_register(&last_trc_miscdev);
	if (ret)
		dev_err(t->dev, "failes to register /dev/last_trc\n");

	dev_info(t->dev, "Trace nodes are initialized.\n");
}

static int ptm_probe(struct platform_device  *dev)
{
	int i, dbg_cnt = 0, ptm_cnt = 0, ret = 0;

	struct tracectx *t = &tracer;

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
		if (!addr || !dev->dev.of_node)
			goto out;

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
			if (ptm_cnt < CONFIG_NR_CPUS)
				t->ptm_t210_regs[ptm_cnt++] = addr;
			break;
		case 9:
			t->funnel_minor_regs = addr;
			break;
		case 10:
			t->ape_regs = addr;
			break;
		default:
			dev_err(&dev->dev,
				"unknown resource for the PTM device\n");
			break;
		}
	}

	WARN_ON(ptm_cnt < num_possible_cpus());
	WARN_ON(dbg_cnt < num_possible_cpus());

	/* Configure Coresight Clocks */
	csite_clk = clk_get_sys("csite", NULL);

	/* Initialise data structures required for saving trace after reset */
	trc_buf_init(t);

	/* save ETF contents to DRAM when system is reset */
	trc_fill_buf(t);

	/* initialising dev nodes */
	trc_nodes(t);

	/* create sysfs */
	for (i = 0; i < ARRAY_SIZE(trace_attr); i++) {
		ret = sysfs_create_file(&dev->dev.kobj, &trace_attr[i].attr);
		if (ret)
			dev_err(&dev->dev, "failed to create %s\n",
				trace_attr[i].attr.name);
	}

#ifdef CONFIG_HOTPLUG_CPU
	register_cpu_notifier(&ptm_cpu_hotplug_notifier_block);
#endif

	dev_info(&dev->dev, "PTM driver initialized.\n");

out:
	if (ret)
		dev_err(&dev->dev, "Failed to start the PTM device\n");
	return ret;
}

static int ptm_remove(struct platform_device *dev)
{
	struct tracectx *t = &tracer;
	int i;

#ifdef CONFIG_HOTPLUG_CPU
	unregister_cpu_notifier(&ptm_cpu_hotplug_notifier_block);
#endif

	for (i = 0; i < ARRAY_SIZE(trace_attr); i++)
		sysfs_remove_file(&dev->dev.kobj, &trace_attr[i].attr);

	mutex_lock(&t->mutex);

	for (i = 0; i < CONFIG_NR_CPUS; i++)
		devm_iounmap(&dev->dev, t->ptm_t210_regs[i]);
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

