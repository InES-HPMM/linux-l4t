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

/* PTM tracer state */
struct tracectx {
	struct device	*dev;
	struct mutex	mutex;
	void __iomem	*funnel_major_regs;
	void __iomem	*etf_regs;
	void __iomem	*replicator_regs;
	void __iomem	*etr_regs;
	void __iomem	*cpu_regs[4];
	void __iomem	*ptm_t210_regs[4];
	void __iomem	*funnel_bccplex_regs;
	unsigned long	flags;
	int		ptm_t210_regs_count;
	int		cpu_regs_count;
	int		*last_etf;
	int		etf_size;
	bool		dump_initial_etf;
};

static struct tracectx tracer;

static void etf_save_last(struct tracectx *t)
{
	int rrp, rrd, rwp, rwp32, rrp32, max, count = 0, serial, overflow;

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
	rrd = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);

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
			t->last_etf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
	} else {
		pr_info("ETF overflown. Reading contents to userspace\n");
		t->etf_size = MAX_ETF_SIZE * 4;
		/* ETF overflow..the last 4K entries are printed */
		etf_writel(t, rwp, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		/* Read from rwp to end of RAM */
		for (count = rwp32; count < MAX_ETF_SIZE; count++) {
			t->last_etf[serial] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
		/* Rollover the RRP and read till rwp-1 */
		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		for (count = 0; count < max; count++) {
			t->last_etf[serial] = etf_readl(t,
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
						i, t->last_etf[i]);

	if (copy_to_user(data, (char *) t->last_etf , t->etf_size)) {
		ret = -EFAULT;
		goto out;
	}
	*ppos += t->etf_size;
	ret = 0;

out:
	mutex_unlock(&t->mutex);
	return ret;
}

static inline bool trace_isrunning(struct tracectx *t)
{
	return !!(t->flags & TRACER_RUNNING);
}

static int trace_t210_start(struct tracectx *t)
{
	int id;

	/* Enabling the ETM */
	for (id = 0; id < t->ptm_t210_regs_count; id++)
		ptm_t210_writel(t, id, 1,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

	return 0;
}

static int trace_t210_stop(struct tracectx *t)
{
	int id;

	/* Disabling the ETM */
	for (id = 0; id < t->ptm_t210_regs_count; id++)
		ptm_t210_writel(t, id, 0,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCPRGCTLR_0);

	return 0;
}

static int etf_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tracectx *t = dev_get_drvdata(miscdev->parent);

	if (NULL == t->etf_regs)
		return -ENODEV;

	file->private_data = t;

	return nonseekable_open(inode, file);
}

static ssize_t etf_read(struct file *file, char __user *data,
	size_t len, loff_t *ppos)
{
	int rrp, rrd, rwp, rwp32, rrp32, max, count = 0, serial, overflow;
	struct tracectx *t = file->private_data;
	u32 *buf;
	long length;
	loff_t pos = *ppos;

	mutex_lock(&t->mutex);

	if (!t->etf_regs)
		return -ENODEV;

	etf_regs_unlock(t);

	/* Manual flush and stop */
	etf_writel(t, 0x1001, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);
	etf_writel(t, 0x1041, CORESIGHT_ETF_HUGO_CXTMC_REGS_FFCR_0);

	udelay(1000);

	/* Disable ETF */
	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_CTL_0);

	/* Get etf data and Check for overflow */
	overflow = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_STS_0);
	pr_info("overflow %x\n", overflow);

	overflow &= 0x01;

	/* Check for data in ETF */
	rwp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RWP_0);
	rrp = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
	rrd = etf_readl(t, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);

	rwp32 = rwp/4;
	rrp32 = rrp/4;

	max = rwp32;
	serial = 0;

	buf = vmalloc(MAX_ETF_SIZE * 4);
	pr_info(" RRP %08x ,RWP  %08x\n ", rrp32, rwp32);

	etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);

	if (!overflow) {
		pr_info("ETF not overflown. Reading contents to userspace\n");
		length = max * 4;
		for (count = 0; count < max; count++) {
			rrd = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			pr_info("COUNT: %08x  RRP32: %08x ETF DATA: %08x\n",
							serial, count, rrd);
			buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
	} else {
		pr_info("ETF overflown. Reading contents to userspace\n");
		length = MAX_ETF_SIZE * 4;
		/* ETF overflow..the last 4K entries are printed */
		etf_writel(t, rwp, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		/* Read from rwp to end of RAM */
		for (count = rwp32; count < MAX_ETF_SIZE; count++) {
			rrd = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			pr_info("COUNT: %08x  RRP32: %08x ETF DATA: %08x\n",
							serial, count, rrd);
			buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
		/* Rollover the RRP and read till rwp-1 */
		etf_writel(t, 0, CORESIGHT_ETF_HUGO_CXTMC_REGS_RRP_0);
		for (count = 0; count < max; count++) {
			rrd = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			pr_info("COUNT: %08x  RRP32: %08x ETF DATA: %08x\n",
							serial, count, rrd);
			buf[count] = etf_readl(t,
					CORESIGHT_ETF_HUGO_CXTMC_REGS_RRD_0);
			serial += 1;
		}
	}
	etf_regs_lock(t);

	length -= copy_to_user(data, (u8 *) buf, length);
	vfree(buf);
	*ppos = pos + length;

	mutex_unlock(&t->mutex);

	return 0;
}

static int etf_release(struct inode *inode, struct file *file)
{
	/* there's nothing to do here, actually */
	return 0;
}

/* use a sysfs file "trace_running" to start/stop tracing */
static ssize_t trace_running_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%x\n", trace_isrunning(&tracer));
}

static ssize_t trace_running_store(struct kobject *kobj,
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

	if (value != 0)
		ret = trace_t210_start(&tracer);
	else
		ret = trace_t210_stop(&tracer);

	mutex_unlock(&tracer.mutex);

	return ret ? : n;
}

#define A(a, b, c, d)   __ATTR(trace_##a, b, \
	trace_##c##_show, trace_##d##_store)
static const struct kobj_attribute trace_attr[] = {
	A(running,      0644, running,      running)
};

#define clk_readl(reg)  __raw_readl(reg_clk_base + (reg))
#define clk_writel(value, reg) __raw_writel(value, reg_clk_base + (reg))
static void __iomem *reg_clk_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

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

static void replicator_init(struct tracectx *t)
{
	/* unlock replicator */
	replicator_regs_unlock(t);

	/* disabling the replicator */
	replicator_writel(t, 0xff ,
	CORESIGHT_ATBREPLICATOR_HUGO_CXATBREPLICATOR_REGISTERS_IDFILTER1_0);
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

	ptm_t210_writel(t, id, 0x1000,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCCONFIGR_0);

	/* Enable Periodic Synchronisation after 256 bytes */
	ptm_t210_writel(t, id, 8, CORESIGHT_BCCPLEX_CPU_TRACE_TRCSYNCPR_0);

	/* Branch broadcasting control */
	ptm_t210_writel(t, id, 0x101, CORESIGHT_BCCPLEX_CPU_TRACE_TRCBBCTLR_0);

	/* Select Address Range Comparator */
	ptm_t210_writel(t, id, 0x10000,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCSSCCR0_0);

	/* Program Address range comparator. Use 0 and 1 */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR0_0);
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR0_0 + 4);
	ptm_t210_writel(t, id, 0xffffffff,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR1_0);
	ptm_t210_writel(t, id, 0xffffffff,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCACVR1_0 + 4);

	/* Program the ARC control register */
	ptm_t210_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_TRACE_TRCACATR0_0);

	/* Program trace ID register */
	switch (id) {
	case 0:
		ptm_t210_writel(t, id, 0x40,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCTRACEIDR_0);
		break;
	case 1:
		ptm_t210_writel(t, id, 0x41,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCTRACEIDR_0);
		break;
	case 2:
		ptm_t210_writel(t, id, 0x42,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCTRACEIDR_0);
		break;
	case 3:
		ptm_t210_writel(t, id, 0x43,
				CORESIGHT_BCCPLEX_CPU_TRACE_TRCTRACEIDR_0);
		break;
	default:
		pr_info("ID out of bounds\n");
	}

	/* Trace everything with Start/Stop logic started */
	ptm_t210_writel(t, id, 0xE01, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVICTLR_0);

	/* Enable ViewInst and Include logic for ARC0. Disable the SSC */
	ptm_t210_writel(t, id, 1, CORESIGHT_BCCPLEX_CPU_TRACE_TRCVIIECTLR_0);
}

static void funnel_bccplex_init(struct tracectx *t)
{
	/* unlock BCCPLEX funnel */
	funnel_bccplex_regs_unlock(t);

	/* set up funnel port 0 and set HT as 0x04 */
	funnel_bccplex_writel(t, 0x301,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_readl(t,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_writel(t, 0x302,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_readl(t,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_writel(t, 0x304,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_readl(t,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_writel(t, 0x308,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
	funnel_bccplex_readl(t,
		CORESIGHT_ATBFUNNEL_BCCPLEX_CXATBFUNNEL_REGS_CTRL_REG_0);
}

static void funnel_major_init(struct tracectx *t)
{
	/* unlock  funnel */
	funnel_major_regs_unlock(t);

	/* enable funnel port 0 and make HT as 0x04 */
	funnel_major_writel(t, 0x301,
		CORESIGHT_ATBFUNNEL_HUGO_MAJOR_CXATBFUNNEL_REGS_CTRL_REG_0);
}

static void etf_init(struct tracectx *t)
{
	int ret = 0;

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

static void etf_last_init(struct tracectx *t)
{
	t->dump_initial_etf = true;

	t->last_etf = devm_kzalloc(t->dev, MAX_ETF_SIZE * sizeof(*t->last_etf),
					GFP_KERNEL);

	if (NULL == t->last_etf)
		dev_err(t->dev, "failes to allocate memory to hold ETF\n");
}

static void clk_init(void)
{
	clk_writel(0x0, CLK_RST_CONTROLLER_CLK_SOURCE_CSITE_0);
	clk_writel(0x0, CLK_RST_CONTROLLER_CLK_CPUG_MISC2_0);
	clk_writel(0x2, CLK_RST_CONTROLLER_CLK_CPUG_MISC_0);
}

static void cpu_debug_init(struct tracectx *t, int id)
{
	cpu_debug_regs_unlock(t, id);
	cpu_debug_writel(t, id, 0, CORESIGHT_BCCPLEX_CPU_DEBUG_OSLAR_EL1_0);
}

static int ptm_t210_init(struct tracectx *t, struct platform_device  *dev)
{
	int i, ret = 0;
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
			default:
				dev_err(&dev->dev, "unknown resourse for the PTM device\n");
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
		}
	}

	/* Configure Coresight Clocks */
	clk_init();

	/* Unlock CPU Debug to monitor PC */
	for (i = 0; i < t->cpu_regs_count; i++)
		cpu_debug_init(t, i);

	/* Initialise data structures required for saving trace after reset */
	etf_last_init(t);

	/* save ETF contents to DRAM when system is reset */
	etf_save_last(t);

	/* Programming PTM */
	for (i = 0; i < t->ptm_t210_regs_count; i++)
		ptm_init(t, i);

	/* Programming BCCPLEX funnel */
	funnel_bccplex_init(t);

	/* Programming major funnel */
	funnel_major_init(t);

	/* programming the ETF */
	etf_init(t);

	/* programming the replicator */
	replicator_init(t);

	/* create sysfs */
	for (i = 0; i < ARRAY_SIZE(trace_attr); i++) {
		ret = sysfs_create_file(&dev->dev.kobj, &trace_attr[i].attr);
		if (ret)
			dev_err(&dev->dev, "failed to create %s\n",
				trace_attr[i].attr.name);
	}

	dev_info(&dev->dev, "PTM driver initialized.\n");

	/* Start the PTM and ETF now */
	trace_t210_start(t);

out:
	if (ret)
		dev_err(&dev->dev, "Failed to start the PTM device\n");
	mutex_unlock(&t->mutex);
	return ret;
}

static int ptm_probe(struct platform_device *dev)
{
	struct tracectx *t = &tracer;

	mutex_lock(&t->mutex);
	t->dev = &dev->dev;
	platform_set_drvdata(dev, t);

	t->cpu_regs_count = 0;
	t->ptm_t210_regs_count = 0;
	ptm_t210_init(t, dev);

	return 0;
}

static int ptm_remove(struct platform_device *dev)
{
	struct tracectx *t = &tracer;
	int i;

	for (i = 0; i < ARRAY_SIZE(trace_attr); i++)
		sysfs_remove_file(&dev->dev.kobj, &trace_attr[i].attr);

	mutex_lock(&t->mutex);

	devm_iounmap(&dev->dev, t->ptm_t210_regs);
	devm_iounmap(&dev->dev, t->cpu_regs);
	devm_iounmap(&dev->dev, t->funnel_major_regs);
	devm_iounmap(&dev->dev, t->etf_regs);
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

