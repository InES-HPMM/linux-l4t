/*
 * arch/arm/mach-tegra/tegra11_soctherm.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <mach/iomap.h>


#define CONF_STAT_MEM0			0x8
#define CONF_STAT_MEM0_UP_THRESH_SHIFT	17
#define CONF_STAT_MEM0_UP_THRESH_MASK	0xff
#define CONF_STAT_MEM0_DN_THRESH_SHIFT	9
#define CONF_STAT_MEM0_DN_THRESH_MASK	0xff
#define CONF_STAT_MEM0_EN_SHIFT		8
#define CONF_STAT_MEM0_EN_MASK		0x1
#define CONF_STAT_MEM0_CPU_THROT_SHIFT	5
#define CONF_STAT_MEM0_CPU_THROT_MASK	0x3
#define CONF_STAT_MEM0_STATUS_SHIFT	0
#define CONF_STAT_MEM0_STATUS_MASK	0x3

#define THERMTRIP			0x80
#define THERMTRIP_ANY_EN_SHIFT		28
#define THERMTRIP_ANY_EN_MASK		0x1
#define THERMTRIP_CPU_EN_SHIFT		25
#define THERMTRIP_CPU_EN_MASK		0x1
#define THERMTRIP_CPU_THRESH_SHIFT	8
#define THERMTRIP_CPU_THRESH_MASK	0xff

#define TS_MEM0_CONFIG0			0x140
#define TS_MEM0_CONFIG0_TALL_SHIFT	8
#define TS_MEM0_CONFIG0_TALL_MASK	0xfffff

#define TS_MEM0_CONFIG1			0x144
#define TS_MEM0_CONFIG1_EN_SHIFT	31
#define TS_MEM0_CONFIG1_EN_MASK		0x1
#define TS_MEM0_CONFIG1_TIDDQ_SHIFT	15
#define TS_MEM0_CONFIG1_TIDDQ_MASK	0x3f
#define TS_MEM0_CONFIG1_TEN_COUNT_SHIFT	24
#define TS_MEM0_CONFIG1_TEN_COUNT_MASK	0x3f
#define TS_MEM0_CONFIG1_TSAMPLE_SHIFT	0
#define TS_MEM0_CONFIG1_TSAMPLE_MASK	0x3ff

#define TS_MEM0_CONFIG2			0x148
#define TS_MEM0_CONFIG2_THERM_A_SHIFT	16
#define TS_MEM0_CONFIG2_THERM_A_MASK	0xffff
#define TS_MEM0_CONFIG2_THERM_B_SHIFT	0
#define TS_MEM0_CONFIG2_THERM_B_MASK	0xffff

#define TS_MEM0_STATUS0			0x14c
#define TS_MEM0_STATUS0_CAPTURE_SHIFT	0
#define TS_MEM0_STATUS0_CAPTURE_MASK	0xffff

#define TS_MEM0_STATUS1			0x150

#define TS_MEM0_STATUS2			0x154

#define TS_PDIV				0x1c0
#define TS_PDIV_MEM_SHIFT		4
#define TS_PDIV_MEM_MASK		0xf


#define UP_STATS_L0		0x10
#define DN_STATS_L0		0x14

#define INTR_STATUS		0x84

#define INTR_EN			0x88
#define INTR_EN_CU0_SHIFT	8
#define INTR_EN_CD0_SHIFT	9

#define INTR_DIS		0x8c
#define LOCK_CTL		0x90
#define STATS_CTL		0x94

#define REG_SET(r,_name,val) \
	((r)&~(_name##_MASK<<_name##_SHIFT))|(((val)&_name##_MASK)<<_name##_SHIFT)

#define REG_GET(r,_name) \
	(((r)&(_name##_MASK<<_name##_SHIFT))>>_name##_SHIFT)

static void __iomem *reg_soctherm_base = IO_ADDRESS(TEGRA_SOCTHERM_BASE);

#define soctherm_writel(value, reg) \
	__raw_writel(value, reg_soctherm_base + (reg))
#define soctherm_readl(reg) \
	__raw_readl(reg_soctherm_base + (reg))

static struct dentry *tegra_soctherm_root;

int soctherm_set_limits(long lo_limit_milli, long hi_limit_milli)
{
	u32 r = soctherm_readl(CONF_STAT_MEM0);
	r = REG_SET(r, CONF_STAT_MEM0_DN_THRESH, lo_limit_milli/1000);
	r = REG_SET(r, CONF_STAT_MEM0_UP_THRESH, hi_limit_milli/1000);
	soctherm_writel(r, CONF_STAT_MEM0);

	soctherm_writel(1<<INTR_EN_CU0_SHIFT, INTR_EN);
	soctherm_writel(1<<INTR_EN_CD0_SHIFT, INTR_EN);

	return 0;
}

int soctherm_get_temp(long *temp)
{
	return 0;
}

int soctherm_set_shutdown(long shutdown_temp_milli)
{
	u32 r = soctherm_readl(THERMTRIP);
	r = REG_SET(r, THERMTRIP_CPU_THRESH, shutdown_temp_milli/1000);
	r = REG_SET(r, THERMTRIP_CPU_EN, 1);
	soctherm_writel(r, THERMTRIP);

	return 0;
}

static int pdiv_set(void *data, u64 val)
{
	u32 r;
	r = REG_SET(0, TS_PDIV_MEM, val);
	soctherm_writel(r, TS_PDIV);
	return 0;
}

static int pdiv_get(void *data, u64 *val)
{
	*val = (u64)soctherm_readl(TS_PDIV);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pdiv_fops,
			pdiv_get,
			pdiv_set,
			"%llu\n");

static int regs_show(struct seq_file *s, void *data)
{
	u32 r;

	r = soctherm_readl(CONF_STAT_MEM0);
	seq_printf(s, "SOC_THERM_THERMCTL_LEVEL0_GROUP_MEM_0: 0x%x\n", r);

	r = soctherm_readl(TS_MEM0_STATUS0);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_STATUS0_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_STATUS1);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_STATUS1_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_CONFIG0);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_CONFIG0_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_CONFIG1);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_CONFIG1_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_CONFIG2);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_CONFIG2_0:      0x%x\n", r);

	r = soctherm_readl(INTR_STATUS);
	seq_printf(s, "SOC_THERM_THERMCTL_INTR_STATUS_0:      0x%x\n", r);

	return 0;
}

static int regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, regs_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static irqreturn_t soctherm_isr(int irq, void *arg_data)
{
	return IRQ_HANDLED;
}

int __init tegra11_soctherm_init(void)
{
	int err;
	u32 r;

	r = soctherm_readl(CONF_STAT_MEM0);
	r = REG_SET(r, CONF_STAT_MEM0_EN, 1);
	soctherm_writel(r, CONF_STAT_MEM0);

	r = REG_SET(0, TS_MEM0_CONFIG0_TALL, 15);
	soctherm_writel(r, TS_MEM0_CONFIG0);

	r = REG_SET(0, TS_MEM0_CONFIG1_TIDDQ, 1);
	r = REG_SET(r, TS_MEM0_CONFIG1_EN, 1);
	r = REG_SET(r, TS_MEM0_CONFIG1_TEN_COUNT, 1);
	r = REG_SET(r, TS_MEM0_CONFIG1_TSAMPLE, 10);
	soctherm_writel(r, TS_MEM0_CONFIG1);

	r = REG_SET(0, TS_MEM0_CONFIG2_THERM_A, 114);
	r = REG_SET(r, TS_MEM0_CONFIG2_THERM_B, 0);
	soctherm_writel(r, TS_MEM0_CONFIG2);


	soctherm_set_limits(20000, 40000);

	err = request_irq(INT_THERMAL, soctherm_isr,
				IRQF_ONESHOT, "soctherm", NULL);

	tegra_soctherm_root = debugfs_create_dir("tegra_soctherm", 0);
	debugfs_create_file("pdiv", 0644, tegra_soctherm_root, NULL, &pdiv_fops);
	debugfs_create_file("regs", 0644, tegra_soctherm_root, NULL, &regs_fops);

	return 0;
}

