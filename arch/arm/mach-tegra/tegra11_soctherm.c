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
#include <linux/delay.h>
#include <linux/slab.h>

#include <mach/iomap.h>

#include "tegra11_soctherm.h"


#define CTL_LVL0_MEM0			0x8
#define CTL_LVL0_MEM0_UP_THRESH_SHIFT	17
#define CTL_LVL0_MEM0_UP_THRESH_MASK	0xff
#define CTL_LVL0_MEM0_DN_THRESH_SHIFT	9
#define CTL_LVL0_MEM0_DN_THRESH_MASK	0xff
#define CTL_LVL0_MEM0_EN_SHIFT		8
#define CTL_LVL0_MEM0_EN_MASK		0x1
#define CTL_LVL0_MEM0_CPU_THROT_SHIFT	5
#define CTL_LVL0_MEM0_CPU_THROT_MASK	0x3
#define CTL_LVL0_MEM0_MEM_THROT_SHIFT	2
#define CTL_LVL0_MEM0_MEM_THROT_MASK	0x1
#define CTL_LVL0_MEM0_STATUS_SHIFT	0
#define CTL_LVL0_MEM0_STATUS_MASK	0x3

#define CTL_LVL1_MEM0			0x28

#define THERMTRIP			0x80
#define THERMTRIP_ANY_EN_SHIFT		28
#
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

#define TS_MEM0_STATUS1				0x150
#define TS_MEM0_STATUS1_TEMP_VALID_SHIFT	31
#define TS_MEM0_STATUS1_TEMP_VALID_MASK		0x1
#define TS_MEM0_STATUS1_TEMP_SHIFT		0
#define TS_MEM0_STATUS1_TEMP_MASK		0xffff


#define TS_MEM0_STATUS2			0x154

#define TS_PDIV				0x1c0
#define TS_PDIV_MEM_SHIFT		4
#define TS_PDIV_MEM_MASK		0xf


#define UP_STATS_L0		0x10
#define DN_STATS_L0		0x14

#define INTR_STATUS			0x84
#define INTR_STATUS_MD0_SHIFT		25
#define INTR_STATUS_MD0_MASK		0x1
#define INTR_STATUS_MU0_SHIFT		24
#define INTR_STATUS_MU0_MASK		0x1

#define INTR_EN			0x88
#define INTR_EN_MU0_SHIFT	24
#define INTR_EN_MD0_SHIFT	25
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

struct thermal_zone_device *thz;
struct soctherm_platform_data plat_data;

static int soctherm_set_limits(void *data,
	long lo_limit_milli,
	long hi_limit_milli)
{
	u32 r = soctherm_readl(CTL_LVL0_MEM0);
	r = REG_SET(r, CTL_LVL0_MEM0_DN_THRESH, lo_limit_milli/1000);
	r = REG_SET(r, CTL_LVL0_MEM0_UP_THRESH, hi_limit_milli/1000);
	soctherm_writel(r, CTL_LVL0_MEM0);

	soctherm_writel(1<<INTR_EN_MU0_SHIFT, INTR_EN);
	soctherm_writel(1<<INTR_EN_MD0_SHIFT, INTR_EN);

	return 0;
}

#ifdef CONFIG_THERMAL
static int soctherm_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice)
{
	return 0;
}

static int soctherm_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice)
{
	return 0;
}

static int soctherm_get_temp(struct thermal_zone_device *thz,
					unsigned long *temp)
{
	u32 r = soctherm_readl(TS_MEM0_STATUS1);
	*temp = REG_GET(r, TS_MEM0_STATUS1_TEMP);
	return 0;
}

static int soctherm_get_trip_type(struct thermal_zone_device *thz,
					int trip,
					enum thermal_trip_type *type) {
	return 0;
}

static int soctherm_get_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long *temp) {
	return 0;
}

static struct thermal_zone_device_ops soctherm_ops = {
	.bind = soctherm_bind,
	.unbind = soctherm_unbind,
	.get_temp = soctherm_get_temp,
	.get_trip_type = soctherm_get_trip_type,
	.get_trip_temp = soctherm_get_trip_temp,
};
#endif

static irqreturn_t soctherm_isr(int irq, void *arg_data)
{
	u32 r;

	r = soctherm_readl(INTR_STATUS);
	soctherm_writel(r, INTR_STATUS);

	return IRQ_HANDLED;
}

int __init tegra11_soctherm_init(struct soctherm_platform_data *data)
{
	int err;
	u32 r;

	memcpy(&plat_data, data, sizeof(struct soctherm_platform_data));

	/* Thermal Sensing programming */
	r = REG_SET(0, TS_MEM0_CONFIG0_TALL, data->sensor.tall);
	soctherm_writel(r, TS_MEM0_CONFIG0);

	r = REG_SET(0, TS_MEM0_CONFIG1_TIDDQ, data->sensor.tiddq);
	r = REG_SET(r, TS_MEM0_CONFIG1_EN, 1);
	r = REG_SET(r, TS_MEM0_CONFIG1_TEN_COUNT, data->sensor.ten_count);
	r = REG_SET(r, TS_MEM0_CONFIG1_TSAMPLE, data->sensor.tsample);
	soctherm_writel(r, TS_MEM0_CONFIG1);

	r = REG_SET(0, TS_MEM0_CONFIG2_THERM_A, data->sensor.therm_a);
	r = REG_SET(r, TS_MEM0_CONFIG2_THERM_B, data->sensor.therm_b);
	soctherm_writel(r, TS_MEM0_CONFIG2);

	/* Enable Level 0 */
	r = soctherm_readl(CTL_LVL0_MEM0);
	r = REG_SET(r, CTL_LVL0_MEM0_EN, 1);
	soctherm_writel(r, CTL_LVL0_MEM0);

	/* Enable Level 1 Hw throttling */
	r = soctherm_readl(CTL_LVL1_MEM0);
	r = REG_SET(r, CTL_LVL0_MEM0_UP_THRESH, 60);
	r = REG_SET(r, CTL_LVL0_MEM0_EN, 1);
	r = REG_SET(r, CTL_LVL0_MEM0_CPU_THROT, 2); /* Heavy throttling */
	soctherm_writel(r, CTL_LVL1_MEM0);

	/* Thermtrip */
	r = soctherm_readl(THERMTRIP);
	r = REG_SET(r, THERMTRIP_CPU_THRESH, data->therm_trip);
	r = REG_SET(r, THERMTRIP_CPU_EN, 1);
	soctherm_writel(r, THERMTRIP);

	/* Pdiv */
	r = REG_SET(0, TS_PDIV_MEM, 10);
	soctherm_writel(r, TS_PDIV);

	err = request_irq(INT_THERMAL, soctherm_isr,
				0, "soctherm", NULL);
	if (err < 0)
		return -1;

#ifdef CONFIG_THERMAL
	thz = thermal_zone_device_register("soctherm",
						0,
						0,
						NULL,
						&soctherm_ops,
						data->passive.tc1,
						data->passive.tc2,
						data->passive.passive_delay,
						0);
#endif
	soctherm_set_limits(NULL, 20000, 40000);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int regs_show(struct seq_file *s, void *data)
{
	u32 r;
	u32 state;

	r = soctherm_readl(CTL_LVL0_MEM0);
	state = REG_GET(r, CTL_LVL0_MEM0_UP_THRESH);
	seq_printf(s, "Up: %d\n", state);
	state = REG_GET(r, CTL_LVL0_MEM0_DN_THRESH);
	seq_printf(s, "Down: %d\n", state);
	state = REG_GET(r, CTL_LVL0_MEM0_EN);
	seq_printf(s, "Enabled: %d\n", state);
	state = REG_GET(r, CTL_LVL0_MEM0_MEM_THROT);
	seq_printf(s, "MEM throttle: %d\n", state);
	state = REG_GET(r, CTL_LVL0_MEM0_STATUS);
	seq_printf(s, "Status: %s\n", state == 0 ? "below" :
					state == 1 ? "in" :
					state == 2 ? "res" :
						"above");

	r = soctherm_readl(TS_MEM0_STATUS0);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_STATUS0_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_STATUS1);
	state = REG_GET(r, TS_MEM0_STATUS1_TEMP_VALID);
	seq_printf(s, "Temp Valid: %d\n", state);
	state = REG_GET(r, TS_MEM0_STATUS1_TEMP);
	seq_printf(s, "Temp: %d\n", state);
	seq_printf(s, "SOC_THERM_TEMP: 0x%x\n", r);

	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_STATUS1_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_CONFIG0);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_CONFIG0_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_CONFIG1);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_CONFIG1_0:      0x%x\n", r);

	r = soctherm_readl(TS_MEM0_CONFIG2);
	seq_printf(s, "SOC_THERM_TSENSOR_MEM0_CONFIG2_0:      0x%x\n", r);

	r = soctherm_readl(INTR_STATUS);
	state = REG_GET(r, INTR_STATUS_MD0);
	seq_printf(s, "MD0: %d\n", state);
	state = REG_GET(r, INTR_STATUS_MU0);
	seq_printf(s, "MU0: %d\n", state);

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

static int __init soctherm_debug_init(void)
{
	struct dentry *tegra_soctherm_root;

	tegra_soctherm_root = debugfs_create_dir("tegra_soctherm", 0);
	debugfs_create_file("regs", 0644, tegra_soctherm_root, NULL, &regs_fops);

	return 0;
}
late_initcall(soctherm_debug_init);
#endif
