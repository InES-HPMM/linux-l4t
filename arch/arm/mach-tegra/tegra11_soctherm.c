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

#define CTL_LVL0_CPU0			0x0
#define CTL_LVL0_CPU0_UP_THRESH_SHIFT	17
#define CTL_LVL0_CPU0_UP_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_DN_THRESH_SHIFT	9
#define CTL_LVL0_CPU0_DN_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_EN_SHIFT		8
#define CTL_LVL0_CPU0_EN_MASK		0x1
#define CTL_LVL0_CPU0_CPU_THROT_SHIFT	5
#define CTL_LVL0_CPU0_CPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_MEM_THROT_SHIFT	2
#define CTL_LVL0_CPU0_MEM_THROT_MASK	0x1
#define CTL_LVL0_CPU0_STATUS_SHIFT	0
#define CTL_LVL0_CPU0_STATUS_MASK	0x3

#define THERMTRIP			0x80
#define THERMTRIP_ANY_EN_SHIFT		28

#define THERMTRIP			0x80
#define THERMTRIP_ANY_EN_SHIFT		28
#define THERMTRIP_ANY_EN_MASK		0x1
#define THERMTRIP_CPU_EN_SHIFT		25
#define THERMTRIP_CPU_EN_MASK		0x1
#define THERMTRIP_CPU_THRESH_SHIFT	8
#define THERMTRIP_CPU_THRESH_MASK	0xff

#define TS_CPU0_CONFIG0				0xc0
#define TS_CPU0_CONFIG0_TALL_SHIFT		8
#define TS_CPU0_CONFIG0_TALL_MASK		0xfffff
#define TS_CPU0_CONFIG0_TCALC_OVER_SHIFT	4
#define TS_CPU0_CONFIG0_TCALC_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_OVER_SHIFT		3
#define TS_CPU0_CONFIG0_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_CPTR_OVER_SHIFT		2
#define TS_CPU0_CONFIG0_CPTR_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_STOP_SHIFT		0
#define TS_CPU0_CONFIG0_STOP_MASK		0x1

#define TS_CPU0_CONFIG1			0xc4
#define TS_CPU0_CONFIG1_EN_SHIFT	31
#define TS_CPU0_CONFIG1_EN_MASK		0x1
#define TS_CPU0_CONFIG1_TIDDQ_SHIFT	15
#define TS_CPU0_CONFIG1_TIDDQ_MASK	0x3f
#define TS_CPU0_CONFIG1_TEN_COUNT_SHIFT	24
#define TS_CPU0_CONFIG1_TEN_COUNT_MASK	0x3f
#define TS_CPU0_CONFIG1_TSAMPLE_SHIFT	0
#define TS_CPU0_CONFIG1_TSAMPLE_MASK	0x3ff

#define TS_CPU0_CONFIG2			0xc8
#define TS_CPU0_CONFIG2_THERM_A_SHIFT	16
#define TS_CPU0_CONFIG2_THERM_A_MASK	0xffff
#define TS_CPU0_CONFIG2_THERM_B_SHIFT	0
#define TS_CPU0_CONFIG2_THERM_B_MASK	0xffff

#define TS_CPU0_STATUS0			0xcc
#define TS_CPU0_STATUS0_VALID_SHIFT	31
#define TS_CPU0_STATUS0_VALID_MASK	0x1
#define TS_CPU0_STATUS0_CAPTURE_SHIFT	0
#define TS_CPU0_STATUS0_CAPTURE_MASK	0xffff

#define TS_CPU0_STATUS1				0xd0
#define TS_CPU0_STATUS1_TEMP_VALID_SHIFT	31
#define TS_CPU0_STATUS1_TEMP_VALID_MASK		0x1
#define TS_CPU0_STATUS1_TEMP_SHIFT		0
#define TS_CPU0_STATUS1_TEMP_MASK		0xffff

#define TS_CPU0_STATUS2			0xd4

#define TS_CONFIG_STATUS_OFFSET		0x20

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
#define TS_PDIV_CPU_SHIFT		12
#define TS_PDIV_CPU_MASK		0xf
#define TS_PDIV_GPU_SHIFT		8
#define TS_PDIV_GPU_MASK		0xf
#define TS_PDIV_MEM_SHIFT		4
#define TS_PDIV_MEM_MASK		0xf
#define TS_PDIV_PLLX_SHIFT		0
#define TS_PDIV_PLLX_MASK		0xf

#define TS_TEMP1			0x1c8
#define TS_TEMP1_CPU_TEMP_SHIFT		16
#define TS_TEMP1_CPU_TEMP_MASK		0xffff
#define TS_TEMP1_GPU_TEMP_SHIFT		0
#define TS_TEMP1_GPU_TEMP_MASK		0xffff

#define TS_TEMP2			0x1cc
#define TS_TEMP2_MEM_TEMP_SHIFT		16
#define TS_TEMP2_MEM_TEMP_MASK		0xffff
#define TS_TEMP2_PLLX_TEMP_SHIFT	0
#define TS_TEMP2_PLLX_TEMP_MASK		0xffff

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

#define PSKIP_STATUS				0x418

#define PSKIP_CTRL_LITE_CPU			0x430

#define PSKIP_CTRL_HEAVY_CPU			0x460
#define PSKIP_CTRL_HEAVY_CPU_ENABLE_SHIFT	31
#define PSKIP_CTRL_HEAVY_CPU_ENABLE_MASK	0x1
#define PSKIP_CTRL_HEAVY_CPU_DIVIDEND_SHIFT	8
#define PSKIP_CTRL_HEAVY_CPU_DIVIDEND_MASK	0xff
#define PSKIP_CTRL_HEAVY_CPU_DIVISOR_SHIFT	0
#define PSKIP_CTRL_HEAVY_CPU_DIVISOR_MASK	0xff

#define PSKIP_RAMP_HEAVY_CPU			0x464
#define PSKIP_RAMP_HEAVY_CPU_DURATION_SHIFT	8
#define PSKIP_RAMP_HEAVY_CPU_DURATION_MASK	0xffff
#define PSKIP_RAMP_HEAVY_CPU_STEP_SHIFT		0
#define PSKIP_RAMP_HEAVY_CPU_STEP_MASK		0xff

#define PSKIP_CTRL_OC1_CPU			0x490

#define REG_SET(r,_name,val) \
	((r)&~(_name##_MASK<<_name##_SHIFT))|(((val)&_name##_MASK)<<_name##_SHIFT)

#define REG_GET(r,_name) \
	(((r)&(_name##_MASK<<_name##_SHIFT))>>_name##_SHIFT)

static void __iomem *reg_soctherm_base = IO_ADDRESS(TEGRA_SOCTHERM_BASE);

#define soctherm_writel(value, reg) \
	__raw_writel(value, reg_soctherm_base + (reg))
#define soctherm_readl(reg) \
	__raw_readl(reg_soctherm_base + (reg))

static struct soctherm_platform_data plat_data;

static char *therm_names[] = {
	[THERM_CPU] = "CPU",
	[THERM_MEM] = "MEM",
	[THERM_GPU] = "GPU",
	[THERM_PLL] = "PLL",
};

static char *sensor_names[] = {
	[TSENSE_CPU0] = "cpu0",
	[TSENSE_CPU1] = "cpu1",
	[TSENSE_CPU2] = "cpu2",
	[TSENSE_CPU3] = "cpu3",
	[TSENSE_MEM0] = "mem0",
	[TSENSE_MEM1] = "mem1",
	[TSENSE_GPU]  = "gpu0",
	[TSENSE_PLLX] = "pllx",
};

static inline long temp_translate(int readback)
{
	int abs = readback >> 8;
	int lsb = (readback & 0x80) >> 7;
	int sign = readback & 0x1;

	return (abs * 1000 + lsb * 500) * (sign * -2 + 1);
}

static int soctherm_set_limits(void *data,
	long lo_limit_milli,
	long hi_limit_milli)
{
	u32 r = soctherm_readl(CTL_LVL0_CPU0);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, lo_limit_milli/1000);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, hi_limit_milli/1000);
	soctherm_writel(r, CTL_LVL0_CPU0);

	soctherm_writel(1<<INTR_EN_CU0_SHIFT, INTR_EN);
	soctherm_writel(1<<INTR_EN_CD0_SHIFT, INTR_EN);

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
	enum soctherm_sense sensor = (enum soctherm_sense)thz->devdata;
	u32 r = soctherm_readl(TS_CPU0_STATUS1 +
				sensor * TS_CONFIG_STATUS_OFFSET);
	*temp = temp_translate(REG_GET(r, TS_CPU0_STATUS1_TEMP));
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

void tegra11_soctherm_skipper_program(struct soctherm_skipper_data *data)
{
	u32 r;
	r = soctherm_readl(PSKIP_CTRL_HEAVY_CPU);
	r = REG_SET(r, PSKIP_CTRL_HEAVY_CPU_ENABLE, data->enable);
	r = REG_SET(r, PSKIP_CTRL_HEAVY_CPU_DIVIDEND, data->dividend);
	r = REG_SET(r, PSKIP_CTRL_HEAVY_CPU_DIVISOR, data->divisor);
	soctherm_writel(r, PSKIP_CTRL_HEAVY_CPU);

	r = soctherm_readl(PSKIP_RAMP_HEAVY_CPU);
	r = REG_SET(r, PSKIP_RAMP_HEAVY_CPU_DURATION, data->duration);
	r = REG_SET(r, PSKIP_RAMP_HEAVY_CPU_STEP, data->step);
	soctherm_writel(r, PSKIP_RAMP_HEAVY_CPU);
}

static void __init soctherm_tsense_program(enum soctherm_sense sensor,
						struct soctherm_sensor *data)
{
	u32 r;
	int offset = sensor * TS_CONFIG_STATUS_OFFSET;

	r = REG_SET(0, TS_CPU0_CONFIG0_TALL, data->tall);
	soctherm_writel(r, TS_CPU0_CONFIG0 + offset);

	r = REG_SET(0, TS_CPU0_CONFIG1_TIDDQ, data->tiddq);
	r = REG_SET(r, TS_CPU0_CONFIG1_EN, data->enable);
	r = REG_SET(r, TS_CPU0_CONFIG1_TEN_COUNT, data->ten_count);
	r = REG_SET(r, TS_CPU0_CONFIG1_TSAMPLE, data->tsample);
	soctherm_writel(r, TS_CPU0_CONFIG1 + offset);

	r = REG_SET(0, TS_CPU0_CONFIG2_THERM_A, data->therm_a);
	r = REG_SET(r, TS_CPU0_CONFIG2_THERM_B, data->therm_b);
	soctherm_writel(r, TS_CPU0_CONFIG2 + offset);
}

int __init tegra11_soctherm_init(struct soctherm_platform_data *data)
{
	int err, i;
	u32 r;

	memcpy(&plat_data, data, sizeof(struct soctherm_platform_data));

	/* Thermal Sensing programming */
	for (i = 0; i < TSENSE_SIZE; i++) {
		if (plat_data.sensor_data[i].enable) {
			soctherm_tsense_program(i, &plat_data.sensor_data[i]);
#ifdef CONFIG_THERMAL
			/* Create a thermal zone device for each sensor */
			thermal_zone_device_register(
					sensor_names[i],
					0,
					0,
					(void *)i,
					&soctherm_ops,
					plat_data.passive[i].tc1,
					plat_data.passive[i].tc2,
					plat_data.passive[i].passive_delay,
					0);
#endif
		}
	}

	/* Enable Level 0 */
	r = soctherm_readl(CTL_LVL0_CPU0);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	soctherm_writel(r, CTL_LVL0_CPU0);

#if 0
	/* Enable Level 1 Hw throttling */
	r = soctherm_readl(CTL_LVL1_MEM0);
	r = REG_SET(r, CTL_LVL0_MEM0_UP_THRESH, 60);
	r = REG_SET(r, CTL_LVL0_MEM0_EN, 1);
	r = REG_SET(r, CTL_LVL0_MEM0_CPU_THROT, 2); /* Heavy throttling */

	soctherm_writel(r, CTL_LVL1_MEM0);
#endif

	/* Thermtrip */
	r = soctherm_readl(THERMTRIP);
	r = REG_SET(r, THERMTRIP_CPU_THRESH, data->therm_trip);
	r = REG_SET(r, THERMTRIP_CPU_EN, 1);
	soctherm_writel(r, THERMTRIP);

	/* Pdiv */
	r = soctherm_readl(TS_PDIV);
	r = REG_SET(r, TS_PDIV_CPU, 10);
	r = REG_SET(r, TS_PDIV_GPU, 10);
	r = REG_SET(r, TS_PDIV_MEM, 10);
	r = REG_SET(r, TS_PDIV_PLLX, 10);
	soctherm_writel(r, TS_PDIV);

	err = request_irq(INT_THERMAL, soctherm_isr,
				0, "soctherm", NULL);
	if (err < 0)
		return -1;

	soctherm_set_limits(NULL, 20000, 38000);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int cpu0_show(struct seq_file *s, void *data)
{
	u32 r, state;
	r = soctherm_readl(TS_CPU0_STATUS0);
	state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
	seq_printf(s, "%d,", state);

	r = soctherm_readl(TS_CPU0_STATUS1);
	state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
	seq_printf(s, "%ld\n", temp_translate(state));

	return 0;
}

static int regs_show(struct seq_file *s, void *data)
{
	u32 r;
	u32 state;
	int i;

	seq_printf(s, "-----TSENSE-----\n");
	for (i = 0; i < TSENSE_SIZE; i++) {
		seq_printf(s, "%s: ", sensor_names[i]);

		r = soctherm_readl(TS_CPU0_STATUS0 +
					i * TS_CONFIG_STATUS_OFFSET);
		state = REG_GET(r, TS_CPU0_STATUS0_VALID);
		seq_printf(s, "Capture(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
		seq_printf(s, "%d) ", state);


		r = soctherm_readl(TS_CPU0_STATUS1 +
					i * TS_CONFIG_STATUS_OFFSET);
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID);
		seq_printf(s, "Temp(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
		seq_printf(s, "%ld) ", temp_translate(state));


		r = soctherm_readl(TS_CPU0_CONFIG0 +
					i * TS_CONFIG_STATUS_OFFSET);
		state = REG_GET(r, TS_CPU0_CONFIG0_TALL);
		seq_printf(s, "Tall(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TCALC_OVER);
		seq_printf(s, "Over(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_OVER);
		seq_printf(s, "%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_CPTR_OVER);
		seq_printf(s, "%d) ", state);

		r = soctherm_readl(TS_CPU0_CONFIG1 +
					i * TS_CONFIG_STATUS_OFFSET);
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		seq_printf(s, "En(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TIDDQ);
		seq_printf(s, "tiddq(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TEN_COUNT);
		seq_printf(s, "ten_count(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TSAMPLE);
		seq_printf(s, "tsample(%d) ", state);

		r = soctherm_readl(TS_CPU0_CONFIG2 +
					i * TS_CONFIG_STATUS_OFFSET);
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_A);
		seq_printf(s, "Therm_A/B(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_B);
		seq_printf(s, "%d)\n", state);
	}

	r = soctherm_readl(TS_PDIV);
	seq_printf(s, "PDIV: 0x%x\n", r);

	seq_printf(s, "\n");
	seq_printf(s, "-----SOC_THERM-----\n");

	r = soctherm_readl(TS_TEMP1);
	state = REG_GET(r, TS_TEMP1_CPU_TEMP);
	seq_printf(s, "Temperature: CPU(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP1_GPU_TEMP);
	seq_printf(s, " GPU(%ld) ", temp_translate(state));
	r = soctherm_readl(TS_TEMP2);
	state = REG_GET(r, TS_TEMP2_MEM_TEMP);
	seq_printf(s, " MEM(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP2_PLLX_TEMP);
	seq_printf(s, " PLLX(%ld)\n", temp_translate(state));

	for (i = 0; i < THERM_SIZE; i++) {
		seq_printf(s, "%s: ", therm_names[i]);

		r = soctherm_readl(CTL_LVL0_CPU0 + i * 4);
		state = REG_GET(r, CTL_LVL0_CPU0_UP_THRESH);
		seq_printf(s, "Up/Dn(%d/", state);
		state = REG_GET(r, CTL_LVL0_CPU0_DN_THRESH);
		seq_printf(s, "%d) ", state);
		state = REG_GET(r, CTL_LVL0_CPU0_EN);
		seq_printf(s, "En(%d) ", state);
		state = REG_GET(r, CTL_LVL0_CPU0_STATUS);
		seq_printf(s, "Status(%s)\n", state == 0 ? "below" :
					state == 1 ? "in" :
					state == 2 ? "res" :
						"above");
	}

	r = soctherm_readl(INTR_STATUS);
	state = REG_GET(r, INTR_STATUS_MD0);
	seq_printf(s, "MD0: %d\n", state);
	state = REG_GET(r, INTR_STATUS_MU0);
	seq_printf(s, "MU0: %d\n", state);

	r = soctherm_readl(PSKIP_STATUS);
	seq_printf(s, "PSKIP: 0x%x\n", r);

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

static int cpu0_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu0_show, inode->i_private);
}

static const struct file_operations cpu0_fops = {
	.open		= cpu0_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init soctherm_debug_init(void)
{
	struct dentry *tegra_soctherm_root;

	tegra_soctherm_root = debugfs_create_dir("tegra_soctherm", 0);
	debugfs_create_file("regs", 0644, tegra_soctherm_root,
				NULL, &regs_fops);
	debugfs_create_file("cpu0", 0644, tegra_soctherm_root,
				NULL, &cpu0_fops);

	return 0;
}
late_initcall(soctherm_debug_init);
#endif
