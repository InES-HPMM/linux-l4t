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
#define THERMTRIP_ANY_EN_MASK		0x1
#define THERMTRIP_MEM_EN_SHIFT		27
#define THERMTRIP_MEM_EN_MASK		0x1
#define THERMTRIP_GPU_EN_SHIFT		26
#define THERMTRIP_GPU_EN_MASK		0x1
#define THERMTRIP_CPU_EN_SHIFT		25
#define THERMTRIP_CPU_EN_MASK		0x1
#define THERMTRIP_TSENSE_EN_SHIFT	24
#define THERMTRIP_TSENSE_EN_MASK	0x1
#define THERMTRIP_GPUMEM_THRESH_SHIFT	16
#define THERMTRIP_GPUMEM_THRESH_MASK	0xff
#define THERMTRIP_CPU_THRESH_SHIFT	8
#define THERMTRIP_CPU_THRESH_MASK	0xff
#define THERMTRIP_TSENSE_THRESH_SHIFT	0
#define THERMTRIP_TSENSE_THRESH_MASK	0xff

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
static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);

#define pmc_writel(value, reg) __raw_writel(value, pmc_base + (reg))
#define pmc_readl(reg) __raw_readl(pmc_base + (reg))

#define soctherm_writel(value, reg) \
	__raw_writel(value, reg_soctherm_base + (reg))
#define soctherm_readl(reg) \
	__raw_readl(reg_soctherm_base + (reg))

static struct soctherm_platform_data plat_data;
static struct thermal_zone_device *thz[THERM_SIZE];
static struct workqueue_struct *workqueue;
static struct work_struct work;

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

static int soctherm_set_limits(enum soctherm_therm therm,
				long lo_limit, long hi_limit)
{
	u32 r = soctherm_readl(CTL_LVL0_CPU0);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, lo_limit);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, hi_limit);
	soctherm_writel(r, CTL_LVL0_CPU0);

	soctherm_writel(1<<INTR_EN_CU0_SHIFT, INTR_EN);
	soctherm_writel(1<<INTR_EN_CD0_SHIFT, INTR_EN);

	return 0;
}

#ifdef CONFIG_THERMAL
static void soctherm_update(void)
{
	struct thermal_zone_device *dev = thz[THERM_CPU];
	long temp, trip_temp, low_temp = 0, high_temp = 128000;
	int count;

	if (!dev)
		return;

	if (!dev->passive)
		thermal_zone_device_update(dev);

	dev->ops->get_temp(dev, &temp);

	for (count = 0; count < dev->trips; count++) {
		dev->ops->get_trip_temp(dev, count, &trip_temp);

		if ((trip_temp >= temp) && (trip_temp < high_temp))
			high_temp = trip_temp;

		if ((trip_temp < temp) && (trip_temp > low_temp))
			low_temp = trip_temp;
	}

	soctherm_set_limits(THERM_CPU, low_temp/1000, high_temp/1000);
}

static int soctherm_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0)
		return 0;

	if (plat_data.passive[index].cdev == cdevice)
		return thermal_zone_bind_cooling_device(thz, 0, cdevice);

	return 0;
}

static int soctherm_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdevice)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0)
		return 0;

	if (plat_data.passive[index].cdev == cdevice)
		return thermal_zone_unbind_cooling_device(thz, 0, cdevice);

	return 0;
}

static int soctherm_get_temp(struct thermal_zone_device *thz,
					unsigned long *temp)
{
	int index = (int)thz->devdata;
	u32 r;

	if (index < TSENSE_SIZE) {
		r = soctherm_readl(TS_CPU0_STATUS1 +
					index * TS_CONFIG_STATUS_OFFSET);
		*temp = temp_translate(REG_GET(r, TS_CPU0_STATUS1_TEMP));
	} else {
		index -= TSENSE_SIZE;

		if (index == THERM_CPU || index == THERM_GPU)
			r = soctherm_readl(TS_TEMP1);
		else
			r = soctherm_readl(TS_TEMP2);

		if (index == THERM_CPU || index == THERM_MEM)
			*temp = temp_translate(REG_GET(r, TS_TEMP1_CPU_TEMP));
		else
			*temp = temp_translate(REG_GET(r, TS_TEMP1_GPU_TEMP));
	}

	return 0;
}

static int soctherm_get_trip_type(struct thermal_zone_device *thz,
					int trip,
					enum thermal_trip_type *type) {
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.passive[index].cdev)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static int soctherm_get_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long *temp) {
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.passive[index].cdev)
		return -EINVAL;

	*temp = plat_data.passive[index].trip_temp;
	return 0;
}

static int soctherm_set_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long temp)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;
	if (index < 0 || !plat_data.passive[index].cdev)
		return -EINVAL;

	plat_data.passive[index].trip_temp = temp;

	soctherm_update();

	return 0;
}

static struct thermal_zone_device_ops soctherm_ops = {
	.bind = soctherm_bind,
	.unbind = soctherm_unbind,
	.get_temp = soctherm_get_temp,
	.get_trip_type = soctherm_get_trip_type,
	.get_trip_temp = soctherm_get_trip_temp,
	.set_trip_temp = soctherm_set_trip_temp,
};
#else
static void soctherm_update(void)
{
}
#endif

static void soctherm_work_func(struct work_struct *work)
{
	soctherm_update();
}

static irqreturn_t soctherm_isr(int irq, void *arg_data)
{
	u32 r;

	queue_work(workqueue, &work);

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
	char name[64];

	/* Can only thermtrip with either GPU or MEM but not both */
	if (data->thermtrip[THERM_GPU] && data->thermtrip[THERM_MEM])
		return -EINVAL;


	memcpy(&plat_data, data, sizeof(struct soctherm_platform_data));

	/* Thermal Sensing programming */
	for (i = 0; i < TSENSE_SIZE; i++) {
		if (plat_data.sensor_data[i].enable) {
			soctherm_tsense_program(i, &plat_data.sensor_data[i]);
			sprintf(name, "%s-tsensor", sensor_names[i]);
#ifdef CONFIG_THERMAL
/* Let's avoid this for now */
#if 0
			/* Create a thermal zone device for each sensor */
			thermal_zone_device_register(
					name,
					0,
					0,
					(void *)i,
					&soctherm_ops,
					0, 0, 0, 0);
#endif
#endif
		}
	}

	/* Pdiv */
	r = soctherm_readl(TS_PDIV);
	r = REG_SET(r, TS_PDIV_CPU, data->sensor_data[TSENSE_CPU0].pdiv);
	r = REG_SET(r, TS_PDIV_GPU, data->sensor_data[TSENSE_GPU].pdiv);
	r = REG_SET(r, TS_PDIV_CPU, data->sensor_data[TSENSE_MEM0].pdiv);
	r = REG_SET(r, TS_PDIV_CPU, data->sensor_data[TSENSE_PLLX].pdiv);
	soctherm_writel(r, TS_PDIV);

#ifdef CONFIG_THERMAL
	for (i = 0; i < THERM_SIZE; i++) {
		sprintf(name, "%s-therm", therm_names[i]);
		thz[i] = thermal_zone_device_register(
					name,
					0x1,
					data->passive[i].cdev ? 1 : 0,
					(void *)TSENSE_SIZE + i,
					&soctherm_ops,
					data->passive[i].tc1,
					data->passive[i].tc2,
					data->passive[i].passive_delay,
					0);
	}
#endif

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
	r = REG_SET(0, THERMTRIP_CPU_EN, !!data->thermtrip[THERM_CPU]);
	r = REG_SET(r, THERMTRIP_GPU_EN, !!data->thermtrip[THERM_GPU]);
	r = REG_SET(r, THERMTRIP_MEM_EN, !!data->thermtrip[THERM_MEM]);
	r = REG_SET(r, THERMTRIP_TSENSE_EN, !!data->thermtrip[THERM_PLL]);
	r = REG_SET(r, THERMTRIP_CPU_THRESH, data->thermtrip[THERM_CPU]);
	r = REG_SET(r, THERMTRIP_GPUMEM_THRESH, data->thermtrip[THERM_GPU] |
						data->thermtrip[THERM_MEM]);
	r = REG_SET(r, THERMTRIP_TSENSE_THRESH, data->thermtrip[THERM_PLL]);
	soctherm_writel(r, THERMTRIP);

	/* Enable PMC to shutdown */
	r = pmc_readl(0x1b0);
	r |= 0x2;
	pmc_writel(r, 0x1b0);

	workqueue = create_singlethread_workqueue("soctherm");
	INIT_WORK(&work, soctherm_work_func);

	err = request_irq(INT_THERMAL, soctherm_isr, 0, "soctherm", NULL);
	if (err < 0)
		return -1;

	soctherm_update();

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

	r = soctherm_readl(THERMTRIP);
	seq_printf(s, "THERMTRIP: 0x%x\n", r);
	state = REG_GET(r, THERMTRIP_CPU_THRESH);
	seq_printf(s, "THERMTRIP_CPU_THRESH: %d ", state);
	state = REG_GET(r, THERMTRIP_CPU_EN);
	seq_printf(s, "%d\n", state);

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
