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
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>

#include <mach/tegra_fuse.h>
#include <mach/iomap.h>

#include "tegra11_soctherm.h"

/* Min temp granularity specified as X in 2^X.
 * -1: Hi precision option: 2^-1 = 0.5C (default)
 *  0: Lo precision option: 2^0  = 1.0C (for use if therm_a/b overflows 16 bits)
 */
static int soc_therm_precision = -1;

#define TS_TSENSE_REGS_SIZE		0x20
#define TS_TSENSE_REG_OFFSET(reg, ts)	((reg) + ((ts) * TS_TSENSE_REGS_SIZE))

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
#define INTR_STATUS_CD0_SHIFT		9
#define INTR_STATUS_CU0_SHIFT		8

#define INTR_EN			0x88
#define INTR_EN_MU0_SHIFT	24
#define INTR_EN_MD0_SHIFT	25
#define INTR_EN_CU0_SHIFT	8
#define INTR_EN_CD0_SHIFT	9

#define INTR_DIS		0x8c
#define LOCK_CTL		0x90
#define STATS_CTL		0x94

#define THROT_GLOBAL_CFG		0x400

#define CPU_PSKIP_STATUS			0x418
#define CPU_PSKIP_STATUS_M_SHIFT		12
#define CPU_PSKIP_STATUS_M_MASK			0xff
#define CPU_PSKIP_STATUS_N_SHIFT		4
#define CPU_PSKIP_STATUS_N_MASK			0xff
#define CPU_PSKIP_STATUS_ENABLED_SHIFT		0
#define CPU_PSKIP_STATUS_ENABLED_MASK		0x1

#define THROT_PRIORITY_LOCK			0x424
#define THROT_PRIORITY_LOCK_PRIORITY_SHIFT	0
#define THROT_PRIORITY_LOCK_PRIORITY_MASK	0xff

#define THROT_STATUS				0x428
#define THROT_STATUS_BREACH_SHIFT		12
#define THROT_STATUS_BREACH_MASK		0x1
#define THROT_STATUS_STATE_SHIFT		4
#define THROT_STATUS_STATE_MASK			0xff
#define THROT_STATUS_ENABLED_SHIFT		0
#define THROT_STATUS_ENABLED_MASK		0x1

#define THROT_PSKIP_CTRL_LITE_CPU			0x430
#define THROT_PSKIP_CTRL_ENABLE_SHIFT		31
#define THROT_PSKIP_CTRL_ENABLE_MASK		0x1
#define THROT_PSKIP_CTRL_DIVIDEND_SHIFT	8
#define THROT_PSKIP_CTRL_DIVIDEND_MASK		0xff
#define THROT_PSKIP_CTRL_DIVISOR_SHIFT		0
#define THROT_PSKIP_CTRL_DIVISOR_MASK		0xff

#define THROT_PSKIP_RAMP_LITE_CPU			0x434
#define THROT_PSKIP_RAMP_DURATION_SHIFT	8
#define THROT_PSKIP_RAMP_DURATION_MASK		0xffff
#define THROT_PSKIP_RAMP_STEP_SHIFT		0
#define THROT_PSKIP_RAMP_STEP_MASK		0xff

#define THROT_LITE_PRIORITY			0x444
#define THROT_LITE_PRIORITY_PRIORITY_SHIFT	0
#define THROT_LITE_PRIORITY_PRIORITY_MASK	0xff

#define THROT_OFFSET				0x30

#define FUSE_BASE_CP_SHIFT	0
#define FUSE_BASE_CP_MASK	0x3ff
#define FUSE_BASE_FT_SHIFT	16
#define FUSE_BASE_FT_MASK	0x7ff
#define FUSE_SHIFT_CP_SHIFT	10
#define FUSE_SHIFT_CP_MASK	0x3f
#define FUSE_SHIFT_CP_BITS	6
#define FUSE_SHIFT_FT_SHIFT	27
#define FUSE_SHIFT_FT_MASK	0x1f
#define FUSE_SHIFT_FT_BITS	5

#define FUSE_TSENSOR_CALIB_FT_SHIFT	13
#define FUSE_TSENSOR_CALIB_FT_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_CP_SHIFT	0
#define FUSE_TSENSOR_CALIB_CP_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_BITS		13

#define THROT_PSKIP_CTRL(throt, dev)		(THROT_PSKIP_CTRL_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define THROT_PSKIP_RAMP(throt, dev)		(THROT_PSKIP_RAMP_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))

#define PSKIP_CTRL_OC1_CPU			0x490

#define REG_SET(r,_name,val) \
	((r)&~(_name##_MASK<<_name##_SHIFT))|(((val)&_name##_MASK)<<_name##_SHIFT)

#define REG_GET(r,_name) \
	(((r)&(_name##_MASK<<_name##_SHIFT))>>_name##_SHIFT)

#define MAKE_SIGNED32(val, nb) \
	((s32)(val) << (32 - (nb)) >> (32 - (nb)))

static void __iomem *reg_soctherm_base = IO_ADDRESS(TEGRA_SOCTHERM_BASE);
static void __iomem *pmc_base = IO_ADDRESS(TEGRA_PMC_BASE);
static void __iomem *clk_reset_base = IO_ADDRESS(TEGRA_CLK_RESET_BASE);

#define clk_reset_writel(value, reg) \
	__raw_writel(value, clk_reset_base + (reg))
#define clk_reset_readl(reg) __raw_readl(clk_reset_base + (reg))

#define pmc_writel(value, reg) __raw_writel(value, pmc_base + (reg))
#define pmc_readl(reg) __raw_readl(pmc_base + (reg))

#define soctherm_writel(value, reg) \
	__raw_writel(value, reg_soctherm_base + (reg))
#define soctherm_readl(reg) \
	__raw_readl(reg_soctherm_base + (reg))

#define max4(a, b, c, d)	max(max(a, b), max(c, d))

static struct soctherm_platform_data plat_data;
#ifdef CONFIG_THERMAL
static struct thermal_zone_device *thz[THERM_SIZE];
#endif
static struct workqueue_struct *workqueue;
static struct work_struct work;

static u32 fuse_calib_base_cp;
static u32 fuse_calib_base_ft;
static s32 actual_temp_cp;
static s32 actual_temp_ft;

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

static int sensor2tsensorcalib[] = {
	[TSENSE_CPU0] = 0,
	[TSENSE_CPU1] = 1,
	[TSENSE_CPU2] = 2,
	[TSENSE_CPU3] = 3,
	[TSENSE_MEM0] = 5,
	[TSENSE_MEM1] = 6,
	[TSENSE_GPU]  = 4,
	[TSENSE_PLLX] = 7,
};

static inline s64 div64_s64_precise(s64 a, s32 b)
{
	s64 r, al;

	/* scale up for increased precision in division */
	al = a << 16;

	r = div64_s64((al * 2) + 1, 2 * b);
	return r >> 16;
}

static inline long temp_translate(int readback)
{
	int abs = readback >> 8;
	int lsb = (readback & 0x80) >> 7;
	int sign = readback & 0x01 ? -1 : 1;

	if (soc_therm_precision == -1)
		return (abs * 1000 + lsb * 500) * sign;
	else
		return (abs * 2000 + lsb * 1000) * sign;
}

#ifdef CONFIG_THERMAL
static int soctherm_set_limits(enum soctherm_therm_id therm,
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

static void soctherm_update(void)
{
	long temp, trip_temp, low_temp = 0, high_temp = 128000;
	int i, count;

	for (i = 0; i < THERM_SIZE; i++) {
		if (!thz[i])
			continue;

		if (!thz[i]->passive)
			thermal_zone_device_update(thz[i]);

		thz[i]->ops->get_temp(thz[i], &temp);
		for (count = 0; count < thz[i]->trips; count++) {
			thz[i]->ops->get_trip_temp(thz[i], count, &trip_temp);

			if ((trip_temp >= temp) && (trip_temp < high_temp))
				high_temp = trip_temp;

			if ((trip_temp < temp) && (trip_temp > low_temp))
				low_temp = trip_temp -
					plat_data.therm[i].hysteresis;
		}
	}

	soctherm_set_limits(THERM_CPU, low_temp/1000, high_temp/1000);
}

static int soctherm_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.therm[index].cdev_type)
		return 0;

	if (!strcmp(plat_data.therm[index].cdev_type, cdev->type))
		return thermal_zone_bind_cooling_device(thz, 0, cdev,
							THERMAL_NO_LIMIT,
							THERMAL_NO_LIMIT);

	return 0;
}

static int soctherm_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.therm[index].cdev_type)
		return 0;

	if (!strcmp(plat_data.therm[index].cdev_type, cdev->type))
		return thermal_zone_unbind_cooling_device(thz, 0, cdev);

	return 0;
}

static int soctherm_get_temp(struct thermal_zone_device *thz,
					unsigned long *temp)
{
	int index = (int)thz->devdata;
	u32 r;

	if (index < TSENSE_SIZE) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS1,
							index));
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

		plat_data.therm[index].etemp = *temp;
	}

	return 0;
}

static int soctherm_get_trip_type(struct thermal_zone_device *thz,
					int trip,
					enum thermal_trip_type *type) {
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.therm[index].cdev_type)
		return -EINVAL;

	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static int soctherm_get_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long *temp) {
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.therm[index].cdev_type)
		return -EINVAL;

	*temp = plat_data.therm[index].trip_temp;
	return 0;
}

static int soctherm_set_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long temp)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;

	if (index < 0 || !plat_data.therm[index].cdev_type)
		return -EINVAL;

	plat_data.therm[index].trip_temp = temp;
	soctherm_update();
	return 0;
}

static int soctherm_get_trend(struct thermal_zone_device *thz,
				int trip,
				enum thermal_trend *trend)
{
	int index = ((int)thz->devdata) - TSENSE_SIZE;
	struct soctherm_therm *trip_state;

	if (index < 0 || !plat_data.therm[index].cdev_type)
		return -EINVAL;

	trip_state = &plat_data.therm[index];

	/* always THERMAL_TRIP_PASSIVE */
	if (trip_state->etemp > trip_state->trip_temp)
		*trend = THERMAL_TREND_RAISING;
	else
		*trend = THERMAL_TREND_DROPPING;

	return 0;
}

static struct thermal_zone_device_ops soctherm_ops = {
	.bind = soctherm_bind,
	.unbind = soctherm_unbind,
	.get_temp = soctherm_get_temp,
	.get_trip_type = soctherm_get_trip_type,
	.get_trip_temp = soctherm_get_trip_temp,
	.set_trip_temp = soctherm_set_trip_temp,
	.get_trend = soctherm_get_trend,
};

static int __init soctherm_thermal_init(void)
{
	char name[64];
	int i;

	for (i = 0; i < TSENSE_SIZE; i++) {
		if (plat_data.sensor_data[i].zone_enable) {
			sprintf(name, "%s-tsensor", sensor_names[i]);
			/* Create a thermal zone device for each sensor */
			thermal_zone_device_register(
					name,
					0,
					0,
					(void *)i,
					&soctherm_ops,
					NULL,
					0,
					0);
		}
	}

	for (i = 0; i < THERM_SIZE; i++) {
		if (plat_data.therm[i].zone_enable) {
			sprintf(name, "%s-therm", therm_names[i]);
			thz[i] = thermal_zone_device_register(
					name,
					plat_data.therm[i].cdev_type ? 1 : 0,
					plat_data.therm[i].cdev_type ? 0x1 : 0,
					(void *)TSENSE_SIZE + i,
					&soctherm_ops,
					NULL,
					plat_data.therm[i].passive_delay,
					0);
		}
	}

	soctherm_update();
	return 0;
}
module_init(soctherm_thermal_init);

#else
static void soctherm_update(void)
{
}
#endif

static void soctherm_work_func(struct work_struct *work)
{
	u32 status;

	status = soctherm_readl(INTR_STATUS);
	soctherm_writel(status, INTR_STATUS);

	soctherm_update();
}

static irqreturn_t soctherm_isr(int irq, void *arg_data)
{
	u32 r;

	queue_work(workqueue, &work);

	r = soctherm_readl(INTR_STATUS);
	soctherm_writel(r, INTR_DIS);

	return IRQ_HANDLED;
}

void tegra11_soctherm_throttle_program(enum soctherm_throttle_id throttle,
					struct soctherm_throttle *data)
{
	u32 r;
	int i;
	struct soctherm_throttle_dev *dev;

	for (i = 0; i < THROTTLE_DEV_SIZE; i++) {
		dev = &data->devs[i];

		r = soctherm_readl(THROT_PSKIP_CTRL(throttle, i));
		r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
		r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, dev->dividend);
		r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, dev->divisor);
		soctherm_writel(r, THROT_PSKIP_CTRL(throttle, i));

		r = soctherm_readl(THROT_PSKIP_RAMP(throttle, i));
		r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, dev->duration);
		r = REG_SET(r, THROT_PSKIP_RAMP_STEP, dev->step);
		soctherm_writel(r, THROT_PSKIP_RAMP(throttle, i));
	}

	r = soctherm_readl(THROT_PRIORITY_LOCK);
	if (r < data->priority) {
		r = REG_SET(0, THROT_PRIORITY_LOCK_PRIORITY, data->priority);
		soctherm_writel(r, THROT_PRIORITY_LOCK);
	}

	r = REG_SET(0, THROT_LITE_PRIORITY_PRIORITY, data->priority);
	soctherm_writel(r, THROT_LITE_PRIORITY + THROT_OFFSET * throttle);
}

static void __init soctherm_tsense_program(enum soctherm_sense sensor,
						struct soctherm_sensor *data)
{
	u32 r;

	r = REG_SET(0, TS_CPU0_CONFIG0_TALL, data->tall);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, sensor));

	r = REG_SET(0, TS_CPU0_CONFIG1_TIDDQ, data->tiddq);
	r = REG_SET(r, TS_CPU0_CONFIG1_EN, 1);
	r = REG_SET(r, TS_CPU0_CONFIG1_TEN_COUNT, data->ten_count);
	r = REG_SET(r, TS_CPU0_CONFIG1_TSAMPLE, data->tsample);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, sensor));
}

static int soctherm_clk_enable(bool enable)
{
	struct clk *soctherm_clk;
	struct clk *pllp_clk;
	struct clk *ts_clk;
	struct clk *clk_m;
	unsigned long soctherm_clk_rate, ts_clk_rate;

	soctherm_clk = clk_get_sys("soc_therm", NULL);
	if (IS_ERR(soctherm_clk))
		return -1;

	ts_clk = clk_get_sys("tegra-tsensor", NULL);
	if (IS_ERR(ts_clk))
		return -1;

	if (enable) {
		pllp_clk = clk_get_sys(NULL, "pll_p");
		if (IS_ERR(pllp_clk))
			return -1;

		clk_m = clk_get_sys(NULL, "clk_m");
		if (IS_ERR(clk_m))
			return -1;

		clk_enable(soctherm_clk);
		soctherm_clk_rate = plat_data.soctherm_clk_rate;
		if (clk_get_parent(soctherm_clk) != pllp_clk)
			if (clk_set_parent(soctherm_clk, pllp_clk))
				return -1;
		if (clk_get_rate(pllp_clk) != soctherm_clk_rate)
			if (clk_set_rate(soctherm_clk, soctherm_clk_rate))
				return -1;

		clk_enable(ts_clk);
		ts_clk_rate = plat_data.tsensor_clk_rate;
		if (clk_get_parent(ts_clk) != clk_m)
			if (clk_set_parent(ts_clk, clk_m))
				return -1;
		if (clk_get_rate(clk_m) != ts_clk_rate)
			if (clk_set_rate(ts_clk, ts_clk_rate))
				return -1;
	} else {
		clk_disable(soctherm_clk);
		clk_disable(ts_clk);
		clk_put(soctherm_clk);
		clk_put(ts_clk);
	}

	return 0;
}

static void soctherm_fuse_read_vsensor(void)
{
	u32 value;
	s32 calib_cp, calib_ft;

	tegra_fuse_get_vsensor_calib(&value);

	/* Extract bits */
	fuse_calib_base_cp = REG_GET(value, FUSE_BASE_CP);
	fuse_calib_base_ft = REG_GET(value, FUSE_BASE_FT);

	/* Extract bits and convert to signed 2's complement */
	calib_cp = REG_GET(value, FUSE_SHIFT_CP);
	calib_cp = MAKE_SIGNED32(calib_cp, FUSE_SHIFT_CP_BITS);

	calib_ft = REG_GET(value, FUSE_SHIFT_FT);
	calib_ft = MAKE_SIGNED32(calib_ft, FUSE_SHIFT_FT_BITS);

	/* default: HI precision: use fuse_temp in 0.5C */
	actual_temp_cp = 2 * 25 + calib_cp;
	actual_temp_ft = 2 * 90 + calib_ft;

	/* adjust: for LO precision: use fuse_temp in 1C */
	if (soc_therm_precision != -1) {
		actual_temp_cp /= 2;
		actual_temp_ft /= 2;
	}
}

static void soctherm_fuse_read_tsensor(enum soctherm_sense sensor)
{
	u32 r, value;
	s32 calib, delta_sens, delta_temp;
	s16 therm_a, therm_b;
	s32 div, mult, actual_tsensor_ft, actual_tsensor_cp;

	tegra_fuse_get_tsensor_calib(sensor2tsensorcalib[sensor], &value);

	/* Extract bits and convert to signed 2's complement */
	calib = REG_GET(value, FUSE_TSENSOR_CALIB_FT);
	calib = MAKE_SIGNED32(calib, FUSE_TSENSOR_CALIB_BITS);
	actual_tsensor_ft = (fuse_calib_base_ft * 32) + calib;

	calib = REG_GET(value, FUSE_TSENSOR_CALIB_CP);
	calib = MAKE_SIGNED32(calib, FUSE_TSENSOR_CALIB_BITS);
	actual_tsensor_cp = (fuse_calib_base_cp * 64) + calib;

	mult = plat_data.sensor_data[sensor].pdiv * 655;
	div = plat_data.sensor_data[sensor].tsample * 10;

	delta_sens = actual_tsensor_ft - actual_tsensor_cp;
	delta_temp = actual_temp_ft - actual_temp_cp;

	therm_a = div64_s64_precise((s64)delta_temp * (1LL << 13) * mult,
				    (s64)delta_sens * div);

	therm_b = div64_s64_precise((((s64)actual_tsensor_ft * actual_temp_cp) -
				     ((s64)actual_tsensor_cp * actual_temp_ft)),
				    (s64)delta_sens);

	r = REG_SET(0, TS_CPU0_CONFIG2_THERM_A, therm_a);
	r = REG_SET(r, TS_CPU0_CONFIG2_THERM_B, therm_b);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, sensor));
}

static int soctherm_init_platform_data(void)
{
	struct soctherm_therm *therm;
	int i;
	u32 r;
	u32 reg_off;

	therm = plat_data.therm;

	/* Can only thermtrip with either GPU or MEM but not both */
	if (therm[THERM_GPU].thermtrip && therm[THERM_MEM].thermtrip)
		return -EINVAL;

	if (soctherm_clk_enable(true) < 0)
		BUG();

	/* Pdiv */
	r = soctherm_readl(TS_PDIV);
	r = REG_SET(r, TS_PDIV_CPU, plat_data.sensor_data[TSENSE_CPU0].pdiv);
	r = REG_SET(r, TS_PDIV_GPU, plat_data.sensor_data[TSENSE_GPU].pdiv);
	r = REG_SET(r, TS_PDIV_MEM, plat_data.sensor_data[TSENSE_MEM0].pdiv);
	r = REG_SET(r, TS_PDIV_PLLX, plat_data.sensor_data[TSENSE_PLLX].pdiv);
	soctherm_writel(r, TS_PDIV);

	/* Thermal Sensing programming */
	soctherm_fuse_read_vsensor();
	for (i = 0; i < TSENSE_SIZE; i++) {
		if (plat_data.sensor_data[i].sensor_enable) {
			soctherm_tsense_program(i, &plat_data.sensor_data[i]);
			soctherm_fuse_read_tsensor(i);
		}
	}

	for (i = 0; i < THERM_SIZE; i++) {
		if (plat_data.therm[i].hw_backstop) {
			reg_off = TS_TSENSE_REG_OFFSET(CTL_LVL0_CPU0, 1)
				+ i * 4;
			r = soctherm_readl(reg_off);
			r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH,
					plat_data.therm[i].hw_backstop);
			r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH,
					plat_data.therm[i].hw_backstop - 2);
			r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
			/* Heavy throttling */
			r = REG_SET(r, CTL_LVL0_CPU0_CPU_THROT, 2);
			soctherm_writel(r, reg_off);
		}
	}

	/* Enable Level 0 */
	r = soctherm_readl(CTL_LVL0_CPU0);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, !!therm[THERM_CPU].trip_temp);
	soctherm_writel(r, CTL_LVL0_CPU0);

	/* Thermtrip */
	r = REG_SET(0, THERMTRIP_CPU_EN, !!therm[THERM_CPU].thermtrip);
	r = REG_SET(r, THERMTRIP_GPU_EN, !!therm[THERM_GPU].thermtrip);
	r = REG_SET(r, THERMTRIP_MEM_EN, !!therm[THERM_MEM].thermtrip);
	r = REG_SET(r, THERMTRIP_TSENSE_EN, !!therm[THERM_PLL].thermtrip);
	r = REG_SET(r, THERMTRIP_CPU_THRESH, therm[THERM_CPU].thermtrip);
	r = REG_SET(r, THERMTRIP_GPUMEM_THRESH, therm[THERM_GPU].thermtrip |
						therm[THERM_MEM].thermtrip);
	r = REG_SET(r, THERMTRIP_TSENSE_THRESH, therm[THERM_PLL].thermtrip);
	soctherm_writel(r, THERMTRIP);

	/* Enable PMC to shutdown */
	r = pmc_readl(0x1b0);
	r |= 0x2;
	pmc_writel(r, 0x1b0);

	/* Throttling */
	for (i = 0; i < THROTTLE_SIZE; i++)
		tegra11_soctherm_throttle_program(i, &plat_data.throttle[i]);

	r = clk_reset_readl(0x24);
	r |= (1 << 30);
	clk_reset_writel(r, 0x24);

	return 0;
}

static int soctherm_suspend(void)
{
	soctherm_writel((u32)-1, INTR_DIS);
	soctherm_clk_enable(false);
	disable_irq(INT_THERMAL);
	cancel_work_sync(&work);

	return 0;
}

static int soctherm_resume(void)
{
	soctherm_clk_enable(true);
	enable_irq(INT_THERMAL);
	soctherm_init_platform_data();
	soctherm_update();

	return 0;
}

static int soctherm_pm_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		soctherm_suspend();
		break;
	case PM_POST_SUSPEND:
		soctherm_resume();
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block soctherm_nb = {
	.notifier_call = soctherm_pm_notify,
};

int __init tegra11_soctherm_init(struct soctherm_platform_data *data)
{
	int err;

	register_pm_notifier(&soctherm_nb);

	memcpy(&plat_data, data, sizeof(struct soctherm_platform_data));

	if (soctherm_clk_enable(true) < 0)
		BUG();

	soctherm_init_platform_data();

	/* enable interrupts */
	workqueue = create_singlethread_workqueue("soctherm");
	INIT_WORK(&work, soctherm_work_func);

	err = request_irq(INT_THERMAL, soctherm_isr, 0, "soctherm", NULL);
	if (err < 0)
		return -1;
	return 0;
}


#ifdef CONFIG_DEBUG_FS
static int regs_show(struct seq_file *s, void *data)
{
	u32 r;
	u32 state;
	long tcpu[TSENSE_SIZE];
	int i, level;

	seq_printf(s, "-----TSENSE (precision %s)-----\n",
		   soc_therm_precision == -1 ? "Hi" : "Lo");
	for (i = 0; i < TSENSE_SIZE; i++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		seq_printf(s, "%s: ", sensor_names[i]);

		seq_printf(s, "En(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TIDDQ);
		seq_printf(s, "tiddq(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TEN_COUNT);
		seq_printf(s, "ten_count(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TSAMPLE);
		seq_printf(s, "tsample(%d) ", state);

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS0, i));
		state = REG_GET(r, TS_CPU0_STATUS0_VALID);
		seq_printf(s, "Capture(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
		seq_printf(s, "%d) ", state);


		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS1, i));
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID);
		seq_printf(s, "Temp(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
		seq_printf(s, "%ld) ", tcpu[i] = temp_translate(state));


		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, i));
		state = REG_GET(r, TS_CPU0_CONFIG0_TALL);
		seq_printf(s, "Tall(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TCALC_OVER);
		seq_printf(s, "Over(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_OVER);
		seq_printf(s, "%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_CPTR_OVER);
		seq_printf(s, "%d) ", state);

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, i));
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
	seq_printf(s, "Temperature: CPU(%ld) [Max(%ld)%c] ",
		   temp_translate(state),
		   max4(tcpu[0], tcpu[1], tcpu[2], tcpu[3]),
		   max4(tcpu[0], tcpu[1], tcpu[2], tcpu[3]) ==
		   temp_translate(state) ? '=' : '!');
	state = REG_GET(r, TS_TEMP1_GPU_TEMP);
	seq_printf(s, " GPU(%ld) ", temp_translate(state));
	r = soctherm_readl(TS_TEMP2);
	state = REG_GET(r, TS_TEMP2_MEM_TEMP);
	seq_printf(s, " MEM(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP2_PLLX_TEMP);
	seq_printf(s, " PLLX(%ld)\n\n", temp_translate(state));

	for (i = 0; i < THERM_SIZE; i++) {
		seq_printf(s, "%s:\n", therm_names[i]);

		for (level = 0; level < 4; level++) {
			r = soctherm_readl(TS_TSENSE_REG_OFFSET(CTL_LVL0_CPU0,
								i) + level * 4);
			state = REG_GET(r, CTL_LVL0_CPU0_UP_THRESH);
			seq_printf(s, "    Up/Dn(%d/", state);
			state = REG_GET(r, CTL_LVL0_CPU0_DN_THRESH);
			seq_printf(s, "%d) ", state);
			state = REG_GET(r, CTL_LVL0_CPU0_EN);
			seq_printf(s, "En(%d) ", state);
			state = REG_GET(r, CTL_LVL0_CPU0_STATUS);
			seq_printf(s, "Status(%s)\n",
				   state == 0 ? "LO" :
				   state == 1 ? "in" :
				   state == 2 ? "??" : "HI");
		}
	}

	r = soctherm_readl(INTR_STATUS);
	state = REG_GET(r, INTR_STATUS_MD0);
	seq_printf(s, "MD0: %d\n", state);
	state = REG_GET(r, INTR_STATUS_MU0);
	seq_printf(s, "MU0: %d\n", state);

	r = soctherm_readl(THERMTRIP);
	state = REG_GET(r, THERMTRIP_CPU_THRESH);
	seq_printf(s, "THERMTRIP_CPU_THRESH: %d ", state);
	state = REG_GET(r, THERMTRIP_CPU_EN);
	seq_printf(s, "%d\n", state);


	seq_printf(s, "\n-----THROTTLE-----\n");

	r = soctherm_readl(THROT_GLOBAL_CFG);
	seq_printf(s, "GLOBAL CONFIG: 0x%x\n", r);

	r = soctherm_readl(THROT_STATUS);
	state = REG_GET(r, THROT_STATUS_BREACH);
	seq_printf(s, "THROT STATUS: breach(%d) ", state);
	state = REG_GET(r, THROT_STATUS_STATE);
	seq_printf(s, "state(%d) ", state);
	state = REG_GET(r, THROT_STATUS_ENABLED);
	seq_printf(s, "enabled(%d)\n", state);

	r = soctherm_readl(CPU_PSKIP_STATUS);
	state = REG_GET(r, CPU_PSKIP_STATUS_M);
	seq_printf(s, "CPU PSKIP: M(%d) ", state);
	state = REG_GET(r, CPU_PSKIP_STATUS_N);
	seq_printf(s, "N(%d) ", state);
	state = REG_GET(r, CPU_PSKIP_STATUS_ENABLED);
	seq_printf(s, "enabled(%d)\n", state);

	r = soctherm_readl(THROT_PSKIP_CTRL(THROTTLE_HEAVY, THROTTLE_DEV_CPU));
	state = REG_GET(r, THROT_PSKIP_CTRL_ENABLE);
	seq_printf(s, "CPU PSKIP HEAVY: enabled(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_CTRL_DIVIDEND);
	seq_printf(s, "dividend(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_CTRL_DIVISOR);
	seq_printf(s, "divisor(%d) ", state);

	r = soctherm_readl(THROT_PSKIP_RAMP(THROTTLE_HEAVY, THROTTLE_DEV_CPU));
	state = REG_GET(r, THROT_PSKIP_RAMP_DURATION);
	seq_printf(s, "duration(%d) ", state);
	state = REG_GET(r, THROT_PSKIP_RAMP_STEP);
	seq_printf(s, "step(%d)\n", state);

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
	debugfs_create_file("regs", 0644, tegra_soctherm_root,
				NULL, &regs_fops);
	return 0;
}
late_initcall(soctherm_debug_init);
#endif
