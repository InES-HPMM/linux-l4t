/*
 * arch/arm/mach-tegra/tegra14_dvfs.c
 *
 * Copyright (C) 2012-2013 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/err.h>

#include "clock.h"
#include "dvfs.h"
#include "fuse.h"
#include "board.h"
#include "tegra_cl_dvfs.h"

static bool tegra_dvfs_cpu_disabled;
static bool tegra_dvfs_core_disabled;

#define KHZ 1000
#define MHZ 1000000

/* FIXME: need tegra14 step */
#define VDD_SAFE_STEP			100

/* FIXME: need to clean-up, once thermal DVFS working */
#define THERMAL_DVFS_ENABLE 0

#if THERMAL_DVFS_ENABLE
static int dvfs_temperatures[] = { 20, };

static struct tegra_cooling_device cpu_dfll_cdev = {
	.cdev_type = "cpu_dfll",
	.trip_temperatures = dvfs_temperatures,
	.trip_temperatures_num = ARRAY_SIZE(dvfs_temperatures),
};

static struct tegra_cooling_device cpu_pll_cdev = {
	.cdev_type = "cpu_pll",
	.trip_temperatures = dvfs_temperatures,
	.trip_temperatures_num = ARRAY_SIZE(dvfs_temperatures),
};

static struct tegra_cooling_device core_cdev = {
	.cdev_type = "core",
	.trip_temperatures = dvfs_temperatures,
	.trip_temperatures_num = ARRAY_SIZE(dvfs_temperatures),
};
#endif

static struct dvfs_rail tegra14_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1400,
	.min_millivolts = 730,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
#if THERMAL_DVFS_ENABLE
	.min_millivolts_cold = 1000,
	.dfll_mode_cdev = &cpu_dfll_cdev,
	.pll_mode_cdev = &cpu_pll_cdev,
#endif
};

static struct dvfs_rail tegra14_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1400,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
#if THERMAL_DVFS_ENABLE
	.min_millivolts_cold = 950,
	.pll_mode_cdev = &core_cdev,
#endif
};

static struct dvfs_rail *tegra14_dvfs_rails[] = {
	&tegra14_dvfs_rail_vdd_cpu,
	&tegra14_dvfs_rail_vdd_core,
};

/* default cvb alignment on Tegra14 - 10mV */
int __attribute__((weak)) tegra_get_cvb_alignment_uV(void)
{
	return 10000;
}

/* CPU DVFS tables */
static struct cpu_cvb_dvfs cpu_cvb_dvfs_table[] = {
	{
		.speedo_id = 0,
		.process_id = 0,
		.dfll_tune_data  = {
			.tune0		= 0x0041061F,
			.tune0_high_mv	= 0x0041001F,
			.tune1		= 0x00000007,
			.droop_rate_min = 1000000,
			.tune_high_min_millivolts = 900,
			.min_millivolts = 730,
		},
		.max_mv = 1250,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f       dfll: c0,     c1,   c2  pll:  c0,   c1,    c2 */
			{ 408000, { 2425141,  -136464,   2699}, {  851885,    0,    0} },
			{ 510000, { 2457995,  -137794,   2699}, {  859934,    0,    0} },
			{ 612000, { 2493741,  -139134,   2699}, {  870653,    0,    0} },
			{ 714000, { 2532375,  -140474,   2699}, {  884041,    0,    0} },
			{ 816000, { 2573898,  -141814,   2699}, {  900099,    0,    0} },
			{ 918000, { 2618310,  -143154,   2699}, {  918826,    0,    0} },
			{1020000, { 2665611,  -144494,   2699}, {  940222,    0,    0} },
			{1122000, { 2715800,  -145834,   2699}, {  964288,    0,    0} },
			{1224000, { 2768873,  -147164,   2699}, {  991023,    0,    0} },
			{1326000, { 2824840,  -148504,   2699}, { 1020427,    0,    0} },
			{1428000, { 2883694,  -149844,   2699}, { 1052500,    0,    0} },
			{1530000, { 2945438,  -151184,   2699}, { 1087244,    0,    0} },
			{1632000, { 3010070,  -152524,   2699}, { 1124896,    0,    0} },
			{1734000, { 3077591,  -153864,   2699}, { 1164978,    0,    0} },
			{1836000, { 3148000,  -155204,   2699}, { 1207729,    0,    0} },
			{1938000, { 3221295,  -156534,   2699}, { 1207729,    0,    0} },
			{2014500, { 3277787,  -157534,   2699}, { 1207729,    0,    0} },
			{      0, {      0,      0,   0}, {      0,    0,    0} },
		},
	},
	{
		.speedo_id = 0,
		.process_id = 1,
		.dfll_tune_data  = {
			.tune0		= 0x0041061F,
			.tune0_high_mv	= 0x0041001F,
			.tune1		= 0x00000007,
			.droop_rate_min = 1000000,
			.tune_high_min_millivolts = 900,
			.min_millivolts = 730,
		},
		.max_mv = 1250,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f       dfll: c0,     c1,   c2  pll:  c0,   c1,    c2 */
			{ 408000, { 2425141,  -136464,   2699}, {  851885,    0,    0} },
			{ 510000, { 2457995,  -137794,   2699}, {  859934,    0,    0} },
			{ 612000, { 2493741,  -139134,   2699}, {  870653,    0,    0} },
			{ 714000, { 2532375,  -140474,   2699}, {  884041,    0,    0} },
			{ 816000, { 2573898,  -141814,   2699}, {  900099,    0,    0} },
			{ 918000, { 2618310,  -143154,   2699}, {  918826,    0,    0} },
			{1020000, { 2665611,  -144494,   2699}, {  940222,    0,    0} },
			{1122000, { 2715800,  -145834,   2699}, {  964288,    0,    0} },
			{1224000, { 2768873,  -147164,   2699}, {  991023,    0,    0} },
			{1326000, { 2824840,  -148504,   2699}, { 1020427,    0,    0} },
			{1428000, { 2883694,  -149844,   2699}, { 1052500,    0,    0} },
			{1530000, { 2945438,  -151184,   2699}, { 1087244,    0,    0} },
			{1632000, { 3010070,  -152524,   2699}, { 1124896,    0,    0} },
			{1734000, { 3077591,  -153864,   2699}, { 1164978,    0,    0} },
			{1836000, { 3148000,  -155204,   2699}, { 1207729,    0,    0} },
			{1938000, { 3221295,  -156534,   2699}, { 1207729,    0,    0} },
			{2014500, { 3277787,  -157534,   2699}, { 1207729,    0,    0} },
			{2116500, { 3356126,  -158874,   2699}, { 1207729,    0,    0} },
			{      0, {      0,      0,   0}, {      0,    0,    0} },
		},
	},
};

static int cpu_millivolts[MAX_DVFS_FREQS];
static int cpu_dfll_millivolts[MAX_DVFS_FREQS];

static struct dvfs cpu_dvfs = {
	.clk_name	= "cpu_g",
	.millivolts	= cpu_millivolts,
	.dfll_millivolts = cpu_dfll_millivolts,
	.auto_dvfs	= true,
	.dvfs_rail	= &tegra14_dvfs_rail_vdd_cpu,
};

/* Core DVFS tables */
static const int core_millivolts[MAX_DVFS_FREQS] = {
	800, 850, 900, 950, 1000, 1050, 1100, 1150, 1200, 1250};

#define CORE_DVFS(_clk_name, _speedo_id, _process_id, _auto, _mult, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= _process_id,			\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra14_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
#ifndef CONFIG_TEGRA_SIMULATION_PLATFORM
	/* FIXME: This is based on T148_Safe_DVFS.xlsx */
	/* Core voltages (mV):		             800,    850,    900,    950,   1000,   1050,   1100,   1150,   1200,   1250 */
	CORE_DVFS("cpu_lp",  -1, -1, 1, KHZ,      201600, 201600, 201600, 211200, 211200, 211200, 710400, 710400, 787200, 806400),

	CORE_DVFS("3d",      -1, -1, 1, KHZ,       96000,  96000,  96000, 105600, 105600, 105600, 576000, 576000, 633600, 710400),
	CORE_DVFS("2d",      -1, -1, 1, KHZ,       96000,  96000,  96000, 105600, 105600, 105600, 576000, 576000, 633600, 710400),
	CORE_DVFS("epp",      -1, -1, 1, KHZ,      96000,  96000,  96000, 105600, 105600, 105600, 576000, 576000, 633600, 710400),

	CORE_DVFS("se",      -1, -1, 1, KHZ,       96000,  96000,  96000, 105600, 105600, 105600, 441600, 441600, 518400, 537600),
	CORE_DVFS("vde",      -1, -1, 1, KHZ,      96000,  96000,  96000, 105600, 105600, 105600, 441600, 441600, 518400, 537600),

	CORE_DVFS("msenc",   -1, -1, 1, KHZ,       72000,  72000,  72000,  77000,  77000,  77000, 360000, 360000, 408000, 408000),
	CORE_DVFS("tsec",    -1, -1, 1, KHZ,      136000, 136000, 136000, 145000, 145000, 145000, 408000, 408000, 408000, 408000),
	CORE_DVFS("host1x",  -1, -1, 1, KHZ,       81000,  81000,  81000,  86000,  86000,  86000, 384000, 384000, 384000, 384000),
	CORE_DVFS("vi",      -1, -1, 1, KHZ,      110000, 110000, 110000, 118000, 118000, 118000, 408000, 408000, 408000, 408000),
	CORE_DVFS("isp",      -1, -1, 1, KHZ,     110000, 110000, 110000, 118000, 118000, 118000, 408000, 408000, 408000, 408000),
	CORE_DVFS("sbus",    -1, -1, 1, KHZ,      102000, 102000, 102000, 109000, 109000, 109000, 336000, 336000, 336000, 336000),

	CORE_DVFS("emc",     -1, -1, 1, KHZ,           1,      1,      1,      1,      1,      1,1066000,1066000,1066000,1066000),

#ifdef CONFIG_TEGRA_DUAL_CBUS
	CORE_DVFS("c2bus",      -1, -1, 1, KHZ,    96000,  96000,  96000, 105600, 105600, 105600, 576000, 576000, 633600, 710400),
	CORE_DVFS("c3bus",      -1, -1, 1, KHZ,    96000,  96000,  96000, 105600, 105600, 105600, 441600, 441600, 518400, 537600),
#else
	CORE_DVFS("cbus",      -1, -1, 1, KHZ,     96000,  96000,  96000, 105600, 105600, 105600, 441600, 441600, 518400, 537600),
#endif
	/* Core voltages (mV):		             800,    850,    900,    950,   1000,   1050,   1100,   1150,   1200,   1250 */
	/* Clock limits for I/O peripherals */
	CORE_DVFS("csi",    -1, -1, 1, KHZ,        75000,  75000,  75000,  75000,  75000,  84400, 102000, 102000, 102000, 102000),
	CORE_DVFS("cilab",  -1, -1, 1, KHZ,       102000, 102000, 102000, 102000, 102000, 114700, 136000, 136000, 136000, 136000),
	CORE_DVFS("cilcd",  -1, -1, 1, KHZ,       102000, 102000, 102000, 102000, 102000, 114700, 136000, 136000, 136000, 136000),
	CORE_DVFS("cile",  -1, -1, 1, KHZ,        102000, 102000, 102000, 102000, 102000, 114700, 136000, 136000, 136000, 136000),
	CORE_DVFS("dsia",  -1, -1, 1, KHZ,         75000,  75000,  75000,  75000,  86100, 105500, 125000, 125000, 125000, 125000),
	CORE_DVFS("dsib",  -1, -1, 1, KHZ,         75000,  75000,  75000,  75000,  86100, 105500, 125000, 125000, 125000, 125000),
	CORE_DVFS("dsialp",  -1, -1, 1, KHZ,       72000,  72000,  72000,  72000,  77500,  94900, 112500, 112500, 112500, 112500),
	CORE_DVFS("dsiblp",  -1, -1, 1, KHZ,       72000,  72000,  72000,  72000,  77500,  94900, 112500, 112500, 112500, 112500),
	CORE_DVFS("hdmi",  -1, -1, 1, KHZ,        111300, 111300, 111300, 135600, 173200, 212300, 270000, 270000, 270000, 270000),
	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block. Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1",  -1, -1, 0, KHZ,        74250, 165000, 165000, 165000, 165000, 165000, 165000, 165000, 165000, 165000),
	CORE_DVFS("disp2",  -1, -1, 0, KHZ,        74250, 165000, 165000, 165000, 165000, 165000, 165000, 165000, 165000, 165000),
#endif
};

int tegra_dvfs_disable_core_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra14_dvfs_rail_vdd_core);
	else
		tegra_dvfs_rail_enable(&tegra14_dvfs_rail_vdd_core);

	return 0;
}

int tegra_dvfs_disable_cpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra14_dvfs_rail_vdd_cpu);
	else
		tegra_dvfs_rail_enable(&tegra14_dvfs_rail_vdd_cpu);

	return 0;
}

int tegra_dvfs_disable_get(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static struct kernel_param_ops tegra_dvfs_disable_core_ops = {
	.set = tegra_dvfs_disable_core_set,
	.get = tegra_dvfs_disable_get,
};

static struct kernel_param_ops tegra_dvfs_disable_cpu_ops = {
	.set = tegra_dvfs_disable_cpu_set,
	.get = tegra_dvfs_disable_get,
};

module_param_cb(disable_core, &tegra_dvfs_disable_core_ops,
	&tegra_dvfs_core_disabled, 0644);
module_param_cb(disable_cpu, &tegra_dvfs_disable_cpu_ops,
	&tegra_dvfs_cpu_disabled, 0644);


static bool __init can_update_max_rate(struct clk *c, struct dvfs *d)
{
	/* Don't update manual dvfs clocks */
	if (!d->auto_dvfs)
		return false;

	/*
	 * Don't update EMC shared bus, since EMC dvfs is board dependent: max
	 * rate and EMC scaling frequencies are determined by tegra BCT (flashed
	 * together with the image) and board specific EMC DFS table; we will
	 * check the scaling ladder against nominal core voltage when the table
	 * is loaded (and if on particular board the table is not loaded, EMC
	 * scaling is disabled).
	 */
	if (c->ops->shared_bus_update && (c->flags & PERIPH_EMC_ENB))
		return false;

	/*
	 * Don't update shared cbus, and don't propagate common cbus dvfs
	 * limit down to shared users, but set maximum rate for each user
	 * equal to the respective client limit.
	 */
	if (c->ops->shared_bus_update && (c->flags & PERIPH_ON_CBUS)) {
		struct clk *user;
		unsigned long rate;

		list_for_each_entry(
			user, &c->shared_bus_list, u.shared_bus_user.node) {
			if (user->u.shared_bus_user.client) {
				rate = user->u.shared_bus_user.client->max_rate;
				user->max_rate = rate;
				user->u.shared_bus_user.rate = rate;
			}
		}
		return false;
	}

	/* Other, than EMC and cbus, auto-dvfs clocks can be updated */
	return true;
}

static void __init init_dvfs_one(struct dvfs *d, int max_freq_index)
{
	int ret;
	struct clk *c = tegra_get_clock_by_name(d->clk_name);

	if (!c) {
		pr_debug("tegra14_dvfs: no clock found for %s\n",
			d->clk_name);
		return;
	}

	/* Update max rate for auto-dvfs clocks, with shared bus exceptions */
	if (can_update_max_rate(c, d)) {
		BUG_ON(!d->freqs[max_freq_index]);
		tegra_init_max_rate(
			c, d->freqs[max_freq_index] * d->freqs_mult);
	}
	d->max_millivolts = d->dvfs_rail->nominal_millivolts;

	ret = tegra_enable_dvfs_on_clk(c, d);
	if (ret)
		pr_err("tegra14_dvfs: failed to enable dvfs on %s\n", c->name);
}

static bool __init match_dvfs_one(struct dvfs *d, int speedo_id, int process_id)
{
	if ((d->process_id != -1 && d->process_id != process_id) ||
		(d->speedo_id != -1 && d->speedo_id != speedo_id)) {
		pr_debug("tegra14_dvfs: rejected %s speedo %d,"
			" process %d\n", d->clk_name, d->speedo_id,
			d->process_id);
		return false;
	}
	return true;
}

static bool __init match_cpu_cvb_one(struct cpu_cvb_dvfs *d,
				     int speedo_id, int process_id)
{
	if ((d->process_id != -1 && d->process_id != process_id) ||
	    (d->speedo_id != -1 && d->speedo_id != speedo_id)) {
		pr_debug("tegra14_dvfs: rejected cpu cvb speedo %d,"
			 " process %d\n", d->speedo_id, d->process_id);
		return false;
	}
	return true;
}

/* cvb_mv = ((c2 * speedo / s_scale + c1) * speedo / s_scale + c0) / v_scale */
static inline int get_cvb_voltage(int speedo, int s_scale,
				  struct cpu_cvb_dvfs_parameters *cvb)
{
	/* apply only speedo scale: output mv = cvb_mv * v_scale */
	int mv;
	mv = DIV_ROUND_CLOSEST(cvb->c2 * speedo, s_scale);
	mv = DIV_ROUND_CLOSEST((mv + cvb->c1) * speedo, s_scale) + cvb->c0;
	return mv;
}

static inline int round_cvb_voltage(int mv, int v_scale)
{
	/* combined: apply voltage scale and round to cvb alignment step */
	int cvb_align_step_uv = tegra_get_cvb_alignment_uV();

	return DIV_ROUND_UP(mv * 1000, v_scale * cvb_align_step_uv) *
		cvb_align_step_uv / 1000;
}

static int __init set_cpu_dvfs_data(
	struct cpu_cvb_dvfs *d, struct dvfs *cpu_dvfs, int *max_freq_index)
{
	int i, j, mv, dfll_mv, min_dfll_mv;
	unsigned long fmax_at_vmin = 0;
	unsigned long fmax_pll_mode = 0;
	unsigned long fmin_use_dfll = 0;
	struct cpu_cvb_dvfs_table *table = NULL;
	int speedo = tegra_cpu_speedo_value();

	min_dfll_mv = d->dfll_tune_data.min_millivolts;
	BUG_ON(min_dfll_mv < tegra14_dvfs_rail_vdd_cpu.min_millivolts);

	/*
	 * Use CVB table to fill in CPU dvfs frequencies and voltages. Each
	 * CVB entry specifies CPU frequency and CVB coefficients to calculate
	 * the respective voltage when either DFLL or PLL is used as CPU clock
	 * source.
	 *
	 * Minimum voltage limit is applied only to DFLL source. For PLL source
	 * voltage can go as low as table specifies. Maximum voltage limit is
	 * applied to both sources, but differently: directly clip voltage for
	 * DFLL, and limit maximum frequency for PLL.
	 */
	for (i = 0, j = 0; i < MAX_DVFS_FREQS; i++) {
		table = &d->cvb_table[i];
		if (!table->freq)
			break;

		dfll_mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_dfll_param);
		dfll_mv = round_cvb_voltage(dfll_mv, d->voltage_scale);

		mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_pll_param);
		mv = round_cvb_voltage(mv, d->voltage_scale);

		/*
		 * Check maximum frequency at minimum voltage for dfll source;
		 * round down unless all table entries are above Vmin, then use
		 * the 1st entry as is.
		 */
		dfll_mv = max(dfll_mv, min_dfll_mv);
		if (dfll_mv > min_dfll_mv) {
			if (!j)
				fmax_at_vmin = table->freq;
			if (!fmax_at_vmin)
				fmax_at_vmin = cpu_dvfs->freqs[j - 1];
		}

		/* Clip maximum frequency at maximum voltage for pll source */
		if (mv > d->max_mv) {
			if (!j)
				break;	/* 1st entry already above Vmax */
			if (!fmax_pll_mode)
				fmax_pll_mode = cpu_dvfs->freqs[j - 1];
		}

		/* Minimum rate with pll source voltage above dfll Vmin */
		if ((mv >= min_dfll_mv) && (!fmin_use_dfll))
			fmin_use_dfll = table->freq;

		/* fill in dvfs tables */
		cpu_dvfs->freqs[j] = table->freq;
		cpu_dfll_millivolts[j] = min(dfll_mv, d->max_mv);
		cpu_millivolts[j] = mv;
		j++;

		/*
		 * "Round-up" frequency list cut-off (keep first entry that
		 *  exceeds max voltage - the voltage limit will be enforced
		 *  anyway, so when requested this frequency dfll will settle
		 *  at whatever high frequency it can on the particular chip)
		 */
		if (dfll_mv > d->max_mv)
			break;
	}
	/* Table must not be empty, must have at least one entry above Vmin */
	if (!i || !j || !fmax_at_vmin) {
		pr_err("tegra14_dvfs: invalid cpu dvfs table\n");
		return -ENOENT;
	}

	/* In the dfll operating range dfll voltage at any rate should be
	   better (below) than pll voltage */
	if (!fmin_use_dfll || (fmin_use_dfll > fmax_at_vmin)) {
		WARN(1, "tegra14_dvfs: pll voltage is below dfll in the dfll"
			" operating range\n");
		fmin_use_dfll = fmax_at_vmin;
	}

	/* dvfs tables are successfully populated - fill in the rest */
	cpu_dvfs->speedo_id = d->speedo_id;
	cpu_dvfs->process_id = d->process_id;
	cpu_dvfs->freqs_mult = d->freqs_mult;
	cpu_dvfs->dvfs_rail->nominal_millivolts = min(d->max_mv,
		max(cpu_millivolts[j - 1], cpu_dfll_millivolts[j - 1]));
	*max_freq_index = j - 1;

	cpu_dvfs->dfll_data = d->dfll_tune_data;
	cpu_dvfs->dfll_data.max_rate_boost = fmax_pll_mode ?
		(cpu_dvfs->freqs[j - 1] - fmax_pll_mode) * d->freqs_mult : 0;
	cpu_dvfs->dfll_data.out_rate_min = fmax_at_vmin * d->freqs_mult;
	cpu_dvfs->dfll_data.use_dfll_rate_min = fmin_use_dfll * d->freqs_mult;
	cpu_dvfs->dfll_data.min_millivolts = min_dfll_mv;
	return 0;
}

static int __init get_core_nominal_mv_index(int speedo_id)
{
	int i;
	int mv = tegra_core_speedo_mv();
	int core_edp_limit = get_core_edp();

	/*
	 * Start with nominal level for the chips with this speedo_id. Then,
	 * make sure core nominal voltage is below edp limit for the board
	 * (if edp limit is set).
	 */
	if (core_edp_limit)
		mv = min(mv, core_edp_limit);

	/* Round nominal level down to the nearest core scaling step */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if ((core_millivolts[i] == 0) || (mv < core_millivolts[i]))
			break;
	}

	if (i == 0) {
		pr_err("tegra14_dvfs: unable to adjust core dvfs table to"
		       " nominal voltage %d\n", mv);
		return -ENOSYS;
	}
	return i - 1;
}

int tegra_cpu_dvfs_alter(int edp_thermal_index, const cpumask_t *cpus,
			 bool before_clk_update, int cpu_event)
{
	/* empty definition for tegra14 */
	return 0;
}

void __init tegra14x_init_dvfs(void)
{
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int cpu_process_id = tegra_cpu_process_id();
	int soc_speedo_id = tegra_soc_speedo_id();
	int core_process_id = tegra_core_process_id();

	int i, ret;
	int core_nominal_mv_index;
	int cpu_max_freq_index = 0;

#ifndef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_core_disabled = true;
#endif
#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif
	/* Setup rail bins */
	tegra14_dvfs_rail_vdd_cpu.stats.bin_uV = tegra_get_cvb_alignment_uV();
	tegra14_dvfs_rail_vdd_core.stats.bin_uV = tegra_get_cvb_alignment_uV();

	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in core scaling ladder can also be
	 * used to determine max dvfs frequencies for all core clocks. In
	 * case of error disable core scaling and set index to 0, so that
	 * core clocks would not exceed rates allowed at minimum voltage.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra14_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra14_dvfs_rail_vdd_core.nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	/*
	 * Setup cpu dvfs and dfll tables from cvb data, determine nominal
	 * voltage for cpu rail, and cpu maximum frequency. Note that entire
	 * frequency range is guaranteed only when dfll is used as cpu clock
	 * source. Reaching maximum frequency with pll as cpu clock source
	 * may not be possible within nominal voltage range (dvfs mechanism
	 * would automatically fail frequency request in this case, so that
	 * voltage limit is not violated). Error when cpu dvfs table can not
	 * be constructed must never happen.
	 */
	for (ret = 0, i = 0; i <  ARRAY_SIZE(cpu_cvb_dvfs_table); i++) {
		struct cpu_cvb_dvfs *d = &cpu_cvb_dvfs_table[i];
		if (match_cpu_cvb_one(d, cpu_speedo_id, cpu_process_id)) {
			ret = set_cpu_dvfs_data(
				d, &cpu_dvfs, &cpu_max_freq_index);
			break;
		}
	}
	BUG_ON((i == ARRAY_SIZE(cpu_cvb_dvfs_table)) || ret);

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra14_dvfs_rails,
		ARRAY_SIZE(tegra14_dvfs_rails));

	/* Search core dvfs table for speedo/process matching entries and
	   initialize dvfs-ed clocks */
	for (i = 0; i <  ARRAY_SIZE(core_dvfs_table); i++) {
		struct dvfs *d = &core_dvfs_table[i];
		if (!match_dvfs_one(d, soc_speedo_id, core_process_id))
			continue;
		init_dvfs_one(d, core_nominal_mv_index);
	}

	/* Initialize matching cpu dvfs entry already found when nominal
	   voltage was determined */
	init_dvfs_one(&cpu_dvfs, cpu_max_freq_index);

	/* Finally disable dvfs on rails if necessary */
	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra14_dvfs_rail_vdd_core);
	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra14_dvfs_rail_vdd_cpu);

	pr_info("tegra dvfs: VDD_CPU nominal %dmV, scaling %s\n",
		tegra14_dvfs_rail_vdd_cpu.nominal_millivolts,
		tegra_dvfs_cpu_disabled ? "disabled" : "enabled");
	pr_info("tegra dvfs: VDD_CORE nominal %dmV, scaling %s\n",
		tegra14_dvfs_rail_vdd_core.nominal_millivolts,
		tegra_dvfs_core_disabled ? "disabled" : "enabled");
}

int tegra_dvfs_rail_disable_prepare(struct dvfs_rail *rail)
{
	return 0;
}

int tegra_dvfs_rail_post_enable(struct dvfs_rail *rail)
{
	return 0;
}

/* Core cap object and table */
static struct kobject *cap_kobj;

static struct core_dvfs_cap_table tegra14_core_cap_table[] = {
#ifdef CONFIG_TEGRA_DUAL_CBUS
	{ .cap_name = "cap.c2bus" },
	{ .cap_name = "cap.c3bus" },
#else
	{ .cap_name = "cap.cbus" },
#endif
	{ .cap_name = "cap.sclk" },
	{ .cap_name = "cap.emc" },
};

static int __init tegra14_dvfs_init_core_cap(void)
{
	int ret;

	cap_kobj = kobject_create_and_add("tegra_cap", kernel_kobj);
	if (!cap_kobj) {
		pr_err("tegra14_dvfs: failed to create sysfs cap object\n");
		return 0;
	}

	ret = tegra_init_core_cap(
		tegra14_core_cap_table, ARRAY_SIZE(tegra14_core_cap_table),
		core_millivolts, ARRAY_SIZE(core_millivolts), cap_kobj);

	if (ret) {
		pr_err("tegra14_dvfs: failed to init core cap interface (%d)\n",
		       ret);
		kobject_del(cap_kobj);
		return 0;
	}
	pr_info("tegra dvfs: tegra sysfs cap interface is initialized\n");

	return 0;
}
late_initcall(tegra14_dvfs_init_core_cap);
