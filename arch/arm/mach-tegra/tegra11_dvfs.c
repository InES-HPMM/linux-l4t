/*
 * arch/arm/mach-tegra/tegra11_dvfs.c
 *
 * Copyright (C) 2012 NVIDIA Corporation.
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

/* FIXME: need tegra11 step */
#define VDD_SAFE_STEP			100

static struct dvfs_rail tegra11_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1250,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
};

static struct dvfs_rail tegra11_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1350,
	.min_millivolts = 850,
	.step = VDD_SAFE_STEP,
};

static struct dvfs_rail *tegra11_dvfs_rails[] = {
	&tegra11_dvfs_rail_vdd_cpu,
	&tegra11_dvfs_rail_vdd_core,
};

/* CPU DVFS tables */
/* FIXME: real data */
static struct cpu_cvb_dvfs cpu_cvb_dvfs_table[] = {
	{
		.speedo_id = 0,
		.min_mv = 850,
		.margin = 103,
		.cvb_table = {
			/*f      c0,   c1,   c2 */
			{ 306,  800,    0,    0},
			{ 408,  812,    0,    0},
			{ 510,  825,    0,    0},
			{ 612,  850,    0,    0},
			{ 714,  850,    0,    0},
			{ 816,  858,    0,    0},
			{ 918,  900,    0,    0},
			{1020,  912,    0,    0},
			{1122,  937,    0,    0},
			{1224,  937,    0,    0},
			{1326,  975,    0,    0},
			{1428, 1000,    0,    0},
			{1530, 1000,    0,    0},
			{1632, 1100,    0,    0},
			{1734, 1150,    0,    0},
			{1836, 1200,    0,    0},
			{   0,    0,    0,    0},
		},
	}
};

/* FIXME: remove */
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
#define CPU_AUTO true
#else
#define CPU_AUTO false
#endif

static int cpu_millivolts[MAX_DVFS_FREQS];
static int cpu_dfll_millivolts[MAX_DVFS_FREQS];

static struct dvfs cpu_dvfs = {
	.clk_name	= "cpu_g",
	.process_id	= -1,
	.freqs_mult	= MHZ,
	.millivolts	= cpu_millivolts,
	.dfll_millivolts = cpu_dfll_millivolts,
	.auto_dvfs	= CPU_AUTO,
	.dvfs_rail	= &tegra11_dvfs_rail_vdd_cpu,
};

static struct tegra_cl_dvfs_dfll_data cpu_dfll_data = {
		.dfll_clk_name	= "dfll_cpu",
		.tune0		= 0x030201,
		.tune1		= 0x000BB0AA,
		.droop_rate_min = 640000000,
};

/* Core DVFS tables */
/* FIXME: real data */
static const int core_millivolts[MAX_DVFS_FREQS] = {
	850,  900,  950, 1000, 1050, 1100, 1150, 1200, 1250};

#define CORE_DVFS(_clk_name, _speedo_id, _auto, _mult, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= -1,				\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra11_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
	/* Core voltages (mV):		    850,    900,    950,   1000,   1050,    1100,    1150,    1200,    1250 */
	/* Clock limits for internal blocks, PLLs */
#ifndef CONFIG_TEGRA_SIMULATION_PLATFORM
	CORE_DVFS("emc",   -1, 1, KHZ,   136000, 266500, 333500, 450000, 633000,  633000,  800000,  933000),

	CORE_DVFS("epp",    0, 1, KHZ,    60400, 110500, 148000, 186400, 248500,  248500,  313000,  391800),
	CORE_DVFS("2d",     0, 1, KHZ,    77200, 141200, 189200, 238200, 317500,  317500,  400000,  500700),
	CORE_DVFS("3d",     0, 1, KHZ,    86800, 158900, 212800, 267900, 357200,  357200,  450000,  563300),
	CORE_DVFS("msenc",  0, 1, KHZ,    64200, 117600, 157500, 198300, 264300,  264300,  333000,  416900),
	CORE_DVFS("se",     0, 1, KHZ,    67500, 123600, 165500, 208400, 277800,  277800,  350000,  438100),
	CORE_DVFS("tsec",   0, 1, KHZ,    67500, 123600, 165500, 208400, 277800,  277800,  350000,  438100),
	CORE_DVFS("vde",    0, 1, KHZ,    70600, 129200, 173100, 217900, 290500,  290500,  366000,  458200),

	CORE_DVFS("host1x", 0, 1, KHZ,    57900, 105900, 141900, 178600, 238200,  238200,  300000,  300000),

#ifdef CONFIG_TEGRA_DUAL_CBUS
	CORE_DVFS("c2bus",  0, 1, KHZ,    77200, 141200, 189200, 238200, 317500,  317500,  400000,  500700),
	CORE_DVFS("c3bus",  0, 1, KHZ,    60400, 110500, 148000, 186400, 248500,  248500,  313000,  391800),
#else
	CORE_DVFS("cbus",   0, 1, KHZ,    60400, 110500, 148000, 186400, 248500,  248500,  313000,  391800),
#endif
#endif
};

int tegra_dvfs_disable_core_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra11_dvfs_rail_vdd_core);
	else
		tegra_dvfs_rail_enable(&tegra11_dvfs_rail_vdd_core);

	return 0;
}

int tegra_dvfs_disable_cpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra11_dvfs_rail_vdd_cpu);
	else
		tegra_dvfs_rail_enable(&tegra11_dvfs_rail_vdd_cpu);

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

static void __init init_dvfs_one(struct dvfs *d, int nominal_mv_index)
{
	int ret;
	struct clk *c = tegra_get_clock_by_name(d->clk_name);

	if (!c) {
		pr_debug("tegra11_dvfs: no clock found for %s\n",
			d->clk_name);
		return;
	}

	/* Update max rate for auto-dvfs clocks, with shared bus exceptions */
	if (can_update_max_rate(c, d)) {
		BUG_ON(!d->freqs[nominal_mv_index]);
		tegra_init_max_rate(
			c, d->freqs[nominal_mv_index] * d->freqs_mult);
	}
	d->max_millivolts = d->dvfs_rail->nominal_millivolts;

	ret = tegra_enable_dvfs_on_clk(c, d);
	if (ret)
		pr_err("tegra11_dvfs: failed to enable dvfs on %s\n", c->name);
}

static bool __init match_dvfs_one(struct dvfs *d, int speedo_id, int process_id)
{
	if ((d->process_id != -1 && d->process_id != process_id) ||
		(d->speedo_id != -1 && d->speedo_id != speedo_id)) {
		pr_debug("tegra11_dvfs: rejected %s speedo %d,"
			" process %d\n", d->clk_name, d->speedo_id,
			d->process_id);
		return false;
	}
	return true;
}

static inline int round_cvb_voltage(int mv)
{
	/* round to 12.5mV */
	return DIV_ROUND_UP(mv * 2, 25) * 25 / 2;
}

static inline int get_cvb_voltage(int speedo,
				  struct cpu_cvb_dvfs_parameters *cvb)
{
	/* FIXME: normalize */
	int mv = cvb->c0 + cvb->c1 * speedo + cvb->c2 * speedo * speedo;
	return mv;
}

static int __init get_cpu_nominal_mv_index(int speedo_id,
	struct dvfs *cpu_dvfs, struct tegra_cl_dvfs_dfll_data *dfll_data)
{
	int i, j, mv, dfll_mv;
	unsigned long fmax_at_vmin = 0;
	struct cpu_cvb_dvfs *d = NULL;
	struct cpu_cvb_dvfs_parameters *cvb = NULL;
	int speedo = 0; /* FIXME: tegra_cpu_speedo_val(); */

	/* Find matching cvb dvfs entry */
	for (i = 0; i < ARRAY_SIZE(cpu_cvb_dvfs_table); i++) {
		d = &cpu_cvb_dvfs_table[i];
		if (speedo_id == d->speedo_id)
			break;
	}

	if (!d) {
		pr_err("tegra11_dvfs: no cpu dvfs table for speedo_id %d\n",
		       speedo_id);
		return -ENOENT;
	}
	BUG_ON(d->min_mv < tegra11_dvfs_rail_vdd_cpu.min_millivolts);

	/*
	 * Use CVB table to fill in CPU dvfs frequencies and voltages. Each
	 * CVB entry specifies CPU frequency and CVB coefficients to calculate
	 * the respective voltage when DFLL is used as CPU clock source. Common
	 * margin is applied to determine voltage requirements for PLL source.
	 */
	for (i = 0, j = 0; i < MAX_DVFS_FREQS; i++) {
		cvb = &d->cvb_table[i];
		if (!cvb->freq_mhz)
			break;

		mv = get_cvb_voltage(speedo, cvb);
		dfll_mv = round_cvb_voltage(mv);
		dfll_mv = max(dfll_mv, d->min_mv);

		/* Check maximum frequency at minimum voltage */
		if (dfll_mv > d->min_mv) {
			if (!j)
				break;	/* 1st entry already above Vmin */
			if (!fmax_at_vmin)
				fmax_at_vmin = cpu_dvfs->freqs[j - 1];
		}

		/* dvfs tables with maximum frequency at any distinct voltage */
		if (!j || (dfll_mv > cpu_dfll_millivolts[j - 1])) {
			cpu_dvfs->freqs[j] = cvb->freq_mhz;
			cpu_dfll_millivolts[j] = dfll_mv;
			mv = round_cvb_voltage(mv * d->margin / 100);
			cpu_millivolts[j] = max(mv, d->min_mv);
			j++;
		} else {
			cpu_dvfs->freqs[j - 1] = cvb->freq_mhz;
		}

	}
	/* Table must not be empty and must have and at least one entry below,
	   and one entry above Vmin */
	if (!i || !j || !fmax_at_vmin) {
		pr_err("tegra11_dvfs: invalid cpu dvfs table for speedo_id %d\n",
		       speedo_id);
		return -ENOENT;
	}

	cpu_dvfs->speedo_id = speedo_id;
	dfll_data->out_rate_min = fmax_at_vmin * MHZ;
	dfll_data->millivolts_min = d->min_mv;
	return j - 1;
}

static int __init get_core_nominal_mv_index(int speedo_id)
{
	int i;
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
	int mv = tegra_core_speedo_mv();
#else
	int mv = 1150;
#endif
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
		pr_err("tegra11_dvfs: unable to adjust core dvfs table to"
		       " nominal voltage %d\n", mv);
		return -ENOSYS;
	}
	return i - 1;
}

void __init tegra11x_init_dvfs(void)
{
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int soc_speedo_id = tegra_soc_speedo_id();
	int core_process_id = tegra_core_process_id();

	int i;
	int core_nominal_mv_index;
	int cpu_nominal_mv_index;

#ifndef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_core_disabled = true;
#endif
#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif

	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in the scaling ladder will also be
	 * used to determine max dvfs frequency for the respective domains.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra11_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra11_dvfs_rail_vdd_core.nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	cpu_nominal_mv_index = get_cpu_nominal_mv_index(
		cpu_speedo_id, &cpu_dvfs, &cpu_dfll_data);
	BUG_ON(cpu_nominal_mv_index < 0);
	tegra11_dvfs_rail_vdd_cpu.nominal_millivolts =
		cpu_millivolts[cpu_nominal_mv_index];

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra11_dvfs_rails,
		ARRAY_SIZE(tegra11_dvfs_rails));

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
	init_dvfs_one(&cpu_dvfs, cpu_nominal_mv_index);

	/* CL DVFS characterization data */
	tegra_cl_dvfs_set_dfll_data(&cpu_dfll_data);

	/* Finally disable dvfs on rails if necessary */
	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra11_dvfs_rail_vdd_core);
	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra11_dvfs_rail_vdd_cpu);

	pr_info("tegra dvfs: VDD_CPU nominal %dmV, scaling %s\n",
		tegra11_dvfs_rail_vdd_cpu.nominal_millivolts,
		tegra_dvfs_cpu_disabled ? "disabled" : "enabled");
	pr_info("tegra dvfs: VDD_CORE nominal %dmV, scaling %s\n",
		tegra11_dvfs_rail_vdd_core.nominal_millivolts,
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

