/*
 * arch/arm/mach-tegra/tegra12_dvfs.c
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
static bool tegra_dvfs_gpu_disabled;
static struct dvfs *cpu_dvfs;
static struct dvfs *gpu_dvfs;

static const int cpu_millivolts[MAX_DVFS_FREQS] = {
	800, 825, 850, 900, 912, 975, 1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175, 1200, 1237, 1250};

static const int core_millivolts[MAX_DVFS_FREQS] = {
	850,  900,  950, 1000, 1050, 1100, 1150, 1200, 1250};

/* TBD: fill in actual hw numbers */
static const int gpu_millivolts[MAX_DVFS_FREQS] = {
	850,  900,  950, 1000, 1050, 1100, 1150, 1200, 1250};

#define KHZ 1000
#define MHZ 1000000

/* FIXME: need tegra12 step */
#define VDD_SAFE_STEP			100

static struct dvfs_rail tegra12_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1250,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
};

static struct dvfs_rail tegra12_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1350,
	.min_millivolts = 850,
	.step = VDD_SAFE_STEP,
};

/* TBD: fill in actual hw number */
static struct dvfs_rail tegra12_dvfs_rail_vdd_gpu = {
	.reg_id = "vdd_gpu",
	.max_millivolts = 1350,
	.min_millivolts = 850,
	.step = VDD_SAFE_STEP,
};

static struct dvfs_rail *tegra12_dvfs_rails[] = {
	&tegra12_dvfs_rail_vdd_cpu,
	&tegra12_dvfs_rail_vdd_core,
	&tegra12_dvfs_rail_vdd_gpu,
};

#ifdef CONFIG_TEGRA_SILICON_PLATFORM
#define CPU_AUTO true
#else
#define CPU_AUTO false
#endif

#define CPU_DVFS(_clk_name, _speedo_id, _process_id, _mult, _freqs...)	\
	{								\
		.clk_name	= _clk_name,				\
		.speedo_id	= _speedo_id,				\
		.process_id	= _process_id,				\
		.freqs		= {_freqs},				\
		.freqs_mult	= _mult,				\
		.millivolts	= cpu_millivolts,			\
		.auto_dvfs	= CPU_AUTO,				\
		.dvfs_rail	= &tegra12_dvfs_rail_vdd_cpu,		\
	}

static struct dvfs cpu_dvfs_table[] = {
	/* Cpu voltages (mV):	      800, 825, 850, 900,  912,  975, 1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175, 1200, 1237 */
	CPU_DVFS("cpu_g",  0, 0, MHZ,   1, 460, 550, 680,  680,  820,  970, 1040, 1080, 1150, 1200, 1240, 1280, 1320, 1360, 1500),

	/*
	 * "Safe entry" to be used when no match for chip speedo, process
	 *  corner is found (just to boot at low rate); must be the last one
	 */
	CPU_DVFS("cpu_g", -1, -1, MHZ,  1,   1, 216, 216, 300),
};

#define CORE_DVFS(_clk_name, _speedo_id, _auto, _mult, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= -1,				\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra12_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
	/* Core voltages (mV):		    850,    900,    950,   1000,   1050,    1100,    1150,    1200,    1250 */
	/* Clock limits for internal blocks, PLLs */
#ifndef CONFIG_TEGRA_SIMULATION_PLATFORM
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

#define CL_DVFS(_speedo_id, _tune0, _tune1, _droop_min, _out_min, _mv_min) \
	{							\
		.dfll_clk_name	= "dfll_cpu",			\
		.speedo_id	= _speedo_id,			\
		.tune0		= _tune0,			\
		.tune1		= _tune1,			\
		.dfll_droop_rate_min = _droop_min,		\
		.dfll_out_rate_min = _out_min,			\
		.dfll_millivolts_min = _mv_min,			\
	}

static struct tegra_cl_dvfs_soc_data cl_dvfs_table[] = {
	CL_DVFS(0, 0x030201, 0x000BB0AA, 640000000, 670000000, 750),
};

#define GPU_DVFS(_clk_name, _speedo_id, _auto, _mult, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= -1,				\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= gpu_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra12_dvfs_rail_vdd_gpu,	\
	}

/* TBD: fill in actual hw numbers */
static struct dvfs gpu_dvfs_table[] = {
	/* Gpu voltages (mV):		    850,    900,    950,   1000,   1050,    1100,    1150,    1200,    1250 */
	/* Clock limits for internal blocks, PLLs */
	GPU_DVFS("gpu",     -1, 1, KHZ,    624000, 650000, 676000, 702000, 728000,  754000,  780000,  806000),
};

int tegra_dvfs_disable_core_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra12_dvfs_rail_vdd_core);
	else
		tegra_dvfs_rail_enable(&tegra12_dvfs_rail_vdd_core);

	return 0;
}

int tegra_dvfs_disable_cpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra12_dvfs_rail_vdd_cpu);
	else
		tegra_dvfs_rail_enable(&tegra12_dvfs_rail_vdd_cpu);

	return 0;
}

int tegra_dvfs_disable_gpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_gpu_disabled)
		tegra_dvfs_rail_disable(&tegra12_dvfs_rail_vdd_gpu);
	else
		tegra_dvfs_rail_enable(&tegra12_dvfs_rail_vdd_gpu);

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

static struct kernel_param_ops tegra_dvfs_disable_gpu_ops = {
	.set = tegra_dvfs_disable_gpu_set,
	.get = tegra_dvfs_disable_get,
};

module_param_cb(disable_core, &tegra_dvfs_disable_core_ops,
	&tegra_dvfs_core_disabled, 0644);
module_param_cb(disable_cpu, &tegra_dvfs_disable_cpu_ops,
	&tegra_dvfs_cpu_disabled, 0644);
module_param_cb(disable_gpu, &tegra_dvfs_disable_gpu_ops,
	&tegra_dvfs_gpu_disabled, 0644);

static void __init init_cl_dvfs_soc_data(int speedo_id)
{
	tegra_cl_dvfs_set_soc_data(cl_dvfs_table);
}

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
		pr_debug("tegra12_dvfs: no clock found for %s\n",
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
		pr_err("tegra12_dvfs: failed to enable dvfs on %s\n", c->name);
}

static bool __init match_dvfs_one(struct dvfs *d, int speedo_id, int process_id)
{
	if ((d->process_id != -1 && d->process_id != process_id) ||
		(d->speedo_id != -1 && d->speedo_id != speedo_id)) {
		pr_debug("tegra12_dvfs: rejected %s speedo %d,"
			" process %d\n", d->clk_name, d->speedo_id,
			d->process_id);
		return false;
	}
	return true;
}

static int __init get_cpu_nominal_mv_index(
	int speedo_id, int process_id, struct dvfs **cpu_dvfs)
{
	int i, j, mv;
	struct dvfs *d;
	struct clk *c;

	/*
	 * Find maximum cpu voltage that satisfies cpu_to_core dependency for
	 * nominal core voltage ("solve from cpu to core at nominal"). Clip
	 * result to the nominal cpu level for the chips with this speedo_id.
	 */
	mv = tegra12_dvfs_rail_vdd_core.nominal_millivolts;
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if (cpu_millivolts[i] == 0)
			break;
	}
	BUG_ON(i == 0);
	mv = cpu_millivolts[i - 1];
	BUG_ON(mv < tegra12_dvfs_rail_vdd_cpu.min_millivolts);
	mv = min(mv, tegra_cpu_speedo_mv());

	/*
	 * Find matching cpu dvfs entry, and use it to determine index to the
	 * final nominal voltage, that satisfies the following requirements:
	 * - allows CPU to run at minimum of the maximum rates specified in
	 *   the dvfs entry and clock tree
	 * - does not violate cpu_to_core dependency as determined above
	 */
	for (i = 0, j = 0; j <  ARRAY_SIZE(cpu_dvfs_table); j++) {
		d = &cpu_dvfs_table[j];
		if (match_dvfs_one(d, speedo_id, process_id)) {
			c = tegra_get_clock_by_name(d->clk_name);
			BUG_ON(!c);

			for (; i < MAX_DVFS_FREQS; i++) {
				if ((d->freqs[i] == 0) ||
				    (cpu_millivolts[i] == 0) ||
				    (mv < cpu_millivolts[i]))
					break;

				if (c->max_rate <= d->freqs[i]*d->freqs_mult) {
					i++;
					break;
				}
			}
			break;
		}
	}

	BUG_ON(i == 0);
	if (j == (ARRAY_SIZE(cpu_dvfs_table) - 1))
		pr_err("tegra12_dvfs: WARNING!!!\n"
		       "tegra12_dvfs: no cpu dvfs table found for chip speedo_id"
		       " %d and process_id %d: set CPU rate limit at %lu\n"
		       "tegra12_dvfs: WARNING!!!\n",
		       speedo_id, process_id, d->freqs[i-1] * d->freqs_mult);

	*cpu_dvfs = d;
	return i - 1;
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
		pr_err("tegra12_dvfs: unable to adjust core dvfs table to"
		       " nominal voltage %d\n", mv);
		return -ENOSYS;
	}
	return i - 1;
}

static int __init get_gpu_nominal_mv_index(
	int speedo_id, int process_id, struct dvfs **gpu_dvfs)
{
	int i, j, mv;
	struct dvfs *d;
	struct clk *c;

	/* TBD: do we have dependency between gpu and core ??
	 * Find maximum gpu voltage that satisfies gpu_to_core dependency for
	 * nominal core voltage ("solve from cpu to core at nominal"). Clip
	 * result to the nominal cpu level for the chips with this speedo_id.
	 */
	mv = tegra12_dvfs_rail_vdd_core.nominal_millivolts;
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if (gpu_millivolts[i] == 0)
			break;
	}
	BUG_ON(i == 0);
	mv = gpu_millivolts[i - 1];
	BUG_ON(mv < tegra12_dvfs_rail_vdd_gpu.min_millivolts);
	mv = min(mv, 1250/* TBD: tegra_gpu_speedo_mv() */);

	/*
	 * Find matching gpu dvfs entry, and use it to determine index to the
	 * final nominal voltage, that satisfies the following requirements:
	 * - allows GPU to run at minimum of the maximum rates specified in
	 *   the dvfs entry and clock tree
	 * - does not violate gpu_to_core dependency as determined above
	 */
	for (i = 0, j = 0; j <  ARRAY_SIZE(gpu_dvfs_table); j++) {
		d = &gpu_dvfs_table[j];
		if (match_dvfs_one(d, speedo_id, process_id)) {
			c = tegra_get_clock_by_name(d->clk_name);
			BUG_ON(!c);

			for (; i < MAX_DVFS_FREQS; i++) {
				if ((d->freqs[i] == 0) ||
				    (gpu_millivolts[i] == 0) ||
				    (mv < gpu_millivolts[i]))
					break;

				if (c->max_rate <= d->freqs[i]*d->freqs_mult) {
					i++;
					break;
				}
			}
			break;
		}
	}

	BUG_ON(i == 0);
	if (j == (ARRAY_SIZE(gpu_dvfs_table) - 1))
		pr_err("tegra12_dvfs: WARNING!!!\n"
		       "tegra12_dvfs: no gpu dvfs table found for chip speedo_id"
		       " %d and process_id %d: set GPU rate limit at %lu\n"
		       "tegra12_dvfs: WARNING!!!\n",
		       speedo_id, process_id, d->freqs[i-1] * d->freqs_mult);

	*gpu_dvfs = d;
	return i - 1;
}

void __init tegra12x_init_dvfs(void)
{
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int soc_speedo_id = tegra_soc_speedo_id();
	int gpu_speedo_id = -1; /* TBD: use gpu speedo */
	int cpu_process_id = tegra_cpu_process_id();
	int core_process_id = tegra_core_process_id();
	int gpu_process_id = -1; /* TBD: use gpu process */

	int i;
	int core_nominal_mv_index;
	int cpu_nominal_mv_index;
	int gpu_nominal_mv_index;

#ifndef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_core_disabled = true;
#endif
#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif
#ifndef CONFIG_TEGRA_GPU_DVFS
	tegra_dvfs_gpu_disabled = true;
#endif

	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in the scaling ladder will also be
	 * used to determine max dvfs frequency for the respective domains.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra12_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra12_dvfs_rail_vdd_core.nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	cpu_nominal_mv_index = get_cpu_nominal_mv_index(
		cpu_speedo_id, cpu_process_id, &cpu_dvfs);
	BUG_ON((cpu_nominal_mv_index < 0) || (!cpu_dvfs));
	tegra12_dvfs_rail_vdd_cpu.nominal_millivolts =
		cpu_millivolts[cpu_nominal_mv_index];

	gpu_nominal_mv_index = get_gpu_nominal_mv_index(
		gpu_speedo_id, gpu_process_id, &gpu_dvfs);
	BUG_ON((gpu_nominal_mv_index < 0) || (!gpu_dvfs));
	tegra12_dvfs_rail_vdd_gpu.nominal_millivolts =
		gpu_millivolts[gpu_nominal_mv_index];

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra12_dvfs_rails,
		ARRAY_SIZE(tegra12_dvfs_rails));

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
	init_dvfs_one(cpu_dvfs, cpu_nominal_mv_index);

	/* CL DVFS characterization data */
	init_cl_dvfs_soc_data(soc_speedo_id);

	init_dvfs_one(gpu_dvfs, gpu_nominal_mv_index);

	/* Finally disable dvfs on rails if necessary */
	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra12_dvfs_rail_vdd_core);
	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra12_dvfs_rail_vdd_cpu);
	if (tegra_dvfs_gpu_disabled)
		tegra_dvfs_rail_disable(&tegra12_dvfs_rail_vdd_gpu);

	pr_info("tegra dvfs: VDD_CPU nominal %dmV, scaling %s\n",
		tegra12_dvfs_rail_vdd_cpu.nominal_millivolts,
		tegra_dvfs_cpu_disabled ? "disabled" : "enabled");
	pr_info("tegra dvfs: VDD_CORE nominal %dmV, scaling %s\n",
		tegra12_dvfs_rail_vdd_core.nominal_millivolts,
		tegra_dvfs_core_disabled ? "disabled" : "enabled");
	pr_info("tegra dvfs: VDD_GPU nominal %dmV, scaling %s\n",
		tegra12_dvfs_rail_vdd_gpu.nominal_millivolts,
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

