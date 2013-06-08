/*
 * arch/arm/mach-tegra/tegra11_edp.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/err.h>

#include <mach/edp.h>

#include "clock.h"
#include "fuse.h"

#define CORE_MODULES_STATES 1
#define TEMPERATURE_RANGES 4
#define CAP_CLKS_NUM 2
#define	TOTAL_CAPS (CORE_EDP_PROFILES_NUM * CORE_MODULES_STATES *\
			TEMPERATURE_RANGES * CAP_CLKS_NUM)

struct core_edp_entry {
	int sku;
	unsigned int cap_mA;
	int mult;
	unsigned long cap_scpu_on[CORE_EDP_PROFILES_NUM][
		CORE_MODULES_STATES][TEMPERATURE_RANGES][CAP_CLKS_NUM];
	unsigned long cap_scpu_off[CORE_EDP_PROFILES_NUM][
		CORE_MODULES_STATES][TEMPERATURE_RANGES][CAP_CLKS_NUM];
};

static int temperatures[] = { 50, 70, 90, 105 };

#ifdef CONFIG_TEGRA_DUAL_CBUS
static char *cap_clks_names[] = { "edp.emc", "edp.c2bus" };
#else
static char *cap_clks_names[] = { "edp.emc", "edp.cbus" };
#endif
static struct clk *cap_clks[CAP_CLKS_NUM];

static struct core_edp_entry core_edp_table[] = {
	{
		.sku		= 0x3,		/* SKU = 4 - T40X */
		.cap_mA		= 6000,		/* 6A cap */
		.mult		= 1000000,	/* MHZ */
		.cap_scpu_on	= {
			/* favor emc */
			{	/* core modules power state 0 (all ON) */
				{{ 924, 636 },
				 { 924, 612 },
				 { 924, 564 },
				 { 924, 480 },
				},
			},
			/* balanced profile */
			{	/* core modules power state 0 (all ON) */
				{{ 792, 636 },
				 { 792, 636 },
				 { 792, 636 },
				 { 792, 552 },
				},
			},
			/* favor gpu */
			{	/* core modules power state 0 (all ON) */
				{{ 624, 672 },
				 { 624, 672 },
				 { 528, 672 },
				 { 408, 672 },
				}
			},
		},
		.cap_scpu_off	= {
			/* favor emc */
			{	/* core modules power state 0 (all ON) */
				{{1066, 700 },
				 { 924, 648 },
				 { 924, 636 },
				 { 924, 588 },
				},
			},
			/* balanced profile */
			{	/* core modules power state 0 (all ON) */
				{{1066, 700 },
				 { 792, 672 },
				 { 792, 672 },
				 { 792, 624 },
				},
			},
			/* favor gpu */
			{	/* core modules power state 0 (all ON) */
				{{1066, 700 },
				 { 792, 672 },
				 { 792, 672 },
				 { 624, 672 },
				}
			},
		},
	},
};

#ifdef CONFIG_TEGRA_EDP_LIMITS
#define LEAKAGE_CONSTS_IJK_COMMON					\
	{								\
		/* i = 0 */						\
		{ {  -42746668,   -5458429,   164998,  -1711, },	\
		  {  178262421,   13375684,  -411791,   4590, },	\
		  { -228866784,  -10482993,   331248,  -4062, },	\
		  {   94301550,    2618719,   -85983,   1193, },	\
		},							\
		/* i = 1 */						\
		{ { -256611791,   49677413, -1655785,  14917, },	\
		  {  584675433, -132620939,  4541560, -41812, },	\
		  { -398106336,  115987156, -4102328,  38737, },	\
		  {   68897184,  -33030745,  1217839, -11801, },	\
		},							\
		/* i = 2 */						\
		{ {  186324676,  -36019083,  1177969, -10669, },	\
		  { -439237936,   98429131, -3276444,  30301, },	\
		  {  315060898,  -88635036,  3004777, -28474, },	\
		  {  -60854399,   26267188,  -907121,   8844, },	\
		}, 							\
		/* i = 3 */						\
		{ {  -35432997,    6154621,  -202200,   1830, }, 	\
		  {   87402153,  -16908683,   565152,  -5220, },	\
		  {  -67775314,   15326770,  -521221,   4927, },	\
		  {   15618709,   -4576116,   158401,  -1538, },	\
		}, 							\
	}

#define LEAKAGE_PARAMS_COMMON_PART					\
	.dyn_scaled         = 1000000,					\
	.dyn_consts_n       = { 1091747, 2035205, 2978661, 3922119 },	\
	.consts_scaled      = 1000000,					\
	.leakage_consts_n   = {  538991,  752463,  959441, 1150000 },	\
	.ijk_scaled         = 100000,					\
	.volt_temp_cap      = { 70, 1300 },				\
	.leakage_consts_ijk = LEAKAGE_CONSTS_IJK_COMMON

static struct tegra_edp_cpu_leakage_params t11x_leakage_params[] = {
	{
		.cpu_speedo_id	    = 0, /* A01 CPU */
		LEAKAGE_PARAMS_COMMON_PART,
	},
	{
		.cpu_speedo_id	    = 1, /* A01P+ CPU */
		.safety_cap         = { 1810500, 1810500, 1606500, 1606500 },
		LEAKAGE_PARAMS_COMMON_PART,
	},
	{
		.cpu_speedo_id	    = 2, /* A01P+ fast CPU */
		.safety_cap         = { 1912500, 1912500, 1912500, 1912500 },
		LEAKAGE_PARAMS_COMMON_PART,
	},
};

struct tegra_edp_cpu_leakage_params *tegra11x_get_leakage_params(int index,
							unsigned int *sz)
{
	BUG_ON(index >= ARRAY_SIZE(t11x_leakage_params));
	if (sz)
		*sz = ARRAY_SIZE(t11x_leakage_params);
	return &t11x_leakage_params[index];
}
#endif

static struct core_edp_entry *find_edp_entry(int sku, unsigned int regulator_mA)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(core_edp_table); i++) {
		struct core_edp_entry *entry = &core_edp_table[i];
		if ((entry->sku == sku) && (entry->cap_mA == regulator_mA))
			return entry;
	}
	return NULL;
}

static unsigned long clip_cap_rate(struct clk *cap_clk, unsigned long rate)
{
	unsigned long floor, ceiling;
	struct clk *p = clk_get_parent(cap_clk);

	if (!p || !p->ops || !p->ops->shared_bus_update) {
		WARN(1, "%s: edp cap clk %s is not a shared bus user\n",
			__func__, cap_clk->name);
		return rate;
	}

	/*
	 * Clip cap rate to shared bus possible rates (going up via shared
	 * bus * ladder since bus clocks always rounds up with resolution of
	 * at least 2kHz)
	 */
	ceiling = clk_round_rate(p, clk_get_min_rate(p));
	do {
		floor = ceiling;
		ceiling = clk_round_rate(p, floor + 2000);
		if (IS_ERR_VALUE(ceiling)) {
			pr_err("%s: failed to clip %lu to %s possible rates\n",
			       __func__, rate, p->name);
			return rate;
		}
	} while ((floor < ceiling) && (ceiling <= rate));

	if (floor > rate)
		WARN(1, "%s: %s cap rate %lu is below %s floor %lu\n",
			__func__, cap_clk->name, rate, p->name, floor);
	return floor;
}

int __init tegra11x_select_core_edp_table(unsigned int regulator_mA,
					  struct tegra_core_edp_limits *limits)
{
	int i;
	int sku = tegra_sku_id;
	unsigned long *cap_rates;
	struct core_edp_entry *edp_entry;

	BUG_ON(ARRAY_SIZE(temperatures) != TEMPERATURE_RANGES);
	BUG_ON(ARRAY_SIZE(cap_clks_names) != CAP_CLKS_NUM);
	for (i = 0; i < CAP_CLKS_NUM; i++) {
		struct clk *c = tegra_get_clock_by_name(cap_clks_names[i]);
		if (!c) {
			pr_err("%s: failed to find edp cap clock %s\n",
			       __func__, cap_clks_names[i]);
			return -ENODEV;
		}
		cap_clks[i] = c;
	}

	edp_entry = find_edp_entry(sku, regulator_mA);
	if (!edp_entry) {
		pr_info("%s: no core edp table for sku %d, %d mA\n",
		       __func__, sku, regulator_mA);
		return -ENODATA;
	}

	limits->sku = sku;
	limits->cap_clocks = cap_clks;
	limits->cap_clocks_num = CAP_CLKS_NUM;
	limits->temperatures = temperatures;
	limits->temperature_ranges = TEMPERATURE_RANGES;
	limits->core_modules_states = CORE_MODULES_STATES;

	cap_rates = &edp_entry->cap_scpu_on[0][0][0][0];
	limits->cap_rates_scpu_on = cap_rates;
	for (i = 0; i < TOTAL_CAPS; i++, cap_rates++) {
		unsigned long rate = *cap_rates * edp_entry->mult;
		*cap_rates = clip_cap_rate(cap_clks[i % CAP_CLKS_NUM], rate);
	}

	cap_rates = &edp_entry->cap_scpu_off[0][0][0][0];
	limits->cap_rates_scpu_off = cap_rates;
	for (i = 0; i < TOTAL_CAPS; i++, cap_rates++) {
		unsigned long rate = *cap_rates * edp_entry->mult;
		*cap_rates = clip_cap_rate(cap_clks[i % CAP_CLKS_NUM], rate);
	}

	return 0;
}
