/*
 * arch/arm/mach-tegra/tegra12_dvfs.c
 *
 * Copyright (c) 2012-2013 NVIDIA CORPORATION. All rights reserved.
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
#include <linux/pm_qos.h>

#include "clock.h"
#include "dvfs.h"
#include "fuse.h"
#include "board.h"
#include "tegra_cl_dvfs.h"
#include "tegra_core_sysfs_limits.h"

static bool tegra_dvfs_cpu_disabled;
static bool tegra_dvfs_core_disabled;
static bool tegra_dvfs_gpu_disabled;

#define KHZ 1000
#define MHZ 1000000

/* FIXME: need tegra12 step */
#define VDD_SAFE_STEP			100

static int vdd_core_therm_trips_table[MAX_THERMAL_LIMITS] = { 20, };
static int vdd_core_therm_floors_table[MAX_THERMAL_LIMITS] = { 950, };

static int vdd_gpu_therm_trips_table[MAX_THERMAL_LIMITS] = { 20, };
static int vdd_gpu_therm_floors_table[MAX_THERMAL_LIMITS] = { 900, };

static struct tegra_cooling_device cpu_cdev = {
	.cdev_type = "cpu_cold",
};

static struct tegra_cooling_device core_cdev = {
	.cdev_type = "core_cold",
};

static struct tegra_cooling_device gpu_cdev = {
	.cdev_type = "gpu_cold",
};

static struct dvfs_rail tegra12_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1400,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
	.vmin_cdev = &cpu_cdev,
	.alignment = {
		.step_uv = 10000, /* 10mV */
	},
	.stats = {
		.bin_uV = 10000, /* 10mV */
	}
};

static struct dvfs_rail tegra12_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1400,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.vmin_cdev = &core_cdev,
};

/* TBD: fill in actual hw number */
static struct dvfs_rail tegra12_dvfs_rail_vdd_gpu = {
	.reg_id = "vdd_gpu",
	.max_millivolts = 1350,
	.min_millivolts = 700,
	.step = VDD_SAFE_STEP,
	.vmin_cdev = &gpu_cdev,
	.alignment = {
		.step_uv = 10000, /* 10mV */
	},
	.stats = {
		.bin_uV = 10000, /* 10mV */
	}
};

static struct dvfs_rail *tegra12_dvfs_rails[] = {
	&tegra12_dvfs_rail_vdd_cpu,
	&tegra12_dvfs_rail_vdd_core,
	&tegra12_dvfs_rail_vdd_gpu,
};

void __init tegra12x_vdd_cpu_align(int step_uv, int offset_uv)
{
	tegra12_dvfs_rail_vdd_cpu.alignment.step_uv = step_uv;
	tegra12_dvfs_rail_vdd_cpu.alignment.offset_uv = offset_uv;
}

/* CPU DVFS tables */
static struct cpu_cvb_dvfs cpu_cvb_dvfs_table[] = {
	{
		.speedo_id = 0,
		.process_id = -1,
		.dfll_tune_data  = {
			.tune0		= 0x00662FFF,
			.tune0_high_mv	= 0x006640FF,
			.tune1		= 0x0000006E,
			.droop_rate_min = 1000000,
			.tune_high_min_millivolts = 900,
			.min_millivolts = 800,
		},
		.max_mv = 1260,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f       dfll: c0,     c1,   c2  pll:  c0,   c1,    c2 */
			{306000  , {1141577,  -30185,  454}, { 777000   , 0, 0}},
			{408000  , {1180454,  -31625,  454}, { 787500   , 0, 0}},
			{510000  , {1221370,  -33065,  454}, { 798000   , 0, 0}},
			{612000  , {1264325,  -34505,  454}, { 819000   , 0, 0}},
			{714000  , {1309319,  -35945,  454}, { 840000   , 0, 0}},
			{816000  , {1356353,  -37385,  454}, { 850500   , 0, 0}},
			{918000  , {1405425,  -38825,  454}, { 882000   , 0, 0}},
			{1020000 , {1456537,  -40265,  454}, { 903000   , 0, 0}},
			{1122000 , {1509687,  -41705,  454}, { 934500   , 0, 0}},
			{1224000 , {1564877,  -43145,  454}, { 955500   , 0, 0}},
			{1326000 , {1622106,  -44585,  454}, { 987000   , 0, 0}},
			{1428000 , {1681374,  -46025,  454}, { 1029000  , 0, 0}},
			{1530000 , {1742681,  -47465,  454}, { 1060500  , 0, 0}},
			{1632000 , {1806027,  -48915,  454}, { 1102500  , 0, 0}},
			{1734000 , {1871412,  -50355,  454}, { 1134000  , 0, 0}},
			{1836000 , {1938836,  -51795,  454}, { 1176000  , 0, 0}},
			{1938000 , {2008300,  -53235,  454}, { 1228500  , 0, 0}},
			{2014500 , {2061382,  -54305,  454}, { 1260000  , 0, 0}},
			{      0 , {      0,       0,    0}, {       0  , 0, 0}},
		},
		.vmin_trips_table = { 20, },
		.therm_floors_table = { 900, },
	},
	{
		.speedo_id = 1,
		.process_id = -1,
		.dfll_tune_data  = {
			.tune0		= 0x00662FFF,
			.tune0_high_mv	= 0x006640FF,
			.tune1		= 0x0000006E,
			.droop_rate_min = 1000000,
			.tune_high_min_millivolts = 900,
			.min_millivolts = 800,
		},
		.max_mv = 1260,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f       dfll: c0,     c1,   c2  pll:  c0,   c1,    c2 */
			{306000  , {1141577,  -30185,  454}, { 735000   , 0, 0}},
			{408000  , {1180454,  -31625,  454}, { 745500   , 0, 0}},
			{510000  , {1221370,  -33065,  454}, { 756000   , 0, 0}},
			{612000  , {1264325,  -34505,  454}, { 766500   , 0, 0}},
			{714000  , {1309319,  -35945,  454}, { 777000   , 0, 0}},
			{816000  , {1356353,  -37385,  454}, { 798000   , 0, 0}},
			{918000  , {1405425,  -38825,  454}, { 819000   , 0, 0}},
			{1020000 , {1456537,  -40265,  454}, { 840000   , 0, 0}},
			{1122000 , {1509687,  -41705,  454}, { 861000   , 0, 0}},
			{1224000 , {1564877,  -43145,  454}, { 882000   , 0, 0}},
			{1326000 , {1622106,  -44585,  454}, { 913500   , 0, 0}},
			{1428000 , {1681374,  -46025,  454}, { 945000   , 0, 0}},
			{1530000 , {1742681,  -47465,  454}, { 976500   , 0, 0}},
			{1632000 , {1806027,  -48915,  454}, { 1008000  , 0, 0}},
			{1734000 , {1871412,  -50355,  454}, { 1039500  , 0, 0}},
			{1836000 , {1938836,  -51795,  454}, { 1081500  , 0, 0}},
			{1938000 , {2008300,  -53235,  454}, { 1123500  , 0, 0}},
			{2014500 , {2061382,  -54305,  454}, { 1144500  , 0, 0}},
			{2116500 , {2134404,  -55745,  454}, { 1197000  , 0, 0}},
			{2218500 , {2209465,  -57185,  454}, { 1239000  , 0, 0}},
			{2320500 , {2286565,  -58625,  454}, { 1291500  , 0, 0}},
			{      0 , {      0,       0,    0}, {       0  , 0, 0}},
		},
		.vmin_trips_table = { 20, },
		.therm_floors_table = { 900, },
	},
};

static int cpu_millivolts[MAX_DVFS_FREQS];
static int cpu_dfll_millivolts[MAX_DVFS_FREQS];

static struct dvfs cpu_dvfs = {
	.clk_name	= "cpu_g",
	.millivolts	= cpu_millivolts,
	.dfll_millivolts = cpu_dfll_millivolts,
	.auto_dvfs	= true,
	.dvfs_rail	= &tegra12_dvfs_rail_vdd_cpu,
};

/* Core DVFS tables */
/* FIXME: real data */
static const int core_millivolts[MAX_DVFS_FREQS] = {
	810, 860, 900, 1000, 1100};

#define CORE_DVFS(_clk_name, _speedo_id, _process_id, _auto, _mult, _freqs...) \
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= _process_id,			\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra12_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
	/* Core voltages (mV):		         810,    860,    900,    1000,    1100*/
	/* Clock limits for internal blocks, PLLs */
	CORE_DVFS("emc",    -1, -1, 1, KHZ,   264000, 348000, 384000, 528000,  924000),

	CORE_DVFS("cpu_lp", -1, -1, 1, KHZ,   144000, 252000, 288000, 444000,  624000),

	CORE_DVFS("sbus",   -1, -1, 1, KHZ,    81600, 102000, 136000, 204000,  204000),

	CORE_DVFS("vic03",  -1, -1, 1, KHZ,   120000, 144000, 168000, 216000,  372000),
	CORE_DVFS("tsec",   -1, -1, 1, KHZ,   120000, 144000, 168000, 216000,  372000),

	CORE_DVFS("msenc",  -1, -1, 1, KHZ,    72000,  84000, 102000, 180000,  252000),
	CORE_DVFS("se",     -1, -1, 1, KHZ,    72000,  84000, 102000, 180000,  252000),
	CORE_DVFS("vde",    -1, -1, 1, KHZ,    72000,  84000, 102000, 180000,  252000),

	CORE_DVFS("host1x", -1, -1, 1, KHZ,    81600, 102000, 136000, 163000,  204000),

	CORE_DVFS("vi",     -1, -1, 1, KHZ,   120000, 156000, 182000, 312000,  444000),
	CORE_DVFS("isp",    -1, -1, 1, KHZ,   120000, 156000, 182000, 312000,  444000),

#ifdef CONFIG_TEGRA_DUAL_CBUS
	CORE_DVFS("c2bus",  -1, -1, 1, KHZ,    72000,  84000, 102000, 180000,  252000),
	CORE_DVFS("c3bus",  -1, -1, 1, KHZ,   120000, 144000, 168000, 216000,  372000),
#else
	CORE_DVFS("cbus",   -1, -1, 1, KHZ,   120000, 144000, 168000, 216000,  372000),
#endif
	CORE_DVFS("c4bus",  -1, -1, 1, KHZ,   120000, 156000, 182000, 312000,  444000),

	CORE_DVFS("pll_m",  -1, -1, 1, KHZ,   800000, 800000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c",  -1, -1, 1, KHZ,   800000, 800000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c2", -1, -1, 1, KHZ,   800000, 800000, 1066000, 1066000, 1066000),
	CORE_DVFS("pll_c3", -1, -1, 1, KHZ,   800000, 800000, 1066000, 1066000, 1066000),

	/* Core voltages (mV):		         810,    860,    900,    990,    1080*/
	/* Clock limits for I/O peripherals */
	CORE_DVFS("sbc1",   -1, -1, 1, KHZ,    24000,  24000,  48000,  48000,   48000),
	CORE_DVFS("sbc2",   -1, -1, 1, KHZ,    24000,  24000,  48000,  48000,   48000),
	CORE_DVFS("sbc3",   -1, -1, 1, KHZ,    24000,  24000,  48000,  48000,   48000),
	CORE_DVFS("sbc4",   -1, -1, 1, KHZ,    24000,  24000,  48000,  48000,   48000),
	CORE_DVFS("sbc5",   -1, -1, 1, KHZ,    24000,  24000,  48000,  48000,   48000),
	CORE_DVFS("sbc6",   -1, -1, 1, KHZ,    24000,  24000,  48000,  48000,   48000),

	CORE_DVFS("sdmmc1", -1, -1, 1, KHZ,   102000, 102000, 163200, 163200,  163200),
	CORE_DVFS("sdmmc3", -1, -1, 1, KHZ,   102000, 102000, 163200, 163200,  163200),
	CORE_DVFS("sdmmc4", -1, -1, 1, KHZ,   102000, 102000, 178200, 178200,  178200),

	CORE_DVFS("hdmi",   -1, -1, 1, KHZ,    99000, 118800, 148500, 198000,  198000),
	/* FIXME: Finalize these values for NOR after qual */
	CORE_DVFS("nor",    -1, -1, 1, KHZ,   102000, 102000, 102000, 102000,  102000),

	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block.  Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1",  -1, -1, 0, KHZ,   108000, 120000, 144000, 297000,  474000),
	CORE_DVFS("disp2",  -1, -1, 0, KHZ,   108000, 120000, 144000, 297000,  474000),

	/* xusb clocks */
	CORE_DVFS("xusb_falcon_src", -1, -1, 1, KHZ,  204000, 204000, 204000, 336000, 336000),
	CORE_DVFS("xusb_host_src",   -1, -1, 1, KHZ,   58300,  58300,  58300, 112000, 112000),
	CORE_DVFS("xusb_dev_src",    -1, -1, 1, KHZ,   58300,  58300,  58300, 112000, 112000),
	CORE_DVFS("xusb_ss_src",     -1, -1, 1, KHZ,   60000,  60000,  60000, 120000, 120000),
	CORE_DVFS("xusb_fs_src",     -1, -1, 1, KHZ,       0,  48000,  48000,  48000,  48000),
	CORE_DVFS("xusb_hs_src",     -1, -1, 1, KHZ,       0,  60000,  60000,  60000,  60000),
};

/* TBD: fill in actual hw numbers */
static struct gpu_cvb_dvfs gpu_cvb_dvfs_table[] = {
	{
		.speedo_id =   0,
		.process_id = -1,
		.max_mv = 1200,
		.min_mv = 800,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f        dfll  pll:   c0,     c1,   c2 */
			{   72000, {  }, {  975248, -10755,  -56}, },
			{  108000, {  }, {  995948, -11645,  -56}, },
			{  180000, {  }, { 1041350, -13415,  -56}, },
			{  252000, {  }, { 1092088, -15195,  -56}, },
			{  324000, {  }, { 1148163, -16975,  -56}, },
			{  396000, {  }, { 1209574, -18745,  -56}, },
			{  468000, {  }, { 1276322, -20525,  -56}, },
			{  540000, {  }, { 1348406, -22295,  -56}, },
			{  612000, {  }, { 1425827, -24075,  -56}, },
			{  648000, {  }, { 1466538, -24965,  -56}, },
			{       0, {  }, {       0,      0,   0}, },
		},
	},
	{
		.speedo_id =   1,
		.process_id = -1,
		.max_mv = 1200,
		.min_mv = 800,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f        dfll  pll:   c0,     c1,   c2 */
			{   72000, {  }, {  975248, -10755,  -56}, },
			{  108000, {  }, {  995948, -11645,  -56}, },
			{  180000, {  }, { 1041350, -13415,  -56}, },
			{  252000, {  }, { 1092088, -15195,  -56}, },
			{  324000, {  }, { 1148163, -16975,  -56}, },
			{  396000, {  }, { 1209574, -18745,  -56}, },
			{  468000, {  }, { 1276322, -20525,  -56}, },
			{  540000, {  }, { 1348406, -22295,  -56}, },
			{  612000, {  }, { 1425827, -24075,  -56}, },
			{  648000, {  }, { 1466538, -24965,  -56}, },
			{  684000, {  }, { 1508583, -25855,  -56}, },
			{  708000, {  }, { 1537355, -26445,  -56}, },
			{  756000, {  }, { 1596677, -27625,  -56}, },
			{  804000, {  }, { 1658370, -28815,  -56}, },
			{       0, {  }, {       0,      0,   0}, },
		},
	}
};

static int gpu_millivolts[MAX_DVFS_FREQS];
static struct dvfs gpu_dvfs = {
	.clk_name	= "gbus",
	.millivolts	= gpu_millivolts,
	.auto_dvfs	= true,
	.dvfs_rail	= &tegra12_dvfs_rail_vdd_gpu,
};

int read_gpu_dvfs_table(int **millivolts, unsigned long **freqs)
{
	*millivolts = gpu_dvfs.millivolts;
	*freqs = gpu_dvfs.freqs;

	return 0;
}
EXPORT_SYMBOL(read_gpu_dvfs_table);

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
		pr_debug("tegra12_dvfs: no clock found for %s\n",
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
		pr_err("tegra12_dvfs: failed to enable dvfs on %s\n", c->name);
}

static bool __init match_dvfs_one(const char *name,
	int dvfs_speedo_id, int dvfs_process_id,
	int speedo_id, int process_id)
{
	if ((dvfs_process_id != -1 && dvfs_process_id != process_id) ||
		(dvfs_speedo_id != -1 && dvfs_speedo_id != speedo_id)) {
		pr_debug("tegra12_dvfs: rejected %s speedo %d, process %d\n",
			 name, dvfs_speedo_id, dvfs_process_id);
		return false;
	}
	return true;
}

/* cvb_mv = ((c2 * speedo / s_scale + c1) * speedo / s_scale + c0) / v_scale */
static inline int get_cvb_voltage(int speedo, int s_scale,
				  struct cvb_dvfs_parameters *cvb)
{
	/* apply only speedo scale: output mv = cvb_mv * v_scale */
	int mv;
	mv = DIV_ROUND_CLOSEST(cvb->c2 * speedo, s_scale);
	mv = DIV_ROUND_CLOSEST((mv + cvb->c1) * speedo, s_scale) + cvb->c0;
	return mv;
}

static int round_cvb_voltage(int mv, int v_scale, struct rail_alignment *align)
{
	/* combined: apply voltage scale and round to cvb alignment step */
	int uv;
	int step = (align->step_uv ? : 1000) * v_scale;
	int offset = align->offset_uv * v_scale;

	uv = max(mv * 1000, offset) - offset;
	uv = DIV_ROUND_UP(uv, step) * align->step_uv + align->offset_uv;
	return uv / 1000;
}

static int __init set_cpu_dvfs_data(
	struct cpu_cvb_dvfs *d, struct dvfs *cpu_dvfs, int *max_freq_index)
{
	int i, j, mv, dfll_mv, min_dfll_mv;
	unsigned long fmax_at_vmin = 0;
	unsigned long fmax_pll_mode = 0;
	unsigned long fmin_use_dfll = 0;
	struct cvb_dvfs_table *table = NULL;
	int speedo = tegra_cpu_speedo_value();
	struct rail_alignment *align = &tegra12_dvfs_rail_vdd_cpu.alignment;

	min_dfll_mv = d->dfll_tune_data.min_millivolts;
	min_dfll_mv =  round_cvb_voltage(min_dfll_mv * 1000, 1000, align);
	d->max_mv = round_cvb_voltage(d->max_mv * 1000, 1000, align);
	BUG_ON(min_dfll_mv < tegra12_dvfs_rail_vdd_cpu.min_millivolts);

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
		dfll_mv = round_cvb_voltage(dfll_mv, d->voltage_scale, align);

		mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_pll_param);
		mv = round_cvb_voltage(mv, d->voltage_scale, align);

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
		pr_err("tegra12_dvfs: invalid cpu dvfs table\n");
		return -ENOENT;
	}

	/* In the dfll operating range dfll voltage at any rate should be
	   better (below) than pll voltage */
	if (!fmin_use_dfll || (fmin_use_dfll > fmax_at_vmin)) {
		WARN(1, "tegra12_dvfs: pll voltage is below dfll in the dfll"
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

static int __init set_gpu_dvfs_data(
	struct gpu_cvb_dvfs *d, struct dvfs *gpu_dvfs, int *max_freq_index)
{
	int i, mv;
	struct cvb_dvfs_table *table = NULL;
	int speedo = tegra_gpu_speedo_value();
	struct rail_alignment *align = &tegra12_dvfs_rail_vdd_gpu.alignment;

	d->max_mv = round_cvb_voltage(d->max_mv * 1000, 1000, align);
	d->min_mv = round_cvb_voltage(d->min_mv * 1000, 1000, align);
	BUG_ON(d->min_mv < tegra12_dvfs_rail_vdd_gpu.min_millivolts);

	/*
	 * Use CVB table to fill in gpu dvfs frequencies and voltages. Each
	 * CVB entry specifies gpu frequency and CVB coefficients to calculate
	 * the respective voltage.
	 */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		table = &d->cvb_table[i];
		if (!table->freq)
			break;

		mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_pll_param);
		mv = round_cvb_voltage(mv, d->voltage_scale, align);

		if (mv > d->max_mv)
			break;

		/* fill in gpu dvfs tables */
		mv = max(mv, d->min_mv);
		gpu_millivolts[i] = mv;
		gpu_dvfs->freqs[i] = table->freq;
	}
	/* Table must not be empty, must have at least one entry in range */
	if (!i || (gpu_millivolts[i - 1] <
		   tegra12_dvfs_rail_vdd_gpu.min_millivolts)) {
		pr_err("tegra14_dvfs: invalid gpu dvfs table\n");
		return -ENOENT;
	}

	/* dvfs tables are successfully populated - fill in the gpu dvfs */
	gpu_dvfs->speedo_id = d->speedo_id;
	gpu_dvfs->process_id = d->process_id;
	gpu_dvfs->freqs_mult = d->freqs_mult;
	gpu_dvfs->dvfs_rail->nominal_millivolts =
		min(d->max_mv, gpu_millivolts[i - 1]);

	*max_freq_index = i - 1;
	return 0;
}

static int __init get_core_nominal_mv_index(int speedo_id)
{
	int i;
	int mv = tegra_core_speedo_mv();
	int core_edp_voltage = get_core_edp();

	/*
	 * Start with nominal level for the chips with this speedo_id. Then,
	 * make sure core nominal voltage is below edp limit for the board
	 * (if edp limit is set).
	 */
	if (!core_edp_voltage)
		core_edp_voltage = 1100;	/* default 1.1V EDP limit */

	mv = min(mv, core_edp_voltage);

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

int tegra_cpu_dvfs_alter(int edp_thermal_index, const cpumask_t *cpus,
			 bool before_clk_update, int cpu_event)
{
	/* empty definition for tegra12 */
	return 0;
}

void __init tegra12x_init_dvfs(void)
{
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int cpu_process_id = tegra_cpu_process_id();
	int soc_speedo_id = tegra_soc_speedo_id();
	int core_process_id = tegra_core_process_id();
	int gpu_speedo_id = tegra_gpu_speedo_id();
	int gpu_process_id = tegra_gpu_process_id();

	int i, ret;
	int core_nominal_mv_index;
	int gpu_max_freq_index = 0;
	int cpu_max_freq_index = 0;

#ifndef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_core_disabled = true;
#endif
#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif
#ifndef CONFIG_TEGRA_GPU_DVFS
	tegra_dvfs_gpu_disabled = true;
#endif
#ifdef CONFIG_TEGRA_PRE_SILICON_SUPPORT
	if (!tegra_platform_is_silicon()) {
		tegra_dvfs_core_disabled = true;
		tegra_dvfs_cpu_disabled = true;
	}
#endif

	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in core scaling ladder can also be
	 * used to determine max dvfs frequencies for all core clocks. In
	 * case of error disable core scaling and set index to 0, so that
	 * core clocks would not exceed rates allowed at minimum voltage.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra12_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra12_dvfs_rail_vdd_core.nominal_millivolts =
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
		if (match_dvfs_one("cpu cvb", d->speedo_id, d->process_id,
				   cpu_speedo_id, cpu_process_id)) {
			ret = set_cpu_dvfs_data(
				d, &cpu_dvfs, &cpu_max_freq_index);
			break;
		}
	}
	BUG_ON((i == ARRAY_SIZE(cpu_cvb_dvfs_table)) || ret);

	/*
	 * Setup gpu dvfs tables from cvb data, determine nominal voltage for
	 * gpu rail, and gpu maximum frequency. Error when gpu dvfs table can
	 * not be constructed must never happen.
	 */
	for (ret = 0, i = 0; i < ARRAY_SIZE(gpu_cvb_dvfs_table); i++) {
		struct gpu_cvb_dvfs *d = &gpu_cvb_dvfs_table[i];
		if (match_dvfs_one("gpu cvb", d->speedo_id, d->process_id,
				   gpu_speedo_id, gpu_process_id)) {
			ret = set_gpu_dvfs_data(
				d, &gpu_dvfs, &gpu_max_freq_index);
			break;
		}
	}
	BUG_ON((i == ARRAY_SIZE(gpu_cvb_dvfs_table)) || ret);

	/* Init thermal floors */
	/* FIXME: Uncomment when proper values are available later */
	/* init_rail_thermal_profile(cpu_cvb_dvfs_table[i].therm_trips_table,
		cpu_cvb_dvfs_table[i].therm_floors_table,
		&tegra12_dvfs_rail_vdd_cpu, &cpu_dvfs.dfll_data);
	init_rail_thermal_profile(vdd_core_therm_trips_table,
		vdd_core_therm_floors_table, &tegra12_dvfs_rail_vdd_core, NULL);*/
	tegra_dvfs_rail_init_vmin_thermal_profile(vdd_gpu_therm_trips_table,
		vdd_gpu_therm_floors_table, &tegra12_dvfs_rail_vdd_gpu, NULL);

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra12_dvfs_rails,
		ARRAY_SIZE(tegra12_dvfs_rails));

	/* Search core dvfs table for speedo/process matching entries and
	   initialize dvfs-ed clocks */
	if (!tegra_platform_is_linsim()) {
		for (i = 0; i <  ARRAY_SIZE(core_dvfs_table); i++) {
			struct dvfs *d = &core_dvfs_table[i];
			if (!match_dvfs_one(d->clk_name, d->speedo_id,
				d->process_id, soc_speedo_id, core_process_id))
				continue;
			init_dvfs_one(d, core_nominal_mv_index);
		}
	}

	/* Initialize matching gpu dvfs entry already found when nominal
	   voltage was determined */
	init_dvfs_one(&gpu_dvfs, gpu_max_freq_index);

	/* Initialize matching cpu dvfs entry already found when nominal
	   voltage was determined */
	init_dvfs_one(&cpu_dvfs, cpu_max_freq_index);

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
		tegra_dvfs_gpu_disabled ? "disabled" : "enabled");
}

int tegra_dvfs_rail_disable_prepare(struct dvfs_rail *rail)
{
	return 0;
}

int tegra_dvfs_rail_post_enable(struct dvfs_rail *rail)
{
	return 0;
}

/* Core voltage and bus cap object and tables */
static struct kobject *cap_kobj;
static struct kobject *gpu_kobj;

static struct core_dvfs_cap_table tegra12_core_cap_table[] = {
#ifdef CONFIG_TEGRA_DUAL_CBUS
	{ .cap_name = "cap.c2bus" },
	{ .cap_name = "cap.c3bus" },
#else
	{ .cap_name = "cap.cbus" },
#endif
	{ .cap_name = "cap.sclk" },
	{ .cap_name = "cap.emc" },
	{ .cap_name = "cap.host1x" },
};

static struct core_bus_limit_table tegra12_gpu_cap_syfs = {
	.limit_clk_name = "cap.profile.gbus",
	.refcnt_attr = {.attr = {.name = "gpu_cap_state", .mode = 0644} },
	.level_attr  = {.attr = {.name = "gpu_cap_level", .mode = 0644} },
	.pm_qos_class = PM_QOS_GPU_FREQ_MAX,
};

static struct core_bus_limit_table tegra12_gpu_floor_sysfs = {
	.limit_clk_name = "floor.profile.gbus",
	.refcnt_attr = {.attr = {.name = "gpu_floor_state", .mode = 0644} },
	.level_attr  = {.attr = {.name = "gpu_floor_level", .mode = 0644} },
	.pm_qos_class = PM_QOS_GPU_FREQ_MIN,
};

static struct core_bus_rates_table tegra12_gpu_rates_sysfs = {
	.bus_clk_name = "gbus",
	.rate_attr = {.attr = {.name = "gpu_rate", .mode = 0444} },
	.available_rates_attr = {
		.attr = {.name = "gpu_available_rates", .mode = 0444} },
};

static int __init tegra12_dvfs_init_core_cap(void)
{
	int ret;

	cap_kobj = kobject_create_and_add("tegra_cap", kernel_kobj);
	if (!cap_kobj) {
		pr_err("tegra12_dvfs: failed to create sysfs cap object\n");
		return 0;
	}

	ret = tegra_init_core_cap(
		tegra12_core_cap_table, ARRAY_SIZE(tegra12_core_cap_table),
		core_millivolts, ARRAY_SIZE(core_millivolts), cap_kobj);

	if (ret) {
		pr_err("tegra12_dvfs: failed to init core cap interface (%d)\n",
		       ret);
		kobject_del(cap_kobj);
		return 0;
	}
	pr_info("tegra dvfs: tegra sysfs cap interface is initialized\n");

	gpu_kobj = kobject_create_and_add("tegra_gpu", kernel_kobj);
	if (!gpu_kobj) {
		pr_err("tegra12_dvfs: failed to create sysfs gpu object\n");
		return 0;
	}

	ret = tegra_init_shared_bus_cap(&tegra12_gpu_cap_syfs,
					1, gpu_kobj);
	if (ret) {
		pr_err("tegra12_dvfs: failed to init gpu cap interface (%d)\n",
		       ret);
		kobject_del(gpu_kobj);
		return 0;
	}

	ret = tegra_init_shared_bus_floor(&tegra12_gpu_floor_sysfs,
					  1, gpu_kobj);
	if (ret) {
		pr_err("tegra12_dvfs: failed to init gpu floor interface (%d)\n",
		       ret);
		kobject_del(gpu_kobj);
		return 0;
	}

	ret = tegra_init_sysfs_shared_bus_rate(&tegra12_gpu_rates_sysfs,
					       1, gpu_kobj);
	if (ret) {
		pr_err("tegra12_dvfs: failed to init gpu rates interface (%d)\n",
		       ret);
		kobject_del(gpu_kobj);
		return 0;
	}
	pr_info("tegra dvfs: tegra sysfs gpu interface is initialized\n");

	return 0;
}
late_initcall(tegra12_dvfs_init_core_cap);
