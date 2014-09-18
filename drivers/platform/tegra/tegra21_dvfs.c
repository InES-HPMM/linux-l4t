/*
 * drivers/platform/tegra/tegra21_dvfs.c
 *
 * Copyright (c) 2012-2014 NVIDIA CORPORATION. All rights reserved.
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
#include <linux/tegra-fuse.h>
#include <linux/pm_qos.h>

#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/dvfs.h>
#include "board.h"
#include <linux/platform/tegra/tegra_cl_dvfs.h>
#include "tegra_core_sysfs_limits.h"

static bool tegra_dvfs_cpu_disabled;
static bool tegra_dvfs_core_disabled;
static bool tegra_dvfs_gpu_disabled;

#define KHZ 1000
#define MHZ 1000000

/* FIXME: need tegra21 step */
#define VDD_SAFE_STEP			100

/* FIXME: tegra21 data */
static int vdd_core_vmin_trips_table[MAX_THERMAL_LIMITS];
static int vdd_core_therm_floors_table[MAX_THERMAL_LIMITS];

static int vdd_core_vmax_trips_table[MAX_THERMAL_LIMITS];
static int vdd_core_therm_caps_table[MAX_THERMAL_LIMITS];

static struct tegra_cooling_device core_vmax_cdev = {
	.cdev_type = "core_hot",
};

static struct tegra_cooling_device core_vmin_cdev = {
	.cdev_type = "core_cold",
};

static struct tegra_cooling_device gpu_vmin_cdev = {
	.cdev_type = "gpu_cold",
};

static struct tegra_cooling_device gpu_vts_cdev = {
	.cdev_type = "gpu_scaling",
};

/* FIXME: fill in actual hw numbers for all rails */
static struct dvfs_rail tegra21_dvfs_rail_vdd_cpu = {
	.reg_id = "vdd_cpu",
	.max_millivolts = 1300,
	.step = VDD_SAFE_STEP,
	.jmp_to_zero = true,
	.alignment = {
		.step_uv = 6250, /* 6.25mV */
	},
	.stats = {
		.bin_uV = 6250, /* 6.25mV */
	},
	.version = "Safe p4v08",
};

static struct dvfs_rail tegra21_dvfs_rail_vdd_core = {
	.reg_id = "vdd_core",
	.max_millivolts = 1300,
	.min_millivolts = 900,
	.step = VDD_SAFE_STEP,
	.step_up = 1300,
	.vmin_cdev = &core_vmin_cdev,
	.vmax_cdev = &core_vmax_cdev,
	.alignment = {
		.step_uv = 6250, /* 6.25mV */
	},
	.version = "Safe p4v10",
};

static struct dvfs_rail tegra21_dvfs_rail_vdd_gpu = {
	.reg_id = "vdd_gpu",
	.max_millivolts = 1300,
	.min_millivolts = 800,
	.step = VDD_SAFE_STEP,
	.step_up = 1300,
	.in_band_pm = true,
	.vts_cdev = &gpu_vts_cdev,
	.vmin_cdev = &gpu_vmin_cdev,
	.alignment = {
		.step_uv = 6250, /* 6.25mV */
	},
	.stats = {
		.bin_uV = 6250, /* 6.25mV */
	},
	.version = "Safe p4v08",
};

static struct dvfs_rail *tegra21_dvfs_rails[] = {
	&tegra21_dvfs_rail_vdd_cpu,
	&tegra21_dvfs_rail_vdd_core,
	&tegra21_dvfs_rail_vdd_gpu,
};

void __init tegra21x_vdd_cpu_align(int step_uv, int offset_uv)
{
	tegra21_dvfs_rail_vdd_cpu.alignment.step_uv = step_uv;
	tegra21_dvfs_rail_vdd_cpu.alignment.offset_uv = offset_uv;
}

/* FIXME: Remove after bringup */
#define BRINGUP_CVB_V_MARGIN	25
#define BRINGUP_CVB_V_MARGIN_EX	5

/* CPU DVFS tables */
/* FIXME: real data */
static unsigned long cpu_max_freq[] = {
/* speedo_id	0	 */
		2040000,
};

static struct cpu_cvb_dvfs cpu_cvb_dvfs_table[] = {
	{
		.speedo_id = -1,
		.process_id = -1,
		.dfll_tune_data  = {
			.droop_rate_min = 1000000,
			.min_millivolts = 900,
		},
		.pll_tune_data = {
			.min_millivolts = 900,
		},
		.max_mv = 1225,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f       dfll:    c0,       c1,      c2  pll:  c0,   c1,    c2 */
			{408000 , {  0       ,  0      ,  0     }, { 0,  0,  0} },
			{510000 , {  0       ,  0      ,  0     }, { 0,  0,  0} },
			{612000 , {  0       ,  0      ,  0     }, { 0,  0,  0} },
			{714000 , {  0       ,  0      ,  0     }, { 0,  0,  0} },
			{816000 , {  0       ,  0      ,  0     }, { 0,  0,  0} },
			{918000 , {  0       ,  0      ,  0     }, { 0,  0,  0} },
			{1020000, { -2875621 ,  358099 , -8585  }, { 0,  0,  0} },
			{1122000, { -52225   ,  104159 , -2816  }, { 0,  0,  0} },
			{1224000, {  1076868 ,  8356   , -727   }, { 0,  0,  0} },
			{1326000, {  2208191 , -84659  ,  1240  }, { 0,  0,  0} },
			{1428000, {  2519460 , -105063 ,  1611  }, { 0,  0,  0} },
			{1530000, {  2639809 , -108729 ,  1626  }, { 0,  0,  0} },
			{1632000, {  2889664 , -122173 ,  1834  }, { 0,  0,  0} },
			{1734000, {  3386160 , -154021 ,  2393  }, { 0,  0,  0} },
			{1836000, {  5100873 , -279186 ,  4747  }, { 0,  0,  0} },
			{1938000, {  5731085 , -314142 ,  5263  }, { 0,  0,  0} },
			{2040000, {  15984463, -1051613,  18628 }, { 0,  0,  0} },
			{      0, {        0,         0,     0  }, { 0,  0,  0} },
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
	.dvfs_rail	= &tegra21_dvfs_rail_vdd_cpu,
};

/* CPU LP DVFS tables */
/* FIXME: real data */
static unsigned long cpu_lp_max_freq[] = {
/* speedo_id	0	 */
		1203200,
};

static struct cpu_cvb_dvfs cpu_lp_cvb_dvfs_table[] = {
	{
		.speedo_id = -1,
		.process_id = -1,
		.pll_tune_data = {
			.min_millivolts = 900,
		},
		.max_mv = 1225,
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f       dfll  pll:  c0,       c1,    c2 */
			{ 384000 , {  }, {  0      ,  0     ,  0    } },
			{ 486400 , {  }, {  0      ,  0     ,  0    } },
			{ 588800 , {  }, { -2001927,  282071, -6899 } },
			{ 691200 , {  }, {  326281 ,  79964 , -2413 } },
			{ 793600 , {  }, {  2711280, -120592,  1899 } },
			{ 896000 , {  }, {  2808375, -120304,  1831 } },
			{ 998400 , {  }, {  3056813, -132036,  2017 } },
			{ 1100800, {  }, {  4415397, -229955,  3887 } },
			{ 1203200, {  }, {  4151796, -197768,  3186 } },
			{       0, {  }, { } },
		},
	},
};

static int cpu_lp_millivolts[MAX_DVFS_FREQS];

static struct dvfs cpu_lp_dvfs = {
	.clk_name	= "cpu_lp",
	.millivolts	= cpu_lp_millivolts,
	.auto_dvfs	= true,
	.dvfs_rail	= &tegra21_dvfs_rail_vdd_cpu,
};

/* GPU DVFS tables */
/* FIXME: fill in actual hw numbers */
static unsigned long gpu_max_freq[] = {
/* speedo_id	0	*/
		1075200,
};
static struct gpu_cvb_dvfs gpu_cvb_dvfs_table[] = {
	{
		.speedo_id =  -1,
		.process_id = -1,
#ifdef CONFIG_TEGRA_GPU_DVFS
		.max_mv = 1225,
#else
		.max_mv = 1000,
#endif
		.freqs_mult = KHZ,
		.speedo_scale = 100,
		.thermal_scale = 10,
		.voltage_scale = 1000,
		.cvb_table = {
			/*f        dfll  pll:   c0,     c1,   c2 */
			{  76800  , {  }, {  0       ,  0      ,  0     }, },
			{  153600 , {  }, {  0       ,  0      ,  0     }, },
			{  230400 , {  }, {  0       ,  0      ,  0     }, },
			{  307200 , {  }, {  0       ,  0      ,  0     }, },
			{  384000 , {  }, {  0       ,  0      ,  0     }, },
			{  460800 , {  }, {  0       ,  0      ,  0     }, },
			{  537600 , {  }, {  0       ,  0      ,  0     }, },
			{  614400 , {  }, { -4854798 ,  547369 , -13088 }, },
			{  691200 , {  }, { -1361387 ,  232613 , -5916  }, },
			{  768000 , {  }, {  220791  ,  99521  , -3041  }, },
			{  844800 , {  }, {  3566508 , -182722 ,  2991  }, },
			{  921600 , {  }, {  5028344 , -290112 ,  5021  }, },
			{  998400 , {  }, {  9165009 , -599835 ,  10880 }, },
			{  1075200, {  }, {  16040730, -1105038,  20239 }, },
			{       0, {  }, {       0,      0,   0}, },
		},
		.cvb_vmin =  {  0, {  }, { 900000, }, },
		.vts_trips_table = { 0, 70, },
	},
};

static int gpu_vmin[MAX_THERMAL_RANGES];
static int gpu_peak_millivolts[MAX_DVFS_FREQS];
static int gpu_millivolts[MAX_THERMAL_RANGES][MAX_DVFS_FREQS];
static struct dvfs gpu_dvfs = {
	.clk_name	= "gbus",
	.auto_dvfs	= true,
	.dvfs_rail	= &tegra21_dvfs_rail_vdd_gpu,
};

/* Core DVFS tables */
/* FIXME: real data */
static const int core_millivolts[MAX_DVFS_FREQS] = {
	830, 850, 875, 900, 925, 950, 975, 1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175, 1200, 1225 };

#define CORE_DVFS(_clk_name, _speedo_id, _process_id, _auto, _mult, _freqs...) \
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= _process_id,			\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.dvfs_rail	= &tegra21_dvfs_rail_vdd_core,	\
	}

#define OVRRD_DVFS(_clk_name, _speedo_id, _process_id, _auto, _mult, _freqs...) \
	{							\
		.clk_name	= _clk_name,			\
		.speedo_id	= _speedo_id,			\
		.process_id	= _process_id,			\
		.freqs		= {_freqs},			\
		.freqs_mult	= _mult,			\
		.millivolts	= core_millivolts,		\
		.auto_dvfs	= _auto,			\
		.can_override	= true,				\
		.dvfs_rail	= &tegra21_dvfs_rail_vdd_core,	\
	}

static struct dvfs core_dvfs_table[] = {
	/* Core voltages (mV):		      830,     850,     875,     900,     925,     950,     975,    1000,    1025,    1050,    1075,    1100,    1125,    1150,    1175,    1200,    1225 */
	/* Clock limits for internal blocks, PLLs */
	CORE_DVFS("emc",    -1, -1, 1, KHZ,        1,       1,       1,       1,       1,       1,       1, 1200000, 1200000, 1200000, 1200000, 1200000, 1200000, 1200000, 1200000, 1200000, 1800000),

	CORE_DVFS("nvjpg",  -1, -1, 1, KHZ,   153600,  166400,  192000,  217600,  243200,  268800,  294400,  320000,  332800,  358400,  384000,  409600,  422400,  448000,  460800,  486400,  499200),
	CORE_DVFS("se",     -1, -1, 1, KHZ,   153600,  166400,  192000,  217600,  243200,  268800,  294400,  320000,  332800,  358400,  384000,  409600,  422400,  448000,  460800,  486400,  499200),
	CORE_DVFS("c2bus",  -1, -1, 1, KHZ,   153600,  166400,  192000,  217600,  243200,  268800,  294400,  320000,  332800,  358400,  384000,  409600,  422400,  448000,  460800,  486400,  499200),

	CORE_DVFS("vic03",  -1, -1, 1, KHZ,   217600,  243200,  281600,  307200,  345600,  384000,  422400,  435200,  448000,  473600,  473600,  486400,  499200,  512000,  524800,  537600,  563200),
	CORE_DVFS("msenc",  -1, -1, 1, KHZ,   217600,  243200,  281600,  307200,  345600,  384000,  422400,  435200,  448000,  473600,  473600,  486400,  499200,  512000,  524800,  537600,  563200),
	CORE_DVFS("nvdec",  -1, -1, 1, KHZ,   217600,  243200,  281600,  307200,  345600,  384000,  422400,  435200,  448000,  473600,  473600,  486400,  499200,  512000,  524800,  537600,  563200),
	CORE_DVFS("tsecb",  -1, -1, 1, KHZ,   217600,  243200,  281600,  307200,  345600,  384000,  422400,  435200,  448000,  473600,  473600,  486400,  499200,  512000,  524800,  537600,  563200),
	CORE_DVFS("c3bus",  -1, -1, 1, KHZ,   217600,  243200,  281600,  307200,  345600,  384000,  422400,  435200,  448000,  473600,  473600,  486400,  499200,  512000,  524800,  537600,  563200),

	CORE_DVFS("vi",     -1, -1, 1, KHZ,   243200,  256000,  294400,  320000,  345600,  371200,  396800,  422400,  448000,  473600,  486400,  499200,  524800,  537600,  563200,  576000,  601600),
	CORE_DVFS("isp",    -1, -1, 1, KHZ,   243200,  256000,  294400,  320000,  345600,  371200,  396800,  422400,  448000,  473600,  486400,  499200,  524800,  537600,  563200,  576000,  601600),
	CORE_DVFS("cbus",   -1, -1, 1, KHZ,   243200,  256000,  294400,  320000,  345600,  371200,  396800,  422400,  448000,  473600,  486400,  499200,  524800,  537600,  563200,  576000,  601600),

	/*
	 * Disable framework auto-dvfs on sbus - let platform code to call
	 * tegra_dvfs_set_rate manually. It is necessary for special handling
	 * of system clock skipper enabled on T210
	 */
	CORE_DVFS("sbus",    -1, -1, 0, KHZ,   107500,  119300,  136100,  153300,  169900,  188000,  204200,  222600,  238300,  255400,  271500,  287000,  299700,  312300,  325100,  336500,  349100),
	CORE_DVFS("mselect", -1, -1, 1, KHZ,   178100,  197600,  225500,  253800,  281500,  311400,  338300,  368700,  394600,  423000,  449600,  475200,  496300,  517200,  538400,  557300,  578200),
	CORE_DVFS("host1x",  -1, -1, 1, KHZ,   89000 ,  98700 ,  112700,  126900,  140700,  155600,  169100,  184300,  197200,  211400,  224700,  237500,  248100,  258500,  269100,  278600,  289000),
	CORE_DVFS("tsec",    -1, -1, 1, KHZ,   178100,  197600,  225500,  253800,  281500,  311400,  338300,  368700,  394600,  423000,  449600,  475200,  496300,  517200,  538400,  557300,  578200),

	CORE_DVFS("pll_c",  -1, -1, 1, KHZ,    611200,	678000,  773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("pll_c2", -1, -1, 1, KHZ,    611200,	678000,  773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("pll_c3", -1, -1, 1, KHZ,    611200,	678000,  773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("pll_c4_out0", -1, -1, 1, KHZ, 611200, 678000, 773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("pll_d_out0", -1, -1, 1, KHZ, 611200,	678000,  773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("pll_d2", -1, -1, 1, KHZ,    611200,	678000,  773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("pll_dp", -1, -1, 1, KHZ,    611200,	678000,  773900, 871100,  965900,  1068500, 1160800, 1265300, 1354100, 1451500,	1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),

	/* Clock limits for DC subsystem */
	/* Core voltages (mV):		       830,     850,     875,     900,     925,     950,     975,    1000,    1025,    1050,    1075,    1100,    1125,    1150,    1175,    1200,    1225 */
	CORE_DVFS("csi",  -1, -1, 0, KHZ,    791000,  877400, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000),
	CORE_DVFS("cilab", -1, -1, 0, KHZ,   102000,  102000, 136000 , 136000 , 136000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000),
	CORE_DVFS("cilcd", -1, -1, 0, KHZ,   102000,  102000, 136000 , 136000 , 136000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000),
	CORE_DVFS("cile",  -1, -1, 0, KHZ,   102000,  102000, 136000 , 136000 , 136000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000),

	CORE_DVFS("dsia", -1, -1, 0, KHZ,    791000,  877400, 1001400, 1127200, 1249900, 1382700, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("dsib", -1, -1, 0, KHZ,    791000,  877400, 1001400, 1127200, 1249900, 1382700, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000, 1500000),
	CORE_DVFS("dsialp", -1, -1, 0, KHZ,  102000,  102000, 136000 , 136000 , 136000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000),
	CORE_DVFS("dsiblp", -1, -1, 0, KHZ,  102000,  102000, 136000 , 136000 , 136000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000 , 204000),
	CORE_DVFS("sor0", -1, -1, 0, KHZ,    1     ,  1     , 270000 , 270000 , 270000 , 270000 , 270000 , 270000 , 270000 , 540000 , 540000 , 540000 , 540000 , 540000 , 540000 , 540000 , 540000),
	CORE_DVFS("sor1", -1, -1, 0, KHZ,    235700,  261500, 298500 , 336000 , 372600 , 412100 , 447700 , 488000 , 522300 , 559900 , 595200 , 600000 , 600000 , 600000 , 600000 , 600000 , 600000),

	/*
	 * The clock rate for the display controllers that determines the
	 * necessary core voltage depends on a divider that is internal
	 * to the display block.  Disable auto-dvfs on the display clocks,
	 * and let the display driver call tegra_dvfs_set_rate manually
	 */
	CORE_DVFS("disp1",  -1, -1, 0, KHZ,    230400,  230400,  256000,  281600,  307200,  320000,  345600,  371200,  384000,  396800,  422400,  435200,  460800,  486400,  499200,  524800,  537600),
	CORE_DVFS("disp2",  -1, -1, 0, KHZ,    230400,  230400,  256000,  281600,  307200,  320000,  345600,  371200,  384000,  396800,  422400,  435200,  460800,  486400,  499200,  524800,  537600),

	/* Clock limits for audio subsystem */
	/* Core voltages (mV):		       830,     850,     875,     900,     925,     950,     975,     1000,    1025,    1050,    1075,    1100,    1125,    1150,    1175,    1200,    1225 */
	CORE_DVFS("i2s0",   -1, -1, 1, KHZ,    1    ,	1    ,   24576 ,  24576	,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,	49152 ,	 49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152),
	CORE_DVFS("i2s1",   -1, -1, 1, KHZ,    1    ,	1    ,   24576 ,  24576	,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,	49152 ,	 49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152),
	CORE_DVFS("i2s2",   -1, -1, 1, KHZ,    1    ,	1    ,   24576 ,  24576	,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,	49152 ,	 49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152),
	CORE_DVFS("i2s3",   -1, -1, 1, KHZ,    1    ,	1    ,   24576 ,  24576	,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,	49152 ,	 49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152),
	CORE_DVFS("i2s4",   -1, -1, 1, KHZ,    1    ,	1    ,   24576 ,  24576	,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,	49152 ,	 49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152),

	CORE_DVFS("d_audio", -1, -1, 1, KHZ,   1    ,   1    ,   49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  98304 ,  98304 ,  98304 ,  98304 ,  98304 ,  98304 ,  98304 ,  98304),
	CORE_DVFS("spdif_out", -1, -1, 1, KHZ, 1    ,   1    ,   24576 ,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,  24576 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152 ,  49152),

	CORE_DVFS("dmic1",  -1, -1, 1, KHZ,    1    ,   1    ,   12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288),
	CORE_DVFS("dmic2",  -1, -1, 1, KHZ,    1    ,   1    ,   12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288),
	CORE_DVFS("dmic3",  -1, -1, 1, KHZ,    1    ,   1    ,   12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288 ,  12288),

	CORE_DVFS("hda",    -1, -1, 1, KHZ,    80600,   89400,   102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000),
	CORE_DVFS("hda2codec_2x", -1, -1, 1, KHZ,  1,   1    ,   1     ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000),

	CORE_DVFS("adsp_cpu", -1, -1, 1, KHZ,  179200,  192000,  230400,  256000,  281600,  320000,  345600,  371200,  396800,  422400,  460800,  486400,  499200,  524800,  550400,  563200,  588800),
	CORE_DVFS("ape",      -1, -1, 1, KHZ,  130900,	145200,	 165700,  186600,  206900,  228900,  248600,  271000,  290000,	300000,	 300000,  300000,  300000,  300000,  300000,  300000,  300000),

	/* Clock limits for I/O peripherals */
	/* Core voltages (mV):		       830,     850,     875,     900,     925,     950,     975,     1000,    1025,    1050,    1075,    1100,    1125,    1150,    1175,    1200,    1225 */
	OVRRD_DVFS("sdmmc1", -1, -1, 1, KHZ,   164500,	182500,	 208000,  208000,  208000,  208000,  208000,  208000,  208000,	208000,  208000,  208000,  208000,  208000,  208000,  208000,  208000),
	OVRRD_DVFS("sdmmc3", -1, -1, 1, KHZ,   164500,	182500,	 208000,  208000,  208000,  208000,  208000,  208000,  208000,	208000,  208000,  208000,  208000,  208000,  208000,  208000,  208000),

	OVRRD_DVFS("sdmmc2", -1, -1, 1, KHZ,   263600,	266000,	 266000,  266000,  266000,  266000,  266000,  266000,  266000,	266000,  266000,  266000,  266000,  266000,  266000,  266000,  266000),
	OVRRD_DVFS("sdmmc4", -1, -1, 1, KHZ,   263600,	266000,	 266000,  266000,  266000,  266000,  266000,  266000,  266000,	266000,  266000,  266000,  266000,  266000,  266000,  266000,  266000),
	OVRRD_DVFS("sdmmc2_ddr", -1, -1, 1, KHZ, 263600, 266000, 266000,  266000,  266000,  266000,  266000,  266000,  266000,	266000,  266000,  266000,  266000,  266000,  266000,  266000,  266000),
	OVRRD_DVFS("sdmmc4_ddr", -1, -1, 1, KHZ, 263600, 266000, 266000,  266000,  266000,  266000,  266000,  266000,  266000,	266000,  266000,  266000,  266000,  266000,  266000,  266000,  266000),

	CORE_DVFS("sbc1",    -1, -1, 1, KHZ,   1     ,	1     ,	 1     ,  33000,   33000,   33000,   33000 ,  33000 ,  33000 ,	33000 ,	 33000 ,  33000	,  33000 ,  33000 ,  33000 ,  33000 ,  33000),
	CORE_DVFS("sbc2",    -1, -1, 1, KHZ,   1     ,	1     ,	 1     ,  33000,   33000,   33000,   33000 ,  33000 ,  33000 ,	33000 ,	 33000 ,  33000	,  33000 ,  33000 ,  33000 ,  33000 ,  33000),
	CORE_DVFS("sbc3",    -1, -1, 1, KHZ,   1     ,	1     ,	 1     ,  33000,   33000,   33000,   33000 ,  33000 ,  33000 ,	33000 ,	 33000 ,  33000	,  33000 ,  33000 ,  33000 ,  33000 ,  33000),
	CORE_DVFS("sbc4",    -1, -1, 1, KHZ,   1     ,	1     ,	 1     ,  33000,   33000,   33000,   33000 ,  33000 ,  33000 ,	33000 ,	 33000 ,  33000	,  33000 ,  33000 ,  33000 ,  33000 ,  33000),
	CORE_DVFS("qspi",    -1, -1, 1, KHZ,   1     ,  1     ,  1     ,  72400,   80300,   91700,   103200,  114400,  126600,	137500,	 149900,  160500,  166000,  166000,  166000,  166000,  166000),

	/* xusb pcie, sata clocks */
	CORE_DVFS("xusb_falcon_src", -1, -1, 1, KHZ,  1, 1    ,  1     ,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000,  312000),
	CORE_DVFS("xusb_host_src",   -1, -1, 1, KHZ,  1, 1    ,  1     ,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000),
	CORE_DVFS("xusb_dev_src",    -1, -1, 1, KHZ,  1, 1    ,  1     ,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000),
	CORE_DVFS("xusb_ss_src",     -1, -1, 1, KHZ,  1, 1    ,  1     ,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000),
	CORE_DVFS("xusb_ssp_src",    -1, -1, 1, KHZ,  1, 1    ,  1     ,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000),
	CORE_DVFS("xusb_fs_src",     -1, -1, 1, KHZ,  1, 1    ,  1     ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000 ,  48000),
	CORE_DVFS("xusb_hs_src",     -1, -1, 1, KHZ,  1, 1    ,  1     ,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000,  120000),

	CORE_DVFS("usbd",  -1, -1, 1, KHZ,            1, 1    ,  1     ,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000),
	CORE_DVFS("usb2",  -1, -1, 1, KHZ,            1, 1    ,  1     ,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000),
	CORE_DVFS("usb3",  -1, -1, 1, KHZ,            1, 1    ,  1     ,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000,  480000),

	CORE_DVFS("sata",   -1, -1, 1, KHZ,           1, 1    ,  1     ,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000,  102000),
	CORE_DVFS("sata_oob", -1, -1, 1, KHZ,         1, 1    ,  1     ,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000,  204000),
	CORE_DVFS("pciex",  -1, -1, 1, KHZ,           1, 1    ,  1     ,  250000,  250000,  500000,  500000,  500000,  500000,  500000,  500000,  500000,  500000,  500000,  500000,  500000,  500000),
};

int tegra_dvfs_disable_core_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra21_dvfs_rail_vdd_core);
	else
		tegra_dvfs_rail_enable(&tegra21_dvfs_rail_vdd_core);

	return 0;
}

int tegra_dvfs_disable_cpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra21_dvfs_rail_vdd_cpu);
	else
		tegra_dvfs_rail_enable(&tegra21_dvfs_rail_vdd_cpu);

	return 0;
}

int tegra_dvfs_disable_gpu_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (tegra_dvfs_gpu_disabled)
		tegra_dvfs_rail_disable(&tegra21_dvfs_rail_vdd_gpu);
	else
		tegra_dvfs_rail_enable(&tegra21_dvfs_rail_vdd_gpu);

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

static bool __init match_dvfs_one(const char *name,
	int dvfs_speedo_id, int dvfs_process_id,
	int speedo_id, int process_id)
{
	if ((dvfs_process_id != -1 && dvfs_process_id != process_id) ||
		(dvfs_speedo_id != -1 && dvfs_speedo_id != speedo_id)) {
		pr_debug("tegra21_dvfs: rejected %s speedo %d, process %d\n",
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

/* cvb_t_mv =
   ((c3 * speedo / s_scale + c4 + c5 * T / t_scale) * T / t_scale) / v_scale */
static inline int get_cvb_t_voltage(int speedo, int s_scale, int t, int t_scale,
				    struct cvb_dvfs_parameters *cvb)
{
	/* apply speedo & temperature scales: output mv = cvb_t_mv * v_scale */
	int mv;
	mv = DIV_ROUND_CLOSEST(cvb->c3 * speedo, s_scale) + cvb->c4 +
		DIV_ROUND_CLOSEST(cvb->c5 * t, t_scale);
	mv = DIV_ROUND_CLOSEST(mv * t, t_scale);
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

static int round_voltage(int mv, struct rail_alignment *align, bool up)
{
	if (align->step_uv) {
		int uv = max(mv * 1000, align->offset_uv) - align->offset_uv;
		uv = (uv + (up ? align->step_uv - 1 : 0)) / align->step_uv;
		return (uv * align->step_uv + align->offset_uv) / 1000;
	}
	return mv;
}

/*
 * Setup fast CPU DVFS tables in PLL and DFLL modes from CVB data, determine
 * nominal voltage for CPU rail, and CPU maximum frequency. Note that entire
 * frequency range is guaranteed only when DFLL is used as CPU clock source.
 * Reaching maximum frequency on PLL may not be possible within nominal voltage
 * range (DVFS core would fail frequency request in this case, so that voltage
 * limit is not violated). Error when CPU DVFS table can not be constructed must
 * never happen.
 */
static int __init set_cpu_dvfs_data(unsigned long max_freq,
	struct cpu_cvb_dvfs *d, struct dvfs *cpu_dvfs, int *max_freq_index)
{
	int j, mv, min_mv, dfll_mv, min_dfll_mv;
	unsigned long fmax_at_vmin = 0;
	unsigned long fmax_pll_mode = 0;
	unsigned long fmin_use_dfll = 0;
	int speedo = tegra_cpu_speedo_value();

	struct cvb_dvfs_table *table = NULL;
	struct dvfs_rail *rail = &tegra21_dvfs_rail_vdd_cpu;
	struct rail_alignment *align = &rail->alignment;

	min_dfll_mv = d->dfll_tune_data.min_millivolts;
	min_dfll_mv =  round_voltage(min_dfll_mv, align, true);

	min_mv = d->pll_tune_data.min_millivolts;
	min_mv =  round_voltage(min_mv, align, true);
	rail->min_millivolts = min(min_mv, min_dfll_mv);

	d->max_mv = round_voltage(d->max_mv, align, false);
	BUG_ON(d->max_mv > rail->max_millivolts);

	/*
	 * Use CVB table to fill in CPU dvfs frequencies and voltages. Each
	 * CVB entry specifies CPU frequency and CVB coefficients to calculate
	 * the respective voltage when either DFLL or PLL is used as CPU clock
	 * source.
	 *
	 * Different minimum voltage limits are applied to DFLL and PLL sources.
	 * Same maximum voltage limit is used for both sources, but differently:
	 * directly limit voltage for DFLL, and limit maximum frequency for PLL.
	 */
	for (j = 0; j < MAX_DVFS_FREQS; j++) {
		table = &d->cvb_table[j];
		if (!table->freq || (table->freq > max_freq))
			break;

		dfll_mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_dfll_param);
#ifdef BRINGUP_CVB_V_MARGIN
		dfll_mv = dfll_mv * (100 + BRINGUP_CVB_V_MARGIN) / 100;
		mv = dfll_mv * (100 + BRINGUP_CVB_V_MARGIN_EX) / 100;
#endif
		dfll_mv = round_cvb_voltage(dfll_mv, d->voltage_scale, align);
		dfll_mv = max(dfll_mv, min_dfll_mv);

#ifndef BRINGUP_CVB_V_MARGIN
		mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_pll_param);
#endif
		mv = round_cvb_voltage(mv, d->voltage_scale, align);
		mv = max(mv, min_mv);

		/*
		 * Check maximum frequency at minimum voltage for dfll source;
		 * round down unless all table entries are above Vmin, then use
		 * the 1st entry as is.
		 */
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
	}

	/* Table must not be empty, must have at least one entry above Vmin */
	if (!j || !fmax_at_vmin) {
		pr_err("tegra21_dvfs: invalid cpu dvfs table\n");
		return -ENOENT;
	}

	/* In the dfll operating range dfll voltage at any rate should be
	   better (below) than pll voltage */
	if (!fmin_use_dfll || (fmin_use_dfll > fmax_at_vmin)) {
		WARN(1, "tegra21_dvfs: pll voltage is below dfll in the dfll"
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

static void __init init_cpu_dvfs_table(int *cpu_max_freq_index)
{
	int i, ret;
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int cpu_process_id = tegra_cpu_process_id();

	BUG_ON(cpu_speedo_id >= ARRAY_SIZE(cpu_max_freq));
	for (ret = 0, i = 0; i <  ARRAY_SIZE(cpu_cvb_dvfs_table); i++) {
		struct cpu_cvb_dvfs *d = &cpu_cvb_dvfs_table[i];
		unsigned long max_freq = cpu_max_freq[cpu_speedo_id];
		if (match_dvfs_one("cpu cvb", d->speedo_id, d->process_id,
				   cpu_speedo_id, cpu_process_id)) {
			ret = set_cpu_dvfs_data(max_freq,
				d, &cpu_dvfs, cpu_max_freq_index);
			break;
		}
	}
	BUG_ON((i == ARRAY_SIZE(cpu_cvb_dvfs_table)) || ret);
}

/*
 * Setup slow CPU (a.k.a LP CPU) DVFS table from CVB data. Only PLL is used as
 * a clock source for slow CPU. Its maximum frequency must be reached within
 * nominal voltage -- CVB frequency list is cut off at rate that exceeds either
 * sku-based maximum limit or requires voltage above nominal. Error when DVFS
 * table can not be constructed must never happen.
 *
 * Final CPU rail nominal voltage is set as maximum of fast and slow CPUs
 * nominal voltages.
 */
static int __init set_cpu_lp_dvfs_data(unsigned long max_freq,
	struct cpu_cvb_dvfs *d, struct dvfs *cpu_lp_dvfs, int *max_freq_index)
{
	int j, mv, min_mv;
	int speedo = tegra_cpu_speedo_value(); /* FIXME cpu_lp_speedo */

	struct cvb_dvfs_table *table = NULL;
	struct dvfs_rail *rail = &tegra21_dvfs_rail_vdd_cpu;
	struct rail_alignment *align = &rail->alignment;

	min_mv = d->pll_tune_data.min_millivolts;
	min_mv =  round_voltage(min_mv, align, true);
	rail->min_millivolts = min(min_mv, rail->min_millivolts);

	d->max_mv = round_voltage(d->max_mv, align, false);
	BUG_ON(d->max_mv > rail->max_millivolts);
	rail->nominal_millivolts = max(rail->nominal_millivolts, d->max_mv);

	/*
	 * Use CVB table to fill in CPU dvfs frequencies and voltages. Each
	 * CVB entry specifies CPU frequency and CVB coefficients to calculate
	 * the respective voltage. Only PLL is used as CPU LP clock source, and
	 * the respective minimum limit is applied to each table entry. Table
	 * construction is aborted if calculated voltage is above maximum limit.
	 */
	for (j = 0; j < MAX_DVFS_FREQS; j++) {
		table = &d->cvb_table[j];
		if (!table->freq || (table->freq > max_freq))
			break;

		mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_pll_param);
#ifdef BRINGUP_CVB_V_MARGIN
		mv = mv * (100 + BRINGUP_CVB_V_MARGIN) / 100;
#endif
		mv = round_cvb_voltage(mv, d->voltage_scale, align);
		mv = max(mv, min_mv);

		if (mv > d->max_mv) {
			pr_warn("tegra21_dvfs: %dmV for %s rate %lu above limit %dmV\n",
			     mv, cpu_lp_dvfs->clk_name, table->freq, d->max_mv);
			break;
		}

		/* fill in dvfs tables */
		cpu_lp_dvfs->freqs[j] = table->freq;
		cpu_lp_millivolts[j] = mv;
	}

	/* Table must not be empty */
	if (!j) {
		pr_err("tegra21_dvfs: invalid cpu lp dvfs table\n");
		return -ENOENT;
	}

	/* dvfs tables are successfully populated - fill in the rest */
	cpu_lp_dvfs->speedo_id = d->speedo_id;
	cpu_lp_dvfs->process_id = d->process_id;
	cpu_lp_dvfs->freqs_mult = d->freqs_mult;
	*max_freq_index = j - 1;

	return 0;
}

static void __init init_cpu_lp_dvfs_table(int *cpu_lp_max_freq_index)
{
	int i, ret;
	int cpu_lp_speedo_id = tegra_cpu_speedo_id(); /* FIXME cpu_lp_ */
	int cpu_lp_process_id = tegra_cpu_process_id(); /* FIXME cpu_lp_ */

	BUG_ON(cpu_lp_speedo_id >= ARRAY_SIZE(cpu_lp_max_freq));
	for (ret = 0, i = 0; i <  ARRAY_SIZE(cpu_lp_cvb_dvfs_table); i++) {
		struct cpu_cvb_dvfs *d = &cpu_lp_cvb_dvfs_table[i];
		unsigned long max_freq = cpu_lp_max_freq[cpu_lp_speedo_id];
		if (match_dvfs_one("cpu lp cvb", d->speedo_id, d->process_id,
				   cpu_lp_speedo_id, cpu_lp_process_id)) {
			ret = set_cpu_lp_dvfs_data(max_freq,
				d, &cpu_lp_dvfs, cpu_lp_max_freq_index);
			break;
		}
	}
	BUG_ON((i == ARRAY_SIZE(cpu_lp_cvb_dvfs_table)) || ret);
}

/*
 * Setup gpu dvfs tables from cvb data, determine nominal voltage for gpu rail,
 * and gpu maximum frequency. Error when gpu dvfs table can not be constructed
 * must never happen.
 */
static int __init set_gpu_dvfs_data(unsigned long max_freq,
	struct gpu_cvb_dvfs *d, struct dvfs *gpu_dvfs, int *max_freq_index)
{
	int i, j, thermal_ranges, mv;
	struct cvb_dvfs_table *table = NULL;
	int speedo = tegra_gpu_speedo_value();
	struct dvfs_rail *rail = &tegra21_dvfs_rail_vdd_gpu;
	struct rail_alignment *align = &rail->alignment;

	d->max_mv = round_voltage(d->max_mv, align, false);

	/*
	 * Init thermal trips, find number of thermal ranges; note that the
	 * first trip-point is used for voltage calculations within the lowest
	 * range, but should not be actually set. Hence, at least 2 trip-points
	 * must be specified.
	 */
	if (tegra_dvfs_rail_init_thermal_dvfs_trips(d->vts_trips_table, rail))
		return -ENOENT;
	thermal_ranges = rail->vts_cdev->trip_temperatures_num;
	rail->vts_cdev->trip_temperatures_num--;

	if (thermal_ranges < 2)
		WARN(1, "tegra21_dvfs: %d gpu trip: thermal dvfs is broken\n",
		     thermal_ranges);

	/*
	 * Use CVB table to calculate Vmin for each temperature range
	 */
	mv = get_cvb_voltage(
		speedo, d->speedo_scale, &d->cvb_vmin.cvb_pll_param);
	for (j = 0; j < thermal_ranges; j++) {
		int mvj = mv;
		int t = rail->vts_cdev->trip_temperatures[j];

		/* add Vmin thermal offset for this trip-point */
		mvj += get_cvb_t_voltage(speedo, d->speedo_scale,
			t, d->thermal_scale, &d->cvb_vmin.cvb_pll_param);
		mvj = round_cvb_voltage(mvj, d->voltage_scale, align);
		if (mvj < rail->min_millivolts) {
			pr_debug("tegra21_dvfs: gpu min %dmV below rail min %dmV\n",
			     mvj, rail->min_millivolts);
			mvj = rail->min_millivolts;
		}

		gpu_vmin[j] = mvj;
	}

	/*
	 * Use CVB table to fill in gpu dvfs frequencies and voltages. Each
	 * CVB entry specifies gpu frequency and CVB coefficients to calculate
	 * the respective voltage.
	 */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		table = &d->cvb_table[i];
		if (!table->freq || (table->freq > max_freq))
			break;

		mv = get_cvb_voltage(
			speedo, d->speedo_scale, &table->cvb_pll_param);
#ifdef BRINGUP_CVB_V_MARGIN
		mv = mv * (100 + BRINGUP_CVB_V_MARGIN) / 100;
#endif
		for (j = 0; j < thermal_ranges; j++) {
			int mvj = mv;
			int t = rail->vts_cdev->trip_temperatures[j];

			/* get thermal offset for this trip-point */
			mvj += get_cvb_t_voltage(speedo, d->speedo_scale,
				t, d->thermal_scale, &table->cvb_pll_param);
			mvj = round_cvb_voltage(mvj, d->voltage_scale, align);

			/* clip to minimum, abort if above maximum */
			mvj = max(mvj, gpu_vmin[j]);
			if (mvj > d->max_mv)
				break;

			/* update voltage for adjacent ranges bounded by this
			   trip-point (cvb & dvfs are transpose matrices) */
			gpu_millivolts[j][i] = mvj;
			if (j && (gpu_millivolts[j-1][i] < mvj))
				gpu_millivolts[j-1][i] = mvj;
		}
		/* Make sure all voltages for this frequency are below max */
		if (j < thermal_ranges)
			break;

		/* fill in gpu dvfs tables */
		gpu_dvfs->freqs[i] = table->freq;
	}

	/*
	 * Table must not be empty, must have at least one entry in range, and
	 * must specify monotonically increasing voltage on frequency dependency
	 * in each temperature range.
	 */
	if (!i || tegra_dvfs_init_thermal_dvfs_voltages(&gpu_millivolts[0][0],
		gpu_peak_millivolts, i, thermal_ranges, gpu_dvfs)) {
		pr_err("tegra21_dvfs: invalid gpu dvfs table\n");
		return -ENOENT;
	}

	/* Shift out the 1st trip-point */
	for (j = 1; j < thermal_ranges; j++)
		rail->vts_cdev->trip_temperatures[j - 1] =
		rail->vts_cdev->trip_temperatures[j];

	/* dvfs tables are successfully populated - fill in the gpu dvfs */
	gpu_dvfs->speedo_id = d->speedo_id;
	gpu_dvfs->process_id = d->process_id;
	gpu_dvfs->freqs_mult = d->freqs_mult;
	gpu_dvfs->dvfs_rail->nominal_millivolts = d->max_mv;

	*max_freq_index = i - 1;

	/* Init thermal floors */
	if (d->therm_floors_table[0]) /* if table contains at least one entry */
		tegra_dvfs_rail_init_vmin_thermal_profile(d->vmin_trips_table,
			d->therm_floors_table, rail, NULL);

	return 0;
}

static void __init init_gpu_dvfs_table(int *gpu_max_freq_index)
{
	int i, ret;
	int gpu_speedo_id = tegra_gpu_speedo_id();
	int gpu_process_id = tegra_gpu_process_id();

	BUG_ON(gpu_speedo_id >= ARRAY_SIZE(gpu_max_freq));
	for (ret = 0, i = 0; i < ARRAY_SIZE(gpu_cvb_dvfs_table); i++) {
		struct gpu_cvb_dvfs *d = &gpu_cvb_dvfs_table[i];
		unsigned long max_freq = gpu_max_freq[gpu_speedo_id];
		if (match_dvfs_one("gpu cvb", d->speedo_id, d->process_id,
				   gpu_speedo_id, gpu_process_id)) {
			ret = set_gpu_dvfs_data(max_freq,
				d, &gpu_dvfs, gpu_max_freq_index);
			break;
		}
	}
	BUG_ON((i == ARRAY_SIZE(gpu_cvb_dvfs_table)) || ret);
}

/*
 * Clip sku-based core nominal voltage to core DVFS voltage ladder
 */
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
		core_edp_voltage = 1225;	/* default 1.225V EDP limit */

	mv = min(mv, core_edp_voltage);
#ifndef CONFIG_TEGRA_CORE_DVFS
	mv = min(mv, 1000);		/* Vmax if scaling is disabled */
#endif
	/* Round nominal level down to the nearest core scaling step */
	for (i = 0; i < MAX_DVFS_FREQS; i++) {
		if ((core_millivolts[i] == 0) || (mv < core_millivolts[i]))
			break;
	}

	if (i == 0) {
		pr_err("tegra21_dvfs: unable to adjust core dvfs table to"
		       " nominal voltage %d\n", mv);
		return -ENOSYS;
	}
	return i - 1;
}

static int __init of_rails_init(struct device_node *dn)
{
	int i;

	if (!of_device_is_available(dn))
		return 0;

	for (i = 0; i < ARRAY_SIZE(tegra21_dvfs_rails); i++) {
		struct dvfs_rail *rail = tegra21_dvfs_rails[i];
		if (!of_tegra_dvfs_rail_node_parse(dn, rail)) {
			rail->stats.bin_uV = rail->alignment.step_uv;
			return 0;
		}
	}
	return -ENOENT;
}

static __initdata struct of_device_id tegra21_dvfs_rail_of_match[] = {
	{ .compatible = "nvidia,tegra210-dvfs-rail", .data = of_rails_init, },
	{ },
};

void __init tegra21x_init_dvfs(void)
{
	int soc_speedo_id = tegra_soc_speedo_id();
	int core_process_id = tegra_core_process_id();

	int i, ret;
	int core_nominal_mv_index;
	int gpu_max_freq_index = 0;
	int cpu_max_freq_index = 0;
	int cpu_lp_max_freq_index = 0;

#ifndef CONFIG_TEGRA_CORE_DVFS
	tegra_dvfs_core_disabled = true;
#endif
#ifndef CONFIG_TEGRA_CPU_DVFS
	tegra_dvfs_cpu_disabled = true;
#endif
#ifndef CONFIG_TEGRA_GPU_DVFS
	tegra_dvfs_gpu_disabled = true;
#endif

	of_tegra_dvfs_init(tegra21_dvfs_rail_of_match);

	/*
	 * Find nominal voltages for core (1st) and cpu rails before rail
	 * init. Nominal voltage index in core scaling ladder can also be
	 * used to determine max dvfs frequencies for all core clocks. In
	 * case of error disable core scaling and set index to 0, so that
	 * core clocks would not exceed rates allowed at minimum voltage.
	 */
	core_nominal_mv_index = get_core_nominal_mv_index(soc_speedo_id);
	if (core_nominal_mv_index < 0) {
		tegra21_dvfs_rail_vdd_core.disabled = true;
		tegra_dvfs_core_disabled = true;
		core_nominal_mv_index = 0;
	}
	tegra21_dvfs_rail_vdd_core.nominal_millivolts =
		core_millivolts[core_nominal_mv_index];

	/*
	 * Construct fast and slow CPU DVFS tables from CVB data; find maximum
	 * frequency, minimum  and nominal voltage for each CPU cluster, and
	 * combined rail limits (fast CPU should be initialized 1st).
	 */
	init_cpu_dvfs_table(&cpu_max_freq_index);
	init_cpu_lp_dvfs_table(&cpu_lp_max_freq_index);

	/*
	 * Construct GPU DVFS table from CVB data; find GPU maximum frequency,
	 * and nominal voltage.
	 */
	init_gpu_dvfs_table(&gpu_max_freq_index);

	/* Init core thermal profile */
	if (vdd_core_therm_floors_table[0])
		tegra_dvfs_rail_init_vmin_thermal_profile(
			vdd_core_vmin_trips_table, vdd_core_therm_floors_table,
			&tegra21_dvfs_rail_vdd_core, NULL);
	if (vdd_core_therm_caps_table[0])
		tegra_dvfs_rail_init_vmax_thermal_profile(
			vdd_core_vmax_trips_table, vdd_core_therm_caps_table,
			&tegra21_dvfs_rail_vdd_core, NULL);

	/* Init rail structures and dependencies */
	tegra_dvfs_init_rails(tegra21_dvfs_rails,
		ARRAY_SIZE(tegra21_dvfs_rails));

	/* Search core dvfs table for speedo/process matching entries and
	   initialize dvfs-ed clocks */
	if (!tegra_platform_is_linsim()) {
		for (i = 0; i <  ARRAY_SIZE(core_dvfs_table); i++) {
			struct dvfs *d = &core_dvfs_table[i];
			if (!match_dvfs_one(d->clk_name, d->speedo_id,
				d->process_id, soc_speedo_id, core_process_id))
				continue;
			tegra_init_dvfs_one(d, core_nominal_mv_index);
		}
	}

	/* Initialize matching gpu dvfs entry already found when nominal
	   voltage was determined */
	tegra_init_dvfs_one(&gpu_dvfs, gpu_max_freq_index);

	/* Initialize matching cpu dvfs entry already found when nominal
	   voltage was determined */
	tegra_init_dvfs_one(&cpu_dvfs, cpu_max_freq_index);
	tegra_init_dvfs_one(&cpu_lp_dvfs, cpu_lp_max_freq_index);

	/* Finally disable dvfs on rails if necessary */
	if (tegra_dvfs_core_disabled)
		tegra_dvfs_rail_disable(&tegra21_dvfs_rail_vdd_core);
	if (tegra_dvfs_cpu_disabled)
		tegra_dvfs_rail_disable(&tegra21_dvfs_rail_vdd_cpu);
	if (tegra_dvfs_gpu_disabled)
		tegra_dvfs_rail_disable(&tegra21_dvfs_rail_vdd_gpu);

	for (i = 0; i < ARRAY_SIZE(tegra21_dvfs_rails); i++) {
		struct dvfs_rail *rail = tegra21_dvfs_rails[i];
		pr_info("tegra dvfs: %s: nominal %dmV, offset %duV, step %duV, scaling %s\n",
			rail->reg_id, rail->nominal_millivolts,
			rail->alignment.offset_uv, rail->alignment.step_uv,
			rail->disabled ? "disabled" : "enabled");
	}
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

static struct core_dvfs_cap_table tegra21_core_cap_table[] = {
	{ .cap_name = "cap.vcore.c2bus" },
	{ .cap_name = "cap.vcore.c3bus" },
	{ .cap_name = "cap.vcore.sclk" },
	{ .cap_name = "cap.vcore.emc" },
	{ .cap_name = "cap.vcore.host1x" },
	{ .cap_name = "cap.vcore.mselect" },
};

static struct core_bus_limit_table tegra21_gpu_cap_syfs = {
	.limit_clk_name = "cap.profile.gbus",
	.refcnt_attr = {.attr = {.name = "gpu_cap_state", .mode = 0644} },
	.level_attr  = {.attr = {.name = "gpu_cap_rate", .mode = 0644} },
	.pm_qos_class = PM_QOS_GPU_FREQ_MAX,
};

static struct core_bus_limit_table tegra21_gpu_floor_sysfs = {
	.limit_clk_name = "floor.profile.gbus",
	.refcnt_attr = {.attr = {.name = "gpu_floor_state", .mode = 0644} },
	.level_attr  = {.attr = {.name = "gpu_floor_rate", .mode = 0644} },
	.pm_qos_class = PM_QOS_GPU_FREQ_MIN,
};

static struct core_bus_rates_table tegra21_gpu_rates_sysfs = {
	.bus_clk_name = "gbus",
	.rate_attr = {.attr = {.name = "gpu_rate", .mode = 0444} },
	.available_rates_attr = {
		.attr = {.name = "gpu_available_rates", .mode = 0444} },
};


static int __init tegra21_dvfs_init_core_cap(void)
{
	int ret;
	const int hack_core_millivolts = 0;

	cap_kobj = kobject_create_and_add("tegra_cap", kernel_kobj);
	if (!cap_kobj) {
		pr_err("tegra21_dvfs: failed to create sysfs cap object\n");
		return 0;
	}

	/* FIXME: skip core cap init b/c it's too slow on QT */
	if (tegra_platform_is_qt())
		ret = tegra_init_core_cap(
			tegra21_core_cap_table, ARRAY_SIZE(tegra21_core_cap_table),
			&hack_core_millivolts, 1, cap_kobj);
	else
		ret = tegra_init_core_cap(
			tegra21_core_cap_table, ARRAY_SIZE(tegra21_core_cap_table),
			core_millivolts, ARRAY_SIZE(core_millivolts), cap_kobj);

	if (ret) {
		pr_err("tegra21_dvfs: failed to init core cap interface (%d)\n",
		       ret);
		kobject_del(cap_kobj);
		return 0;
	}
	pr_info("tegra dvfs: tegra sysfs cap interface is initialized\n");

	gpu_kobj = kobject_create_and_add("tegra_gpu", kernel_kobj);
	if (!gpu_kobj) {
		pr_err("tegra21_dvfs: failed to create sysfs gpu object\n");
		return 0;
	}

	ret = tegra_init_shared_bus_cap(&tegra21_gpu_cap_syfs,
					1, gpu_kobj);
	if (ret) {
		pr_err("tegra21_dvfs: failed to init gpu cap interface (%d)\n",
		       ret);
		kobject_del(gpu_kobj);
		return 0;
	}

	ret = tegra_init_shared_bus_floor(&tegra21_gpu_floor_sysfs,
					  1, gpu_kobj);
	if (ret) {
		pr_err("tegra21_dvfs: failed to init gpu floor interface (%d)\n",
		       ret);
		kobject_del(gpu_kobj);
		return 0;
	}

	ret = tegra_init_sysfs_shared_bus_rate(&tegra21_gpu_rates_sysfs,
					       1, gpu_kobj);
	if (ret) {
		pr_err("tegra21_dvfs: failed to init gpu rates interface (%d)\n",
		       ret);
		kobject_del(gpu_kobj);
		return 0;
	}

	pr_info("tegra dvfs: tegra sysfs gpu interface is initialized\n");

	return 0;
}
late_initcall(tegra21_dvfs_init_core_cap);
