/*
 * arch/arm/mach-tegra/board-tn8-power.c
 *
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/edp.h>
#include <linux/edpdev.h>
#include <mach/edp.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>

#include <linux/gpio.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/power/power_supply_extcon.h>
#include <linux/max17048_battery.h>
#include <linux/tegra-soc.h>
#include <linux/generic_adc_thermal.h>

#include <mach/irqs.h>

#include <asm/mach-types.h>
#include <linux/power/sbs-battery.h>

#include "pm.h"
#include "board.h"
#include "tegra-board-id.h"
#include "board-common.h"
#include "board-ardbeg.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include "iomap.h"
#include "tegra-board-id.h"
#include "battery-ini-model-data.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

/* -40 to 125 degC */
static int tn8_batt_temperature_table[] = {
	259, 266, 272, 279, 286, 293, 301, 308,
	316, 324, 332, 340, 349, 358, 367, 376,
	386, 395, 405, 416, 426, 437, 448, 459,
	471, 483, 495, 508, 520, 533, 547, 561,
	575, 589, 604, 619, 634, 650, 666, 682,
	699, 716, 733, 751, 769, 787, 806, 825,
	845, 865, 885, 905, 926, 947, 969, 990,
	1013, 1035, 1058, 1081, 1104, 1127, 1151, 1175,
	1199, 1224, 1249, 1273, 1298, 1324, 1349, 1374,
	1400, 1426, 1451, 1477, 1503, 1529, 1554, 1580,
	1606, 1631, 1657, 1682, 1707, 1732, 1757, 1782,
	1807, 1831, 1855, 1878, 1902, 1925, 1948, 1970,
	1992, 2014, 2036, 2057, 2077, 2097, 2117, 2136,
	2155, 2174, 2192, 2209, 2227, 2243, 2259, 2275,
	2291, 2305, 2320, 2334, 2347, 2361, 2373, 2386,
	2397, 2409, 2420, 2430, 2441, 2450, 2460, 2469,
	2478, 2486, 2494, 2502, 2509, 2516, 2523, 2529,
	2535, 2541, 2547, 2552, 2557, 2562, 2567, 2571,
	2575, 2579, 2583, 2587, 2590, 2593, 2596, 2599,
	2602, 2605, 2607, 2609, 2611, 2614, 2615, 2617,
	2619, 2621, 2622, 2624, 2625, 2626,
};

static struct gadc_thermal_platform_data gadc_thermal_battery_pdata = {
	.iio_channel_name = "battery-temp-channel",
	.tz_name = "battery-temp",
	.temp_offset = 0,
	.adc_to_temp = NULL,
	.adc_temp_lookup = tn8_batt_temperature_table,
	.lookup_table_size = ARRAY_SIZE(tn8_batt_temperature_table),
	.first_index_temp = 125,
	.last_index_temp = -40,
};

static struct platform_device gadc_thermal_battery = {
	.name   = "generic-adc-thermal",
	.id     = 0,
	.dev    = {
		.platform_data = &gadc_thermal_battery_pdata,
	},
};


static struct power_supply_extcon_plat_data extcon_pdata = {
	.extcon_name = "tegra-udc",
};

static struct platform_device power_supply_extcon_device = {
	.name	= "power-supply-extcon",
	.id	= -1,
	.dev	= {
		.platform_data = &extcon_pdata,
	},
};

int __init tn8_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* TPS65913: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	platform_device_register(&gadc_thermal_battery);
	platform_device_register(&power_supply_extcon_device);
	return 0;
}

int __init tn8_fixed_regulator_init(void)
{
	return 0;
}

int __init tn8_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 9000;

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	return 0;
}
static struct edp_manager tn8_sysedp_manager = {
	.name = "battery",
	.max = 19000  /*TODO*/
};

void __init tn8_sysedp_init(void)
{
	struct edp_governor *g;
	int r;

	if (!IS_ENABLED(CONFIG_EDP_FRAMEWORK))
		return;

	if (get_power_supply_type() != POWER_SUPPLY_TYPE_BATTERY)
		tn8_sysedp_manager.max = INT_MAX;

	r = edp_register_manager(&tn8_sysedp_manager);
	WARN_ON(r);
	if (r)
		return;

	/* start with priority governor */
	g = edp_get_governor("priority");
	WARN_ON(!g);
	if (!g)
		return;

	r = edp_set_governor(&tn8_sysedp_manager, g);
	WARN_ON(r);
}

static unsigned int tn8_psydepl_states[] = {
	18000, 17000, 16000, 15000, 14000, 13000, 12000, 11000,
	 9900,  9600,  9300,  9000,  8700,  8400,  8100,  7800,
	 7500,  7200,  6900,  6600,  6300,  6000,  5800,  5600,
	 5400,  5200,  5000,  4800,  4600,  4400,  4200,  4000,
	 3800,  3600,  3400,  3200,  3000,  2800,  2600,  2400,
	 2200,  2000,  1900,  1800,  1700,  1600,  1500,  1400,
	 1300,  1200,  1100,  1000,   900,   800,   700,   600,
	  500,   400,   300,   200,   100,     0

};

/* Temperature in celcius */
static struct psy_depletion_ibat_lut tn8_ibat_lut[] = {
	{  60, 6150 }, /* TODO */
	{  40, 6150 },
	{   0, 6150 },
	{ -30,    0 }

};

static struct psy_depletion_rbat_lut tn8_rbat_lut[] = {
	{ 100,	76000  },
	{  88,	85000  },
	{  75,  102000 },
	{  63,  93000  },
	{  50,  85000  },
	{  38,  93000  },
	{  25,  102000 },
	{  13,  119000 },
	{   0,  119000 }
};

static struct psy_depletion_ocv_lut tn8_ocv_lut[] = {
	{ 100, 4200000 },
	{  90, 4151388 },
	{  80, 4064953 },
	{  70, 3990914 },
	{  60, 3916230 },
	{  50, 3863778 },
	{  40, 3807535 },
	{  30, 3781554 },
	{  20, 3761117 },
	{  10, 3663381 },
	{   0, 3514236 }
};

static struct psy_depletion_platform_data tn8_psydepl_pdata = {
	.power_supply = "battery",
	.states = tn8_psydepl_states,
	.num_states = ARRAY_SIZE(tn8_psydepl_states),
	.e0_index = 16, /* depletion client at around 30% charge remaining */
	.r_const = 38000,
	.vsys_min = 3250000, /*TODO*/
	.vcharge = 4200000, /* voltage at 100% */
	.ibat_nom = 6150,
	.ibat_lut = tn8_ibat_lut,
	.rbat_lut = tn8_rbat_lut,
	/*.ocv_lut = tn8_ocv_lut*/

};

static struct platform_device tn8_psydepl_device = {
	.name = "psy_depletion",
	.id = -1,
	.dev = { .platform_data = &tn8_psydepl_pdata }
};

void __init tn8_sysedp_psydepl_init(void)
{
	int r;

	r = platform_device_register(&tn8_psydepl_device);
	WARN_ON(r);
}

static struct tegra_sysedp_corecap tn8_sysedp_corecap[] = {
	{  1000, {  1000, 240000, 204000 }, {  1000, 240000, 204000 } },
	{  2000, {  1000, 240000, 204000 }, {  1000, 240000, 204000 } },
	{  3000, {  1000, 240000, 204000 }, {  1000, 240000, 204000 } },
	{  4000, {  1000, 240000, 204000 }, {  1000, 240000, 204000 } },
	{  5000, {  1000, 240000, 204000 }, {  1000, 240000, 204000 } },
	{  5500, {  1000, 240000, 312000 }, {  1000, 240000, 312000 } },
	{  6000, {  1283, 240000, 312000 }, {  1283, 240000, 312000 } },
	{  6500, {  1783, 240000, 312000 }, {  1275, 324000, 312000 } },
	{  7000, {  1843, 240000, 624000 }, {  1975, 324000, 408000 } },
	{  7500, {  2343, 240000, 624000 }, {  1806, 420000, 408000 } },
	{  8000, {  2843, 240000, 624000 }, {  2306, 420000, 624000 } },
	{  8500, {  3343, 240000, 624000 }, {  2806, 420000, 624000 } },
	{  9000, {  3843, 240000, 624000 }, {  2606, 420000, 792000 } },
	{  9500, {  4343, 240000, 624000 }, {  2898, 528000, 792000 } },
	{ 10000, {  4565, 240000, 792000 }, {  3398, 528000, 792000 } },
	{ 10500, {  5065, 240000, 792000 }, {  3898, 528000, 792000 } },
	{ 11000, {  5565, 240000, 792000 }, {  4398, 528000, 792000 } },
	{ 11500, {  6065, 240000, 792000 }, {  3777, 600000, 792000 } },
	{ 12000, {  6565, 240000, 792000 }, {  4277, 600000, 792000 } },
	{ 12500, {  7065, 240000, 792000 }, {  4777, 600000, 792000 } },
	{ 13000, {  7565, 240000, 792000 }, {  5277, 600000, 792000 } },
	{ 13500, {  8065, 240000, 792000 }, {  5777, 600000, 792000 } },
	{ 14000, {  8565, 240000, 792000 }, {  6277, 600000, 792000 } },
	{ 14500, {  9065, 240000, 792000 }, {  6777, 600000, 792000 } },
	{ 15000, {  9565, 384000, 792000 }, {  7277, 600000, 792000 } },
	{ 15500, {  9565, 468000, 792000 }, {  7777, 600000, 792000 } },
	{ 16000, { 10565, 468000, 792000 }, {  8277, 600000, 792000 } },
	{ 16500, { 11065, 468000, 792000 }, {  8777, 600000, 792000 } },
	{ 17000, { 11565, 468000, 792000 }, {  9277, 600000, 792000 } },
	{ 17500, { 12065, 468000, 792000 }, {  9577, 600000, 792000 } },
	{ 18000, { 12565, 468000, 792000 }, { 10277, 600000, 792000 } },
	{ 18500, { 13065, 468000, 792000 }, { 10777, 600000, 792000 } },
	{ 19000, { 13565, 468000, 792000 }, { 11277, 600000, 792000 } },
	{ 19500, { 14065, 468000, 792000 }, { 11777, 600000, 792000 } },
	{ 20000, { 14565, 468000, 792000 }, { 12277, 600000, 792000 } },
	{ 23000, { 14565, 600000, 792000 }, { 14565, 600000, 792000 } },
};

static struct tegra_sysedp_platform_data tn8_sysedp_platdata = {
	.corecap = tn8_sysedp_corecap,
	.corecap_size = ARRAY_SIZE(tn8_sysedp_corecap),
	.core_gain = 130,
	.init_req_watts = 20000,
	/*.bbc = "bbc"*/
};

static struct platform_device tn8_sysedp_device = {
	.name = "tegra_sysedp",
	.id = -1,
	.dev = { .platform_data = &tn8_sysedp_platdata }
};

void __init tn8_sysedp_core_init(void)
{
	int r;

	tn8_sysedp_platdata.cpufreq_lim = tegra_get_system_edp_entries(
			&tn8_sysedp_platdata.cpufreq_lim_size);
	if (!tn8_sysedp_platdata.cpufreq_lim) {
		WARN_ON(1);
		return;
	}

	r = platform_device_register(&tn8_sysedp_device);
	WARN_ON(r);
}
