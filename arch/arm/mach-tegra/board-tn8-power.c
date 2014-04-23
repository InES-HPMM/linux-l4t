/*
 * arch/arm/mach-tegra/board-tn8-power.c
 *
 * Copyright (c) 2014 NVIDIA Corporation. All rights reserved.
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
#include "common.h"
#include "iomap.h"
#include "tegra-board-id.h"
#include "battery-ini-model-data.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

static u32 tegra_chip_id;
#define IS_T13X			(tegra_chip_id == TEGRA_CHIPID_TEGRA13)

struct batt_thermistor_adc_table {
	int temp;
	int min_volt;
	int avg_volt;
	int max_volt;
};

/* This table is only for P1761-A00 and P1761-A01*/
/* -40 to 125 degC */
static struct batt_thermistor_adc_table adc_table_default[] = {
	{-40, 2315, 2361, 2401}, {-39, 2299, 2345, 2386},
	{-38, 2283, 2329, 2369}, {-37, 2266, 2312, 2352},
	{-36, 2248, 2294, 2335}, {-35, 2230, 2276, 2316},
	{-34, 2211, 2257, 2297}, {-33, 2191, 2237, 2278},
	{-32, 2171, 2217, 2258}, {-31, 2150, 2196, 2237},
	{-30, 2129, 2175, 2215}, {-29, 2107, 2153, 2193},
	{-28, 2085, 2130, 2170}, {-27, 2062, 2107, 2147},
	{-26, 2038, 2083, 2123}, {-25, 2014, 2059, 2099},
	{-24, 1990, 2034, 2074}, {-23, 1964, 2008, 2048},
	{-22, 1939, 1982, 2022}, {-21, 1913, 1956, 1995},
	{-20, 1886, 1929, 1968}, {-19, 1860, 1902, 1941},
	{-18, 1832, 1874, 1913}, {-17, 1805, 1846, 1884},
	{-16, 1777, 1818, 1856}, {-15, 1749, 1789, 1827},
	{-14, 1720, 1760, 1797}, {-13, 1692, 1731, 1767},
	{-12, 1663, 1701, 1737}, {-11, 1634, 1672, 1707},
	{-10, 1604, 1642, 1677}, {-9, 1575, 1612, 1646},
	{-8, 1546, 1582, 1616},  {-7, 1516, 1552, 1585},
	{-6, 1487, 1522, 1554},  {-5, 1457, 1491, 1524},
	{-4, 1428, 1461, 1493},  {-3, 1399, 1431, 1462},
	{-2, 1369, 1401, 1432},  {-1, 1340, 1371, 1401},
	{0, 1311, 1342, 1371},   {1, 1283, 1312, 1341},
	{2, 1254, 1283, 1311},   {3, 1226, 1254, 1281},
	{4, 1198, 1225, 1252},   {5, 1170, 1197, 1222},
	{6, 1143, 1169, 1193},   {7, 1116, 1141, 1165},
	{8, 1089, 1113, 1137},   {9, 1062, 1086, 1109},
	{10, 1036, 1059, 1081},  {11, 1011, 1033, 1054},
	{12, 986, 1007, 1028},   {13, 961, 982, 1002},
	{14, 936, 956, 976},     {15, 912, 932, 951},
	{16, 889, 908, 926},     {17, 866, 884, 901},
	{18, 843, 861, 877},     {19, 821, 838, 854},
	{20, 799, 815, 831},     {21, 778, 793, 809},
	{22, 757, 772, 787},     {23, 737, 751, 765},
	{24, 717, 731, 744},     {25, 697, 711, 724},
	{26, 678, 691, 704},     {27, 659, 672, 685},
	{28, 641, 654, 666},     {29, 623, 635, 648},
	{30, 605, 618, 630},     {31, 588, 600, 613},
	{32, 571, 584, 596},     {33, 555, 567, 579},
	{34, 540, 551, 563},     {35, 524, 536, 548},
	{36, 509, 521, 532},     {37, 495, 506, 517},
	{38, 481, 492, 503},     {39, 467, 478, 489},
	{40, 454, 464, 475},     {41, 441, 451, 462},
	{42, 428, 439, 449},     {43, 416, 426, 437},
	{44, 404, 414, 424},     {45, 392, 402, 413},
	{46, 381, 391, 401},     {47, 370, 380, 390},
	{48, 360, 369, 379},     {49, 349, 359, 369},
	{50, 340, 349, 358},     {51, 330, 339, 348},
	{52, 321, 330, 339},     {53, 311, 320, 329},
	{54, 303, 311, 320},     {55, 294, 303, 311},
	{56, 286, 294, 303},     {57, 278, 286, 295},
	{58, 270, 278, 286},     {59, 262, 270, 279},
	{60, 255, 263, 271},     {61, 248, 256, 264},
	{62, 241, 249, 256},     {63, 234, 242, 249},
	{64, 228, 235, 243},     {65, 222, 229, 236},
	{66, 215, 223, 230},     {67, 210, 217, 224},
	{68, 204, 211, 218},     {69, 198, 205, 212},
	{70, 193, 199, 206},     {71, 188, 194, 201},
	{72, 183, 189, 195},     {73, 178, 184, 190},
	{74, 173, 179, 185},     {75, 168, 174, 180},
	{76, 164, 170, 176},     {77, 159, 165, 171},
	{78, 155, 161, 167},     {79, 151, 157, 162},
	{80, 147, 152, 158},     {81, 143, 149, 154},
	{82, 139, 145, 150},     {83, 136, 141, 146},
	{84, 132, 137, 143},     {85, 129, 134, 139},
	{86, 125, 130, 135},     {87, 122, 127, 132},
	{88, 119, 124, 129},     {89, 116, 121, 125},
	{90, 113, 118, 122},     {91, 110, 115, 119},
	{92, 107, 112, 116},     {93, 105, 109, 113},
	{94, 102, 106, 111},     {95, 100, 104, 108},
	{96, 97, 101, 105},      {97, 95, 99, 103},
	{98, 92, 96, 100},       {99, 90, 94, 98},
	{100, 88, 92, 96},       {101, 86, 89, 93},
	{102, 84, 87, 91},       {103, 82, 85, 89},
	{104, 80, 83, 87},       {105, 78, 81, 85},
	{106, 76, 79, 83},       {107, 74, 77, 81},
	{108, 72, 75, 79},       {109, 70, 74, 77},
	{110, 69, 72, 75},       {111, 67, 70, 74},
	{112, 66, 69, 72},       {113, 64, 67, 70},
	{114, 63, 65, 69},       {115, 61, 64, 67},
	{116, 60, 63, 65},       {117, 58, 61, 64},
	{118, 57, 60, 63},       {119, 56, 58, 61},
	{120, 54, 57, 60},       {121, 53, 56, 58},
	{122, 52, 55, 57},       {123, 51, 53, 56},
	{124, 50, 52, 55},       {125, 49, 51, 54},
};

static struct batt_thermistor_adc_table *adc_tbl;
static int adc_tbl_size;

static int tn8_batt_adc_to_temp(struct gadc_thermal_platform_data *pdata,
				int val, int val2)
{
	int temp, i, f_part = 0;
	struct batt_thermistor_adc_table *entry = adc_tbl;

	for (i = 0; i < adc_tbl_size; i++) {
		if (val >= entry->avg_volt)
			break;

		entry++;
	}

	if (i == adc_tbl_size)
		temp = pdata->last_index_temp * 10000;
	else {
		/* Find floating part with interpolate computing */
		f_part = ((val - entry->min_volt) * 10)/
				(entry->max_volt - entry->min_volt);
		temp = entry->temp;
		temp = (temp * 10000) + (f_part * 1000);
	}

	return temp;
}

static struct gadc_thermal_platform_data gadc_thermal_battery_pdata = {
	.iio_channel_name = "battery-temp-channel",
	.tz_name = "battery-temp",
	.temp_offset = 0,
	.adc_to_temp = tn8_batt_adc_to_temp,
	.first_index_temp = -40,
	.last_index_temp = 125,
};

static struct platform_device gadc_thermal_battery = {
	.name   = "generic-adc-thermal",
	.id     = 0,
	.dev    = {
		.platform_data = &gadc_thermal_battery_pdata,
	},
};

int __init tn8_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	/* TPS65913: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	if ((board_info.board_id == BOARD_P1761) &&
		(board_info.fab < BOARD_FAB_A02)) {
		adc_tbl = adc_table_default;
		adc_tbl_size = ARRAY_SIZE(adc_table_default);
		platform_device_register(&gadc_thermal_battery);
	}

	return 0;
}

int __init tn8_edp_init(void)
{
	unsigned int regulator_mA;

	if (!tegra_chip_id)
		tegra_chip_id = tegra_get_chip_id();

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		if (IS_T13X)
			regulator_mA = 16800;
		else
			regulator_mA = 12000;
	}

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	/* gpu maximum current */
	regulator_mA = 11200;
	pr_info("%s: GPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_gpu_edp_limits(regulator_mA);

	return 0;
}
