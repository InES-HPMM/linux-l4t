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

/* This table is only for P1761-A02 */
/* -40 to 125 degC */
static struct batt_thermistor_adc_table adc_table_a02[] = {
	{-40, 3622, 3649, 3671}, {-39, 3611, 3638, 3660},
	{-38, 3599, 3626, 3649}, {-37, 3586, 3614, 3637},
	{-36, 3573, 3601, 3624}, {-35, 3560, 3588, 3611},
	{-34, 3545, 3574, 3598}, {-33, 3531, 3559, 3583},
	{-32, 3515, 3544, 3569}, {-31, 3499, 3529, 3553},
	{-30, 3482, 3512, 3537}, {-29, 3465, 3495, 3521},
	{-28, 3447, 3478, 3503}, {-27, 3429, 3459, 3485},
	{-26, 3409, 3440, 3467}, {-25, 3389, 3421, 3448},
	{-24, 3369, 3400, 3428}, {-23, 3348, 3379, 3407},
	{-22, 3326, 3358, 3386}, {-21, 3303, 3335, 3363},
	{-20, 3280, 3312, 3341}, {-19, 3256, 3288, 3317},
	{-18, 3231, 3264, 3293}, {-17, 3206, 3239, 3268},
	{-16, 3180, 3213, 3243}, {-15, 3153, 3186, 3216},
	{-14, 3125, 3159, 3189}, {-13, 3097, 3131, 3161},
	{-12, 3069, 3103, 3133}, {-11, 3039, 3073, 3104},
	{-10, 3009, 3044, 3074}, {-9, 2979, 3013, 3044},
	{-8, 2948, 2982, 3013},  {-7, 2916, 2950, 2981},
	{-6, 2884, 2918, 2949},  {-5, 2851, 2885, 2916},
	{-4, 2817, 2852, 2883},  {-3, 2784, 2818, 2849},
	{-2, 2749, 2783, 2814},  {-1, 2715, 2748, 2780},
	{0, 2679, 2713, 2744},   {1, 2644, 2677, 2708},
	{2, 2608, 2641, 2672},   {3, 2572, 2605, 2635},
	{4, 2535, 2568, 2598},   {5, 2498, 2531, 2561},
	{6, 2461, 2493, 2524},   {7, 2424, 2456, 2486},
	{8, 2387, 2418, 2448},   {9, 2349, 2380, 2409},
	{10, 2311, 2342, 2371},  {11, 2273, 2304, 2333},
	{12, 2236, 2266, 2294},  {13, 2198, 2227, 2255},
	{14, 2160, 2189, 2217},  {15, 2122, 2151, 2178},
	{16, 2084, 2113, 2139},  {17, 2047, 2074, 2101},
	{18, 2009, 2036, 2062},  {19, 1972, 1999, 2024},
	{20, 1935, 1961, 1986},  {21, 1898, 1923, 1948},
	{22, 1861, 1886, 1910},  {23, 1825, 1849, 1872},
	{24, 1789, 1812, 1835},  {25, 1753, 1776, 1798},
	{26, 1717, 1740, 1762},  {27, 1681, 1704, 1727},
	{28, 1645, 1669, 1691},  {29, 1610, 1634, 1657},
	{30, 1576, 1599, 1622},  {31, 1541, 1565, 1588},
	{32, 1508, 1531, 1554},  {33, 1474, 1498, 1521},
	{34, 1441, 1465, 1488},  {35, 1409, 1433, 1456},
	{36, 1377, 1401, 1424},  {37, 1346, 1370, 1393},
	{38, 1315, 1339, 1362},  {39, 1285, 1308, 1332},
	{40, 1255, 1278, 1302},  {41, 1225, 1249, 1272},
	{42, 1197, 1220, 1244},  {43, 1168, 1192, 1215},
	{44, 1141, 1164, 1187},  {45, 1114, 1137, 1160},
	{46, 1087, 1110, 1133},  {47, 1061, 1084, 1107},
	{48, 1035, 1058, 1081},  {49, 1010, 1033, 1056},
	{50, 986, 1008, 1031},   {51, 962, 984, 1007},
	{52, 938, 960, 983},     {53, 915, 937, 959},
	{54, 893, 915, 937},     {55, 871, 893, 914},
	{56, 849, 871, 893},     {57, 829, 850, 871},
	{58, 808, 829, 850},     {59, 788, 809, 830},
	{60, 769, 789, 810},     {61, 750, 770, 791},
	{62, 731, 751, 772},     {63, 713, 733, 753},
	{64, 695, 715, 735},     {65, 678, 697, 717},
	{66, 661, 680, 700},     {67, 645, 664, 683},
	{68, 629, 647, 667},     {69, 613, 632, 651},
	{70, 598, 616, 635},     {71, 583, 601, 620},
	{72, 569, 586, 605},     {73, 554, 572, 590},
	{74, 541, 558, 576},     {75, 527, 545, 562},
	{76, 514, 531, 549},     {77, 502, 518, 535},
	{78, 489, 506, 523},     {79, 477, 494, 510},
	{80, 466, 482, 498},     {81, 454, 470, 486},
	{82, 443, 459, 475},     {83, 432, 447, 463},
	{84, 422, 437, 452},     {85, 411, 426, 442},
	{86, 401, 416, 431},     {87, 392, 406, 421},
	{88, 382, 396, 411},     {89, 373, 387, 401},
	{90, 364, 378, 392},     {91, 355, 369, 383},
	{92, 347, 360, 374},     {93, 338, 351, 365},
	{94, 330, 343, 357},     {95, 322, 335, 348},
	{96, 315, 327, 340},     {97, 307, 320, 332},
	{98, 300, 312, 325},     {99, 293, 305, 317},
	{100, 286, 298, 310},    {101, 279, 291, 303},
	{102, 273, 284, 296},    {103, 266, 278, 289},
	{104, 260, 271, 283},    {105, 254, 265, 276},
	{106, 248, 259, 270},    {107, 242, 253, 264},
	{108, 237, 247, 258},    {109, 232, 242, 252},
	{110, 226, 236, 247},    {111, 221, 231, 241},
	{112, 216, 226, 236},    {113, 211, 221, 231},
	{114, 206, 216, 225},    {115, 202, 211, 221},
	{116, 197, 206, 216},    {117, 193, 202, 211},
	{118, 188, 197, 206},    {119, 184, 193, 202},
	{120, 180, 189, 198},    {121, 176, 185, 193},
	{122, 172, 181, 189},    {123, 169, 177, 185},
	{124, 165, 173, 181},    {125, 161, 169, 177}
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
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	/* TPS65913: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	if (board_info.board_id == BOARD_P1761) {
		platform_device_register(&gadc_thermal_battery);

		if (board_info.fab == BOARD_FAB_A02) {
			adc_tbl = adc_table_a02;
			adc_tbl_size = ARRAY_SIZE(adc_table_a02);
		} else {
			adc_tbl = adc_table_default;
			adc_tbl_size = ARRAY_SIZE(adc_table_default);
		}
	}

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
		regulator_mA = 12000;

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	/* gpu maximum current */
	regulator_mA = 11200;
	pr_info("%s: GPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_gpu_edp_limits(regulator_mA);

	return 0;
}
