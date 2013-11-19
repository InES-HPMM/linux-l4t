/*
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

#include <linux/sysedp.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/power_supply.h>
#include <mach/edp.h>
#include "board.h"
#include "board-panel.h"

/* --- EDP consumers data --- */
/* TODO static unsigned int ov5693_states[] = { 0, 300 };*/
static unsigned int mt9m114_states[] = { 0, 150 };
static unsigned int sdhci_states[] = { 0, 966 };
static unsigned int speaker_states[] = { 0, 1080 };
static unsigned int wifi_states[] = { 0, 3070 };
static unsigned int modem_states[] = { 0, 4100 };

/* 10 inch panel */
static unsigned int pwm_backlight_states[] = {
	0, 425, 851, 1276, 1702, 2127, 2553, 2978, 3404, 3829, 4255
};
/* TODO
static unsigned int as364x_states[] = {
	0, 350, 700, 1050, 1400, 1750, 2100, 2450, 2800, 3150, 3500
};
*/

static struct sysedp_consumer_data shield_sysedp_consumer_data[] = {
	/* TODO SYSEDP_CONSUMER_DATA("ov5693", ov5693_states),*/
	SYSEDP_CONSUMER_DATA("mt9m114", mt9m114_states),
	SYSEDP_CONSUMER_DATA("speaker", speaker_states),
	SYSEDP_CONSUMER_DATA("wifi", wifi_states),
	SYSEDP_CONSUMER_DATA("pwm-backlight", pwm_backlight_states),
	SYSEDP_CONSUMER_DATA("sdhci-tegra.2", sdhci_states),
	SYSEDP_CONSUMER_DATA("sdhci-tegra.3", sdhci_states),
	/* TODO SYSEDP_CONSUMER_DATA("as364x", as364x_states), */
	SYSEDP_CONSUMER_DATA("modem", modem_states),
};

static struct sysedp_platform_data shield_sysedp_platform_data = {
	.consumer_data = shield_sysedp_consumer_data,
	.consumer_data_size = ARRAY_SIZE(shield_sysedp_consumer_data),
	.margin = 0,
};

static struct platform_device shield_sysedp_device = {
	.name = "sysedp",
	.id = -1,
	.dev = { .platform_data = &shield_sysedp_platform_data }
};

void __init shield_new_sysedp_init(void)
{
	int r;

	r = platform_device_register(&shield_sysedp_device);
	WARN_ON(r);
}

/* --- Battery monitor data --- */
static struct sysedp_batmon_ibat_lut shield_ibat_lut[] = {
/*-- temp in C, current in milli ampere --*/
	{  60, 7800 }, /* TODO */
	{  40, 7800 },
	{   0, 7800 },
	{ -30,    0 }
};

/*                           23C              */
static int rbat_data[] = {  118000,   /* 100% */
			    119000,   /* 80%  */
			    121000,   /* 60%  */
			    123000,   /* 40%  */
			    124000,   /* 10%  */
			    124000};  /* 0%   */
static int rbat_temp_axis[] = { 23 };
static int rbat_capacity_axis[] = { 100, 80, 60, 40, 10, 0 };

struct sysedp_batmon_rbat_lut shield_rbat_lut = {
	.temp_axis = rbat_temp_axis,
	.temp_size = ARRAY_SIZE(rbat_temp_axis),
	.capacity_axis = rbat_capacity_axis,
	.capacity_size = ARRAY_SIZE(rbat_capacity_axis),
	.data = rbat_data,
	.data_size = ARRAY_SIZE(rbat_data),
};

/* Fuel Gauge is BQ20z45 (SBS battery) */
static struct sysedp_batmon_ocv_lut shield_ocv_lut[] = {
	/*SOC, OCV in micro volt */
	{100,  8372010 },
	{95 ,  8163880 },
	{90 ,  8069280 },
	{85 ,  7970700 },
	{80 ,  7894100 },
	{75 ,  7820860 },
	{70 ,  7751890 },
	{65 ,  7691770 },
	{60 ,  7641110 },
	{55 ,  7598990 },
	{50 ,  7564200 },
	{45 ,  7534290 },
	{40 ,  7511410 },
	{35 ,  7491870 },
	{30 ,  7468380 },
	{25 ,  7435720 },
	{20 ,  7388720 },
	{15 ,  7338370 },
	{10 ,  7219650 },
	{0  ,  5999850 },
};

static struct sysedp_batmon_calc_platform_data shield_batmon_pdata = {
	.power_supply = "sbs-battery",
	.r_const = 70000,    /* in micro ohm */
	.vsys_min = 5880000, /* in micro volt*/
	.ibat_lut = shield_ibat_lut,
	.rbat_lut = &shield_rbat_lut,
	.ocv_lut = shield_ocv_lut,
};

static struct platform_device shield_batmon_device = {
	.name = "sysedp_batmon_calc",
	.id = -1,
	.dev = { .platform_data = &shield_batmon_pdata }
};

void __init shield_sysedp_batmon_init(void)
{
	int r;

	if (get_power_supply_type() != POWER_SUPPLY_TYPE_BATTERY) {
		/* modify platform data on-the-fly to enable virtual battery */
		shield_batmon_pdata.power_supply = "test_battery";
		shield_batmon_pdata.update_interval = 2000;
	}

	r = platform_device_register(&shield_batmon_device);
	WARN_ON(r);
}

/* --- sysedp capping device data --- */
/* --- for 780D sku -- */
static struct tegra_sysedp_corecap shield_sysedp_corecap[] = {
	{ 5000,  {2000,  108000, 924000 },  {2000, 108000, 924000 } },
	{ 6000,  {3000,  108000, 924000 },  {2000, 180000, 924000 } },
	{ 7000,  {4000,  108000, 924000 },  {2000, 252000, 924000 } },
	{ 8000,  {5000,  108000, 924000 },  {2000, 396000, 924000 } },
	{ 9000,  {6000,  108000, 924000 },  {2000, 396000, 924000 } },
	{ 10000, {7000,  108000, 924000 },  {4000, 396000, 924000 } },
	{ 11000, {7000,  108000, 924000 },  {4000, 468000, 924000 } },
	{ 12000, {8000,  108000, 924000 },  {3000, 540000, 924000 } },
	{ 13000, {9000,  108000, 924000 },  {5000, 540000, 924000 } },
	{ 14000, {10000, 108000, 924000 },  {3000, 648000, 924000 } },
	{ 15000, {11000, 108000, 924000 },  {4000, 648000, 924000 } },
	{ 16000, {11000, 180000, 924000 },  {3000, 708000, 924000 } },
	{ 17000, {12000, 180000, 924000 },  {5000, 708000, 924000 } },
	{ 18000, {13000, 108000, 924000 },  {3000, 756000, 924000 } },
	{ 19000, {14000, 108000, 924000 },  {5000, 756000, 924000 } },
	{ 20000, {14000, 180000, 924000 },  {3000, 804000, 924000 } },
	{ 21000, {14000, 252000, 924000 },  {4000, 804000, 924000 } },
	{ 22000, {16000, 108000, 924000 },  {3000, 853000, 924000 } },
	{ 23000, {16000, 180000, 924000 },  {5000, 853000, 924000 } },
};
static struct tegra_sysedp_platform_data shield_sysedp_dynamic_capping_platdata = {
	.corecap = shield_sysedp_corecap,
	.corecap_size = ARRAY_SIZE(shield_sysedp_corecap),
	.core_gain = 100,
	.init_req_watts = 20000,
};

static struct platform_device shield_sysedp_dynamic_capping = {
	.name = "sysedp_dynamic_capping",
	.id = -1,
	.dev = { .platform_data = &shield_sysedp_dynamic_capping_platdata }
};

void __init shield_sysedp_dynamic_capping_init(void)
{
	int r;

	shield_sysedp_dynamic_capping_platdata.cpufreq_lim = tegra_get_system_edp_entries(
		&shield_sysedp_dynamic_capping_platdata.cpufreq_lim_size);
	if (!shield_sysedp_dynamic_capping_platdata.cpufreq_lim) {
		WARN_ON(1);
		return;
	}

	r = platform_device_register(&shield_sysedp_dynamic_capping);
	WARN_ON(r);
}
