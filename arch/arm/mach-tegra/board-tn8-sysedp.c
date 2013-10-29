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
static unsigned int ov5693_states[] = { 0, 300 };
static unsigned int mt9m114_states[] = { 0, 150 };
static unsigned int sdhci_states[] = { 0, 966 };
static unsigned int speaker_states[] = { 0, 1080 };
static unsigned int wifi_states[] = { 0, 1020 };
/* 10" panel */
static unsigned int pwm_backlight_states[] = {
	0, 425, 851, 1276, 1702, 2127, 2553, 2978, 3404, 3829, 4255
};
static unsigned int as364x_states[] = {
	0, 350, 700, 1050, 1400, 1750, 2100, 2450, 2800, 3150, 3500
};

static struct sysedp_consumer_data tn8_sysedp_consumer_data[] = {
	SYSEDP_CONSUMER_DATA("ov5693", ov5693_states),
	SYSEDP_CONSUMER_DATA("mt9m114", mt9m114_states),
	SYSEDP_CONSUMER_DATA("speaker", speaker_states),
	SYSEDP_CONSUMER_DATA("wifi", wifi_states),
	SYSEDP_CONSUMER_DATA("pwm-backlight", pwm_backlight_states),
	SYSEDP_CONSUMER_DATA("sdhci-tegra.2", sdhci_states),
	SYSEDP_CONSUMER_DATA("sdhci-tegra.3", sdhci_states),
	SYSEDP_CONSUMER_DATA("as364x", as364x_states),
};

static struct sysedp_platform_data tn8_sysedp_platform_data = {
	.consumer_data = tn8_sysedp_consumer_data,
	.consumer_data_size = ARRAY_SIZE(tn8_sysedp_consumer_data),
	.margin = 0,
};

static struct platform_device tn8_sysedp_device = {
	.name = "sysedp",
	.id = -1,
	.dev = { .platform_data = &tn8_sysedp_platform_data }
};

void __init tn8_new_sysedp_init(void)
{
	int r;

	r = platform_device_register(&tn8_sysedp_device);
	WARN_ON(r);
}

/* --- Battery monitor data --- */
static struct sysedp_batmon_ibat_lut tn8_ibat_lut[] = {
	{  60, 6150 }, /* TODO until battery temp is fixed*/
	{  40, 6150 },
	{   0, 6150 },
	{ -30,    0 }
};

static struct sysedp_batmon_rbat_lut tn8_rbat_lut[] = {
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

static struct sysedp_batmon_calc_platform_data tn8_batmon_pdata = {
	.power_supply = "battery",
	.r_const = 38000,
	.vsys_min = 2900000,
	.ibat_lut = tn8_ibat_lut,
	.rbat_lut = tn8_rbat_lut,
};

static struct platform_device tn8_batmon_device = {
	.name = "sysedp_batmon_calc",
	.id = -1,
	.dev = { .platform_data = &tn8_batmon_pdata }
};

void __init tn8_sysedp_batmon_init(void)
{
	int r;

	if (get_power_supply_type() != POWER_SUPPLY_TYPE_BATTERY) {
		/* modify platform data on-the-fly to enable virtual battery */
		tn8_batmon_pdata.power_supply = "test_battery";
		tn8_batmon_pdata.update_interval = 2000;
	}

	r = platform_device_register(&tn8_batmon_device);
	WARN_ON(r);
}

/* --- sysedp capping device data --- */
static struct tegra_sysedp_corecap tn8_sysedp_corecap[] = {
	{  4000, {  1000, 180000, 924000 }, { 1000, 180000, 924000 } },
	{  5000, {  2000, 180000, 924000 }, { 2000, 180000, 924000 } },
	{  6000, {  3000, 180000, 924000 }, { 2000, 252000, 924000 } },
	{  7000, {  4000, 180000, 924000 }, { 2000, 324000, 924000 } },
	{  8000, {  5000, 180000, 924000 }, { 2000, 396000, 924000 } },
	{  9000, {  6000, 180000, 924000 }, { 2000, 468000, 924000 } },
	{ 10000, {  6600, 180000, 924000 }, { 2000, 540000, 924000 } },
	{ 11000, {  7300, 180000, 924000 }, { 2000, 540000, 924000 } },
	{ 12000, {  8000, 180000, 924000 }, { 2000, 612000, 924000 } },
	{ 13000, {  8000, 252000, 924000 }, { 2000, 648000, 924000 } },
	{ 14000, {  8000, 324000, 924000 }, { 2000, 684000, 924000 } },
	{ 15000, {  9000, 324000, 924000 }, { 2000, 708000, 924000 } },
	{ 16000, {  9000, 396000, 924000 }, { 2000, 756000, 924000 } },
	{ 17000, { 10000, 468000, 924000 }, { 3000, 756000, 924000 } },
	{ 18000, { 11000, 540000, 924000 }, { 4000, 756000, 924000 } },
	{ 19000, { 12000, 540000, 924000 }, { 5000, 756000, 924000 } },
	{ 20000, { 13000, 612000, 924000 }, { 5000, 756000, 924000 } },
	{ 21000, { 14000, 648000, 924000 }, { 6000, 804000, 924000 } },
	{ 22000, { 15000, 708000, 924000 }, { 6500, 804000, 924000 } },
	{ 23000, { 16000, 708000, 924000 }, { 7000, 804000, 924000 } },
};

static struct tegra_sysedp_platform_data tn8_sysedp_dynamic_capping_platdata = {
	.corecap = tn8_sysedp_corecap,
	.corecap_size = ARRAY_SIZE(tn8_sysedp_corecap),
	.core_gain = 130,
	.init_req_watts = 20000,
};

static struct platform_device tn8_sysedp_dynamic_capping = {
	.name = "sysedp_dynamic_capping",
	.id = -1,
	.dev = { .platform_data = &tn8_sysedp_dynamic_capping_platdata }
};

void __init tn8_sysedp_dynamic_capping_init(void)
{
	int r;

	tn8_sysedp_dynamic_capping_platdata.cpufreq_lim = tegra_get_system_edp_entries(
		&tn8_sysedp_dynamic_capping_platdata.cpufreq_lim_size);
	if (!tn8_sysedp_dynamic_capping_platdata.cpufreq_lim) {
		WARN_ON(1);
		return;
	}

	r = platform_device_register(&tn8_sysedp_dynamic_capping);
	WARN_ON(r);
}
