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
static unsigned int modem_states[] = { 0, 4100 };
static unsigned int pwm_backlight_states[] = {
	0, 125, 250, 375, 500, 625, 750, 875, 1000, 1125, 1250
};

/* (optional) 10" panel */
static unsigned int pwm_backlight_10_states[] = {
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
	SYSEDP_CONSUMER_DATA("modem", modem_states),
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
	struct board_info board;

	tegra_get_display_board_info(&board);

	/* Some TN8 boards use non-default display */
	if (board.board_id != BOARD_E1549)
		memcpy(pwm_backlight_states, pwm_backlight_10_states,
		       sizeof(pwm_backlight_states));

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

#if 0
/* TODO: use this table when battery temp sensing is fixed */
/*                           60C    40C    25C    0C      -20C      */
static int rbat_data[] = {  90000, 60000, 70000,  90000, 110000,   /* 100% */
			    90000, 60000, 70000,  90000, 110000,   /*  13% */
			   110000, 80000, 90000, 110000, 130000 }; /*   0% */
static int rbat_temp_axis[] = { 60, 40, 25, 0, -20 };
static int rbat_capacity_axis[] = { 100, 13, 0 };
#else
static int rbat_data[] = {  70000,   /* 100% */
			    70000,   /*  25% */
			   110000,   /*  10% */
			   130000 }; /*   0% */
static int rbat_temp_axis[] = { 25 };
static int rbat_capacity_axis[] = { 100, 25, 10, 0 };
#endif
struct sysedp_batmon_rbat_lut tn8_rbat_lut = {
	.temp_axis = rbat_temp_axis,
	.temp_size = ARRAY_SIZE(rbat_temp_axis),
	.capacity_axis = rbat_capacity_axis,
	.capacity_size = ARRAY_SIZE(rbat_capacity_axis),
	.data = rbat_data,
	.data_size = ARRAY_SIZE(rbat_data),
};

static struct sysedp_batmon_calc_platform_data tn8_batmon_pdata = {
	.power_supply = "battery",
	.r_const = 60000,
	.vsys_min = 3000000,
	.ibat_lut = tn8_ibat_lut,
	.rbat_lut = &tn8_rbat_lut,
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

static struct tegra_sysedp_platform_data tn8_sysedp_dynamic_capping_platdata = {
	.corecap = td570d_sysedp_corecap,
	.corecap_size = td570d_sysedp_corecap_sz,
	.core_gain = 100,
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
