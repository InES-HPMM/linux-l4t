/*
 * battery-ini-model-data.h: Battery INI model data for different platforms.
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BATTERY_INI_MODEL_DATA_H
#define _MACH_TEGRA_BATTERY_INI_MODEL_DATA_H

#include <linux/max17048_battery.h>

/*
 * Battery model data for YOKU 4100mA for MAX17048 for Macallan.
 * INI Files: 1283683
 */
static struct max17048_battery_model macallan_yoku_4100mA_max17048_battery = {
	.rcomp		= 57,
	.soccheck_A	= 119,
	.soccheck_B	= 121,
	.bits		= 18,
	.alert_threshold = 0x00,
	.one_percent_alerts = 0x40,
	.alert_on_reset = 0x40,
	.rcomp_seg	= 0x0200,
	.hibernate	= 0x3080,
	.vreset		= 0x3c96,
	.valert		= 0x00ff,
	.ocvtest	= 55952,
	.data_tbl = {
		0x98, 0x80, 0xB3, 0x50, 0xB7, 0x90, 0xB9, 0x00,
		0xBA, 0x70, 0xBC, 0x10, 0xBC, 0x50, 0xBC, 0xA0,
		0xBD, 0x20, 0xBE, 0x30, 0xBF, 0x40, 0xC2, 0xF0,
		0xC4, 0x20, 0xC7, 0xE0, 0xCB, 0xF0, 0xD0, 0x90,
		0x00, 0x40, 0x06, 0x70, 0x0E, 0x50, 0x12, 0x00,
		0x18, 0xD0, 0x33, 0x10, 0x31, 0x40, 0x35, 0xD0,
		0x18, 0xD0, 0x19, 0x00, 0x0B, 0xF0, 0x0C, 0x10,
		0x0D, 0x10, 0x07, 0x90, 0x08, 0x00, 0x08, 0x00,
	},
};

#endif
