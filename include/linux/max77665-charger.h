/*
 * MAX77665 Haptic Driver
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 * Author: Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_INPUT_MAX77665_CHARGER_H
#define _LINUX_INPUT_MAX77665_CHARGER_H

#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/extcon.h>
#include <linux/mfd/max77665.h>

#define MAX77665_CHG_INT        0xb0
#define MAX77665_CHG_INT_MASK   0xb1
#define BAT_I			(1 << 3)
#define CHG_I			(1 << 4)
#define MAX77665_CHG_INT_OK     0xb2
#define BYP_OK			(1 << 0)
#define DETBAT_OK		(1 << 2)
#define BAT_OK			(1 << 3)
#define CHG_OK			(1 << 4)
#define CHGIN_OK		(1 << 6)
#define MAX77665_CHG_DTLS_00    0xb3
#define MAX77665_CHG_DTLS_01    0xb4
#define MAX77665_CHG_DTLS_02    0xb5
#define MAX77665_CHG_DTLS_03    0xb6
#define MAX77665_CHG_CNFG_00    0xb7
#define CHARGER_ON_OTG_OFF_BUCK_OFF_BOOST_ON 0x05
#define CHARGER_OFF_OTG_ON_BUCK_OFF_BOOST_ON 0x2A
#define WDTEN	(1 << 4)
#define MAX77665_CHG_CNFG_01    0xb8
#define MAX77665_CHG_CNFG_02    0xb9
#define MAX77665_CHG_CNFG_03    0xba
#define MAX77665_CHG_CNFG_04    0xbb
#define MAX77665_CHG_CNFG_05    0xbc
#define MAX77665_CHG_CNFG_06    0xbd
#define WDTCLR	(1 << 0)
#define MAX77665_CHG_CNFG_07    0xbe
#define MAX77665_CHG_CNFG_08    0xbf
#define MAX77665_CHG_CNFG_09    0xc0
#define MAX77665_CHG_CNFG_10    0xc1
#define MAX77665_CHG_CNFG_11    0xc2
#define MAX77665_CHG_CNFG_12    0xc3
#define MAX77665_CHG_CNFG_13    0xc4
#define MAX77665_CHG_CNFG_14    0xc5
#define MAX77665_SAFEOUTCTRL    0xc6
#define MAX77665_WATCHDOG_TIMER_PERIOD_S 80

#define MAX_CABLES	6

enum max77665_mode {
	CHARGER,
	OTG,
};

struct max77665_muic_platform_data {
	const char *ext_conn_name;
};

struct max77665_charger_cable {
	const char *extcon_name;
	const char *name;
	struct notifier_block nb;
	struct max77665_charger *charger;
	struct extcon_specific_cable_nb *extcon_dev;
	struct delayed_work extcon_notifier_work;
	long int event;
};

struct max77665_charger_plat_data {
	uint32_t fast_chg_cc; /* fast charger current*/
	uint32_t term_volt; /* charger termination voltage */
	uint32_t curr_lim; /* input current limit */
	uint8_t num_cables;
	struct max77665_charger_cable *cables;
	char *extcon_name;
	void (*update_status)(int);
};
#endif
