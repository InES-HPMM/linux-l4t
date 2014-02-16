/*
 * arch/arm/mach-tegra/board-tn8-p1761-powermon.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/ina3221.h>
#include <linux/platform_data/ina230.h>

#include "board.h"
#include "board-ardbeg.h"
#include "tegra-board-id.h"

#define PRECISION_MULTIPLIER_TN8	1000
#define VDD_SOC_SD1_REWORKED		10
#define VDD_CPU_BUCKCPU_REWORKED	10
#define VDD_1V35_SD2_REWORKED		10

#define AVG_SAMPLES (2 << 9) /* 16 samples */

/* AVG is specified from platform data */
#define INA230_CONT_CONFIG	(AVG_SAMPLES | INA230_VBUS_CT | \
				INA230_VSH_CT | INA230_CONT_MODE)
#define INA230_TRIG_CONFIG	(AVG_SAMPLES | INA230_VBUS_CT | \
				INA230_VSH_CT | INA230_TRIG_MODE)

/* rails on TN8_P1761 i2c */
enum {
	VDD_BAT_CPU_GPU,
};
/* rails on TN8_P1761-A02 i2c */
enum {
	VDD_BAT_USB_MDM,
};
enum {
	INA_I2C_ADDR_40,
};

/* Configure 140us conversion time, 512 averages */
#define P1761_CONT_CONFIG_DATA 0x7c07

static struct ina3221_platform_data tn8_p1761_power_mon_info[] = {
	[VDD_BAT_CPU_GPU] = {
		.rail_name = {"VDD_BAT", "VDD_CPU", "VDD_GPU"},
		.shunt_resistor = {1, 1, 1},
		.cont_conf_data = P1761_CONT_CONFIG_DATA,
		.trig_conf_data = INA3221_TRIG_CONFIG_DATA,
		.warn_conf_limits = {8000, -1, -1},
		.crit_conf_limits = {9000, -1, -1},
	},
};

static struct i2c_board_info tn8_p1761_i2c_ina3221_info[] = {
	[INA_I2C_ADDR_40] = {
		I2C_BOARD_INFO("ina3221", 0x40),
		.platform_data = &tn8_p1761_power_mon_info[VDD_BAT_CPU_GPU],
		.irq = -1,
	},
};

static struct ina3221_platform_data tn8_p1761_a02_power_mon_info[] = {
	[VDD_BAT_USB_MDM] = {
		.rail_name = {"VDD_BAT", "VDD_USB_5V0", "VDD_SYS_MDM"},
		.shunt_resistor = {1, 10, 1},
		.cont_conf_data = P1761_CONT_CONFIG_DATA,
		.trig_conf_data = INA3221_TRIG_CONFIG_DATA,
		.warn_conf_limits = {8000, -1, -1},
		.crit_conf_limits = {9000, -1, -1},
	},
};

static struct i2c_board_info tn8_p1761_a02_i2c_ina3221_info[] = {
	[INA_I2C_ADDR_40] = {
		I2C_BOARD_INFO("ina3221", 0x40),
		.platform_data = &tn8_p1761_a02_power_mon_info[VDD_BAT_USB_MDM],
		.irq = -1,
	},
};

int __init tn8_p1761_pmon_init(void)
{
	struct board_info bi;
	int ret = 0;

	tegra_get_board_info(&bi);

	/* power mon for E1784 is in DT */
	if (bi.board_id == BOARD_E1784)
		return 0;

	if (bi.fab >= BOARD_FAB_A02)
		ret = i2c_register_board_info(1, tn8_p1761_a02_i2c_ina3221_info,
					      ARRAY_SIZE(tn8_p1761_a02_i2c_ina3221_info));
	else
		ret = i2c_register_board_info(1, tn8_p1761_i2c_ina3221_info,
					      ARRAY_SIZE(tn8_p1761_i2c_ina3221_info));

	return ret;
}
