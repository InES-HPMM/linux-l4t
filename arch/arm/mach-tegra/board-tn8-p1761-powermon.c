/*
 * arch/arm/mach-tegra/board-tn8-p1761-powermon.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
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

/* rails on TN8_P1761 i2c */
enum {
	VDD_BAT_CPU_GPU,
};
enum {
	INA_I2C_ADDR_40,
};

static struct ina3221_platform_data tn8_p1761_power_mon_info[] = {
	[VDD_BAT_CPU_GPU] = {
		.rail_name = {"VDD_BAT", "VDD_CPU", "VDD_GPU"},
		.shunt_resistor = {1, 1, 1},
		.cont_conf_data = INA3221_CONT_CONFIG_DATA,
		.trig_conf_data = INA3221_TRIG_CONFIG_DATA,
		.warn_conf_limits = {-1, -1, -1},
		.crit_conf_limits = {6450, -1, -1},
	},
};

static struct i2c_board_info tn8_p1761_i2c_ina3221_info[] = {
	[INA_I2C_ADDR_40] = {
		I2C_BOARD_INFO("ina3221", 0x40),
		.platform_data = &tn8_p1761_power_mon_info[VDD_BAT_CPU_GPU],
		.irq = -1,
	},
};

int __init tn8_p1761_pmon_init(void)
{
	int ret;
	ret = i2c_register_board_info(1, tn8_p1761_i2c_ina3221_info,
				      ARRAY_SIZE(tn8_p1761_i2c_ina3221_info));
	return ret;
}
