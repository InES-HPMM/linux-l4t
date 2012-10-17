/*
 * arch/arm/mach-tegra/board-cardhu-powermon.c
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/ina219.h>
#include <linux/i2c/pca954x.h>

#include "board.h"
#include "board-pluto.h"

enum {
	VDD_AC_BAT,
	VDD_DRAM_IN,
	VDD_BACKLIGHT_IN,
	VDD_CPU_IN,
	VDD_CORE_IN,
	VDD_DISPLAY_IN,
	VDD_3V3_TEGRA,
	VDD_OTHER_PMU_IN,
	VDD_1V8_TEGRA,
	VDD_1V8_OTHER,
	UNUSED_RAIL,
};

static struct ina219_platform_data power_mon_info[] = {
	[VDD_AC_BAT] = {
		.calibration_data  = 0xa000,
		.power_lsb = 2,
		.rail_name = "VDD_AC_BAT",
		.divisor = 20,
	},

	[VDD_DRAM_IN] = {
		.calibration_data  = 0xa000,
		.power_lsb = 2,
		.rail_name = "VDD_DRAM_IN",
		.divisor = 20,
	},

	[VDD_BACKLIGHT_IN] = {
		.calibration_data  = 0x6aaa,
		.power_lsb = 1,
		.rail_name = "VDD_BACKLIGHT_IN",
		.divisor = 20,
	},

	[VDD_CPU_IN] = {
		.calibration_data  = 0xa000,
		.power_lsb = 1,
		.rail_name = "VDD_CPU_IN",
		.divisor = 20,
	},

	[VDD_CORE_IN] = {
		.calibration_data  = 0x6aaa,
		.power_lsb = 1,
		.rail_name = "VDD_CORE_IN",
		.divisor = 20,
	},

	[VDD_DISPLAY_IN] = {
		.calibration_data  = 0x4000,
		.power_lsb = 1,
		.rail_name = "VDD_DISPLAY_IN",
		.divisor = 20,
	},

	[VDD_3V3_TEGRA] = {
		.calibration_data  = 0x6aaa,
		.power_lsb = 1,
		.rail_name = "VDD_3V3_TEGRA",
		.divisor = 20,
	},

	[VDD_OTHER_PMU_IN] = {
		.calibration_data  = 0xa000,
		.power_lsb = 1,
		.rail_name = "VDD_OTHER_PMU_IN",
		.divisor = 20,
	},

	[VDD_1V8_TEGRA] = {
		.calibration_data  = 0x4000,
		.power_lsb = 1,
		.rail_name = "VDD_1V8_TEGRA",
		.divisor = 20,
	},

	[VDD_1V8_OTHER] = {
		.calibration_data  = 0xa000,
		.power_lsb = 1,
		.rail_name = "VDD_1V8_OTHER",
		.divisor = 20,
	},

	/* All unused INA219 devices use below data*/
	[UNUSED_RAIL] = {
		.calibration_data  = 0x4000,
		.power_lsb = 1,
		.rail_name = "unused_rail",
		.divisor = 20,
	},
};

enum {
	INA_I2C_ADDR_40,
	INA_I2C_ADDR_41,
	INA_I2C_ADDR_42,
	INA_I2C_ADDR_43,
	INA_I2C_ADDR_44,
	INA_I2C_ADDR_45,
	INA_I2C_ADDR_46,
	INA_I2C_ADDR_47,
	INA_I2C_ADDR_48,
	INA_I2C_ADDR_49,
	INA_I2C_ADDR_4B,
	INA_I2C_ADDR_4C,
	INA_I2C_ADDR_4E,
	INA_I2C_ADDR_4F,
};

enum {
	HPA_I2C_ADDR_40,
	HPA_I2C_ADDR_41,
	HPA_I2C_ADDR_42,
	HPA_I2C_ADDR_43,
};

static struct i2c_board_info pluto_i2c0_ina219_board_info[] = {
	[INA_I2C_ADDR_40] = {
		I2C_BOARD_INFO("ina219", 0x40),
		.platform_data = &power_mon_info[VDD_AC_BAT],
		.irq = -1,
	},

	[INA_I2C_ADDR_41] = {
		I2C_BOARD_INFO("ina219", 0x41),
		.platform_data = &power_mon_info[VDD_DRAM_IN],
		.irq = -1,
	},

	[INA_I2C_ADDR_42] = {
		I2C_BOARD_INFO("ina219", 0x42),
		.platform_data = &power_mon_info[VDD_BACKLIGHT_IN],
		.irq = -1,
	},

	[INA_I2C_ADDR_43] = {
		I2C_BOARD_INFO("ina219", 0x43),
		.platform_data = &power_mon_info[VDD_CPU_IN],
		.irq = -1,
	},

	[INA_I2C_ADDR_44] = {
		I2C_BOARD_INFO("ina219", 0x44),
		.platform_data = &power_mon_info[VDD_CORE_IN],
		.irq = -1,
	},

	[INA_I2C_ADDR_45] = {
		I2C_BOARD_INFO("ina219", 0x45),
		.platform_data = &power_mon_info[VDD_DISPLAY_IN],
		.irq = -1,
	},

	[INA_I2C_ADDR_46] = {
		I2C_BOARD_INFO("ina219", 0x46),
		.platform_data = &power_mon_info[VDD_3V3_TEGRA],
		.irq = -1,
	},

	[INA_I2C_ADDR_47] = {
		I2C_BOARD_INFO("ina219", 0x47),
		.platform_data = &power_mon_info[VDD_OTHER_PMU_IN],
		.irq = -1,
	},

	[INA_I2C_ADDR_48] = {
		I2C_BOARD_INFO("ina219", 0x48),
		.platform_data = &power_mon_info[VDD_1V8_TEGRA],
		.irq = -1,
	},

	[INA_I2C_ADDR_49] = {
		I2C_BOARD_INFO("ina219", 0x49),
		.platform_data = &power_mon_info[VDD_1V8_OTHER],
		.irq = -1,
	},


	[INA_I2C_ADDR_4B] = {
		I2C_BOARD_INFO("ina219", 0x4B),
		.platform_data = &power_mon_info[UNUSED_RAIL],
		.irq = -1,
	},

	[INA_I2C_ADDR_4C] = {
		I2C_BOARD_INFO("ina219", 0x4C),
		.platform_data = &power_mon_info[UNUSED_RAIL],
		.irq = -1,
	},

	[INA_I2C_ADDR_4E] = {
		I2C_BOARD_INFO("ina219", 0x4E),
		.platform_data = &power_mon_info[UNUSED_RAIL],
		.irq = -1,
	},

	[INA_I2C_ADDR_4F] = {
		I2C_BOARD_INFO("ina219", 0x4F),
		.platform_data = &power_mon_info[UNUSED_RAIL],
		.irq = -1,
	},
};

static struct i2c_board_info pluto_i2c0_ina2191_board_info[] = {
	[INA_I2C_ADDR_40] = {
		I2C_BOARD_INFO("ina219", 0x49),
		.platform_data = &power_mon_info[VDD_AC_BAT],
		.irq = -1,
	},

	[INA_I2C_ADDR_41] = {
		I2C_BOARD_INFO("ina219", 0x4C),
		.platform_data = &power_mon_info[VDD_DRAM_IN],
		.irq = -1,
	},
};

static struct i2c_board_info pluto_i2c0_hpa219_board_info[] = {
	[HPA_I2C_ADDR_40] = {
		I2C_BOARD_INFO("ina219", 0x40),
		.platform_data = &power_mon_info[VDD_AC_BAT],
		.irq = -1,
	},

	[HPA_I2C_ADDR_41] = {
		I2C_BOARD_INFO("ina219", 0x41),
		.platform_data = &power_mon_info[VDD_DRAM_IN],
		.irq = -1,
	},

	[HPA_I2C_ADDR_42] = {
		I2C_BOARD_INFO("ina219", 0x42),
		.platform_data = &power_mon_info[VDD_BACKLIGHT_IN],
		.irq = -1,
	},

	[HPA_I2C_ADDR_43] = {
		I2C_BOARD_INFO("ina219", 0x43),
		.platform_data = &power_mon_info[VDD_CPU_IN],
		.irq = -1,
	},
};

static struct pca954x_platform_mode pluto_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data pluto_pca954x_data = {
	.modes    = pluto_pca954x_modes,
	.num_modes      = ARRAY_SIZE(pluto_pca954x_modes),
};

static const struct i2c_board_info pluto_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x71),
		.platform_data = &pluto_pca954x_data,
	},
};

int __init pluto_pmon_init(void)
{
	i2c_register_board_info(1, pluto_i2c2_board_info,
		ARRAY_SIZE(pluto_i2c2_board_info));

	i2c_register_board_info(PCA954x_I2C_BUS1, pluto_i2c0_ina219_board_info,
			ARRAY_SIZE(pluto_i2c0_ina219_board_info));

	i2c_register_board_info(PCA954x_I2C_BUS0, pluto_i2c0_hpa219_board_info,
			ARRAY_SIZE(pluto_i2c0_hpa219_board_info));

	i2c_register_board_info(PCA954x_I2C_BUS2, pluto_i2c0_ina2191_board_info,
			ARRAY_SIZE(pluto_i2c0_ina2191_board_info));
	return 0;
}

