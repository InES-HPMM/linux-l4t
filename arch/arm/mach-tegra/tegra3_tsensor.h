/*
 * arch/arm/mach-tegra/tegra3_tsensor.h
 *
 * Tegra tsensor header file
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_TEGRA3_TSENSOR_H
#define __MACH_TEGRA_TEGRA3_TSENSOR_H


/**
 * struct tegra_tsensor_pmu_data - PMIC temperature sensor
 * configuration
 * @poweroff_reg_data:	The data to write to turn the system off
 * @poweroff_reg_addr:	The PMU address of poweroff register
 * @reset_tegra:	Flag indicating whether or not the system
 *			will shutdown during a thermal trip.
 * @controller_type:	If this field is set to 0, the PMIC is
 *			connected via I2C. If it is set to 1,
 *			it is connected via SPI. If it is set to
 *			2, it is connected via GPIO.
 * @i2c_controller_id:	The i2c bus controller id
 * @pinmux:		An array index used to configure which pins
 *			on the chip are muxed to the I2C/SPI/GPIO
 *			controller that is in use. Contact NVIDIA
 *			for more information on what these index values
 *			mean for a given chip.
 * @pmu_16bit_ops:	If 0, sends three bytes from the PMC_SCRATCH54
 *			register to the PMIC to turn it off; if 1, sends
 *			four bytes from the PMC_SCRATCH54 register to the PMIC
 *			to turn it off, plus one other byte. Must be set to
 *			0 - the current code does not support 16 bit
 *			operations.
 * @pmu_i2c_addr:	The address of the PMIC on the I2C bus
 *
 * When the temperature gets too high, the PMIC will power off the device
 * based on what is written to the PMIC registers.
 *
 * @poweroff_reg_data and @poweroff_reg_addr are written to the PMC SCRATCH54
 * register.
 *
 * @reset_tegra, @controller_type, @i2c_controller_id, @pinmux, @pmu_16bit_ops
 * and @pmu_i2c_addr are written to the PMC SCRATCH55 register.
 */
struct tegra_tsensor_pmu_data {
	u8 poweroff_reg_data;
	u8 poweroff_reg_addr;
	u8 reset_tegra;
	u8 controller_type;
	u8 i2c_controller_id;
	u8 pinmux;
	u8 pmu_16bit_ops;
	u8 pmu_i2c_addr;
};

#ifdef CONFIG_SENSORS_TEGRA_TSENSOR
void __init tegra3_tsensor_init(struct tegra_tsensor_pmu_data *data);
#else
static inline void tegra3_tsensor_init(struct tegra_tsensor_pmu_data *data)
{}
#endif

#endif
