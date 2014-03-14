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

#ifdef CONFIG_SENSORS_TEGRA_TSENSOR
void __init tegra3_tsensor_init(struct tegra_tsensor_pmu_data *data);
#else
static inline void tegra3_tsensor_init(struct tegra_tsensor_pmu_data *data)
{}
#endif

#endif
