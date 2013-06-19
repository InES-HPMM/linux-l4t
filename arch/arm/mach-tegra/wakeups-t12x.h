/*
 * arch/arm/mach-tegra/wakeups-t12x.h
 *
 * Declarations of Tegra 12x LP0 wakeup sources
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#ifndef __MACH_TEGRA_WAKEUPS_T12X_H
#define __MACH_TEGRA_WAKEUPS_T12X_H

#ifndef CONFIG_ARCH_TEGRA_12x_SOC
#error "Tegra 12x wakeup sources valid only for CONFIG_ARCH_TEGRA_12x_SOC"
#endif

#define TEGRA_WAKE_GPIO_PO5	0
#define TEGRA_WAKE_GPIO_PV1	1
#define TEGRA_WAKE_GPIO_PN7	4
#define TEGRA_WAKE_GPIO_PU5	6
#define TEGRA_WAKE_GPIO_PU6	7
#define TEGRA_WAKE_GPIO_PC7	8
#define TEGRA_WAKE_GPIO_PW3	11
#define TEGRA_WAKE_GPIO_PW2	12
#define TEGRA_WAKE_GPIO_PJ2	15
#define TEGRA_WAKE_GPIO_PI5	23
#define TEGRA_WAKE_GPIO_PV0	24
#define TEGRA_WAKE_GPIO_PS4	25
#define TEGRA_WAKE_GPIO_PS5	26
#define TEGRA_WAKE_GPIO_PS0	27
#define TEGRA_WAKE_GPIO_PS6	28
#define TEGRA_WAKE_GPIO_PJ0	33
#define TEGRA_WAKE_GPIO_PK2	34
#define TEGRA_WAKE_GPIO_PI6	35
#define TEGRA_WAKE_GPIO_PBB6	45
#define TEGRA_WAKE_GPIO_PR7	49
#define TEGRA_WAKE_GPIO_PR4	50
#define TEGRA_WAKE_GPIO_PQ0	51
#define TEGRA_WAKE_GPIO_PQ5	54
#define TEGRA_WAKE_GPIO_PK6	57
#define TEGRA_WAKE_GPIO_PFF2	59

#endif
