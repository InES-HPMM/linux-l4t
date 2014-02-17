/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
 */

#ifndef _LINUX_TEGRA_BPMP_H
#define _LINUX_TEGRA_BPMP_H

#include <linux/kernel.h>

#define TEGRA_PM_C0	0
#define TEGRA_PM_C1	1
#define TEGRA_PM_C2	2
#define TEGRA_PM_C3	3
#define TEGRA_PM_C4	4
#define TEGRA_PM_C5	5
#define TEGRA_PM_C6	6
#define TEGRA_PM_C7	7
#define TEGRA_PM_CC0	8
#define TEGRA_PM_CC1	9
#define TEGRA_PM_CC2	10
#define TEGRA_PM_CC3	11
#define TEGRA_PM_CC4	12
#define TEGRA_PM_CC5	13
#define TEGRA_PM_CC6	14
#define TEGRA_PM_CC7	15
#define TEGRA_PM_SC0	16
#define TEGRA_PM_SC1	17
#define TEGRA_PM_SC2	18
#define TEGRA_PM_SC3	19
#define TEGRA_PM_SC4	20
#define TEGRA_PM_SC5	21
#define TEGRA_PM_SC6	22
#define TEGRA_PM_SC7	23

struct tegra_bpmp_platform_data {
	phys_addr_t phys_start;
	phys_addr_t size;
};

int tegra_bpmp_pm_target(int cpu, int tolerance);

#endif
