/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/platform_data/tegra_bpmp.h>
#include "bpmp_private.h"

int bpmp_ping(void)
{
	return -ENODEV;
}

int tegra_bpmp_pm_target(int cpu, int tolerance)
{
	return min(TEGRA_PM_C7, tolerance);
}

int tegra_bpmp_pm_target_entered(void)
{
	return 0;
}
