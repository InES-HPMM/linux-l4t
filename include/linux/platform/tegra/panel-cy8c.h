/*
 * Copyright (c) 2015, NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MACH_TEGRA_PANEL_CY8C_H__
#define __MACH_TEGRA_PANEL_CY8C_H__

#ifdef CONFIG_TEGRA_PANEL_CY8C
int cy8c_panel_set_state(bool enable);
#else
int cy8c_panel_set_state(bool enable) { return -ENODEV; }
#endif

#endif /* __MACH_TEGRA_PANEL_CY8C_H__ */
