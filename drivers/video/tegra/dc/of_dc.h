/*
 * drivers/video/tegra/dc/of_dc.h
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_OF_DC_H__
#define __DRIVERS_VIDEO_TEGRA_DC_OF_DC_H__

#ifdef CONFIG_OF
char *of_get_panel_name(void);
#else
char *of_get_panel_name(void) { return NULL; }
#endif

#endif
