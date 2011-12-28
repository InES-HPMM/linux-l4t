/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corp.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
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

#ifndef MACH_TEGRA_HARDWARE_H
#define MACH_TEGRA_HARDWARE_H

enum tegra_revision {
	TEGRA_REVISION_UNKNOWN = 0,
	TEGRA_REVISION_A01,
	TEGRA_REVISION_A02,
	TEGRA_REVISION_A03,
	TEGRA_REVISION_A03p,
	TEGRA_REVISION_A04,
	TEGRA_REVISION_A04p,
	TEGRA_REVISION_MAX,
};

#define TEGRA20		0x20
#define TEGRA30		0x30
#define TEGRA11		0x35

extern int tegra_chip_id;
extern enum tegra_revision tegra_revision;

#ifndef CONFIG_TEGRA_SILICON_PLATFORM
void tegra_get_netlist_revision(u32 *netlist, u32* patchid);
#else
static inline void tegra_get_netlist_revision(u32 *netlist, u32* patchid)
{
	BUG();
}
#endif

#endif
