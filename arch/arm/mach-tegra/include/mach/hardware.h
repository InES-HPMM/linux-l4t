/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2011-2013 NVIDIA CORPORATION. All rights reserved.
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

#if defined(CONFIG_ARCH_TEGRA_2x_SOC)
#define pcibios_assign_all_busses()		1

#else

#define pcibios_assign_all_busses()		0
#endif

enum tegra_chipid {
	TEGRA_CHIPID_UNKNOWN = 0,
	TEGRA_CHIPID_TEGRA14 = 0x14,
	TEGRA_CHIPID_TEGRA2 = 0x20,
	TEGRA_CHIPID_TEGRA3 = 0x30,
	TEGRA_CHIPID_TEGRA11 = 0x35,
	TEGRA_CHIPID_TEGRA12 = 0x40,
};

enum tegra_revision {
	TEGRA_REVISION_UNKNOWN = 0,
	TEGRA_REVISION_A01,
	TEGRA_REVISION_A02,
	TEGRA_REVISION_A03,
	TEGRA_REVISION_A03p,
	TEGRA_REVISION_A04,
	TEGRA_REVISION_A04p,
	TEGRA_REVISION_QT,
	TEGRA_REVISION_SIM,
	TEGRA_REVISION_MAX,
};

enum tegra_platform {
	TEGRA_PLATFORM_SILICON = 0,
	TEGRA_PLATFORM_QT,
	TEGRA_PLATFORM_LINSIM,
	TEGRA_PLATFORM_FPGA,
	TEGRA_PLATFORM_MAX,
};

extern enum tegra_revision tegra_revision;
enum tegra_chipid tegra_get_chipid(void);
unsigned int tegra_get_minor_rev(void);

#ifdef CONFIG_TEGRA_SIMULATION_SPLIT_MEM
int tegra_split_mem_active(void);
#endif

#ifdef CONFIG_TEGRA_PRE_SILICON_SUPPORT
void tegra_get_netlist_revision(u32 *netlist, u32* patchid);
bool tegra_cpu_is_asim(void);
bool tegra_cpu_is_dsim(void);
enum tegra_platform tegra_get_platform(void);
static inline bool tegra_platform_is_silicon(void)
{
	return tegra_get_platform() == TEGRA_PLATFORM_SILICON;
}
static inline bool tegra_platform_is_qt(void)
{
	return tegra_get_platform() == TEGRA_PLATFORM_QT;
}
static inline bool tegra_platform_is_linsim(void)
{
	return tegra_get_platform() == TEGRA_PLATFORM_LINSIM;
}
static inline bool tegra_platform_is_fpga(void)
{
	return tegra_get_platform() == TEGRA_PLATFORM_FPGA;
}
#else
static inline void tegra_get_netlist_revision(u32 *netlist, u32* patchid)
{
	BUG();
}
static inline bool tegra_cpu_is_asim(void) { return false; }
static inline bool tegra_cpu_is_dsim(void) { return false; }
static inline bool tegra_platform_is_silicon(void) { return true; }
static inline bool tegra_platform_is_fpga(void) { return false; }
static inline bool tegra_platform_is_qt(void) { return false; }
static inline bool tegra_platform_is_linsim(void) { return false; }
#endif

#endif
