/*
 * arch/arm/mach-tegra/board-dt-exuma.c
 *
 * NVIDIA Tegra132 device tree board support
 *
 * Copyright (C) 2012-2014 NVIDIA Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/tegra-pmc.h>

#include "pm.h"

#include "../../arm/mach-tegra/board-ardbeg.c"

static const char * const tegra132_dt_board_compat[] = {
	"nvidia,tegra132",
	NULL
};

DT_MACHINE_START(TEGRA132_DT, "exuma")
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra12x_init_early,
	.init_late	= tegra_init_late,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tegra132_dt_board_compat,
MACHINE_END

/* Needed by board-ardbeg.c */
int __init laguna_pinmux_init(void) { return 0; }
int __init laguna_regulator_init(void) { return 0; }
int __init laguna_pm358_pmon_init(void) { return 0; }
int __init laguna_edp_init(void) { return 0; }
void __init tn8_new_sysedp_init(void) {}
void __init tn8_sysedp_dynamic_capping_init(void) {}

extern int bonaire_panel_init(void);
int ardbeg_panel_init(void)
{
	return bonaire_panel_init();
}

