/*
 * arch/arm/mach-tegra/board-dt-exuma.c
 *
 * NVIDIA Tegra132 device tree board support
 *
 * Copyright (C) 2012-2013 NVIDIA Corporation. All rights reserved.
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

#include "../../arm/mach-tegra/board-bonaire.c"

static void __init tegra132_dt_init(void)
{
	tegra_bonaire_init();
	of_platform_populate(NULL, NULL, NULL, NULL);
}

static const char * const tegra132_dt_board_compat[] = {
	"nvidia,tegra132",
	NULL
};

DT_MACHINE_START(TEGRA132_DT, "exuma")
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_bonaire_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= tegra_dt_init_irq,
	.timer		= &tegra_sys_timer,
	.init_machine	= tegra132_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tegra132_dt_board_compat,
MACHINE_END

/*******************************************************************
 * FIXME: fake missing symbols from arm64 until ported by ARM/NV.  *
 *******************************************************************/
/*
 * Defined in arch/arm/kernel/suspend.c
 */
int cpu_suspend(unsigned long arg, int (*fn)(unsigned long))
{
	return 0;
}
void cpu_resume(void)
{
}
