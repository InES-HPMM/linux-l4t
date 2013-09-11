/*
 * arch/arm/mach-tegra/board-dt-grenada.c
 *
 * NVIDIA Tegra210 device tree board support
 *
 * Copyright (C) 2013 NVIDIA Corporation. All rights reserved.
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


#include <asm/mach/arch.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <linux/nvmap.h>

#include "board.h"
#include "clock.h"
#include "common.h"

static void __init tegra_grenada_reserve(void)
{
        tegra_reserve(SZ_128M, SZ_16M + SZ_2M, SZ_4M);
}

static void __init tegra210_dt_init(void)
{
	of_platform_populate(NULL, NULL, NULL, NULL);
}

static const char * const tegra210_dt_board_compat[] = {
	"nvidia,tegra210",
	NULL
};

DT_MACHINE_START(TEGRA210_DT, "grenada")
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_grenada_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= tegra_dt_init_irq,
	.timer		= &tegra_sys_timer,
	.init_machine	= tegra210_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tegra210_dt_board_compat,
MACHINE_END

void tegra_pd_in_idle(bool enable) {}

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
