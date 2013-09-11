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

struct of_dev_auxdata t210_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra210-host1x", TEGRA_HOST1X_BASE, "host1x",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvenc", TEGRA_NVENC_BASE, "msenc",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vi", TEGRA_VI_BASE, "vi",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-isp", TEGRA_ISP_BASE, "isp.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-isp", TEGRA_ISPB_BASE, "isp.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-tsec", TEGRA_TSEC_BASE, "tsec",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vic", TEGRA_VIC_BASE, "vic03",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvdec", TEGRA_NVDEC_BASE, "nvdec",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvjpg", TEGRA_NVJPG_BASE, "nvjpg",
				NULL),
	{}
};

static void __init tegra210_dt_init(void)
{
	of_platform_populate(NULL,
			of_default_bus_match_table,
			t210_auxdata_lookup,
			&platform_bus);
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
