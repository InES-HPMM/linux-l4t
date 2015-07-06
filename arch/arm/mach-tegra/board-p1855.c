/*
 * arch/arm/mach-tegra/board-p1855.c
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>
#include <linux/platform/tegra/isomgr.h>
#include <mach/board_id.h>

#include "iomap.h"
#include "board.h"
#include <linux/platform/tegra/clock.h>
#include "vcm30_t124.h"
#include "board-p1855.h"
#include "devices.h"
#include "board-common.h"
#include <linux/platform/tegra/common.h>
#include "board-panel.h"
#include "tegra-of-dev-auxdata.h"
#include "linux/irqchip/tegra.h"

#define EXTERNAL_PMU_INT_N_WAKE         18



static struct platform_device *p1855_devices[] __initdata = {
	&tegra_rtc_device,
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
};

static void __init tegra_p1855_early_init(void)
{
	/* Early init for vcm30t124 MCM */
	tegra_vcm30_t124_early_init();

	/* Board specific clock POR */
	/*BLANK*/

	tegra_clk_verify_parents();

	tegra_soc_device_init("p1855");
}

static void __init tegra_p1855_late_init(void)
{
	/* Create procfs entries for board_serial, skuinfo etc */
	tegra_init_board_info();

	/* Initialize the vcm30t124 specific devices */
	tegra_vcm30_t124_therm_mon_init();
	tegra_vcm30_t124_soctherm_init();
	tegra_vcm30_t124_usb_init();


	platform_add_devices(p1855_devices, ARRAY_SIZE(p1855_devices));

	tegra_vcm30_t124_suspend_init();

	isomgr_init();

	/* Enable PMC wake source PWR_INT_N (Jetson TK1 Pro, Switch SW2) */
	tegra_pm_irq_set_wake(EXTERNAL_PMU_INT_N_WAKE, true);
}

static void __init tegra_p1855_dt_init(void)
{
	tegra_p1855_early_init();

#ifdef CONFIG_USE_OF
	tegra_vcm30_t124_populate_auxdata();
#endif

	tegra_p1855_late_init();
}

static const char * const p1855_dt_board_compat[] = {
	"nvidia,p1855",
	NULL
};

DT_MACHINE_START(P1855, "p1855")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_vcm30_t124_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_p1855_dt_init,
	.dt_compat	= p1855_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END
