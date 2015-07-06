/*
 * arch/arm/mach-tegra/board-p2360.c
 * Based on arch/arm/mach-tegra/board-p1859.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/gpio.h>

#include <asm/mach/arch.h>
#include <linux/platform/tegra/isomgr.h>
#include <mach/board_id.h>

#include "board.h"
#include <linux/platform/tegra/clock.h>
#include "vcm30_t124.h"
#include "board-p2360.h"
#include "devices.h"
#include "board-common.h"
#include <linux/platform/tegra/common.h>
#include "board-panel.h"
#include "tegra-of-dev-auxdata.h"

static int is_p2360_a00;

/* Display panel/HDMI */
static int p2360_dev_dummy(struct device *dev)
{
	return 0;
}

static int p2360_dummy(void)
{
	return 0;
}

struct tegra_panel_ops p2360_panel_ops = {
	.enable = p2360_dev_dummy,
	.disable = p2360_dev_dummy,
	.postsuspend = p2360_dummy,
	.hotplug_init = p2360_dev_dummy,
};

static int tegra_p2360_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	struct device *dev = data;
#endif

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		if (dev->of_node) {
			if (of_device_is_compatible(dev->of_node,
				"pwm-backlight")) {
				tegra_pwm_bl_ops_register(dev);
			}
		}
#endif
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = tegra_p2360_notifier_call,
};

static void p2360_panel_init(void)
{
	tegra_set_fixed_panel_ops(true, &p2360_panel_ops, "lvds,display");
	tegra_set_fixed_panel_ops(false, &p2360_panel_ops, "hdmi,display");
	bus_register_notifier(&platform_bus_type, &platform_nb);
}

static struct platform_device *p2360_devices[] __initdata = {
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
};

static __initdata struct tegra_clk_init_table p2360_fixed_rate_clk_table[] = {
	{ "gk20a.gbus",	NULL,	600000000,	false},
	{ NULL,		NULL,	0,		0},
};

static void __init tegra_p2360_early_init(void)
{
	struct clk *c;

	/* Early init for vcm30t124 MCM */
	tegra_vcm30_t124_early_init();

	/* Board specific clock POR */
	c = tegra_get_clock_by_name("gbus");
	tegra_init_min_rate(c, 600000000);
	tegra_init_max_rate(c, 600000000);

	c = tegra_get_clock_by_name("cpu_g");
	tegra_init_min_rate(c, 1524000000);
	tegra_init_max_rate(c, 1524000000);

	tegra_vcm30_t124_set_fixed_rate(p2360_fixed_rate_clk_table);

	tegra_clk_verify_parents();

	tegra_soc_device_init("p2360");
}

static void __init tegra_p2360_late_init(void)
{
	/* Create procfs entries for board_serial, skuinfo etc */
	tegra_init_board_info();

	/* Initialize the vcm30t124 specific devices */
	tegra_vcm30_t124_therm_mon_init();
	tegra_vcm30_t124_soctherm_init();
	tegra_vcm30_t124_usb_init();

	platform_add_devices(p2360_devices, ARRAY_SIZE(p2360_devices));

	tegra_vcm30_t124_suspend_init();

	isomgr_init();
	p2360_panel_init();
}

static void __init tegra_p2360_dt_init(void)
{
	is_p2360_a00 = tegra_is_board(NULL, "62360", NULL, "000", NULL);

	tegra_p2360_early_init();

#ifdef CONFIG_USE_OF
	tegra_vcm30_t124_populate_auxdata();
#endif

	tegra_p2360_late_init();
}

static const char * const p2360_dt_board_compat[] = {
	"nvidia,p2360",
	NULL
};

DT_MACHINE_START(P2360, "p2360")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_vcm30_t124_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_p2360_dt_init,
	.dt_compat	= p2360_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END
