/*
 * arch/arm/mach-tegra/board-p1889.c
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
#include "board-p1889.h"
#include "devices.h"
#include "board-common.h"
#include <linux/platform/tegra/common.h>
#include "board-panel.h"
#include "tegra-of-dev-auxdata.h"
#include "linux/irqchip/tegra.h"

#define EXTERNAL_PMU_INT_N_WAKE         18

static __initdata struct tegra_clk_init_table p1889_clk_init_table[] = {
/*
 * audio clock tables based on baseboard revision. These rates will be set
 * early on during boot up. Also, they will remain fixed throughout.
 */
/*         clock        parent          rate            enable (always-on) */
	{ "blink",	"clk_32k",		32768,	true},
	{ "i2s4_sync",	NULL,		12288000,	false},
	{ "audio4",	"i2s4_sync",	12288000,	false},
	{ "audio4_2x",	"audio4",	12288000,	false},
	{ "i2s2_sync",	NULL,		12288000,	false},
	{ "audio2",	"i2s2_sync",	12288000,	false},
	{ "audio2_2x",	"audio2",	12288000,	false},
	{ "i2s0_sync",	NULL,		12288000,	false},
	{ "audio0",	"i2s0_sync",	12288000,	false},
	{ "audio0_2x",	"audio0",	12288000,	false},
	{ "i2s0",	"audio0_2x",	12288000,	false},
	{ "i2s1",	"pll_a_out0",	3072000,	false},
	{ "i2s2",	"audio2_2x",	12288000,	false},
	{ "i2s3",	"pll_a_out0",	3072000,	false},
	{ "i2s4",	"pll_a_out0",	1024000,	false},
	{ "extern1",	"pll_a_out0",	12288000,	false},
	{ NULL,		NULL,		0,		0},
};

static int __init p1889_fixed_target_rate_init(void)
{

	struct tegra_clk_init_table *clk_table = p1889_clk_init_table;

	/* Set rate of audio clocks */
	tegra_clk_init_from_table(clk_table);

	/* Set target fixed rate of audio clocks */
	tegra_vcm30_t124_set_fixed_rate(clk_table);

	return 0;
}


/* Display panel/HDMI */
static int p1889_dev_dummy(struct device *dev)
{
	return 0;
}

static int p1889_dummy(void)
{
	return 0;
}

struct tegra_panel_ops p1889_hdmi_ops = {
	.enable = p1889_dev_dummy,
	.disable = p1889_dev_dummy,
	.postsuspend = p1889_dummy,
	.hotplug_init = p1889_dev_dummy,
};

static int tegra_p1889_notifier_call(struct notifier_block *nb,
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
	.notifier_call = tegra_p1889_notifier_call,
};

static void p1889_panel_init(void)
{
	if (of_machine_is_compatible("nvidia,p1889,1")) {
		/* for HDMI primary */
		tegra_set_fixed_panel_ops(true,
				&p1889_hdmi_ops, "hdmi,display");
	} else {
		tegra_set_fixed_panel_ops(true,
				&edp_a_1080p_14_0_ops, "a-edp,1080p-14-0");
		tegra_set_fixed_panel_ops(false,
				&p1889_hdmi_ops, "hdmi,display");
		tegra_set_fixed_pwm_bl_ops(edp_a_1080p_14_0_ops.pwm_bl_ops);
	}
	bus_register_notifier(&platform_bus_type, &platform_nb);
}


static struct platform_device *p1889_devices[] __initdata = {
	&tegra_rtc_device,
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
};

static void __init tegra_p1889_early_init(void)
{
	struct tegra_clk_init_table *clk_table = p1889_clk_init_table;

	/* Early init for vcm30t124 MCM */
	tegra_vcm30_t124_early_init();

	/* Board specific clock POR */
	tegra_clk_init_from_table(clk_table);
	p1889_fixed_target_rate_init();

	tegra_clk_verify_parents();

	tegra_soc_device_init("p1889");
}

static void __init tegra_p1889_late_init(void)
{
	/* Create procfs entries for board_serial, skuinfo etc */
	tegra_init_board_info();

	/* Initialize the vcm30t124 specific devices */
	tegra_vcm30_t124_therm_mon_init();
	tegra_vcm30_t124_soctherm_init();
	tegra_vcm30_t124_usb_init();

	/* Initialize p1889 board specific devices */
	p1889_pca953x_init();
	p1889_audio_init();

	platform_add_devices(p1889_devices, ARRAY_SIZE(p1889_devices));

#ifdef CONFIG_TEGRA_WIFI_ENABLE
	p1889_wifi_init();
#endif

	tegra_vcm30_t124_suspend_init();

	isomgr_init();
	p1889_panel_init();

	/* Enable PMC wake source PWR_INT_N (Jetson TK1 Pro, Switch SW2) */
	tegra_pm_irq_set_wake(EXTERNAL_PMU_INT_N_WAKE, true);
}

static void __init tegra_p1889_dt_init(void)
{
	tegra_p1889_early_init();

#ifdef CONFIG_USE_OF
	tegra_vcm30_t124_populate_auxdata();
#endif

	tegra_p1889_late_init();
}

static const char * const p1889_dt_board_compat[] = {
	"nvidia,p1889",
	"nvidia,p1889,1", /* for HDMI primary*/
	NULL
};

DT_MACHINE_START(P1889, "p1889")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_vcm30_t124_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_p1889_dt_init,
	.dt_compat	= p1889_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END
