/*
 * arch/arm/mach-tegra/board-vcm30_t124.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <mach/isomgr.h>
#include <mach/board_id.h>

#include "iomap.h"
#include "board.h"
#include "clock.h"
#include "vcm30_t124.h"
#include "board-vcm30_t124.h"
#include "devices.h"
#include "board-common.h"
#include "common.h"
#include "therm-monitor.h"
#include "board-panel.h"

#include "tegra-of-dev-auxdata.h"

static int is_e1860_b00;

static __initdata struct tegra_clk_init_table vcm30t124_a0x_i2s_clk_table[] = {
	{ "i2s4_sync",	NULL,		12288000,	false},
	{ "audio4",	"i2s4_sync",	12288000,	false},
	{ "audio4_2x",	"audio4",	12288000,	false},
	{ "i2s2_sync",	NULL,		12288000,	false},
	{ "audio2",	"i2s2_sync",	12288000,	false},
	{ "audio2_2x",	"audio2",	12288000,	false},
	{ "i2s0",	"pll_a_out0",	3072000,	false},
	{ "i2s1",	"pll_a_out0",	3072000,	false},
	{ "i2s2",	"audio2_2x",	12288000,	false},
	{ "i2s3",	"pll_a_out0",	3072000,	false},
	{ "i2s4",	"audio4_2x",	12288000,	false},
	{ "extern1",	"pll_a_out0",	12288000,	false},
	{ NULL,		NULL,		0,		0},
};

static __initdata struct tegra_clk_init_table vcm30t124_b0x_i2s_clk_table[] = {
	{ "i2s4_sync",	NULL,		12288000,	false},
	{ "audio4",	"i2s4_sync",	12288000,	false},
	{ "audio4_2x",	"audio4",	12288000,	false},
	{ "i2s2_sync",	NULL,		12288000,	false},
	{ "audio2",	"i2s2_sync",	12288000,	false},
	{ "audio2_2x",	"audio2",	12288000,	false},
	{ "i2s0",	"pll_a_out0",	12288000,	false},
	{ "i2s1",	"pll_a_out0",	3072000,	false},
	{ "i2s2",	"audio2_2x",	12288000,	false},
	{ "i2s3",	"pll_a_out0",	3072000,	false},
	{ "i2s4",	"audio4_2x",	12288000,	false},
	{ "extern1",	"pll_a_out0",	24576000,	false},
	{ NULL,		NULL,		0,		0},
};

static int __init tegra_fixed_target_rate_init(void)
{
	int modem_id = tegra_get_modem_id();

	struct tegra_clk_init_table vcm30t124_b0x_i2s4_clk_table[] = {
		{ "i2s4", "pll_a_out0", 1024000, false},
		{ NULL,    NULL,        0,        0},
	};

	struct tegra_clk_init_table *clk_table = is_e1860_b00 ?
		vcm30t124_b0x_i2s_clk_table : vcm30t124_a0x_i2s_clk_table;

	tegra_vcm30_t124_set_fixed_rate(clk_table);

	/* For voice call in b0x, set i2s4 clock in master mode */
	if (is_e1860_b00 && modem_id)
		tegra_vcm30_t124_set_fixed_rate(vcm30t124_b0x_i2s4_clk_table);

	return 0;
}


static struct i2c_board_info __initdata ak4618_board_info = {
	I2C_BOARD_INFO("ak4618", 0x10),
};

static struct i2c_board_info __initdata wm8731_board_info = {
	I2C_BOARD_INFO("wm8731", 0x1a),
};

static struct i2c_board_info __initdata ad1937_board_info = {
	I2C_BOARD_INFO("ad1937", 0x07),
};

/* Display panel/HDMI */
static int vcm30_t124_dev_dummy(struct device *dev)
{
	return 0;
}

static int vcm30_t124_dummy(void)
{
	return 0;
}

struct tegra_panel_ops vcm30t124_hdmi_ops = {
	.enable = vcm30_t124_dev_dummy,
	.disable = vcm30_t124_dummy,
	.postsuspend = vcm30_t124_dummy,
	.hotplug_init = vcm30_t124_dev_dummy,
};

static int tegra_vcm30t124_notifier_call(struct notifier_block *nb,
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
	.notifier_call = tegra_vcm30t124_notifier_call,
};

static void vcm30_t124_panel_init(void)
{
	tegra_set_fixed_panel_ops(true,
		&edp_a_1080p_14_0_ops, "a-edp,1080p-14-0");
	tegra_set_fixed_panel_ops(false, &vcm30t124_hdmi_ops, "hdmi,display");
	tegra_set_fixed_pwm_bl_ops(edp_a_1080p_14_0_ops.pwm_bl_ops);
	bus_register_notifier(&platform_bus_type, &platform_nb);
}

static void __init vcm30_t124_i2c_init(void)
{
	i2c_register_board_info(0, &ak4618_board_info, 1);
	i2c_register_board_info(0, &wm8731_board_info, 1);
	i2c_register_board_info(0, &ad1937_board_info, 1);
}

static struct platform_device *vcm30_t124_devices[] __initdata = {
	&tegra_rtc_device,
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
};

static void __init tegra_vcm30_t124_machine_early_init(void)
{
	struct tegra_clk_init_table *clk_table;

	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);

	clk_table = is_e1860_b00 ?
		vcm30t124_b0x_i2s_clk_table : vcm30t124_a0x_i2s_clk_table;

	tegra_vcm30_t124_early_init();

	tegra_clk_init_from_table(clk_table);

	tegra_clk_verify_parents();
	tegra_fixed_target_rate_init();
	tegra_soc_device_init("vcm30t124");
}

static void __init tegra_vcm30_t124_late_init(void)
{
	/* Create procfs entries for board_serial, skuinfo etc */
	tegra_init_board_info();

	/* Initialize the vcm30t124 specific devices */
	tegra_vcm30_t124_nor_init();
	tegra_vcm30_t124_therm_mon_init();
	tegra_vcm30_t124_soctherm_init();
	tegra_vcm30_t124_usb_init();

	/* Initialize VCM30_T124 board specific devcies */
	vcm30_t124_i2c_init();
	vcm30_t124_pca953x_init();
	vcm30_t124_audio_init();
	platform_add_devices(vcm30_t124_devices,
			ARRAY_SIZE(vcm30_t124_devices));
#ifdef CONFIG_TEGRA_WIFI_ENABLE
	vcm30_t124_wifi_init();
#endif
	vcm30_t124_regulator_init();

	tegra_vcm30_t124_suspend_init();

	isomgr_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	vcm30_t124_panel_init();
}

static void __init tegra_vcm30_t124_dt_init(void)
{
	tegra_vcm30_t124_machine_early_init();
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	carveout_linear_set(&tegra_vpr_cma_dev);
#endif
#ifdef CONFIG_USE_OF
	tegra_vcm30_t124_populate_auxdata();
#endif

	tegra_vcm30_t124_late_init();
}

static const char * const vcm30_t124_dt_board_compat[] = {
	"nvidia,vcm30t124",
	NULL
};

DT_MACHINE_START(VCM30_T124, "vcm30t124")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_vcm30_t124_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
        .init_time      = clocksource_of_init,
	.init_machine	= tegra_vcm30_t124_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= vcm30_t124_dt_board_compat,
        .init_late      = tegra_init_late
MACHINE_END
