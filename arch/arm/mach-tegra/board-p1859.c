/*
 * arch/arm/mach-tegra/board-p1859.c
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
#include "board-p1859.h"
#include "devices.h"
#include "board-common.h"
#include <linux/platform/tegra/common.h>
#include "board-panel.h"
#include "tegra-of-dev-auxdata.h"
#include "linux/irqchip/tegra.h"

#define EXTERNAL_PMU_INT_N_WAKE         18

static int is_e1860_b00;

static __initdata struct tegra_clk_init_table e1860_a0x_i2s_clk_table[] = {
/*
 * audio clock tables based on baseboard revision. These rates will be set
 * early on during boot up. Also, they will remain fixed throughout.
 */
/*         clock        parent          rate            enable (always-on) */
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

static __initdata struct tegra_clk_init_table e1860_b0x_i2s_clk_table[] = {
/*         clock        parent          rate            enable (always-on) */
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

static __initdata struct tegra_clk_init_table
				e1860_b0x_modem_i2s_clk_table[] = {
/*         clock        parent          rate            enable (always-on) */
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
	{ "i2s4",	"pll_a_out0",	1024000,	false},
	{ "extern1",	"pll_a_out0",	24576000,	false},
	{ NULL,		NULL,		0,		0},
};

static int __init e1860_fixed_target_rate_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct tegra_clk_init_table *clk_table = (!is_e1860_b00) ?
			e1860_a0x_i2s_clk_table :
				modem_id ? e1860_b0x_modem_i2s_clk_table :
					e1860_b0x_i2s_clk_table;

	/* Set rate of audio clocks */
	tegra_clk_init_from_table(clk_table);

	/* Set target fixed rate of audio clocks */
	tegra_vcm30_t124_set_fixed_rate(clk_table);

	return 0;
}

static __initdata struct tegra_clk_init_table p1859_gpu_696mhz_clk[] = {
	 { "gk20a.gbus",         NULL,           696000000,      false},
	 { NULL,         NULL,           0,              0},
};
static __initdata struct tegra_clk_init_table p1859_gpu_780mhz_clk[] = {
	 { "gk20a.gbus",         NULL,           780000000,      false},
	 { NULL,         NULL,           0,              0},
};

/*
 * Set GPU default clock rate, based on p1859 board revision
 * On  older boards maximum gpu clock allowed is 696Mhz only.
 */
static void __init p1859_gpu_clk_rate_init(void)
{
	int sku_rev;
	sku_rev = tegra_board_get_skurev("61859");

	if (sku_rev < 300) {
		struct clk *c = tegra_get_clock_by_name("gbus");
		tegra_init_max_rate(c, 696000000);
		tegra_clk_init_from_table(p1859_gpu_696mhz_clk);
	} else
		tegra_clk_init_from_table(p1859_gpu_780mhz_clk);
}

/* I2C devices */
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
static int p1859_dev_dummy(struct device *dev)
{
	return 0;
}

static int p1859_dummy(void)
{
	return 0;
}

struct tegra_panel_ops p1859_hdmi_ops = {
	.enable = p1859_dev_dummy,
	.disable = p1859_dev_dummy,
	.postsuspend = p1859_dummy,
	.hotplug_init = p1859_dev_dummy,
};

static int tegra_p1859_notifier_call(struct notifier_block *nb,
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
	.notifier_call = tegra_p1859_notifier_call,
};

static void p1859_panel_init(void)
{
	if (of_machine_is_compatible("nvidia,p1859,1")) {
		/* device is hdmi primary. */
		tegra_set_fixed_panel_ops(true,
				&p1859_hdmi_ops, "hdmi,display");
	} else {
		tegra_set_fixed_panel_ops(true,
				&edp_a_1080p_14_0_ops, "a-edp,1080p-14-0");
		tegra_set_fixed_panel_ops(false,
				&p1859_hdmi_ops, "hdmi,display");
		tegra_set_fixed_pwm_bl_ops(edp_a_1080p_14_0_ops.pwm_bl_ops);
	}
	bus_register_notifier(&platform_bus_type, &platform_nb);
}

static void __init p1859_i2c_init(void)
{
	i2c_register_board_info(0, &ak4618_board_info, 1);
	i2c_register_board_info(0, &wm8731_board_info, 1);
	i2c_register_board_info(0, &ad1937_board_info, 1);
}

static struct platform_device *p1859_devices[] __initdata = {
	&tegra_rtc_device,
};

static void __init tegra_p1859_early_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct tegra_clk_init_table *clk_table = (!is_e1860_b00) ?
			e1860_a0x_i2s_clk_table :
				modem_id ? e1860_b0x_modem_i2s_clk_table :
					e1860_b0x_i2s_clk_table;

	/* Early init for vcm30t124 MCM */
	tegra_vcm30_t124_early_init();

	/* Board specific clock POR */
	tegra_clk_init_from_table(clk_table);
	e1860_fixed_target_rate_init();
	p1859_gpu_clk_rate_init();

	tegra_clk_verify_parents();

	tegra_soc_device_init("p1859");
}

static void __init tegra_p1859_late_init(void)
{
	/* Create procfs entries for board_serial, skuinfo etc */
	tegra_init_board_info();

	/* Initialize the vcm30t124 specific devices */
	tegra_vcm30_t124_therm_mon_init();
	tegra_vcm30_t124_soctherm_init();
	tegra_vcm30_t124_usb_init();

	/* Initialize p1859 board specific devices */
	p1859_i2c_init();
	p1859_audio_init();

	platform_add_devices(p1859_devices, ARRAY_SIZE(p1859_devices));

#ifdef CONFIG_TEGRA_WIFI_ENABLE
	p1859_wifi_init();
#endif
	tegra_vcm30_t124_suspend_init();

	isomgr_init();
	p1859_panel_init();

#ifdef CONFIG_BOARD_HAS_PWR_INT_WAKE_SOURCE
	/* Enable PMC wake source PWR_INT_N (Jetson TK1 Pro, Switch SW2) */
	tegra_pm_irq_set_wake(EXTERNAL_PMU_INT_N_WAKE, true);
#endif
}

static void __init tegra_p1859_dt_init(void)
{
	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);

	tegra_p1859_early_init();

#ifdef CONFIG_USE_OF
	tegra_vcm30_t124_populate_auxdata();
#endif

	tegra_p1859_late_init();
}

static const char * const p1859_dt_board_compat[] = {
	"nvidia,p1859",
	"nvidia,p1859,1", /* for HDMI primary*/
	NULL
};

void __init tegra_p1859_init_late(void)
{
	tegra_init_late();
	p1859_audio_dap_d_sel();
}

DT_MACHINE_START(P1859, "p1859")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_vcm30_t124_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_p1859_dt_init,
	.dt_compat	= p1859_dt_board_compat,
	.init_late	= tegra_p1859_init_late
MACHINE_END
