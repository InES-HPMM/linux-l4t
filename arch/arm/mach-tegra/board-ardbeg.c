/*
 * arch/arm/mach-tegra/board-ardbeg.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_data/tegra_xusb.h>
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi/spi-tegra.h>
#include <linux/nfc/pn544.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/smb349-charger.h>
#include <linux/max17048_battery.h>
#include <linux/leds.h>
#include <linux/i2c/at24.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/tegra_uart.h>
#include <linux/serial_tegra.h>
#include <linux/edp.h>
#include <linux/usb/tegra_usb_phy.h>

#include <mach/clk.h>
#include <mach/irqs.h>
#include <mach/tegra_fiq_debugger.h>

#include <mach/pinmux.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio-tegra.h>
#include <mach/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_usb_modem_power.h>

#include "board.h"
#include "board-ardbeg.h"
#include "board-common.h"
#include "board-touch-raydium.h"
#include "clock.h"
#include "common.h"
#include "devices.h"
#include "fuse.h"
#include "gpio-names.h"
#include "iomap.h"
#include "pm.h"
#include "pm-irq.h"
#include "tegra-board-id.h"

static struct board_info board_info, display_board_info;

static __initdata struct tegra_clk_init_table ardbeg_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	/* Setting vi_sensor-clk to true for validation purpose, will imapact
	 * power, later set to be false.*/
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "sbc1",	"pll_p",	25000000,	false},
	{ "sbc2",	"pll_p",	25000000,	false},
	{ "sbc3",	"pll_p",	25000000,	false},
	{ "sbc4",	"pll_p",	25000000,	false},
	{ "sbc5",	"pll_p",	25000000,	false},
	{ "sbc6",	"pll_p",	25000000,	false},
	{ NULL,		NULL,		0,		0},
};

#ifndef CONFIG_USE_OF
static struct tegra_i2c_platform_data ardbeg_i2c1_platform_data = {
	.bus_clk_rate	= 100000,
	.scl_gpio	= TEGRA_GPIO_I2C1_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C1_SDA,
};

static struct tegra_i2c_platform_data ardbeg_i2c2_platform_data = {
	.bus_clk_rate	= 100000,
	.is_clkon_always = true,
	.scl_gpio	= TEGRA_GPIO_I2C2_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C2_SDA,
};

static struct tegra_i2c_platform_data ardbeg_i2c3_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= TEGRA_GPIO_I2C3_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C3_SDA,
};

static struct tegra_i2c_platform_data ardbeg_i2c4_platform_data = {
	.bus_clk_rate	= 10000,
	.scl_gpio	= TEGRA_GPIO_I2C4_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C4_SDA,
};

static struct tegra_i2c_platform_data ardbeg_i2c5_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= TEGRA_GPIO_I2C5_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C5_SDA,
	.needs_cl_dvfs_clock = true,
};
#endif

static void ardbeg_i2c_init(void)
{
#ifndef CONFIG_USE_OF
	tegra11_i2c_device1.dev.platform_data = &ardbeg_i2c1_platform_data;
	tegra11_i2c_device2.dev.platform_data = &ardbeg_i2c2_platform_data;
	tegra11_i2c_device3.dev.platform_data = &ardbeg_i2c3_platform_data;
	tegra11_i2c_device4.dev.platform_data = &ardbeg_i2c4_platform_data;
	tegra11_i2c_device5.dev.platform_data = &ardbeg_i2c5_platform_data;

	platform_device_register(&tegra11_i2c_device5);
	platform_device_register(&tegra11_i2c_device4);
	platform_device_register(&tegra11_i2c_device3);
	platform_device_register(&tegra11_i2c_device2);
	platform_device_register(&tegra11_i2c_device1);
#endif
}

static struct platform_device *ardbeg_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};

static struct tegra_serial_platform_data ardbeg_uarta_pdata = {
	.dma_req_selector = 8,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data ardbeg_uartb_pdata = {
	.dma_req_selector = 9,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data ardbeg_uartc_pdata = {
	.dma_req_selector = 10,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data ardbeg_uartd_pdata = {
	.dma_req_selector = 19,
	.modem_interrupt = false,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data ardbeg_uart_pdata;
static struct tegra_uart_platform_data ardbeg_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = uart_console_debug_init(3);
	if (debug_port_id < 0)
		return;

	ardbeg_uart_devices[debug_port_id] = uart_console_debug_device;
}

static void __init ardbeg_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	ardbeg_uart_pdata.parent_clk_list = uart_parent_clk;
	ardbeg_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	ardbeg_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	ardbeg_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	ardbeg_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &ardbeg_uarta_pdata;
	tegra_uartb_device.dev.platform_data = &ardbeg_uartb_pdata;
	tegra_uartc_device.dev.platform_data = &ardbeg_uartc_pdata;
	tegra_uartd_device.dev.platform_data = &ardbeg_uartd_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(ardbeg_uart_devices,
			ARRAY_SIZE(ardbeg_uart_devices));
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device *ardbeg_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
	&tegra_ahub_device,
#if 0
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
	&ardbeg_audio_device,
	&tegra_hda_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

#ifndef CONFIG_USE_OF
static struct platform_device *ardbeg_spi_devices[] __initdata = {
	&tegra11_spi_device4,
};

static struct tegra_spi_platform_data ardbeg_spi_pdata = {
	.dma_req_sel		= 0,
	.spi_max_frequency	= 25000000,
	.clock_always_on	= false,
};

static void __init ardbeg_spi_init(void)
{
	tegra11_spi_device4.dev.platform_data = &ardbeg_spi_pdata;
	platform_add_devices(ardbeg_spi_devices,
			ARRAY_SIZE(ardbeg_spi_devices));
}
#else
static void __init ardbeg_spi_init(void)
{
}
#endif

#ifdef CONFIG_USE_OF
struct of_dev_auxdata ardbeg_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000600, "sdhci-tegra.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000400, "sdhci-tegra.2",
				NULL),
#if 0
	OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000000, "sdhci-tegra.0",
				&ardbeg_tegra_sdhci_platform_data0),
#endif
	OF_DEV_AUXDATA("nvidia,tegra114-camera", 0x0, "tegra_camera",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-host1x", TEGRA_HOST1X_BASE, "host1x",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-gr3d", TEGRA_GR3D_BASE, "gr3d",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-gr2d", TEGRA_GR2D_BASE, "gr2d",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-msenc", TEGRA_MSENC_BASE, "msenc",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-vi", TEGRA_VI_BASE, "vi",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-isp", TEGRA_ISP_BASE, "isp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-tsec", TEGRA_TSEC_BASE, "tsec",
				NULL),

	OF_DEV_AUXDATA("nvidia,tegra114-i2c", 0x7000c000, "tegra11-i2c.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-i2c", 0x7000c400, "tegra11-i2c.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-i2c", 0x7000c500, "tegra11-i2c.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-i2c", 0x7000c700, "tegra11-i2c.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-i2c", 0x7000d000, "tegra11-i2c.4",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-spi", 0x7000d400, "spi-tegra114.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-spi", 0x7000d600, "spi-tegra114.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-spi", 0x7000d800, "spi-tegra114.2",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-spi", 0x7000da00, "spi-tegra114.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-spi", 0x7000dc00, "spi-tegra114.4",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-spi", 0x7000de00, "spi-tegra114.5",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-apbdma", 0x6000a000, "tegra-apbdma",
				NULL),
	{}
};
#endif

static void __init tegra_ardbeg_early_init(void)
{
	tegra_clk_init_from_table(ardbeg_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("ardbeg");
}

static void __init tegra_ardbeg_late_init(void)
{
	platform_device_register(&tegra_pinmux_device);
	ardbeg_pinmux_init();
	ardbeg_i2c_init();
	ardbeg_uart_init();
	platform_add_devices(ardbeg_devices, ARRAY_SIZE(ardbeg_devices));
	//tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	ardbeg_sdhci_init();
	ardbeg_regulator_init();
	ardbeg_suspend_init();
#if 0
	ardbeg_emc_init();
	ardbeg_edp_init();
#endif
	isomgr_init();
#if 0
	ardbeg_touch_init();
#endif
	ardbeg_panel_init();
	ardbeg_kbc_init();
#if 0
	ardbeg_pmon_init();
#endif
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
#if 0
	ardbeg_sensors_init();
	ardbeg_soctherm_init();
#endif
	tegra_register_fuse();
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
}

static void __init ardbeg_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_ardbeg_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	tegra_ardbeg_early_init();
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, ardbeg_auxdata_lookup,
		&platform_bus);
#else
	platform_device_register(&tegra_gpio_device);
#endif

	tegra_ardbeg_late_init();
}

static void __init tegra_ardbeg_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1200*4*2 = 18432000 bytes */
	tegra_reserve(0, SZ_16M + SZ_2M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_16M + SZ_2M, SZ_4M);
#endif
	ardbeg_ramconsole_reserve(SZ_1M);
}

static const char * const ardbeg_dt_board_compat[] = {
	"nvidia,ardbeg",
	NULL
};

DT_MACHINE_START(ARDBEG, "ardbeg")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra11x_init_early,
	.init_irq	= tegra_dt_init_irq,
	.init_time	= tegra_init_timer,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= ardbeg_dt_board_compat,
MACHINE_END
