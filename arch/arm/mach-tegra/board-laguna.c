/*
 * arch/arm/mach-tegra/board-laguna.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <asm/hardware/gic.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/tegra_fiq_debugger.h>

#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/gpio-tegra.h>
#include <mach/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_usb_modem_power.h>

#include "board-touch-raydium.h"
#include "board.h"
#include "board-laguna.h"
#include "board-common.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "pm-irq.h"
#include "common.h"
#include "tegra-board-id.h"

static struct board_info board_info, display_board_info;

static struct i2c_board_info __initdata rt5645_board_info = {
	I2C_BOARD_INFO("rt5645", 0x1a),
};

static __initdata struct tegra_clk_init_table laguna_clk_init_table[] = {
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
static struct tegra_i2c_platform_data laguna_i2c1_platform_data = {
	.bus_clk_rate	= 100000,
	.scl_gpio	= TEGRA_GPIO_I2C1_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C1_SDA,
};

static struct tegra_i2c_platform_data laguna_i2c2_platform_data = {
	.bus_clk_rate	= 100000,
	.is_clkon_always = true,
	.scl_gpio	= TEGRA_GPIO_I2C2_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C2_SDA,
};

static struct tegra_i2c_platform_data laguna_i2c3_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= TEGRA_GPIO_I2C3_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C3_SDA,
};

static struct tegra_i2c_platform_data laguna_i2c4_platform_data = {
	.bus_clk_rate	= 10000,
	.scl_gpio	= TEGRA_GPIO_I2C4_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C4_SDA,
};

static struct tegra_i2c_platform_data laguna_i2c5_platform_data = {
	.bus_clk_rate	= 400000,
	.scl_gpio	= TEGRA_GPIO_I2C5_SCL,
	.sda_gpio	= TEGRA_GPIO_I2C5_SDA,
	.needs_cl_dvfs_clock = true,
};
#endif

static void laguna_i2c_init(void)
{
#ifndef CONFIG_USE_OF
	tegra11_i2c_device1.dev.platform_data = &laguna_i2c1_platform_data;
	tegra11_i2c_device2.dev.platform_data = &laguna_i2c2_platform_data;
	tegra11_i2c_device3.dev.platform_data = &laguna_i2c3_platform_data;
	tegra11_i2c_device4.dev.platform_data = &laguna_i2c4_platform_data;
	tegra11_i2c_device5.dev.platform_data = &laguna_i2c5_platform_data;

	platform_device_register(&tegra11_i2c_device5);
	platform_device_register(&tegra11_i2c_device4);
	platform_device_register(&tegra11_i2c_device3);
	platform_device_register(&tegra11_i2c_device2);
	platform_device_register(&tegra11_i2c_device1);
#endif
	i2c_register_board_info(0, &rt5645_board_info, 1);
}

static struct platform_device *laguna_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};

static struct tegra_serial_platform_data laguna_uarta_pdata = {
	.dma_req_selector = 8,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data laguna_uartb_pdata = {
	.dma_req_selector = 9,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data laguna_uartc_pdata = {
	.dma_req_selector = 10,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data laguna_uartd_pdata = {
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

static struct tegra_uart_platform_data laguna_uart_pdata;

static struct tegra_asoc_platform_data laguna_audio_pdata = {
	.gpio_spkr_en = TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_hp_mute = -1,
	.gpio_int_mic_en = TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en = TEGRA_GPIO_EXT_MIC_EN,
	.gpio_ldo1_en = TEGRA_GPIO_LDO1_EN,
	.gpio_codec1 = TEGRA_GPIO_CODEC1_EN,
	.gpio_codec2 = TEGRA_GPIO_CODEC2_EN,
	.gpio_codec3 = TEGRA_GPIO_CODEC3_EN,
	.i2s_param[HIFI_CODEC]       = {
		.audio_port_id = 1,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BT_SCO] = {
		.audio_port_id = 3,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_DSP_A,
	},
};

static void laguna_audio_init(void)
{
	laguna_audio_pdata.codec_name = "rt5645.0-001a";
	laguna_audio_pdata.codec_dai_name = "rt5645-aif1";
}

static struct platform_device laguna_audio_device = {
	.name       = "tegra-snd-rt5645",
	.id       = 0,
	.dev       = {
		.platform_data = &laguna_audio_pdata,
	},
};

static struct tegra_uart_platform_data laguna_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = uart_console_debug_init(3);
	if (debug_port_id < 0)
		return;

	laguna_uart_devices[debug_port_id] = uart_console_debug_device;
}

static void __init laguna_uart_init(void)
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
	laguna_uart_pdata.parent_clk_list = uart_parent_clk;
	laguna_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	laguna_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	laguna_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	laguna_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &laguna_uarta_pdata;
	tegra_uartb_device.dev.platform_data = &laguna_uartb_pdata;
	tegra_uartc_device.dev.platform_data = &laguna_uartc_pdata;
	tegra_uartd_device.dev.platform_data = &laguna_uartd_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(laguna_uart_devices,
			ARRAY_SIZE(laguna_uart_devices));
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

static struct platform_device *laguna_devices[] __initdata = {
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
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&laguna_audio_device,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
	.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_smsc_hub_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};


static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void laguna_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
		platform_device_register(&tegra_otg_device);
		/* Setup the udc platform data */
		tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
		platform_device_register(&tegra_ehci3_device);
	}
}

static void laguna_modem_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct board_info board_info;
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	tegra_get_board_info(&board_info);
	pr_info("%s: modem_id = %d\n", __func__, modem_id);

	switch (modem_id) {
	case TEGRA_BB_HSIC_HUB: /* HSIC hub */
		if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)) {
			tegra_ehci2_device.dev.platform_data =
				&tegra_ehci2_hsic_smsc_hub_pdata;
			platform_device_register(&tegra_ehci2_device);
		}
		break;
	default:
		return;
	}
}

#ifndef CONFIG_USE_OF
static struct platform_device *laguna_spi_devices[] __initdata = {
	&tegra11_spi_device4,
};

static struct tegra_spi_platform_data laguna_spi_pdata = {
	.dma_req_sel		= 0,
	.spi_max_frequency	= 25000000,
	.clock_always_on	= false,
};

static void __init laguna_spi_init(void)
{
	tegra11_spi_device4.dev.platform_data = &laguna_spi_pdata;
	platform_add_devices(laguna_spi_devices,
			ARRAY_SIZE(laguna_spi_devices));
}
#else
static void __init laguna_spi_init(void)
{
}
#endif

#ifdef CONFIG_USE_OF
struct of_dev_auxdata laguna_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000600, "sdhci-tegra.3",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000400, "sdhci-tegra.2",
				NULL),
#if 0
	OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000000, "sdhci-tegra.0",
				&laguna_tegra_sdhci_platform_data0),
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

static void __init tegra_laguna_early_init(void)
{
	tegra_clk_init_from_table(laguna_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("laguna");
}

static void __init tegra_laguna_late_init(void)
{
	platform_device_register(&tegra_pinmux_device);
	laguna_pinmux_init();
	laguna_usb_init();
	laguna_modem_init();
	laguna_i2c_init();
	laguna_uart_init();
	laguna_audio_init();
	platform_add_devices(laguna_devices, ARRAY_SIZE(laguna_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	laguna_sdhci_init();
	laguna_regulator_init();
	laguna_suspend_init();
#if 0
	laguna_emc_init();
	laguna_edp_init();
#endif
	isomgr_init();
#if 0
	laguna_touch_init();
#endif
	laguna_panel_init();
	laguna_kbc_init();
#if 0
	laguna_pmon_init();
#endif
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
#if 0
	laguna_sensors_init();
	laguna_soctherm_init();
#endif
	tegra_register_fuse();
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
}

static void __init laguna_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_laguna_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	tegra_laguna_early_init();
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, laguna_auxdata_lookup,
		&platform_bus);
#else
	platform_device_register(&tegra_gpio_device);
#endif
	tegra_laguna_late_init();
}

static void __init tegra_laguna_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1200*4*2 = 18432000 bytes */
	tegra_reserve(0, SZ_16M + SZ_2M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_16M + SZ_2M, SZ_4M);
#endif
	laguna_ramconsole_reserve(SZ_1M);
}

static const char * const laguna_dt_board_compat[] = {
	"nvidia,laguna",
	NULL
};

DT_MACHINE_START(LAGUNA, "laguna")
	.atag_offset		= 0x100,
	.smp			= smp_ops(tegra_smp_ops),
	.map_io			= tegra_map_common_io,
	.reserve		= tegra_laguna_reserve,
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	.init_early		= tegra11x_init_early,
#else
	.init_early		= tegra12x_init_early,
#endif
	.init_irq		= tegra_dt_init_irq,
	.handle_irq		= gic_handle_irq,
	.timer			= &tegra_sys_timer,
	.init_machine		= tegra_laguna_dt_init,
	.restart		= tegra_assert_system_reset,
	.dt_compat		= laguna_dt_board_compat,
MACHINE_END
