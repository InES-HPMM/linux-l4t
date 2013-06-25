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
#include <linux/i2c/i2c-hid.h>
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
#include <mach/pci.h>
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
#include <mach/xusb.h>
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

static struct resource ardbeg_bluedroid_pm_resources[] = {
	[0] = {
		.name   = "shutdown_gpio",
		.start  = TEGRA_GPIO_PR1,
		.end    = TEGRA_GPIO_PR1,
		.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "host_wake",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
	[2] = {
		.name = "gpio_ext_wake",
		.start  = TEGRA_GPIO_PEE1,
		.end    = TEGRA_GPIO_PEE1,
		.flags  = IORESOURCE_IO,
	},
	[3] = {
		.name = "gpio_host_wake",
		.start  = TEGRA_GPIO_PU6,
		.end    = TEGRA_GPIO_PU6,
		.flags  = IORESOURCE_IO,
	},
	[4] = {
		.name = "reset_gpio",
		.start  = TEGRA_GPIO_PX1,
		.end    = TEGRA_GPIO_PX1,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device ardbeg_bluedroid_pm_device = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ardbeg_bluedroid_pm_resources),
	.resource       = ardbeg_bluedroid_pm_resources,
};

static noinline void __init ardbeg_setup_bluedroid_pm(void)
{
	ardbeg_bluedroid_pm_resources[1].start =
		ardbeg_bluedroid_pm_resources[1].end =
				gpio_to_irq(TEGRA_GPIO_PU6);
	platform_device_register(&ardbeg_bluedroid_pm_device);
}

/*use board file for T12x*/
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) || !defined(CONFIG_USE_OF)
static struct i2c_board_info __initdata rt5639_board_info = {
	I2C_BOARD_INFO("rt5639", 0x1c),
};
static struct i2c_board_info __initdata rt5645_board_info = {
	I2C_BOARD_INFO("rt5645", 0x1a),
};
#endif

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
	{ "uarta",	"pll_p",	408000000,	false},
	{ "uartb",	"pll_p",	408000000,	false},
	{ "uartc",	"pll_p",	408000000,	false},
	{ "uartd",	"pll_p",	408000000,	false},
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

static struct i2c_hid_platform_data i2c_keyboard_pdata = {
	.hid_descriptor_address = 0x0,
};

static struct i2c_board_info __initdata i2c_keyboard_board_info = {
	I2C_BOARD_INFO("hid", 0x3B),
	.platform_data  = &i2c_keyboard_pdata,
};

static struct i2c_hid_platform_data i2c_touchpad_pdata = {
	.hid_descriptor_address = 0x20,
};

static struct i2c_board_info __initdata i2c_touchpad_board_info = {
	I2C_BOARD_INFO("hid", 0x2C),
	.platform_data  = &i2c_touchpad_pdata,
};

static void ardbeg_i2c_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
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
/*use board file for T12x*/
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) || !defined(CONFIG_USE_OF)
	i2c_register_board_info(0, &rt5639_board_info, 1);
	i2c_register_board_info(0, &rt5645_board_info, 1);
#endif

	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM363) {
		i2c_keyboard_board_info.irq = gpio_to_irq(I2C_KB_IRQ);
		i2c_register_board_info(1, &i2c_keyboard_board_info , 1);

		i2c_touchpad_board_info.irq = gpio_to_irq(I2C_TP_IRQ);
		i2c_register_board_info(1, &i2c_touchpad_board_info , 1);
	}
}

#ifndef CONFIG_USE_OF
static struct platform_device *ardbeg_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
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
#endif

static struct tegra_serial_platform_data ardbeg_uartd_pdata = {
	.dma_req_selector = 19,
	.modem_interrupt = false,
};

/*use board file for T12x*/
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) || !defined(CONFIG_USE_OF)
static struct tegra_asoc_platform_data ardbeg_audio_pdata_rt5639 = {
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_ldo1_en = TEGRA_GPIO_LDO_EN,
	.gpio_spkr_en = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.gpio_hp_mute = -1,
	.gpio_codec1 = -1,
	.gpio_codec2 = -1,
	.gpio_codec3 = -1,
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

static struct tegra_asoc_platform_data ardbeg_audio_pdata_rt5645 = {
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_ldo1_en = TEGRA_GPIO_LDO_EN,
	.gpio_spkr_en = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.gpio_hp_mute = -1,
	.gpio_codec1 = -1,
	.gpio_codec2 = -1,
	.gpio_codec3 = -1,
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

static void ardbeg_audio_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM363) {
		/*Laguna*/
		ardbeg_audio_pdata_rt5645.gpio_hp_det =
			TEGRA_GPIO_HP_DET;
		ardbeg_audio_pdata_rt5645.gpio_ldo1_en = -1;
	} else {
		/*Ardbeg*/
		ardbeg_audio_pdata_rt5645.gpio_hp_det =
			TEGRA_GPIO_HP_DET;
		ardbeg_audio_pdata_rt5645.gpio_ldo1_en =
			TEGRA_GPIO_LDO_EN;
	}

	ardbeg_audio_pdata_rt5639.gpio_hp_det =
		ardbeg_audio_pdata_rt5645.gpio_hp_det;

	ardbeg_audio_pdata_rt5639.gpio_ldo1_en =
		ardbeg_audio_pdata_rt5645.gpio_ldo1_en;

	ardbeg_audio_pdata_rt5639.codec_name = "rt5639.0-001c";
	ardbeg_audio_pdata_rt5639.codec_dai_name = "rt5639-aif1";
	ardbeg_audio_pdata_rt5645.codec_name = "rt5645.0-001a";
	ardbeg_audio_pdata_rt5645.codec_dai_name = "rt5645-aif1";
}

static struct platform_device ardbeg_audio_device_rt5645 = {
	.name = "tegra-snd-rt5645",
	.id = 0,
	.dev = {
		.platform_data = &ardbeg_audio_pdata_rt5645,
	},
};

static struct platform_device ardbeg_audio_device_rt5639 = {
	.name = "tegra-snd-rt5639",
	.id = 0,
	.dev = {
		.platform_data = &ardbeg_audio_pdata_rt5639,
	},
};

#endif

static void __init ardbeg_uart_init(void)
{
	int debug_port_id;

#ifndef CONFIG_USE_OF
	tegra_uarta_device.dev.platform_data = &ardbeg_uarta_pdata;
	tegra_uartb_device.dev.platform_data = &ardbeg_uartb_pdata;
	tegra_uartc_device.dev.platform_data = &ardbeg_uartc_pdata;
	platform_add_devices(ardbeg_uart_devices,
			ARRAY_SIZE(ardbeg_uart_devices));
#endif
	tegra_uartd_device.dev.platform_data = &ardbeg_uartd_pdata;
	if (!is_tegra_debug_uartport_hs()) {
		debug_port_id = uart_console_debug_init(3);
		if (debug_port_id < 0)
			return;

		platform_device_register(uart_console_debug_device);
	} else {
		tegra_uartd_device.dev.platform_data = &ardbeg_uartd_pdata;
		platform_device_register(&tegra_uartd_device);
	}

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

#ifdef CONFIG_ARCH_TEGRA_12x_SOC
static struct tegra_pci_platform_data laguna_pcie_platform_data = {
	.port_status[0]	= 1,
	.port_status[1]	= 1,
	.use_dock_detect	= 1,
	.gpio	= TEGRA_GPIO_PO1,
	.gpio_x1_slot	= PMU_TCA6416_GPIO(12),
};

static void laguna_pcie_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* root port 1(x1 slot) is supported only on of ERS-S board */
	if (board_info.board_id == BOARD_PM358 ||
		board_info.board_id == BOARD_PM363)
			laguna_pcie_platform_data.port_status[1] = 0;

	tegra_pci_device.dev.platform_data = &laguna_pcie_platform_data;
	platform_device_register(&tegra_pci_device);
}
#endif

static struct platform_device *ardbeg_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
#ifdef CONFIG_ARCH_TEGRA_12x_SOC
	&tegra12_se_device,
#endif
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	&tegra11_se_device,
#endif
#endif
/*use board file for T12x*/
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) || !defined(CONFIG_USE_OF)
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&ardbeg_audio_device_rt5639,
	&ardbeg_audio_device_rt5645,
	&tegra_spdif_device,
#endif
	&spdif_dit_device,
	&bluetooth_dit_device,
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

static void ardbeg_usb_init(void)
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

static struct tegra_xusb_board_data xusb_bdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P0 |
			TEGRA_XUSB_SS_P1 | TEGRA_XUSB_USB2_P1,
	/* ss_portmap[0:3] = SS0 map, ss_portmap[4:7] = SS1 map */
	.ss_portmap = (TEGRA_XUSB_SS_PORT_MAP_USB2_P0 << 0) |
			(TEGRA_XUSB_SS_PORT_MAP_USB2_P1 << 4),
};

static void ardbeg_xusb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
		xusb_bdata.portmap &= ~(TEGRA_XUSB_USB2_P0 | TEGRA_XUSB_SS_P0);

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
		xusb_bdata.portmap &= ~(TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_SS_P1);

	tegra_xusb_init(&xusb_bdata);
}

static void ardbeg_modem_init(void)
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
static struct platform_device *ardbeg_spi_devices[] __initdata = {
	&tegra11_spi_device1,
	&tegra11_spi_device4,
};

static struct tegra_spi_platform_data ardbeg_spi1_pdata = {
	.dma_req_sel		= 15,
	.spi_max_frequency	= 25000000,
	.clock_always_on	= false,
};

static struct tegra_spi_platform_data ardbeg_spi4_pdata = {
	.dma_req_sel		= 18,
	.spi_max_frequency	= 25000000,
	.clock_always_on	= false,
};

static void __init ardbeg_spi_init(void)
{
	tegra11_spi_device1.dev.platform_data = &ardbeg_spi1_pdata;
	tegra11_spi_device4.dev.platform_data = &ardbeg_spi4_pdata;
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
	OF_DEV_AUXDATA("nvidia,tegra30-ahub", 0x70080000, "tegra30-ahub",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra-audio-rt5645", 0x0, "tegra-snd-rt5645",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra-audio-rt5639", 0x0, "tegra-snd-rt5639",
				NULL),
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	OF_DEV_AUXDATA("nvidia,tegra114-apbdma", 0x6000a000, "tegra-apbdma",
				NULL),
#else
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
				NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006000, "serial-tegra.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006040, "serial-tegra.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006200, "serial-tegra.2",
				NULL),
	{}
};
#endif

static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern2",    "pll_p",        41000000,       false},
	{ "clk_out_2",  "extern2",      40800000,       false},
	{ NULL,         NULL,           0,              0},
};

struct rm_spi_ts_platform_data rm31080ts_ardbeg_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_D010,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 16,
};

struct spi_board_info rm31080a_ardbeg_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = TOUCH_SPI_ID,
		.chip_select = TOUCH_SPI_CS,
		.max_speed_hz = 12 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_ardbeg_data,
	},
};

static int __init ardbeg_touch_init(void)
{
	tegra_clk_init_from_table(touch_clk_init_table);
	rm31080a_ardbeg_spi_board[0].irq =
		gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
	touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
				TOUCH_GPIO_RST_RAYDIUM_SPI,
				&rm31080ts_ardbeg_data,
				&rm31080a_ardbeg_spi_board[0],
				ARRAY_SIZE(rm31080a_ardbeg_spi_board));
	return 0;
}

static void __init tegra_ardbeg_early_init(void)
{
	tegra_clk_init_from_table(ardbeg_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("ardbeg");
}

static void __init tegra_ardbeg_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);
	platform_device_register(&tegra_pinmux_device);
	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM363)
		laguna_pinmux_init();
	else
		ardbeg_pinmux_init();
	ardbeg_usb_init();
	ardbeg_modem_init();
	ardbeg_xusb_init();
	ardbeg_i2c_init();
	ardbeg_spi_init();
	ardbeg_uart_init();
/*use board file for T12x*/
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) || !defined(CONFIG_USE_OF)
	ardbeg_audio_init();
#endif
	platform_add_devices(ardbeg_devices, ARRAY_SIZE(ardbeg_devices));
	//tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	ardbeg_sdhci_init();
	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM363)
		laguna_regulator_init();
	else
		ardbeg_regulator_init();
	ardbeg_suspend_init();
#if 0
	ardbeg_emc_init();
	ardbeg_edp_init();
#endif
	isomgr_init();
	ardbeg_touch_init();
	ardbeg_panel_init();
	ardbeg_kbc_init();
	ardbeg_pmon_init();
	tegra_release_bootloader_fb();
#ifdef CONFIG_ARCH_TEGRA_12x_SOC
	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM363)
		laguna_pcie_init();
#endif
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
#if 0
	ardbeg_sensors_init();
	ardbeg_soctherm_init();
#endif
	ardbeg_setup_bluedroid_pm();
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
#ifdef CONFIG_ARCH_TEGRA_12x_SOC
	.init_early	= tegra12x_init_early,
#else
	.init_early	= tegra11x_init_early,
#endif
	.init_irq	= tegra_dt_init_irq,
	.init_time	= tegra_init_timer,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= ardbeg_dt_board_compat,
MACHINE_END
