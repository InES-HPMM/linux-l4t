/*
 * arch/arm/mach-tegra/board-pluto.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/nfc/pn544.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/smb349-charger.h>
#include <linux/max17048_battery.h>
#include <linux/leds.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/max8831.h>
#include <linux/of_platform.h>
#include <linux/a2220.h>

#include <asm/hardware/gic.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-tegra30.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/gpio-tegra.h>
#include <mach/tegra_fiq_debugger.h>
#include <mach/edp.h>
#include <mach/tegra_usb_modem_power.h>

#include "board.h"
#include "board-common.h"
#include "board-touch-raydium.h"
#include "clock.h"
#include "board-pluto.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "common.h"

static struct rfkill_gpio_platform_data pluto_bt_rfkill_pdata = {
	.name           = "bt_rfkill",
	.shutdown_gpio  = TEGRA_GPIO_PQ7,
	.type           = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device pluto_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &pluto_bt_rfkill_pdata,
	},
};

static noinline void __init pluto_setup_bt_rfkill(void)
{
	if ((tegra_get_commchip_id() == COMMCHIP_BROADCOM_BCM43241) ||
				(tegra_get_commchip_id() == COMMCHIP_DEFAULT))
		pluto_bt_rfkill_pdata.reset_gpio = TEGRA_GPIO_INVALID;
	else
		pluto_bt_rfkill_pdata.reset_gpio = TEGRA_GPIO_PU6;
	platform_device_register(&pluto_bt_rfkill_device);
}

static struct resource pluto_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PEE1,
			.end    = TEGRA_GPIO_PEE1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device pluto_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(pluto_bluesleep_resources),
	.resource       = pluto_bluesleep_resources,
};

static noinline void __init pluto_setup_bluesleep(void)
{
	pluto_bluesleep_resources[2].start =
		pluto_bluesleep_resources[2].end =
			gpio_to_irq(TEGRA_GPIO_PU6);
	platform_device_register(&pluto_bluesleep_device);
	return;
}
static __initdata struct tegra_clk_init_table pluto_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio2",	"i2s2_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "audio4",	"i2s4_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "extern3",	"clk_m",	12000000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data pluto_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C1_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C1_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data pluto_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_I2C2_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C2_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data pluto_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C3_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C3_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data pluto_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C4_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C4_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data pluto_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C5_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C5_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct i2c_board_info __initdata cs42l73_board_info = {
	I2C_BOARD_INFO("cs42l73", 0x4a),
};

static struct i2c_board_info __initdata pluto_codec_a2220_info = {
	I2C_BOARD_INFO("audience_a2220", 0x3E),
};


static void pluto_i2c_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
#ifndef CONFIG_ARCH_TEGRA_11x_SOC
	tegra_i2c_device1.dev.platform_data = &pluto_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &pluto_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &pluto_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &pluto_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &pluto_i2c5_platform_data;

	i2c_register_board_info(1, &pluto_i2c_led_info, 1);

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

#else
	tegra11_i2c_device1.dev.platform_data = &pluto_i2c1_platform_data;
	tegra11_i2c_device2.dev.platform_data = &pluto_i2c2_platform_data;
	tegra11_i2c_device3.dev.platform_data = &pluto_i2c3_platform_data;
	tegra11_i2c_device4.dev.platform_data = &pluto_i2c4_platform_data;
	tegra11_i2c_device5.dev.platform_data = &pluto_i2c5_platform_data;

	platform_device_register(&tegra11_i2c_device5);
	platform_device_register(&tegra11_i2c_device4);
	platform_device_register(&tegra11_i2c_device3);
	platform_device_register(&tegra11_i2c_device2);
	platform_device_register(&tegra11_i2c_device1);

#endif

	i2c_register_board_info(0, &pluto_codec_a2220_info, 1);
	i2c_register_board_info(0, &cs42l73_board_info, 1);
}

static struct platform_device *pluto_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data pluto_uart_pdata;
static struct tegra_uart_platform_data pluto_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = uart_console_debug_init(3);
	if (debug_port_id < 0)
		return;
	pluto_uart_devices[debug_port_id] = uart_console_debug_device;
}

static void __init pluto_uart_init(void)
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
	pluto_uart_pdata.parent_clk_list = uart_parent_clk;
	pluto_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	pluto_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	pluto_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	pluto_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &pluto_uart_pdata;
	tegra_uartb_device.dev.platform_data = &pluto_uart_pdata;
	tegra_uartc_device.dev.platform_data = &pluto_uart_pdata;
	tegra_uartd_device.dev.platform_data = &pluto_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(pluto_uart_devices,
				ARRAY_SIZE(pluto_uart_devices));
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

static struct tegra_asoc_platform_data pluto_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= TEGRA_GPIO_EXT_MIC_EN,
	.gpio_ldo1_en		= TEGRA_GPIO_LDO1_EN,
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 0,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.channels       = 2,
	},
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= 2,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.rate		= 16000,
		.channels	= 2,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
		.sample_size	= 16,
	},
};

static struct platform_device pluto_audio_device = {
	.name	= "tegra-snd-cs42l73",
	.id	= 2,
	.dev	= {
		.platform_data = &pluto_audio_pdata,
	},
};

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *pluto_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&pluto_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

#ifdef CONFIG_USB_SUPPORT
static struct tegra_usb_platform_data tegra_ehci3_hsic_smsc_hub_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
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
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = true,
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
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static struct regulator *baseband_reg;
static struct gpio modem_gpios[] = { /* i500 modem */
	{MDM_RST, GPIOF_OUT_INIT_LOW, "MODEM RESET"},
	{MDM_ACK, GPIOF_OUT_INIT_HIGH, "MODEM ACK1"},
};

static void baseband_post_phy_on(void);
static void baseband_pre_phy_off(void);

static struct tegra_usb_phy_platform_ops baseband_plat_ops = {
	.pre_phy_off = baseband_pre_phy_off,
	.post_phy_on = baseband_post_phy_on,
};

static struct gpio modem2_gpios[] = {
	{MDM2_PWR_ON, GPIOF_OUT_INIT_LOW, "MODEM2 PWR ON"},
	{MDM2_RST, GPIOF_DIR_OUT, "MODEM2 RESET"},
	{MDM2_ACK2, GPIOF_OUT_INIT_HIGH, "MODEM2 ACK2"},
	{MDM2_ACK1, GPIOF_OUT_INIT_LOW, "MODEM2 ACK1"},
};

static void baseband2_post_phy_on(void);
static void baseband2_pre_phy_off(void);

static struct tegra_usb_phy_platform_ops baseband2_plat_ops = {
	.pre_phy_off = baseband2_pre_phy_off,
	.post_phy_on = baseband2_post_phy_on,
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_baseband_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.ops = &baseband_plat_ops,
};

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
static struct tegra_usb_platform_data tegra_ehci3_hsic_baseband2_pdata = {
#else
static struct tegra_usb_platform_data tegra_ehci2_hsic_baseband2_pdata = {
#endif
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
	},
	.ops = &baseband2_plat_ops,
};

static void baseband_post_phy_on(void)
{
	gpio_set_value(MDM_ACK, 0);
}

static void baseband_pre_phy_off(void)
{
	gpio_set_value(MDM_ACK, 1);
}

static int baseband_init(void)
{
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret) {
		pr_warn("%s:gpio request failed\n", __func__);
		return ret;
	}

	baseband_reg = regulator_get(NULL, "vdd_core_bb");
	if (IS_ERR_OR_NULL(baseband_reg))
		pr_warn("%s: baseband regulator get failed\n", __func__);
	else
		regulator_enable(baseband_reg);

	/* export GPIO for user space access through sysfs */
	gpio_export(MDM_RST, false);

	return 0;
}

static const struct tegra_modem_operations baseband_operations = {
	.init = baseband_init,
};

static struct tegra_usb_modem_power_platform_data baseband_pdata = {
	.ops = &baseband_operations,
	.wake_gpio = MDM_REQ,
	.wake_irq_flags = IRQF_TRIGGER_FALLING,
	.boot_gpio = MDM_COLDBOOT,
	.boot_irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.autosuspend_delay = 2000,
	.short_autosuspend_delay = 50,
	.tegra_ehci_device = &tegra_ehci2_device,
	.tegra_ehci_pdata = &tegra_ehci2_hsic_baseband_pdata,
};

static struct platform_device icera_baseband_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

static void baseband2_post_phy_on(void)
{
	/* set MDM2_ACK2 low */
	gpio_set_value(MDM2_ACK2, 0);
}

static void baseband2_pre_phy_off(void)
{
	/* set MDM2_ACK2 high */
	gpio_set_value(MDM2_ACK2, 1);
}

static void baseband2_start(void)
{
	/*
	 *  Leave baseband powered OFF.
	 *  User-space daemons will take care of powering it up.
	 */
	pr_info("%s\n", __func__);
	gpio_set_value(MDM2_PWR_ON, 0);
}

static void baseband2_reset(void)
{
	/* Initiate power cycle on baseband sub system */
	pr_info("%s\n", __func__);
	gpio_set_value(MDM2_PWR_ON, 0);
	mdelay(200);
	gpio_set_value(MDM2_PWR_ON, 1);
}

static int baseband2_init(void)
{
	int ret;

	ret = gpio_request_array(modem2_gpios, ARRAY_SIZE(modem2_gpios));
	if (ret)
		return ret;

	/* enable pull-up for MDM2_REQ2 */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_GPIO_PV1,
				    TEGRA_PUPD_PULL_UP);

	/* export GPIO for user space access through sysfs */
	gpio_export(MDM2_PWR_ON, false);

	return 0;
}

static const struct tegra_modem_operations baseband2_operations = {
	.init = baseband2_init,
	.start = baseband2_start,
	.reset = baseband2_reset,
};

static struct tegra_usb_modem_power_platform_data baseband2_pdata = {
	.ops = &baseband2_operations,
	.wake_gpio = MDM2_REQ2,
	.wake_irq_flags = IRQF_TRIGGER_FALLING,
	.boot_gpio = MDM2_COLDBOOT,
	.boot_irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.autosuspend_delay = 2000,
	.short_autosuspend_delay = 50,
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	.tegra_ehci_device = &tegra_ehci3_device,
	.tegra_ehci_pdata = &tegra_ehci3_hsic_baseband2_pdata,
#else
	.tegra_ehci_device = &tegra_ehci2_device,
	.tegra_ehci_pdata = &tegra_ehci2_hsic_baseband2_pdata,
#endif
};

static struct platform_device icera_baseband2_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband2_pdata,
	},
};

static void pluto_usb_init(void)
{
	int modem_id = tegra_get_modem_id();

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	if (!modem_id) {
		tegra_ehci3_device.dev.platform_data =
			&tegra_ehci3_hsic_smsc_hub_pdata;
		platform_device_register(&tegra_ehci3_device);
	}
}

static void pluto_modem_init(void)
{
	int modem_id = tegra_get_modem_id();

	pr_info("%s: modem_id = %d\n", __func__, modem_id);

	switch (modem_id) {
	case TEGRA_BB_I500: /* on board i500 HSIC */
		platform_device_register(&icera_baseband_device);
		break;
	case TEGRA_BB_I500SWD: /* i500 SWD HSIC */
		platform_device_register(&icera_baseband2_device);
		break;
	default:
		return;
	}
}

#else
static void pluto_usb_init(void) { }
static void pluto_modem_init(void) { }
#endif

static void pluto_audio_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	pluto_audio_pdata.codec_name = "cs42l73.0-004a";
	pluto_audio_pdata.codec_dai_name = "cs42l73-vsp";
}

static struct platform_device *pluto_spi_devices[] __initdata = {
        &tegra11_spi_device4,
};

struct spi_clk_parent spi_parent_clk_pluto[] = {
        [0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
        [1] = {.name = "pll_m"},
        [2] = {.name = "clk_m"},
#else
        [1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data pluto_spi_pdata = {
	.is_dma_based           = false,
	.max_dma_buffer         = 16 * 1024,
        .is_clkon_always        = false,
        .max_rate               = 25000000,
};

static void __init pluto_spi_init(void)
{
        int i;
        struct clk *c;
        struct board_info board_info, display_board_info;

        tegra_get_board_info(&board_info);
        tegra_get_display_board_info(&display_board_info);

        for (i = 0; i < ARRAY_SIZE(spi_parent_clk_pluto); ++i) {
                c = tegra_get_clock_by_name(spi_parent_clk_pluto[i].name);
                if (IS_ERR_OR_NULL(c)) {
                        pr_err("Not able to get the clock for %s\n",
                                                spi_parent_clk_pluto[i].name);
                        continue;
                }
                spi_parent_clk_pluto[i].parent_clk = c;
                spi_parent_clk_pluto[i].fixed_clk_rate = clk_get_rate(c);
        }
        pluto_spi_pdata.parent_clk_list = spi_parent_clk_pluto;
        pluto_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk_pluto);
	tegra11_spi_device4.dev.platform_data = &pluto_spi_pdata;
        platform_add_devices(pluto_spi_devices,
                                ARRAY_SIZE(pluto_spi_devices));
}

static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern2",    "pll_p",        41000000,       false},
	{ "clk_out_2",  "extern2",      40800000,       false},
	{ NULL,         NULL,           0,              0},
};

struct rm_spi_ts_platform_data rm31080ts_pluto_data = {
	.gpio_reset = 0,
	.config = 0,
	.platform_id = RM_PLATFORM_P005,
	.name_of_clock = "clk_out_2",
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 0,
};

struct spi_board_info rm31080a_pluto_spi_board[1] = {
	{
	 .modalias = "rm_ts_spidev",
	 .bus_num = 3,
	 .chip_select = 2,
	 .max_speed_hz = 12 * 1000 * 1000,
	 .mode = SPI_MODE_0,
	 .controller_data = &dev_cdata,
	 .platform_data = &rm31080ts_pluto_data,
	 },
};

static int __init pluto_touch_init(void)
{
	tegra_clk_init_from_table(touch_clk_init_table);
	clk_enable(tegra_get_clock_by_name("clk_out_2"));
	mdelay(20);
	rm31080a_pluto_spi_board[0].irq = gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
	touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
				TOUCH_GPIO_RST_RAYDIUM_SPI,
				&rm31080ts_pluto_data,
				&rm31080a_pluto_spi_board[0],
				ARRAY_SIZE(rm31080a_pluto_spi_board));
	return 0;
}

static void __init tegra_pluto_init(void)
{
	tegra_battery_edp_init(2500);
	tegra_clk_init_from_table(pluto_clk_init_table);
	tegra_soc_device_init("tegra_pluto");
	tegra_enable_pinmux();
	pluto_pinmux_init();
	pluto_i2c_init();
	pluto_spi_init();
	pluto_usb_init();
	pluto_edp_init();
	pluto_uart_init();
	pluto_audio_init();
	platform_add_devices(pluto_devices, ARRAY_SIZE(pluto_devices));
	//tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	pluto_sdhci_init();
	pluto_regulator_init();
	pluto_suspend_init();
	pluto_touch_init();
	pluto_emc_init();
	pluto_panel_init();
	pluto_pmon_init();
	pluto_kbc_init();
	pluto_setup_bluesleep();
	pluto_setup_bt_rfkill();
	tegra_release_bootloader_fb();
	pluto_modem_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	pluto_sensors_init();
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
}

static void __init pluto_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_pluto_dt_init(void)
{
	tegra_pluto_init();

	of_platform_populate(NULL,
		of_default_bus_match_table, NULL, NULL);
}

static void __init tegra_pluto_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* for PANEL_5_SHARP_1080p: 1920*1080*4*2 = 16588800 bytes */
	tegra_reserve(0, SZ_16M, SZ_4M);
#else
	tegra_reserve(SZ_128M, SZ_16M, SZ_4M);
#endif
	pluto_ramconsole_reserve(SZ_1M);
}

static const char * const pluto_dt_board_compat[] = {
	"nvidia,pluto",
	NULL
};

MACHINE_START(TEGRA_PLUTO, "tegra_pluto")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_pluto_reserve,
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	.init_early	= tegra30_init_early,
#else
	.init_early     = tegra11x_init_early,
#endif
	.init_irq	= tegra_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_pluto_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= pluto_dt_board_compat,
MACHINE_END
