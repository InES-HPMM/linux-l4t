 /*
 * arch/arm64/mach-tegra/board-t210ref.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/maxim_sti.h>
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
#include <linux/platform_data/serial-tegra.h>
#include <linux/edp.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/mfd/palmas.h>
#include <linux/clk/tegra.h>
#include <media/tegra_dtv.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-soc.h>
#include <linux/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <linux/platform_data/tegra_ahci.h>
#include <linux/irqchip/tegra.h>
#include <sound/max98090.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>

#include <mach/irqs.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/dc.h>
#include <mach/tegra_usb_pad_ctrl.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio-tegra.h>
#include <mach/xusb.h>

#include "board.h"
#include "board-panel.h"
#include "board-common.h"
#include "board-t210ref.h"
#include "board-touch-raydium.h"
#include "board-touch-maxim_sti.h"
#include "clock.h"
#include "common.h"
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "pm.h"
#include "tegra-board-id.h"
#include "tegra-of-dev-auxdata.h"
#include "tegra12_emc.h"

static struct board_info board_info, display_board_info;

static struct i2c_board_info __initdata rt5639_board_info = {
	I2C_BOARD_INFO("rt5639", 0x1c),
};

static __initdata struct tegra_clk_init_table t210ref_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	48000000,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	12288000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "vi_sensor2",	"pll_p",	150000000,	false},
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

static void t210ref_i2c_init(void)
{
	i2c_register_board_info(0, &rt5639_board_info, 1);
}

static struct tegra_serial_platform_data t210ref_uarta_pdata = {
	.dma_req_selector = 8,
	.modem_interrupt = false,
};

static struct tegra_asoc_platform_data t210ref_audio_pdata_rt5639 = {
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
		.sample_size	= 16,
		.channels       = 2,
		.bit_clk	= 1536000,
	},
	.i2s_param[BT_SCO] = {
		.audio_port_id = 3,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_DSP_A,
	},
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.rate		= 16000,
		.channels	= 2,
		.bit_clk	= 1024000,
	},
};

static void t210ref_audio_init(void)
{
	/*T210ref*/
	t210ref_audio_pdata_rt5639.gpio_hp_det =
		TEGRA_GPIO_HP_DET;
	t210ref_audio_pdata_rt5639.use_codec_jd_irq = false;

	t210ref_audio_pdata_rt5639.gpio_hp_det_active_high = 0;
	t210ref_audio_pdata_rt5639.gpio_ldo1_en = TEGRA_GPIO_LDO_EN;

	t210ref_audio_pdata_rt5639.i2s_param[HIFI_CODEC].audio_port_id = 0;
	t210ref_audio_pdata_rt5639.i2s_param[BT_SCO].audio_port_id = 1;
	t210ref_audio_pdata_rt5639.codec_name = "rt5639.0-001c";
	t210ref_audio_pdata_rt5639.codec_dai_name = "rt5639-aif1";
}

static struct platform_device t210ref_audio_device_rt5639 = {
	.name = "tegra-snd-rt5639",
	.id = 0,
	.dev = {
		.platform_data = &t210ref_audio_pdata_rt5639,
	},
};

static void __init e2141_uart_init(void)
{

	tegra_uarta_device.dev.platform_data = &t210ref_uarta_pdata;
	if (!is_tegra_debug_uartport_hs()) {
		int debug_port_id = uart_console_debug_init(0);
		if (debug_port_id < 0)
			return;

#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
		tegra_serial_debug_init(TEGRA_UARTA_BASE,
				INT_WDT_CPU, NULL, -1, -1);
#else
		platform_device_register(uart_console_debug_device);
#endif
	} else {
		tegra_uarta_device.dev.platform_data = &t210ref_uarta_pdata;
		platform_device_register(&tegra_uarta_device);
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

static struct platform_device *t210ref_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
#if !defined(CONFIG_ARM64)
	&tegra_udc_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE) && !defined(CONFIG_USE_OF)
	&tegra12_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_hda_device,
	&tegra_offload_device,
	&tegra30_avp_audio_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
#if defined(CONFIG_TEGRA_PTM)
	&tegra_ptm_device,
#endif
#ifdef CONFIG_TEGRA_CEC_SUPPORT
	&tegra_cec_device,
#endif
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.charging_supported = true,
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
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
		.xcvr_hsslew_lsb = 2,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
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

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
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

static struct gpio modem_gpios[] = { /* Bruce modem */
	{MODEM_EN, GPIOF_OUT_INIT_HIGH, "MODEM EN"},
	{MDM_RST, GPIOF_OUT_INIT_LOW, "MODEM RESET"},
	{MDM_SAR0, GPIOF_OUT_INIT_LOW, "MODEM SAR0"},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_baseband_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_smsc_hub_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};


static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void t210ref_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();
	int modem_id = tegra_get_modem_id();
	struct board_info bi;
	tegra_get_pmu_board_info(&bi);


	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_pdata.is_xhci = false;
		tegra_udc_pdata.u_data.dev.is_xhci = false;
	} else {
		tegra_otg_pdata.is_xhci = true;
		tegra_udc_pdata.u_data.dev.is_xhci = true;
	}

#if !defined(CONFIG_ARM64)
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
#endif

	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		if (!modem_id) {
				tegra_ehci2_device.dev.platform_data =
					&tegra_ehci2_utmi_pdata;
				platform_device_register(&tegra_ehci2_device);
		}
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
			tegra_ehci3_device.dev.platform_data =
				&tegra_ehci3_utmi_pdata;
			platform_device_register(&tegra_ehci3_device);
	}

}

static struct tegra_xusb_platform_data xusb_pdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P0 | TEGRA_XUSB_SS_P1 |
			TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_USB2_P2,
};

#ifdef CONFIG_TEGRA_XUSB_PLATFORM
static void t210ref_xusb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	xusb_pdata.lane_owner = (u8) tegra_get_lane_owner_info();


	pr_info("Shield ERS 0x%x\n", board_info.board_id);
	/* Shield ERS */
	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
		xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0 |
			TEGRA_XUSB_SS_P0);

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
		xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P1 |
				TEGRA_XUSB_USB2_P2 | TEGRA_XUSB_SS_P1);
		/* FIXME Add for UTMIP2 when have odmdata assigend */

	if (usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)
		xusb_pdata.portmap |= TEGRA_XUSB_HSIC_P0;

	if (usb_port_owner_info & HSIC2_PORT_OWNER_XUSB)
		xusb_pdata.portmap |= TEGRA_XUSB_HSIC_P1;
}
#endif

static int baseband_init(void)
{
	struct pinctrl_dev *pctl_dev;
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret) {
		pr_warn("%s:gpio request failed\n", __func__);
		return ret;
	}

	/* enable pull-down for MDM_COLD_BOOT */
	pctl_dev = tegra_get_pinctrl_device_handle();
	if (pctl_dev) {
		unsigned long config;

		config = TEGRA_PINCONF_PACK(TEGRA_PINCONF_PARAM_PULL,
					TEGRA_PIN_PULL_DOWN);
		ret = pinctrl_set_config_for_group_name(pctl_dev,
				"ulpi_data4_po5", config);
		if (ret < 0) {
			pr_err("ERROR: %s(): ulpi_data4 config failed: %d\n",
					__func__, ret);
			return ret;
		}
	} else {
		pr_err("ERROR: %s(): No Tegra pincontrol driver\n", __func__);
		return -EINVAL;
	}

	/* Release modem reset to start boot */
	gpio_set_value(MDM_RST, 1);

	/* export GPIO for user space access through sysfs */
	gpio_export(MDM_RST, false);
	gpio_export(MDM_SAR0, false);

	return 0;
}

static const struct tegra_modem_operations baseband_operations = {
	.init = baseband_init,
};

static struct tegra_usb_modem_power_platform_data baseband_pdata = {
	.ops = &baseband_operations,
	.regulator_name = "vdd_wwan_mdm",
	.wake_gpio = -1,
	.boot_gpio = MDM_COLDBOOT,
	.boot_irq_flags = IRQF_TRIGGER_RISING |
				    IRQF_TRIGGER_FALLING |
				    IRQF_ONESHOT,
	.autosuspend_delay = 2000,
	.short_autosuspend_delay = 50,
	.tegra_ehci_device = &tegra_ehci2_device,
	.tegra_ehci_pdata = &tegra_ehci2_hsic_baseband_pdata,
	.mdm_power_report_gpio = MDM_POWER_REPORT,
	.mdm_power_irq_flags = IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
};

static struct platform_device icera_bruce_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

static void t210ref_modem_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct board_info board_info;
	struct board_info pmu_board_info;
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	tegra_get_board_info(&board_info);
	tegra_get_pmu_board_info(&pmu_board_info);
	pr_info("%s: modem_id = %d\n", __func__, modem_id);

	switch (modem_id) {
	case TEGRA_BB_BRUCE:
		if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)) {
			/* Set specific USB wake source for T210ref */
			if (board_info.board_id == BOARD_E1780)
				tegra_set_wake_source(42, INT_USB2);
			if (pmu_board_info.board_id == BOARD_E1736 ||
				pmu_board_info.board_id == BOARD_E1769 ||
				pmu_board_info.board_id == BOARD_E1936)
				baseband_pdata.regulator_name = NULL;
			platform_device_register(&icera_bruce_device);
		}
		break;
	case TEGRA_BB_HSIC_HUB: /* HSIC hub */
		if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)) {
			tegra_ehci2_device.dev.platform_data =
				&tegra_ehci2_hsic_smsc_hub_pdata;
			/* Set specific USB wake source for T210ref */
			if (board_info.board_id == BOARD_E1780)
				tegra_set_wake_source(42, INT_USB2);
			platform_device_register(&tegra_ehci2_device);
		} else
			xusb_pdata.pretend_connect_0 = true;
		break;
	default:
		return;
	}
}

#ifdef CONFIG_USE_OF
static struct of_dev_auxdata t210ref_auxdata_lookup[] __initdata = {
	T124_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-se", 0x70012000, "tegra12-se", NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-dtv", 0x7000c300, "dtv", NULL),
#if defined(CONFIG_ARM64)
	OF_DEV_AUXDATA("nvidia,tegra132-udc", 0x7d000000, "tegra-udc.0",
			&tegra_udc_pdata.u_data.dev),
	OF_DEV_AUXDATA("nvidia,tegra132-otg", 0x7d000000, "tegra-otg",
			&tegra_otg_pdata),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
		"gk20a.0", NULL),
#ifdef CONFIG_ARCH_TEGRA_VIC
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
		NULL),
#ifdef CONFIG_VI_ONE_DEVICE
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi", NULL),
#else
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	T124_UART_OF_DEV_AUXDATA,
	T124_I2C_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci",
				&xusb_pdata),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dfll", 0x70110000, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-dfll", 0x70040084, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-camera", 0, "pcl-generic",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-ahci-sata", 0x70027000, "tegra-sata.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra-bluedroid_pm", 0, "bluedroid_pm",
		NULL),
	OF_DEV_AUXDATA("pwm-backlight", 0, "pwm-backlight", NULL),
	{}
};
#endif


static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern2",    "pll_p",        41000000,       false},
	{ "clk_out_2",  "extern2",      40800000,       false},
	{ NULL,         NULL,           0,              0},
};

static struct rm_spi_ts_platform_data rm31080ts_e2141_data = {
	.gpio_reset = E2141_TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_T008,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 16,
};

static struct spi_board_info rm31080a_e2141_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = E2141_TOUCH_SPI_ID,
		.chip_select = TOUCH_SPI_CS,
		.max_speed_hz = 12 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_e2141_data,
	},
};

static int __init t210ref_touch_init(void)
{
	pr_info("%s init raydium touch\n", __func__);
	tegra_clk_init_from_table(touch_clk_init_table);
	rm31080a_e2141_spi_board[0].irq =
		gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);

	touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
		E2141_TOUCH_GPIO_RST_RAYDIUM_SPI,
		&rm31080ts_e2141_data,
		&rm31080a_e2141_spi_board[0],
		ARRAY_SIZE(rm31080a_e2141_spi_board));
	return 0;
}


static void __init tegra_t210ref_early_init(void)
{
	tegra_clk_init_from_table(t210ref_clk_init_table);
	tegra_clk_verify_parents();
	if (of_machine_is_compatible("nvidia,e2141"))
		tegra_soc_device_init("e2141");
}

static struct tegra_io_dpd pexbias_io = {
	.name			= "PEX_BIAS",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 4,
};
static struct tegra_io_dpd pexclk1_io = {
	.name			= "PEX_CLK1",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 5,
};
static struct tegra_io_dpd pexclk2_io = {
	.name			= "PEX_CLK2",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 6,
};

static void __init tegra_t210ref_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);

#ifndef CONFIG_MACH_EXUMA
	t210ref_display_init();
#endif
	e2141_uart_init();
	t210ref_usb_init();
	t210ref_modem_init();
#ifdef CONFIG_TEGRA_XUSB_PLATFORM
	t210ref_xusb_init();
#endif
	t210ref_i2c_init();
	t210ref_audio_init();
	platform_add_devices(t210ref_devices, ARRAY_SIZE(t210ref_devices));
	platform_device_register(&t210ref_audio_device_rt5639);
	tegra_io_dpd_init();
	t210ref_sdhci_init();
	t210ref_suspend_init();

	tegra12_emc_init();
	t210ref_edp_init();
	isomgr_init();
	t210ref_touch_init();
	t210ref_panel_init();

		/* put PEX pads into DPD mode to save additional power */
	tegra_io_dpd_enable(&pexbias_io);
	tegra_io_dpd_enable(&pexclk1_io);
	tegra_io_dpd_enable(&pexclk2_io);

#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif


	t210ref_sensors_init();
	t210ref_soctherm_init();
}

static void __init tegra_t210ref_init_early(void)
{
	t210ref_rail_alignment_init();
	tegra12x_init_early();
}

static int tegra_ardbeg_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct device *dev = data;

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
	.notifier_call = tegra_ardbeg_notifier_call,
};
static void __init tegra_t210ref_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);
	bus_register_notifier(&platform_bus_type, &platform_nb);

	tegra_t210ref_early_init();
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	carveout_linear_set(&tegra_vpr_cma_dev);
#endif
#ifdef CONFIG_USE_OF
	t210ref_camera_auxdata(t210ref_auxdata_lookup);
	of_platform_populate(NULL,
		of_default_bus_match_table, t210ref_auxdata_lookup,
		&platform_bus);
#endif

	tegra_t210ref_late_init();
}

static void __init tegra_t210ref_reserve(void)
{
#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	ulong tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM) || \
		defined(CONFIG_TEGRA_NO_CARVEOUT)
	ulong carveout_size = 0;
	ulong fb2_size = SZ_16M;
#else
	ulong carveout_size = SZ_1G;
	ulong fb2_size = SZ_4M;
#endif
	ulong fb1_size = SZ_16M + SZ_2M;
	ulong vpr_size = 186 * SZ_1M;

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* support FBcon on 4K monitors */
	fb2_size = SZ_64M + SZ_8M;	/* 4096*2160*4*2 = 70778880 bytes */
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	tmp = fb1_size;
	fb1_size = fb2_size;
	fb2_size = tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

	tegra_reserve4(carveout_size, fb1_size, fb2_size, vpr_size);
}

static const char * const e2141_dt_board_compat[] = {
	"nvidia,e2141",
	NULL
};

DT_MACHINE_START(E2141, "e2141")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_t210ref_reserve,
	.init_early	= tegra_t210ref_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_t210ref_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= e2141_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END
