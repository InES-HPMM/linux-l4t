/*
 * arch/arm/mach-tegra/board-loki.c
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform_data/tegra_c2port_platform_data.h>
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
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
#include <linux/tegra-soc.h>
#include <linux/tegra-powergate.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/edp.h>
#include <linux/mfd/palmas.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/clk/tegra.h>
#include <linux/clocksource.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <linux/irqchip.h>
#include <linux/irqchip/tegra.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>


#include <mach/irqs.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <linux/platform/tegra/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/system_info.h>
#include <mach/xusb.h>

#include "board-touch-raydium.h"
#include "board.h"
#include "board-common.h"
#include <linux/platform/tegra/clock.h>
#include "board-loki.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include <linux/platform/tegra/common.h>
#include "tegra-board-id.h"
#include "iomap.h"
#include "tegra-of-dev-auxdata.h"

static struct board_info board_info, display_board_info;

static struct resource loki_bluedroid_pm_resources[] = {
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
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
	[4] = {
		.name = "reset_gpio",
		.start  = TEGRA_GPIO_PQ6,
		.end    = TEGRA_GPIO_PQ6,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device loki_bluedroid_pm_device = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(loki_bluedroid_pm_resources),
	.resource       = loki_bluedroid_pm_resources,
};

static noinline void __init loki_setup_bluedroid_pm(void)
{
	loki_bluedroid_pm_resources[1].start =
		loki_bluedroid_pm_resources[1].end =
				gpio_to_irq(TEGRA_GPIO_PU0);
	platform_device_register(&loki_bluedroid_pm_device);
}

static struct i2c_board_info __initdata rt5639_board_info = {
	I2C_BOARD_INFO("rt5639", 0x1c),
};

static __initdata struct tegra_clk_init_table loki_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	48000000,	false},
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

static void loki_i2c_init(void)
{
	i2c_register_board_info(0, &rt5639_board_info, 1);
}

static struct tegra_asoc_platform_data loki_audio_pdata_rt5639 = {
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
		.sample_size   = 16,
		.channels      = 2,
		.bit_clk       = 1536000,
		.i2s_mode = TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BT_SCO] = {
		.audio_port_id = 3,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_DSP_A,
	},
};

static void loki_audio_init(void)
{
	loki_audio_pdata_rt5639.gpio_hp_det =
			TEGRA_GPIO_HP_DET;

	loki_audio_pdata_rt5639.gpio_hp_det_active_high = 0;

	loki_audio_pdata_rt5639.gpio_ldo1_en =
			TEGRA_GPIO_LDO_EN;

	loki_audio_pdata_rt5639.codec_name = "rt5639.0-001c";
	loki_audio_pdata_rt5639.codec_dai_name = "rt5639-aif1";
}

static struct platform_device loki_audio_device_rt5639 = {
	.name = "tegra-snd-rt5639",
	.id = 0,
	.dev = {
		.platform_data = &loki_audio_pdata_rt5639,
	},
};

static struct platform_device *loki_devices[] __initdata = {
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE) && !defined(CONFIG_USE_OF)
	&tegra12_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&loki_audio_device_rt5639,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_offload_device,
	&tegra30_avp_audio_device,
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.dcp_current_limit_ma = 2000,
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
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
		.turn_off_vbus_on_lp0 = true,
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
		.xcvr_use_fuses = 0,
		.vbus_oc_map = 0x4,
		.xcvr_hsslew_lsb = 2,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
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
	.unaligned_dma_buf_supported = false,
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
	{MDM_RST, GPIOF_OUT_INIT_LOW, "MODEM RESET"},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_baseband_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
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
	.unaligned_dma_buf_supported = false,
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

static void loki_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();
	int modem_id = tegra_get_modem_id();
	int rc = 0;

	/* Device cable is detected through PMU Interrupt */
	tegra_udc_pdata.support_pmu_vbus = true;
	tegra_udc_pdata.vbus_extcon_dev_name = "palmas-extcon";
	tegra_ehci1_utmi_pdata.support_pmu_vbus = true;
	tegra_ehci1_utmi_pdata.vbus_extcon_dev_name = "palmas-extcon";

	if (board_info.board_id == BOARD_P2530 &&
		board_info.sku == BOARD_SKU_FOSTER &&
		board_info.fab >= 0xC0) {
		rc = gpio_request(TEGRA_GPIO_PK5, "r8152_rst");
		if (rc)
			pr_warn("RTL8152 gpio request failed:%d\n", rc);
		rc = gpio_direction_output(TEGRA_GPIO_PK5, 0);
		if (rc)
			pr_warn("RTL8152 gpio direction failed:%d\n", rc);
		rc = gpio_direction_output(TEGRA_GPIO_PK5, 1);
		if (rc)
			pr_warn("RTL8152 gpio direction failed:%d\n", rc);
	}

	/* Enable Y-Cable support */
	tegra_ehci1_utmi_pdata.u_data.host.support_y_cable = true;

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
		platform_device_register(&tegra_otg_device);
		/* Setup the udc platform data */
		tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	}
	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		if (!modem_id) {
			tegra_ehci2_device.dev.platform_data =
				&tegra_ehci2_utmi_pdata;
			platform_device_register(&tegra_ehci2_device);
		}
	}
	if (!(usb_port_owner_info & UTMI3_PORT_OWNER_XUSB)) {
		tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
		platform_device_register(&tegra_ehci3_device);
	}
}

static struct tegra_xusb_platform_data xusb_pdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P0 | TEGRA_XUSB_SS_P1 |
			TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_USB2_P2,
};

static void loki_xusb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	xusb_pdata.lane_owner = (u8) tegra_get_lane_owner_info();

	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM363) {
		/* Laguna */
		if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
			xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0 |
				TEGRA_XUSB_SS_P0);

		if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
			xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P1 |
				TEGRA_XUSB_SS_P1);

		/* FIXME Add for UTMIP2 when have odmdata assigend */
	} else {
		/* Loki */
		if (board_info.board_id == BOARD_E1781) {
			pr_info("Shield ERS-S. 0x%x\n", board_info.board_id);
			/* Shield ERS-S */
			if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0);

			if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(
					TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_SS_P0 |
					TEGRA_XUSB_USB2_P2 | TEGRA_XUSB_SS_P1);
		} else if (board_info.board_id == BOARD_P2530 &&
					board_info.sku == BOARD_SKU_FOSTER) {
			if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0);

			if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P1 |
					TEGRA_XUSB_SS_P0);

			if (!(usb_port_owner_info & UTMI3_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P2 |
					TEGRA_XUSB_SS_P1);
		} else {
			pr_info("Shield ERS 0x%x\n", board_info.board_id);
			/* Shield ERS */

			if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0 |
					TEGRA_XUSB_SS_P0);

			if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P1 |
					TEGRA_XUSB_USB2_P2 | TEGRA_XUSB_SS_P1);
		}
		/* FIXME Add for UTMIP2 when have odmdata assigend */
	}
}

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

	/* export GPIO for user space access through sysfs */
	gpio_export(MDM_RST, false);

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
	.tegra_ehci_device = &tegra_ehci2_device,
	.tegra_ehci_pdata = &tegra_ehci2_hsic_baseband_pdata,
	.mdm_power_report_gpio = -1,
};

static struct platform_device icera_bruce_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

static void loki_modem_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct board_info board_info;
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	tegra_get_board_info(&board_info);
	pr_info("%s: modem_id = %d\n", __func__, modem_id);

	switch (modem_id) {
	case TEGRA_BB_BRUCE:
		if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB))
			platform_device_register(&icera_bruce_device);
		break;
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

#ifdef CONFIG_USE_OF
struct of_dev_auxdata loki_auxdata_lookup[] __initdata = {
	T124_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-se", 0x70012000, "tegra12-se", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
		"gk20a.0", NULL),
#ifdef CONFIG_ARCH_TEGRA_VIC
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci",
				&xusb_pdata),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-camera", 0, "pcl-generic",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahci-sata", 0x70027000, "tegra-sata.0",
		NULL),
#ifdef CONFIG_TEGRA_CEC_SUPPORT
	OF_DEV_AUXDATA("nvidia,tegra124-cec", 0x70015000, "tegra_cec", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra30-hda", 0x70030000, "tegra30-hda", NULL),
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

struct rm_spi_ts_platform_data rm31080ts_loki_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_L005,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

struct rm_spi_ts_platform_data rm31080ts_loki_data_t_1_95 = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_L005,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
	.gpio_sensor_select0 = true,
	.gpio_sensor_select1 = false,
};

struct rm_spi_ts_platform_data rm31080ts_loki_data_jdi_5 = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_L005,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
	.gpio_sensor_select0 = false,
	.gpio_sensor_select1 = true,
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 16,
};

struct spi_board_info rm31080a_loki_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = TOUCH_SPI_ID,
		.chip_select = TOUCH_SPI_CS,
		.max_speed_hz = 12 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_loki_data,
	},
};

static int __init loki_touch_init(void)
{
	struct board_info bi;
	tegra_get_board_info(&bi);
	if (bi.board_id == BOARD_P2530 && bi.sku == BOARD_SKU_FOSTER)
		return 0;

	if (tegra_get_touch_panel_id() == TOUCHPANEL_THOR_WINTEK)
		rm31080a_loki_spi_board[0].platform_data =
					&rm31080ts_loki_data_t_1_95;
	else if (tegra_get_touch_panel_id() == TOUCHPANEL_LOKI_JDI5)
		rm31080a_loki_spi_board[0].platform_data =
					&rm31080ts_loki_data_jdi_5;
	/*
	** remove touch clock initialization for ffd fab a3, higher
	** Move clock from tegra clock to external xtal
	*/
	if (bi.board_id == BOARD_P2530 && bi.fab >= 0xa3) {
		rm31080ts_loki_data.name_of_clock = NULL;
		rm31080ts_loki_data.name_of_clock_con = NULL;
	} else
		tegra_clk_init_from_table(touch_clk_init_table);
	rm31080a_loki_spi_board[0].irq =
		gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
	touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
				TOUCH_GPIO_RST_RAYDIUM_SPI,
				&rm31080ts_loki_data,
				&rm31080a_loki_spi_board[0],
				ARRAY_SIZE(rm31080a_loki_spi_board));
	return 0;
}

static void __init loki_revision_init(struct board_info *binf)
{

	struct board_info disp_board_info;

	system_rev = P2530_LOKI;
	tegra_get_display_board_info(&disp_board_info);
	if (!binf)
		return;
	if (binf->board_id == BOARD_E2548) {
		system_rev = E2548;
	} else if (binf->board_id == BOARD_E2549) {
		system_rev = E2549;
	} else if (binf->board_id == BOARD_P2530) {
		if (binf->sku == BOARD_SKU_FOSTER)
			system_rev = P2530_FOSTER;
		else if (disp_board_info.fab == 0x1)
			system_rev = P2530_LOKI_PREM;
	}
}

static void __init tegra_loki_early_init(void)
{
	tegra_clk_init_from_table(loki_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("loki");
}

#ifdef CONFIG_C2PORT_LOKI
/* Init mcu debugger for Loki */
static struct tegra_c2port_platform_data mcu_init_data = {
	.gpio_c2ck = TEGRA_GPIO_PK0,
	.gpio_c2d = TEGRA_GPIO_PS0
};
static struct platform_device tegra_loki_mcu_debugger = {
		.name = "tegra_c2port",
		.id = 0,
		.dev = {
			.platform_data = &mcu_init_data,
		}
};
static void __init tegra_loki_mcu_debugger_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if (board_info.fab >= 0xa1)
		mcu_init_data.gpio_c2ck = TEGRA_GPIO_PQ7;
	platform_device_register(&tegra_loki_mcu_debugger);
}
#endif

static void __init tegra_loki_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);
	loki_revision_init(&board_info);
	loki_usb_init();
	loki_modem_init();
	loki_xusb_init();
	loki_i2c_init();
	loki_audio_init();
	platform_add_devices(loki_devices, ARRAY_SIZE(loki_devices));
	tegra_io_dpd_init();
	loki_sdhci_init();
	loki_regulator_init();
	loki_suspend_init();
	loki_emc_init();
	isomgr_init();
	loki_touch_init();
	loki_panel_init();
	loki_kbc_init();
	loki_sensors_init();

	loki_soctherm_init();
	loki_setup_bluedroid_pm();
#ifdef CONFIG_C2PORT_LOKI
	tegra_loki_mcu_debugger_init();
#endif
}

static void __init tegra_loki_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	tegra_loki_early_init();
#ifdef CONFIG_USE_OF
	loki_camera_auxdata(loki_auxdata_lookup);
	of_platform_populate(NULL,
		of_default_bus_match_table, loki_auxdata_lookup,
		&platform_bus);
#endif

	tegra_loki_late_init();
}

static void __init tegra_loki_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM) || \
		defined(CONFIG_TEGRA_NO_CARVEOUT)
	/* 1920*1200*4*2 = 18432000 bytes */
	tegra_reserve4(0, SZ_16M + SZ_2M, SZ_16M, 186 * SZ_1M);
#else
	tegra_reserve4(SZ_1G, SZ_16M + SZ_2M, SZ_4M, 186 * SZ_1M);
#endif
}

static const char * const loki_dt_board_compat[] = {
	"nvidia,loki",
	NULL
};

static const char * const foster_dt_board_compat[] = {
	"nvidia,foster",
	NULL
};

static const char * const foster_hdd_dt_board_compat[] = {
	"nvidia,foster_hdd",
	NULL
};

static void __init tegra_loki_init_early(void)
{
	loki_rail_alignment_init();
	tegra12x_init_early();
}

DT_MACHINE_START(LOKI, "loki")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_loki_reserve,
	.init_early	= tegra_loki_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_loki_dt_init,
	.dt_compat	= loki_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END

DT_MACHINE_START(FOSTER, "foster")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_loki_reserve,
	.init_early	= tegra_loki_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_loki_dt_init,
	.dt_compat	= foster_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END

DT_MACHINE_START(FOSTER_HDD, "foster_hdd")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_loki_reserve,
	.init_early	= tegra_loki_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_loki_dt_init,
	.dt_compat	= foster_hdd_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END
