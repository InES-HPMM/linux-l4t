/*
 * arch/arm/mach-tegra/vcm30_t124.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * This file is intended to hold all code related to vcm30_t124 MCM
 *
 * Broadly, it contains Clock POR values, internal flash and temparature
 * sensor etc
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/platform_data/tegra_nor.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/tegra-pmc.h>
#include <linux/pid_thermal_gov.h>
#include <linux/tegra_soctherm.h>
#include <asm/io.h>
#include "board.h"
#include "board-common.h"
#include <linux/platform/tegra/clock.h>
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "therm-monitor.h"
#include "tegra-of-dev-auxdata.h"
#include "vcm30_t124.h"

/*
 * Set clock values as per automotive POR for VCM30T124
 *
 * If the clock POR values are different for a board, eg for i2s, those
 * would be hanled in the board files
 *
 * Use this table to initialize set of clocks during boot up with specified
 * rate. If enabled, they will remain ON throughout.
 *
 * Add clocks here if they've attributes different from the ones in common
 * clock table (mach-tegra/common.c). Parent=NULL implies the parent will not
 * be changed by this table; its previous value will be retained.
 * "rate" specifies only the initial rate for this clock.
 *
 * System busses should have clock always on, while individual controllers
 * clocks can be enabled/disabled by the controller drivers.
 */
static __initdata struct tegra_clk_init_table vcm30_t124_clk_init_table[] = {
	/* name			parent		rate	enabled (always on)*/

	{ "automotive.sclk",	NULL,		282000000,	true},
	{ "automotive.hclk",	NULL,		282000000,	true},
	{ "automotive.pclk",	NULL,		141000000,	true},

	{ "mselect",		"pll_p",	204000000,	true},
	{ "automotive.mselect",	NULL,		204000000,	true},

	{ "se.cbus",		NULL,		372000000,	false},
	{ "msenc.cbus",		NULL,		372000000,	false},
	{ "vde",		"pll_c3",	450000000,	false},

	{ "vic03.cbus",		NULL,		564000000,      false},
	{ "tsec.cbus",		NULL,		564000000,      false},
	{ "tsec",		"pll_c",	564000000,	false},
	{ "vic03",		"pll_c",	564000000,	false},

	{ "vi.c4bus",		NULL,		600000000,      false},
	{ "isp.c4bus",		NULL,		600000000,      false},
	{ "vi",			"pll_c4",	600000000,	false},
	{ "isp",		"pll_c4",	600000000,	false},

	{ "pll_d_out0",		"pll_d",	474000000,	true},
	{ "disp2",		"pll_d_out0",	474000000,	false},
	{ "disp1",		"pll_d_out0",	474000000,	false},

	{ "pll_d2",		NULL,		297000000,	true},
	{ "hdmi",		"pll_d2",	297000000,	false},

	{ "pll_a",		"pll_p_out1",	368640000,	true},
	{ "pll_a_out0",		"pll_a",	24576000,	true},

	{ "dam0",		"pll_p",	19900000,	false},
	{ "dam1",		"pll_p",	19900000,	false},
	{ "dam2",		"pll_p",	19900000,	false},
	{ "adx",		"pll_a_out0",	4096000,	false},
	{ "adx1",		"pll_a_out0",	4096000,	false},
	{ "amx",		"pll_a_out0",	4096000,	false},
	{ "amx1",		"pll_a_out0",	4096000,	false},

	{ "spdif_out",		"pll_a_out0",	6144000,	false},
	{ "spdif_in",		"pll_p",	48000000,	false},
	{ "hda",		"pll_p",	48000000,	false},
	{ "cilab",		"pll_p",	102000000,	false},
	{ "cilcd",		"pll_p",	102000000,	false},
	{ "cile",		"pll_p",	102000000,	false},

	{ "nor",		"pll_p",	102000000,	false},

	{ "sbc1",		"pll_p",	25000000,	false},
	{ "sbc2",		"pll_p",	25000000,	false},
	{ "sbc3",		"pll_p",	25000000,	false},
	{ "sbc4",		"pll_p",	25000000,	false},
	{ "sbc5",		"pll_p",	25000000,	false},
	{ "sbc6",		"pll_p",	25000000,	false},

	{ "uarta",		"pll_p",	408000000,	false},
	{ "uartb",		"pll_p",	408000000,	false},
	{ "uartc",		"pll_p",	408000000,	false},
	{ "uartd",		"pll_p",	408000000,	false},

	{ "vi_sensor",		"pll_p",	68000000,	false},
	{ "vi_sensor2",		"pll_p",	68000000,	false},

	{ "automotive.host1x",	NULL,		282000000,	true},

	{ "i2c1",		"pll_p",	408000000,	false},
	{ "i2c2",		"pll_p",	408000000,	false},
	{ "i2c3",		"pll_p",	408000000,	false},
	{ "i2c4",		"pll_p",	408000000,	false},
	{ "i2c5",		"pll_p",	408000000,	false},
	{ "i2c6",		"pll_p",	408000000,	false},

	{ "sdmmc2",		"pll_p",	48000000,	false},
	{"gk20a.gbus",		NULL,		600000000,	false},

	{ NULL,			NULL,		0,		0},
};

#define SET_FIXED_TARGET_RATE(clk_name, fixed_target_rate) \
	{clk_name,	NULL,	fixed_target_rate,	false}

/*
 * FIXME: Need to revisit for following clocks:
 * csi, dsi, dsilp, audio
 */

/*
 * Table of clocks that have fixed, characterized rates. Entries in this table
 * are orthagonal to the above table, but the "rate" set in the above table
 * will be un-used if a clock is made FIXED via this table.
 *
 * Parent and enable fields cannot be set thru' this table.
 */

static  __initdata struct tegra_clk_init_table
				vcm30t124_fixed_target_clk_table[] = {

	/*			name,		fixed target rate*/
	SET_FIXED_TARGET_RATE("pll_m",		792000000),
	SET_FIXED_TARGET_RATE("sbus",		282000000),

#ifdef CONFIG_TEGRA_PLLCX_FIXED
#ifdef CONFIG_TEGRA_DUAL_CBUS
	SET_FIXED_TARGET_RATE("pll_c2",		372000000),
	SET_FIXED_TARGET_RATE("c2bus",		372000000),
	SET_FIXED_TARGET_RATE("pll_c3",		450000000),
	SET_FIXED_TARGET_RATE("c3bus",		450000000),
#endif
	SET_FIXED_TARGET_RATE("pll_c",		564000000),
	SET_FIXED_TARGET_RATE("pll_c4",		600000000),
	SET_FIXED_TARGET_RATE("c4bus",		600000000),
	SET_FIXED_TARGET_RATE("pll_c_out1",	282000000),
#endif
	SET_FIXED_TARGET_RATE("pll_p",		408000000),

	SET_FIXED_TARGET_RATE("sclk",		282000000),
	SET_FIXED_TARGET_RATE("hclk",		282000000),
	SET_FIXED_TARGET_RATE("ahb.sclk",	282000000),
	SET_FIXED_TARGET_RATE("pclk",		141000000),
	SET_FIXED_TARGET_RATE("apb.sclk",	141000000),
	SET_FIXED_TARGET_RATE("cpu_lp",		948000000),

	SET_FIXED_TARGET_RATE("vde",		450000000),
	SET_FIXED_TARGET_RATE("se",		372000000),
	SET_FIXED_TARGET_RATE("msenc",		372000000),

	SET_FIXED_TARGET_RATE("tsec",		564000000),
	SET_FIXED_TARGET_RATE("vic03",		564000000),

	SET_FIXED_TARGET_RATE("vi",		600000000),
	SET_FIXED_TARGET_RATE("isp",		600000000),

	SET_FIXED_TARGET_RATE("host1x",		282000000),
	SET_FIXED_TARGET_RATE("mselect",	204000000),

	SET_FIXED_TARGET_RATE("pll_a_out0",	24576000),
	SET_FIXED_TARGET_RATE("spdif_in",	48000000),
	SET_FIXED_TARGET_RATE("spdif_out",	6144000),
	SET_FIXED_TARGET_RATE("d_audio",	24576000),
	SET_FIXED_TARGET_RATE("adx",		4096000),
	SET_FIXED_TARGET_RATE("adx1",		4096000),
	SET_FIXED_TARGET_RATE("amx",		4096000),
	SET_FIXED_TARGET_RATE("amx1",		4096000),

	SET_FIXED_TARGET_RATE("cilab",		102000000),
	SET_FIXED_TARGET_RATE("cilcd",		102000000),
	SET_FIXED_TARGET_RATE("cile",		102000000),

	SET_FIXED_TARGET_RATE(NULL,		0),
};

/* Therm Monitor */

/*
 *  Padcontrol registers which need to modified
 *  based on local temperature changes.
 *  Format ex:
 *  {
 *    .reg_addr = 0x000008ec, --  only Offset
 *    .temperat = {10000, 85000, INT_MAX}, -- in Milli Deg C, Maximum 9 values
 *    .value    = {0xf101a000, 0xf161d000}, -- Maximum 9 values
 *  },
 *
 *  If local temperature is  < 10000  Milli Deg C then value "0xf101a000"
 *  will be written and if between 10000 to 85000 then value "0xf161d000"
 *  will be wriiten and so on.
 */
struct therm_monitor_ldep_data ltemp_reg_data[] = {
	{
		.reg_addr = INVALID_ADDR,
	},
};

#ifdef CONFIG_SENSORS_TMON_TMP411
static struct therm_monitor_data vcm30t124_therm_monitor_data = {
	.brd_ltemp_reg_data = NULL,
	.delta_temp = 4000,
	.delta_time = 2000,
	.remote_offset = 8000,
	.alert_gpio = TEGRA_GPIO_PI6,
	.local_temp_update = false,

	/* USB regsiter update not requred */
	.utmip_reg_update = false,

	.i2c_bus_num = I2C_BUS_TMP411,
	.i2c_dev_addrs = I2C_ADDR_TMP411,
	.i2c_dev_name = "tmon-tmp411-sensor",
};
#endif

void __init tegra_vcm30_t124_therm_mon_init(void)
{
#ifdef CONFIG_SENSORS_TMON_TMP411
	register_therm_monitor(&vcm30t124_therm_monitor_data);
#endif
}

static struct tegra_usb_platform_data tegra_udc_pdata = {
#if defined(CONFIG_USB_TEGRA_OTG)
	.port_otg = true,
#else
	.port_otg = false,
#endif
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 63,
		.xcvr_setup_offset = 6,
		.xcvr_use_fuses = 1,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_use_lsb = 1,
	},
};

/* USB/UDC/OTG devices */
static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
#if defined(CONFIG_USB_TEGRA_OTG)
	.port_otg = true,
	.id_det_type = TEGRA_USB_VIRTUAL_ID,
#else
	.port_otg = false,
#endif
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
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


static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
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
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
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

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

void __init tegra_vcm30_t124_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_pdata.is_xhci = false;
		tegra_udc_pdata.u_data.dev.is_xhci = false;

#if defined(CONFIG_USB_TEGRA_OTG)
		/* register OTG device */
		tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
		platform_device_register(&tegra_otg_device);

		/* Setup the udc platform data */
		tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
		platform_device_register(&tegra_udc_device);
#else
		/* register host mode */
		tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;
		platform_device_register(&tegra_ehci1_device);
#endif
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
		platform_device_register(&tegra_ehci2_device);
	}

	if (!(usb_port_owner_info & UTMI3_PORT_OWNER_XUSB)) {
		tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
		platform_device_register(&tegra_ehci3_device);
	}
}

#ifdef CONFIG_USE_OF
/*
 * This table is used to populate the device name which is necessary to
 * make the clock framework to work.
 * eg the dt device name "nvidia,tegra124-vi" is modified to "vi.0" for
 * clk_get and clk_enable to work properly
 */
static struct of_dev_auxdata tegra_vcm30_t124_auxdata_lookup[] __initdata = {
	T124_UART_OF_DEV_AUXDATA,
	T124_I2C_OF_DEV_AUXDATA,
	T124_SPI_OF_DEV_AUXDATA,

	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci", NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-nvavp", 0x60001000, "nvavp", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	OF_DEV_AUXDATA("nvidia,tegra124-se", TEGRA_SE_BASE, "tegra12-se", NULL),
#endif

	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
			"gk20a.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahub", 0x70300000, "tegra30-ahub-apbif",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-ahub-master", 0x70300000,
				"tegra30-ahub-apbif", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-ahub-slave", 0x70300000,
				"tegra124-virt-ahub-slave", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
			NULL),
#ifdef CONFIG_SATA_AHCI_TEGRA
	OF_DEV_AUXDATA("nvidia,tegra124-ahci-sata", TEGRA_SATA_BAR5_BASE,
			"tegra-sata", NULL),
#endif

	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC1_BASE,
			"sdhci-tegra.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC2_BASE,
			"sdhci-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC3_BASE,
			"sdhci-tegra.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-sdhci", TEGRA_SDMMC4_BASE,
			"sdhci-tegra.3", NULL),

	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
		NULL),
	OF_DEV_AUXDATA("pwm-backlight", 0, "pwm-backlight", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nor", TEGRA_SNOR_BASE, "tegra-nor",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dfll", 0x70110000, "tegra_cl_dvfs",
		 NULL),
	{}
};

void __init tegra_vcm30_t124_populate_auxdata(void)
{
	of_platform_populate(NULL, of_default_bus_match_table,
			tegra_vcm30_t124_auxdata_lookup, &platform_bus);
}
#endif

/* Soc Therm */
static struct pid_thermal_gov_params soctherm_pid_params = {
	.max_err_temp = 9000,
	.max_err_gain = 1000,
	.gain_p = 1000,
	.gain_d = 0,
	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &soctherm_pid_params,
};

static struct soctherm_platform_data vcm30_t124_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 0,
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 0,
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
		},
	},
};

static const struct soctherm_therm vcm30t124_therm[] = {
};

int __init tegra_vcm30_t124_soctherm_init(void)
{
	struct soctherm_therm *therm;

	therm = &vcm30_t124_soctherm_data.therm[THERM_CPU];
	tegra_add_cpu_clk_switch_trips(therm->trips, &therm->num_trips);

	therm = &vcm30_t124_soctherm_data.therm[THERM_GPU];
	tegra_add_tgpu_trips(therm->trips, &therm->num_trips);

	return tegra_soctherm_init(&vcm30_t124_soctherm_data);
}

/* Set POR value for all clocks given in the table */
void __init tegra_vcm30_t124_set_fixed_rate(struct tegra_clk_init_table *table)
{
	struct clk *c;
	unsigned long flags;

	for (; table->name; table++) {
		c = tegra_get_clock_by_name(table->name);
		if (c) {
			clk_lock_save(c, &flags);
			c->fixed_target_rate = table->rate;
			clk_unlock_restore(c, &flags);
		} else {
			pr_warn("%s: Clock %s not found\n", __func__, table->name);
			BUG_ON(1);
		}
	}
}

int __init tegra_vcm30_t124_early_init(void)
{
	tegra_clk_init_from_table(vcm30_t124_clk_init_table);
	tegra_vcm30_t124_set_fixed_rate(vcm30t124_fixed_target_clk_table);

	return 0;
}

void __init tegra_vcm30_t124_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 2560*1440*4*2 = 29491200 = 28.125*1M = 29M bytes */
	tegra_reserve4(0, 29 * SZ_1M, 29 * SZ_1M, 64 * SZ_1M);
#else
	tegra_reserve4(SZ_128M, SZ_16M + SZ_2M, SZ_16M + SZ_2M, 64 * SZ_1M);
#endif
}

/* VCM30T124 suspend init */
static struct tegra_suspend_platform_data vcm30_t124_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 2000,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0xfefe,
	.core_off_timer = 2000,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 1000,
};

int __init tegra_vcm30_t124_suspend_init(void)
{
	tegra_init_suspend(&vcm30_t124_suspend_data);
	return 0;
}
