/*
 * arch/arm/mach-tegra/board-tn8-power.c
 *
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/palmas.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>

#include <linux/gpio.h>
#include <linux/power/bq2419x-charger.h>
#include <linux/power/power_supply_extcon.h>
#include <linux/max17048_battery.h>
#include <linux/tegra-soc.h>
#include <linux/generic_adc_thermal.h>

#include <mach/irqs.h>

#include <asm/mach-types.h>

#include "pm.h"
#include "board.h"
#include "tegra-board-id.h"
#include "board-common.h"
#include "board-ardbeg.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include "iomap.h"
#include "tegra-board-id.h"
#include "battery-ini-model-data.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

/* -40 to 125 degC */
static int tn8_batt_temperature_table[] = {
	259, 266, 272, 279, 286, 293, 301, 308,
	316, 324, 332, 340, 349, 358, 367, 376,
	386, 395, 405, 416, 426, 437, 448, 459,
	471, 483, 495, 508, 520, 533, 547, 561,
	575, 589, 604, 619, 634, 650, 666, 682,
	699, 716, 733, 751, 769, 787, 806, 825,
	845, 865, 885, 905, 926, 947, 969, 990,
	1013, 1035, 1058, 1081, 1104, 1127, 1151, 1175,
	1199, 1224, 1249, 1273, 1298, 1324, 1349, 1374,
	1400, 1426, 1451, 1477, 1503, 1529, 1554, 1580,
	1606, 1631, 1657, 1682, 1707, 1732, 1757, 1782,
	1807, 1831, 1855, 1878, 1902, 1925, 1948, 1970,
	1992, 2014, 2036, 2057, 2077, 2097, 2117, 2136,
	2155, 2174, 2192, 2209, 2227, 2243, 2259, 2275,
	2291, 2305, 2320, 2334, 2347, 2361, 2373, 2386,
	2397, 2409, 2420, 2430, 2441, 2450, 2460, 2469,
	2478, 2486, 2494, 2502, 2509, 2516, 2523, 2529,
	2535, 2541, 2547, 2552, 2557, 2562, 2567, 2571,
	2575, 2579, 2583, 2587, 2590, 2593, 2596, 2599,
	2602, 2605, 2607, 2609, 2611, 2614, 2615, 2617,
	2619, 2621, 2622, 2624, 2625, 2626,
};

/* BQ2419X VBUS regulator */
static struct regulator_consumer_supply bq2419x_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-otg"),
};

static struct regulator_consumer_supply bq2419x_batt_supply[] = {
	REGULATOR_SUPPLY("usb_bat_chg", "tegra-udc.0"),
};

static struct bq2419x_vbus_platform_data bq2419x_vbus_pdata = {
	.gpio_otg_iusb = TEGRA_GPIO_PI4,
	.num_consumer_supplies = ARRAY_SIZE(bq2419x_vbus_supply),
	.consumer_supplies = bq2419x_vbus_supply,
};

static struct bq2419x_charger_platform_data bq2419x_charger_pdata = {
	.max_charge_current_mA = 3000,
	.charging_term_current_mA = 100,
	.consumer_supplies = bq2419x_batt_supply,
	.num_consumer_supplies = ARRAY_SIZE(bq2419x_batt_supply),
	.wdt_timeout    = 40,
	.rtc_alarm_time = 3600,
	.chg_restart_time = 1800,
};

struct bq2419x_platform_data tn8_bq2419x_pdata = {
	.vbus_pdata = &bq2419x_vbus_pdata,
};

static struct i2c_board_info __initdata bq2419x_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq2419x", 0x6b),
		.platform_data = &tn8_bq2419x_pdata,
	},
};


static struct regulator_consumer_supply palmas_smps123_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply palmas_smps45_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};

static struct regulator_consumer_supply palmas_smps6_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr_mclk", NULL),
	REGULATOR_SUPPLY("vddio_ddr3", NULL),
	REGULATOR_SUPPLY("vcore1_ddr3", NULL),
};

static struct regulator_consumer_supply palmas_smps7_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply palmas_smps8_supply[] = {
	REGULATOR_SUPPLY("dbvdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("dbvdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("avdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("avdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("dmicvdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("dmicvdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sys_2", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-xhci"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vdd_dtv", NULL),
	REGULATOR_SUPPLY("vdd_1v8_eeprom", NULL),
	REGULATOR_SUPPLY("vddio_cam", "tegra_camera"),
	REGULATOR_SUPPLY("vddio_cam", "vi"),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("dvdd", "spi0.0"),
	REGULATOR_SUPPLY("vlogic", "0-0069"),
	REGULATOR_SUPPLY("vid", "0-000d"),
	REGULATOR_SUPPLY("vddio", "0-0077"),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_lcd_1v8_s", NULL),
};

static struct regulator_consumer_supply palmas_smps9_supply[] = {
	REGULATOR_SUPPLY("vdd_snsr", NULL),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("vdd_3v3_gps", NULL),
	REGULATOR_SUPPLY("vdd_3v3_nfc", NULL),
	REGULATOR_SUPPLY("vdd", "0-0069"),
	REGULATOR_SUPPLY("vdd", "0-000d"),
	REGULATOR_SUPPLY("vdd", "0-0077"),
	REGULATOR_SUPPLY("vdd", "1-004c"),
	REGULATOR_SUPPLY("vdd", "1-004d"),
};

static struct regulator_consumer_supply palmas_smps10_out1_supply[] = {
};

static struct regulator_consumer_supply palmas_smps10_out2_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_mdm", NULL),
	REGULATOR_SUPPLY("vdd_5v0_snsr", NULL),
	REGULATOR_SUPPLY("vdd_5v0_dis", NULL),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("avddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("dvddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("avddio_usb", "tegra-xhci"),
};

static struct regulator_consumer_supply palmas_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_pll_m", NULL),
	REGULATOR_SUPPLY("avdd_pll_ap_c2_c3", NULL),
	REGULATOR_SUPPLY("avdd_pll_cud2dpd", NULL),
	REGULATOR_SUPPLY("avdd_pll_c4", NULL),
	REGULATOR_SUPPLY("avdd_lvds0_io", NULL),
	REGULATOR_SUPPLY("vddio_ddr_hs", NULL),
	REGULATOR_SUPPLY("avdd_pll_erefe", NULL),
	REGULATOR_SUPPLY("avdd_pll_x", NULL),
	REGULATOR_SUPPLY("avdd_pll_cg", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", "tegra-pcie"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
};

static struct regulator_consumer_supply palmas_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg2", NULL),
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
	REGULATOR_SUPPLY("vdig", "2-0010"),
	REGULATOR_SUPPLY("dvdd", "2-0010"),
	REGULATOR_SUPPLY("vdig", "2-0036"),
};

static struct regulator_consumer_supply palmas_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd", "spi0.0"),
};


static struct regulator_consumer_supply palmas_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_2v7_hv", NULL),
	REGULATOR_SUPPLY("avdd_cam1_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam2_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam3_cam", NULL),
	REGULATOR_SUPPLY("vana", "2-0010"),
	REGULATOR_SUPPLY("avdd", "2-0010"),
	REGULATOR_SUPPLY("vana", "2-0036"),
};

static struct regulator_consumer_supply palmas_ldo5_supply[] = {
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-xhci"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
	REGULATOR_SUPPLY("avdd_hsic_com", NULL),
	REGULATOR_SUPPLY("avdd_hsic_mdm", NULL),
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
};

static struct regulator_consumer_supply palmas_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_cam1_1v8_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam2_1v8_cam", NULL),
	REGULATOR_SUPPLY("vif", "2-0010"),
	REGULATOR_SUPPLY("dovdd", "2-0010"),
	REGULATOR_SUPPLY("vif2", "2-0021"),
	REGULATOR_SUPPLY("vif", "2-0036"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000c"),
	REGULATOR_SUPPLY("vi2c", "2-0030"),
};

static struct regulator_consumer_supply palmas_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_af1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg1", NULL),
	REGULATOR_SUPPLY("vana", "2-0021"),
	REGULATOR_SUPPLY("vdd", "2-000c"),
};

static struct regulator_consumer_supply palmas_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};


static struct regulator_consumer_supply palmas_ldo9_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply palmas_ldousb_supply[] = {
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("hvdd_usb", "tegra-xhci"),
	REGULATOR_SUPPLY("hvdd_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("hvdd_pex_pll_e", "tegra-pcie"),
};

static struct regulator_consumer_supply palmas_ldoln_supply[] = {
	REGULATOR_SUPPLY("vddio_hv", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("pwrdet_hv", NULL),
	REGULATOR_SUPPLY("vddio_pex_ctl", "tegra-pcie"),
};

PALMAS_REGS_PDATA(smps123, 900, 1400, NULL, 1, 1, 1, NORMAL,
	0, PALMAS_EXT_CONTROL_ENABLE1, 0, 0, 0);
PALMAS_REGS_PDATA(smps45, 900, 1400, NULL, 1, 1, 1, NORMAL,
	0, PALMAS_EXT_CONTROL_ENABLE2, 0, 0, 0);
PALMAS_REGS_PDATA(smps6, 1350, 1350, NULL, 1, 1, 1, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(smps7, 900, 1400, NULL, 1, 1, 1, NORMAL,
	0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(smps8, 1800, 1800, NULL, 1, 1, 1, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(smps9, 3300, 3300, NULL, 0, 0, 1, NORMAL,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(smps10_out1, 5000, 5000, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(smps10_out2, 5000, 5000, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo1, 1050, 1050, palmas_rails(smps6), 1, 1, 1, 0,
	0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REGS_PDATA(ldo2, 1050, 1200, palmas_rails(smps6), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo3, 3300, 3300, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo4, 2700, 2700, NULL, 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo5, 1200, 1200, palmas_rails(smps8), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo6, 1800, 1800, palmas_rails(smps9), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo7, 2700, 2700, palmas_rails(smps9), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo8, 900, 900, NULL, 1, 1, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldo9, 1800, 3300, palmas_rails(smps9), 0, 0, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldoln, 3300, 3300, palmas_rails(smps10_out2), 1, 1, 1, 0,
	0, 0, 0, 0, 0);
PALMAS_REGS_PDATA(ldousb, 3000, 3300, NULL, 1, 1, 1, 0,
	0, 0, 0, 0, 0);

#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname
static struct regulator_init_data *tn8_reg_data[PALMAS_NUM_REGS] = {
	NULL,
	PALMAS_REG_PDATA(smps123),
	NULL,
	PALMAS_REG_PDATA(smps45),
	NULL,
	PALMAS_REG_PDATA(smps6),
	PALMAS_REG_PDATA(smps7),
	PALMAS_REG_PDATA(smps8),
	PALMAS_REG_PDATA(smps9),
	PALMAS_REG_PDATA(smps10_out2),
	PALMAS_REG_PDATA(smps10_out1),
	PALMAS_REG_PDATA(ldo1),
	PALMAS_REG_PDATA(ldo2),
	PALMAS_REG_PDATA(ldo3),
	PALMAS_REG_PDATA(ldo4),
	PALMAS_REG_PDATA(ldo5),
	PALMAS_REG_PDATA(ldo6),
	PALMAS_REG_PDATA(ldo7),
	PALMAS_REG_PDATA(ldo8),
	PALMAS_REG_PDATA(ldo9),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	PALMAS_REG_PDATA(ldoln),
	PALMAS_REG_PDATA(ldousb),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname
static struct palmas_reg_init *tn8_reg_init[PALMAS_NUM_REGS] = {
	NULL,
	PALMAS_REG_INIT_DATA(smps123),
	NULL,
	PALMAS_REG_INIT_DATA(smps45),
	NULL,
	PALMAS_REG_INIT_DATA(smps6),
	PALMAS_REG_INIT_DATA(smps7),
	PALMAS_REG_INIT_DATA(smps8),
	PALMAS_REG_INIT_DATA(smps9),
	PALMAS_REG_INIT_DATA(smps10_out2),
	PALMAS_REG_INIT_DATA(smps10_out1),
	PALMAS_REG_INIT_DATA(ldo1),
	PALMAS_REG_INIT_DATA(ldo2),
	PALMAS_REG_INIT_DATA(ldo3),
	PALMAS_REG_INIT_DATA(ldo4),
	PALMAS_REG_INIT_DATA(ldo5),
	PALMAS_REG_INIT_DATA(ldo6),
	PALMAS_REG_INIT_DATA(ldo7),
	PALMAS_REG_INIT_DATA(ldo8),
	PALMAS_REG_INIT_DATA(ldo9),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	PALMAS_REG_INIT_DATA(ldoln),
	PALMAS_REG_INIT_DATA(ldousb),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

static struct palmas_pinctrl_config palmas_pincfg[] = {
	PALMAS_PINMUX("powergood", "powergood", NULL, NULL),
	PALMAS_PINMUX("vac", "vac", NULL, NULL),
	PALMAS_PINMUX("gpio0", "id", "pull-up", NULL),
	PALMAS_PINMUX("gpio1", "vbus_det", NULL, NULL),
	PALMAS_PINMUX("gpio2", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio3", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio4", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio5", "clk32kgaudio", NULL, NULL),
	PALMAS_PINMUX("gpio6", "gpio", NULL, NULL),
	PALMAS_PINMUX("gpio7", "gpio", NULL, NULL),
};

static struct palmas_pinctrl_platform_data palmas_pinctrl_pdata = {
	.pincfg = palmas_pincfg,
	.num_pinctrl = ARRAY_SIZE(palmas_pincfg),
	.dvfs1_enable = true,
	.dvfs2_enable = false,
};

static struct palmas_pmic_platform_data pmic_platform = {
};

static struct palmas_clk32k_init_data palmas_clk32k_idata[] = {
	{
		.clk32k_id = PALMAS_CLOCK32KG,
		.enable = true,
	}, {
		.clk32k_id = PALMAS_CLOCK32KG_AUDIO,
		.enable = true,
	},
};

struct max17048_platform_data tn8_max17048_pdata = {
	.model_data = &tn8_yoku_4100mA_max17048_battery,
	.tz_name = "battery-temp",
};

static struct i2c_board_info __initdata tn8_max17048_boardinfo[] = {
	{
		I2C_BOARD_INFO("max17048", 0x36),
		.platform_data = &tn8_max17048_pdata,
	},
};

static struct gadc_thermal_platform_data gadc_thermal_battery_pdata = {
	.iio_channel_name = "battery-temp-channel",
	.tz_name = "battery-temp",
	.temp_offset = 0,
	.adc_to_temp = NULL,
	.adc_temp_lookup = tn8_batt_temperature_table,
	.lookup_table_size = ARRAY_SIZE(tn8_batt_temperature_table),
	.first_index_temp = 125,
	.last_index_temp = -40,
};

static struct platform_device gadc_thermal_battery = {
	.name   = "generic-adc-thermal",
	.id     = 0,
	.dev    = {
		.platform_data = &gadc_thermal_battery_pdata,
	},
};

static struct iio_map palmas_iio_map[] = {
	PALMAS_GPADC_IIO_MAP(IN1, "generic-adc-thermal.0", "battery-temp-channel"),
	PALMAS_GPADC_IIO_MAP(IN6, "palmas-battery", "vbat_channel"),
	PALMAS_GPADC_IIO_MAP(NULL, NULL, NULL),
};

struct palmas_gpadc_platform_data gpadc_pdata = {
	.ch0_current = 0,
	.ch3_current = 0,
	.iio_maps = palmas_iio_map,
};

static struct palmas_extcon_platform_data palmas_extcon_pdata = {
	.connection_name = "palmas-extcon",
	.enable_vbus_detection = true,
	.enable_id_pin_detection = true,
};

static struct palmas_pm_platform_data palmas_pm_pdata = {
	.use_power_off = true,
	.use_power_reset = true,
};

static struct palmas_platform_data palmas_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_platform,
	.pinctrl_pdata = &palmas_pinctrl_pdata,
	.clk32k_init_data =  palmas_clk32k_idata,
	.clk32k_init_data_size = ARRAY_SIZE(palmas_clk32k_idata),
	.extcon_pdata = &palmas_extcon_pdata,
	.gpadc_pdata = &gpadc_pdata,
	.pm_pdata = &palmas_pm_pdata,
};

static struct i2c_board_info palma_device[] = {
	{
		I2C_BOARD_INFO("tps65913", 0x58),
		.irq            = INT_EXTERNAL_PMU,
		.platform_data  = &palmas_pdata,
	},
};

static struct power_supply_extcon_plat_data extcon_pdata = {
	.extcon_name = "tegra-udc",
};

static struct platform_device power_supply_extcon_device = {
	.name	= "power-supply-extcon",
	.id	= -1,
	.dev	= {
		.platform_data = &extcon_pdata,
	},
};

int __init tn8_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	int i;

	/* TPS65913: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);
	for (i = 0; i < PALMAS_NUM_REGS ; i++) {
		pmic_platform.reg_data[i] = tn8_reg_data[i];
		pmic_platform.reg_init[i] = tn8_reg_init[i];
	}

	/* Default PJ0 is connected to charger stat,
	 * HW rework is needed to connect to charger-int.
	 * Do not configure the charger int by default.
	 */
	/* bq2419x_boardinfo[0].irq = gpio_to_irq(TEGRA_GPIO_PJ0); */
	if (get_power_supply_type() == POWER_SUPPLY_TYPE_BATTERY) {
		tn8_bq2419x_pdata.bcharger_pdata = &bq2419x_charger_pdata;
		/* Only register fuel gauge when using battery. */
		i2c_register_board_info(0, tn8_max17048_boardinfo,
			ARRAY_SIZE(tn8_max17048_boardinfo));
	} else
		tn8_bq2419x_pdata.bcharger_pdata = NULL;

	i2c_register_board_info(0, bq2419x_boardinfo,
		ARRAY_SIZE(bq2419x_boardinfo));

	/* Tracking configuration */
	/* TODO
	reg_init_data_ldo8.config_flags =
		PALMAS_REGULATOR_CONFIG_TRACKING_ENABLE |
		PALMAS_REGULATOR_CONFIG_SUSPEND_TRACKING_DISABLE;
	*/
	reg_idata_smps45.constraints.init_uV = 1000000;

	reg_init_data_smps45.enable_gpio = TEGRA_GPIO_PR5;

	i2c_register_board_info(4, palma_device,
			ARRAY_SIZE(palma_device));

	platform_device_register(&gadc_thermal_battery);
	platform_device_register(&power_supply_extcon_device);
	return 0;
}
/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_en_"#_name
#define FIXED_REG(_id, _var, _name, _in_supply,			\
	_always_on, _boot_on, _gpio_nr, _open_drain,		\
	_active_high, _boot_state, _millivolts, _sdelay)	\
static struct regulator_init_data ri_data_##_var =		\
{								\
	.supply_regulator = _in_supply,				\
	.num_consumer_supplies =				\
	ARRAY_SIZE(fixed_reg_en_##_name##_supply),		\
	.consumer_supplies = fixed_reg_en_##_name##_supply,	\
	.constraints = {					\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
				REGULATOR_MODE_STANDBY),	\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
				REGULATOR_CHANGE_STATUS |	\
				REGULATOR_CHANGE_VOLTAGE),	\
		.always_on = _always_on,			\
		.boot_on = _boot_on,				\
	},							\
};								\
static struct fixed_voltage_config fixed_reg_en_##_var##_pdata =	\
{								\
	.supply_name = FIXED_SUPPLY(_name),			\
	.microvolts = _millivolts * 1000,			\
	.gpio = _gpio_nr,					\
	.gpio_is_open_drain = _open_drain,			\
	.enable_high = _active_high,				\
	.enabled_at_boot = _boot_state,				\
	.init_data = &ri_data_##_var,				\
	.startup_delay = _sdelay				\
};								\
static struct platform_device fixed_reg_en_##_var##_dev = {	\
	.name = "reg-fixed-voltage",				\
	.id = _id,						\
	.dev = {						\
		.platform_data = &fixed_reg_en_##_var##_pdata,	\
	},							\
}

/* Always ON Battery regulator */
static struct regulator_consumer_supply fixed_reg_en_battery_supply[] = {
		REGULATOR_SUPPLY("vdd_sys_bl", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_usb0_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus0", "tegra-xhci"),
};

static struct regulator_consumer_supply fixed_reg_en_usb1_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.1"),
	REGULATOR_SUPPLY("usb_vbus1", "tegra-xhci"),
};

static struct regulator_consumer_supply fixed_reg_en_usb2_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.2"),
	REGULATOR_SUPPLY("usb_vbus2", "tegra-xhci"),
};

static struct regulator_consumer_supply fixed_reg_en_palmas_gpio3_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_palmas_gpio4_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_1v2_s", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_palmas_gpio6_supply[] = {
	REGULATOR_SUPPLY("ldoen", "tegra-snd-rt5639"),
};

static struct regulator_consumer_supply fixed_reg_en_palmas_gpio7_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_lcd_bl_en_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_bl_en", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

FIXED_REG(0,	battery,	battery,	NULL,
	0,	0,	-1,
	false,	true,	0,	3300, 0);

FIXED_SYNC_REG(1,	usb0_vbus,	usb0_vbus,	NULL,
		0,	0,	TEGRA_GPIO_PN4,
		true,	true,	0,	5000,	0);

FIXED_SYNC_REG(2,	usb1_vbus,	usb1_vbus,	palmas_rails(smps10_out2),
		0,	0,	TEGRA_GPIO_PN5,
		true,	true,	0,	5000,	0);

FIXED_SYNC_REG(3,	usb2_vbus,	usb2_vbus,	palmas_rails(smps10_out2),
		0,	0,	TEGRA_GPIO_PFF1,
		true,	true,	0,	5000,	0);

FIXED_SYNC_REG(4,	palmas_gpio3,	palmas_gpio3,	palmas_rails(smps9),
		0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO3,
		false,	true,	0,	3300,	0);

FIXED_SYNC_REG(5,	palmas_gpio4,	palmas_gpio4,	palmas_rails(smps8),
		0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO4,
		false,	true,	0,	1200,	0);

FIXED_SYNC_REG(6,	palmas_gpio6,	palmas_gpio6,	palmas_rails(smps8),
		0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO6,
		false,	true,	0,	1200,	0);

FIXED_SYNC_REG(7,	palmas_gpio7,	palmas_gpio7,	palmas_rails(smps8),
		0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO7,
		false,	true,	0,	1800,	0);

FIXED_SYNC_REG(8,	lcd_bl_en,	lcd_bl_en, NULL,
		0,	0, TEGRA_GPIO_PH2,
		false,	true,	0,	5000,	0);

FIXED_SYNC_REG(9,	vdd_hdmi_5v0,	vdd_hdmi_5v0, palmas_rails(smps10_out1),
		0,	0, TEGRA_GPIO_PK6,
		false,	true,	0,	5000,	0);

/*
 * Creating fixed regulator device tables
 */
#define ADD_FIXED_REG(_name)    (&fixed_reg_en_##_name##_dev)
#define TN8_E1736_FIXED_REG		\
	ADD_FIXED_REG(battery),		\
	ADD_FIXED_REG(usb0_vbus),	\
	ADD_FIXED_REG(usb1_vbus),	\
	ADD_FIXED_REG(usb2_vbus),	\
	ADD_FIXED_REG(palmas_gpio3),	\
	ADD_FIXED_REG(palmas_gpio4),	\
	ADD_FIXED_REG(palmas_gpio6),	\
	ADD_FIXED_REG(palmas_gpio7),	\
	ADD_FIXED_REG(lcd_bl_en),	\
	ADD_FIXED_REG(vdd_hdmi_5v0),


static struct platform_device *fixed_reg_devs_e1736[] = {
	TN8_E1736_FIXED_REG
};

int __init tn8_fixed_regulator_init(void)
{
	struct board_info pmu_board_info;

	if (!of_machine_is_compatible("nvidia,tn8"))
		return 0;

	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_E1736)
		return platform_add_devices(fixed_reg_devs_e1736,
			ARRAY_SIZE(fixed_reg_devs_e1736));

	return 0;
}
