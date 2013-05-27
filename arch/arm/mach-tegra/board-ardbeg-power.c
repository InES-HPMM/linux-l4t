/*
 * arch/arm/mach-tegra/board-ardbeg-power.c
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
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/palmas.h>
#include <linux/regulator/tps51632-regulator.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>

#include <asm/mach-types.h>

#include "pm.h"
#include "board.h"
#include "tegra-board-id.h"
#include "board-common.h"
#include "board-ardbeg.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include "iomap.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)


/************************ ARDBEG based regulator *****************/
static struct regulator_consumer_supply palmas_smps12_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply palmas_smps3_supply[] = {
	REGULATOR_SUPPLY("vdd_modem", NULL),
};

static struct regulator_consumer_supply palmas_smps6_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr_mclk", NULL),
	REGULATOR_SUPPLY("vddio_ddr3", NULL),
	REGULATOR_SUPPLY("vcore1_ddr3", NULL),
};

static struct regulator_consumer_supply palmas_smps8_supply[] = {
	REGULATOR_SUPPLY("avdd_pll_ap_c2_c3", NULL),
	REGULATOR_SUPPLY("avdd_pll_c4", NULL),
	REGULATOR_SUPPLY("avdd_pll_cg", NULL),
	REGULATOR_SUPPLY("avdd_pll_erefe", NULL),
	REGULATOR_SUPPLY("avdd_pll_m", NULL),
	REGULATOR_SUPPLY("avdd_pll_cud2dpd", NULL),
	REGULATOR_SUPPLY("avdd_pll_utmip", NULL),
	REGULATOR_SUPPLY("avdd_pll_x", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.1"),
};

static struct regulator_consumer_supply palmas_smps9_supply[] = {
	REGULATOR_SUPPLY("vdd_aud_dgtl", NULL),
	REGULATOR_SUPPLY("vdd_aud_anlg", NULL),
	REGULATOR_SUPPLY("vdd_aud_mic", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sys_2", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vdd_1v8b_pll_utmip", NULL),
	REGULATOR_SUPPLY("vddio_cam", "vi"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vdd_sys_mb", NULL),
	REGULATOR_SUPPLY("vdd_gmi_mb", NULL),
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vdd_1v8b_ts", NULL),
	REGULATOR_SUPPLY("vdd_1v8b_audio_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8b_com_f", NULL),
	REGULATOR_SUPPLY("vdd_1v8b_gps_f", NULL),
	REGULATOR_SUPPLY("vdd_1v8b_nfc", NULL),
	REGULATOR_SUPPLY("vdd_1v8b_uart_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8b", "0-0048"),
	REGULATOR_SUPPLY("vdd_dtv", NULL),
	REGULATOR_SUPPLY("vdd_1v8_bb_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8_uart_mdm", NULL),
	REGULATOR_SUPPLY("vdd_1v8_eeprom", NULL),
	REGULATOR_SUPPLY("vdd_1v8_dbg", NULL),
	REGULATOR_SUPPLY("vdd_1v8_pm", NULL),
};


static struct regulator_consumer_supply palmas_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v1_cam", NULL),
};

static struct regulator_consumer_supply palmas_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avddio_pex_pll", NULL),
	REGULATOR_SUPPLY("dvddio_pex", NULL),
};

static struct regulator_consumer_supply palmas_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddio_cam_mb", NULL),
	REGULATOR_SUPPLY("vdd_cam_1v8_cam", NULL),
	REGULATOR_SUPPLY("vif", "2-0010"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000e"),
};

static struct regulator_consumer_supply palmas_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
	REGULATOR_SUPPLY("vdig", "2-0010"),
};

static struct regulator_consumer_supply palmas_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply palmas_ldo6_supply[] = {
	REGULATOR_SUPPLY("avdd_cam1_cam", NULL),
	REGULATOR_SUPPLY("vana", "2-0010"),
};

static struct regulator_consumer_supply palmas_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_cam2_cam", NULL),
};

static struct regulator_consumer_supply palmas_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd_snsr_mb", NULL),
	REGULATOR_SUPPLY("vdd_snsr_temp", NULL),
	REGULATOR_SUPPLY("vdd", "0-0048"),
	REGULATOR_SUPPLY("vdd_snsr_pm", NULL),
	REGULATOR_SUPPLY("vdd_pca", "1-0071"),
};

static struct regulator_consumer_supply palmas_ldo9_supply[] = {
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_1v2_hsic_mdm", NULL),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
	REGULATOR_SUPPLY("avdd_1v2_hsic_com", NULL),
	REGULATOR_SUPPLY("vdd_lcd_bl_en", NULL),
};

static struct regulator_consumer_supply palmas_ldo10_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply palmas_ldo11_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_3v3_pex", NULL),
	REGULATOR_SUPPLY("avdd_3v3_pex_pll", NULL),
};

static struct regulator_consumer_supply palmas_ldo12_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_1v8b_dis", NULL),
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
};

static struct regulator_consumer_supply palmas_ldo13_supply[] = {
	REGULATOR_SUPPLY("dvdd", "spi0.0"),
};

static struct regulator_consumer_supply palmas_ldo14_supply[] = {
	REGULATOR_SUPPLY("avdd_af1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg1", NULL),
	REGULATOR_SUPPLY("vdd", "2-000e"),
};

static struct regulator_consumer_supply palmas_ldoln_supply[] = {
	REGULATOR_SUPPLY("avdd", "spi0.0"),
};

static struct regulator_consumer_supply palmas_ldousb_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply palmas_regen1_supply[] = {
	REGULATOR_SUPPLY("vdd_com_3v3", NULL),
	REGULATOR_SUPPLY("vdd_gps_3v3", NULL),
	REGULATOR_SUPPLY("vdd_nfc_3v3", NULL),
};

static struct regulator_consumer_supply palmas_regen2_supply[] = {
};

static struct regulator_consumer_supply palmas_regen4_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

/* TPS51632 DC-DC converter */
static struct regulator_consumer_supply tps51632_dcdc_cpu_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_init_data tps51632_cpu_init_data = {
	.constraints = {						\
		.min_uV = 500000,					\
		.max_uV = 1520000,					\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
					REGULATOR_MODE_STANDBY),	\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
					REGULATOR_CHANGE_STATUS |	\
					 REGULATOR_CHANGE_CONTROL |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		.always_on = 1,						\
		.boot_on =  1,						\
		.apply_uV = 0,						\
	},								\
	.num_consumer_supplies = ARRAY_SIZE(tps51632_dcdc_cpu_supply),	\
	.consumer_supplies = tps51632_dcdc_cpu_supply,		\
	.supply_regulator = palmas_rails(regen2),			\
};

static struct tps51632_regulator_platform_data tps51632_pdata_cpu = {
	.reg_init_data = &tps51632_cpu_init_data,		\
	.enable_pwm = false,				\
	.max_voltage_uV = 1520000,			\
	.base_voltage_uV = 500000,			\
	.slew_rate_uv_per_us = 6000,			\
};

static struct i2c_board_info tps51632_cpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps51632_cpu", 0x43),
		.platform_data	= &tps51632_pdata_cpu,
	},
};

static struct regulator_consumer_supply tps51632_dcdc_gpu_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};

static struct regulator_init_data tps51632_init_gpu_data = {
	.constraints = {						\
		.min_uV = 500000,					\
		.max_uV = 1520000,					\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
					REGULATOR_MODE_STANDBY),	\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
					REGULATOR_CHANGE_STATUS |	\
					 REGULATOR_CHANGE_CONTROL |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		.always_on = 1,						\
		.boot_on =  1,						\
		.apply_uV = 0,						\
	},								\
	.num_consumer_supplies = ARRAY_SIZE(tps51632_dcdc_gpu_supply),	\
	.consumer_supplies = tps51632_dcdc_gpu_supply,		\
	.supply_regulator = palmas_rails(regen2),			\
};

static struct tps51632_regulator_platform_data tps51632_pdata_gpu = {
	.reg_init_data = &tps51632_init_gpu_data,		\
	.enable_pwm = false,				\
	.max_voltage_uV = 1520000,			\
	.base_voltage_uV = 500000,			\
	.slew_rate_uv_per_us = 6000,			\
};

static struct i2c_board_info tps51632_gpu_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps51632_gpu", 0x45),
		.platform_data	= &tps51632_pdata_gpu,
	},
};

#define PALMAS_PDATA_INIT(_name, _minmv, _maxmv, _supply_reg, _always_on, \
	_boot_on, _apply_uv)						\
	static struct regulator_init_data reg_idata_##_name = {		\
		.constraints = {					\
			.name = palmas_rails(_name),			\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
			.apply_uV = _apply_uv,				\
		},							\
		.num_consumer_supplies =				\
			ARRAY_SIZE(palmas_##_name##_supply),		\
		.consumer_supplies = palmas_##_name##_supply,		\
		.supply_regulator = _supply_reg,			\
	}

PALMAS_PDATA_INIT(smps12, 900, 1300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps3, 1000, 3300, NULL, 1, 1, 0);
PALMAS_PDATA_INIT(smps6, 500, 1650, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps8, 1000, 3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps9, 1800, 1800, NULL, 1, 1, 0);
PALMAS_PDATA_INIT(ldo1, 1050, 1050, palmas_rails(smps3), 0, 0, 1);
PALMAS_PDATA_INIT(ldo2, 2800, 3000, palmas_rails(smps6), 0, 0, 0);
PALMAS_PDATA_INIT(ldo3, 2800, 3000, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo4, 2800, 3000, palmas_rails(smps3), 0, 0, 0);
PALMAS_PDATA_INIT(ldo5, 1100, 1100, palmas_rails(smps6), 1, 1, 1);
PALMAS_PDATA_INIT(ldo6, 2700, 2700, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo7, 2800, 2800, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo8, 2800, 3000, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo9, 2800, 3000, palmas_rails(smps3), 1, 1, 0);
PALMAS_PDATA_INIT(ldo10, 1800, 3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo11, 3300, 3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo12, 2800, 3000, palmas_rails(smps9), 1, 1, 0);
PALMAS_PDATA_INIT(ldo13, 1800, 1800, palmas_rails(smps9), 1, 1, 1);
PALMAS_PDATA_INIT(ldo14, 2800, 3000, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldoln, 3300, 3300, NULL, 1, 1, 1);
PALMAS_PDATA_INIT(ldousb, 2800, 3000, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(regen1, 3300, 3300, NULL, 1, 0, 0);
PALMAS_PDATA_INIT(regen2, 5000, 5000, NULL, 1, 0, 0);
PALMAS_PDATA_INIT(regen4, 5000, 5000, NULL, 0, 0, 0);

#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname

static struct regulator_init_data *ardbeg_reg_data[PALMAS_NUM_REGS] = {
	PALMAS_REG_PDATA(smps12),
	NULL,
	PALMAS_REG_PDATA(smps3),
	NULL,
	NULL,
	PALMAS_REG_PDATA(smps6),
	NULL,
	PALMAS_REG_PDATA(smps8),
	PALMAS_REG_PDATA(smps9),
	NULL,
	PALMAS_REG_PDATA(ldo1),
	PALMAS_REG_PDATA(ldo2),
	PALMAS_REG_PDATA(ldo3),
	PALMAS_REG_PDATA(ldo4),
	PALMAS_REG_PDATA(ldo5),
	PALMAS_REG_PDATA(ldo6),
	PALMAS_REG_PDATA(ldo7),
	PALMAS_REG_PDATA(ldo8),
	PALMAS_REG_PDATA(ldo9),
	PALMAS_REG_PDATA(ldo10),
	PALMAS_REG_PDATA(ldo11),
	PALMAS_REG_PDATA(ldo12),
	PALMAS_REG_PDATA(ldo13),
	PALMAS_REG_PDATA(ldo14),
	PALMAS_REG_PDATA(ldoln),
	PALMAS_REG_PDATA(ldousb),
	PALMAS_REG_PDATA(regen1),
	PALMAS_REG_PDATA(regen2),
	NULL,
	PALMAS_REG_PDATA(regen4),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
};

#define PALMAS_REG_INIT(_name, _warm_reset, _roof_floor, _mode_sleep,	\
		_tstep, _vsel)						\
	static struct palmas_reg_init reg_init_data_##_name = {		\
		.warm_reset = _warm_reset,				\
		.roof_floor =	_roof_floor,				\
		.mode_sleep = _mode_sleep,		\
		.tstep = _tstep,			\
		.vsel = _vsel,		\
	}

PALMAS_REG_INIT(smps12, 0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REG_INIT(smps123, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps3, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps45, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps457, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps6, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps7, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps8, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps9, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps10, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo1, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo2, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo3, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo4, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo5, 0, PALMAS_EXT_CONTROL_NSLEEP, 1, 0, 0);
PALMAS_REG_INIT(ldo6, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo7, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo8, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo9, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo10, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo11, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo12, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo13, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo14, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldoln, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldousb, 0, 0, 0, 0, 0);

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname
static struct palmas_reg_init *ardbeg_reg_init[PALMAS_NUM_REGS] = {
	PALMAS_REG_INIT_DATA(smps12),
	PALMAS_REG_INIT_DATA(smps123),
	PALMAS_REG_INIT_DATA(smps3),
	PALMAS_REG_INIT_DATA(smps45),
	PALMAS_REG_INIT_DATA(smps457),
	PALMAS_REG_INIT_DATA(smps6),
	PALMAS_REG_INIT_DATA(smps7),
	PALMAS_REG_INIT_DATA(smps8),
	PALMAS_REG_INIT_DATA(smps9),
	PALMAS_REG_INIT_DATA(smps10),
	PALMAS_REG_INIT_DATA(ldo1),
	PALMAS_REG_INIT_DATA(ldo2),
	PALMAS_REG_INIT_DATA(ldo3),
	PALMAS_REG_INIT_DATA(ldo4),
	PALMAS_REG_INIT_DATA(ldo5),
	PALMAS_REG_INIT_DATA(ldo6),
	PALMAS_REG_INIT_DATA(ldo7),
	PALMAS_REG_INIT_DATA(ldo8),
	PALMAS_REG_INIT_DATA(ldo9),
	PALMAS_REG_INIT_DATA(ldo10),
	PALMAS_REG_INIT_DATA(ldo11),
	PALMAS_REG_INIT_DATA(ldo12),
	PALMAS_REG_INIT_DATA(ldo13),
	PALMAS_REG_INIT_DATA(ldo14),
	PALMAS_REG_INIT_DATA(ldoln),
	PALMAS_REG_INIT_DATA(ldousb),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_en"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _open_drain, _active_high, _boot_state, _millivolts,	\
	_sdelay)							\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_en_##_name##_supply),	\
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
	static struct fixed_voltage_config fixed_reg_en_##_var##_pdata = \
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


/* Always ON /Battery regulator */
static struct regulator_consumer_supply fixed_reg_en_battery_supply[] = {
		REGULATOR_SUPPLY("vdd_sys_bl", NULL),
		REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
		REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_cdc_1v2_aud_supply[] = {
	REGULATOR_SUPPLY("vdd_cdc_1v2_aud", NULL),
};


static struct regulator_consumer_supply fixed_reg_en_vdd_lcd_1v2_dis_supply[] = {
	REGULATOR_SUPPLY("vdd_cdc_1v2_dis", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_cdc_3v3a_aud_supply[] = {
	REGULATOR_SUPPLY("vdd_cdc_3v3a_aud", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_usb0_5v0_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.2"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-xhci.1"),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_dis_3v3a_sw_supply[] = {
	REGULATOR_SUPPLY("vdd_dis_3v3_lcd", NULL),
	REGULATOR_SUPPLY("vdd_dis_3v3_lvds", NULL),
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

FIXED_REG(0,	battery,	battery,
	NULL,	0,	0,
	-1,	false, true,	0,	3300,	0);

FIXED_REG(1,	vdd_cdc_1v2_aud,	vdd_cdc_1v2_aud,
	palmas_rails(smps3),	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO14,	false, true,	0,	1200,
	0);

FIXED_REG(2,	vdd_lcd_1v2_dis,	vdd_lcd_1v2_dis,
	palmas_rails(smps3),	1,	1,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO4,	false, true,	0,	1200,
	0);

FIXED_REG(3,	vdd_cdc_3v3a_aud,	vdd_cdc_3v3a_aud,
	NULL,	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO10,	false, true,	0,	3300,
	0);

FIXED_REG(4,	vdd_usb0_5v0,	vdd_usb0_5v0,
	NULL,	0,	0,
	TEGRA_GPIO_PN4,	true, true,	0,	5000,
	0);

FIXED_REG(5,	vdd_dis_3v3a_sw,	vdd_dis_3v3a_sw,
	NULL,	1,	1,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO3,	true, true,	0,	3300,
	0);

FIXED_REG(6,	vdd_hdmi_5v0,	vdd_hdmi_5v0,
	NULL,	0,	0,
	TEGRA_GPIO_PH7,	true,	true,	0,	5000,	5000);

/*
 * Creating fixed regulator device tables
 */
#define ADD_FIXED_REG(_name)    (&fixed_reg_en_##_name##_dev)

#define E1780_COMMON_FIXED_REG			\
	ADD_FIXED_REG(battery),			\
	ADD_FIXED_REG(vdd_cdc_1v2_aud),		\
	ADD_FIXED_REG(vdd_lcd_1v2_dis),		\
	ADD_FIXED_REG(vdd_cdc_3v3a_aud),	\
	ADD_FIXED_REG(vdd_usb0_5v0),		\
	ADD_FIXED_REG(vdd_dis_3v3a_sw),		\
	ADD_FIXED_REG(vdd_hdmi_5v0),

/* Gpio switch regulator platform data for ardbeg E1580 */
static struct platform_device *pfixed_reg_devs[] = {
	E1780_COMMON_FIXED_REG
};

static struct palmas_pmic_platform_data pmic_platform = {
	.enable_ldo8_tracking = false,
	.disabe_ldo8_tracking_suspend = false,
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

static struct palmas_pinctrl_config palmas_pincfg[] = {
	PALMAS_PINMUX(POWERGOOD, POWERGOOD, DEFAULT, DEFAULT),
	PALMAS_PINMUX(VAC, VAC, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO0, ID, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO1, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO2, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO3, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO4, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO5, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO6, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO7, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO8, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO9, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO10, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO11, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO12, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO13, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO14, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO15, GPIO, DEFAULT, DEFAULT),
};

static struct palmas_pinctrl_platform_data palmas_pinctrl_pdata = {
	.pincfg = palmas_pincfg,
	.num_pinctrl = ARRAY_SIZE(palmas_pincfg),
	.dvfs1_enable = false,
	.dvfs2_enable = false,
};

static struct palmas_platform_data palmas_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_platform,
	.clk32k_init_data =  palmas_clk32k_idata,
	.clk32k_init_data_size = ARRAY_SIZE(palmas_clk32k_idata),
	.irq_flags = IRQ_TYPE_LEVEL_HIGH,
	.use_power_off = true,
	.pinctrl_pdata = &palmas_pinctrl_pdata,
};

static struct i2c_board_info palma_device[] = {
	{
		I2C_BOARD_INFO("tps80036", 0x58),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &palmas_pdata,
	},
};

static struct tegra_suspend_platform_data ardbeg_suspend_data = {
	.cpu_timer      = 500,
	.cpu_off_timer  = 300,
	.suspend_mode   = TEGRA_SUSPEND_LP0,
	.core_timer     = 0x157e,
	.core_off_timer = 2000,
	.corereq_high   = true,
	.sysclkreq_high = true,
	.cpu_lp2_min_residency = 1000,
	.min_residency_crail = 20000,
};

int __init ardbeg_suspend_init(void)
{
	tegra_init_suspend(&ardbeg_suspend_data);
	return 0;
}

int __init ardbeg_regulator_init(void)
{
	int i;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when high */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl & ~PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);


	for (i = 0; i < PALMAS_NUM_REGS ; i++) {
		pmic_platform.reg_data[i] = ardbeg_reg_data[i];
		pmic_platform.reg_init[i] = ardbeg_reg_init[i];
	}

	i2c_register_board_info(4, palma_device,
			ARRAY_SIZE(palma_device));
	i2c_register_board_info(4, tps51632_cpu_boardinfo, 1);
	i2c_register_board_info(4, tps51632_gpu_boardinfo, 1);
	reg_init_data_ldo5.enable_tracking = true;
	reg_init_data_ldo5.tracking_regulator = PALMAS_REG_SMPS12;

	return 0;
}

static int __init ardbeg_fixed_regulator_init(void)
{
	struct board_info board_info;

	if (!of_machine_is_compatible("nvidia,ardbeg"))
		return 0;

	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E1780)
		return platform_add_devices(pfixed_reg_devs,
				ARRAY_SIZE(pfixed_reg_devs));
}

subsys_initcall_sync(ardbeg_fixed_regulator_init);
