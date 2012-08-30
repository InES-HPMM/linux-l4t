/*
 * arch/arm/mach-tegra/board-pluto-power.c
 *
 * Copyright (C) 2012 NVIDIA Corporation.
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
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/palmas.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>

#include "pm.h"
#include "board.h"
#include "board-pluto.h"


/************************ Pluto based regulator ****************/
static struct regulator_consumer_supply palmas_smps12_supply[] = {
	REGULATOR_SUPPLY("unused_smps12", NULL),
};

static struct regulator_consumer_supply palmas_smps123_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply palmas_smps3_supply[] = {
	REGULATOR_SUPPLY("unused_smps3", NULL),
};

static struct regulator_consumer_supply palmas_smps45_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply palmas_smps457_supply[] = {
	REGULATOR_SUPPLY("unused_smps457", NULL),
};

static struct regulator_consumer_supply palmas_smps6_supply[] = {
	REGULATOR_SUPPLY("vdd_core_bb", NULL),
};

static struct regulator_consumer_supply palmas_smps7_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr", NULL),
};

static struct regulator_consumer_supply palmas_smps8_supply[] = {

	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vdd_1v8_gps", NULL),
	REGULATOR_SUPPLY("vdd_1v8_nfc", NULL),
	REGULATOR_SUPPLY("vdd_1v8_snsr", NULL),
	REGULATOR_SUPPLY("vdd_1v8_dtv", NULL),
	REGULATOR_SUPPLY("vdd_1v8_bb", NULL),
	REGULATOR_SUPPLY("vcore_lpddr", NULL),
	REGULATOR_SUPPLY("vddio_lpddr", NULL),
	REGULATOR_SUPPLY("vdd_1v8_rf", NULL),
	REGULATOR_SUPPLY("vdd_1v8_mdm2", NULL),
	REGULATOR_SUPPLY("vdd_1v8_dbg", NULL),
};

static struct regulator_consumer_supply palmas_smps9_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_core", NULL),
};

static struct regulator_consumer_supply palmas_smps10_supply[] = {
	REGULATOR_SUPPLY("unused_smps10", NULL),
};

static struct regulator_consumer_supply palmas_ldo1_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu", NULL),
	REGULATOR_SUPPLY("avdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
};

static struct regulator_consumer_supply palmas_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply palmas_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply palmas_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_spare", NULL),
};

static struct regulator_consumer_supply palmas_ldo5_supply[] = {
	REGULATOR_SUPPLY("avdd_cam1", NULL),
};

static struct regulator_consumer_supply palmas_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_3v0_temp", NULL),
	REGULATOR_SUPPLY("vdd_3v0_mb", NULL),
	REGULATOR_SUPPLY("avdd_ts", NULL),
	REGULATOR_SUPPLY("vdd_3v0_nfc", NULL),
	REGULATOR_SUPPLY("vdd_3v0_irled", NULL),
	REGULATOR_SUPPLY("vdd_3v0_snsr", NULL),
	REGULATOR_SUPPLY("vdd_3v0_pm", NULL),
};

static struct regulator_consumer_supply palmas_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_af_cam1", NULL),
};
static struct regulator_consumer_supply palmas_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};
static struct regulator_consumer_supply palmas_ldo9_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};
static struct regulator_consumer_supply palmas_ldoln_supply[] = {
	REGULATOR_SUPPLY("avdd_cam2", NULL),
};

static struct regulator_consumer_supply palmas_ldousb_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("hvdd_usb3", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vddio_hv", NULL),
	REGULATOR_SUPPLY("vdd_3v3_dtv", NULL),

};

#define PALMAS_PDATA_INIT(_name, _minmv, _maxmv, _supply_reg, _always_on,	\
	_boot_on, _apply_uv)			\
	static struct regulator_init_data reg_idata_##_name = {		\
		.constraints = {							\
			.name = palmas_rails(_name),					\
			.min_uV = (_minmv)*1000,					\
			.max_uV = (_maxmv)*1000,					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |			\
					REGULATOR_MODE_STANDBY),			\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |			\
					REGULATOR_CHANGE_STATUS |			\
					REGULATOR_CHANGE_VOLTAGE),			\
			.always_on = _always_on,					\
			.boot_on = _boot_on,						\
			.apply_uV = _apply_uv,						\
		},									\
		.num_consumer_supplies =						\
			ARRAY_SIZE(palmas_##_name##_supply),			\
		.consumer_supplies = palmas_##_name##_supply,			\
		.supply_regulator = _supply_reg,	\
	}

PALMAS_PDATA_INIT(smps12, 500,  1650, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps123, 500,  1650, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps3, 500,  1650, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps45, 500,  1650, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps457, 500,  1650, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps6, 1000,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps7, 1000,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps8, 1000,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps9, 1000,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(smps10, 3600,  5050, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo1, 900,  3300, palmas_rails(smps7), 0, 0, 0);
PALMAS_PDATA_INIT(ldo2, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo3, 900,  3300, palmas_rails(smps8), 0, 0, 0);
PALMAS_PDATA_INIT(ldo4, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo5, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo6, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo7, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo8, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo9, 900,  3300, palmas_rails(smps9), 0, 0, 0);
PALMAS_PDATA_INIT(ldoln, 900,  3300, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldousb, 900,  3300, NULL, 0, 0, 0);

static struct palmas_platform_data palmas_pdata;

static struct palmas_pmic_platform_data pmic_platform;
static struct i2c_board_info pluto_regulators[] = {
	{
		I2C_BOARD_INFO("tps65913", 0x58),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &palmas_pdata,
	},
};

#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname
static struct regulator_init_data *pluto_reg_data[] = {
	PALMAS_REG_PDATA(smps12),
	PALMAS_REG_PDATA(smps123),
	PALMAS_REG_PDATA(smps3),
	PALMAS_REG_PDATA(smps45),
	PALMAS_REG_PDATA(smps457),
	PALMAS_REG_PDATA(smps6),
	PALMAS_REG_PDATA(smps7),
	PALMAS_REG_PDATA(smps8),
	PALMAS_REG_PDATA(smps9),
	PALMAS_REG_PDATA(smps10),
	PALMAS_REG_PDATA(ldo1),
	PALMAS_REG_PDATA(ldo2),
	PALMAS_REG_PDATA(ldo3),
	PALMAS_REG_PDATA(ldo4),
	PALMAS_REG_PDATA(ldo5),
	PALMAS_REG_PDATA(ldo6),
	PALMAS_REG_PDATA(ldo7),
	PALMAS_REG_PDATA(ldo8),
	PALMAS_REG_PDATA(ldo9),
	PALMAS_REG_PDATA(ldoln),
	PALMAS_REG_PDATA(ldousb),
};

#define PALMAS_REG_INIT(_name, _warm_reset, _roof_floor, _mode_sleep, _tstep, _vsel)		\
	static struct palmas_reg_init reg_init_data_##_name = {		\
		.warm_reset = _warm_reset,				\
		.roof_floor =	_roof_floor,				\
		.mode_sleep = _mode_sleep,		\
		.tstep = _tstep,			\
		.vsel = _vsel,		\
	}

PALMAS_REG_INIT(smps12, 0, 0, 0, 0, 0);
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
PALMAS_REG_INIT(ldo5, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo6, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo7, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo8, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo9, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldoln, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ldousb, 0, 0, 0, 0, 0);

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname
static struct palmas_reg_init *pluto_reg_init[] = {
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
	PALMAS_REG_INIT_DATA(ldoln),
	PALMAS_REG_INIT_DATA(ldousb),
};

static int ac_online(void)
{
	return 1;
}

static struct resource pluto_pda_resources[] = {
	[0] = {
		.name	= "ac",
	},
};

static struct pda_power_pdata pluto_pda_data = {
	.is_ac_online	= ac_online,
};

static struct platform_device pluto_pda_power_device = {
	.name		= "pda-power",
	.id		= -1,
	.resource	= pluto_pda_resources,
	.num_resources	= ARRAY_SIZE(pluto_pda_resources),
	.dev	= {
		.platform_data	= &pluto_pda_data,
	},
};

static struct tegra_suspend_platform_data pluto_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= false,
	.sysclkreq_high	= true,
};

int __init pluto_regulator_init(void)
{
	pmic_platform.reg_data = pluto_reg_data;
	pmic_platform.reg_init = pluto_reg_init;
	palmas_pdata.pmic_pdata = &pmic_platform;
	palmas_pdata.gpio_base = PALMAS_TEGRA_GPIO_BASE;
	palmas_pdata.irq_base = PALMAS_TEGRA_IRQ_BASE;
	platform_device_register(&pluto_pda_power_device);
	i2c_register_board_info(4, pluto_regulators,
			ARRAY_SIZE(pluto_regulators));
	return 0;
}

int __init pluto_suspend_init(void)
{
	tegra_init_suspend(&pluto_suspend_data);
	return 0;
}

/* Fixed regulators */
static struct regulator_consumer_supply fixed_reg_en_vdd_1v8_cam_supply[] = {
	REGULATOR_SUPPLY("vddio_cam_mb", NULL),
	REGULATOR_SUPPLY("vdd_1v8_cam12", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_1v2_cam_supply[] = {
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_1v8_ldousb3_supply[] = {
	REGULATOR_SUPPLY("avdd_usb3_pll", NULL),
	REGULATOR_SUPPLY("avddio_usb3", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_mmc_sdmmc3_supply[] = {
	REGULATOR_SUPPLY("vdd_sdmmc3", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_1v8_lcd_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_lcd_s", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_mmc_lcd_supply[] = {
	REGULATOR_SUPPLY("vdd_mmc_lcd_s_f", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_1v8_mic_supply[] = {
	REGULATOR_SUPPLY("unused_fixed_reg_en_vdd_1v8_mic", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_5v0_hdmi_supply[] = {
	REGULATOR_SUPPLY("vddio_hdmi", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_1v8_fuse_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("v_efuse", NULL),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_en"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _active_high, _boot_state, _millivolts)	\
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
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
	};								\
	static struct platform_device fixed_reg_en_##_var##_dev = {	\
		.name = "reg-fixed-voltage",				\
		.id = _id,						\
		.dev = {						\
			.platform_data = &fixed_reg_en_##_var##_pdata,	\
		},							\
	}

FIXED_REG(1,	vdd_1v8_cam,	vdd_1v8_cam,
	palmas_rails(smps8),	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO1,	true,	0,	1800);
FIXED_REG(2,	vdd_1v2_cam,	vdd_1v2_cam,
	palmas_rails(smps7),	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO2,	true,	0,	1200);
FIXED_REG(3,	vdd_1v8_ldousb3,	vdd_1v8_ldousb3,
	palmas_rails(smps8),	0,	0,
	TEGRA_GPIO_PK5,	true,	0,	1050);
FIXED_REG(4,	vdd_mmc_sdmmc3,	vdd_mmc_sdmmc3,
	palmas_rails(smps9),	0,	0,
	TEGRA_GPIO_PK1,	true,	0,	3300);
FIXED_REG(5,	vdd_1v8_lcd,	vdd_1v8_lcd,
	palmas_rails(smps8),	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO4,	true,	0,	1800);
FIXED_REG(6,	vdd_mmc_lcd,	vdd_mmc_lcd,
	palmas_rails(smps9),	0,	0,
	TEGRA_GPIO_PI4,	true,	0,	1800);
FIXED_REG(7,	vdd_1v8_mic,	vdd_1v8_mic,
	palmas_rails(smps8),	0,	0,
	-1,	true,	0,	1800);
FIXED_REG(8,	vdd_5v0_hdmi,	vdd_5v0_hdmi,
	palmas_rails(smps10),	0,	0,
	TEGRA_GPIO_PK6,	true,	0,	5000);

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
FIXED_REG(9,	vdd_1v8_fuse,	vdd_1v8_fuse,
	palmas_rails(smps8),	0,	0,
	TEGRA_GPIO_PX4,	true,	0,	1800);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
FIXED_REG(9,	vdd_1v8_fuse,	vdd_1v8_fuse,
	palmas_rails(smps8),	0,	0,
	TEGRA_GPIO_PX0,	true,	0,	1800);
#endif

/*
 * Creating the fixed regulator device tables
 */
#define ADD_FIXED_REG(_name)    (&fixed_reg_en_##_name##_dev)

#define E1580_COMMON_FIXED_REG			\
	ADD_FIXED_REG(vdd_1v8_cam),		\
	ADD_FIXED_REG(vdd_1v2_cam),	\
	ADD_FIXED_REG(vdd_1v8_ldousb3),		\
	ADD_FIXED_REG(vdd_mmc_sdmmc3),			\
	ADD_FIXED_REG(vdd_1v8_lcd),		\
	ADD_FIXED_REG(vdd_mmc_lcd),		\
	ADD_FIXED_REG(vdd_1v8_mic),	\
	ADD_FIXED_REG(vdd_5v0_hdmi),

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
#define E1580_T114_FIXED_REG			\
	ADD_FIXED_REG(vdd_1v8_fuse),
#endif

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
#define E1580_T30_FIXED_REG			\
	ADD_FIXED_REG(vdd_1v8_fuse),
#endif

/* Gpio switch regulator platform data for Pluto E1580 */
static struct platform_device *pfixed_reg_devs[] = {
	E1580_COMMON_FIXED_REG
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	E1580_T114_FIXED_REG
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	E1580_T30_FIXED_REG
#endif
};

static int __init pluto_fixed_regulator_init(void)
{
	int i;
	struct board_info board_info;
	struct platform_device **fixed_reg_devs;
	int nfixreg_devs;

	tegra_get_board_info(&board_info);
	fixed_reg_devs = pfixed_reg_devs;
	nfixreg_devs = ARRAY_SIZE(pfixed_reg_devs);

	if (!machine_is_tegra_pluto())
		return 0;

	for (i = 0; i < nfixreg_devs; ++i) {
		int gpio_nr;
		struct fixed_voltage_config *fixed_reg_pdata =
			fixed_reg_devs[i]->dev.platform_data;
		gpio_nr = fixed_reg_pdata->gpio;

	}

	return platform_add_devices(fixed_reg_devs, nfixreg_devs);
}
subsys_initcall_sync(pluto_fixed_regulator_init);

