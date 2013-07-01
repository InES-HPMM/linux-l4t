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

#include <mach/irqs.h>
#include <mach/hardware.h>


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

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

static struct regulator_consumer_supply palmas_smps123_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply palmas_smps45_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};

static struct regulator_consumer_supply palmas_smps6_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr_mclk", NULL),
	REGULATOR_SUPPLY("vddio_ddr_hs", NULL),
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
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
#ifdef CONFIG_ARCH_TEGRA_12x_SOC
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_pll_utmip", "tegra-xhci"),
#endif
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vdd_1v8b", "0-0048"),
	REGULATOR_SUPPLY("vdd_dtv", NULL),
	REGULATOR_SUPPLY("vdd_1v8_eeprom", NULL),
	REGULATOR_SUPPLY("vddio_cam", "tegra_camera"),
	REGULATOR_SUPPLY("vddio_cam", "vi"),
	REGULATOR_SUPPLY("dvdd", "spi0.0"),
	REGULATOR_SUPPLY("vlogic", "0-0069"),
	REGULATOR_SUPPLY("vid", "0-000c"),
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
	REGULATOR_SUPPLY("vdd", "0-000c"),
	REGULATOR_SUPPLY("vdd", "0-0077"),
	REGULATOR_SUPPLY("vdd", "1-004c"),
};

static struct regulator_consumer_supply palmas_smps10_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
	REGULATOR_SUPPLY("vdd_5v0_mdm", NULL),
	REGULATOR_SUPPLY("vdd_5v0_snsr", NULL),
	REGULATOR_SUPPLY("vdd_5v0_dis", NULL),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5639.0"),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5645.0"),
	REGULATOR_SUPPLY("avddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("dvddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("avddio_usb", "tegra-xhci"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
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

};

static struct regulator_consumer_supply palmas_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg2", NULL),
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
	REGULATOR_SUPPLY("vdig", "2-0010"),
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
	REGULATOR_SUPPLY("vana", "2-0036"),
};

static struct regulator_consumer_supply palmas_ldo5_supply[] = {
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-xhci"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "vi"),
	REGULATOR_SUPPLY("avdd_hsic_com", NULL),
	REGULATOR_SUPPLY("avdd_hsic_mdm", NULL),
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
};

static struct regulator_consumer_supply palmas_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd_cam1_1v8_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam2_1v8_cam", NULL),
	REGULATOR_SUPPLY("vif", "2-0010"),
	REGULATOR_SUPPLY("vif", "2-0036"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000c"),

};

static struct regulator_consumer_supply palmas_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_af1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg1", NULL),
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
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
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
	REGULATOR_SUPPLY("vddio_pex_ctl", "tegra-pcie"),
};

PALMAS_PDATA_INIT(smps123, 900, 1400, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(smps45, 900, 1400, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(smps6, 1350, 1350, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(smps7, 900, 1400, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(smps8, 1800, 1800, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(smps9, 3300, 3300, NULL, 0, 0, 1, NORMAL);
PALMAS_PDATA_INIT(smps10, 5000, 5000, NULL, 0, 0, 1, NORMAL);
PALMAS_PDATA_INIT(ldo1, 1050, 1050, palmas_rails(smps6), 1, 1, 1, 0);
PALMAS_PDATA_INIT(ldo2, 1050, 1200, palmas_rails(smps6), 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldo3, 3300, 3300, NULL, 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldo4, 2700, 2700, NULL, 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldo5, 1200, 1200, palmas_rails(smps8), 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldo6, 1800, 1800, palmas_rails(smps9), 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldo7, 2700, 2700, palmas_rails(smps9), 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldo8, 1000, 1000, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(ldo9, 1800, 3300, palmas_rails(smps9), 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldoln, 1050, 1050, palmas_rails(smps10), 0, 0, 1, 0);
PALMAS_PDATA_INIT(ldousb, 3000, 3300, NULL, 1, 1, 1, 0);

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

#define PALMAS_REG_INIT(_name, _warm_reset, _roof_floor, _mode_sleep,	\
		_vsel)						\
	static struct palmas_reg_init reg_init_data_##_name = {		\
		.warm_reset = _warm_reset,				\
		.roof_floor =	_roof_floor,				\
		.mode_sleep = _mode_sleep,		\
		.vsel = _vsel,		\
	}

PALMAS_REG_INIT(smps123, 0, PALMAS_EXT_CONTROL_ENABLE1, 0, 0);
PALMAS_REG_INIT(smps45, 0, PALMAS_EXT_CONTROL_ENABLE2, 0, 0);
PALMAS_REG_INIT(smps6, 0, 0, 0, 0);
PALMAS_REG_INIT(smps7, 0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0);
PALMAS_REG_INIT(smps8, 0, 0, 0, 0);
PALMAS_REG_INIT(smps9, 0, 0, 0, 0);
PALMAS_REG_INIT(smps10, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo1, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo2, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo3, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo4, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo5, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo6, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo7, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo8, 0, 0, 0, 0);
PALMAS_REG_INIT(ldo9, 0, 0, 0, 0);
PALMAS_REG_INIT(ldoln, 0, 0, 0, 0);
PALMAS_REG_INIT(ldousb, 0, 0, 0, 0);

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
	PALMAS_PINMUX(POWERGOOD, POWERGOOD, DEFAULT, DEFAULT),
	PALMAS_PINMUX(VAC, VAC, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO0, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO1, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO2, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO3, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO4, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO5, CLK32KGAUDIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO6, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO7, GPIO, DEFAULT, DEFAULT),
};

static struct palmas_pinctrl_platform_data palmas_pinctrl_pdata = {
	.pincfg = palmas_pincfg,
	.num_pinctrl = ARRAY_SIZE(palmas_pincfg),
	.dvfs1_enable = true,
	.dvfs2_enable = false,
};

static struct palmas_pmic_platform_data pmic_platform = {
};

static struct palmas_platform_data palmas_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_platform,
	.use_power_off = true,
	.pinctrl_pdata = &palmas_pinctrl_pdata,
};

static struct i2c_board_info palma_device[] = {
	{
		I2C_BOARD_INFO("tps65913", 0x58),
		.irq            = INT_EXTERNAL_PMU,
		.platform_data  = &palmas_pdata,
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

	/* Tracking configuration */
	/* TODO
	reg_init_data_ldo8.config_flags =
		PALMAS_REGULATOR_CONFIG_TRACKING_ENABLE |
		PALMAS_REGULATOR_CONFIG_SUSPEND_TRACKING_DISABLE;
	*/
	i2c_register_board_info(4, palma_device,
			ARRAY_SIZE(palma_device));
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
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
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

FIXED_REG(0,	battery,	battery,	NULL,
	0,	0,	-1,
	false,	true,	0,	3300, 0);

FIXED_REG(1,	usb0_vbus,	usb0_vbus,	NULL,
	0,	0,	TEGRA_GPIO_PN4,
	false,	true,	0,	5000,	0);

FIXED_REG(2,	usb1_vbus,	usb1_vbus,	palmas_rails(smps10),
	0,	0,	TEGRA_GPIO_PN5,
	false,	true,	0,	5000,	0);

#ifdef CONFIG_ARCH_TEGRA_12x_SOC
FIXED_REG(3,	usb2_vbus,	usb2_vbus,	palmas_rails(smps10),
	0,	0,	TEGRA_GPIO_PFF1,
	false,	true,	0,	5000,	0);
#else
FIXED_REG(3,	usb2_vbus,	usb2_vbus,	palmas_rails(smps10),
	0,	0,	-1,
	false,	true,	0,	5000,	0);
#endif

FIXED_REG(4,	palmas_gpio3,	palmas_gpio3,	palmas_rails(smps9),
	0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO3,
	false,	true,	0,	3300,	0);

FIXED_REG(5,	palmas_gpio4,	palmas_gpio4,	palmas_rails(smps8),
	0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO4,
	false,	true,	0,	1200,	0);

FIXED_REG(6,	palmas_gpio6,	palmas_gpio6,	palmas_rails(smps8),
	0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO6,
	false,	true,	0,	1200,	0);

FIXED_REG(7,	palmas_gpio7,	palmas_gpio7,	palmas_rails(smps8),
	0,	0,	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO7,
	false,	true,	0,	1800,	0);

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
	ADD_FIXED_REG(palmas_gpio7),


static struct platform_device *fixed_reg_devs_e1736[] = {
	TN8_E1736_FIXED_REG
};

static int __init tn8_fixed_regulator_init(void)
{
	struct board_info pmu_board_info;

	if (!of_machine_is_compatible("nvidia,ardbeg"))
		return 0;

	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_E1736)
		return platform_add_devices(fixed_reg_devs_e1736,
			ARRAY_SIZE(fixed_reg_devs_e1736));

	return 0;
}

subsys_initcall_sync(tn8_fixed_regulator_init);
