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
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/palmas.h>
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>
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
#include "tegra-board-id.h"

#define PMC_CTRL                0x0
#define PMC_CTRL_INTR_LOW       (1 << 17)

#define PALMAS_REG_INIT(_name, _warm_reset, _roof_floor, _mode_sleep,	\
		_tstep, _vsel)						\
	static struct palmas_reg_init reg_init_data_##_name = {		\
		.warm_reset = _warm_reset,				\
		.roof_floor =	_roof_floor,				\
		.mode_sleep = _mode_sleep,		\
		.tstep = _tstep,			\
		.vsel = _vsel,		\
	}


/************************ ARDBEG E1733 based regulators ***********/
static struct regulator_consumer_supply as3722_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_pll_m", NULL),
	REGULATOR_SUPPLY("avdd_pll_ap_c2_c3", NULL),
	REGULATOR_SUPPLY("avdd_pll_cud2dpd", NULL),
	REGULATOR_SUPPLY("avdd_pll_c4", NULL),
	REGULATOR_SUPPLY("avdd_lvds0_io", NULL),
	REGULATOR_SUPPLY("vddio_ddr_hs", NULL),
	REGULATOR_SUPPLY("avdd_pll_erefe", NULL),
	REGULATOR_SUPPLY("avdd_pll_x", NULL),
	REGULATOR_SUPPLY("avdd_pll_cg", NULL),
};

static struct regulator_consumer_supply as3722_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_cam1_1v8_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam2_1v8_cam", NULL),
	REGULATOR_SUPPLY("vif", "2-0010"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000e"),

};

static struct regulator_consumer_supply as3722_ldo2_supply[] = {
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

static struct regulator_consumer_supply as3722_ldo3_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply as3722_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_2v7_hv", NULL),
	REGULATOR_SUPPLY("avdd_cam1_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam2_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam3_cam", NULL),
	REGULATOR_SUPPLY("vana", "2-0010"),
};

static struct regulator_consumer_supply as3722_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
	REGULATOR_SUPPLY("vdig", "2-0010"),
};

static struct regulator_consumer_supply as3722_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply as3722_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v1_cam", NULL),
};

static struct regulator_consumer_supply as3722_ldo9_supply[] = {
	REGULATOR_SUPPLY("avdd", "spi0.0"),
};

static struct regulator_consumer_supply as3722_ldo10_supply[] = {
	REGULATOR_SUPPLY("avdd_af1_cam", NULL),
	REGULATOR_SUPPLY("imx135_reg1", NULL),
	REGULATOR_SUPPLY("vdd", "2-000e"),
};

static struct regulator_consumer_supply as3722_ldo11_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply as3722_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply as3722_sd1_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply as3722_sd2_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr_mclk", NULL),
	REGULATOR_SUPPLY("vddio_ddr3", NULL),
	REGULATOR_SUPPLY("vcore1_ddr3", NULL),
};

static struct regulator_consumer_supply as3722_sd4_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi","tegradc.1"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_pex_pll", "tegra-pcie"),
	REGULATOR_SUPPLY("avddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("dvddio_pex", "tegra-pcie"),
	REGULATOR_SUPPLY("avddio_usb", "tegra-xhci"),
};

static struct regulator_consumer_supply as3722_sd5_supply[] = {
	REGULATOR_SUPPLY("dbvdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("dbvdd", "tegra-snd-rt5645"),
	REGULATOR_SUPPLY("avdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("avdd", "tegra-snd-rt5645"),
	REGULATOR_SUPPLY("dmicvdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("dmicvdd", "tegra-snd-rt5645"),
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
};

static struct regulator_consumer_supply as3722_sd6_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};

AMS_PDATA_INIT(sd0, NULL, 900000, 1400000, 1, 1, 1, 2);
AMS_PDATA_INIT(sd1, NULL, 900000, 1400000, 1, 1, 1, 1);
AMS_PDATA_INIT(sd2, NULL, 1350000, 1350000, 1, 1, 1, 0);
AMS_PDATA_INIT(sd4, NULL, 1050000, 1050000, 0, 1, 1, 0);
AMS_PDATA_INIT(sd5, NULL, 1800000, 1800000, 1, 1, 1, 0);
AMS_PDATA_INIT(sd6, NULL, 900000, 1400000, 1, 1, 1, 0);
AMS_PDATA_INIT(ldo0, AS3722_SUPPLY(sd2), 1050000, 1250000, 1, 1, 1, 1);
AMS_PDATA_INIT(ldo1, NULL, 1800000, 1800000, 0, 1, 1, 0);
AMS_PDATA_INIT(ldo2, AS3722_SUPPLY(sd5), 1200000, 1200000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo3, NULL, 1000000, 1000000, 1, 1, 1, 0);
AMS_PDATA_INIT(ldo4, NULL, 2700000, 2700000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo5, AS3722_SUPPLY(sd5), 1200000, 1200000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo6, NULL, 3300000, 3300000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo7, AS3722_SUPPLY(sd5), 1050000, 1050000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo9, NULL, 3300000, 3300000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo10, NULL, 2700000, 2700000, 0, 0, 1, 0);
AMS_PDATA_INIT(ldo11, NULL, 1800000, 1800000, 0, 0, 1, 0);

/* config settings are OTP plus initial state
 * GPIOsignal_out at 20h not configurable through OTP and is initialized to
 * zero. To enable output, the invert bit must be turned on.
 * GPIOxcontrol register format
 * bit(s)  bitname
 * ---------------------
 *  7     gpiox_invert   invert input or output
 * 6:3    gpiox_iosf     0: normal
 * 2:0    gpiox_mode     0: input, 1: output push/pull, 3: ADC input (tristate)
 *
 * Examples:
 * otp  meaning
 * ------------
 * 0x3  gpiox_invert=0(no invert), gpiox_iosf=0(normal), gpiox_mode=3(ADC input)
 * 0x81 gpiox_invert=1(invert), gpiox_iosf=0(normal), gpiox_mode=1(output)
 *
 * Note: output state should be defined for gpiox_mode = output.  Do not change
 * the state of the invert bit for critical devices such as GPIO 7 which enables
 * SDRAM. Driver applies invert mask to output state to configure GPIOsignal_out
 * register correctly.
 * E.g. Invert = 1, (requested) output state = 1 => GPIOsignal_out = 0
 */
static struct as3722_gpio_config as3722_gpio_cfgs[] = {
	{
		/* otp = 0x3 IGPU_PRDGD*/
		.gpio = AS3722_GPIO0,
		.mode = AS3722_GPIO_MODE_OUTPUT_VDDL,
	},
	{
		/* otp = 0x1  => REGEN_3 = LP0 gate (1.8V, 5 V)*/
		.gpio = AS3722_GPIO1,
		.invert     = AS3722_GPIO_CFG_INVERT, /* don't go into LP0 */
		.mode       = AS3722_GPIO_MODE_OUTPUT_VDDH,
		.output_state = AS3722_GPIO_CFG_OUTPUT_ENABLED,
	},
	{
		/* otp = 0x3 PMU_REGEN1*/
		.gpio = AS3722_GPIO2,
		.invert     = AS3722_GPIO_CFG_NO_INVERT, /* don't go into LP0 */
		.mode       = AS3722_GPIO_MODE_OUTPUT_VDDH,
		.output_state = AS3722_GPIO_CFG_OUTPUT_ENABLED,
	},
	{
		/* otp = 0x03 AP THERMISTOR */
		.gpio = AS3722_GPIO3,
		.mode = AS3722_GPIO_MODE_ADC_IN,
	},
	{
		/* otp = 0x81 => on by default
		 * gates EN_AVDD_LCD
		 */
		.gpio       = AS3722_GPIO4,
		.invert     = AS3722_GPIO_CFG_NO_INVERT,
		.mode       = AS3722_GPIO_MODE_OUTPUT_VDDH,
		.output_state = AS3722_GPIO_CFG_OUTPUT_ENABLED,
	},
	{
		/* otp = 0x3  CLK 23KHZ WIFI */
		.gpio = AS3722_GPIO5,
		.mode = AS3722_GPIO_MODE_ADC_IN,
	},
	{
		/* otp = 0x3  SKIN TEMP */
		.gpio = AS3722_GPIO6,
		.mode = AS3722_GPIO_MODE_ADC_IN,
	},
	{
		/* otp = 0x81  1.6V LP0*/
		.gpio       = AS3722_GPIO7,
		.invert     = AS3722_GPIO_CFG_INVERT,
		.mode       = AS3722_GPIO_MODE_OUTPUT_VDDH,
		.output_state = AS3722_GPIO_CFG_OUTPUT_ENABLED,
	},
};

static struct as3722_rtc_platform_data as3722_rtc_pdata = {
	.enable_clk32k	= 1,
};

static struct as3722_platform_data as3722_pdata = {
	.reg_pdata[AS3722_SD0] = &as3722_sd0_reg_pdata,
	.reg_pdata[AS3722_SD1] = &as3722_sd1_reg_pdata,
	.reg_pdata[AS3722_SD2] = &as3722_sd2_reg_pdata,
	.reg_pdata[AS3722_SD4] = &as3722_sd4_reg_pdata,
	.reg_pdata[AS3722_SD5] = &as3722_sd5_reg_pdata,
	.reg_pdata[AS3722_SD6] = &as3722_sd6_reg_pdata,
	.reg_pdata[AS3722_LDO0] = &as3722_ldo0_reg_pdata,
	.reg_pdata[AS3722_LDO1] = &as3722_ldo1_reg_pdata,
	.reg_pdata[AS3722_LDO2] = &as3722_ldo2_reg_pdata,
	.reg_pdata[AS3722_LDO3] = &as3722_ldo3_reg_pdata,
	.reg_pdata[AS3722_LDO4] = &as3722_ldo4_reg_pdata,
	.reg_pdata[AS3722_LDO5] = &as3722_ldo5_reg_pdata,
	.reg_pdata[AS3722_LDO6] = &as3722_ldo6_reg_pdata,
	.reg_pdata[AS3722_LDO7] = &as3722_ldo7_reg_pdata,
	.reg_pdata[AS3722_LDO9] = &as3722_ldo9_reg_pdata,
	.reg_pdata[AS3722_LDO10] = &as3722_ldo10_reg_pdata,
	.reg_pdata[AS3722_LDO11] = &as3722_ldo11_reg_pdata,
	.core_init_data = NULL,
	.gpio_base = AS3722_GPIO_BASE,
	.irq_base = AS3722_IRQ_BASE,
	.use_internal_int_pullup = 0,
	.use_internal_i2c_pullup = 0,
	.num_gpio_cfgs = ARRAY_SIZE(as3722_gpio_cfgs),
	.gpio_cfgs     = as3722_gpio_cfgs,
	.rtc_pdata	= &as3722_rtc_pdata,
};

static struct i2c_board_info __initdata as3722_regulators[] = {
	{
		I2C_BOARD_INFO("as3722", 0x40),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_EXTERNAL_PMU,
		.platform_data = &as3722_pdata,
	},
};

int __init ardbeg_as3722_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;


	/* AS3722: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	pr_info("%s: i2c_register_board_info\n", __func__);
	i2c_register_board_info(4, as3722_regulators,
			ARRAY_SIZE(as3722_regulators));
	return 0;
}

int __init ardbeg_ams_regulator_init(void)
{
	ardbeg_as3722_regulator_init();
	return 0;
}

/**************** ARDBEG-TI913 based regulator ************/
#define palmas_ti913_smps123_supply as3722_sd6_supply
#define palmas_ti913_smps45_supply as3722_sd1_supply
#define palmas_ti913_smps6_supply as3722_sd5_supply
#define palmas_ti913_smps7_supply as3722_sd2_supply
#define palmas_ti913_smps9_supply as3722_sd4_supply
#define palmas_ti913_ldo1_supply as3722_ldo0_supply
#define palmas_ti913_ldo2_supply as3722_ldo5_supply
#define palmas_ti913_ldo3_supply as3722_ldo9_supply
#define palmas_ti913_ldo4_supply as3722_ldo2_supply
#define palmas_ti913_ldo5_supply as3722_ldo4_supply
#define palmas_ti913_ldo6_supply as3722_ldo1_supply
#define palmas_ti913_ldo7_supply as3722_ldo10_supply
#define palmas_ti913_ldo8_supply as3722_ldo3_supply
#define palmas_ti913_ldo9_supply as3722_ldo6_supply
#define palmas_ti913_ldoln_supply as3722_ldo7_supply
#define palmas_ti913_ldousb_supply as3722_ldo11_supply

static struct regulator_consumer_supply palmas_ti913_regen1_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sys_1s_cdc", NULL),
	REGULATOR_SUPPLY("vdd_sys_1s_bl", NULL),
	REGULATOR_SUPPLY("micvdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("vddio_hv", "NULL"),
};

PALMAS_PDATA_INIT(ti913_smps123, 900, 1400, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(ti913_smps45, 900, 1400, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(ti913_smps6, 1800, 1800, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(ti913_smps7, 900, 1350, NULL, 1, 1, 1, NORMAL);
PALMAS_PDATA_INIT(ti913_smps9, 1050, 1050, NULL, 0, 0, 0, NORMAL);
PALMAS_PDATA_INIT(ti913_ldo1, 1050, 1250, palmas_rails(ti913_smps7),
		1, 1, 1, 0);
PALMAS_PDATA_INIT(ti913_ldo2, 1200, 1200, palmas_rails(ti913_smps6),
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldo3, 3100, 3100, NULL,
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldo4, 1200, 1200, palmas_rails(ti913_smps6),
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldo5, 2700, 2700, NULL,
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldo6, 1800, 1800, NULL,
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldo7, 2700, 2700, NULL,
		0, 0, 1, 0);
/* TODO PALMAS_PDATA_INIT(ti913_ldo8, 1000, 1100, NULL, 1, 1, 1, NORMAL);*/
PALMAS_PDATA_INIT(ti913_ldo9, 1800, 3300, NULL,
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldoln, 1050, 1050, palmas_rails(ti913_smps6),
		0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_ldousb, 1800, 1800, NULL, 0, 0, 1, 0);
PALMAS_PDATA_INIT(ti913_regen1, 3000, 3300, NULL, 1, 1, 1, 0);

#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname
static struct regulator_init_data *ardbeg_1735_reg_data[PALMAS_NUM_REGS] = {
	NULL,
	PALMAS_REG_PDATA(ti913_smps123),
	NULL,
	PALMAS_REG_PDATA(ti913_smps45),
	NULL,
	PALMAS_REG_PDATA(ti913_smps6),
	PALMAS_REG_PDATA(ti913_smps7),
	NULL,
	PALMAS_REG_PDATA(ti913_smps9),
	NULL,
	PALMAS_REG_PDATA(ti913_ldo1),
	PALMAS_REG_PDATA(ti913_ldo2),
	PALMAS_REG_PDATA(ti913_ldo3),
	PALMAS_REG_PDATA(ti913_ldo4),
	PALMAS_REG_PDATA(ti913_ldo5),
	PALMAS_REG_PDATA(ti913_ldo6),
	PALMAS_REG_PDATA(ti913_ldo7),
	NULL, /* TODO PALMAS_REG_PDATA(ti913_ldo8),*/
	PALMAS_REG_PDATA(ti913_ldo9),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	PALMAS_REG_PDATA(ti913_ldoln),
	PALMAS_REG_PDATA(ti913_ldousb),
	PALMAS_REG_PDATA(ti913_regen1),
	NULL,
	NULL,
	NULL,
	NULL,
};

PALMAS_REG_INIT(ti913_smps123, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_smps45, 0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REG_INIT(ti913_smps6, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_smps7, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_smps9, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo1, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo2, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo3, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo4, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo5, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo6, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldo7, 0, 0, 0, 0, 0);
/* TODO PALMAS_REG_INIT(ti913_ldo8, 0, 0, 0, 0, 0);*/
PALMAS_REG_INIT(ti913_ldo9, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldoln, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_ldousb, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(ti913_regen1, 0, 0, 0, 0, 0);

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname
static struct palmas_reg_init *ardbeg_1735_reg_init[PALMAS_NUM_REGS] = {
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps123),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps45),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps6),
	PALMAS_REG_INIT_DATA(ti913_smps7),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_smps9),
	NULL,
	PALMAS_REG_INIT_DATA(ti913_ldo1),
	PALMAS_REG_INIT_DATA(ti913_ldo2),
	PALMAS_REG_INIT_DATA(ti913_ldo3),
	PALMAS_REG_INIT_DATA(ti913_ldo4),
	PALMAS_REG_INIT_DATA(ti913_ldo5),
	PALMAS_REG_INIT_DATA(ti913_ldo6),
	PALMAS_REG_INIT_DATA(ti913_ldo7),
	NULL, /*PALMAS_REG_INIT_DATA(ti913_ldo8), */
	PALMAS_REG_INIT_DATA(ti913_ldo9),
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	PALMAS_REG_INIT_DATA(ti913_ldoln),
	PALMAS_REG_INIT_DATA(ti913_ldousb),
	PALMAS_REG_INIT_DATA(ti913_regen1),
	NULL,
	NULL,
	NULL,
	NULL,
};

static struct palmas_pinctrl_config palmas_ti913_pincfg[] = {
	PALMAS_PINMUX(POWERGOOD, POWERGOOD, DEFAULT, DEFAULT),
	PALMAS_PINMUX(VAC, VAC, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO0, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO1, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO2, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO3, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO4, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO5, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO6, GPIO, DEFAULT, DEFAULT),
	PALMAS_PINMUX(GPIO7, GPIO, DEFAULT, DEFAULT),
};

static struct palmas_pinctrl_platform_data palmas_ti913_pinctrl_pdata = {
	.pincfg = palmas_ti913_pincfg,
	.num_pinctrl = ARRAY_SIZE(palmas_ti913_pincfg),
	.dvfs1_enable = true,
	.dvfs2_enable = false,
};

static struct palmas_pmic_platform_data pmic_ti913_platform = {
	.enable_ldo8_tracking = true,
	.disabe_ldo8_tracking_suspend = true,
};

static struct palmas_platform_data palmas_ti913_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_ti913_platform,
	.use_power_off = true,
	.pinctrl_pdata = &palmas_ti913_pinctrl_pdata,
};

static struct i2c_board_info palma_ti913_device[] = {
	{
		I2C_BOARD_INFO("tps65913", 0x58),
		.irq            = INT_EXTERNAL_PMU,
		.platform_data  = &palmas_ti913_pdata,
	},
};

int __init ardbeg_tps65913_regulator_init(void)
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
		pmic_ti913_platform.reg_data[i] = ardbeg_1735_reg_data[i];
		pmic_ti913_platform.reg_init[i] = ardbeg_1735_reg_init[i];
	}

	i2c_register_board_info(4, palma_ti913_device,
			ARRAY_SIZE(palma_ti913_device));
	return 0;
}

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

#define palmas_smps9_supply as3722_sd5_supply

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
	REGULATOR_SUPPLY("vddio_hsic", "tegra-xhci"),
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
	REGULATOR_SUPPLY("hvdd_usb", "tegra-xhci"),
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
	REGULATOR_SUPPLY("micvdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5645"),
};

static struct regulator_consumer_supply palmas_regen4_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

/* TPS51632 DC-DC converter */
static struct regulator_consumer_supply tps51632_dcdc_cpu_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_init_data tps51632_cpu_init_data = {
	.constraints = {
		.min_uV = 500000,
		.max_uV = 1520000,
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY),
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					 REGULATOR_CHANGE_CONTROL |
					REGULATOR_CHANGE_VOLTAGE),
		.always_on = 1,
		.boot_on =  1,
		.apply_uV = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(tps51632_dcdc_cpu_supply),
	.consumer_supplies = tps51632_dcdc_cpu_supply,
	.supply_regulator = palmas_rails(regen2),
};

static struct tps51632_regulator_platform_data tps51632_pdata_cpu = {
	.reg_init_data = &tps51632_cpu_init_data,
	.enable_pwm = false,
	.max_voltage_uV = 1520000,
	.base_voltage_uV = 500000,
	.slew_rate_uv_per_us = 6000,
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
	.constraints = {
		.min_uV = 500000,
		.max_uV = 1520000,
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY),
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					 REGULATOR_CHANGE_CONTROL |
					REGULATOR_CHANGE_VOLTAGE),
		.always_on = 1,
		.boot_on =  1,
		.apply_uV = 0,
	},
	.num_consumer_supplies = ARRAY_SIZE(tps51632_dcdc_gpu_supply),
	.consumer_supplies = tps51632_dcdc_gpu_supply,
	.supply_regulator = palmas_rails(regen2),
};

static struct tps51632_regulator_platform_data tps51632_pdata_gpu = {
	.reg_init_data = &tps51632_init_gpu_data,
	.enable_pwm = false,
	.max_voltage_uV = 1520000,
	.base_voltage_uV = 500000,
	.slew_rate_uv_per_us = 6000,
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
//PALMAS_PDATA_INIT(ldo4, 2800, 3000, palmas_rails(smps3), 0, 0, 0);
PALMAS_PDATA_INIT(ldo4, 1500, 2100, palmas_rails(smps3), 0, 0, 0);
PALMAS_PDATA_INIT(ldo5, 1100, 1100, palmas_rails(smps6), 1, 1, 1);
PALMAS_PDATA_INIT(ldo6, 2700, 2700, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo7, 2800, 2800, NULL, 0, 0, 0);
PALMAS_PDATA_INIT(ldo8, 2800, 3000, NULL, 0, 0, 0);
//PALMAS_PDATA_INIT(ldo9, 2800, 3000, palmas_rails(smps3), 1, 1, 0);
PALMAS_PDATA_INIT(ldo9, 1500, 2100, palmas_rails(smps3), 1, 1, 0);
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

static struct pca953x_platform_data tca6416_pdata = {
	        .gpio_base = PMU_TCA6416_GPIO_BASE,
};

static const struct i2c_board_info tca6416_expander[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &tca6416_pdata,
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
	struct board_info pmu_board_info;
	int i;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	tegra_get_pmu_board_info(&pmu_board_info);

	if ((pmu_board_info.board_id == BOARD_E1733) ||
		(pmu_board_info.board_id == BOARD_E1734)) {
		i2c_register_board_info(0, tca6416_expander,
				ARRAY_SIZE(tca6416_expander));
		ardbeg_ams_regulator_init();
		regulator_has_full_constraints();
	} else if (pmu_board_info.board_id == BOARD_E1731) {
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
	} else if (pmu_board_info.board_id == BOARD_E1735) {
		ardbeg_tps65913_regulator_init();
	}

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
static struct regulator_consumer_supply fixed_reg_en_battery_e1733_supply[] = {
		REGULATOR_SUPPLY("vdd_sys_bl", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_usb0_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-otg"),
};

static struct regulator_consumer_supply fixed_reg_en_usb1_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.1"),
};

static struct regulator_consumer_supply fixed_reg_en_usb2_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.2"),
};

static struct regulator_consumer_supply fixed_reg_en_as3722_gpio2_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("hvdd_usb", "tegra-xhci"),
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-xhci"),
#endif
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("avdd_pex", "tegra-pcie"),
};

static struct regulator_consumer_supply fixed_reg_en_lcd_bl_en_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_bl_en", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_3v3_supply[] = {
	REGULATOR_SUPPLY("vddio_hv", "NULL"),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_dcdc_1v8_supply[] = {
	REGULATOR_SUPPLY("avdd_lvds0_pll", NULL),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_ds_1v8", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_e1733_cdc_1v2_supply[] = {
	REGULATOR_SUPPLY("ldoen", "tegra-snd-rt5639"),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_sys_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_spk_5v0", NULL),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5639"),
	REGULATOR_SUPPLY("spkvdd", "tegra-snd-rt5645"),
};

static struct regulator_consumer_supply fixed_reg_en_e1733_vdd_wwan_supply[] = {
	REGULATOR_SUPPLY("vdd_wwan_mdm", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_sd_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

FIXED_REG(0,	battery_e1733,	battery_e1733,
	NULL,	0,	0,	-1,
	false,	true,	0,	3300, 0);

FIXED_REG(1,	usb0_vbus,	usb0_vbus,
	NULL,	0,	0,	TEGRA_GPIO_PN4,
	true,	true,	0,	5000,	0);

FIXED_REG(2,	usb1_vbus,	usb1_vbus,
	NULL,	0,	0,	TEGRA_GPIO_PN5,
	true,	true,	0,	5000,	0);

#ifdef CONFIG_ARCH_TEGRA_12x_SOC
FIXED_REG(3,	usb2_vbus,	usb2_vbus,
	NULL,	0,	0,	TEGRA_GPIO_PFF1,
	true,	true,	0,	5000,	0);
#else
FIXED_REG(3,	usb2_vbus,	usb2_vbus,
	NULL,	0,	0,	TEGRA_GPIO_PK5,
	true,	true,	0,	5000,	0);
#endif

/* EN for LDO1/6/9/10 So keep always_ON */
FIXED_REG(4,	as3722_gpio2,	as3722_gpio2,
	NULL,	1,	1, AS3722_GPIO_BASE + AS3722_GPIO2,
	false,	true,	0,	3300, 0);

FIXED_REG(5,	lcd_bl_en,	lcd_bl_en,
	NULL,	0,	0, TEGRA_GPIO_PH2,
	false,	true,	0,	5000,	0);

FIXED_REG(6,	3v3,	3v3,
	NULL,	0,	0, AS3722_GPIO_BASE + AS3722_GPIO4,
	false,	true,	0,	3300,	0);

FIXED_REG(7,	dcdc_1v8,	dcdc_1v8,
	NULL,	0,	0,	-1,
	false,	true,	0,	1800, 0);

FIXED_REG(8,	e1733_cdc_1v2,	e1733_cdc_1v2,
	AS3722_SUPPLY(sd5),	0,	0,	PMU_TCA6416_GPIO(2),
	false,	true,	0,	1200, 0);

FIXED_REG(9,	vdd_sys_5v0,	vdd_sys_5v0,
	NULL,	1,	1,	-1,
	false,	true,	0,	5000, 0);

FIXED_REG(10,	e1733_vdd_wwan,	e1733_vdd_wwan,
	NULL,	0,	0,	AS3722_GPIO_BASE + AS3722_GPIO1,
	false,	true,	0,	3300,	0);

FIXED_REG(11,	vdd_sd,	vdd_sd,
	NULL,	0,	0,	TEGRA_GPIO_PR0,
	false,	true,	0,	3300,	0);

/* Always ON /Battery regulator */
static struct regulator_consumer_supply fixed_reg_en_battery_e1731_supply[] = {
		REGULATOR_SUPPLY("vdd_sys_bl", NULL),
		REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
		REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
		REGULATOR_SUPPLY("avdd_usb_pll", "tegra-xhci"),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_cdc_1v2_supply[] = {
	REGULATOR_SUPPLY("vdd_cdc_1v2_aud", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_lcd_1v2_supply[] = {
	REGULATOR_SUPPLY("vdd_cdc_1v2_dis", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_cdc_3v3a_supply[] = {
	REGULATOR_SUPPLY("vdd_cdc_3v3a_aud", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_usb0_5v0_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-otg"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.2"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-xhci"),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_dis_3v3a_supply[] = {
	REGULATOR_SUPPLY("vdd_dis_3v3_lcd", NULL),
	REGULATOR_SUPPLY("vdd_dis_3v3_lvds", NULL),
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply fixed_reg_en_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

FIXED_REG(0,	battery_e1731,	battery_e1731,
	NULL,	0,	0,
	-1,	false, true,	0,	3300,	0);

FIXED_REG(1,	vdd_cdc_1v2,	vdd_cdc_1v2,
	palmas_rails(smps3),	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO14,	false, true,	0,	1200,
	0);

FIXED_REG(2,	vdd_lcd_1v2,	vdd_lcd_1v2,
	palmas_rails(smps3),	1,	1,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO4,	false, true,	0,	1200,
	0);

FIXED_REG(3,	vdd_cdc_3v3a,	vdd_cdc_3v3a,
	NULL,	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO10,	false, true,	0,	3300,
	0);

FIXED_REG(4,	vdd_usb0_5v0,	vdd_usb0_5v0,
	NULL,	0,	0,
	TEGRA_GPIO_PN4,	true, true,	0,	5000,
	0);

FIXED_REG(5,	vdd_dis_3v3a,	vdd_dis_3v3a,
	NULL,	1,	1,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO3,	true, true,	0,	3300,
	0);

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
FIXED_REG(12,	vdd_hdmi_5v0,	vdd_hdmi_5v0,
	NULL,	0,	0,
	TEGRA_GPIO_PH7,	false,	true,	0,	5000,	5000);
#else
FIXED_REG(12,	vdd_hdmi_5v0,	vdd_hdmi_5v0,
	NULL,	0,	0,
	TEGRA_GPIO_PK6,	false,	true,	0,	5000,	5000);
#endif

/*
 * Creating fixed regulator device tables
 */
#define ADD_FIXED_REG(_name)    (&fixed_reg_en_##_name##_dev)
#define ARDBEG_E1733_COMMON_FIXED_REG	\
	ADD_FIXED_REG(battery_e1733),		\
	ADD_FIXED_REG(vdd_hdmi_5v0),		\
	ADD_FIXED_REG(usb0_vbus),		\
	ADD_FIXED_REG(usb1_vbus),		\
	ADD_FIXED_REG(usb2_vbus),		\
	ADD_FIXED_REG(as3722_gpio2),		\
	ADD_FIXED_REG(lcd_bl_en),		\
	ADD_FIXED_REG(3v3),			\
	ADD_FIXED_REG(dcdc_1v8),		\
	ADD_FIXED_REG(vdd_sys_5v0),		\
	ADD_FIXED_REG(e1733_cdc_1v2),		\
	ADD_FIXED_REG(e1733_vdd_wwan),		\
	ADD_FIXED_REG(vdd_sd),

static struct platform_device *fixed_reg_devs_e1733[] = {
	ARDBEG_E1733_COMMON_FIXED_REG
};

#define ARBDEG_1731_COMMON_FIXED_REG	\
	ADD_FIXED_REG(battery_e1731),		\
	ADD_FIXED_REG(vdd_cdc_1v2),		\
	ADD_FIXED_REG(vdd_lcd_1v2),		\
	ADD_FIXED_REG(vdd_cdc_3v3a),		\
	ADD_FIXED_REG(vdd_usb0_5v0),		\
	ADD_FIXED_REG(vdd_dis_3v3a),		\
	ADD_FIXED_REG(vdd_hdmi_5v0),

static struct platform_device *pfixed_reg_devs[] = {
	ARBDEG_1731_COMMON_FIXED_REG
};

static int __init ardbeg_fixed_regulator_init(void)
{
	struct board_info pmu_board_info;

	if (!of_machine_is_compatible("nvidia,ardbeg"))
		return 0;

	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_E1731)
		return platform_add_devices(pfixed_reg_devs,
			ARRAY_SIZE(pfixed_reg_devs));
	else if (pmu_board_info.board_id == BOARD_E1733)
		return platform_add_devices(fixed_reg_devs_e1733,
			ARRAY_SIZE(fixed_reg_devs_e1733));

	return 0;
}

subsys_initcall_sync(ardbeg_fixed_regulator_init);
