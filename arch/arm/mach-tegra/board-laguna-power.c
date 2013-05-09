/*
 * arch/arm/mach-tegra/board-laguna-power.c
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
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>
#include <linux/gpio.h>
#include <linux/regulator/userspace-consumer.h>

#include <asm/mach-types.h>

#include <mach/irqs.h>
#include <mach/edp.h>
#include <mach/gpio-tegra.h>

#include "cpu-tegra.h"
#include "pm.h"
#include "tegra-board-id.h"
#include "board.h"
#include "gpio-names.h"
#include "board-common.h"
#include "board-laguna.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"
#include "tegra11_soctherm.h"
#include "iomap.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

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
	REGULATOR_SUPPLY("vddio_cam", "tegra_camera"),
	REGULATOR_SUPPLY("vdd_cam1_1v8_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam2_1v8_cam", NULL),
};

static struct regulator_consumer_supply as3722_ldo2_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
};

static struct regulator_consumer_supply as3722_ldo3_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply as3722_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_2v7_hv", NULL),
	REGULATOR_SUPPLY("avdd_cam1_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam2_cam", NULL),
	REGULATOR_SUPPLY("avdd_cam3_cam", NULL),
};

static struct regulator_consumer_supply as3722_ldo5_supply[] = {
	REGULATOR_SUPPLY("vdd_1v2_cam", NULL),
};

static struct regulator_consumer_supply as3722_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

static struct regulator_consumer_supply as3722_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_cam_1v1_cam", NULL),
};

static struct regulator_consumer_supply as3722_ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd_ts_3v0b_dis", NULL),
};

static struct regulator_consumer_supply as3722_ldo10_supply[] = {
	REGULATOR_SUPPLY("avdd_af1_cam", NULL),
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
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avddio_pex_pll", NULL),
	REGULATOR_SUPPLY("dvddio_pex", NULL),
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.2"),
};

static struct regulator_consumer_supply as3722_sd5_supply[] = {
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sys_2", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	/* emmc 1.8v misssing
	keyboard & touchpad 1.8v missing */
};

static struct regulator_consumer_supply as3722_sd6_supply[] = {
	REGULATOR_SUPPLY("vdd_gpu", NULL),
};

static struct regulator_init_data as3722_ldo0 = {
	.constraints = {
		.min_uV = 1050000,
		.max_uV = 1250000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo0_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo0_supply),
};

static struct regulator_init_data as3722_ldo1 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo1_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo1_supply),
};

static struct regulator_init_data as3722_ldo2 = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 1200000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo2_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo2_supply),
};

static struct regulator_init_data as3722_ldo3 = {
	.constraints = {
		.min_uV = 1000000,
		.max_uV = 1000000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo3_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo3_supply),
};

static struct regulator_init_data as3722_ldo4 = {
	.constraints = {
		.min_uV = 2700000,
		.max_uV = 2700000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 0,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo4_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo4_supply),
};

static struct regulator_init_data as3722_ldo5 = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 1200000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 0,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo5_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo5_supply),
};

static struct regulator_init_data as3722_ldo6 = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 0,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo6_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo6_supply),
};

static struct regulator_init_data as3722_ldo7 = {
	.constraints = {
		.min_uV = 1050000,
		.max_uV = 1050000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 0,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo7_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo7_supply),
};

static struct regulator_init_data as3722_ldo9 = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo9_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo9_supply),
};

static struct regulator_init_data as3722_ldo10 = {
	.constraints = {
		.min_uV = 2700000,
		.max_uV = 2700000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 0,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo10_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo10_supply),
};

static struct regulator_init_data as3722_ldo11 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_CURRENT,
		.always_on = false,
		.boot_on = 0,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_ldo11_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_ldo11_supply),
};


static struct regulator_init_data as3722_sd0 = {
	.constraints = {
		.min_uV = 1000000,
		.max_uV = 1000000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_FAST,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_sd0_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_sd0_supply),
};

static struct regulator_init_data as3722_sd1 = {
	.constraints = {
		.min_uV = 1000000,
		.max_uV = 1000000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_FAST,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 0,
	},
	.consumer_supplies = as3722_sd1_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_sd1_supply),
};

static struct regulator_init_data as3722_sd2 = {
	.constraints = {
		.min_uV = 1350000,
		.max_uV = 1350000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_FAST,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_sd2_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_sd2_supply),
};

static struct regulator_init_data as3722_sd4 = {
	.constraints = {
		.min_uV = 1050000,
		.max_uV = 1050000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_FAST,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE,
		.always_on = false,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_sd4_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_sd4_supply),
};

static struct regulator_init_data as3722_sd5 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_FAST,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_sd5_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_sd5_supply),
};

static struct regulator_init_data as3722_sd6 = {
	.constraints = {
		.min_uV = 1000000,
		.max_uV = 1000000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_FAST,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE,
		.always_on = true,
		.boot_on = 1,
		.apply_uV = 1,
	},
	.consumer_supplies = as3722_sd6_supply,
	.num_consumer_supplies = ARRAY_SIZE(as3722_sd6_supply),
};

static struct as3722_reg_init as3722_core_init_data[] = {
	/* disable all regulators */
#if 0
	AS3722_REG_INIT(AS3722_SD_CONTROL_REG, 0x00),
	AS3722_REG_INIT(AS3722_LDOCONTROL0_REG, 0x00),
	AS3722_REG_INIT(AS3722_LDOCONTROL1_REG, 0x00),
	/* set to lowest voltage output */
	AS3722_REG_INIT(AS3722_SD0_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_SD1_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_SD2_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_SD3_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_SD4_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_SD5_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_SD6_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO0_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO1_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO2_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO3_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO4_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO5_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO6_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO7_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO9_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO10_VOLTAGE_REG, 0x01),
	AS3722_REG_INIT(AS3722_LDO11_VOLTAGE_REG, 0x01),
#endif
	{.reg = AS3722_REG_INIT_TERMINATE},
};

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
		.invert     = AS3722_GPIO_CFG_INVERT, /* don't go into LP0 */
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

static struct as3722_platform_data as3722_pdata = {
	.reg_init[AS3722_LDO0] = &as3722_ldo0,
	.reg_init[AS3722_LDO1] = &as3722_ldo1,
	.reg_init[AS3722_LDO2] = &as3722_ldo2,
	.reg_init[AS3722_LDO3] = &as3722_ldo3,
	.reg_init[AS3722_LDO4] = &as3722_ldo4,
	.reg_init[AS3722_LDO5] = &as3722_ldo5,
	.reg_init[AS3722_LDO6] = &as3722_ldo6,
	.reg_init[AS3722_LDO7] = &as3722_ldo7,
	.reg_init[AS3722_LDO9] = &as3722_ldo9,
	.reg_init[AS3722_LDO10] = &as3722_ldo10,
	.reg_init[AS3722_LDO11] = &as3722_ldo11,

	.reg_init[AS3722_SD0] = &as3722_sd0,
	.reg_init[AS3722_SD1] = &as3722_sd1,
	.reg_init[AS3722_SD2] = &as3722_sd2,
	.reg_init[AS3722_SD4] = &as3722_sd4,
	.reg_init[AS3722_SD5] = &as3722_sd5,
	.reg_init[AS3722_SD6] = &as3722_sd6,

	.core_init_data = &as3722_core_init_data[0],
	.gpio_base = AS3722_GPIO_BASE,
	.irq_base = AS3722_IRQ_BASE,
	/* .irq_type = IRQF_TRIGGER_FALLING, */
	.use_internal_int_pullup = 0,
	.use_internal_i2c_pullup = 0,
	.num_gpio_cfgs = ARRAY_SIZE(as3722_gpio_cfgs),
	.gpio_cfgs     = as3722_gpio_cfgs,
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


static struct i2c_board_info __initdata as3722_regulators[] = {
	{
		I2C_BOARD_INFO("as3722", 0x40),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_EXTERNAL_PMU,
		.platform_data = &as3722_pdata,
	},
};

int __init laguna_as3722_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;


	/* AS3722: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	printk(KERN_INFO "%s: i2c_register_board_info\n",
			__func__);
	i2c_register_board_info(4, as3722_regulators,
			ARRAY_SIZE(as3722_regulators));
	i2c_register_board_info(0, tca6416_expander,
			ARRAY_SIZE(tca6416_expander));
	return 0;
}

static struct tegra_suspend_platform_data laguna_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 2000,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 2000,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 1000,
};

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param laguna_cl_dvfs_param = {
	.sample_rate = 12500,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};
#endif

/* TPS51632: fixed 10mV steps from 600mV to 1400mV, with offset 0x23 */
#define PMU_CPU_VDD_MAP_SIZE ((1400000 - 600000) / 10000 + 1)
static struct voltage_reg_map pmu_cpu_vdd_map[PMU_CPU_VDD_MAP_SIZE];
static inline void fill_reg_map(void)
{
	int i;
	for (i = 0; i < PMU_CPU_VDD_MAP_SIZE; i++) {
		pmu_cpu_vdd_map[i].reg_value = i + 0x23;
		pmu_cpu_vdd_map[i].reg_uV = 600000 + 10000 * i;
	}
}

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
static struct tegra_cl_dvfs_platform_data laguna_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0x86,
		.reg = 0x00,
	},
	.vdd_map = pmu_cpu_vdd_map,
	.vdd_map_size = PMU_CPU_VDD_MAP_SIZE,

	.cfg_param = &laguna_cl_dvfs_param,
};

static int __init laguna_cl_dvfs_init(void)
{
	fill_reg_map();
	tegra_cl_dvfs_device.dev.platform_data = &laguna_cl_dvfs_data;
	platform_device_register(&tegra_cl_dvfs_device);

	return 0;
}
#endif

/* Always ON /Battery regulator */
static struct regulator_consumer_supply fixed_reg_battery_supply[] = {
	REGULATOR_SUPPLY("vdd_sys_bl", NULL),
};

/* EN_USB1_VBUS From TEGRA GPIO PN4 */
static struct regulator_consumer_supply fixed_reg_usb1_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
};

/* EN_USB3_VBUS From TEGRA GPIO PK6 */
static struct regulator_consumer_supply fixed_reg_usb3_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.2"),
	REGULATOR_SUPPLY("usb_vbus", "tegra-xhci"),
};


/* Gated by PMU_REGEN3 From AMS7230 GPIO3*/
static struct regulator_consumer_supply fixed_reg_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

/* LCD_BL_EN GMI_AD10 */
static struct regulator_consumer_supply fixed_reg_lcd_bl_en_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_bl_en", NULL),
};

/* GPIO ?*/
static struct regulator_consumer_supply fixed_reg_3v3_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_hdmi", "NULL"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_3v3_pex", NULL),
	REGULATOR_SUPPLY("avdd_3v3_pex_pll", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_hv", "NULL"),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_lcd", NULL),
};

static struct regulator_consumer_supply fixed_reg_dcdc_1v8_supply[] = {
	REGULATOR_SUPPLY("avdd_lvds0_pll", NULL),
	REGULATOR_SUPPLY("vdd_utmip_pll", NULL),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_ds_1v8", NULL),
};

/* gated by GPIO EXP GPIO0 */
static struct regulator_consumer_supply fixed_reg_dcdc_1v2_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
};


/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG(_id, _var, _name, _always_on, _boot_on,	\
		_gpio_nr, _open_drain, _active_high, _boot_state, _millivolts)	\
static struct regulator_init_data ri_data_##_var =		\
{								\
	.num_consumer_supplies =				\
	ARRAY_SIZE(fixed_reg_##_name##_supply),		\
	.consumer_supplies = fixed_reg_##_name##_supply,	\
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
static struct fixed_voltage_config fixed_reg_##_var##_pdata =	\
{								\
	.supply_name = FIXED_SUPPLY(_name),			\
	.microvolts = _millivolts * 1000,			\
	.gpio = _gpio_nr,					\
	.gpio_is_open_drain = _open_drain,			\
	.enable_high = _active_high,				\
	.enabled_at_boot = _boot_state,				\
	.init_data = &ri_data_##_var,				\
};								\
static struct platform_device fixed_reg_##_var##_dev = {	\
	.name = "reg-fixed-voltage",				\
	.id = _id,						\
	.dev = {						\
		.platform_data = &fixed_reg_##_var##_pdata,	\
	},							\
}

FIXED_REG(0,	battery,	battery,	0,	0,
		-1,	false, true,	0,	8400);

FIXED_REG(1,	vdd_hdmi_5v0,	vdd_hdmi_5v0,	0,	0,
		TEGRA_GPIO_PK1,	false,	true,	0,	5000);

FIXED_REG(2,	usb1_vbus,	usb1_vbus,	0,	0,
		TEGRA_GPIO_PN4,	true,	true,	0,	5000);

FIXED_REG(3,	usb3_vbus,	usb3_vbus,	0,	0,
		TEGRA_GPIO_PK6,	true,	true,	0,	5000);

FIXED_REG(4,	lcd_bl_en,	lcd_bl_en,	0,	0,
		TEGRA_GPIO_PH2,	false,	true,	0,	5000);

FIXED_REG(5,	3v3,		3v3,		0,	0,
		-1,	false,	true,	0,	3300);

FIXED_REG(6,	dcdc_1v8,	dcdc_1v8,	0,	0,
		-1,	false,	true,	0,	1800);

FIXED_REG(7,    dcdc_1v2,       dcdc_1v2,       0,      0,
		PMU_TCA6416_GPIO_BASE,     false,  true,   0,      1200);
/*
 * Creating the fixed regulator device tables
 */

#define ADD_FIXED_REG(_name)    (&fixed_reg_##_name##_dev)

#define LAGUNA_COMMON_FIXED_REG			\
	ADD_FIXED_REG(battery),			\
	ADD_FIXED_REG(vdd_hdmi_5v0),		\
	ADD_FIXED_REG(usb1_vbus),		\
	ADD_FIXED_REG(usb3_vbus),		\
	ADD_FIXED_REG(lcd_bl_en),		\
	ADD_FIXED_REG(3v3),			\
	ADD_FIXED_REG(dcdc_1v8),		\
	ADD_FIXED_REG(dcdc_1v2),

/* Gpio switch regulator platform data for laguna */
static struct platform_device *fixed_reg_devs_pm360[] = {
	LAGUNA_COMMON_FIXED_REG
};


static int __init laguna_fixed_regulator_init(void)
{

	if (!of_machine_is_compatible("nvidia,ardbeg"))
		return 0;

	return platform_add_devices(fixed_reg_devs_pm360,
			ARRAY_SIZE(fixed_reg_devs_pm360));
}

subsys_initcall_sync(laguna_fixed_regulator_init);

int __init laguna_regulator_init(void)
{

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	laguna_cl_dvfs_init();
#endif
	laguna_as3722_regulator_init();

	return 0;
}

int __init laguna_suspend_init(void)
{
	tegra_init_suspend(&laguna_suspend_data);
	return 0;
}

int __init laguna_edp_init(void)
{
#ifdef CONFIG_TEGRA_EDP_LIMITS
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 15000;

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
#endif
	return 0;
}


static struct soctherm_platform_data laguna_soctherm_data = {
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 100000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 102000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
		},
		[THERM_PLL] = {
			.zone_enable = true,
		},
	},
	.throttle = {
		[THROTTLE_HEAVY] = {
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = 1,
				},
			},
		},
	},
};

int __init laguna_soctherm_init(void)
{
	tegra_platform_edp_init(laguna_soctherm_data.therm[THERM_CPU].trips,
			&laguna_soctherm_data.therm[THERM_CPU].num_trips,
			8000); /* edp temperature margin */
	tegra_add_tj_trips(laguna_soctherm_data.therm[THERM_CPU].trips,
			&laguna_soctherm_data.therm[THERM_CPU].num_trips);

	return tegra11_soctherm_init(&laguna_soctherm_data);
}
