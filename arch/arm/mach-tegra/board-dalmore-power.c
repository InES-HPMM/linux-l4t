/*
 * arch/arm/mach-tegra/board-dalmore-power.c
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
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max77663-core.h>
#include <linux/mfd/palmas.h>
#include <linux/mfd/tps65090.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/regulator/tps65090-regulator.h>
#include <linux/regulator/tps51632-regulator.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/userspace-consumer.h>

#include <asm/mach-types.h>
#include <linux/power/sbs-battery.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/edp.h>
#include <mach/gpio-tegra.h>

#include "cpu-tegra.h"
#include "pm.h"
#include "tegra-board-id.h"
#include "board.h"
#include "gpio-names.h"
#include "board-dalmore.h"
#include "tegra_cl_dvfs.h"
#include "devices.h"
#include "tegra11_soctherm.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)
#define TPS65090_CHARGER_INT	TEGRA_GPIO_PJ0
/*TPS65090 consumer rails */
static struct regulator_consumer_supply tps65090_dcdc1_supply[] = {
	REGULATOR_SUPPLY("vdd_sys_5v0", NULL),
	REGULATOR_SUPPLY("vdd_spk", NULL),
	REGULATOR_SUPPLY("vdd_sys_modem_5v0", NULL),
	REGULATOR_SUPPLY("vdd_sys_cam_5v0", NULL),
};

static struct regulator_consumer_supply tps65090_dcdc2_supply[] = {
	REGULATOR_SUPPLY("vdd_sys_3v3", NULL),
	REGULATOR_SUPPLY("vddio_hv", "tegradc.1"),
	REGULATOR_SUPPLY("pwrdet_hv", NULL),
	REGULATOR_SUPPLY("vdd_sys_ds_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sys_nfc_3v3", NULL),
	REGULATOR_SUPPLY("vdd_hv_nfc_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sys_cam_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sys_sensor_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sys_audio_3v3", NULL),
	REGULATOR_SUPPLY("vdd_sys_dtv_3v3", NULL),
	REGULATOR_SUPPLY("vcc", "0-007c"),
	REGULATOR_SUPPLY("vcc", "0-0030"),
	REGULATOR_SUPPLY("vin", "2-0030"),
};

static struct regulator_consumer_supply tps65090_dcdc3_supply[] = {
	REGULATOR_SUPPLY("vdd_ao", NULL),
};

static struct regulator_consumer_supply tps65090_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_sby_5v0", NULL),
};

static struct regulator_consumer_supply tps65090_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_sby_3v3", NULL),
};

static struct regulator_consumer_supply tps65090_fet1_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
};

static struct regulator_consumer_supply tps65090_fet3_supply[] = {
	REGULATOR_SUPPLY("vdd_modem_3v3", NULL),
};

static struct regulator_consumer_supply tps65090_fet4_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
	REGULATOR_SUPPLY("avdd", "spi3.2"),
};

static struct regulator_consumer_supply tps65090_fet5_supply[] = {
	REGULATOR_SUPPLY("vdd_lvds", NULL),
};

static struct regulator_consumer_supply tps65090_fet6_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply tps65090_fet7_supply[] = {
	REGULATOR_SUPPLY("vdd_wifi_3v3", "bcm4329_wlan.1"),
	REGULATOR_SUPPLY("vdd_gps_3v3", "reg-userspace-consumer.2"),
	REGULATOR_SUPPLY("vdd_bt_3v3", "reg-userspace-consumer.1"),
};

#define TPS65090_PDATA_INIT(_id, _name, _supply_reg,			\
		_always_on, _boot_on, _apply_uV, _en_ext_ctrl, _gpio)	\
static struct regulator_init_data ri_data_##_name =			\
{									\
	.supply_regulator = _supply_reg,				\
	.constraints = {						\
		.name = tps65090_rails(_id),				\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
				     REGULATOR_MODE_STANDBY),		\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
				   REGULATOR_CHANGE_STATUS |		\
				   REGULATOR_CHANGE_VOLTAGE),		\
		.always_on = _always_on,				\
		.boot_on = _boot_on,					\
		.apply_uV = _apply_uV,					\
	},								\
	.num_consumer_supplies =					\
		ARRAY_SIZE(tps65090_##_name##_supply),			\
	.consumer_supplies = tps65090_##_name##_supply,			\
};									\
static struct tps65090_regulator_platform_data				\
			tps65090_regulator_pdata_##_name =		\
{									\
	.id = TPS65090_REGULATOR_##_id,					\
	.enable_ext_control = _en_ext_ctrl,				\
	.gpio = _gpio,							\
	.reg_init_data = &ri_data_##_name ,				\
}

TPS65090_PDATA_INIT(DCDC1, dcdc1, NULL, 1, 1, 0, false, -1);
TPS65090_PDATA_INIT(DCDC2, dcdc2, NULL, 1, 1, 0, false, -1);
TPS65090_PDATA_INIT(DCDC3, dcdc3, NULL, 1, 1, 0, false, -1);
TPS65090_PDATA_INIT(LDO1, ldo1, NULL, 1, 1, 0, false, -1);
TPS65090_PDATA_INIT(LDO2, ldo2, NULL, 1, 1, 0, false, -1);
TPS65090_PDATA_INIT(FET1, fet1, NULL, 0, 0, 0, false, -1);
TPS65090_PDATA_INIT(FET3, fet3, tps65090_rails(DCDC2), 0, 0, 0, false, -1);
TPS65090_PDATA_INIT(FET4, fet4, tps65090_rails(DCDC2), 0, 0, 0, false, -1);
TPS65090_PDATA_INIT(FET5, fet5, tps65090_rails(DCDC2), 0, 0, 0, false, -1);
TPS65090_PDATA_INIT(FET6, fet6, tps65090_rails(DCDC2), 0, 0, 0, false, -1);
TPS65090_PDATA_INIT(FET7, fet7, tps65090_rails(DCDC2), 0, 0, 0, false, -1);

#define ADD_TPS65090_REG(_name) (&tps65090_regulator_pdata_##_name)
static struct tps65090_regulator_platform_data *tps65090_reg_pdata[] = {
	ADD_TPS65090_REG(dcdc1),
	ADD_TPS65090_REG(dcdc2),
	ADD_TPS65090_REG(dcdc3),
	ADD_TPS65090_REG(ldo1),
	ADD_TPS65090_REG(ldo2),
	ADD_TPS65090_REG(fet1),
	ADD_TPS65090_REG(fet3),
	ADD_TPS65090_REG(fet4),
	ADD_TPS65090_REG(fet5),
	ADD_TPS65090_REG(fet6),
	ADD_TPS65090_REG(fet7),
};

static struct tps65090_charger_data bcharger_pdata = {
	.irq_base = TPS65090_TEGRA_IRQ_BASE,
	.update_status = sbs_update,
};

static struct tps65090_platform_data tps65090_pdata = {
	.irq_base = TPS65090_TEGRA_IRQ_BASE,
	.irq_flag = IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
	.num_reg_pdata =  ARRAY_SIZE(tps65090_reg_pdata),
	.reg_pdata = tps65090_reg_pdata,
	.charger_pdata = &bcharger_pdata,
};

/* MAX77663 consumer rails */
static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("vddio_ddr0", NULL),
	REGULATOR_SUPPLY("vddio_ddr1", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_cam", "tegra_camera"),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vdd_emmc", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("avdd_audio_1v8", NULL),
	REGULATOR_SUPPLY("vdd_audio_1v8", NULL),
	REGULATOR_SUPPLY("vddio_modem", NULL),
	REGULATOR_SUPPLY("vddio_modem_1v8", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_bb_1v8", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("vdd_sensor_1v8", NULL),
	REGULATOR_SUPPLY("vdd_mic_1v8", NULL),
	REGULATOR_SUPPLY("vdd_nfc_1v8", NULL),
	REGULATOR_SUPPLY("vdd_ds_1v8", NULL),
	REGULATOR_SUPPLY("vdd_spi_1v8", NULL),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_com_1v8", NULL),
	REGULATOR_SUPPLY("vddio_wifi_1v8", "bcm4329_wlan.1"),
	REGULATOR_SUPPLY("vdd_gps_1v8", "reg-userspace-consumer.2"),
	REGULATOR_SUPPLY("vddio_bt_1v8", "reg-userspace-consumer.1"),
	REGULATOR_SUPPLY("vdd_dtv_1v8", NULL),
	REGULATOR_SUPPLY("vlogic", "0-0069"),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vcore_emmc", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegra_camera"),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_sensor_2v85", NULL),
	REGULATOR_SUPPLY("vdd_als", NULL),
	REGULATOR_SUPPLY("vdd", "0-004c"),
	REGULATOR_SUPPLY("vdd", "0-0069"),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avddio_usb", "tegra-ehci.2"),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_dsi_csi", "tegra_camera"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.1"),
	REGULATOR_SUPPLY("vddio_hsic", "tegra-ehci.2"),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
	REGULATOR_SUPPLY("vddio_bb_hsic", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
};

/* FIXME!! Put the device address of camera */
static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_cam1", NULL),
	REGULATOR_SUPPLY("avdd_2v8_cam_af", NULL),
	REGULATOR_SUPPLY("vana", "2-0036"),
};

/* FIXME!! Put the device address of camera */
static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_cam2", NULL),
	REGULATOR_SUPPLY("avdd", "2-0010"),
};

static struct max77663_regulator_fps_cfg max77663_fps_cfgs[] = {
	{
		.src = FPS_SRC_0,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_1,
		.en_src = FPS_EN_SRC_EN1,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
	{
		.src = FPS_SRC_2,
		.en_src = FPS_EN_SRC_EN0,
		.time_period = FPS_TIME_PERIOD_DEF,
	},
};

#define MAX77663_PDATA_INIT(_rid, _id, _min_uV, _max_uV, _supply_reg,	\
		_always_on, _boot_on, _apply_uV,			\
		_fps_src, _fps_pu_period, _fps_pd_period, _flags)	\
	static struct regulator_init_data max77663_regulator_idata_##_id = {   \
		.supply_regulator = _supply_reg,			\
		.constraints = {					\
			.name = max77663_rails(_id),			\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
			.apply_uV = _apply_uV,				\
		},							\
		.num_consumer_supplies =				\
			ARRAY_SIZE(max77663_##_id##_supply),		\
		.consumer_supplies = max77663_##_id##_supply,		\
	};								\
static struct max77663_regulator_platform_data max77663_regulator_pdata_##_id =\
{									\
		.reg_init_data = &max77663_regulator_idata_##_id,	\
		.id = MAX77663_REGULATOR_ID_##_rid,			\
		.fps_src = _fps_src,					\
		.fps_pu_period = _fps_pu_period,			\
		.fps_pd_period = _fps_pd_period,			\
		.fps_cfgs = max77663_fps_cfgs,				\
		.flags = _flags,					\
	}

MAX77663_PDATA_INIT(SD0, sd0,  900000, 1400000, tps65090_rails(DCDC3), 1, 1, 0,
		    FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(SD1, sd1,  1200000, 1200000, tps65090_rails(DCDC3), 1, 1, 1,
		    FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(SD2, sd2,  1800000, 1800000, tps65090_rails(DCDC3), 1, 1, 1,
		    FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(SD3, sd3,  2850000, 2850000, tps65090_rails(DCDC3), 1, 1, 1,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO0, ldo0, 1050000, 1050000, max77663_rails(sd2), 1, 1, 1,
		    FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO1, ldo1, 1050000, 1050000, max77663_rails(sd2), 0, 0, 1,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO2, ldo2, 2850000, 2850000, tps65090_rails(DCDC2), 1, 1,
		    1, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO3, ldo3, 1050000, 1050000, max77663_rails(sd2), 1, 1, 1,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO4, ldo4, 1100000, 1100000, tps65090_rails(DCDC2), 1, 1,
		    1, FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO5, ldo5, 1200000, 1200000, max77663_rails(sd2), 0, 1, 1,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO6, ldo6, 1800000, 3300000, tps65090_rails(DCDC2), 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO7, ldo7, 2800000, 2800000, tps65090_rails(DCDC2), 0, 0, 1,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO8, ldo8, 2800000, 2800000, tps65090_rails(DCDC2), 0, 1, 1,
		    FPS_SRC_1, -1, -1, 0);

#define MAX77663_REG(_id, _data) (&max77663_regulator_pdata_##_data)

static struct max77663_regulator_platform_data *max77663_reg_pdata[] = {
	MAX77663_REG(SD0, sd0),
	MAX77663_REG(SD1, sd1),
	MAX77663_REG(SD2, sd2),
	MAX77663_REG(SD3, sd3),
	MAX77663_REG(LDO0, ldo0),
	MAX77663_REG(LDO1, ldo1),
	MAX77663_REG(LDO2, ldo2),
	MAX77663_REG(LDO3, ldo3),
	MAX77663_REG(LDO4, ldo4),
	MAX77663_REG(LDO5, ldo5),
	MAX77663_REG(LDO6, ldo6),
	MAX77663_REG(LDO7, ldo7),
	MAX77663_REG(LDO8, ldo8),
};

static struct max77663_gpio_config max77663_gpio_cfgs[] = {
	{
		.gpio = MAX77663_GPIO0,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO1,
		.dir = GPIO_DIR_IN,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.pull_up = GPIO_PU_ENABLE,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO2,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.pull_up = GPIO_PU_ENABLE,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO3,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.pull_up = GPIO_PU_ENABLE,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO4,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_HIGH,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_ENABLE,
	},
	{
		.gpio = MAX77663_GPIO5,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_PUSH_PULL,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO6,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
	{
		.gpio = MAX77663_GPIO7,
		.dir = GPIO_DIR_OUT,
		.dout = GPIO_DOUT_LOW,
		.out_drv = GPIO_OUT_DRV_OPEN_DRAIN,
		.alternate = GPIO_ALT_DISABLE,
	},
};

static struct max77663_platform_data max77663_pdata = {
	.irq_base	= MAX77663_IRQ_BASE,
	.gpio_base	= MAX77663_GPIO_BASE,

	.num_gpio_cfgs	= ARRAY_SIZE(max77663_gpio_cfgs),
	.gpio_cfgs	= max77663_gpio_cfgs,

	.regulator_pdata = max77663_reg_pdata,
	.num_regulator_pdata = ARRAY_SIZE(max77663_reg_pdata),

	.rtc_i2c_addr	= 0x68,

	.use_power_off	= false,
};

static struct i2c_board_info __initdata max77663_regulators[] = {
	{
		/* The I2C address was determined by OTP factory setting */
		I2C_BOARD_INFO("max77663", 0x3c),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &max77663_pdata,
	},
};

static struct i2c_board_info __initdata tps65090_regulators[] = {
	{
		I2C_BOARD_INFO("tps65090", 0x48),
		.platform_data	= &tps65090_pdata,
	},
};

/* TPS51632 DC-DC converter */
static struct regulator_consumer_supply tps51632_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_init_data tps51632_init_data = {
	.constraints = {						\
		.min_uV = 500000,					\
		.max_uV = 1520000,					\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
					REGULATOR_MODE_STANDBY),	\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		.always_on = 1,						\
		.boot_on =  1,						\
		.apply_uV = 0,						\
	},								\
	.num_consumer_supplies = ARRAY_SIZE(tps51632_dcdc_supply),	\
		.consumer_supplies = tps51632_dcdc_supply,		\
};

static struct tps51632_regulator_platform_data tps51632_pdata = {
	.reg_init_data = &tps51632_init_data,		\
	.enable_pwm = false,				\
	.max_voltage_uV = 1520000,			\
	.base_voltage_uV = 500000,			\
	.slew_rate_uv_per_us = 6000,			\
};

static struct i2c_board_info __initdata tps51632_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps51632", 0x43),
		.platform_data	= &tps51632_pdata,
	},
};

/************************ Palmas based regulator ****************/
static struct regulator_consumer_supply palmas_smps12_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr3l", NULL),
	REGULATOR_SUPPLY("vcore_ddr3l", NULL),
	REGULATOR_SUPPLY("vref2_ddr3l", NULL),
};

#define palmas_smps3_supply max77663_sd2_supply
#define palmas_smps45_supply max77663_sd0_supply
#define palmas_smps457_supply max77663_sd0_supply

static struct regulator_consumer_supply palmas_smps8_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.0"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", "tegra_camera"),
	REGULATOR_SUPPLY("avdd_hdmi_pll", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_usb_pll", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avddio_usb", "tegra-ehci.2"),

};

static struct regulator_consumer_supply palmas_smps9_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.3"),
};

#define palmas_ldo1_supply max77663_ldo7_supply
#define palmas_ldo2_supply max77663_ldo8_supply
#define palmas_ldo3_supply max77663_ldo5_supply

static struct regulator_consumer_supply palmas_ldo4_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

#define palmas_ldo6_supply max77663_ldo2_supply

static struct regulator_consumer_supply palmas_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd_af_cam1", NULL),
	REGULATOR_SUPPLY("vdd", "2-000e"),
};

#define palmas_ldo8_supply max77663_ldo4_supply
#define palmas_ldo9_supply max77663_ldo6_supply

static struct regulator_consumer_supply palmas_ldoln_supply[] = {
	REGULATOR_SUPPLY("hvdd_usb", "tegra-ehci.2"),
};

static struct regulator_consumer_supply palmas_ldousb_supply[] = {
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
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

PALMAS_PDATA_INIT(smps12, 1350,  1350, tps65090_rails(DCDC3), 0, 0, 0);
PALMAS_PDATA_INIT(smps3, 1800,  1800, tps65090_rails(DCDC3), 0, 0, 0);
PALMAS_PDATA_INIT(smps45, 900,  1400, tps65090_rails(DCDC2), 1, 1, 0);
PALMAS_PDATA_INIT(smps457, 900,  1400, tps65090_rails(DCDC2), 1, 1, 0);
PALMAS_PDATA_INIT(smps8, 1050,  1050, tps65090_rails(DCDC2), 0, 1, 1);
PALMAS_PDATA_INIT(smps9, 2800,  2800, tps65090_rails(DCDC2), 1, 0, 0);
PALMAS_PDATA_INIT(ldo1, 2800,  2800, tps65090_rails(DCDC2), 0, 0, 1);
PALMAS_PDATA_INIT(ldo2, 2800,  2800, tps65090_rails(DCDC2), 0, 0, 1);
PALMAS_PDATA_INIT(ldo3, 1200,  1200, palmas_rails(smps3), 0, 0, 1);
PALMAS_PDATA_INIT(ldo4, 1800,  1800, tps65090_rails(DCDC2), 0, 0, 0);
PALMAS_PDATA_INIT(ldo6, 2850,  2850, tps65090_rails(DCDC2), 0, 0, 1);
PALMAS_PDATA_INIT(ldo7, 2800,  2800, tps65090_rails(DCDC2), 0, 0, 1);
PALMAS_PDATA_INIT(ldo8, 900,  900, tps65090_rails(DCDC3), 1, 1, 1);
PALMAS_PDATA_INIT(ldo9, 1800,  3300, palmas_rails(smps9), 0, 0, 1);
PALMAS_PDATA_INIT(ldoln, 3300, 3300, tps65090_rails(DCDC1), 0, 0, 1);
PALMAS_PDATA_INIT(ldousb, 3300,  3300, tps65090_rails(DCDC1), 0, 0, 1);

#define PALMAS_REG_PDATA(_sname) &reg_idata_##_sname

static struct regulator_init_data *dalmore_e1611_reg_data[PALMAS_NUM_REGS] = {
	PALMAS_REG_PDATA(smps12),
	NULL,
	PALMAS_REG_PDATA(smps3),
	PALMAS_REG_PDATA(smps45),
	PALMAS_REG_PDATA(smps457),
	NULL,
	NULL,
	PALMAS_REG_PDATA(smps8),
	PALMAS_REG_PDATA(smps9),
	NULL,
	PALMAS_REG_PDATA(ldo1),
	PALMAS_REG_PDATA(ldo2),
	PALMAS_REG_PDATA(ldo3),
	PALMAS_REG_PDATA(ldo4),
	NULL,
	PALMAS_REG_PDATA(ldo6),
	PALMAS_REG_PDATA(ldo7),
	PALMAS_REG_PDATA(ldo8),
	PALMAS_REG_PDATA(ldo9),
	PALMAS_REG_PDATA(ldoln),
	PALMAS_REG_PDATA(ldousb),
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
		.mode_sleep = _mode_sleep,				\
		.tstep = _tstep,					\
		.vsel = _vsel,						\
	}

PALMAS_REG_INIT(smps12, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps123, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps3, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(smps45, 0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
PALMAS_REG_INIT(smps457, 0, PALMAS_EXT_CONTROL_NSLEEP, 0, 0, 0);
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
PALMAS_REG_INIT(regen1, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(regen2, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(regen3, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(sysen1, 0, 0, 0, 0, 0);
PALMAS_REG_INIT(sysen2, 0, 0, 0, 0, 0);

#define PALMAS_REG_INIT_DATA(_sname) &reg_init_data_##_sname
static struct palmas_reg_init *dalmore_e1611_reg_init[PALMAS_NUM_REGS] = {
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
	PALMAS_REG_INIT_DATA(regen1),
	PALMAS_REG_INIT_DATA(regen2),
	PALMAS_REG_INIT_DATA(regen3),
	PALMAS_REG_INIT_DATA(sysen1),
	PALMAS_REG_INIT_DATA(sysen2),
};

static struct palmas_pmic_platform_data pmic_platform = {
	.enable_ldo8_tracking = true,
	.disabe_ldo8_tracking_suspend = true,
};

static struct palmas_platform_data palmas_pdata = {
	.gpio_base = PALMAS_TEGRA_GPIO_BASE,
	.irq_base = PALMAS_TEGRA_IRQ_BASE,
	.pmic_pdata = &pmic_platform,
	.mux_from_pdata = true,
	.pad1 = 0,
	.pad2 = 0,
	.pad3 = PALMAS_PRIMARY_SECONDARY_PAD3_DVFS1,
	.use_power_off = true,
};

static struct i2c_board_info palma_device[] = {
	{
		I2C_BOARD_INFO("tps65913", 0x58),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &palmas_pdata,
	},
};

/* EN_AVDD_USB_HDMI From PMU GP1 */
static struct regulator_consumer_supply fixed_reg_avdd_usb_hdmi_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi", "tegradc.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-udc.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.0"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.1"),
	REGULATOR_SUPPLY("avdd_usb", "tegra-ehci.2"),
	REGULATOR_SUPPLY("hvdd_usb", "tegra-ehci.2"),
};

/* EN_CAM_1v8 From PMU GP5 */
static struct regulator_consumer_supply fixed_reg_en_1v8_cam_supply[] = {
	REGULATOR_SUPPLY("dvdd_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam_1v8", NULL),
	REGULATOR_SUPPLY("vi2c", "2-0030"),
	REGULATOR_SUPPLY("vif", "2-0036"),
	REGULATOR_SUPPLY("dovdd", "2-0010"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000e"),
};

/* EN_CAM_1v8 on e1611 From PMU GP6 */
static struct regulator_consumer_supply fixed_reg_en_1v8_cam_e1611_supply[] = {
	REGULATOR_SUPPLY("dvdd_cam", NULL),
	REGULATOR_SUPPLY("vdd_cam_1v8", NULL),
	REGULATOR_SUPPLY("vi2c", "2-0030"),
	REGULATOR_SUPPLY("vif", "2-0036"),
	REGULATOR_SUPPLY("dovdd", "2-0010"),
	REGULATOR_SUPPLY("vdd_i2c", "2-000e"),
};

static struct regulator_consumer_supply fixed_reg_vdd_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_5v0", "tegradc.1"),
};

/* EN_USB1_VBUS From TEGRA GPIO PN4 PR3(T30) */
static struct regulator_consumer_supply fixed_reg_usb1_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.0"),
};

/* EN_3V3_FUSE From TEGRA GPIO PX4 */
static struct regulator_consumer_supply fixed_reg_vpp_fuse_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

/* EN_USB3_VBUS From TEGRA GPIO PM5 */
static struct regulator_consumer_supply fixed_reg_usb3_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus", "tegra-ehci.2"),
};

/* EN_1V8_TS From TEGRA_GPIO_PH5 */
static struct regulator_consumer_supply fixed_reg_dvdd_ts_supply[] = {
	REGULATOR_SUPPLY("dvdd", "spi3.2"),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _open_drain, _active_high, _boot_state, _millivolts)	\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
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

FIXED_REG(1,	avdd_usb_hdmi,	avdd_usb_hdmi,
	tps65090_rails(DCDC2),	0,	0,
	MAX77663_GPIO_BASE + MAX77663_GPIO1,	true,	true,	1,	3300);

FIXED_REG(2,	en_1v8_cam,	en_1v8_cam,
	max77663_rails(sd2),	0,	0,
	MAX77663_GPIO_BASE + MAX77663_GPIO5,	false,	true,	0,	1800);

FIXED_REG(3,	vdd_hdmi_5v0,	vdd_hdmi_5v0,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PK1,	false,	true,	0,	5000);

FIXED_REG(4,	vpp_fuse,	vpp_fuse,
	max77663_rails(sd2),	0,	0,
	TEGRA_GPIO_PX4,	false,	true,	0,	3300);

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
FIXED_REG(5,	usb1_vbus,	usb1_vbus,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PN4,	true,	true,	0,	5000);
#else
FIXED_REG(5,	usb1_vbus,	usb1_vbus,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PR3,	true,	true,	0,	5000);
#endif

FIXED_REG(6,	usb3_vbus,	usb3_vbus,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PK6,	true,	true,	0,	5000);

FIXED_REG(7,	en_1v8_cam_e1611,	en_1v8_cam_e1611,
	palmas_rails(smps3),	0,	0,
	PALMAS_TEGRA_GPIO_BASE + PALMAS_GPIO6,	false,	true,	0,	1800);

FIXED_REG(8,	dvdd_ts,	dvdd_ts,
	palmas_rails(smps3),	0,	0,
	TEGRA_GPIO_PH5,	false,	false,	1,	1800);
/*
 * Creating the fixed regulator device tables
 */

#define ADD_FIXED_REG(_name)    (&fixed_reg_##_name##_dev)

#define DALMORE_COMMON_FIXED_REG		\
	ADD_FIXED_REG(usb1_vbus),		\
	ADD_FIXED_REG(usb3_vbus),		\
	ADD_FIXED_REG(vdd_hdmi_5v0),

#define E1612_FIXED_REG				\
	ADD_FIXED_REG(avdd_usb_hdmi),		\
	ADD_FIXED_REG(en_1v8_cam),		\
	ADD_FIXED_REG(vpp_fuse),		\

#define E1611_FIXED_REG				\
	ADD_FIXED_REG(en_1v8_cam_e1611), \
	ADD_FIXED_REG(dvdd_ts),

/* Gpio switch regulator platform data for Dalmore E1611 */
static struct platform_device *fixed_reg_devs_e1611_a00[] = {
	DALMORE_COMMON_FIXED_REG
	E1611_FIXED_REG
};

/* Gpio switch regulator platform data for Dalmore E1612 */
static struct platform_device *fixed_reg_devs_e1612_a00[] = {
	DALMORE_COMMON_FIXED_REG
	E1612_FIXED_REG
};

int __init dalmore_palmas_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	int i;
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	int ret;

	ret = gpio_request(TEGRA_GPIO_PCC3, "pmic_nreswarm");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
				__func__, TEGRA_GPIO_PCC3);
	else
		gpio_direction_output(TEGRA_GPIO_PCC3, 1);
#endif
	/* TPS65913: Normal state of INT request line is LOW.
	 * configure the power management controller to trigger PMU
	 * interrupts when HIGH.
	 */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);
	for (i = 0; i < PALMAS_NUM_REGS ; i++) {
		pmic_platform.reg_data[i] = dalmore_e1611_reg_data[i];
		pmic_platform.reg_init[i] = dalmore_e1611_reg_init[i];
	}

	i2c_register_board_info(4, palma_device,
			ARRAY_SIZE(palma_device));
	return 0;
}

static int ac_online(void)
{
	return 1;
}

static struct resource dalmore_pda_resources[] = {
	[0] = {
		.name	= "ac",
	},
};

static struct pda_power_pdata dalmore_pda_data = {
	.is_ac_online	= ac_online,
};

static struct platform_device dalmore_pda_power_device = {
	.name		= "pda-power",
	.id		= -1,
	.resource	= dalmore_pda_resources,
	.num_resources	= ARRAY_SIZE(dalmore_pda_resources),
	.dev	= {
		.platform_data	= &dalmore_pda_data,
	},
};

static struct tegra_suspend_platform_data dalmore_suspend_data = {
	.cpu_timer	= 300,
	.cpu_off_timer	= 300,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x157e,
	.core_off_timer = 2000,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.min_residency_noncpu = 600,
	.min_residency_crail = 1000,
};
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param dalmore_cl_dvfs_param = {
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
static struct tegra_cl_dvfs_platform_data dalmore_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_I2C,
	.u.pmu_i2c = {
		.fs_rate = 400000,
		.slave_addr = 0x86,
		.reg = 0x00,
	},
	.vdd_map = pmu_cpu_vdd_map,
	.vdd_map_size = PMU_CPU_VDD_MAP_SIZE,

	.cfg_param = &dalmore_cl_dvfs_param,
};

static int __init dalmore_cl_dvfs_init(void)
{
	fill_reg_map();
	tegra_cl_dvfs_device.dev.platform_data = &dalmore_cl_dvfs_data;
	platform_device_register(&tegra_cl_dvfs_device);

	return 0;
}
#endif

static int __init dalmore_max77663_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */
	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	i2c_register_board_info(4, max77663_regulators,
				ARRAY_SIZE(max77663_regulators));

	return 0;
}

static struct regulator_bulk_data dalmore_gps_regulator_supply[] = {
	[0] = {
		.supply	= "vdd_gps_3v3",
	},
	[1] = {
		.supply	= "vdd_gps_1v8",
	},
};

static struct regulator_userspace_consumer_data dalmore_gps_regulator_pdata = {
	.num_supplies	= ARRAY_SIZE(dalmore_gps_regulator_supply),
	.supplies	= dalmore_gps_regulator_supply,
};

static struct platform_device dalmore_gps_regulator_device = {
	.name	= "reg-userspace-consumer",
	.id	= 2,
	.dev	= {
			.platform_data = &dalmore_gps_regulator_pdata,
	},
};

static struct regulator_bulk_data dalmore_bt_regulator_supply[] = {
	[0] = {
		.supply	= "vdd_bt_3v3",
	},
	[1] = {
		.supply	= "vddio_bt_1v8",
	},
};

static struct regulator_userspace_consumer_data dalmore_bt_regulator_pdata = {
	.num_supplies	= ARRAY_SIZE(dalmore_bt_regulator_supply),
	.supplies	= dalmore_bt_regulator_supply,
};

static struct platform_device dalmore_bt_regulator_device = {
	.name	= "reg-userspace-consumer",
	.id	= 1,
	.dev	= {
			.platform_data = &dalmore_bt_regulator_pdata,
	},
};

static int __init dalmore_fixed_regulator_init(void)
{
	struct board_info board_info;

	if (!machine_is_dalmore())
		return 0;

	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_E1611 ||
		board_info.board_id == BOARD_P2454)
		return platform_add_devices(fixed_reg_devs_e1611_a00,
				ARRAY_SIZE(fixed_reg_devs_e1611_a00));
	else
		return platform_add_devices(fixed_reg_devs_e1612_a00,
				ARRAY_SIZE(fixed_reg_devs_e1612_a00));
}
subsys_initcall_sync(dalmore_fixed_regulator_init);

static void dalmore_tps65090_init(void)
{
	int err;

	err = gpio_request(TPS65090_CHARGER_INT, "CHARGER_INT");
	if (err < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, err);
		goto fail_init_irq;
	}

	err = gpio_direction_input(TPS65090_CHARGER_INT);
	if (err < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, err);
		goto fail_init_irq;
	}

	tps65090_regulators[0].irq = gpio_to_irq(TPS65090_CHARGER_INT);
fail_init_irq:
	i2c_register_board_info(4, tps65090_regulators,
			ARRAY_SIZE(tps65090_regulators));
	return;
}

int __init dalmore_regulator_init(void)
{
	struct board_info board_info;

	dalmore_tps65090_init();
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	dalmore_cl_dvfs_init();
#endif
	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E1611 ||
		board_info.board_id == BOARD_P2454)
		dalmore_palmas_regulator_init();
	else
		dalmore_max77663_regulator_init();

	i2c_register_board_info(4, tps51632_boardinfo, 1);
	platform_device_register(&dalmore_pda_power_device);
	platform_device_register(&dalmore_bt_regulator_device);
	platform_device_register(&dalmore_gps_regulator_device);
	return 0;
}

int __init dalmore_suspend_init(void)
{
	tegra_init_suspend(&dalmore_suspend_data);
	return 0;
}

int __init dalmore_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA)
		regulator_mA = 15000;

	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);
	tegra_init_cpu_edp_limits(regulator_mA);

	regulator_mA = get_maximum_core_current_supported();
	if (!regulator_mA)
		regulator_mA = 4000;

	pr_info("%s: core regulator %d mA\n", __func__, regulator_mA);
	tegra_init_core_edp_limits(regulator_mA);

	return 0;
}

static struct soctherm_platform_data dalmore_soctherm_data = {
	.soctherm_clk_rate = 136000000,
	.tsensor_clk_rate = 500000,
	.sensor_data = {
		[TSENSE_CPU0] = {
			.enable = true,
			.tall = 16300,
			.tiddq = 1,
			.ten_count = 1,
			.tsample = 163,
			.pdiv = 10,
		},
		[TSENSE_CPU1] = {
			.enable = true,
			.tall = 16300,
			.tiddq = 1,
			.ten_count = 1,
			.tsample = 163,
			.pdiv = 10,
		},
		[TSENSE_CPU2] = {
			.enable = true,
			.tall = 16300,
			.tiddq = 1,
			.ten_count = 1,
			.tsample = 163,
			.pdiv = 10,
		},
		[TSENSE_CPU3] = {
			.enable = true,
			.tall = 16300,
			.tiddq = 1,
			.ten_count = 1,
			.tsample = 163,
			.pdiv = 10,
		},
		/* MEM0/MEM1 won't be used */
		[TSENSE_MEM0] = {
			.enable = false,
		},
		[TSENSE_MEM1] = {
			.enable = false,
		},
		[TSENSE_GPU] = {
			.enable = true,
			.tall = 16300,
			.tiddq = 1,
			.ten_count = 1,
			.tsample = 163,
			.pdiv = 10,
		},
		[TSENSE_PLLX] = {
			.enable = true,
			.tall = 16300,
			.tiddq = 1,
			.ten_count = 1,
			.tsample = 163,
			.pdiv = 10,
		},
	},
};

int __init dalmore_soctherm_init(void)
{
	return tegra11_soctherm_init(&dalmore_soctherm_data);
}
