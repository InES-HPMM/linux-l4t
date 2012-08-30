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
#include <linux/mfd/tps65090.h>
#include <linux/regulator/max77663-regulator.h>
#include <linux/regulator/tps65090-regulator.h>
#include <linux/regulator/tps51632-regulator.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>

#include "pm.h"
#include "board.h"
#include "board-dalmore.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

/*TPS65090 consumer rails */
static struct regulator_consumer_supply tps65090_dcdc1_supply[] = {
	REGULATOR_SUPPLY("vdd_spk", NULL),
	REGULATOR_SUPPLY("vdd_5v0_sys_modem", NULL),
};

static struct regulator_consumer_supply tps65090_dcdc2_supply[] = {
	REGULATOR_SUPPLY("vddio_hv", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys_ds", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys_nfc", NULL),
	REGULATOR_SUPPLY("vdd_hv_3v3_sys_nfc", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys_cam", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys_sensor", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys_audio", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys_dtv", NULL),
};

static struct regulator_consumer_supply tps65090_dcdc3_supply[] = {
	REGULATOR_SUPPLY("vdd_ao", NULL),
};

static struct regulator_consumer_supply tps65090_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sby", NULL),
};

static struct regulator_consumer_supply tps65090_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_sby", NULL),
};

static struct regulator_consumer_supply tps65090_fet1_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_bl", NULL),
};

static struct regulator_consumer_supply tps65090_fet3_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_modem", NULL),
};

static struct regulator_consumer_supply tps65090_fet4_supply[] = {
	REGULATOR_SUPPLY("avdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_3v3_ts", NULL),
};

static struct regulator_consumer_supply tps65090_fet5_supply[] = {
	REGULATOR_SUPPLY("vdd_lvds", NULL),
};

static struct regulator_consumer_supply tps65090_fet6_supply[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply tps65090_fet7_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_com", NULL),
	REGULATOR_SUPPLY("vdd_3v3_gps", NULL),
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

static struct tps65090_platform_data tps65090_pdata = {
	.irq_base = -1,
	.num_reg_pdata =  ARRAY_SIZE(tps65090_reg_pdata),
	.reg_pdata = tps65090_reg_pdata
};

/* MAX77663 consumer rails */
static struct regulator_consumer_supply max77663_sd0_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply max77663_sd1_supply[] = {
	REGULATOR_SUPPLY("vddio_ddr", NULL),
};

static struct regulator_consumer_supply max77663_sd2_supply[] = {
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("avdd_1v8_audio", NULL),
	REGULATOR_SUPPLY("vdd_1v8_audio", NULL),
	REGULATOR_SUPPLY("vddio_modem", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vdd_1v8_sensor", NULL),
	REGULATOR_SUPPLY("vdd_1v8_mic", NULL),
	REGULATOR_SUPPLY("vdd_1v8_nfc", NULL),
	REGULATOR_SUPPLY("vdd_1v8_ds", NULL),
	REGULATOR_SUPPLY("vdd_1v8_ts", NULL),
	REGULATOR_SUPPLY("vdd_1v8_spi", NULL),
	REGULATOR_SUPPLY("dvdd_lcd", NULL),
	REGULATOR_SUPPLY("vdd_1v8_com", NULL),
	REGULATOR_SUPPLY("vdd_1v8_gps", NULL),
	REGULATOR_SUPPLY("vdd_1v8_dtv", NULL),
};

static struct regulator_consumer_supply max77663_sd3_supply[] = {
	REGULATOR_SUPPLY("vdd_emmc_core", NULL),
};

static struct regulator_consumer_supply max77663_ldo0_supply[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu", NULL),
	REGULATOR_SUPPLY("avdd_csi_dsi_pll", NULL),
};

static struct regulator_consumer_supply max77663_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply max77663_ldo2_supply[] = {
	REGULATOR_SUPPLY("vdd_2v85_sensor", NULL),
	REGULATOR_SUPPLY("vdd_temp_sensor", NULL),
};

static struct regulator_consumer_supply max77663_ldo3_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb3_pll", NULL),
	REGULATOR_SUPPLY("avddio_usb3", NULL),
};

static struct regulator_consumer_supply max77663_ldo4_supply[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply max77663_ldo5_supply[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
	REGULATOR_SUPPLY("vddio_bb_hsic", NULL),
};

static struct regulator_consumer_supply max77663_ldo6_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};

static struct regulator_consumer_supply max77663_ldo7_supply[] = {
	REGULATOR_SUPPLY("avdd_cam1", NULL),
};

static struct regulator_consumer_supply max77663_ldo8_supply[] = {
	REGULATOR_SUPPLY("avdd_cam2", NULL),
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

MAX77663_PDATA_INIT(SD0, sd0,  600000, 3387500, tps65090_rails(DCDC3), 1, 1, 0,
		    FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(SD1, sd1,  800000, 1587500, tps65090_rails(DCDC3), 1, 1, 0,
		    FPS_SRC_1, -1, -1, SD_FSRADE_DISABLE);

MAX77663_PDATA_INIT(SD2, sd2,  1800000, 1800000, tps65090_rails(DCDC3), 1, 1, 0,
		    FPS_SRC_0, -1, -1, 0);

MAX77663_PDATA_INIT(SD3, sd3,  600000, 3387500, tps65090_rails(DCDC3), 1, 1, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO0, ldo0, 800000, 2350000, max77663_rails(sd2), 1, 1, 0,
		    FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO1, ldo1, 800000, 2350000, max77663_rails(sd2), 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO2, ldo2, 2850000, 2850000, tps65090_rails(DCDC2), 1, 1,
		    0, FPS_SRC_1, -1, -1, 0);

MAX77663_PDATA_INIT(LDO3, ldo3, 800000, 3950000, max77663_rails(sd2), 1, 1, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO4, ldo4, 1000000, 1000000, tps65090_rails(DCDC2), 0, 1,
		    0, FPS_SRC_NONE, -1, -1, LDO4_EN_TRACKING);

MAX77663_PDATA_INIT(LDO5, ldo5, 800000, 2800000, max77663_rails(sd2), 0, 1, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO6, ldo6, 800000, 3950000, tps65090_rails(DCDC2), 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO7, ldo7, 800000, 3950000, tps65090_rails(DCDC2), 0, 0, 0,
		    FPS_SRC_NONE, -1, -1, 0);

MAX77663_PDATA_INIT(LDO8, ldo8, 800000, 3950000, tps65090_rails(DCDC2), 0, 1, 0,
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

/* EN_AVDD_USB_HDMI From PMU GP1 */
static struct regulator_consumer_supply fixed_reg_en_avdd_usb_hdmi_supply[] = {
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("hvdd_usb3", NULL),
};

/* EN_CAM_1v8 From PMU GP5 */
static struct regulator_consumer_supply fixed_reg_en_1v8_cam_supply[] = {
	REGULATOR_SUPPLY("dvdd_cam", NULL),
};


static struct regulator_consumer_supply fixed_reg_en_hdmi_con_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_con", NULL),
};

/* EN_USB1_VBUS From TEGRA GPIO PN4 PR3(T30) */
static struct regulator_consumer_supply fixed_reg_en_usb1_vbus_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_micro_usb", NULL),
};

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
/* EN_3V3_FUSE From TEGRA GPIO PX4 */
static struct regulator_consumer_supply fixed_reg_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

/* EN_USB3_VBUS From TEGRA GPIO PM5 */
static struct regulator_consumer_supply fixed_reg_en_usb3_vbus_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_typea_usb", NULL),
};
#endif

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
	_gpio_nr, _active_high, _boot_state, _millivolts)	\
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

FIXED_REG(1,	en_avdd_usb_hdmi,	en_avdd_usb_hdmi,
	tps65090_rails(DCDC2),	0,	0,
	MAX77663_GPIO_BASE + MAX77663_GPIO1,	true,	1,	3300);
FIXED_REG(2,	en_1v8_cam,	en_1v8_cam,
	max77663_rails(sd2),	0,	0,
	MAX77663_GPIO_BASE + MAX77663_GPIO5,	true,	0,	1800);
FIXED_REG(3,	en_hdmi_con,	en_hdmi_con,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PK1,	true,	0,	5000);
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
FIXED_REG(4,	en_3v3_fuse,	en_3v3_fuse,
	max77663_rails(sd2),	0,	0,
	TEGRA_GPIO_PX4,	true,	0,	3300);
FIXED_REG(5,	en_usb1_vbus,	en_usb1_vbus,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PN4,	true,	0,	5000);
FIXED_REG(6,	en_usb3_vbus,	en_usb3_vbus,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PM5,	true,	0,	5000);
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
FIXED_REG(4,	en_usb1_vbus,	en_usb1_vbus,
	tps65090_rails(DCDC1),	0,	0,
	TEGRA_GPIO_PR3,	true,	0,	5000);
#endif
/*
 * Creating the fixed regulator device tables
 */

#define ADD_FIXED_REG(_name)    (&fixed_reg_##_name##_dev)

#define E1612_COMMON_FIXED_REG			\
	ADD_FIXED_REG(en_avdd_usb_hdmi),	\
	ADD_FIXED_REG(en_1v8_cam),		\
	ADD_FIXED_REG(en_hdmi_con),
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
#define E1612_T114_FIXED_REG			\
	ADD_FIXED_REG(en_3v3_fuse),		\
	ADD_FIXED_REG(en_usb1_vbus),		\
	ADD_FIXED_REG(en_usb3_vbus),
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
#define E1612_T30_FIXED_REG			\
	ADD_FIXED_REG(en_usb1_vbus),
#endif

/* Gpio switch regulator platform data for Dalmore E1612 */
static struct platform_device *fixed_reg_devs_a00[] = {
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	E1612_T114_FIXED_REG
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	E1612_T30_FIXED_REG
#endif
	E1612_COMMON_FIXED_REG

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
			.max_uV = 1520000,				\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = 1,					\
			.boot_on =  1,					\
			.apply_uV = 0,					\
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
	.cpu_timer	= 2000,
	.cpu_off_timer	= 0,
	.suspend_mode	= TEGRA_SUSPEND_NONE,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= false,
	.sysclkreq_high	= true,
};

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

static int __init dalmore_fixed_regulator_init(void)
{
	if (!machine_is_dalmore())
		return 0;

	return platform_add_devices(fixed_reg_devs_a00,
				ARRAY_SIZE(fixed_reg_devs_a00));
}
subsys_initcall_sync(dalmore_fixed_regulator_init);

int __init dalmore_regulator_init(void)
{
	i2c_register_board_info(4, tps65090_regulators,
				ARRAY_SIZE(tps65090_regulators));

	dalmore_max77663_regulator_init();
	i2c_register_board_info(4, tps51632_boardinfo, 1);
	platform_device_register(&dalmore_pda_power_device);
	return 0;
}

int __init dalmore_suspend_init(void)
{
	tegra_init_suspend(&dalmore_suspend_data);
	return 0;
}

