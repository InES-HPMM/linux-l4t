/*
 * arch/arm/mach-tegra/board-roth-sensors.c
 *
 * Copyright (c) 2012 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/nct1008.h>
#include <mach/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <media/imx091.h>
#include <media/ov9772.h>
#include <media/as364x.h>
#include <media/ad5816.h>
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-roth.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"

static struct board_info board_info;

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = 10,
	.throt_tab = {
		{      0, 1000 },
		{ 640000, 1000 },
		{ 640000, 1000 },
		{ 640000, 1000 },
		{ 640000, 1000 },
		{ 640000, 1000 },
		{ 760000, 1000 },
		{ 760000, 1050 },
		{1000000, 1050 },
		{1000000, 1100 },
	},
};

static struct nct1008_platform_data roth_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 80, /* 4 * 20C. Bug 844025 - 1C for device accuracies */
	.shutdown_ext_limit = 90, /* C */
	.shutdown_local_limit = 120, /* C */

	/* Thermal Throttling */
	.passive = {
		.create_cdev = (struct thermal_cooling_device *(*)(void *))
				balanced_throttle_register,
		.cdev_data = &tj_throttle,
		.trip_temp = 80000,
		.tc1 = 0,
		.tc2 = 1,
		.passive_delay = 2000,
	}
};

static struct i2c_board_info roth_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &roth_nct1008_pdata,
		.irq = -1,
	}
};

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

static struct tegra_pingroup_config mclk_disable =
	VI_PINMUX(CAM_MCLK, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config mclk_enable =
	VI_PINMUX(CAM_MCLK, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_disable =
	VI_PINMUX(GPIO_PBB0, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_enable =
	VI_PINMUX(GPIO_PBB0, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

/*
 * As a workaround, roth_vcmvdd need to be allocated to activate the
 * sensor devices. This is due to the focuser device(AD5816) will hook up
 * the i2c bus if it is not powered up.
*/
static struct regulator *roth_vcmvdd;

static int roth_get_vcmvdd(void)
{
	if (!roth_vcmvdd) {
		roth_vcmvdd = regulator_get(NULL, "vdd_af_cam1");
		if (unlikely(WARN_ON(IS_ERR(roth_vcmvdd)))) {
			pr_err("%s: can't get regulator vcmvdd: %ld\n",
				__func__, PTR_ERR(roth_vcmvdd));
			roth_vcmvdd = NULL;
			return -ENODEV;
		}
	}
	return 0;
}

static int roth_imx091_power_on(struct imx091_power_rail *pw)
{
	int err;

	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	if (roth_get_vcmvdd())
		goto imx091_poweron_fail;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx091_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx091_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	err = regulator_enable(roth_vcmvdd);
	if (unlikely(err))
		goto imx091_vcmvdd_fail;

	tegra_pinmux_config_table(&mclk_enable, 1);
	usleep_range(300, 310);

	return 1;

imx091_vcmvdd_fail:
	regulator_disable(pw->iovdd);

imx091_iovdd_fail:
	regulator_disable(pw->avdd);

imx091_avdd_fail:
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

imx091_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int roth_imx091_power_off(struct imx091_power_rail *pw)
{
	if (unlikely(!pw || !roth_vcmvdd || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	usleep_range(1, 2);
	tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(1, 2);

	regulator_disable(roth_vcmvdd);
	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	return 1;
}

struct imx091_platform_data roth_imx091_data = {
	.power_on = roth_imx091_power_on,
	.power_off = roth_imx091_power_off,
};

static int roth_ov9772_power_on(struct ov9772_power_rail *pw)
{
	int err;

	if (unlikely(!pw || !pw->avdd || !pw->dovdd))
		return -EFAULT;

	if (roth_get_vcmvdd())
		goto ov9772_get_vcmvdd_fail;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
	gpio_set_value(CAM_RSTN, 0);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ov9772_avdd_fail;

	err = regulator_enable(pw->dovdd);
	if (unlikely(err))
		goto ov9772_dovdd_fail;

	gpio_set_value(CAM_RSTN, 1);
	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);

	err = regulator_enable(roth_vcmvdd);
	if (unlikely(err))
		goto ov9772_vcmvdd_fail;

	tegra_pinmux_config_table(&pbb0_enable, 1);
	usleep_range(340, 380);

	/* return 1 to skip the in-driver power_on sequence */
	return 1;

ov9772_vcmvdd_fail:
	regulator_disable(pw->dovdd);

ov9772_dovdd_fail:
	regulator_disable(pw->avdd);

ov9772_avdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

ov9772_get_vcmvdd_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int roth_ov9772_power_off(struct ov9772_power_rail *pw)
{
	if (unlikely(!pw || !roth_vcmvdd || !pw->avdd || !pw->dovdd))
		return -EFAULT;

	usleep_range(21, 25);
	tegra_pinmux_config_table(&pbb0_disable, 1);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
	gpio_set_value(CAM_RSTN, 0);

	regulator_disable(roth_vcmvdd);
	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	/* return 1 to skip the in-driver power_off sequence */
	return 1;
}

static struct nvc_gpio_pdata ov9772_gpio_pdata[] = {
	{ OV9772_GPIO_TYPE_SHTDN, CAM2_POWER_DWN_GPIO, true, 0, },
	{ OV9772_GPIO_TYPE_PWRDN, CAM_RSTN, true, 0, },
};

static struct ov9772_platform_data roth_ov9772_pdata = {
	.num		= 1,
	.dev_name	= "camera",
	.gpio_count	= ARRAY_SIZE(ov9772_gpio_pdata),
	.gpio		= ov9772_gpio_pdata,
	.power_on	= roth_ov9772_power_on,
	.power_off	= roth_ov9772_power_off,
};

static int roth_as3648_power_on(struct as364x_power_rail *pw)
{
	int err = roth_get_vcmvdd();

	if (err)
		return err;

	return regulator_enable(roth_vcmvdd);
}

static int roth_as3648_power_off(struct as364x_power_rail *pw)
{
	if (!roth_vcmvdd)
		return -ENODEV;

	return regulator_disable(roth_vcmvdd);
}

static struct as364x_platform_data roth_as3648_pdata = {
	.config		= {
		.max_total_current_mA = 1000,
		.max_peak_current_mA = 600,
		.strobe_type = 1,
		},
	.pinstate	= {
		.mask	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0),
		.values	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0)
		},
	.dev_name	= "torch",
	.type		= AS3648,
	.gpio_strobe	= CAM_FLASH_STROBE,
	.led_mask	= 3,

	.power_on_callback = roth_as3648_power_on,
	.power_off_callback = roth_as3648_power_off,
};

static struct ad5816_platform_data pluto_ad5816_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "focuser",
};

static struct i2c_board_info roth_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("imx091", 0x36),
		.platform_data = &roth_imx091_data,
	},
	{
		I2C_BOARD_INFO("ov9772", 0x10),
		.platform_data = &roth_ov9772_pdata,
	},
	{
		I2C_BOARD_INFO("as3648", 0x30),
		.platform_data = &roth_as3648_pdata,
	},
	{
		I2C_BOARD_INFO("ad5816", 0x0E),
		.platform_data = &pluto_ad5816_pdata,
	},
};

static int roth_camera_init(void)
{
	tegra_pinmux_config_table(&mclk_disable, 1);
	tegra_pinmux_config_table(&pbb0_disable, 1);

	i2c_register_board_info(2, roth_i2c_board_info_e1625,
		ARRAY_SIZE(roth_i2c_board_info_e1625));
	return 0;
}

/* MPU board file definition	*/
static struct mpu_platform_data mpu9150_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id	= COMPASS_ID_AK8975,
	.secondary_i2c_addr	= MPU_COMPASS_ADDR,
	.secondary_read_reg	= 0x06,
	.secondary_orientation	= MPU_COMPASS_ORIENTATION,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}

static struct i2c_board_info roth_i2c_board_info_cm3218[] = {
	{
		I2C_BOARD_INFO("cm3218", 0x48),
	},
};

static struct i2c_board_info __initdata inv_mpu9150_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu9150_gyro_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu9150_i2c2_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu9150_i2c2_board_info,
		ARRAY_SIZE(inv_mpu9150_i2c2_board_info));
}

static int roth_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
	if ((board_info.board_id == BOARD_E1611) ||
	    (board_info.board_id == BOARD_E1612) ||
	    (board_info.board_id == BOARD_E1641) ||
	    (board_info.board_id == BOARD_E1613) ||
	    (board_info.board_id == BOARD_P2454))
	{
		/* per email from Matt 9/10/2012 */
		nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert_port assumed TEGRA_GPIO_PX6"
		       " for unknown roth board id E%d\n",
		       board_info.board_id);
	}
#else
	/* roth + AP30 interposer has SPI2_CS0 gpio */
	nct1008_port = TEGRA_GPIO_PX3;
#endif

	if (nct1008_port >= 0) {
#ifdef CONFIG_TEGRA_EDP_LIMITS
		const struct tegra_edp_limits *cpu_edp_limits;
		int cpu_edp_limits_size;
		int i;

		/* edp capping */
		tegra_get_cpu_edp_limits(&cpu_edp_limits, &cpu_edp_limits_size);

		if (cpu_edp_limits_size > MAX_THROT_TABLE_SIZE)
			BUG();

		for (i = 0; i < cpu_edp_limits_size-1; i++) {
			roth_nct1008_pdata.active[i].create_cdev =
				(struct thermal_cooling_device *(*)(void *))
					edp_cooling_device_create;
			roth_nct1008_pdata.active[i].cdev_data = (void *)i;
			roth_nct1008_pdata.active[i].trip_temp =
				cpu_edp_limits[i].temperature * 1000;
			roth_nct1008_pdata.active[i].hysteresis = 1000;
		}
		roth_nct1008_pdata.active[i].create_cdev = NULL;
#endif

		roth_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
		pr_info("%s: roth nct1008 irq %d", __func__, roth_i2c4_nct1008_board_info[0].irq);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0) {
			pr_info("%s: calling gpio_free(nct1008_port)", __func__);
			gpio_free(nct1008_port);
		}
	}

	/* roth has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, roth_i2c4_nct1008_board_info,
		ARRAY_SIZE(roth_i2c4_nct1008_board_info));

	return ret;
}

static struct i2c_board_info __initdata bq20z45_pdata[] = {
	{
		I2C_BOARD_INFO("sbs-battery", 0x0B),
	},
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int tegra_skin_match(struct thermal_zone_device *thz, void *data)
{
	return strcmp((char *)data, thz->type) == 0;
}

static int tegra_skin_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, tegra_skin_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static struct therm_est_data skin_data = {
	.toffset = 9793,
	.polling_period = 1100,
	.ndevs = 2,
	.devs = {
			{
				.dev_data = "nct_ext",
				.get_temp = tegra_skin_get_temp,
				.coeffs = {
					2, 1, 1, 1,
					1, 1, 1, 1,
					1, 1, 1, 0,
					1, 1, 0, 0,
					0, 0, -1, -7
				},
			},
			{
				.dev_data = "nct_int",
				.get_temp = tegra_skin_get_temp,
				.coeffs = {
					-11, -7, -5, -3,
					-3, -2, -1, 0,
					0, 0, 1, 1,
					1, 2, 2, 3,
					4, 6, 11, 18
				},
			},
	},
	.trip_temp = 43000,
	.tc1 = 1,
	.tc2 = 15,
	.passive_delay = 15000,
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = 6,
	.throt_tab = {
		{ 640000, 1200 },
		{ 640000, 1200 },
		{ 760000, 1200 },
		{ 760000, 1200 },
		{1000000, 1200 },
		{1000000, 1200 },
	},
};

static int __init roth_skin_init(void)
{
	struct thermal_cooling_device *skin_cdev;

	skin_cdev = balanced_throttle_register(&skin_throttle);

	skin_data.cdev = skin_cdev;
	tegra_skin_therm_est_device.dev.platform_data = &skin_data;
	platform_device_register(&tegra_skin_therm_est_device);

	return 0;
}
late_initcall(roth_skin_init);
#endif

int __init roth_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	err = roth_nct1008_init();
	if (err)
		return err;

	roth_camera_init();
	mpuirq_init();

	i2c_register_board_info(0, roth_i2c_board_info_cm3218,
		ARRAY_SIZE(roth_i2c_board_info_cm3218));

	i2c_register_board_info(0, bq20z45_pdata,
		ARRAY_SIZE(bq20z45_pdata));

	return 0;
}
