/*
 * arch/arm/mach-tegra/board-dalmore-sensors.c
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
#include <mach/gpio.h>
#include "gpio-names.h"
#include "board.h"
#include <mach/gpio.h>
#include <media/imx091.h>
#include <media/ov9772.h>
#include "board-dalmore.h"
#include "cpu-tegra.h"
#include <generated/mach-types.h>
#include <linux/mpu.h>

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>

#include <mach/fb.h>
#include <mach/edp.h>
#include <mach/gpio.h>
#include <mach/gpio-tegra.h>

#include "gpio-names.h"
#include "board.h"
#include "board-dalmore.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"

static struct board_info board_info;

static char *dalmore_cam_reg_name[] = {
	"vdd_sensor_2v85",	/* 2.85V */
	"avddio_usb",		/* VDDIO USB CAM */
	"dvdd_cam",		/* DVDD CAM */
	"vddio_cam",		/* Tegra CAM_I2C, CAM_MCLK, VDD 1.8V */
	"avdd_cam1",		/* Analog VDD 2.7V */
	"avdd_cam2",		/* Analog VDD 2.7V */
};

static struct regulator *dalmore_cam_reg[ARRAY_SIZE(dalmore_cam_reg_name)];

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

static struct nct1008_platform_data dalmore_nct1008_pdata = {
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

static struct i2c_board_info dalmore_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &dalmore_nct1008_pdata,
		.irq = -1,
	}
};

static int dalmore_imx091_power_on(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(dalmore_cam_reg_name); i++) {
		if (!dalmore_cam_reg[i]) {
			dalmore_cam_reg[i] = regulator_get(dev,
					dalmore_cam_reg_name[i]);
			if (WARN_ON(IS_ERR(dalmore_cam_reg[i]))) {
				pr_err("%s: didn't get regulator #%d: %ld\n",
				__func__, i, PTR_ERR(dalmore_cam_reg[i]));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(dalmore_cam_reg[i]);
	}

	gpio_direction_output(CAM_RSTN, 0);
	mdelay(10);
	gpio_direction_output(CAM_AF_PWDN, 1);
	gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM_RSTN, 1);

	return 0;

reg_alloc_fail:

	for (i = ARRAY_SIZE(dalmore_cam_reg_name) - 1; i >= 0; i--) {
		if (dalmore_cam_reg[i]) {
			regulator_put(dalmore_cam_reg[i]);
			dalmore_cam_reg[i] = NULL;
		}
	}

	return -ENODEV;
}

static int dalmore_imx091_power_off(struct device *dev)
{
	int i;
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);

	for (i = ARRAY_SIZE(dalmore_cam_reg_name) - 1; i >= 0; i--) {
		if (dalmore_cam_reg[i]) {
			regulator_disable(dalmore_cam_reg[i]);
			regulator_put(dalmore_cam_reg[i]);
			dalmore_cam_reg[i] = NULL;
		}
	}

	return 0;
}

static int dalmore_ov9772_power_on(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(dalmore_cam_reg_name); i++) {
		if (!dalmore_cam_reg[i]) {
			dalmore_cam_reg[i] = regulator_get(dev,
					dalmore_cam_reg_name[i]);
			if (WARN_ON(IS_ERR(dalmore_cam_reg[i]))) {
				pr_err("%s: didn't get regulator #%d: %ld\n",
				__func__, i, PTR_ERR(dalmore_cam_reg[i]));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(dalmore_cam_reg[i]);
	}

	gpio_direction_output(CAM_RSTN, 0);
	mdelay(10);
	gpio_direction_output(CAM_AF_PWDN, 1);
	gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM_RSTN, 1);

	return 0;

reg_alloc_fail:

	for (i = ARRAY_SIZE(dalmore_cam_reg_name) - 1; i >= 0; i--) {
		if (dalmore_cam_reg[i]) {
			regulator_put(dalmore_cam_reg[i]);
			dalmore_cam_reg[i] = NULL;
		}
	}

	return -ENODEV;
}

static int dalmore_ov9772_power_off(struct device *dev)
{
	int i;
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);

	for (i = ARRAY_SIZE(dalmore_cam_reg_name) - 1; i >= 0; i--) {
		if (dalmore_cam_reg[i]) {
			regulator_disable(dalmore_cam_reg[i]);
			regulator_put(dalmore_cam_reg[i]);
			dalmore_cam_reg[i] = NULL;
		}
	}

	return 0;
}

static struct nvc_gpio_pdata ov9772_gpio_pdata[] = {
	{ OV9772_GPIO_TYPE_SHTDN, TEGRA_GPIO_PBB5, true, 0, },
	{ OV9772_GPIO_TYPE_PWRDN, TEGRA_GPIO_PBB3, true, 0, },
};

static struct ov9772_platform_data ov9772_pdata = {
	.num		= 1,
	.dev_name	= "camera",
	.gpio_count	= ARRAY_SIZE(ov9772_gpio_pdata),
	.gpio		= ov9772_gpio_pdata,
	.power_on	= dalmore_ov9772_power_on,
	.power_off	= dalmore_ov9772_power_off,
};

struct imx091_platform_data dalmore_imx091_data = {
	.power_on = dalmore_imx091_power_on,
	.power_off = dalmore_imx091_power_off,
};

static struct i2c_board_info dalmore_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("imx091", 0x36),
		.platform_data = &dalmore_imx091_data,
	},
	{
		I2C_BOARD_INFO("ov9772", 0x10),
		.platform_data = &ov9772_pdata,
	},
};

struct dalmore_cam_gpio {
	int gpio;
	const char *label;
	int value;
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}

static struct dalmore_cam_gpio dalmore_cam_gpio_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM1_POWER_DWN_GPIO, "camera_power_en", 0),
	[1] = TEGRA_CAMERA_GPIO(CAM2_POWER_DWN_GPIO, "camera2_power_en", 0),
	[2] = TEGRA_CAMERA_GPIO(CAM_GPIO1, "camera_gpio1", 0),
	[3] = TEGRA_CAMERA_GPIO(CAM_GPIO2, "camera_gpio2", 0),
	[4] = TEGRA_CAMERA_GPIO(CAM_RSTN, "camera_rstn", 1),
	[5] = TEGRA_CAMERA_GPIO(CAM_AF_PWDN, "camera_af_pwdn", 1),
};

static int dalmore_camera_init(void)
{

	int ret;
	int i;

	pr_debug("%s: ++\n", __func__);

	for (i = 0; i < ARRAY_SIZE(dalmore_cam_gpio_data); i++) {
		ret = gpio_request(dalmore_cam_gpio_data[i].gpio,
				dalmore_cam_gpio_data[i].label);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail_free_gpio;
		}
		gpio_direction_output(dalmore_cam_gpio_data[i].gpio,
					dalmore_cam_gpio_data[i].value);
		gpio_export(dalmore_cam_gpio_data[i].gpio, false);
	}

	i2c_register_board_info(2, dalmore_i2c_board_info_e1625,
		ARRAY_SIZE(dalmore_i2c_board_info_e1625));

fail_free_gpio:

	while (i--)
		gpio_free(dalmore_cam_gpio_data[i].gpio);
	return ret;

}


/* MPU board file definition	*/
static struct mpu_platform_data mpu9150_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
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

static struct i2c_board_info dalmore_i2c_board_info_cm3218[] = {
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

static int dalmore_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
	if ((board_info.board_id == BOARD_E1611) ||
	    (board_info.board_id == BOARD_E1612) ||
	    (board_info.board_id == BOARD_E1641) ||
	    (board_info.board_id == BOARD_E1613))
	{
		/* per email from Matt 9/10/2012 */
		nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert_port assumed TEGRA_GPIO_PX6"
		       " for unknown dalmore board id E%d\n",
		       board_info.board_id);
	}
#else
	/* dalmore + AP30 interposer has SPI2_CS0 gpio */
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
			dalmore_nct1008_pdata.active[i].create_cdev =
				(struct thermal_cooling_device *(*)(void *))
					edp_cooling_device_create;
			dalmore_nct1008_pdata.active[i].cdev_data = (void *)i;
			dalmore_nct1008_pdata.active[i].trip_temp =
				cpu_edp_limits[i].temperature * 1000;
			dalmore_nct1008_pdata.active[i].hysteresis = 1000;
		}
		dalmore_nct1008_pdata.active[i].create_cdev = NULL;
#endif

		dalmore_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
		pr_info("%s: dalmore nct1008 irq %d", __func__, dalmore_i2c4_nct1008_board_info[0].irq);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0) {
			pr_info("%s: calling gpio_free(nct1008_port)", __func__);
			gpio_free(nct1008_port);
		}
	}

	/* dalmore has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, dalmore_i2c4_nct1008_board_info,
		ARRAY_SIZE(dalmore_i2c4_nct1008_board_info));

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

static int __init dalmore_skin_init(void)
{
	struct thermal_cooling_device *skin_cdev;

	skin_cdev = balanced_throttle_register(&skin_throttle);

	skin_data.cdev = skin_cdev;
	tegra_skin_therm_est_device.dev.platform_data = &skin_data;
	platform_device_register(&tegra_skin_therm_est_device);

	return 0;
}
late_initcall(dalmore_skin_init);
#endif

int __init dalmore_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	err = dalmore_nct1008_init();
	if (err)
		return err;

	dalmore_camera_init();
	mpuirq_init();

	i2c_register_board_info(0, dalmore_i2c_board_info_cm3218,
		ARRAY_SIZE(dalmore_i2c_board_info_cm3218));

	i2c_register_board_info(0, bq20z45_pdata,
		ARRAY_SIZE(bq20z45_pdata));

	return 0;
}
