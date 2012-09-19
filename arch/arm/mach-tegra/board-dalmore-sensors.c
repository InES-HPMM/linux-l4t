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

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/gpio.h>

#include <mach/gpio-tegra.h>
#include <mach/fb.h>

#include <media/imx091.h>
#include <generated/mach-types.h>

#include <linux/mpu.h>
#include "board-dalmore.h"

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

static int dalmore_imx091_power_on(void)
{

	int i;

	for (i = 0; i < ARRAY_SIZE(dalmore_cam_reg_name); i++) {
		if (!dalmore_cam_reg[i]) {
			dalmore_cam_reg[i] = regulator_get(NULL,
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

static int dalmore_imx091_power_off(void)
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

static int dalmore_ov9772_power_on(void)
{

	int i;

	for (i = 0; i < ARRAY_SIZE(dalmore_cam_reg_name); i++) {
		if (!dalmore_cam_reg[i]) {
			dalmore_cam_reg[i] = regulator_get(NULL,
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

static int dalmore_ov9772_power_off(void)
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

struct imx091_platform_data dalmore_imx091_data = {
	.power_on = dalmore_imx091_power_on,
	.power_off = dalmore_imx091_power_off,
};

struct ov9772_platform_data dalmore_ov9772_data = {
	.power_on = dalmore_ov9772_power_on,
	.power_off = dalmore_ov9772_power_off,
};

static struct i2c_board_info dalmore_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("imx091", 0x36),
		.platform_data = &dalmore_imx091_data,
	},
	{
		I2C_BOARD_INFO("ov9772", 0x10),
		.platform_data = &dalmore_ov9772_data,
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


int __init dalmore_sensors_init(void)
{
	tegra_get_board_info(&board_info);

	dalmore_camera_init();

	return 0;
}
