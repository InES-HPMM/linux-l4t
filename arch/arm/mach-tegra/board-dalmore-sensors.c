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

static struct regulator *dalmore_avdd_cam1;
static struct regulator *dalmore_dvdd_cam;
static struct regulator *dalmore_vddio_cam;
static struct regulator *dalmore_vdd_sensor;

static struct regulator *dalmore_avdd_cam2;
static struct regulator *dalmore_vdd_ldo3;
static struct regulator *dalmore_vdd_csidsi;
/*static struct regulator *dalmore_vdd_2v8_cam1;*/

static struct board_info board_info;

static int dalmore_camera_init(void)
{
	int ret;

	ret = gpio_request(CAM1_POWER_DWN_GPIO, "camera_power_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM1_POWER_DWN_GPIO");
		goto Exit;
	}

	ret = gpio_request(CAM2_POWER_DWN_GPIO, "camera2_power_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM2_POWER_DWN_GPIO");
		goto Exit;
	}

	ret = gpio_request(CAM_GPIO1, "camera_gpio1");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_GPIO1");
		goto Exit;
	}

	ret = gpio_request(CAM_GPIO2, "camera_gpio2");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_GPIO2");
		goto Exit;
	}

	ret = gpio_request(CAM_RSTN, "camera_rstn");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_RSTN");
		goto Exit;
	}

	ret = gpio_request(CAM_AF_PWDN, "camera_af_pwdn");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_AF_PWDN");
		goto Exit;
	}

Exit:
	return ret;

}

static int dalmore_imx091_power_on(void)
{
	if (dalmore_vdd_sensor == NULL)
		dalmore_vdd_sensor = regulator_get(NULL, "vdd_sensor_2v85");
	if (WARN_ON(IS_ERR(dalmore_vdd_sensor))) {
		pr_err("%s: couldn't get regulator dvdd_cam: %ld\n",
			__func__, PTR_ERR(dalmore_vdd_sensor));
		goto reg_alloc_fail;
	}
	regulator_enable(dalmore_vdd_sensor);

	if (dalmore_vdd_ldo3 == NULL)
		dalmore_vdd_ldo3 = regulator_get(NULL, "avddio_usb");
	if (WARN_ON(IS_ERR(dalmore_vdd_ldo3))) {
		pr_err("%s: couldn't get regulator dvdd_cam: %ld\n",
			__func__, PTR_ERR(dalmore_vdd_ldo3));
		goto reg_alloc_fail;
	}
	regulator_enable(dalmore_vdd_ldo3);

	if (dalmore_vdd_csidsi == NULL)
		dalmore_vdd_csidsi = regulator_get(NULL, "avdd_dsi_csi");
	if (WARN_ON(IS_ERR(dalmore_vdd_csidsi))) {
		pr_err("%s: couldn't get regulator dvdd_cam: %ld\n",
			__func__, PTR_ERR(dalmore_vdd_csidsi));
		goto reg_alloc_fail;
	}
	regulator_enable(dalmore_vdd_csidsi);

	if (dalmore_dvdd_cam == NULL) {
		dalmore_dvdd_cam = regulator_get(NULL, "dvdd_cam");
		if (WARN_ON(IS_ERR(dalmore_dvdd_cam))) {
			pr_err("%s: couldn't get regulator dvdd_cam: %ld\n",
				__func__, PTR_ERR(dalmore_dvdd_cam));
			goto reg_alloc_fail;
		}
	}

	regulator_enable(dalmore_dvdd_cam);

	if (dalmore_vddio_cam == NULL) {
		dalmore_vddio_cam = regulator_get(NULL, "vddio_cam");
		if (WARN_ON(IS_ERR(dalmore_vddio_cam))) {
			pr_err("%s: couldn't get regulator vddio_cam: %ld\n",
				__func__, PTR_ERR(dalmore_vddio_cam));
			goto reg_alloc_fail;
		}
	}

	regulator_enable(dalmore_vddio_cam);

	/* Enable avdd_cam1 */
	if (dalmore_avdd_cam1 == NULL) {
		dalmore_avdd_cam1 = regulator_get(NULL, "avdd_cam1");
		if (WARN_ON(IS_ERR(dalmore_avdd_cam1))) {
			pr_err("%s: couldn't get regulator avdd_cam1: %ld\n",
				__func__, PTR_ERR(dalmore_avdd_cam1));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(dalmore_avdd_cam1);

	/* Enable avdd_cam2 */
	if (dalmore_avdd_cam2 == NULL) {
		dalmore_avdd_cam2 = regulator_get(NULL, "avdd_cam2");
		if (WARN_ON(IS_ERR(dalmore_avdd_cam2))) {
			pr_err("%s: couldn't get regulator avdd_cam2: %ld\n",
				__func__, PTR_ERR(dalmore_avdd_cam2));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(dalmore_avdd_cam2);

	/* Enable dvdd_cam */
	mdelay(5);
	gpio_direction_output(CAM_RSTN, 0);
	mdelay(10);
	gpio_direction_output(CAM_AF_PWDN, 1);
	gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
	gpio_direction_output(CAM_RSTN, 1);

	return 0;

reg_alloc_fail:

	if (dalmore_vdd_sensor) {
		regulator_put(dalmore_vdd_sensor);
		dalmore_vdd_sensor = NULL;
	}

	if (dalmore_vdd_ldo3) {
		regulator_put(dalmore_vdd_ldo3);
		dalmore_vdd_ldo3 = NULL;
	}

	if (dalmore_vdd_csidsi) {
		regulator_put(dalmore_vdd_csidsi);
		dalmore_vdd_csidsi = NULL;
	}

	if (dalmore_dvdd_cam) {
		regulator_put(dalmore_dvdd_cam);
		dalmore_dvdd_cam = NULL;
	}

	if (dalmore_vddio_cam) {
		regulator_put(dalmore_vddio_cam);
		dalmore_vddio_cam = NULL;
	}

	if (dalmore_avdd_cam1) {
		regulator_put(dalmore_avdd_cam1);
		dalmore_avdd_cam1 = NULL;
	}

	if (dalmore_avdd_cam2) {
		regulator_put(dalmore_avdd_cam2);
		dalmore_avdd_cam2 = NULL;
	}


	return -ENODEV;
}

static int dalmore_imx091_power_off(void)
{
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);

	if (dalmore_vdd_sensor)
		regulator_disable(dalmore_vdd_sensor);

	if (dalmore_vdd_ldo3)
		regulator_disable(dalmore_vdd_ldo3);

	if (dalmore_vdd_csidsi)
		regulator_disable(dalmore_vdd_csidsi);

	if (dalmore_dvdd_cam)
		regulator_disable(dalmore_dvdd_cam);

	if (dalmore_vddio_cam)
		regulator_disable(dalmore_vddio_cam);

	if (dalmore_avdd_cam1)
		regulator_disable(dalmore_avdd_cam1);

	if (dalmore_avdd_cam2)
		regulator_disable(dalmore_avdd_cam2);

	return 0;
}

struct imx091_platform_data dalmore_imx091_data = {
	.power_on = dalmore_imx091_power_on,
	.power_off = dalmore_imx091_power_off,
};

static struct i2c_board_info dalmore_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("imx091", 0x36),
		.platform_data = &dalmore_imx091_data,
	},
};


int __init dalmore_sensors_init(void)
{
	tegra_get_board_info(&board_info);

	dalmore_camera_init();

	i2c_register_board_info(2, dalmore_i2c_board_info_e1625,
		ARRAY_SIZE(dalmore_i2c_board_info_e1625));

	return 0;
}
