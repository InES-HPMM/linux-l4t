/*
 * arch/arm/mach-tegra/board-pluto-sensors.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include "gpio-names.h"
#include "board.h"
#include <mach/gpio.h>
#include <media/imx091.h>
#include "board-pluto.h"
#include "cpu-tegra.h"
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <mach/gpio-tegra.h>
#include <mach/fb.h>
#include <media/imx091.h>
#include <media/imx132.h>
#include <generated/mach-types.h>
#include <linux/mpu.h>

static struct board_info board_info;

/* isl29029 support is provided by isl29028*/
static struct i2c_board_info pluto_i2c1_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

struct pluto_cam_gpio {
	int gpio;
	const char *label;
	int value;
};

static char *pluto_cam_reg_name[] = {
	"avdd_cam1",		/* Analog VDD 2.7V */
	"avdd_cam2",		/* Analog VDD 2.7V */
	"vdd_1v2_cam",		/* Digital VDD 1.2V */
	"vdd_1v8_cam12",	/* Digital VDDIO 1.8V */
	"vddio_cam",		/* Tegra CAM_I2C, CAM_MCLK, VDD 1.8V */
	"vddio_cam_mb",		/* CAM_I2C PULL-UP VDD 1.8V */
	"vdd_af_cam1",		/* AF VDD */
};

static struct regulator *pluto_cam_reg[ARRAY_SIZE(pluto_cam_reg_name)];

static int pluto_imx091_power_on(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (!pluto_cam_reg[i]) {
			pluto_cam_reg[i] = regulator_get(NULL,
					pluto_cam_reg_name[i]);
			if (WARN_ON(IS_ERR(pluto_cam_reg[i]))) {
				pr_err("%s: didn't get regulator #%d: %ld\n",
				__func__, i, PTR_ERR(pluto_cam_reg[i]));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(pluto_cam_reg[i]);
	}

	gpio_direction_output(CAM_RSTN, 0);
	gpio_direction_output(CAM_GPIO1, 1);
	gpio_direction_output(CAM_RSTN, 1);

	mdelay(1);

	return 0;

reg_alloc_fail:

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (pluto_cam_reg[i]) {
			regulator_put(pluto_cam_reg[i]);
			pluto_cam_reg[i] = NULL;
		}
	}

	return -ENODEV;
}

static int pluto_imx091_power_off(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (pluto_cam_reg[i]) {
			regulator_disable(pluto_cam_reg[i]);
			regulator_put(pluto_cam_reg[i]);
		}
	}

	return 0;
}

static int pluto_imx132_power_on(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (!pluto_cam_reg[i]) {
			pluto_cam_reg[i] = regulator_get(NULL,
					pluto_cam_reg_name[i]);
			if (WARN_ON(IS_ERR(pluto_cam_reg[i]))) {
				pr_err("%s: didn't get regulator #%d: %ld\n",
				__func__, i, PTR_ERR(pluto_cam_reg[i]));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(pluto_cam_reg[i]);
	}

	gpio_direction_output(CAM_RSTN, 0);
	gpio_direction_output(CAM_GPIO2, 1);
	gpio_direction_output(CAM_RSTN, 1);

	mdelay(1);

	return 0;

reg_alloc_fail:

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (pluto_cam_reg[i]) {
			regulator_put(pluto_cam_reg[i]);
			pluto_cam_reg[i] = NULL;
		}
	}

	return -ENODEV;
}


static int pluto_imx132_power_off(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (pluto_cam_reg[i]) {
			regulator_disable(pluto_cam_reg[i]);
			regulator_put(pluto_cam_reg[i]);
		}
	}

	return 0;
}

struct imx091_platform_data pluto_imx091_data = {
	.power_on = pluto_imx091_power_on,
	.power_off = pluto_imx091_power_off,
};

struct imx132_platform_data pluto_imx132_data = {
	.power_on = pluto_imx132_power_on,
	.power_off = pluto_imx132_power_off,
};

static struct i2c_board_info pluto_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("imx091", 0x10),
		.platform_data = &pluto_imx091_data,
	},
	{
		I2C_BOARD_INFO("imx132", 0x36),
		.platform_data = &pluto_imx132_data,
	},
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}
static struct pluto_cam_gpio pluto_cam_gpio_data[] = {
	[0] = TEGRA_CAMERA_GPIO(CAM1_POWER_DWN_GPIO, "camera_power_en", 1),
	[1] = TEGRA_CAMERA_GPIO(CAM2_POWER_DWN_GPIO, "camera2_power_en", 1),
	[2] = TEGRA_CAMERA_GPIO(CAM_GPIO1, "camera_gpio1", 0),
	[3] = TEGRA_CAMERA_GPIO(CAM_GPIO2, "camera_gpio2", 0),
	[4] = TEGRA_CAMERA_GPIO(CAM_RSTN, "camera_rstn", 1),
	[5] = TEGRA_CAMERA_GPIO(CAM_AF_PWDN, "camera_af_pwdn", 1),
};

static int pluto_camera_init(void)
{
	int ret;
	int i;

	pr_debug("%s: ++\n", __func__);

	for (i = 0; i < ARRAY_SIZE(pluto_cam_gpio_data); i++) {
		ret = gpio_request(pluto_cam_gpio_data[i].gpio,
				pluto_cam_gpio_data[i].label);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail_free_gpio;
		}
		gpio_direction_output(pluto_cam_gpio_data[i].gpio,
					pluto_cam_gpio_data[i].value);
		gpio_export(pluto_cam_gpio_data[i].gpio, false);
	}

	i2c_register_board_info(2, pluto_i2c_board_info_e1625,
		ARRAY_SIZE(pluto_i2c_board_info_e1625));

	return 0;

fail_free_gpio:
	while (i--)
		gpio_free(pluto_cam_gpio_data[i].gpio);
	return ret;
}

int __init pluto_sensors_init(void)
{
	pr_debug("%s: ++\n", __func__);
	tegra_get_board_info(&board_info);
	pluto_camera_init();

	i2c_register_board_info(0, pluto_i2c1_isl_board_info,
				ARRAY_SIZE(pluto_i2c1_isl_board_info));

	return 0;
}
