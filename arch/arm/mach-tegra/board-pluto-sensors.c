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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <mach/edp.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <media/imx091.h>
#include <media/imx132.h>
#include <media/ad5816.h>

#include <linux/nct1008.h>
#include "gpio-names.h"
#include "board.h"
#include "board-pluto.h"
#include <mach/gpio.h>
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"

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
	"vddio_cam_mb",		/* CAM_I2C PULL-UP VDD 1.8V */
	"vdd_af_cam1",		/* AF VDD */
};

static struct regulator *pluto_cam_reg[ARRAY_SIZE(pluto_cam_reg_name)];

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

static struct nct1008_platform_data pluto_nct1008_pdata = {
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

static struct i2c_board_info pluto_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &pluto_nct1008_pdata,
		.irq = -1,
	}
};

static int pluto_imx091_power_on(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (!pluto_cam_reg[i]) {
			pluto_cam_reg[i] = regulator_get(dev,
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

	for (i = ARRAY_SIZE(pluto_cam_reg_name) - 1; i >= 0; i--) {
		if (pluto_cam_reg[i]) {
			regulator_put(pluto_cam_reg[i]);
			pluto_cam_reg[i] = NULL;
		}
	}
	pr_info("%s fail\n", __func__);

	return -ENODEV;
}

static int pluto_imx091_power_off(struct device *dev)
{
	int i;

	for (i = ARRAY_SIZE(pluto_cam_reg_name) - 1; i >= 0; i--) {
		if (pluto_cam_reg[i]) {
			regulator_disable(pluto_cam_reg[i]);
			regulator_put(pluto_cam_reg[i]);
			pluto_cam_reg[i] = NULL;
		}
	}

	return 0;
}

static int pluto_imx132_power_on(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pluto_cam_reg_name); i++) {
		if (!pluto_cam_reg[i]) {
			pluto_cam_reg[i] = regulator_get(dev,
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
	for (i = ARRAY_SIZE(pluto_cam_reg_name) - 1; i >= 0; i--) {
		if (pluto_cam_reg[i]) {
			regulator_put(pluto_cam_reg[i]);
			pluto_cam_reg[i] = NULL;
		}
	}

	return -ENODEV;
}


static int pluto_imx132_power_off(struct device *dev)
{
	int i;

	for (i = ARRAY_SIZE(pluto_cam_reg_name) - 1; i >= 0; i--) {
		if (pluto_cam_reg[i]) {
			regulator_disable(pluto_cam_reg[i]);
			regulator_put(pluto_cam_reg[i]);
			pluto_cam_reg[i] = NULL;
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

static struct ad5816_platform_data pluto_ad5816_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "focuser",
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
	{
		I2C_BOARD_INFO("ad5816", 0x0E),
		.platform_data = &pluto_ad5816_pdata,
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

/* MPU board file definition */
static struct mpu_platform_data mpu_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation    = MPU_COMPASS_ORIENTATION,
	.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
};

static struct i2c_board_info __initdata inv_mpu_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu_gyro_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	int i = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

	/* MPU-IRQ assignment */
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu_i2c0_board_info[i++].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
#if MPU_COMPASS_IRQ_GPIO
	inv_mpu_i2c0_board_info[i++].irq = gpio_to_irq(MPU_COMPASS_IRQ_GPIO);
#endif
	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c0_board_info,
		ARRAY_SIZE(inv_mpu_i2c0_board_info));
}

static int pluto_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
	if (board_info.board_id == BOARD_E1580) {
		nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert port assumed TEGRA_GPIO_PX6 for unknown pluto board id E%d\n",
		       board_info.board_id);
	}
#else
	/* pluto + AP30 interposer has SPI2_CS0 gpio */
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
			pluto_nct1008_pdata.active[i].create_cdev =
				(struct thermal_cooling_device *(*)(void *))
					edp_cooling_device_create;
			pluto_nct1008_pdata.active[i].cdev_data = (void *)i;
			pluto_nct1008_pdata.active[i].trip_temp =
				cpu_edp_limits[i].temperature * 1000;
			pluto_nct1008_pdata.active[i].hysteresis = 1000;
		}
		pluto_nct1008_pdata.active[i].create_cdev = NULL;
#endif

		pluto_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
		pr_info("%s: pluto nct1008 irq %d", __func__, pluto_i2c4_nct1008_board_info[0].irq);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0) {
			pr_info("%s: calling gpio_free(nct1008_port)", __func__);
			gpio_free(nct1008_port);
		}
	}

	/* pluto has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, pluto_i2c4_nct1008_board_info,
		ARRAY_SIZE(pluto_i2c4_nct1008_board_info));

	return ret;
}

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

static int __init pluto_skin_init(void)
{
	struct thermal_cooling_device *skin_cdev;

	skin_cdev = balanced_throttle_register(&skin_throttle);

	skin_data.cdev = skin_cdev;
	tegra_skin_therm_est_device.dev.platform_data = &skin_data;
	platform_device_register(&tegra_skin_therm_est_device);

	return 0;
}
late_initcall(pluto_skin_init);
#endif

int __init pluto_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	pr_debug("%s: ++\n", __func__);
	pluto_camera_init();

	err = pluto_nct1008_init();
	if (err)
		return err;

	i2c_register_board_info(0, pluto_i2c1_isl_board_info,
				ARRAY_SIZE(pluto_i2c1_isl_board_info));

	return 0;
}
