/*
 * arch/arm/mach-tegra/board-loki-sensors.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/nct1008.h>
#include <linux/tegra-fuse.h>
#include <linux/of_platform.h>
#include <media/camera.h>
#include <media/mt9m114.h>
#include <media/ov7695.h>
#include <mach/gpio-tegra.h>
#include <mach/edp.h>
#include <mach/io_dpd.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pid_thermal_gov.h>

#include "board.h"
#include "board-common.h"
#include "board-loki.h"
#include "tegra-board-id.h"
#include "dvfs.h"
#include "cpu-tegra.h"

static struct board_info board_info;

#ifndef CONFIG_USE_OF

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data = {
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION,
};

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data_fab_0 = {
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION_FAB0,
};

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data_t_1_95 = {
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION_T_1_95,
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation	= MPU_COMPASS_ORIENTATION,
};

static struct i2c_board_info __initdata inv_mpu6050_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu6050_gyro_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu_compass_data,
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

	if (board_info.board_id == BOARD_E2549)
		inv_mpu6050_i2c0_board_info[0].platform_data =
						&mpu6050_gyro_data_t_1_95;
	else {
		struct board_info displayboard_info;
		tegra_get_display_board_info(&displayboard_info);
		if (displayboard_info.fab == 0x0)
			inv_mpu6050_i2c0_board_info[0].platform_data =
				&mpu6050_gyro_data_fab_0;
	}
	inv_mpu6050_i2c0_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu6050_i2c0_board_info,
		ARRAY_SIZE(inv_mpu6050_i2c0_board_info));
}

#else

static void mpu_dt_update(void)
{
	struct device_node *np;
	struct board_info displayboard_info;
	static signed char mpu_gyro_orientation_1_95[] = {
		0,  1,  0,  0,  0,  1,  1,  0,  0
	};
	static signed char mpu_gyro_orientation_fab0[] = {
		0, -1,  0, -1,  0,  0,  0,  0, -1
	};
	static struct property orientation = {
		.name = "invensense,orientation",
		.value = mpu_gyro_orientation_1_95,
		.length = sizeof(mpu_gyro_orientation_1_95),
	};

	np = of_find_compatible_node(NULL, NULL, "invensense,mpu6050");
	if (np == NULL) {
		pr_err("%s: Cannot find mpu6050 node\n", __func__);
		return;
	}

	if (board_info.board_id == BOARD_E2549) {
		of_update_property(np, &orientation);
	} else {
		tegra_get_display_board_info(&displayboard_info);
		if (displayboard_info.fab == 0x0) {
			orientation.value = mpu_gyro_orientation_fab0;
			of_update_property(np, &orientation);
		}
	}

	of_node_put(np);
}

#endif

static struct tegra_io_dpd csia_io = {
	.name			= "CSIA",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 0,
};

static struct tegra_io_dpd csib_io = {
	.name			= "CSIB",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 1,
};

static struct tegra_io_dpd csie_io = {
	.name			= "CSIE",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 12,
};

static int loki_mt9m114_power_on(struct mt9m114_power_rail *pw)
{
	int err;
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	/* disable CSIA IOs DPD mode to turn on front camera for ardbeg */
	tegra_io_dpd_disable(&csia_io);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM2_PWDN, 1);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto mt9m114_iovdd_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto mt9m114_avdd_fail;

	usleep_range(1000, 1020);
	gpio_set_value(CAM_RSTN, 1);
	gpio_set_value(CAM2_PWDN, 0);
	usleep_range(1000, 1020);

	/* return 1 to skip the in-driver power_on swquence */
	return 1;

mt9m114_avdd_fail:
	regulator_disable(pw->iovdd);

mt9m114_iovdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return -ENODEV;
}

static int loki_mt9m114_power_off(struct mt9m114_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd)) {
		/* put CSIA IOs into DPD mode to
		 * save additional power for ardbeg
		 */
		tegra_io_dpd_enable(&csia_io);
		return -EFAULT;
	}

	usleep_range(100, 120);
	gpio_set_value(CAM_RSTN, 0);
	usleep_range(100, 120);
	regulator_disable(pw->avdd);
	usleep_range(100, 120);
	regulator_disable(pw->iovdd);

	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return 1;
}

struct mt9m114_platform_data loki_mt9m114_pdata = {
	.power_on = loki_mt9m114_power_on,
	.power_off = loki_mt9m114_power_off,
};

static int loki_ov7695_power_on(struct ov7695_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd)))
		return -EFAULT;

	/* disable CSIA IOs DPD mode to turn on front camera for ardbeg */
	tegra_io_dpd_disable(&csia_io);

	gpio_set_value(CAM2_PWDN, 0);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ov7695_avdd_fail;
	usleep_range(300, 320);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto ov7695_iovdd_fail;
	usleep_range(1000, 1020);

	gpio_set_value(CAM2_PWDN, 1);
	usleep_range(1000, 1020);

	return 0;

ov7695_iovdd_fail:
	regulator_disable(pw->avdd);

ov7695_avdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return -ENODEV;
}

static int loki_ov7695_power_off(struct ov7695_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd))) {
		/* put CSIA IOs into DPD mode to
		 * save additional power for ardbeg
		 */
		tegra_io_dpd_enable(&csia_io);
		return -EFAULT;
	}

	usleep_range(100, 120);

	regulator_disable(pw->iovdd);
	usleep_range(100, 120);

	regulator_disable(pw->avdd);
	usleep_range(100, 120);

	gpio_set_value(CAM2_PWDN, 0);

	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return 0;
}

struct ov7695_platform_data loki_ov7695_pdata = {
	.power_on = loki_ov7695_power_on,
	.power_off = loki_ov7695_power_off,
};

static struct camera_data_blob loki_camera_lut[] = {
	{"loki_ov7695_pdata", &loki_ov7695_pdata},
	{},
};

void __init loki_camera_auxdata(void *data)
{
	struct of_dev_auxdata *aux_lut = data;
	while (aux_lut && aux_lut->compatible) {
		if (!strcmp(aux_lut->compatible, "nvidia,tegra124-camera")) {
			pr_info("%s: update camera lookup table.\n", __func__);
			aux_lut->platform_data = loki_camera_lut;
		}
		aux_lut++;
	}
}

static int loki_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	/* put CSIA/B/E IOs into DPD mode to
	 * save additional power
	 */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	tegra_io_dpd_enable(&csie_io);

	/* Don't turn on CAM_MCLK for FFD fab a3 or higher */
	if (board_info.board_id == BOARD_P2530 && board_info.fab >= 0xa3)
		loki_ov7695_pdata.mclk_name = "ext_mclk";

	return 0;
}

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,    GPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 2295000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2269500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2244000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2218500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2193000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2167500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2142000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2116500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2091000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2065500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2040000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2014500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1989000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1963500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1938000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1912500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1887000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1861500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1836000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, 790000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, 776000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, 762000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, 749000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, 735000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, 721000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, 707000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, 693000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, 679000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, 666000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, 652000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, 638000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, 624000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, 610000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, 596000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, 582000, NO_CAP, NO_CAP, NO_CAP, 792000 } },
	{ { 1198500, 569000, NO_CAP, NO_CAP, NO_CAP, 792000 } },
	{ { 1173000, 555000, NO_CAP, NO_CAP, 360000, 792000 } },
	{ { 1147500, 541000, NO_CAP, NO_CAP, 360000, 792000 } },
	{ { 1122000, 527000, NO_CAP, 684000, 360000, 792000 } },
	{ { 1096500, 513000, 444000, 684000, 360000, 792000 } },
	{ { 1071000, 499000, 444000, 684000, 360000, 792000 } },
	{ { 1045500, 486000, 444000, 684000, 360000, 792000 } },
	{ { 1020000, 472000, 444000, 684000, 324000, 792000 } },
	{ {  994500, 458000, 444000, 684000, 324000, 792000 } },
	{ {  969000, 444000, 444000, 600000, 324000, 792000 } },
	{ {  943500, 430000, 444000, 600000, 324000, 792000 } },
	{ {  918000, 416000, 396000, 600000, 324000, 792000 } },
	{ {  892500, 402000, 396000, 600000, 324000, 792000 } },
	{ {  867000, 389000, 396000, 600000, 324000, 792000 } },
	{ {  841500, 375000, 396000, 600000, 288000, 792000 } },
	{ {  816000, 361000, 396000, 600000, 288000, 792000 } },
	{ {  790500, 347000, 396000, 600000, 288000, 792000 } },
	{ {  765000, 333000, 396000, 504000, 288000, 792000 } },
	{ {  739500, 319000, 348000, 504000, 288000, 792000 } },
	{ {  714000, 306000, 348000, 504000, 288000, 624000 } },
	{ {  688500, 292000, 348000, 504000, 288000, 624000 } },
	{ {  663000, 278000, 348000, 504000, 288000, 624000 } },
	{ {  637500, 264000, 348000, 504000, 288000, 624000 } },
	{ {  612000, 250000, 348000, 504000, 252000, 624000 } },
	{ {  586500, 236000, 348000, 504000, 252000, 624000 } },
	{ {  561000, 222000, 348000, 420000, 252000, 624000 } },
	{ {  535500, 209000, 288000, 420000, 252000, 624000 } },
	{ {  510000, 195000, 288000, 420000, 252000, 624000 } },
	{ {  484500, 181000, 288000, 420000, 252000, 624000 } },
	{ {  459000, 167000, 288000, 420000, 252000, 624000 } },
	{ {  433500, 153000, 288000, 420000, 252000, 396000 } },
	{ {  408000, 139000, 288000, 420000, 252000, 396000 } },
	{ {  382500, 126000, 288000, 420000, 252000, 396000 } },
	{ {  357000, 112000, 288000, 420000, 252000, 396000 } },
	{ {  331500,  98000, 288000, 420000, 252000, 396000 } },
	{ {  306000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  280500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  255000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  229500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  204000,  84000, 288000, 420000, 252000, 396000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init loki_throttle_init(void)
{
	if (of_machine_is_compatible("nvidia,loki") ||
		of_machine_is_compatible("nvidia,t132loki"))
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(loki_throttle_init);

static int loki_fan_est_match(struct thermal_zone_device *thz, void *data)
{
	return (strcmp((char *)data, thz->type) == 0);
}

static int loki_fan_est_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, loki_fan_est_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static struct thermal_zone_params fan_tzp = {
	.governor_name = "pid_thermal_gov",
};

static int active_trip_temps_loki[] = {0, 60000, 68000, 79000, 90000,
				140000, 150000, 160000, 170000, 180000};
static int active_hysteresis_loki[] = {0, 20000, 7000, 10000, 10000,
							0, 0, 0, 0, 0};

static int active_trip_temps_foster[] = {0, 63000, 74000, 85000, 120000,
				140000, 150000, 160000, 170000, 180000};
static int active_hysteresis_foster[] = {0, 15000, 11000, 6000, 4000,
							0, 0, 0, 0, 0};
/*Fan thermal estimator data for P2548*/
static struct therm_fan_est_data fan_est_data = {
	.toffset = 0,
	.polling_period = 1100,
	.ndevs = 2,
	.devs = {
			{
				.dev_data = "CPU-therm",
				.get_temp = loki_fan_est_get_temp,
				.coeffs = {
					50, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
			{
				.dev_data = "GPU-therm",
				.get_temp = loki_fan_est_get_temp,
				.coeffs = {
					50, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
	},
	.cdev_type = "pwm-fan",
	.tzp = &fan_tzp,
};

static struct platform_device loki_fan_therm_est_device = {
	.name   = "therm-fan-est",
	.id     = -1,
	.num_resources  = 0,
	.dev = {
		.platform_data = &fan_est_data,
	},
};

static int __init loki_fan_est_init(void)
{
	if ((board_info.sku == 900) && (board_info.board_id == BOARD_P2530)) {
		memcpy((&fan_est_data)->active_trip_temps,
				&active_trip_temps_foster,
				sizeof(active_trip_temps_foster));
		memcpy((&fan_est_data)->active_hysteresis,
				&active_hysteresis_foster,
				sizeof(active_hysteresis_foster));
	} else {
		memcpy((&fan_est_data)->active_trip_temps,
				&active_trip_temps_loki,
				sizeof(active_trip_temps_loki));
		memcpy((&fan_est_data)->active_hysteresis,
				&active_hysteresis_loki,
				sizeof(active_hysteresis_loki));
	}

	platform_device_register(&loki_fan_therm_est_device);

	return 0;
}

int __init loki_sensors_init(void)
{
	tegra_get_board_info(&board_info);

	loki_fan_est_init();
	if (!(board_info.board_id == BOARD_P2530 &&
		board_info.sku == BOARD_SKU_FOSTER)) {
#ifndef CONFIG_USE_OF
		mpuirq_init();
#else
		mpu_dt_update();
#endif

		if (board_info.board_id != BOARD_E2549)
			loki_camera_init();
	}
	return 0;
}
