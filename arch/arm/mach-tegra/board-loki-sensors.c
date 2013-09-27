/*
 * arch/arm/mach-tegra/board-loki-sensors.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <media/mt9m114.h>
#include <mach/gpio-tegra.h>
#include <mach/edp.h>
#include <mach/tegra_fuse.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/iio/light/jsa1127.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "board.h"
#include "board-common.h"
#include "board-loki.h"
#include "tegra-board-id.h"
#include "dvfs.h"
#include "cpu-tegra.h"

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data = {
	.int_config     = 0x10,
	.level_shifter  = 0,
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION,
	.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
	.key            = {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};


static struct i2c_board_info __initdata inv_mpu6050_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu6050_gyro_data,
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

	inv_mpu6050_i2c0_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu6050_i2c0_board_info,
		ARRAY_SIZE(inv_mpu6050_i2c0_board_info));
}

struct jsa1127_platform_data jsa1127_platform_data = {
	.rint = 100,
	.integration_time = 200,
	.use_internal_integration_timing = 1,
};

static struct i2c_board_info loki_i2c_jsa1127_board_info[] = {
	{
		I2C_BOARD_INFO(JSA1127_NAME, JSA1127_SLAVE_ADDRESS),
		.platform_data = &jsa1127_platform_data,
	}
};

static void loki_jsa1127_init(void)
{
	i2c_register_board_info(0, loki_i2c_jsa1127_board_info,
		ARRAY_SIZE(loki_i2c_jsa1127_board_info));
}

static int loki_mt9m114_power_on(struct mt9m114_power_rail *pw)
{
	int err;
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

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
	return -ENODEV;
}

static int loki_mt9m114_power_off(struct mt9m114_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	usleep_range(100, 120);
	gpio_set_value(CAM_RSTN, 0);
	usleep_range(100, 120);
	regulator_disable(pw->avdd);
	usleep_range(100, 120);
	regulator_disable(pw->iovdd);

	return 1;
}

struct mt9m114_platform_data loki_mt9m114_pdata = {
	.power_on = loki_mt9m114_power_on,
	.power_off = loki_mt9m114_power_off,
};

static struct i2c_board_info loki_i2c_board_info_e2548[] = {
	{
		I2C_BOARD_INFO("mt9m114", 0x48),
		.platform_data = &loki_mt9m114_pdata,
	},
};

static int loki_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	i2c_register_board_info(2, loki_i2c_board_info_e2548,
			ARRAY_SIZE(loki_i2c_board_info_e2548));
	return 0;
}

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init loki_throttle_init(void)
{
	if (of_machine_is_compatible("nvidia,loki"))
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(loki_throttle_init);

static struct nct1008_platform_data loki_nct72_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.shutdown_ext_limit = 95, /* C */
	.shutdown_local_limit = 120, /* C */

	.passive_delay = 2000,

	.num_trips = 3,
	.trips = {
		/* Thermal Throttling */
		[0] = {
			.cdev_type = "tegra-balanced",
			.trip_temp = 83000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
			.hysteresis = 1000,
		},
		[1] = {
			.cdev_type = "tegra-hard",
			.trip_temp = 86000, /* shutdown_ext_limit - 2C */
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 6000,
		},
		[2] = {
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

static struct i2c_board_info loki_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4c),
		.platform_data = &loki_nct72_pdata,
		.irq = -1,
	}
};

static int loki_nct72_init(void)
{
	int nct72_port = TEGRA_GPIO_PI6;
	int ret = 0;
	int i;
	struct thermal_trip_info *trip_state;

	/* Raise NCT's thresholds if soctherm CP,FT fuses are ok */
	if (!tegra_fuse_calib_base_get_cp(NULL, NULL) &&
	    !tegra_fuse_calib_base_get_ft(NULL, NULL)) {
		/* Raise NCT's shutdown point by 20C */
		loki_nct72_pdata.shutdown_ext_limit += 20;
		/* Remove tegra-balanced cooling device from NCT pdata */
		for (i = 0; i < loki_nct72_pdata.num_trips; i++) {
			trip_state = &loki_nct72_pdata.trips[i];
			if (!strncmp(trip_state->cdev_type, "tegra-balanced",
					THERMAL_NAME_LENGTH)) {
				trip_state->cdev_type = "_none_";
				break;
			}
		}
	} else {
		tegra_platform_edp_init(loki_nct72_pdata.trips,
					&loki_nct72_pdata.num_trips,
					12000); /* edp temperature margin */
	}


	tegra_add_cdev_trips(loki_nct72_pdata.trips,
				&loki_nct72_pdata.num_trips);
	tegra_add_tj_trips(loki_nct72_pdata.trips,
				&loki_nct72_pdata.num_trips);
	loki_i2c_nct72_board_info[0].irq = gpio_to_irq(nct72_port);

	ret = gpio_request(nct72_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct72_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct72_port)", __func__);
		gpio_free(nct72_port);
	}

	/* loki has thermal sensor on GEN2-I2C i.e. instance 1 */
	i2c_register_board_info(0, loki_i2c_nct72_board_info,
		ARRAY_SIZE(loki_i2c_nct72_board_info));

	return ret;
}

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

/*Fan thermal estimator data for P2548*/
static struct therm_fan_est_data fan_est_data_p2548 = {
	.toffset = 0,
	.polling_period = 1100,
	.ndevs = 2,
	.devs = {
			{
				.dev_data = "Tdiode_soc",
				.get_temp = loki_fan_est_get_temp,
				.coeffs = {
					100, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
			{
				.dev_data = "Tboard_soc",
				.get_temp = loki_fan_est_get_temp,
				.coeffs = {
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
	},
	.cdev_type = "pwm-fan",
	.active_trip_temps = {0, 47000, 55000, 67000, 103000,
				140000, 150000, 160000, 170000, 180000},
	.active_hysteresis = {0, 12000, 7000, 10000, 0, 0, 0, 0, 0, 0},
};

static struct platform_device loki_fan_therm_est_device_p2548 = {
	.name   = "therm-fan-est",
	.id     = -1,
	.num_resources  = 0,
	.dev = {
		.platform_data = &fan_est_data_p2548,
	},
};

static int __init loki_fan_est_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	platform_device_register(&loki_fan_therm_est_device_p2548);

	return 0;
}
int __init loki_sensors_init(void)
{
	mpuirq_init();
	loki_camera_init();
	loki_nct72_init();
	loki_jsa1127_init();
	loki_fan_est_init();

	return 0;
}
