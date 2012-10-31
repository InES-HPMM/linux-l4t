/*
 * arch/arm/mach-tegra/board-pluto-sensors.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/max77665-charger.h>
#include <linux/mfd/max77665.h>
#include <linux/input/max77665-haptic.h>
#include <linux/power/max17042_battery.h>
#include <linux/nct1008.h>
#include <mach/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <media/max77665-flash.h>
#include <media/imx091.h>
#include <media/imx132.h>
#include <media/ad5816.h>

#include "gpio-names.h"
#include "board.h"
#include "board-pluto.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"

#define NTC_10K_TGAIN   0xE6A2
#define NTC_10K_TOFF    0x2694

static struct board_info board_info;
static struct max17042_config_data conf_data = {
	.valrt_thresh = 0xff00,
	.talrt_thresh = 0xff00,
	.soc_alrt_thresh = 0xff00,
	.shdntimer = 0xe000,
	.design_cap = 0x07d0,
	.at_rate = 0x0000,
	.tgain = NTC_10K_TGAIN,
	.toff = NTC_10K_TOFF,
	.vempty = 0xACDA,
	.qrtbl00 = 0x5C80,
	.qrtbl10 = 0x438C,
	.qrtbl20 = 0x1198,
	.qrtbl30 = 0x0E19,
	.full_soc_thresh = 0x5A00,
	.rcomp0 = 0x0077,
	.tcompc0 = 0x1F2A,
	.ichgt_term = 0x0320,
	.temp_nom = 0x1400,
	.temp_lim = 0x2305,
	.filter_cfg = 0x87A4,
	.config = 0x2210,
	.learn_cfg = 0x2606,
	.misc_cfg = 0x0810,
	.fullcap =  0x07d0,
	.fullcapnom = 0x07d0,
	.lavg_empty = 0x1000,
	.dqacc = 0x01f4,
	.dpacc = 0x3200,
	.fctc = 0x05e0,
	.kempty0 = 0x0600,
	.cell_technology = POWER_SUPPLY_TECHNOLOGY_LION,
	.cell_char_tbl = {
		/* Data to be written from 0x80h */
		0x9180, 0xA4C0, 0xB6A0, 0xB760, 0xB980, 0xBB30,
		0xBBC0, 0xBC50, 0xBD50, 0xBE50, 0xBF80, 0xC290,
		0xC470, 0xC7D0, 0xCC40, 0xCFB0,
		/* Data to be written from 0x90h */
		0x00C0, 0x0200, 0x1C10, 0x0B00, 0x0900, 0x1F00,
		0x1F00, 0x23C0, 0x1990, 0x19F0, 0x09A0, 0x0CE0,
		0x0BE0, 0x07D0, 0x0990, 0x0990,
		/* Data to be written from 0xA0h */
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100,
	},
};

static struct max17042_platform_data max17042_pdata = {
	.config_data = &conf_data,
	.init_data  = NULL,
	.num_init_data = 0,
	.enable_por_init = 1, /* Use POR init from Maxim appnote */
	.enable_current_sense = 1,
	.r_sns = 0,
};

static struct i2c_board_info max17042_device[] = {
	{
		I2C_BOARD_INFO("max17042", 0x36),
		.platform_data = &max17042_pdata,
	},
};

static struct max77665_f_platform_data pluto_max77665_flash_pdata = {
	.config		= {
		.led_mask		= 3,
		.flash_on_torch         = true,
		.max_total_current_mA	= 1000,
		.max_peak_current_mA	= 600,
		},
	.pinstate	= {
		.mask	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0),
		.values	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0),
		},
	.dev_name	= "torch",
	.gpio_strobe	= CAM_FLASH_STROBE,
};

static struct max77665_haptic_platform_data max77665_haptic_pdata = {
	.pwm_channel_id = 2,
	.pwm_period = 50,
	.type = MAX77665_HAPTIC_LRA,
	.mode = MAX77665_INTERNAL_MODE,
	.internal_mode_pattern = 0,
	.pattern_cycle = 10,
	.pattern_signal_period = 0xD0,
	.pwm_divisor = MAX77665_PWM_DIVISOR_128,
	.feedback_duty_cycle = 12,
	.invert = MAX77665_INVERT_OFF,
	.cont_mode = MAX77665_CONT_MODE,
	.motor_startup_val = 0,
	.scf_val = 2,
};

static struct max77665_charger_cable maxim_cable[] = {
	{
		.name           = "USB",
	},
	{
		.name           = "USB-Host",
	},
	{
		.name           = "TA",
	},
	{
		.name           = "Fast-charger",
	},
	{
		.name           = "Slow-charger",
	},
	{
		.name           = "Charge-downstream",
	},
};

static struct max77665_charger_plat_data max77665_charger = {
	.fast_chg_cc = 1500, /* fast charger current*/
	.term_volt = 3700, /* charger termination voltage */
	.curr_lim = 1500, /* input current limit */
	.num_cables = MAX_CABLES,
	.cables = maxim_cable,
};

static struct max77665_muic_platform_data max77665_muic = {
	.irq_base = 0,
};

static struct max77665_platform_data pluto_max77665_pdata = {
	.irq_base = 0,
	.muic_platform_data = {
		.pdata = &max77665_muic,
		.size =	sizeof(max77665_muic),
		},
	.charger_platform_data = {
		.pdata = &max77665_charger,
		.size =	sizeof(max77665_charger),
		},
	.flash_platform_data = {
		.pdata = &pluto_max77665_flash_pdata,
		.size =	sizeof(pluto_max77665_flash_pdata),
		},
	.haptic_platform_data = {
		.pdata = &max77665_haptic_pdata,
		.size = sizeof(max77665_haptic_pdata),
		},
};

static const struct i2c_board_info pluto_i2c_board_info_max77665[] = {
	{
		I2C_BOARD_INFO("max77665", 0x66),
		.platform_data = &pluto_max77665_pdata,
	},
};

/* isl29029 support is provided by isl29028*/
static struct i2c_board_info pluto_i2c1_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29028", 0x44),
	}
};

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
 * more regulators need to be allocated to activate the sensor devices.
 * pluto_vcmvdd: this is a workaround due to the focuser device(AD5816) will
 *               hook up the i2c bus if it is not powered up.
 * pluto_i2cvdd: by default, the power supply on the i2c bus is OFF. So it
 *               should be turned on every time any sensor device is activated.
*/
static struct regulator *pluto_vcmvdd;
static struct regulator *pluto_i2cvdd;

static int pluto_get_extra_regulators(void)
{
	if (!pluto_vcmvdd) {
		pluto_vcmvdd = regulator_get(NULL, "vdd_af_cam1");
		if (WARN_ON(IS_ERR(pluto_vcmvdd))) {
			pr_err("%s: can't get regulator vdd_af_cam1: %ld\n",
				__func__, PTR_ERR(pluto_vcmvdd));
			pluto_vcmvdd = NULL;
			return -ENODEV;
		}
	}

	if (!pluto_i2cvdd) {
		pluto_i2cvdd = regulator_get(NULL, "vddio_cam_mb");
		if (unlikely(WARN_ON(IS_ERR(pluto_i2cvdd)))) {
			pr_err("%s: can't get regulator vddio_cam_mb: %ld\n",
				__func__, PTR_ERR(pluto_i2cvdd));
			pluto_i2cvdd = NULL;
			return -ENODEV;
		}
	}

	return 0;
}

static int pluto_imx091_power_on(struct imx091_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->dvdd || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	if (pluto_get_extra_regulators())
		goto imx091_poweron_fail;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto imx091_avdd_fail;

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto imx091_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto imx091_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	tegra_pinmux_config_table(&mclk_enable, 1);
	err = regulator_enable(pluto_i2cvdd);
	if (unlikely(err))
		goto imx091_i2c_fail;

	err = regulator_enable(pluto_vcmvdd);
	if (unlikely(err))
		goto imx091_vcm_fail;
	usleep_range(300, 310);

	return 0;

imx091_vcm_fail:
	regulator_disable(pluto_i2cvdd);

imx091_i2c_fail:
	tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	regulator_disable(pw->iovdd);

imx091_iovdd_fail:
	regulator_disable(pw->dvdd);

imx091_dvdd_fail:
	regulator_disable(pw->avdd);

imx091_avdd_fail:
imx091_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int pluto_imx091_power_off(struct imx091_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pluto_i2cvdd || !pluto_vcmvdd ||
			!pw->dvdd || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	usleep_range(1, 2);
	tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);
	regulator_disable(pluto_i2cvdd);
	regulator_disable(pluto_vcmvdd);

	return 0;
}

struct imx091_platform_data pluto_imx091_data = {
	.power_on = pluto_imx091_power_on,
	.power_off = pluto_imx091_power_off,
};

static int pluto_imx132_power_on(struct imx132_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	if (pluto_get_extra_regulators())
		goto pluto_imx132_poweron_fail;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

	tegra_pinmux_config_table(&pbb0_enable, 1);

	err = regulator_enable(pluto_i2cvdd);
	if (unlikely(err))
		goto imx132_i2c_fail;

	err = regulator_enable(pluto_vcmvdd);
	if (unlikely(err))
		goto imx132_vcm_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto imx132_avdd_fail;

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto imx132_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto imx132_iovdd_fail;

	usleep_range(1, 2);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);

	return 0;

imx132_iovdd_fail:
	regulator_disable(pw->dvdd);

imx132_dvdd_fail:
	regulator_disable(pw->avdd);

imx132_avdd_fail:
	regulator_disable(pluto_vcmvdd);

imx132_vcm_fail:
	regulator_disable(pluto_i2cvdd);

imx132_i2c_fail:
	tegra_pinmux_config_table(&pbb0_disable, 1);

pluto_imx132_poweron_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int pluto_imx132_power_off(struct imx132_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd ||
			!pluto_i2cvdd || !pluto_vcmvdd)))
		return -EFAULT;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	tegra_pinmux_config_table(&pbb0_disable, 1);

	regulator_disable(pluto_vcmvdd);
	regulator_disable(pluto_i2cvdd);

	return 0;
}

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

static int pluto_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	tegra_pinmux_config_table(&mclk_disable, 1);
	tegra_pinmux_config_table(&pbb0_disable, 1);

	i2c_register_board_info(2, pluto_i2c_board_info_e1625,
		ARRAY_SIZE(pluto_i2c_board_info_e1625));

	return 0;
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
	if (board_info.board_id == BOARD_E1580 ||
	    board_info.board_id == BOARD_E1575) {
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

	err = i2c_register_board_info(0, pluto_i2c1_isl_board_info,
				ARRAY_SIZE(pluto_i2c1_isl_board_info));
	if (err)
		pr_err("%s: isl board register failed.\n", __func__);

	mpuirq_init();

	err = i2c_register_board_info(4, pluto_i2c_board_info_max77665,
		ARRAY_SIZE(pluto_i2c_board_info_max77665));
	if (err)
		pr_err("%s: max77665 device register failed.\n", __func__);

	err = i2c_register_board_info(0, max17042_device,
				ARRAY_SIZE(max17042_device));
	if (err)
		pr_err("%s: max17042 device register failed.\n", __func__);


	return 0;
}
