/*
 * arch/arm/mach-tegra/board-ardbeg-sensors.c
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
#include <linux/pid_thermal_gov.h>
#include <media/ar0261.h>
#include <media/imx135.h>
#include <media/dw9718.h>
#include <media/as364x.h>
#include <media/ov5693.h>
#include <media/ov7695.h>
#include <media/ad5823.h>
#include <linux/pid_thermal_gov.h>
#include <linux/power/sbs-battery.h>
#include <mach/edp.h>
#include <mach/tegra_fuse.h>
#include <mach/pinmux-t12.h>
#include <mach/pinmux.h>
#include <mach/io_dpd.h>

#include "cpu-tegra.h"
#include "devices.h"
#include "board.h"
#include "board-common.h"
#include "board-ardbeg.h"
#include "tegra-board-id.h"

static struct i2c_board_info ardbeg_i2c_board_info_cm32181[] = {
	{
		I2C_BOARD_INFO("cm32181", 0x48),
	},
};

/* MPU board file definition    */
static struct mpu_platform_data mpu9250_gyro_data = {
	.int_config     = 0x10,
	.level_shifter  = 0,
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION,
	.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
	.key            = {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation    = MPU_COMPASS_ORIENTATION,
	.config         = NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data mpu_bmp_pdata = {
	.config         = NVI_CONFIG_BOOT_MPU,
};

static struct i2c_board_info __initdata inv_mpu9250_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu9250_gyro_data,
	},
	{
		/* The actual BMP180 address is 0x77 but because this conflicts
		 * with another device, this address is hacked so Linux will
		 * call the driver.  The conflict is technically okay since the
		 * BMP180 is behind the MPU.  Also, the BMP180 driver uses a
		 * hard-coded address of 0x77 since it can't be changed anyway.
		 */
		I2C_BOARD_INFO(MPU_BMP_NAME, MPU_BMP_ADDR),
		.platform_data = &mpu_bmp_pdata,
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

	/* TN8 with diferent Compass address from ardbeg */
	if (of_machine_is_compatible("nvidia,tn8"))
		inv_mpu9250_i2c0_board_info[2].addr = MPU_COMPASS_ADDR_TN8;

	inv_mpu9250_i2c0_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu9250_i2c0_board_info,
		ARRAY_SIZE(inv_mpu9250_i2c0_board_info));
}

static struct regulator *ardbeg_vcmvdd;

static int ardbeg_get_extra_regulators(void)
{
	if (!ardbeg_vcmvdd) {
		ardbeg_vcmvdd = regulator_get(NULL, "avdd_af1_cam");
		if (WARN_ON(IS_ERR(ardbeg_vcmvdd))) {
			pr_err("%s: can't get regulator avdd_af1_cam: %ld\n",
					__func__, PTR_ERR(ardbeg_vcmvdd));
			regulator_put(ardbeg_vcmvdd);
			ardbeg_vcmvdd = NULL;
			return -ENODEV;
		}
	}

	return 0;
}

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

static int ardbeg_ar0261_power_on(struct ar0261_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	/* disable CSIE IOs DPD mode to turn on front camera for ardbeg */
	tegra_io_dpd_disable(&csie_io);

	if (ardbeg_get_extra_regulators())
		goto ardbeg_ar0261_poweron_fail;

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 1);


	err = regulator_enable(ardbeg_vcmvdd);
	if (unlikely(err))
		goto ar0261_vcm_fail;

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto ar0261_dvdd_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ar0261_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto ar0261_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM2_PWDN, 1);

	gpio_set_value(CAM_RSTN, 1);

	return 0;
ar0261_iovdd_fail:
	regulator_disable(pw->dvdd);

ar0261_dvdd_fail:
	regulator_disable(pw->avdd);

ar0261_avdd_fail:
	regulator_disable(ardbeg_vcmvdd);

ar0261_vcm_fail:
	pr_err("%s vcmvdd failed.\n", __func__);
	return -ENODEV;

ardbeg_ar0261_poweron_fail:
	/* put CSIE IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csie_io);
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int ardbeg_ar0261_power_off(struct ar0261_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd ||
					!ardbeg_vcmvdd))) {
		/* put CSIE IOs into DPD mode to
		 * save additional power for ardbeg
		 */
		tegra_io_dpd_enable(&csie_io);
		return -EFAULT;
	}

	gpio_set_value(CAM_RSTN, 0);

	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);


	regulator_disable(ardbeg_vcmvdd);
	/* put CSIE IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csie_io);
	return 0;
}

struct ar0261_platform_data ardbeg_ar0261_data = {
	.power_on = ardbeg_ar0261_power_on,
	.power_off = ardbeg_ar0261_power_off,
	.mclk_name = "mclk2",
};

static int ardbeg_imx135_get_extra_regulators(struct imx135_power_rail *pw)
{
	if (!pw->ext_reg1) {
		pw->ext_reg1 = regulator_get(NULL, "imx135_reg1");
		if (WARN_ON(IS_ERR(pw->ext_reg1))) {
			pr_err("%s: can't get regulator imx135_reg1: %ld\n",
				__func__, PTR_ERR(pw->ext_reg1));
			pw->ext_reg1 = NULL;
			return -ENODEV;
		}
	}

	if (!pw->ext_reg2) {
		pw->ext_reg2 = regulator_get(NULL, "imx135_reg2");
		if (WARN_ON(IS_ERR(pw->ext_reg2))) {
			pr_err("%s: can't get regulator imx135_reg2: %ld\n",
				__func__, PTR_ERR(pw->ext_reg2));
			pw->ext_reg2 = NULL;
			return -ENODEV;
		}
	}

	return 0;
}

static int ardbeg_imx135_power_on(struct imx135_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for ardbeg */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	if (ardbeg_imx135_get_extra_regulators(pw))
		goto imx135_poweron_fail;

	err = regulator_enable(pw->ext_reg1);
	if (unlikely(err))
		goto imx135_ext_reg1_fail;

	err = regulator_enable(pw->ext_reg2);
	if (unlikely(err))
		goto imx135_ext_reg2_fail;


	gpio_set_value(CAM_AF_PWDN, 1);
	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx135_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx135_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_PWDN, 1);

	usleep_range(300, 310);

	return 1;


imx135_iovdd_fail:
	regulator_disable(pw->avdd);

imx135_avdd_fail:
	if (pw->ext_reg2)
		regulator_disable(pw->ext_reg2);

imx135_ext_reg2_fail:
	if (pw->ext_reg1)
		regulator_disable(pw->ext_reg1);
	gpio_set_value(CAM_AF_PWDN, 0);

imx135_ext_reg1_fail:
imx135_poweron_fail:
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int ardbeg_imx135_power_off(struct imx135_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd))) {
		tegra_io_dpd_enable(&csia_io);
		tegra_io_dpd_enable(&csib_io);
		return -EFAULT;
	}

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	regulator_disable(pw->ext_reg1);
	regulator_disable(pw->ext_reg2);

	/* put CSIA/B IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	return 0;
}

struct imx135_platform_data ardbeg_imx135_data = {
	.flash_cap = {
		.enable = 1,
		.edge_trig_en = 1,
		.start_edge = 0,
		.repeat = 1,
		.delay_frm = 0,
	},
	.power_on = ardbeg_imx135_power_on,
	.power_off = ardbeg_imx135_power_off,
};

static int ardbeg_dw9718_power_on(struct dw9718_power_rail *pw)
{
	int err;
	pr_info("%s\n", __func__);

	if (unlikely(!pw || !pw->vdd || !pw->vdd_i2c))
		return -EFAULT;

	err = regulator_enable(pw->vdd);
	if (unlikely(err))
		goto dw9718_vdd_fail;

	err = regulator_enable(pw->vdd_i2c);
	if (unlikely(err))
		goto dw9718_i2c_fail;

	usleep_range(1000, 1020);

	/* return 1 to skip the in-driver power_on sequence */
	pr_debug("%s --\n", __func__);
	return 1;

dw9718_i2c_fail:
	regulator_disable(pw->vdd);

dw9718_vdd_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int ardbeg_dw9718_power_off(struct dw9718_power_rail *pw)
{
	pr_info("%s\n", __func__);

	if (unlikely(!pw || !pw->vdd || !pw->vdd_i2c))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 1;
}

static u16 dw9718_devid;
static int ardbeg_dw9718_detect(void *buf, size_t size)
{
	dw9718_devid = 0x9718;
	return 0;
}

static struct nvc_focus_cap dw9718_cap = {
	.settle_time = 30,
	.slew_rate = 0x3A200C,
	.focus_macro = 450,
	.focus_infinity = 200,
	.focus_hyper = 200,
};

static struct dw9718_platform_data ardbeg_dw9718_data = {
	.cfg = NVC_CFG_NODEV,
	.num = 0,
	.sync = 0,
	.dev_name = "focuser",
	.cap = &dw9718_cap,
	.power_on = ardbeg_dw9718_power_on,
	.power_off = ardbeg_dw9718_power_off,
	.detect = ardbeg_dw9718_detect,
};

static struct as364x_platform_data ardbeg_as3648_data = {
	.config		= {
		.led_mask	= 3,
		.max_total_current_mA = 1000,
		.max_peak_current_mA = 600,
		.max_torch_current_mA = 600,
		.vin_low_v_run_mV = 3070,
		.strobe_type = 1,
		},
	.pinstate	= {
		.mask	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0),
		.values	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PBB0)
		},
	.dev_name	= "torch",
	.type		= AS3648,
	.gpio_strobe	= CAM_FLASH_STROBE,
};

static int ardbeg_ov7695_power_on(struct ov7695_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd)))
		return -EFAULT;

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
	return -ENODEV;
}

static int ardbeg_ov7695_power_off(struct ov7695_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd)))
		return -EFAULT;
	usleep_range(100, 120);

	gpio_set_value(CAM2_PWDN, 0);
	usleep_range(100, 120);

	regulator_disable(pw->iovdd);
	usleep_range(100, 120);

	regulator_disable(pw->avdd);
	return 0;
}

struct ov7695_platform_data ardbeg_ov7695_pdata = {
	.power_on = ardbeg_ov7695_power_on,
	.power_off = ardbeg_ov7695_power_off,
	.mclk_name = "mclk2",
};

static int ardbeg_ov5693_power_on(struct ov5693_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd)))
		return -EFAULT;

	if (ardbeg_get_extra_regulators())
		goto ov5693_poweron_fail;

	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ov5693_avdd_fail;

	err = regulator_enable(pw->dovdd);
	if (err)
		goto ov5693_iovdd_fail;

	udelay(2);
	gpio_set_value(CAM1_PWDN, 1);

	err = regulator_enable(ardbeg_vcmvdd);
	if (unlikely(err))
		goto ov5693_vcmvdd_fail;

	usleep_range(300, 310);

	return 0;

ov5693_vcmvdd_fail:
	regulator_disable(pw->dovdd);

ov5693_iovdd_fail:
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	gpio_set_value(CAM1_PWDN, 0);

ov5693_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int ardbeg_ov5693_power_off(struct ov5693_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd)))
		return -EFAULT;

	usleep_range(21, 25);
	gpio_set_value(CAM1_PWDN, 0);
	udelay(2);

	regulator_disable(ardbeg_vcmvdd);
	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	return 0;
}

static struct nvc_gpio_pdata ov5693_gpio_pdata[] = {
	{ OV5693_GPIO_TYPE_PWRDN, CAM_RSTN, true, 0, },
};

static struct ov5693_platform_data ardbeg_ov5693_pdata = {
	.gpio_count	= ARRAY_SIZE(ov5693_gpio_pdata),
	.gpio		= ov5693_gpio_pdata,
	.power_on	= ardbeg_ov5693_power_on,
	.power_off	= ardbeg_ov5693_power_off,
};

static int ardbeg_ad5823_power_on(struct ad5823_platform_data *pdata)
{
	int err = 0;

	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 1);

	return err;
}

static int ardbeg_ad5823_power_off(struct ad5823_platform_data *pdata)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 0);

	return 0;
}

static struct ad5823_platform_data ardbeg_ad5823_pdata = {
	.gpio = CAM_AF_PWDN,
	.power_on	= ardbeg_ad5823_power_on,
	.power_off	= ardbeg_ad5823_power_off,
};

static struct i2c_board_info ardbeg_i2c_board_info_e1823[] = {
	{
		I2C_BOARD_INFO("imx135", 0x10),
		.platform_data = &ardbeg_imx135_data,
	},
	{
		I2C_BOARD_INFO("ar0261", 0x36),
		.platform_data = &ardbeg_ar0261_data,
	},
	{
		I2C_BOARD_INFO("dw9718", 0x0c),
		.platform_data = &ardbeg_dw9718_data,
	},
	{
		I2C_BOARD_INFO("as3648", 0x30),
		.platform_data = &ardbeg_as3648_data,
	},
};

static struct i2c_board_info ardbeg_i2c_board_info_e1793[] = {
	{
		I2C_BOARD_INFO("ov5693", 0x10),
		.platform_data = &ardbeg_ov5693_pdata,
	},
	{
		I2C_BOARD_INFO("ov7695", 0x21),
		.platform_data = &ardbeg_ov7695_pdata,
	},
	{
		I2C_BOARD_INFO("ad5823", 0x0c),
		.platform_data = &ardbeg_ad5823_pdata,
	},
	{
		I2C_BOARD_INFO("as3648", 0x30),
		.platform_data = &ardbeg_as3648_data,
	},
};


static int ardbeg_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	if (!of_machine_is_compatible("nvidia,tn8")) {
		i2c_register_board_info(2, ardbeg_i2c_board_info_e1823,
				ARRAY_SIZE(ardbeg_i2c_board_info_e1823));

		/* put CSIA/B/E IOs into DPD mode to
		 * save additional power for ardbeg
		 */
		tegra_io_dpd_enable(&csia_io);
		tegra_io_dpd_enable(&csib_io);
		tegra_io_dpd_enable(&csie_io);
	} else {
		i2c_register_board_info(2, ardbeg_i2c_board_info_e1793,
				ARRAY_SIZE(ardbeg_i2c_board_info_e1793));
	}
	return 0;
}

static struct pid_thermal_gov_params tj_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params tj_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &tj_pid_params,
};

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,    GPU,  C3BUS,   SCLK,    EMC   */
	{ { 2014500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1989000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1963500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1938000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1912500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1887000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1861500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1836000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
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
	{ { 1173000, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 720000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 720000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 720000, NO_CAP, NO_CAP, 792000 } },
	{ {  969000, 720000, NO_CAP, NO_CAP, 792000 } },
	{ {  943500, 720000, NO_CAP, NO_CAP, 792000 } },
	{ {  918000, 636000, NO_CAP, NO_CAP, 792000 } },
	{ {  892500, 636000, NO_CAP, NO_CAP, 792000 } },
	{ {  867000, 636000, NO_CAP, NO_CAP, 624000 } },
	{ {  841500, 636000, NO_CAP, NO_CAP, 624000 } },
	{ {  816000, 636000, NO_CAP, NO_CAP, 624000 } },
	{ {  790500, 540000, NO_CAP, 163200, 624000 } },
	{ {  765000, 540000, NO_CAP, 163200, 624000 } },
	{ {  739500, 540000, NO_CAP, 163200, 624000 } },
	{ {  714000, 540000, 216000, 163200, 528000 } },
	{ {  688500, 540000, 216000, 163200, 528000 } },
	{ {  663000, 540000, 216000, 163200, 528000 } },
	{ {  637500, 456000, 216000, 163200, 528000 } },
	{ {  612000, 456000, 216000, 163200, 528000 } },
	{ {  586500, 456000, 216000, 163200, 408000 } },
	{ {  561000, 456000, 216000, 163200, 408000 } },
	{ {  535500, 456000, 168000, 136000, 408000 } },
	{ {  510000, 456000, 168000, 136000, 408000 } },
	{ {  484500, 360000, 168000, 136000, 408000 } },
	{ {  459000, 360000, 168000, 136000, 348000 } },
	{ {  433500, 360000, 168000, 136000, 348000 } },
	{ {  408000, 360000, 168000, 102000, 348000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init ardbeg_tj_throttle_init(void)
{
	if (of_machine_is_compatible("nvidia,ardbeg") ||
	    of_machine_is_compatible("nvidia,tn8"))
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(ardbeg_tj_throttle_init);

static struct pid_thermal_gov_params skin_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params skin_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &skin_pid_params,
};

static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,    GPU,  C3BUS,   SCLK,    EMC   */
	{ { 2014500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1989000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1963500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1938000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1912500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1887000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1861500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1836000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
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
	{ { 1173000, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 816000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 720000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 720000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 720000, NO_CAP, NO_CAP, 792000 } },
	{ {  969000, 720000, NO_CAP, NO_CAP, 792000 } },
	{ {  943500, 720000, NO_CAP, NO_CAP, 792000 } },
	{ {  918000, 636000, NO_CAP, NO_CAP, 792000 } },
	{ {  892500, 636000, NO_CAP, NO_CAP, 792000 } },
	{ {  867000, 636000, NO_CAP, NO_CAP, 624000 } },
	{ {  841500, 636000, NO_CAP, NO_CAP, 624000 } },
	{ {  816000, 636000, NO_CAP, NO_CAP, 624000 } },
	{ {  790500, 540000, NO_CAP, 163200, 624000 } },
	{ {  765000, 540000, NO_CAP, 163200, 624000 } },
	{ {  739500, 540000, NO_CAP, 163200, 624000 } },
	{ {  714000, 540000, 216000, 163200, 528000 } },
	{ {  688500, 540000, 216000, 163200, 528000 } },
	{ {  663000, 540000, 216000, 163200, 528000 } },
	{ {  637500, 456000, 216000, 163200, 528000 } },
	{ {  612000, 456000, 216000, 163200, 528000 } },
	{ {  586500, 456000, 216000, 163200, 408000 } },
	{ {  561000, 456000, 216000, 163200, 408000 } },
	{ {  535500, 456000, 168000, 136000, 408000 } },
	{ {  510000, 456000, 168000, 136000, 408000 } },
	{ {  484500, 360000, 168000, 136000, 408000 } },
	{ {  459000, 360000, 168000, 136000, 348000 } },
	{ {  433500, 360000, 168000, 136000, 348000 } },
	{ {  408000, 360000, 168000, 102000, 348000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init ardbeg_skin_init(void)
{
	balanced_throttle_register(&skin_throttle, "skin-balanced");
	return 0;
}
late_initcall(ardbeg_skin_init);

static struct nct1008_platform_data ardbeg_nct72_pdata = {
	.loc_name = "tegra",

	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.shutdown_ext_limit = 95, /* C */
	.shutdown_local_limit = 120, /* C */

	.passive_delay = 1000,
	.tzp = &tj_tzp,

	.num_trips = 2,
	.trips = {
		{
			.cdev_type = "shutdown_warning",
			.trip_temp = 93000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
		},
		{
			.cdev_type = "tegra-balanced",
			.trip_temp = 83000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
			.hysteresis = 1000,
		},
	},
};

static struct nct1008_platform_data ardbeg_nct72_tskin_pdata = {
	.loc_name = "skin",

	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.shutdown_ext_limit = 85, /* C */
	.shutdown_local_limit = 120, /* C */

	.passive_delay = 5000,
	.polling_delay = 1000,
	.tzp = &skin_tzp,

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "skin-balanced",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_PASSIVE,
			.upper = THERMAL_NO_LIMIT,
			.lower = THERMAL_NO_LIMIT,
		},
	},
};

static struct i2c_board_info ardbeg_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4c),
		.platform_data = &ardbeg_nct72_pdata,
		.irq = -1,
	},
	{
		I2C_BOARD_INFO("nct72", 0x4d),
		.platform_data = &ardbeg_nct72_tskin_pdata,
		.irq = -1,
	}
};

static int ardbeg_nct72_init(void)
{
	int nct72_port = TEGRA_GPIO_PI6;
	int ret = 0;
	int i;
	struct thermal_trip_info *trip_state;

	/* raise NCT's thresholds if soctherm CP,FT fuses are ok */
	if (!tegra_fuse_calib_base_get_cp(NULL, NULL) &&
	    !tegra_fuse_calib_base_get_ft(NULL, NULL)) {
		ardbeg_nct72_pdata.shutdown_ext_limit += 20;
		for (i = 0; i < ardbeg_nct72_pdata.num_trips; i++) {
			trip_state = &ardbeg_nct72_pdata.trips[i];
			if (!strncmp(trip_state->cdev_type, "tegra-balanced",
					THERMAL_NAME_LENGTH)) {
				trip_state->cdev_type = "_none_";
				break;
			}
		}
	} else {
		tegra_platform_edp_init(ardbeg_nct72_pdata.trips,
					&ardbeg_nct72_pdata.num_trips,
					12000); /* edp temperature margin */
	}

	tegra_add_cdev_trips(ardbeg_nct72_pdata.trips,
				&ardbeg_nct72_pdata.num_trips);

	ardbeg_i2c_nct72_board_info[0].irq = gpio_to_irq(nct72_port);

	ret = gpio_request(nct72_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct72_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct72_port)", __func__);
		gpio_free(nct72_port);
	}

	/* ardbeg has thermal sensor on GEN2-I2C i.e. instance 1 */
	i2c_register_board_info(1, ardbeg_i2c_nct72_board_info,
		ARRAY_SIZE(ardbeg_i2c_nct72_board_info));

	return ret;
}

static struct sbs_platform_data sbs_pdata = {
	.poll_retry_count	= 100,
	.i2c_retry_count	= 2,
};

static struct i2c_board_info __initdata bq20z45_pdata[] = {
	{
		I2C_BOARD_INFO("sbs-battery", 0x0B),
		.platform_data = &sbs_pdata,
	},
};

int __init ardbeg_sensors_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	/* PM363 don't have mpu 9250 mounted */
	/* TN8 sensors use Device Tree */
	if (board_info.board_id != BOARD_PM363 &&
		!of_machine_is_compatible("nvidia,tn8"))
		mpuirq_init();
	ardbeg_camera_init();
	ardbeg_nct72_init();

	/* TN8 don't have ALS CM32181 */
	if (!of_machine_is_compatible("nvidia,tn8"))
		i2c_register_board_info(0, ardbeg_i2c_board_info_cm32181,
			ARRAY_SIZE(ardbeg_i2c_board_info_cm32181));

	if (get_power_supply_type() == POWER_SUPPLY_TYPE_BATTERY)
		i2c_register_board_info(1, bq20z45_pdata,
			ARRAY_SIZE(bq20z45_pdata));

	return 0;
}
