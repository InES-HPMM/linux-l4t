/*
 * arch/arm/mach-tegra/board-t210ref-sensors.c
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
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/nct1008.h>
#include <linux/pid_thermal_gov.h>
#include <linux/tegra-fuse.h>
#include <linux/of_platform.h>
#include <mach/edp.h>
#include <mach/io_dpd.h>
#include <media/camera.h>
#include <media/ar0261.h>
#include <media/imx135.h>
#include <media/imx179.h>
#include <media/dw9718.h>
#include <media/as364x.h>
#include <media/ov5693.h>
#include <media/ov7695.h>
#include <media/mt9m114.h>
#include <media/ad5823.h>
#include <media/max77387.h>

#include <linux/platform_device.h>
#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>
#include <media/tegra_v4l2_camera.h>

#include "cpu-tegra.h"
#include "devices.h"
#include "board.h"
#include "board-common.h"
#include "board-t210ref.h"
#include "tegra-board-id.h"

#if defined(ARCH_TEGRA_12x_SOC)
static struct i2c_board_info t210ref_i2c_board_info_cm32181[] = {
	{
		I2C_BOARD_INFO("cm32181", 0x48),
	},
};
#endif

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

static struct mpu_platform_data mpu9250_gyro_data_e1762 = {
	.int_config     = 0x10,
	.level_shifter  = 0,
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION_E1762,
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
	struct board_info board_info;

	pr_info("*** MPU START *** mpuirq_init...\n");

	tegra_get_board_info(&board_info);

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

	/* TN8 with diferent Compass address from t210ref */
	if (of_machine_is_compatible("nvidia,tn8"))
		inv_mpu9250_i2c0_board_info[2].addr = MPU_COMPASS_ADDR_TN8;

	if (board_info.board_id == BOARD_E1762)
		inv_mpu9250_i2c0_board_info[0].platform_data =
					&mpu9250_gyro_data_e1762;
	inv_mpu9250_i2c0_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu9250_i2c0_board_info,
		ARRAY_SIZE(inv_mpu9250_i2c0_board_info));
}

/*
 * Soc Camera platform driver for testing
 */
#if IS_ENABLED(CONFIG_SOC_CAMERA_PLATFORM)
static int t210ref_soc_camera_add(struct soc_camera_device *icd);
static void t210ref_soc_camera_del(struct soc_camera_device *icd);

static int t210ref_soc_camera_set_capture(struct soc_camera_platform_info *info,
		int enable)
{
	/* TODO: probably add clk opertaion here */
	return 0; /* camera sensor always enabled */
}

static struct soc_camera_platform_info t210ref_soc_camera_info = {
	.format_name = "RGB4",
	.format_depth = 32,
	.format = {
		.code = V4L2_MBUS_FMT_RGBA8888_4X8_LE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.field = V4L2_FIELD_NONE,
		.width = 1280,
		.height = 720,
	},
	.set_capture = t210ref_soc_camera_set_capture,
};

static struct tegra_camera_platform_data t210ref_camera_platform_data = {
	.flip_v                 = 0,
	.flip_h                 = 0,
	.port                   = TEGRA_CAMERA_PORT_CSI_A,
	.lanes                  = 4,
	.continuous_clk         = 0,
};

static struct soc_camera_link t210ref_soc_camera_link = {
	.bus_id         = 0, /* This must match the .id of tegra_vi01_device */
	.add_device     = t210ref_soc_camera_add,
	.del_device     = t210ref_soc_camera_del,
	.module_name    = "soc_camera_platform",
	.priv		= &t210ref_camera_platform_data,
	.dev_priv	= &t210ref_soc_camera_info,
};

static struct platform_device *t210ref_pdev;

static void t210ref_soc_camera_release(struct device *dev)
{
	soc_camera_platform_release(&t210ref_pdev);
}

static int t210ref_soc_camera_add(struct soc_camera_device *icd)
{
	return soc_camera_platform_add(icd, &t210ref_pdev,
			&t210ref_soc_camera_link,
			t210ref_soc_camera_release, 0);
}

static void t210ref_soc_camera_del(struct soc_camera_device *icd)
{
	soc_camera_platform_del(icd, t210ref_pdev, &t210ref_soc_camera_link);
}

static struct platform_device t210ref_soc_camera_device = {
	.name   = "soc-camera-pdrv",
	.id     = 0,
	.dev    = {
		.platform_data = &t210ref_soc_camera_link,
	},
};
#endif

static struct regulator *t210ref_vcmvdd;

static int t210ref_get_extra_regulators(void)
{
	if (!t210ref_vcmvdd) {
		t210ref_vcmvdd = regulator_get(NULL, "avdd_af1_cam");
		if (WARN_ON(IS_ERR(t210ref_vcmvdd))) {
			pr_err("%s: can't get regulator avdd_af1_cam: %ld\n",
					__func__, PTR_ERR(t210ref_vcmvdd));
			regulator_put(t210ref_vcmvdd);
			t210ref_vcmvdd = NULL;
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

static int t210ref_ar0261_power_on(struct ar0261_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	/* disable CSIE IOs DPD mode to turn on front camera for t210ref */
	tegra_io_dpd_disable(&csie_io);

	if (t210ref_get_extra_regulators())
		goto t210ref_ar0261_poweron_fail;

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 1);


	err = regulator_enable(t210ref_vcmvdd);
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
	regulator_disable(t210ref_vcmvdd);

ar0261_vcm_fail:
	pr_err("%s vcmvdd failed.\n", __func__);
	return -ENODEV;

t210ref_ar0261_poweron_fail:
	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int t210ref_ar0261_power_off(struct ar0261_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd ||
					!t210ref_vcmvdd))) {
		/* put CSIE IOs into DPD mode to
		 * save additional power for t210ref
		 */
		tegra_io_dpd_enable(&csie_io);
		return -EFAULT;
	}

	gpio_set_value(CAM_RSTN, 0);

	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);
	regulator_disable(t210ref_vcmvdd);
	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	return 0;
}

struct ar0261_platform_data t210ref_ar0261_data = {
	.power_on = t210ref_ar0261_power_on,
	.power_off = t210ref_ar0261_power_off,
	.mclk_name = "mclk2",
};

static int t210ref_imx135_get_extra_regulators(struct imx135_power_rail *pw)
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

static int t210ref_imx135_power_on(struct imx135_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for t210ref */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	if (t210ref_imx135_get_extra_regulators(pw))
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

static int t210ref_imx135_power_off(struct imx135_power_rail *pw)
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

	/* put CSIA/B IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	return 0;
}

static int t210ref_imx179_power_on(struct imx179_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for t210ref */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	gpio_set_value(CAM_AF_PWDN, 1);
	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx179_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx179_iovdd_fail;

	err = regulator_enable(pw->dvdd);
	if (err)
		goto imx179_dvdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM_RSTN, 1);

	usleep_range(300, 310);

	return 1;


imx179_dvdd_fail:
	regulator_disable(pw->iovdd);

imx179_iovdd_fail:
	regulator_disable(pw->avdd);

imx179_avdd_fail:
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int t210ref_imx179_power_off(struct imx179_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd))) {
		tegra_io_dpd_enable(&csia_io);
		tegra_io_dpd_enable(&csib_io);
		return -EFAULT;
	}

	regulator_disable(pw->dvdd);
	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	/* put CSIA/B IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	return 0;
}

struct imx135_platform_data t210ref_imx135_data = {
	.flash_cap = {
		.enable = 1,
		.edge_trig_en = 1,
		.start_edge = 0,
		.repeat = 1,
		.delay_frm = 0,
	},
	.power_on = t210ref_imx135_power_on,
	.power_off = t210ref_imx135_power_off,
};

struct imx179_platform_data t210ref_imx179_data = {
	.flash_cap = {
		.enable = 1,
		.edge_trig_en = 1,
		.start_edge = 0,
		.repeat = 1,
		.delay_frm = 0,
	},
	.power_on = t210ref_imx179_power_on,
	.power_off = t210ref_imx179_power_off,
};

static int t210ref_dw9718_power_on(struct dw9718_power_rail *pw)
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

static int t210ref_dw9718_power_off(struct dw9718_power_rail *pw)
{
	pr_info("%s\n", __func__);

	if (unlikely(!pw || !pw->vdd || !pw->vdd_i2c))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 1;
}

static u16 dw9718_devid;
static int t210ref_dw9718_detect(void *buf, size_t size)
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

static struct dw9718_platform_data t210ref_dw9718_data = {
	.cfg = NVC_CFG_NODEV,
	.num = 0,
	.sync = 0,
	.dev_name = "focuser",
	.cap = &dw9718_cap,
	.power_on = t210ref_dw9718_power_on,
	.power_off = t210ref_dw9718_power_off,
	.detect = t210ref_dw9718_detect,
};

static struct as364x_platform_data t210ref_as3648_data = {
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

static int t210ref_ov7695_power_on(struct ov7695_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd)))
		return -EFAULT;

	/* disable CSIE IOs DPD mode to turn on front camera for t210ref */
	tegra_io_dpd_disable(&csie_io);

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
	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	return -ENODEV;
}

static int t210ref_ov7695_power_off(struct ov7695_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd))) {
		/* put CSIE IOs into DPD mode to
		 * save additional power for t210ref
		 */
		tegra_io_dpd_enable(&csie_io);
		return -EFAULT;
	}
	usleep_range(100, 120);

	gpio_set_value(CAM2_PWDN, 0);
	usleep_range(100, 120);

	regulator_disable(pw->iovdd);
	usleep_range(100, 120);

	regulator_disable(pw->avdd);

	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	return 0;
}

struct ov7695_platform_data t210ref_ov7695_pdata = {
	.power_on = t210ref_ov7695_power_on,
	.power_off = t210ref_ov7695_power_off,
	.mclk_name = "mclk2",
};

static int t210ref_mt9m114_power_on(struct mt9m114_power_rail *pw)
{
	int err;
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	/* disable CSIE IOs DPD mode to turn on front camera for t210ref */
	tegra_io_dpd_disable(&csie_io);

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
	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	return -ENODEV;
}

static int t210ref_mt9m114_power_off(struct mt9m114_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd)) {
		/* put CSIE IOs into DPD mode to
		 * save additional power for t210ref
		 */
		tegra_io_dpd_enable(&csie_io);
		return -EFAULT;
	}

	usleep_range(100, 120);
	gpio_set_value(CAM_RSTN, 0);
	usleep_range(100, 120);
	regulator_disable(pw->avdd);
	usleep_range(100, 120);
	regulator_disable(pw->iovdd);

	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	return 1;
}

struct mt9m114_platform_data t210ref_mt9m114_pdata = {
	.power_on = t210ref_mt9m114_power_on,
	.power_off = t210ref_mt9m114_power_off,
	.mclk_name = "mclk2",
};


static int t210ref_ov5693_power_on(struct ov5693_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for t210ref */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	if (t210ref_get_extra_regulators())
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

	err = regulator_enable(t210ref_vcmvdd);
	if (unlikely(err))
		goto ov5693_vcmvdd_fail;

	usleep_range(1000, 1110);

	return 0;

ov5693_vcmvdd_fail:
	regulator_disable(pw->dovdd);

ov5693_iovdd_fail:
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	gpio_set_value(CAM1_PWDN, 0);

ov5693_poweron_fail:
	/* put CSIA/B IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int t210ref_ov5693_power_off(struct ov5693_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd))) {
		/* put CSIA/B IOs into DPD mode to
		 * save additional power for t210ref
		 */
		tegra_io_dpd_enable(&csia_io);
		tegra_io_dpd_enable(&csib_io);
		return -EFAULT;
	}

	usleep_range(21, 25);
	gpio_set_value(CAM1_PWDN, 0);
	udelay(2);

	regulator_disable(t210ref_vcmvdd);
	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	/* put CSIA/B IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	return 0;
}

static struct nvc_gpio_pdata ov5693_gpio_pdata[] = {
	{ OV5693_GPIO_TYPE_PWRDN, CAM1_PWDN, true, 0, },
};

#define NV_GUID(a, b, c, d, e, f, g, h) \
	((u64) ((((a)&0xffULL) << 56ULL) | (((b)&0xffULL) << 48ULL) | \
	(((c)&0xffULL) << 40ULL) | (((d)&0xffULL) << 32ULL) | \
	(((e)&0xffULL) << 24ULL) | (((f)&0xffULL) << 16ULL) | \
	(((g)&0xffULL) << 8ULL) | (((h)&0xffULL))))

static struct nvc_imager_cap ov5693_cap = {
	.identifier				= "OV5693",
	.sensor_nvc_interface	= 3,
	.pixel_types[0]			= 0x101,
	.orientation			= 0,
	.direction				= 0,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 8000000, /* value * 1000000 */
	},
	.clock_profiles[1] = {
		.external_clock_khz	= 0,
		.clock_multiplier	= 0,
	},
	.h_sync_edge			= 0,
	.v_sync_edge			= 0,
	.mclk_on_vgp0			= 0,
	.csi_port				= 0,
	.data_lanes				= 2,
	.virtual_channel_id		= 0,
	.discontinuous_clk_mode	= 1,
	.cil_threshold_settle	= 0,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid			=
		NV_GUID('f', '_', 'A', 'D', '5', '8', '2', '3'),
	.torch_guid				=
		NV_GUID('l', '_', 'N', 'V', 'C', 'A', 'M', '0'),
	.cap_version			= NVC_IMAGER_CAPABILITIES_VERSION2,
	.flash_control_enabled	= 0,
	.adjustable_flash_timing	= 0,
	.is_hdr					= 1,
};


static struct ov5693_platform_data t210ref_ov5693_pdata = {
	.gpio_count	= ARRAY_SIZE(ov5693_gpio_pdata),
	.gpio		= ov5693_gpio_pdata,
	.power_on	= t210ref_ov5693_power_on,
	.power_off	= t210ref_ov5693_power_off,
	.dev_name	= "ov5693",
	.cap		= &ov5693_cap,
	.mclk_name	= "mclk",
	.regulators = {
			.avdd = "avdd_ov5693",
			.dvdd = "dvdd",
			.dovdd = "dovdd",
	},
	.has_eeprom = 1,
};

static int t210ref_ov5693_front_power_on(struct ov5693_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd)))
		return -EFAULT;

	if (t210ref_get_extra_regulators())
		goto ov5693_front_poweron_fail;

	gpio_set_value(CAM2_PWDN, 0);
	gpio_set_value(CAM_RSTN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ov5693_front_avdd_fail;

	err = regulator_enable(pw->dovdd);
	if (err)
		goto ov5693_front_iovdd_fail;

	udelay(2);
	gpio_set_value(CAM2_PWDN, 1);
	gpio_set_value(CAM_RSTN, 1);

	err = regulator_enable(t210ref_vcmvdd);
	if (unlikely(err))
		goto ov5693_front_vcmvdd_fail;

	usleep_range(1000, 1110);

	return 0;

ov5693_front_vcmvdd_fail:
	regulator_disable(pw->dovdd);

ov5693_front_iovdd_fail:
	regulator_disable(pw->avdd);

ov5693_front_avdd_fail:
	gpio_set_value(CAM2_PWDN, 0);
	gpio_set_value(CAM_RSTN, 0);

ov5693_front_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int t210ref_ov5693_front_power_off(struct ov5693_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->dovdd || !pw->avdd))) {
		return -EFAULT;
	}

	usleep_range(21, 25);
	gpio_set_value(CAM2_PWDN, 0);
	gpio_set_value(CAM_RSTN, 0);
	udelay(2);

	regulator_disable(t210ref_vcmvdd);
	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	return 0;
}

static struct nvc_gpio_pdata ov5693_front_gpio_pdata[] = {
	{ OV5693_GPIO_TYPE_PWRDN, CAM2_PWDN, true, 0, },
	{ OV5693_GPIO_TYPE_PWRDN, CAM_RSTN, true, 0, },
};

static struct nvc_imager_cap ov5693_front_cap = {
	.identifier				= "OV5693.1",
	.sensor_nvc_interface	= 4,
	.pixel_types[0]			= 0x101,
	.orientation			= 0,
	.direction				= 1,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 8000000, /* value * 1000000 */
	},
	.clock_profiles[1] = {
		.external_clock_khz	= 0,
		.clock_multiplier	= 0,
	},
	.h_sync_edge			= 0,
	.v_sync_edge			= 0,
	.mclk_on_vgp0			= 0,
	.csi_port				= 1,
	.data_lanes				= 2,
	.virtual_channel_id		= 0,
	.discontinuous_clk_mode	= 1,
	.cil_threshold_settle	= 0,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid			= 0,
	.torch_guid				= 0,
	.cap_version			= NVC_IMAGER_CAPABILITIES_VERSION2,
	.flash_control_enabled	= 0,
	.adjustable_flash_timing	= 0,
	.is_hdr					= 1,
};

static struct ov5693_platform_data t210ref_ov5693_front_pdata = {
	.gpio_count	= ARRAY_SIZE(ov5693_front_gpio_pdata),
	.gpio		= ov5693_front_gpio_pdata,
	.power_on	= t210ref_ov5693_front_power_on,
	.power_off	= t210ref_ov5693_front_power_off,
	.dev_name	= "ov5693.1",
	.mclk_name	= "mclk2",
	.cap		= &ov5693_front_cap,
	.regulators = {
			.avdd = "vana",
			.dvdd = "vdig",
			.dovdd = "vif",
	},
	.has_eeprom = 0,
};

static int t210ref_ad5823_power_on(struct ad5823_platform_data *pdata)
{
	int err = 0;

	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 1);
	pdata->pwr_dev = AD5823_PWR_DEV_ON;

	return err;
}

static int t210ref_ad5823_power_off(struct ad5823_platform_data *pdata)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 0);
	pdata->pwr_dev = AD5823_PWR_DEV_OFF;

	return 0;
}

static struct ad5823_platform_data t210ref_ad5823_pdata = {
	.gpio = CAM_AF_PWDN,
	.power_on	= t210ref_ad5823_power_on,
	.power_off	= t210ref_ad5823_power_off,
};

static struct camera_data_blob t210ref_camera_lut[] = {
	{"t210ref_imx135_pdata", &t210ref_imx135_data},
	{"t210ref_dw9718_pdata", &t210ref_dw9718_data},
	{"t210ref_ar0261_pdata", &t210ref_ar0261_data},
	{"t210ref_mt9m114_pdata", &t210ref_mt9m114_pdata},
	{"t210ref_ov5693_pdata", &t210ref_ov5693_pdata},
	{"t210ref_ad5823_pdata", &t210ref_ad5823_pdata},
	{"t210ref_as3648_pdata", &t210ref_as3648_data},
	{"t210ref_ov7695_pdata", &t210ref_ov7695_pdata},
	{"t210ref_imx179_pdata", &t210ref_imx179_data},
	{"t210ref_ov5693f_pdata", &t210ref_ov5693_front_pdata},
	{},
};

void __init t210ref_camera_auxdata(void *data)
{
	struct of_dev_auxdata *aux_lut = data;
	while (aux_lut && aux_lut->compatible) {
		if (!strcmp(aux_lut->compatible, "nvidia,tegra124-camera")) {
			pr_info("%s: update camera lookup table.\n", __func__);
			aux_lut->platform_data = t210ref_camera_lut;
		}
		aux_lut++;
	}
}

static int t210ref_camera_init(void)
{
	struct board_info board_info;

	pr_debug("%s: ++\n", __func__);
	tegra_get_board_info(&board_info);

	/* put CSIA/B/C/D/E IOs into DPD mode to
	 * save additional power for t210ref
	 */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	tegra_io_dpd_enable(&csie_io);

#if IS_ENABLED(CONFIG_SOC_CAMERA_PLATFORM)
	platform_device_register(&t210ref_soc_camera_device);
#endif

	return 0;
}

static struct pid_thermal_gov_params cpu_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params cpu_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &cpu_pid_params,
};

static struct thermal_zone_params board_tzp = {
	.governor_name = "pid_thermal_gov"
};

static struct throttle_table cpu_throttle_table[] = {
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

static struct balanced_throttle cpu_throttle = {
	.throt_tab_size = ARRAY_SIZE(cpu_throttle_table),
	.throt_tab = cpu_throttle_table,
};

static struct throttle_table gpu_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,    GPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 2295000, 782800, 480000, 756000, 384000, 924000 } },
	{ { 2269500, 772200, 480000, 756000, 384000, 924000 } },
	{ { 2244000, 761600, 480000, 756000, 384000, 924000 } },
	{ { 2218500, 751100, 480000, 756000, 384000, 924000 } },
	{ { 2193000, 740500, 480000, 756000, 384000, 924000 } },
	{ { 2167500, 729900, 480000, 756000, 384000, 924000 } },
	{ { 2142000, 719300, 480000, 756000, 384000, 924000 } },
	{ { 2116500, 708700, 480000, 756000, 384000, 924000 } },
	{ { 2091000, 698100, 480000, 756000, 384000, 924000 } },
	{ { 2065500, 687500, 480000, 756000, 384000, 924000 } },
	{ { 2040000, 676900, 480000, 756000, 384000, 924000 } },
	{ { 2014500, 666000, 480000, 756000, 384000, 924000 } },
	{ { 1989000, 656000, 480000, 756000, 384000, 924000 } },
	{ { 1963500, 645000, 480000, 756000, 384000, 924000 } },
	{ { 1938000, 635000, 480000, 756000, 384000, 924000 } },
	{ { 1912500, 624000, 480000, 756000, 384000, 924000 } },
	{ { 1887000, 613000, 480000, 756000, 384000, 924000 } },
	{ { 1861500, 603000, 480000, 756000, 384000, 924000 } },
	{ { 1836000, 592000, 480000, 756000, 384000, 924000 } },
	{ { 1810500, 582000, 480000, 756000, 384000, 924000 } },
	{ { 1785000, 571000, 480000, 756000, 384000, 924000 } },
	{ { 1759500, 560000, 480000, 756000, 384000, 924000 } },
	{ { 1734000, 550000, 480000, 756000, 384000, 924000 } },
	{ { 1708500, 539000, 480000, 756000, 384000, 924000 } },
	{ { 1683000, 529000, 480000, 756000, 384000, 924000 } },
	{ { 1657500, 518000, 480000, 756000, 384000, 924000 } },
	{ { 1632000, 508000, 480000, 756000, 384000, 924000 } },
	{ { 1606500, 497000, 480000, 756000, 384000, 924000 } },
	{ { 1581000, 486000, 480000, 756000, 384000, 924000 } },
	{ { 1555500, 476000, 480000, 756000, 384000, 924000 } },
	{ { 1530000, 465000, 480000, 756000, 384000, 924000 } },
	{ { 1504500, 455000, 480000, 756000, 384000, 924000 } },
	{ { 1479000, 444000, 480000, 756000, 384000, 924000 } },
	{ { 1453500, 433000, 480000, 756000, 384000, 924000 } },
	{ { 1428000, 423000, 480000, 756000, 384000, 924000 } },
	{ { 1402500, 412000, 480000, 756000, 384000, 924000 } },
	{ { 1377000, 402000, 480000, 756000, 384000, 924000 } },
	{ { 1351500, 391000, 480000, 756000, 384000, 924000 } },
	{ { 1326000, 380000, 480000, 756000, 384000, 924000 } },
	{ { 1300500, 370000, 480000, 756000, 384000, 924000 } },
	{ { 1275000, 359000, 480000, 756000, 384000, 924000 } },
	{ { 1249500, 349000, 480000, 756000, 384000, 924000 } },
	{ { 1224000, 338000, 480000, 756000, 384000, 792000 } },
	{ { 1198500, 328000, 480000, 756000, 384000, 792000 } },
	{ { 1173000, 317000, 480000, 756000, 360000, 792000 } },
	{ { 1147500, 306000, 480000, 756000, 360000, 792000 } },
	{ { 1122000, 296000, 480000, 684000, 360000, 792000 } },
	{ { 1096500, 285000, 444000, 684000, 360000, 792000 } },
	{ { 1071000, 275000, 444000, 684000, 360000, 792000 } },
	{ { 1045500, 264000, 444000, 684000, 360000, 792000 } },
	{ { 1020000, 253000, 444000, 684000, 324000, 792000 } },
	{ {  994500, 243000, 444000, 684000, 324000, 792000 } },
	{ {  969000, 232000, 444000, 600000, 324000, 792000 } },
	{ {  943500, 222000, 444000, 600000, 324000, 792000 } },
	{ {  918000, 211000, 396000, 600000, 324000, 792000 } },
	{ {  892500, 200000, 396000, 600000, 324000, 792000 } },
	{ {  867000, 190000, 396000, 600000, 324000, 792000 } },
	{ {  841500, 179000, 396000, 600000, 288000, 792000 } },
	{ {  816000, 169000, 396000, 600000, 288000, 792000 } },
	{ {  790500, 158000, 396000, 600000, 288000, 792000 } },
	{ {  765000, 148000, 396000, 504000, 288000, 792000 } },
	{ {  739500, 137000, 348000, 504000, 288000, 792000 } },
	{ {  714000, 126000, 348000, 504000, 288000, 624000 } },
	{ {  688500, 116000, 348000, 504000, 288000, 624000 } },
	{ {  663000, 105000, 348000, 504000, 288000, 624000 } },
	{ {  637500,  95000, 348000, 504000, 288000, 624000 } },
	{ {  612000,  84000, 348000, 504000, 252000, 624000 } },
	{ {  586500,  84000, 348000, 504000, 252000, 624000 } },
	{ {  561000,  84000, 348000, 420000, 252000, 624000 } },
	{ {  535500,  84000, 288000, 420000, 252000, 624000 } },
	{ {  510000,  84000, 288000, 420000, 252000, 624000 } },
	{ {  484500,  84000, 288000, 420000, 252000, 624000 } },
	{ {  459000,  84000, 288000, 420000, 252000, 624000 } },
	{ {  433500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  408000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  382500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  357000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  331500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  306000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  280500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  255000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  229500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  204000,  84000, 288000, 420000, 252000, 396000 } },
};

static struct balanced_throttle gpu_throttle = {
	.throt_tab_size = ARRAY_SIZE(gpu_throttle_table),
	.throt_tab = gpu_throttle_table,
};

/* throttle table that sets all clocks to approximately 50% of their max */
static struct throttle_table emergency_throttle_table[] = {
	/*      CPU,    GPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1122000, 391000, 288000, 420000, 252000, 396000 } },
};

static struct balanced_throttle emergency_throttle = {
	.throt_tab_size = ARRAY_SIZE(emergency_throttle_table),
	.throt_tab = emergency_throttle_table,
};

static int __init t210ref_balanced_throttle_init(void)
{
	if (!of_machine_is_compatible("nvidia,e2141"))
		return 0;

	if (!balanced_throttle_register(&cpu_throttle, "cpu-balanced"))
		pr_err("balanced_throttle_register 'cpu-balanced' FAILED.\n");
	if (!balanced_throttle_register(&gpu_throttle, "gpu-balanced"))
		pr_err("balanced_throttle_register 'gpu-balanced' FAILED.\n");
	if (!balanced_throttle_register(&emergency_throttle,
							"emergency-balanced"))
		pr_err("balanced_throttle_register 'emergency-balanced' FAILED\n");
	return 0;
}
late_initcall(t210ref_balanced_throttle_init);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 43000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	}
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode_tegra",
		.coeffs = {
			2, 1, 1, 1,
			1, 1, 1, 1,
			1, 1, 1, 0,
			1, 1, 0, 0,
			0, 0, -1, -7
		},
	},
	{
		.dev_data = "Tboard_tegra",
		.coeffs = {
			-11, -7, -5, -3,
			-3, -2, -1, 0,
			0, 0, 1, 1,
			1, 2, 2, 3,
			4, 6, 11, 18
		},
	},
};

static struct therm_est_subdevice tn8ffd_skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			3, 0, 0, 0,
			1, 0, -1, 0,
			1, 0, 0, 1,
			1, 0, 0, 0,
			0, 1, 2, 2
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			1, 1, 2, 8,
			6, -8, -13, -9,
			-9, -8, -17, -18,
			-18, -16, 2, 17,
			15, 27, 42, 60
		},
	},
};

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

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.polling_period = 1100,
	.passive_delay = 15000,
	.tc1 = 10,
	.tc2 = 1,
	.tzp = &skin_tzp,
	.use_activator = 1,
};

static struct throttle_table skin_throttle_table[] = {
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

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init t210ref_skin_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (!of_machine_is_compatible("nvidia,e2141"))
		return 0;

	if (board_info.board_id == BOARD_P1761 ||
			board_info.board_id == BOARD_E1784 ||
			board_info.board_id == BOARD_E1991 ||
			board_info.board_id == BOARD_E1971 ||
			board_info.board_id == BOARD_E1922) {
		skin_data.ndevs = ARRAY_SIZE(tn8ffd_skin_devs);
		skin_data.devs = tn8ffd_skin_devs;
		skin_data.toffset = 4034;
	} else {
		skin_data.ndevs = ARRAY_SIZE(skin_devs);
		skin_data.devs = skin_devs;
		skin_data.toffset = 9793;
	}

	tegra_skin_therm_est_device.dev.platform_data = &skin_data;
	platform_device_register(&tegra_skin_therm_est_device);

	if (!balanced_throttle_register(&skin_throttle, "skin-balanced"))
		pr_err("balanced_throttle_register 'skin-balanced' FAILED.\n");

	return 0;
}
late_initcall(t210ref_skin_init);
#endif

static struct nct1008_platform_data t210ref_nct72_pdata = {
	.loc_name = "tegra",
	.supported_hwrev = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.extended_range = true,

	.sensors = {
		[LOC] = {
			.tzp = &board_tzp,
			.shutdown_limit = 120, /* C */
			.passive_delay = 1000,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "therm_est_activ",
					.trip_temp = 40000,
					.trip_type = THERMAL_TRIP_ACTIVE,
					.hysteresis = 1000,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 1,
				},
			},
		},
		[EXT] = {
			.tzp = &cpu_tzp,
			.shutdown_limit = 95, /* C */
			.passive_delay = 1000,
			.num_trips = 2,
			.trips = {
				{
					.cdev_type = "shutdown_warning",
					.trip_temp = 93000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 0,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 83000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.hysteresis = 1000,
					.mask = 1,
				},
			}
		}
	}
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct nct1008_platform_data t210ref_nct72_tskin_pdata = {
	.loc_name = "skin",

	.supported_hwrev = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.extended_range = true,

	.sensors = {
		[LOC] = {
			.shutdown_limit = 95, /* C */
			.num_trips = 0,
			.tzp = NULL,
		},
		[EXT] = {
			.shutdown_limit = 85, /* C */
			.passive_delay = 10000,
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
					.mask = 1,
				},
			},
		}
	}
};
#endif

static struct i2c_board_info t210ref_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4c),
		.platform_data = &t210ref_nct72_pdata,
		.irq = -1,
	},
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		I2C_BOARD_INFO("nct72", 0x4d),
		.platform_data = &t210ref_nct72_tskin_pdata,
		.irq = -1,
	}
#endif
};

static int t210ref_nct72_init(void)
{
	int nct72_port = TEGRA_GPIO_PI6;
	int ret = 0;
	int i;
	struct thermal_trip_info *trip_state;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* raise NCT's thresholds if soctherm CP,FT fuses are ok */
	if ((tegra_fuse_calib_base_get_cp(NULL, NULL) >= 0) &&
	    (tegra_fuse_calib_base_get_ft(NULL, NULL) >= 0)) {
		t210ref_nct72_pdata.sensors[EXT].shutdown_limit += 20;
		for (i = 0; i < t210ref_nct72_pdata.sensors[EXT].num_trips;
			 i++) {
			trip_state = &t210ref_nct72_pdata.sensors[EXT].trips[i];
			if (!strncmp(trip_state->cdev_type, "cpu-balanced",
					THERMAL_NAME_LENGTH)) {
				trip_state->cdev_type = "_none_";
				break;
			}
		}
	} else {
		tegra_platform_edp_init(
			t210ref_nct72_pdata.sensors[EXT].trips,
			&t210ref_nct72_pdata.sensors[EXT].num_trips,
					12000); /* edp temperature margin */
		tegra_add_cpu_vmax_trips(
			t210ref_nct72_pdata.sensors[EXT].trips,
			&t210ref_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_tgpu_trips(
			t210ref_nct72_pdata.sensors[EXT].trips,
			&t210ref_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_vc_trips(
			t210ref_nct72_pdata.sensors[EXT].trips,
			&t210ref_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_core_vmax_trips(
			t210ref_nct72_pdata.sensors[EXT].trips,
			&t210ref_nct72_pdata.sensors[EXT].num_trips);
	}

	/* vmin trips are bound to soctherm on norrin and bowmore */
	if (!(board_info.board_id == BOARD_PM374 ||
		board_info.board_id == BOARD_E2141 ||
		board_info.board_id == BOARD_E1971 ||
		board_info.board_id == BOARD_E1991))
		tegra_add_all_vmin_trips(t210ref_nct72_pdata.sensors[EXT].trips,
			&t210ref_nct72_pdata.sensors[EXT].num_trips);

	/* T210_interposer use GPIO_PC7 for alert*/
	if (board_info.board_id == BOARD_E2141)
		nct72_port = TEGRA_GPIO_PC7;

	t210ref_i2c_nct72_board_info[0].irq = gpio_to_irq(nct72_port);

	ret = gpio_request(nct72_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct72_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct72_port)", __func__);
		gpio_free(nct72_port);
	}

	/* norrin has thermal sensor on GEN1-I2C i.e. instance 0 */
	if (board_info.board_id == BOARD_PM374)
		i2c_register_board_info(0, t210ref_i2c_nct72_board_info,
					1); /* only register device[0] */
	/* t210ref has thermal sensor on GEN2-I2C i.e. instance 1 */
	else if (board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM370 ||
			board_info.board_id == BOARD_PM363)
		i2c_register_board_info(1, t210ref_i2c_nct72_board_info,
		ARRAY_SIZE(t210ref_i2c_nct72_board_info));
	else if (board_info.board_id == BOARD_PM375) {
		t210ref_nct72_pdata.sensors[EXT].shutdown_limit = 100;
		t210ref_nct72_pdata.sensors[LOC].shutdown_limit = 95;
		i2c_register_board_info(0, t210ref_i2c_nct72_board_info,
					1); /* only register device[0] */
	}
	else
		i2c_register_board_info(1, t210ref_i2c_nct72_board_info,
			ARRAY_SIZE(t210ref_i2c_nct72_board_info));

	return ret;
}

int __init t210ref_sensors_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	/* PM363 and PM359 don't have mpu 9250 mounted */
	/* TN8 sensors use Device Tree */
	if (board_info.board_id != BOARD_PM363 &&
		board_info.board_id != BOARD_PM359 &&
		!of_machine_is_compatible("nvidia,tn8") &&
		!of_machine_is_compatible("nvidia,bowmore") &&
		!of_machine_is_compatible("nvidia,e2141") &&
		board_info.board_id != BOARD_PM375)
		mpuirq_init();
	t210ref_camera_init();

	if (board_info.board_id == BOARD_P1761 ||
		board_info.board_id == BOARD_E1784 ||
		board_info.board_id == BOARD_E1971 ||
		board_info.board_id == BOARD_E1991 ||
		board_info.board_id == BOARD_E1922) {
		/* Sensor is on DT */
		pr_err("Temp sensor are from DT\n");
	} else
		t210ref_nct72_init();

#if defined(ARCH_TEGRA_12x_SOC)
	/* TN8 and PM359 don't have ALS CM32181 */
	if (!of_machine_is_compatible("nvidia,tn8") &&
	    board_info.board_id != BOARD_PM359 &&
	    board_info.board_id != BOARD_PM375)
		i2c_register_board_info(0, t210ref_i2c_board_info_cm32181,
			ARRAY_SIZE(t210ref_i2c_board_info_cm32181));
#endif
	return 0;
}
