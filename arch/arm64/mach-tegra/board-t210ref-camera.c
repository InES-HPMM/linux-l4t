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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/pid_thermal_gov.h>
#include <linux/tegra-fuse.h>
#include <linux/of_platform.h>
#include <mach/edp.h>
#include <mach/io_dpd.h>
#include <media/camera.h>
#include <media/ar0261.h>
#include <media/imx135.h>
#include <media/imx214.h>
#include <media/imx179.h>
#include <media/dw9718.h>
#include <media/dw9714.h>
#include <media/as364x.h>
#include <media/ov5693.h>
#include <media/ov7695.h>
#include <media/ad5823.h>

#include <linux/platform_device.h>
#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>
#include <media/tegra_v4l2_camera.h>

#include <linux/platform/tegra/cpu-tegra.h>
#include "devices.h"
#include "board.h"
#include "board-common.h"
#include "board-t210ref.h"
#include "tegra-board-id.h"

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

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd)))
		return -EFAULT;

	/* disable CSIE IOs DPD mode to turn on front camera for t210ref */
	tegra_io_dpd_disable(&csie_io);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 1);

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
	regulator_disable(pw->avdd);

ar0261_avdd_fail:
	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int t210ref_ar0261_power_off(struct ar0261_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd))) {
		/* put CSIE IOs into DPD mode to
		 * save additional power for t210ref
		 */
		tegra_io_dpd_enable(&csie_io);
		return -EFAULT;
	}

	gpio_set_value(CAM_RSTN, 0);

	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);
	/* put CSIE IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csie_io);
	return 0;
}

struct ar0261_platform_data t210ref_ar0261_data = {
	.power_on = t210ref_ar0261_power_on,
	.power_off = t210ref_ar0261_power_off,
	.mclk_name = "clk_out_3",
};

static int t210ref_imx135_power_on(struct imx135_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for t210ref */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	gpio_set_value(CAM_AF_PWDN, 1);
	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx135_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx135_iovdd_fail;

	usleep_range(2000, 2500);
	gpio_set_value(CAM1_PWDN, 1);

	usleep_range(300, 310);

	return 1;


imx135_iovdd_fail:
	regulator_disable(pw->avdd);

imx135_avdd_fail:
	gpio_set_value(CAM_AF_PWDN, 0);

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

static int t210ref_imx214_power_on(struct imx214_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	/* disable CSIA/B IOs DPD mode to turn on camera for t210ref */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 0);
	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx214_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx214_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM_RSTN, 1);
	gpio_set_value(CAM1_PWDN, 1);
	gpio_set_value(CAM_AF_PWDN, 1);

	usleep_range(300, 310);

	return 1;


imx214_iovdd_fail:
	regulator_disable(pw->avdd);

imx214_avdd_fail:
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int t210ref_imx214_power_off(struct imx214_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd))) {
		tegra_io_dpd_enable(&csia_io);
		tegra_io_dpd_enable(&csib_io);
		return -EFAULT;
	}

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM_AF_PWDN, 0);
	gpio_set_value(CAM1_PWDN, 0);

	/* put CSIA/B IOs into DPD mode to save additional power for t210ref */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	return 0;
}

struct imx214_platform_data t210ref_imx214_data = {
	.power_on = t210ref_imx214_power_on,
	.power_off = t210ref_imx214_power_off,
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

static int t210ref_dw9714_power_on(struct dw9714_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	err = regulator_enable(pw->vdd_i2c);
	if (unlikely(err))
		goto dw9714_vdd_i2c_fail;

	err = regulator_enable(pw->vdd);
	if (unlikely(err))
		goto dw9714_vdd_fail;

	gpio_set_value(CAM_AF_PWDN, 1);

	usleep_range(12000, 12500);

	return 0;

dw9714_vdd_fail:
	regulator_disable(pw->vdd_i2c);

dw9714_vdd_i2c_fail:
	pr_err("%s FAILED\n", __func__);

	return -ENODEV;
}

static int t210ref_dw9714_power_off(struct dw9714_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	gpio_set_value(CAM_AF_PWDN, 0);
	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 0;
}

static struct nvc_focus_cap dw9714_cap = {
	.settle_time = 30,
	.slew_rate = 0x3A200C,
	.focus_macro = 450,
	.focus_infinity = 200,
	.focus_hyper = 200,
};

static struct dw9714_platform_data t210ref_dw9714_data = {
	.cfg = NVC_CFG_NODEV,
	.num = 0,
	.sync = 0,
	.dev_name = "focuser",
	.cap = &dw9714_cap,
	.power_on = t210ref_dw9714_power_on,
	.power_off = t210ref_dw9714_power_off,
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
		.mask	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PS2),
		.values	= 1 << (CAM_FLASH_STROBE - TEGRA_GPIO_PS2)
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

	gpio_set_value(CAM1_PWDN, 0);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ov7695_avdd_fail;
	usleep_range(300, 320);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto ov7695_iovdd_fail;
	usleep_range(1000, 1020);

	gpio_set_value(CAM1_PWDN, 1);
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

	gpio_set_value(CAM1_PWDN, 0);
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
	.mclk_name = "cam_mclk1",
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

static struct nvc_imager_cap ov5693_front_cap = {
	.identifier				= "OV5693.1",
	.sensor_nvc_interface	= 5,
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
	.power_on	= t210ref_ov5693_front_power_on,
	.power_off	= t210ref_ov5693_front_power_off,
	.dev_name	= "ov5693.1",
	.cap		= &ov5693_front_cap,
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
	{"t210ref_ov5693_pdata", &t210ref_ov5693_pdata},
	{"t210ref_ad5823_pdata", &t210ref_ad5823_pdata},
	{"t210ref_as3648_pdata", &t210ref_as3648_data},
	{"t210ref_ov7695_pdata", &t210ref_ov7695_pdata},
	{"t210ref_ov5693f_pdata", &t210ref_ov5693_front_pdata},
	{"t210ref_imx214_pdata", &t210ref_imx214_data},
	{"t210ref_dw9714_pdata", &t210ref_dw9714_data},
	{},
};

void __init t210ref_camera_auxdata(void *data)
{
	struct of_dev_auxdata *aux_lut = data;
	while (aux_lut && aux_lut->compatible) {
		if (!strcmp(aux_lut->compatible, "nvidia,tegra210-camera")) {
			pr_info("%s: update camera lookup table.\n", __func__);
			aux_lut->platform_data = t210ref_camera_lut;
		}
		aux_lut++;
	}
}

int t210ref_camera_init(void)
{
	struct board_info board_info;

	pr_debug("%s: ++\n", __func__);
	tegra_get_board_info(&board_info);

	/* TODO: DPD mode is temarary turned off, need to put CSIA/B/C/D/E IOs
	 * into DPD mode to save additional power for t210ref later
	 */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);
	tegra_io_dpd_disable(&csie_io);

#if IS_ENABLED(CONFIG_SOC_CAMERA_PLATFORM)
	platform_device_register(&t210ref_soc_camera_device);
#endif

	return 0;
}
