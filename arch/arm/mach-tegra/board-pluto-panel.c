/*
 * arch/arm/mach-tegra/board-pluto-panel.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/ioport.h>
#include <linux/fb.h>
#include <linux/nvmap.h>
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>

#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
#include "tegra3_host1x_devices.h"
#else
#include "tegra11_host1x_devices.h"
#endif

#define TEGRA_PANEL_ENABLE	1

#if TEGRA_PANEL_ENABLE
#define IS_EXTERNAL_PWM		1

/* PANEL_<diagonal length in inches>_<vendor name>_<resolution> */
#define PANEL_5_LG_720_1280	1
#define PANEL_4_7_JDI_720_1280	0

#define DSI_PANEL_RESET		1
#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PH5
#define DSI_PANEL_BL_EN_GPIO	TEGRA_GPIO_PH2

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

static atomic_t sd_brightness = ATOMIC_INIT(255);

static bool dsi_reg_requested;
static bool dsi_gpio_requested;

/* for PANEL_5_LG_720_1280 and PANEL_4_7_JDI_720_1280 */
static struct regulator *vdd_lcd_s_1v8;
static struct regulator *vdd_sys_bl_3v7;
static struct regulator *avdd_lcd_3v0_2v8;

/* for PANEL_5_LG_720_1280 */
static struct regulator *avdd_ts_3v0;

/* hdmi pins for hotplug */
#define pluto_hdmi_hpd		TEGRA_GPIO_PN7

/* hdmi related regulators */
#ifdef CONFIG_TEGRA_DC
static struct regulator *pluto_hdmi_vddio;
#endif

static struct resource pluto_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by pluto_panel_init() */
		.end	= 0, /* Filled in by pluto_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource pluto_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by pluto_panel_init() */
		.end	= 0, /* Filled in by pluto_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

#if PANEL_5_LG_720_1280
static u8 panel_dsi_config[] = {0xe0, 0x43, 0x0, 0x80, 0x0, 0x0};
static u8 panel_disp_ctrl1[] = {0xb5, 0x34, 0x20, 0x40, 0x0, 0x20};
static u8 panel_disp_ctrl2[] = {0xb6, 0x04, 0x74, 0x0f, 0x16, 0x13};
static u8 panel_internal_clk[] = {0xc0, 0x01, 0x08};
static u8 panel_pwr_ctrl3[] =
	{0xc3, 0x0, 0x09, 0x10, 0x02, 0x0, 0x66, 0x20, 0x13, 0x0};
static u8 panel_pwr_ctrl4[] = {0xc4, 0x23, 0x24, 0x17, 0x17, 0x59};
static u8 panel_positive_gamma_red[] =
	{0xd0, 0x21, 0x13, 0x67, 0x37, 0x0c, 0x06, 0x62, 0x23, 0x03};
static u8 panel_negetive_gamma_red[] =
	{0xd1, 0x32, 0x13, 0x66, 0x37, 0x02, 0x06, 0x62, 0x23, 0x03};
static u8 panel_positive_gamma_green[] =
	{0xd2, 0x41, 0x14, 0x56, 0x37, 0x0c, 0x06, 0x62, 0x23, 0x03};
static u8 panel_negetive_gamma_green[] =
	{0xd3, 0x52, 0x14, 0x55, 0x37, 0x02, 0x06, 0x62, 0x23, 0x03};
static u8 panel_positive_gamma_blue[] =
	{0xd4, 0x41, 0x14, 0x56, 0x37, 0x0c, 0x06, 0x62, 0x23, 0x03};
static u8 panel_negetive_gamma_blue[] =
	{0xd5, 0x52, 0x14, 0x55, 0x37, 0x02, 0x06, 0x62, 0x23, 0x03};
static u8 panel_ce2[] = {0x71, 0x0, 0x0, 0x01, 0x01};
static u8 panel_ce3[] = {0x72, 0x01, 0x0e};
static u8 panel_ce4[] = {0x73, 0x34, 0x52, 0x0};
static u8 panel_ce5[] = {0x74, 0x05, 0x0, 0x06};
static u8 panel_ce6[] = {0x75, 0x03, 0x0, 0x07};
static u8 panel_ce7[] = {0x76, 0x07, 0x0, 0x06};
static u8 panel_ce8[] = {0x77, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f};
static u8 panel_ce9[] = {0x78, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40};
static u8 panel_ce10[] =
	{0x79, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40};
static u8 panel_ce11[] = {0x7a, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static u8 panel_ce12[] = {0x7b, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
static u8 panel_ce13[] = {0x7c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
#endif

static struct tegra_dsi_cmd dsi_init_cmd[] = {
#if PANEL_5_LG_720_1280
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_dsi_config),

	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_disp_ctrl1),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_disp_ctrl2),

	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_internal_clk),

	/*  panel power control 1 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc1, 0x0),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_pwr_ctrl3),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_pwr_ctrl4),

	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_positive_gamma_red),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_negetive_gamma_red),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_positive_gamma_green),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_negetive_gamma_green),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_positive_gamma_blue),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_negetive_gamma_blue),

	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, DSI_DCS_SET_ADDR_MODE, 0x08),

	/* panel OTP 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xf9, 0x0),

	/* panel CE 1 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0x70, 0x0),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce2),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce3),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce4),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce5),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce6),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce7),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce8),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce9),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce10),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce11),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce12),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_ce13),

	DSI_DLY_MS(100),
	/* panel power control 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc2, 0x02),
	DSI_DLY_MS(200),
	/* panel power control 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc2, 0x06),
	DSI_DLY_MS(200),
	/* panel power control 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc2, 0x4e),
	DSI_DLY_MS(200),

	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(1000),

	/* panel OTP 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xf9, 0x80),
	DSI_DLY_MS(1000),

	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_DLY_MS(1000),
#endif
#if PANEL_4_7_JDI_720_1280
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x00),
	DSI_DLY_MS(15),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x00),
#endif
};

static struct tegra_dsi_out pluto_dsi = {
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	.n_data_lanes = 2,
	.controller_vs = DSI_VS_0,
	.dsi_instance = DSI_INSTANCE_0,
#else
#if PANEL_4_7_JDI_720_1280
	.n_data_lanes = 3,
	.dsi_instance = DSI_INSTANCE_1,
#else
	.n_data_lanes = 4,
	.dsi_instance = DSI_INSTANCE_0,
#endif
	.controller_vs = DSI_VS_1,
#endif
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
#else
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
#endif
	.dsi_init_cmd = dsi_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
};

static int pluto_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (dsi_reg_requested)
		return 0;

#if PANEL_5_LG_720_1280
	avdd_ts_3v0 = regulator_get(dev, "avdd_ts_3v0");
	if (IS_ERR_OR_NULL(avdd_ts_3v0)) {
		pr_err("avdd_ts_3v0 regulator get failed\n");
		err = PTR_ERR(avdd_ts_3v0);
		avdd_ts_3v0 = NULL;
		goto fail;
	}
#endif
#if PANEL_5_LG_720_1280 || PANEL_4_7_JDI_720_1280
	avdd_lcd_3v0_2v8 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd_3v0_2v8)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v0_2v8);
		avdd_lcd_3v0_2v8 = NULL;
		goto fail;
	}

	vdd_lcd_s_1v8 = regulator_get(dev, "vdd_lcd_1v8_s");
	if (IS_ERR_OR_NULL(vdd_lcd_s_1v8)) {
		pr_err("vdd_lcd_1v8_s regulator get failed\n");
		err = PTR_ERR(vdd_lcd_s_1v8);
		vdd_lcd_s_1v8 = NULL;
		goto fail;
	}

	vdd_sys_bl_3v7 = regulator_get(dev, "vdd_sys_bl");
	if (IS_ERR_OR_NULL(vdd_sys_bl_3v7)) {
		pr_err("vdd_sys_bl regulator get failed\n");
		err = PTR_ERR(vdd_sys_bl_3v7);
		vdd_sys_bl_3v7 = NULL;
		goto fail;
	}
#endif
	dsi_reg_requested = true;
	return 0;
fail:
	return err;
}

static int pluto_dsi_gpio_get(void)
{
	int err = 0;

	if (dsi_gpio_requested)
		return 0;

	err = gpio_request(DSI_PANEL_RST_GPIO, "panel rst");
	if (err < 0) {
		pr_err("panel reset gpio request failed\n");
		goto fail;
	}

	err = gpio_request(DSI_PANEL_BL_EN_GPIO, "panel backlight");
	if (err < 0) {
		pr_err("panel backlight gpio request failed\n");
		goto fail;
	}

	dsi_gpio_requested = true;
	return 0;
fail:
	return err;
}

static int pluto_dsi_panel_enable(struct device *dev)
{
	int err = 0;

	err = pluto_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = pluto_dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	if (avdd_lcd_3v0_2v8) {
		err = regulator_enable(avdd_lcd_3v0_2v8);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
#if PANEL_5_LG_720_1280
		regulator_set_voltage(avdd_lcd_3v0_2v8, 2800000, 2800000);
#endif
#if PANEL_4_7_JDI_720_1280
		regulator_set_voltage(avdd_lcd_3v0_2v8, 3000000, 3000000);
#endif
	}

	if (avdd_ts_3v0) {
		err = regulator_enable(avdd_ts_3v0);
		if (err < 0) {
			pr_err("avdd_ts_3v0 regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_lcd_s_1v8) {
		err = regulator_enable(vdd_lcd_s_1v8);
		if (err < 0) {
			pr_err("vdd_lcd_1v8_s regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_sys_bl_3v7) {
		err = regulator_enable(vdd_sys_bl_3v7);
		if (err < 0) {
			pr_err("vdd_sys_bl regulator enable failed\n");
			goto fail;
		}
	}

#if DSI_PANEL_RESET
	gpio_direction_output(DSI_PANEL_RST_GPIO, 1);
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 0);
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
	msleep(20);
#endif
	gpio_direction_output(DSI_PANEL_BL_EN_GPIO, 1);
	gpio_direction_output(TEGRA_GPIO_PH1, 1);

	return 0;
fail:
	return err;
}

static int pluto_dsi_panel_disable(void)
{
	gpio_set_value(DSI_PANEL_BL_EN_GPIO, 0);

	if (vdd_sys_bl_3v7)
		regulator_disable(vdd_sys_bl_3v7);

	if (vdd_lcd_s_1v8)
		regulator_disable(vdd_lcd_s_1v8);

	if (avdd_ts_3v0)
		regulator_disable(avdd_ts_3v0);

	if (avdd_lcd_3v0_2v8)
		regulator_disable(avdd_lcd_3v0_2v8);

	return 0;
}

static int pluto_dsi_panel_postsuspend(void)
{
	/* TODO */
	return 0;
}

static struct tegra_dc_mode pluto_dsi_modes[] = {
#if PANEL_5_LG_720_1280
	{
		.pclk = 10000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 4,
		.v_sync_width = 4,
		.h_back_porch = 82,
		.v_back_porch = 7,
		.h_active = 720,
		.v_active = 1280,
		.h_front_porch = 4,
		.v_front_porch = 20,
	},
#endif
#if PANEL_4_7_JDI_720_1280
	{
		.pclk = 10000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 4,
		.h_back_porch = 32,
		.v_back_porch = 4,
		.h_active = 720,
		.v_active = 1280,
		.h_front_porch = 48,
		.v_front_porch = 4,
	},
#endif
};

static struct tegra_dc_out pluto_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
	.dsi		= &pluto_dsi,

	.flags		= DC_CTRL_MODE,

	.modes		= pluto_dsi_modes,
	.n_modes	= ARRAY_SIZE(pluto_dsi_modes),

	.enable		= pluto_dsi_panel_enable,
	.disable	= pluto_dsi_panel_disable,
	.postsuspend	= pluto_dsi_panel_postsuspend,

#if PANEL_5_LG_720_1280
	.width		= 62,
	.height		= 110,
#endif
#if PANEL_4_7_JDI_720_1280
	.width = 58,
	.height = 103,
#endif
};

static int pluto_hdmi_enable(struct device *dev)
{
	/* TODO */
	return 0;
}

static int pluto_hdmi_disable(void)
{
	/* TODO */
	return 0;
}

static int pluto_hdmi_postsuspend(void)
{
	if (pluto_hdmi_vddio) {
		regulator_disable(pluto_hdmi_vddio);
		regulator_put(pluto_hdmi_vddio);
		pluto_hdmi_vddio = NULL;
	}
	return 0;
}

static int pluto_hdmi_hotplug_init(struct device *dev)
{
	int ret = 0;
	if (!pluto_hdmi_vddio) {
		pluto_hdmi_vddio = regulator_get(dev, "vdd_hdmi_5v0");
		if (IS_ERR_OR_NULL(pluto_hdmi_vddio)) {
			ret = PTR_ERR(pluto_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_5v0\n");
			pluto_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(pluto_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_5v0\n");
		regulator_put(pluto_hdmi_vddio);
		pluto_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static struct tegra_dc_out pluto_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	= "pll_d2_out0",

	.dcc_bus	= 3,
	.hotplug_gpio	= pluto_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.enable		= pluto_hdmi_enable,
	.disable	= pluto_hdmi_disable,
	.postsuspend	= pluto_hdmi_postsuspend,
	.hotplug_init	= pluto_hdmi_hotplug_init,
};

static struct tegra_fb_data pluto_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
#if PANEL_5_LG_720_1280 || PANEL_4_7_JDI_720_1280
	.xres		= 720,
	.yres		= 1280,
#endif
};

static struct tegra_dc_platform_data pluto_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &pluto_disp1_out,
	.fb		= &pluto_disp1_fb_data,
	.emc_clk_rate	= 204000000,
};

static struct tegra_fb_data pluto_disp2_fb_data = {
	.win		= 0,
	.xres		= 1024,
	.yres		= 600,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data pluto_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &pluto_disp2_out,
	.fb		= &pluto_disp2_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct nvhost_device pluto_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= pluto_disp2_resources,
	.num_resources	= ARRAY_SIZE(pluto_disp2_resources),
	.dev = {
		.platform_data = &pluto_disp2_pdata,
	},
};

static struct nvhost_device pluto_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= pluto_disp1_resources,
	.num_resources	= ARRAY_SIZE(pluto_disp1_resources),
	.dev = {
		.platform_data = &pluto_disp1_pdata,
	},
};

static struct nvmap_platform_carveout pluto_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.buddy_size	= 0, /* no buddy allocation for IRAM */
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0, /* Filled in by pluto_panel_init() */
		.size		= 0, /* Filled in by pluto_panel_init() */
		.buddy_size	= SZ_32K,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by pluto_panel_init() */
		.size		= 0, /* Filled in by pluto_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data pluto_nvmap_data = {
	.carveouts	= pluto_carveouts,
	.nr_carveouts	= ARRAY_SIZE(pluto_carveouts),
};

static struct platform_device pluto_nvmap_device __initdata = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &pluto_nvmap_data,
	},
};

static int pluto_disp1_bl_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		/* TODO: backlight response LUT */
		brightness = brightness;

	return brightness;
}

static int pluto_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &pluto_disp1_device.dev;
}

static struct platform_tegra_pwm_backlight_data pluto_disp1_bl_data = {
	.which_dc		= 0,
	.which_pwm		= TEGRA_PWM_PM1,
	.gpio_conf_to_sfio	= TEGRA_GPIO_PH1,
	.max_brightness		= 255,
	.dft_brightness		= 224,
	.notify			= pluto_disp1_bl_notify,
	.period			= 0x3F,
	.clk_div		= 0x3FF,
	.clk_select		= 0,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb		= pluto_disp1_check_fb,
};

static struct platform_device __maybe_unused pluto_disp1_bl_device __initdata = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &pluto_disp1_bl_data,
	},
};

int __init pluto_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;

#ifdef CONFIG_TEGRA_NVMAP
	pluto_carveouts[1].base = tegra_carveout_start;
	pluto_carveouts[1].size = tegra_carveout_size;
	pluto_carveouts[2].base = tegra_vpr_start;
	pluto_carveouts[2].size = tegra_vpr_size;

	err = platform_device_register(&pluto_nvmap_device);
	if (err) {
		pr_err("nvmap device registration failed\n");
		return err;
	}
#endif
	gpio_request(pluto_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(pluto_hdmi_hpd);

#ifdef CONFIG_TEGRA_GRHOST
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	err = tegra3_register_host1x_devices();
#else
	err = tegra11_register_host1x_devices();
#endif
	if (err) {
		pr_err("host1x devices registration failed\n");
		return err;
	}

#ifdef CONFIG_TEGRA_DC
	res = nvhost_get_resource_byname(&pluto_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&pluto_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	err = nvhost_device_register(&pluto_disp1_device);
	if (err) {
		pr_err("disp1 device registration failed\n");
		return err;
	}

	err = nvhost_device_register(&pluto_disp2_device);
	if (err) {
		pr_err("disp2 device registration failed\n");
		return err;
	}

#if !IS_EXTERNAL_PWM
	err = platform_device_register(&pluto_disp1_bl_device);
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
#endif
#endif

#ifdef CONFIG_TEGRA_NVAVP
	err = nvhost_device_register(&nvavp_device);
	if (err) {
		pr_err("nvavp device registration failed\n");
		return err;
	}
#endif
#endif
	return err;
}
#else
int __init pluto_panel_init(void)
{
	return -ENODEV;
}
#endif
