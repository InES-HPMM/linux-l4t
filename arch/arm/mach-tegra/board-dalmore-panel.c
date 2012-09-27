/*
 * arch/arm/mach-tegra/board-dalmore-panel.c
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

#define TEGRA_DSI_GANGED_MODE	0
#define IS_EXTERNAL_PWM		1

/* PANEL_<diagonal length in inches>_<vendor name>_<resolution> */
#define PANEL_10_1_PANASONIC_1920_1200	1
#define PANEL_11_6_AUO_1920_1080	0
#define PANEL_10_1_SHARP_2560_1600	0

#define DSI_PANEL_RESET		1
#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PH3
#define DSI_PANEL_BL_EN_GPIO	TEGRA_GPIO_PH2

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

/* HDMI Hotplug detection pin */
#define dalmore_hdmi_hpd	TEGRA_GPIO_PN7

static atomic_t sd_brightness = ATOMIC_INIT(255);

static bool reg_requested;
static bool gpio_requested;

/* for PANEL_10_1_PANASONIC_1920_1200, PANEL_11_6_AUO_1920_1080
 * and PANEL_10_1_SHARP_2560_1600
 */
static struct regulator *avdd_lcd_3v3;
static struct regulator *vdd_lcd_bl_12v;

/* for PANEL_11_6_AUO_1920_1080 and PANEL_10_1_SHARP_2560_1600 */
static struct regulator *dvdd_lcd_1v8;

/* for PANEL_11_6_AUO_1920_1080 */
static struct regulator *vdd_ds_1v8;
#define en_vdd_bl	TEGRA_GPIO_PG0
#define lvds_en		TEGRA_GPIO_PG3

#ifdef CONFIG_TEGRA_DC
static struct regulator *dalmore_hdmi_reg;
static struct regulator *dalmore_hdmi_pll;
static struct regulator *dalmore_hdmi_vddio;
#endif

static struct resource dalmore_disp1_resources[] = {
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
		.start	= 0, /* Filled in by dalmore_panel_init() */
		.end	= 0, /* Filled in by dalmore_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
#if TEGRA_DSI_GANGED_MODE
	{
		.name	= "ganged_dsia_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsib_regs",
		.start	= TEGRA_DSIB_BASE,
		.end	= TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#else
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

static struct resource dalmore_disp2_resources[] = {
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
		.start	= 0, /* Filled in by dalmore_panel_init() */
		.end	= 0, /* Filled in by dalmore_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dsi_cmd dsi_init_cmd[] = {
#if PANEL_10_1_PANASONIC_1920_1200
	/* no init command required */
#endif
#if PANEL_11_6_AUO_1920_1080
	/* no init command required */
#endif
#if PANEL_10_1_SHARP_2560_1600
	/* TODO */
#endif
};

static struct tegra_dsi_out dalmore_dsi = {
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	.n_data_lanes = 2,
	.controller_vs = DSI_VS_0,
#else
	.n_data_lanes = 4,
	.controller_vs = DSI_VS_1,
#endif
#if PANEL_11_6_AUO_1920_1080
	.dsi2edp_bridge_enable = true,
#endif
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 59,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.dsi_instance = DSI_INSTANCE_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
#else
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
#endif
	.dsi_init_cmd = dsi_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
};

static int dalmore_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

#if PANEL_11_6_AUO_1920_1080 || \
	PANEL_10_1_SHARP_2560_1600
	dvdd_lcd_1v8 = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR_OR_NULL(dvdd_lcd_1v8)) {
		pr_err("dvdd_lcd regulator get failed\n");
		err = PTR_ERR(dvdd_lcd_1v8);
		dvdd_lcd_1v8 = NULL;
		goto fail;
	}
#endif
#if PANEL_11_6_AUO_1920_1080
	vdd_ds_1v8 = regulator_get(dev, "vdd_ds_1v8");
	if (IS_ERR_OR_NULL(vdd_ds_1v8)) {
		pr_err("vdd_ds_1v8 regulator get failed\n");
		err = PTR_ERR(vdd_ds_1v8);
		vdd_ds_1v8 = NULL;
		goto fail;
	}
#endif
#if PANEL_10_1_PANASONIC_1920_1200 || \
	PANEL_11_6_AUO_1920_1080 || \
	PANEL_10_1_SHARP_2560_1600
	avdd_lcd_3v3 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd_3v3)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v3);
		avdd_lcd_3v3 = NULL;
		goto fail;
	}

	vdd_lcd_bl_12v = regulator_get(dev, "vdd_lcd_bl");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_12v)) {
		pr_err("vdd_lcd_bl regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_12v);
		vdd_lcd_bl_12v = NULL;
		goto fail;
	}
#endif
	reg_requested = true;
	return 0;
fail:
	return err;
}

static int dalmore_dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
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

#if PANEL_11_6_AUO_1920_1080
	err = gpio_request(en_vdd_bl, "edp bridge 1v2 enable");
	if (err < 0) {
		pr_err("edp bridge 1v2 enable gpio request failed\n");
		goto fail;
	}

	err = gpio_request(lvds_en, "edp bridge 1v8 enable");
	if (err < 0) {
		pr_err("edp bridge 1v8 enable gpio request failed\n");
		goto fail;
	}
#endif
	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dalmore_dsi_panel_enable(struct device *dev)
{
	int err = 0;

	err = dalmore_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	err = dalmore_dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	if (vdd_ds_1v8) {
		err = regulator_enable(vdd_ds_1v8);
		if (err < 0) {
			pr_err("vdd_ds_1v8 regulator enable failed\n");
			goto fail;
		}
	}

	if (dvdd_lcd_1v8) {
		err = regulator_enable(dvdd_lcd_1v8);
		if (err < 0) {
			pr_err("dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	if (avdd_lcd_3v3) {
		err = regulator_enable(avdd_lcd_3v3);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_lcd_bl_12v) {
		err = regulator_enable(vdd_lcd_bl_12v);
		if (err < 0) {
			pr_err("vdd_lcd_bl regulator enable failed\n");
			goto fail;
		}
	}

	msleep(100);
#if DSI_PANEL_RESET
	gpio_direction_output(DSI_PANEL_RST_GPIO, 1);
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 0);
	msleep(150);
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
	msleep(1500);
#endif

	gpio_direction_output(DSI_PANEL_BL_EN_GPIO, 1);
	gpio_direction_output(57, 1);

#if PANEL_11_6_AUO_1920_1080
	gpio_direction_output(en_vdd_bl, 1);
	msleep(100);
	gpio_direction_output(lvds_en, 1);
#endif
	return 0;
fail:
	return err;
}

static int dalmore_dsi_panel_disable(void)
{
	gpio_set_value(DSI_PANEL_BL_EN_GPIO, 0);

#if PANEL_11_6_AUO_1920_1080
	gpio_set_value(lvds_en, 0);
	msleep(100);
	gpio_set_value(en_vdd_bl, 0);
#endif
	if (vdd_lcd_bl_12v)
		regulator_disable(vdd_lcd_bl_12v);

	if (avdd_lcd_3v3)
		regulator_disable(avdd_lcd_3v3);

	if (dvdd_lcd_1v8)
		regulator_disable(dvdd_lcd_1v8);

	if (vdd_ds_1v8)
		regulator_disable(vdd_ds_1v8);

	return 0;
}

static int dalmore_dsi_panel_postsuspend(void)
{
	return 0;
}

static struct tegra_dc_mode dalmore_dsi_modes[] = {
#if PANEL_10_1_PANASONIC_1920_1200
	{
		.pclk = 10000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 2,
		.h_back_porch = 32,
		.v_back_porch = 16,
		.h_active = 1920,
		.v_active = 1200,
		.h_front_porch = 120,
		.v_front_porch = 17,
	},
#endif
#if PANEL_11_6_AUO_1920_1080
	{
		.pclk = 10000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 28,
		.v_sync_width = 5,
		.h_back_porch = 148,
		.v_back_porch = 23,
		.h_active = 1920,
		.v_active = 1080,
		.h_front_porch = 66,
		.v_front_porch = 4,
	},
#endif
#if PANEL_10_1_SHARP_2560_1600
	{
		.pclk = 10000000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 16,
		.v_sync_width = 2,
		.h_back_porch = 16,
		.v_back_porch = 33,
		.h_active = 2560,
		.v_active = 1600,
		.h_front_porch = 128,
		.v_front_porch = 10,
	},
#endif
};

static struct tegra_dc_out dalmore_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
	.dsi		= &dalmore_dsi,

	.flags		= DC_CTRL_MODE,

	.modes		= dalmore_dsi_modes,
	.n_modes	= ARRAY_SIZE(dalmore_dsi_modes),

	.enable		= dalmore_dsi_panel_enable,
	.disable	= dalmore_dsi_panel_disable,
	.postsuspend	= dalmore_dsi_panel_postsuspend,

#if PANEL_10_1_PANASONIC_1920_1200
	.width		= 217,
	.height		= 135,
#endif
#if PANEL_11_6_AUO_1920_1080
	.width		= 256,
	.height		= 144,
#endif
#if PANEL_10_1_SHARP_2560_1600
	.width		= 216,
	.height		= 135,
#endif
};

static int dalmore_hdmi_enable(struct device *dev)
{
	int ret;
	if (!dalmore_hdmi_reg) {
			dalmore_hdmi_reg = regulator_get(dev, "avdd_hdmi");
			if (IS_ERR_OR_NULL(dalmore_hdmi_reg)) {
				pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
				dalmore_hdmi_reg = NULL;
				return PTR_ERR(dalmore_hdmi_reg);
			}
	}
	ret = regulator_enable(dalmore_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!dalmore_hdmi_pll) {
		dalmore_hdmi_pll = regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(dalmore_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			dalmore_hdmi_pll = NULL;
			regulator_put(dalmore_hdmi_reg);
			dalmore_hdmi_reg = NULL;
			return PTR_ERR(dalmore_hdmi_pll);
		}
	}
	ret = regulator_enable(dalmore_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int dalmore_hdmi_disable(void)
{
	if (dalmore_hdmi_reg) {
		regulator_disable(dalmore_hdmi_reg);
		regulator_put(dalmore_hdmi_reg);
		dalmore_hdmi_reg = NULL;
	}

	if (dalmore_hdmi_pll) {
		regulator_disable(dalmore_hdmi_pll);
		regulator_put(dalmore_hdmi_pll);
		dalmore_hdmi_pll = NULL;
	}

	return 0;
}

static int dalmore_hdmi_postsuspend(void)
{
	if (dalmore_hdmi_vddio) {
		regulator_disable(dalmore_hdmi_vddio);
		regulator_put(dalmore_hdmi_vddio);
		dalmore_hdmi_vddio = NULL;
	}
	return 0;
}

static int dalmore_hdmi_hotplug_init(struct device *dev)
{
	if (!dalmore_hdmi_vddio) {
		dalmore_hdmi_vddio = regulator_get(dev, "vdd_hdmi_5v0");
		if (WARN_ON(IS_ERR(dalmore_hdmi_vddio))) {
			pr_err("%s: couldn't get regulator vdd_hdmi_5v0: %ld\n",
				__func__, PTR_ERR(dalmore_hdmi_vddio));
				dalmore_hdmi_vddio = NULL;
		} else {
			regulator_enable(dalmore_hdmi_vddio);
		}
	}

	return 0;
}

static struct tegra_dc_out dalmore_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	= "pll_d2_out0",

	.dcc_bus	= 3,
	.hotplug_gpio	= dalmore_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= dalmore_hdmi_enable,
	.disable	= dalmore_hdmi_disable,
	.postsuspend	= dalmore_hdmi_postsuspend,
	.hotplug_init	= dalmore_hdmi_hotplug_init,
};

static struct tegra_fb_data dalmore_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
#if PANEL_10_1_PANASONIC_1920_1200
	.xres		= 1920,
	.yres		= 1200,
#endif
#if PANEL_11_6_AUO_1920_1080
	.xres		= 1920,
	.yres		= 1080,
#endif
#if PANEL_10_1_SHARP_2560_1600
	.xres		= 2560,
	.yres		= 1600,
#endif
};

static struct tegra_dc_platform_data dalmore_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &dalmore_disp1_out,
	.fb		= &dalmore_disp1_fb_data,
	.emc_clk_rate	= 204000000,
};

static struct tegra_fb_data dalmore_disp2_fb_data = {
	.win		= 0,
	.xres		= 1024,
	.yres		= 600,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data dalmore_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &dalmore_disp2_out,
	.fb		= &dalmore_disp2_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct nvhost_device dalmore_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= dalmore_disp2_resources,
	.num_resources	= ARRAY_SIZE(dalmore_disp2_resources),
	.dev = {
		.platform_data = &dalmore_disp2_pdata,
	},
};

static struct nvhost_device dalmore_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= dalmore_disp1_resources,
	.num_resources	= ARRAY_SIZE(dalmore_disp1_resources),
	.dev = {
		.platform_data = &dalmore_disp1_pdata,
	},
};

static struct nvmap_platform_carveout dalmore_carveouts[] = {
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
		.base		= 0, /* Filled in by dalmore_panel_init() */
		.size		= 0, /* Filled in by dalmore_panel_init() */
		.buddy_size	= SZ_32K,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by dalmore_panel_init() */
		.size		= 0, /* Filled in by dalmore_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data dalmore_nvmap_data = {
	.carveouts	= dalmore_carveouts,
	.nr_carveouts	= ARRAY_SIZE(dalmore_carveouts),
};

static struct platform_device dalmore_nvmap_device __initdata = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &dalmore_nvmap_data,
	},
};

static int dalmore_disp1_bl_notify(struct device *unused, int brightness)
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

static int dalmore_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &dalmore_disp1_device.dev;
}

static struct platform_tegra_pwm_backlight_data dalmore_disp1_bl_data = {
	.which_dc		= 0,
	.which_pwm		= TEGRA_PWM_PM1,
	.gpio_conf_to_sfio	= TEGRA_GPIO_PH1,
	.max_brightness		= 255,
	.dft_brightness		= 224,
	.notify			= dalmore_disp1_bl_notify,
	.period			= 0x3F,
	.clk_div		= 0x3FF,
	.clk_select		= 0,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb		= dalmore_disp1_check_fb,
};

static struct platform_device __maybe_unused
		dalmore_disp1_bl_device __initdata = {
	.name	= "tegra-pwm-bl",
	.id	= -1,
	.dev	= {
		.platform_data = &dalmore_disp1_bl_data,
	},
};

#if PANEL_11_6_AUO_1920_1080
static struct i2c_board_info dalmore_tc358770_dsi2edp_board_info __initdata = {
		I2C_BOARD_INFO("tc358770_dsi2edp", 0x68),
};
#endif

int __init dalmore_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;

#ifdef CONFIG_TEGRA_NVMAP
	dalmore_carveouts[1].base = tegra_carveout_start;
	dalmore_carveouts[1].size = tegra_carveout_size;
	dalmore_carveouts[2].base = tegra_vpr_start;
	dalmore_carveouts[2].size = tegra_vpr_size;

	err = platform_device_register(&dalmore_nvmap_device);
	if (err) {
		pr_err("nvmap device registration failed\n");
		return err;
	}
#endif

	gpio_request(dalmore_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(dalmore_hdmi_hpd);

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
	res = nvhost_get_resource_byname(&dalmore_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	res = nvhost_get_resource_byname(&dalmore_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	err = nvhost_device_register(&dalmore_disp1_device);
	if (err) {
		pr_err("disp1 device registration failed\n");
		return err;
	}

	err = nvhost_device_register(&dalmore_disp2_device);
	if (err) {
		pr_err("disp2 device registration failed\n");
		return err;
	}

#if !IS_EXTERNAL_PWM
	err = platform_device_register(&dalmore_disp1_bl_device);
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
#endif
#if PANEL_11_6_AUO_1920_1080
	i2c_register_board_info(0, &dalmore_tc358770_dsi2edp_board_info, 1);
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
int __init dalmore_panel_init(void)
{
	return -ENODEV;
}
#endif
