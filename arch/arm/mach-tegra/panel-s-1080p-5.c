/*
 * arch/arm/mach-tegra/panel-s-1080p-5.c
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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

#include <mach/dc.h>
#include <mach/iomap.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/mfd/max8831.h>
#include <linux/max8831_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>

#include "gpio-names.h"
#include "board-panel.h"

#define DSI_PANEL_RESET         1
#define DSI_PANEL_RST_GPIO      TEGRA_GPIO_PH5
#define DSI_PANEL_BL_EN_GPIO    TEGRA_GPIO_PH2
#define DSI_PANEL_BL_PWM        TEGRA_GPIO_PH1

#define DC_CTRL_MODE            TEGRA_DC_OUT_CONTINUOUS_MODE

static struct regulator *vdd_lcd_s_1v8;
static struct regulator *vdd_sys_bl_3v7;

static bool dsi_s_1080p_5_reg_requested;
static bool dsi_s_1080p_5_gpio_requested;
static bool is_bl_powered;

static tegra_dc_bl_output dsi_s_1080p_5_bl_response_curve = {
	0, 2, 5, 7, 10, 13, 15, 18,
	20, 23, 26, 27, 29, 30, 31, 33,
	34, 36, 37, 39, 40, 41, 42, 44,
	45, 46, 47, 48, 50, 51, 52, 53,
	54, 55, 56, 57, 58, 59, 60, 61,
	62, 63, 64, 65, 66, 67, 68, 69,
	70, 71, 73, 74, 75, 76, 78, 79,
	80, 82, 83, 84, 86, 86, 87, 88,
	89, 89, 90, 91, 92, 92, 93, 94,
	95, 96, 97, 98, 99, 100, 101, 102,
	103, 104, 105, 106, 107, 107, 108, 109,
	110, 111, 112, 112, 113, 114, 114, 115,
	115, 116, 117, 117, 118, 119, 120, 121,
	121, 122, 123, 124, 125, 126, 127, 128,
	129, 130, 131, 132, 133, 134, 135, 136,
	136, 138, 139, 140, 141, 142, 143, 144,
	145, 146, 147, 148, 149, 150, 151, 152,
	153, 154, 155, 155, 156, 157, 158, 159,
	161, 162, 163, 164, 165, 166, 167, 167,
	167, 167, 168, 168, 168, 168, 168, 169,
	169, 170, 171, 172, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183,
	184, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 195, 195, 196, 197,
	198, 199, 200, 201, 202, 203, 204, 205,
	206, 206, 207, 207, 208, 208, 209, 209,
	210, 211, 211, 212, 213, 213, 214, 215,
	216, 216, 217, 218, 219, 220, 221, 222,
	223, 224, 225, 226, 227, 228, 229, 230,
	231, 232, 233, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};

static int __maybe_unused dsi_s_1080p_5_bl_notify(struct device *unused,
							int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_s_1080p_5_bl_response_curve[brightness];

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	return brightness;
}
static bool __maybe_unused dsi_s_1080p_5_check_bl_power(void)
{
	return is_bl_powered;
}

/*
	Sharp uses I2C max8831 blacklight device
*/
static struct led_info dsi_s_1080p_5_max8831_leds[] = {
	[MAX8831_ID_LED3] = {
		.name = "max8831:red:pluto",
	},
	[MAX8831_ID_LED4] = {
		.name = "max8831:green:pluto",
	},
	[MAX8831_ID_LED5] = {
		.name = "max8831:blue:pluto",
	},
};

static struct platform_max8831_backlight_data dsi_s_1080p_5_max8831_bl_data = {
	.id	= -1,
	.name	= "pluto_display_bl",
	.max_brightness	= MAX8831_BL_LEDS_MAX_CURR,
	.dft_brightness	= 100,
	.notify	= dsi_s_1080p_5_bl_notify,
	.is_powered = dsi_s_1080p_5_check_bl_power,
	.edp_states = {297, 267, 237, 208, 178, 148, 118, 89, 59, 29, 0},
	.edp_brightness = {255, 230, 204, 179, 153, 128, 102, 77, 51, 26, 0},
};

static struct max8831_subdev_info dsi_s_1080p_5_max8831_subdevs[] = {
	{
		.id = MAX8831_ID_LED3,
		.name = "max8831_led_bl",
		.platform_data = &dsi_s_1080p_5_max8831_leds[MAX8831_ID_LED3],
		.pdata_size = sizeof(
				dsi_s_1080p_5_max8831_leds[MAX8831_ID_LED3]),
	}, {
		.id = MAX8831_ID_LED4,
		.name = "max8831_led_bl",
		.platform_data = &dsi_s_1080p_5_max8831_leds[MAX8831_ID_LED4],
		.pdata_size = sizeof(
				dsi_s_1080p_5_max8831_leds[MAX8831_ID_LED4]),
	}, {
		.id = MAX8831_ID_LED5,
		.name = "max8831_led_bl",
		.platform_data = &dsi_s_1080p_5_max8831_leds[MAX8831_ID_LED5],
		.pdata_size = sizeof(
				dsi_s_1080p_5_max8831_leds[MAX8831_ID_LED5]),
	}, {
		.id = MAX8831_BL_LEDS,
		.name = "max8831_display_bl",
		.platform_data = &dsi_s_1080p_5_max8831_bl_data,
		.pdata_size = sizeof(dsi_s_1080p_5_max8831_bl_data),
	},
};

static struct max8831_platform_data dsi_s_1080p_5_max8831 = {
	.num_subdevs = ARRAY_SIZE(dsi_s_1080p_5_max8831_subdevs),
	.subdevs = dsi_s_1080p_5_max8831_subdevs,
};

static struct i2c_board_info dsi_s_1080p_5_i2c_led_info = {
	.type		= "max8831",
	.addr		= 0x4d,
	.platform_data	= &dsi_s_1080p_5_max8831,
};
static int __init dsi_s_1080p_5_register_bl_dev(void)
{
	int err = 0;
	err = i2c_register_board_info(1, &dsi_s_1080p_5_i2c_led_info, 1);
	return err;
}
struct tegra_dc_mode dsi_s_1080p_5_modes[] = {
	/* 1080x1920@60Hz */
	{
		.pclk = 143700000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 10,
		.v_sync_width = 2,
		.h_back_porch = 50,
		.v_back_porch = 4,
		.h_active = 1080,
		.v_active = 1920,
		.h_front_porch = 100,
		.v_front_porch = 4,
	},
	/* 1080x1920@53Hz */
	{
		.pclk = 143700000,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 10,
		.v_sync_width = 2,
		.h_back_porch = 50,
		.v_back_porch = 4,
		.h_active = 1080,
		.v_active = 1920,
		.h_front_porch = 100,
		.v_front_porch = 259,
	},
};
static int dsi_s_1080p_5_reg_get(void)
{
	int err = 0;

	if (dsi_s_1080p_5_reg_requested)
		return 0;

	vdd_lcd_s_1v8 = regulator_get(NULL, "vdd_lcd_1v8_s");
	if (IS_ERR_OR_NULL(vdd_lcd_s_1v8)) {
		pr_err("vdd_lcd_1v8_s regulator get failed\n");
		err = PTR_ERR(vdd_lcd_s_1v8);
		vdd_lcd_s_1v8 = NULL;
		goto fail;
	}

	vdd_sys_bl_3v7 = regulator_get(NULL, "vdd_sys_bl");
	if (IS_ERR_OR_NULL(vdd_sys_bl_3v7)) {
		pr_err("vdd_sys_bl regulator get failed\n");
		err = PTR_ERR(vdd_sys_bl_3v7);
		vdd_sys_bl_3v7 = NULL;
		goto fail;
	}

	dsi_s_1080p_5_reg_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_s_1080p_5_gpio_get(void)
{
	int err = 0;

	if (dsi_s_1080p_5_gpio_requested)
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



	dsi_s_1080p_5_gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_s_1080p_5_enable(struct device *dev)
{
	int err = 0;

	err = dsi_s_1080p_5_reg_get();
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = dsi_s_1080p_5_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}
	gpio_direction_output(DSI_PANEL_RST_GPIO, 0);

	if (vdd_lcd_s_1v8) {
		err = regulator_enable(vdd_lcd_s_1v8);
		if (err < 0) {
			pr_err("vdd_lcd_1v8_s regulator enable failed\n");
			goto fail;
		}
	}
	usleep_range(3000, 5000);

	if (vdd_sys_bl_3v7) {
		err = regulator_enable(vdd_sys_bl_3v7);
		if (err < 0) {
			pr_err("vdd_sys_bl regulator enable failed\n");
			goto fail;
		}
	}
	gpio_direction_output(DSI_PANEL_BL_EN_GPIO, 1);
	is_bl_powered = true;
	usleep_range(3000, 5000);

#if DSI_PANEL_RESET
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
	msleep(20);
#endif
	return 0;
fail:
	return err;
}

static u8 panel_internal[] = {0x51, 0x0f, 0xff};

static struct tegra_dsi_cmd dsi_s_1080p_5_init_cmd[] = {

	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xb0, 0x04),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_NO_OP, 0x0),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_NO_OP, 0x0),
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xd6, 0x01),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_internal),
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0x53, 0x04),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
};

static struct tegra_dsi_cmd dsi_s_1080p_5_suspend_cmd[] = {
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_OFF, 0x0),
	DSI_DLY_MS(50),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_ENTER_SLEEP_MODE, 0x0),
};

static struct tegra_dsi_out dsi_s_1080p_5_pdata = {
	.n_data_lanes = 4,

	.dsi_instance = DSI_INSTANCE_1,

	.refresh_rate = 60,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
	.controller_vs = DSI_VS_1,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,

	.dsi_init_cmd = dsi_s_1080p_5_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_s_1080p_5_init_cmd),

	.dsi_suspend_cmd = dsi_s_1080p_5_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_s_1080p_5_suspend_cmd),
};

static int dsi_s_1080p_5_disable(void)
{
	/* delay between sleep in and reset low */
	msleep(100);

	gpio_set_value(DSI_PANEL_RST_GPIO, 0);
	usleep_range(3000, 5000);

	gpio_set_value(DSI_PANEL_BL_EN_GPIO, 0);
	if (vdd_sys_bl_3v7)
		regulator_disable(vdd_sys_bl_3v7);
	is_bl_powered = false;
	usleep_range(3000, 5000);

	if (vdd_lcd_s_1v8)
		regulator_disable(vdd_lcd_s_1v8);

	return 0;
}

static void dsi_s_1080p_5_resources_init(struct resource *
resources, int n_resources)
{
	int i;
	for (i = 0; i < n_resources; i++) {
		struct resource *r = &resources[i];
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "dsi_regs")) {
			r->start = TEGRA_DSIB_BASE;
			r->end = TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1;
		}
	}
}

static void dsi_s_1080p_5_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_s_1080p_5_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_s_1080p_5_modes;
	dc->n_modes = ARRAY_SIZE(dsi_s_1080p_5_modes);
	dc->enable = dsi_s_1080p_5_enable;
	dc->disable = dsi_s_1080p_5_disable;
	dc->width = 62;
	dc->height = 110;
	dc->flags = DC_CTRL_MODE;
}
static void dsi_s_1080p_5_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_s_1080p_5_modes[0].h_active;
	fb->yres = dsi_s_1080p_5_modes[0].v_active;
}

static void dsi_s_1080p_5_sd_settings_init
(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "max8831_display_bl";
}

struct tegra_panel __initdata dsi_s_1080p_5 = {
	.init_sd_settings = dsi_s_1080p_5_sd_settings_init,
	.init_dc_out = dsi_s_1080p_5_dc_out_init,
	.init_fb_data = dsi_s_1080p_5_fb_data_init,
	.init_resources = dsi_s_1080p_5_resources_init,
	.register_bl_dev = dsi_s_1080p_5_register_bl_dev,
};
EXPORT_SYMBOL(dsi_s_1080p_5);
