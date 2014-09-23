/*
 * arch/arm/mach-tegra/panel-grayson-dsi-rx.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

/*
 * Grayson DSI input driver as a panel driver
 * - Grayson is a Tegra companion chip made by Toshiba also called Brickyard.
 * - Tegra DSI display output signal drives the DSI input of Grayson.
 * - A breakout board P1892 holds the Grayson chip that is plugged to the
 *   E1860 Jetson Pro base board designed for P1859 + VCM3.0-T124 MCM.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/backlight.h>
#include <linux/pwm_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>

#include <mach/dc.h>

#include "board.h"
#include "board-panel.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra11_host1x_devices.h"

#define DSI_PANEL_RESET	0
#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

static bool reg_requested;
static bool gpio_requested;
static struct platform_device *disp_device;


static struct tegra_dc_sd_settings dsi_grayson_sd_settings = {
	.enable = 1, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 5,
	.use_vid_luma = false,
	.phase_in_adjustments = 0,
	.k_limit_enable = true,
	.k_limit = 200,
	.sd_window_enable = false,
	.soft_clipping_enable = true,
	/* Low soft clipping threshold to compensate for aggressive k_limit */
	.soft_clipping_threshold = 128,
	.smooth_k_enable = false,
	.smooth_k_incr = 64,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */

	/* Default LUT */
	.lut = {
			{
				{255, 255, 255},
				{199, 199, 199},
				{153, 153, 153},
				{116, 116, 116},
				{85, 85, 85},
				{59, 59, 59},
				{36, 36, 36},
				{17, 17, 17},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.use_vpulse2 = true,
};

static struct tegra_dsi_cmd dsi_grayson_init_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x01, 0x0),
	DSI_DLY_MS(20),
	DSI_CMD_SHORT(0x15, 0xAE, 0x0B),
	DSI_CMD_SHORT(0x15, 0xEE, 0xEA),
	DSI_CMD_SHORT(0x15, 0xEF, 0x5F),
	DSI_CMD_SHORT(0x15, 0xF2, 0x68),
	DSI_CMD_SHORT(0x15, 0xEE, 0x0),
	DSI_CMD_SHORT(0x15, 0xEF, 0x0),
};

static struct tegra_dsi_cmd dsi_grayson_late_resume_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x10, 0x0),
	DSI_DLY_MS(120),
};

static struct tegra_dsi_cmd dsi_grayson_early_suspend_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x11, 0x0),
	DSI_DLY_MS(160),
};

static struct tegra_dsi_cmd dsi_grayson_suspend_cmd[] = {
	DSI_CMD_SHORT(0x15, 0x11, 0x0),
	DSI_DLY_MS(160),
};

static struct tegra_dsi_out dsi_grayson_pdata = {
	.controller_vs = DSI_VS_1,

	.n_data_lanes = 4,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,

	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.dsi_instance = DSI_INSTANCE_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,

	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,

	.dsi_init_cmd = dsi_grayson_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_grayson_init_cmd),

	.dsi_early_suspend_cmd = dsi_grayson_early_suspend_cmd,
	.n_early_suspend_cmd = ARRAY_SIZE(dsi_grayson_early_suspend_cmd),

	.dsi_late_resume_cmd = dsi_grayson_late_resume_cmd,
	.n_late_resume_cmd = ARRAY_SIZE(dsi_grayson_late_resume_cmd),

	.dsi_suspend_cmd = dsi_grayson_suspend_cmd,
	.n_suspend_cmd = ARRAY_SIZE(dsi_grayson_suspend_cmd),
};

static int tegratab_dsi_regulator_get(struct device *dev)
{
	if (reg_requested)
		return 0;

	reg_requested = true;
	return 0;
}

static int tegratab_dsi_gpio_get(void)
{
	if (gpio_requested)
		return 0;

	gpio_requested = true;
	return 0;
}

static int dsi_grayson_enable(struct device *dev)
{
	int err = 0;

	err = tegratab_dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = tegra_panel_gpio_get_dt("nvidia,grayson-dsi-rx", &panel_of);
	if (err < 0) {
		/* try to request gpios from board file */
		err = tegratab_dsi_gpio_get();
		if (err < 0) {
			pr_err("dsi gpio request failed\n");
			goto fail;
		}
	}

	/* Turning on regulators */
	msleep(20);

#if DSI_PANEL_RESET
/*
 * Nothing is requested.
 */
#endif

	return 0;
fail:
	return err;
}

static int dsi_grayson_disable(struct device *dev)
{
	/* Turning off regulators */

	return 0;
}

static int dsi_grayson_postsuspend(void)
{
	return 0;
}

/*
 * See display standard timings and a few constraints underneath
 * \vendor\nvidia\tegra\core\drivers\hwinc
 *
 * Class: Display Standard Timings
 *
 * Programming of display timing registers must meet these restrictions:
 * Constraint 1: H_REF_TO_SYNC + H_SYNC_WIDTH + H_BACK_PORCH > 11.
 * Constraint 2: V_REF_TO_SYNC + V_SYNC_WIDTH + V_BACK_PORCH > 1.
 * Constraint 3: V_FRONT_PORCH + V_SYNC_WIDTH +
				V_BACK_PORCH > 1 (vertical blank).
 * Constraint 4: V_SYNC_WIDTH >= 1, H_SYNC_WIDTH >= 1
 * Constraint 5: V_REF_TO_SYNC >= 1, H_REF_TO_SYNC >= 0
 * Constraint 6: V_FRONT_PORT >= (V_REF_TO_SYNC + 1),
				H_FRONT_PORT >= (H_REF_TO_SYNC + 1)
 * Constraint 7: H_DISP_ACTIVE >= 16, V_DISP_ACTIVE >= 16
 */

/*
 * how to determine pclk
 * h_total =
 * Horiz_BackPorch + Horiz_SyncWidth + Horiz_DispActive + Horiz_FrontPorch;
 *
 * v_total =
 * Vert_BackPorch + Vert_SyncWidth + Vert_DispActive + Vert_FrontPorch;
 * panel_freq = ( h_total * v_total * refresh_freq );
 */

static struct tegra_dc_mode dsi_grayson_modes[] = {
	{
		.pclk = 71000000, /* 890 *1323 *60 = 70648200 */
		.h_ref_to_sync = 10,
		.v_ref_to_sync = 1,
		.h_sync_width = 1,
		.v_sync_width = 1,
		.h_back_porch = 57,
		.v_back_porch = 14,
		.h_active = 800,
		.v_active = 1280,
		.h_front_porch = 32,
		.v_front_porch = 28,
	},
};


static void dsi_grayson_set_disp_device(
	struct platform_device *display_device)
{
	disp_device = display_device;
}

static void dsi_grayson_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_grayson_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_grayson_modes;
	dc->n_modes = ARRAY_SIZE(dsi_grayson_modes);
	dc->enable = dsi_grayson_enable;
	dc->disable = dsi_grayson_disable;
	dc->postsuspend	= dsi_grayson_postsuspend,
	dc->width = 94;
	dc->height = 150;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_grayson_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_grayson_modes[0].h_active;
	fb->yres = dsi_grayson_modes[0].v_active;
}

static void
dsi_grayson_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	*settings = dsi_grayson_sd_settings;
}

#ifdef CONFIG_TEGRA_DC_CMU
static void dsi_grayson_cmu_init(struct tegra_dc_platform_data *pdata)
{
	pdata->cmu = NULL; /* will write CMU stuff after calibration */
}
#endif


struct tegra_panel_ops dsi_grayson_dsi_rx_ops = {
	.enable = dsi_grayson_enable,
	.disable = dsi_grayson_disable,
	.postsuspend = dsi_grayson_postsuspend,
};

struct tegra_panel __initdata dsi_grayson_dsi_rx = {
	.init_sd_settings = dsi_grayson_sd_settings_init,
	.init_dc_out = dsi_grayson_dc_out_init,
	.init_fb_data = dsi_grayson_fb_data_init,
/*
	.register_i2c_bridge = ,
*/
#ifdef CONFIG_TEGRA_DC_CMU
	.init_cmu_data = dsi_grayson_cmu_init,
#endif
	.set_disp_device = dsi_grayson_set_disp_device,
};
EXPORT_SYMBOL(dsi_grayson_dsi_rx);
