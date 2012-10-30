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
#include <linux/pwm_backlight.h>

#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"

#include "tegra11_host1x_devices.h"

int __init pluto_host1x_init(void)
{
	int err = -EINVAL;

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra11_register_host1x_devices();
	if (err) {
		pr_err("host1x devices registration failed\n");
		return err;
	}
#endif
	return err;
}

#ifdef CONFIG_TEGRA_DC

/* PANEL_<diagonal length in inches>_<vendor name>_<resolution> */
#define PANEL_5_LG_720_1280	1
#define PANEL_4_7_JDI_720_1280	0
#define PANEL_5_SHARP_1080p	0

#if PANEL_5_LG_720_1280 || PANEL_5_SHARP_1080p
#define DSI_PANEL_RESET		1
#else
#define DSI_PANEL_RESET		0
#endif

#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PH5
#define DSI_PANEL_BL_EN_GPIO	TEGRA_GPIO_PH2
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1


#if PANEL_4_7_JDI_720_1280
#define DC_CTRL_MODE	(TEGRA_DC_OUT_ONE_SHOT_MODE | \
			 TEGRA_DC_OUT_ONE_SHOT_LP_MODE)
#else
#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE
#endif

static atomic_t __maybe_unused sd_brightness = ATOMIC_INIT(255);

static bool dsi_reg_requested;
static bool dsi_gpio_requested;

/*
 * for PANEL_5_LG_720_1280, PANEL_4_7_JDI_720_1280
 * and PANEL_5_SHARP_1080p
 */
static struct regulator *vdd_lcd_s_1v8;
static struct regulator *vdd_sys_bl_3v7;

/* for PANEL_5_LG_720_1280 and PANEL_4_7_JDI_720_1280 */
static struct regulator *avdd_lcd_3v0_2v8;

/* for PANEL_5_LG_720_1280 and PANEL_5_SHARP_1080p */
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
#if PANEL_5_LG_720_1280
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSI_BASE,
		.end	= TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#elif PANEL_4_7_JDI_720_1280 || PANEL_5_SHARP_1080p
	{
		.name	= "dsi_regs",
		.start	= TEGRA_DSIB_BASE,
		.end	= TEGRA_DSIB_BASE + TEGRA_DSIB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
	{
		.name	= "mipi_cal",
		.start	= TEGRA_MIPI_CAL_BASE,
		.end	= TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
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

#if PANEL_5_SHARP_1080p
static u8 panel_internal[] = {0x51, 0x0f, 0xff};
#endif

#if PANEL_4_7_JDI_720_1280
static tegra_dc_bl_output pluto_bl_output_measured = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 18, 19, 20, 21, 22, 23, 25,
	26, 27, 28, 30, 31, 32, 33, 35,
	36, 38, 39, 41, 42, 43, 45, 46,
	48, 49, 51, 52, 54, 55, 57, 58,
	60, 61, 63, 64, 66, 67, 68, 70,
	71, 72, 74, 75, 77, 78, 79, 80,
	81, 82, 83, 85, 86, 87, 88, 89,
	90, 90, 91, 92, 93, 93, 94, 95,
	96, 96, 96, 97, 97, 97, 97, 98,
	98, 98, 98, 99, 100, 101, 101, 102,
	103, 104, 104, 105, 106, 107, 108, 109,
	110, 112, 113, 114, 115, 116, 117, 119,
	120, 121, 122, 123, 125, 126, 127, 128,
	129, 131, 132, 133, 134, 135, 136, 137,
	138, 140, 141, 142, 142, 143, 144, 145,
	146, 147, 148, 149, 149, 150, 151, 152,
	153, 154, 154, 155, 156, 157, 158, 159,
	160, 162, 163, 164, 165, 167, 168, 169,
	170, 171, 172, 173, 173, 174, 175, 176,
	176, 177, 178, 179, 179, 180, 181, 182,
	182, 183, 184, 184, 185, 186, 186, 187,
	188, 188, 189, 189, 190, 190, 191, 192,
	193, 194, 195, 195, 196, 197, 198, 199,
	200, 201, 202, 203, 203, 204, 205, 206,
	207, 208, 209, 210, 211, 212, 213, 213,
	214, 215, 216, 217, 218, 219, 220, 221,
	222, 224, 225, 226, 227, 228, 229, 230,
	231, 232, 233, 234, 235, 236, 237, 238,
	239, 240, 241, 242, 243, 244, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};
#elif PANEL_5_LG_720_1280
static tegra_dc_bl_output pluto_bl_output_measured = {
	0, 1, 3, 5, 7, 9, 11, 12,
	14, 15, 16, 18, 19, 21, 22, 24,
	25, 26, 27, 28, 29, 30, 31, 32,
	33, 34, 35, 36, 38, 39, 40, 41,
	42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 51, 52, 52, 53, 54, 55,
	56, 56, 57, 58, 59, 60, 61, 62,
	63, 64, 65, 66, 67, 68, 69, 70,
	71, 72, 73, 74, 75, 76, 76, 77,
	78, 79, 80, 81, 81, 82, 83, 83,
	84, 85, 85, 86, 87, 88, 89, 90,
	91, 92, 93, 94, 95, 96, 96, 97,
	98, 99, 100, 101, 102, 103, 103, 104,
	104, 105, 106, 107, 108, 109, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 124, 125, 127
};
#elif PANEL_5_SHARP_1080p
static tegra_dc_bl_output pluto_bl_output_measured = {
	/* TODO */
};
#endif

static p_tegra_dc_bl_output bl_output = pluto_bl_output_measured;

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

	/* panel power control 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc2, 0x02),
	DSI_DLY_MS(20),

	/* panel power control 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc2, 0x06),
	DSI_DLY_MS(20),

	/* panel power control 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xc2, 0x4e),
	DSI_DLY_MS(100),

	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
	DSI_DLY_MS(20),

	/* panel OTP 2 */
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xf9, 0x80),
	DSI_DLY_MS(20),

	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
#endif

#if PANEL_4_7_JDI_720_1280
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFF, 0xEE),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x26, 0x08),
	DSI_DLY_MS(10),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x26, 0x00),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFF, 0x00),
	DSI_DLY_MS(15),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 1),
	DSI_DLY_MS(10),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 0),
	DSI_DLY_MS(20),
	DSI_GPIO_SET(DSI_PANEL_RST_GPIO, 1),
	DSI_DLY_MS(100),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xBA, 0x02),
	DSI_DLY_MS(5),
#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xC2, 0x08),
#else
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xC2, 0x03),
#endif
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFF, 0x04),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x09, 0x00),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x0A, 0x00),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFB, 0x01),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFF, 0xEE),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x12, 0x53),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x13, 0x05),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x6A, 0x60),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFB, 0x01),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0xFF, 0x00),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, 0x3A, 0x77),
	DSI_DLY_MS(5),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x00),
	DSI_DLY_MS(2000),
#if (DC_CTRL_MODE & TEGRA_DC_OUT_ONE_SHOT_MODE)
	DSI_CMD_SHORT(DSI_DCS_WRITE_1_PARAM, DSI_DCS_SET_TEARING_EFFECT_ON, 0),
#endif
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x00),
	DSI_DLY_MS(150),
#endif

#if PANEL_5_SHARP_1080p
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xb0, 0x04),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_NO_OP, 0x0),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_NO_OP, 0x0),
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0xd6, 0x01),
	DSI_CMD_LONG(DSI_GENERIC_LONG_WRITE, panel_internal),
	DSI_CMD_SHORT(DSI_GENERIC_SHORT_WRITE_2_PARAMS, 0x53, 0x04),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_SET_DISPLAY_ON, 0x0),
	DSI_CMD_SHORT(DSI_DCS_WRITE_0_PARAM, DSI_DCS_EXIT_SLEEP_MODE, 0x0),
#endif
};

static struct tegra_dsi_out pluto_dsi = {
#if PANEL_4_7_JDI_720_1280
	.n_data_lanes = 3,
	.dsi_instance = DSI_INSTANCE_1,
	.rated_refresh_rate = 60,
	.refresh_rate = 60,
	.suspend_aggr = DSI_HOST_SUSPEND_LV2,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_COMMAND_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
#else
	.n_data_lanes = 4,
#if	PANEL_5_SHARP_1080p
	.dsi_instance = DSI_INSTANCE_1,
#else
	.dsi_instance = DSI_INSTANCE_0,
#endif
	.refresh_rate = 60,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,
#endif
	.controller_vs = DSI_VS_1,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.dsi_init_cmd = dsi_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
};

static int pluto_dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (dsi_reg_requested)
		return 0;

#if PANEL_5_LG_720_1280 || PANEL_5_SHARP_1080p
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
#endif

#if PANEL_5_LG_720_1280 || PANEL_4_7_JDI_720_1280 || PANEL_5_SHARP_1080p
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

#if PANEL_5_LG_720_1280
	err = gpio_request(DSI_PANEL_BL_PWM, "panel pwm");
	if (err < 0) {
		pr_err("panel backlight pwm gpio request failed\n");
		goto fail;
	}
#endif

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

	gpio_direction_output(DSI_PANEL_RST_GPIO, 0);

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
	usleep_range(3000, 5000);

	if (avdd_ts_3v0) {
		err = regulator_enable(avdd_ts_3v0);
		if (err < 0) {
			pr_err("avdd_ts_3v0 regulator enable failed\n");
			goto fail;
		}
	}
	usleep_range(3000, 5000);

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
	usleep_range(3000, 5000);

#if DSI_PANEL_RESET
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
#if !PANEL_5_SHARP_1080p
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 0);
	usleep_range(1000, 5000);
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
#endif
	msleep(20);
#endif

	gpio_direction_output(DSI_PANEL_BL_EN_GPIO, 1);

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
		.pclk = 62625000,
		.h_ref_to_sync = 2,
		.v_ref_to_sync = 1,
		.h_sync_width = 2,
		.v_sync_width = 2,
		.h_back_porch = 84,
		.v_back_porch = 2,
		.h_active = 720,
		.v_active = 1280,
		.h_front_porch = 4,
		.v_front_porch = 4,
	},
#endif
#if PANEL_5_SHARP_1080p
	{
		.pclk = 10000000,
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
#endif
};

static struct tegra_dc_sd_settings sd_settings;

static struct tegra_dc_out pluto_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
	.dsi		= &pluto_dsi,
	.sd_settings	= &sd_settings,

	.flags		= DC_CTRL_MODE,
#if PANEL_4_7_JDI_720_1280 || PANEL_5_SHARP_1080p
	.parent_clk	= "pll_d_out0",
#endif

	.modes		= pluto_dsi_modes,
	.n_modes	= ARRAY_SIZE(pluto_dsi_modes),

	.enable		= pluto_dsi_panel_enable,
	.disable	= pluto_dsi_panel_disable,
	.postsuspend	= pluto_dsi_panel_postsuspend,

#if PANEL_5_LG_720_1280 || PANEL_5_SHARP_1080p
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
#if PANEL_5_SHARP_1080p
	.xres		= 1080,
	.yres		= 1920,
#endif
};

static struct tegra_dc_platform_data pluto_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &pluto_disp1_out,
	.fb		= &pluto_disp1_fb_data,
	.emc_clk_rate	= 204000000,

	.cmu_enable	= 1,
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

static int __maybe_unused pluto_disp1_bl_notify(struct device *unused,
						int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = bl_output[brightness];

	return brightness;
}

static int __maybe_unused pluto_disp1_check_fb(struct device *dev,
					     struct fb_info *info)
{
	return info->device == &pluto_disp1_device.dev;
}

#if PANEL_4_7_JDI_720_1280
static struct platform_pwm_backlight_data pluto_disp1_bl_data = {
	.pwm_id         = 1,
	.max_brightness = 255,
	.dft_brightness = 77,
	.pwm_period_ns  = 40000,
	.notify         = pluto_disp1_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb       = pluto_disp1_check_fb,
};

static struct platform_device pluto_disp1_bl_device = {
	.name   = "pwm-backlight",
	.id     = -1,
	.dev    = {
		.platform_data = &pluto_disp1_bl_data,
	},
};
#endif

static struct tegra_dc_sd_settings pluto_sd_settings = {
	.enable = 1, /* enabled by default */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 5,
	.use_vid_luma = false,
	.phase_in_adjustments = 1,
	.k_limit_enable = true,
	/* Aggressive k_limit */
	.k_limit = 180,
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
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 73, 82},
				{92, 103, 114, 125},
				{138, 150, 164, 178},
				{193, 208, 224, 241},
			},
		},
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
	.bl_device_name = "pwm-backlight",
};

int __init pluto_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;

	sd_settings = pluto_sd_settings;

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

	err = pluto_host1x_init();
	if (err)
		return err;

	res = nvhost_get_resource_byname(&pluto_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
			min(tegra_fb_size, tegra_bootloader_fb_size));

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

#if PANEL_4_7_JDI_720_1280
	err = platform_device_register(&tegra_pwfm1_device);
	if (err) {
		pr_err("disp1 pwm device registration failed");
		return err;
	}

	err = platform_device_register(&pluto_disp1_bl_device);
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}

	err = gpio_request(DSI_PANEL_BL_PWM, "panel pwm");
	if (err < 0) {
		pr_err("panel backlight pwm gpio request failed\n");
		return err;
	}
	gpio_free(DSI_PANEL_BL_PWM);
#endif

#ifdef CONFIG_TEGRA_NVAVP
	err = nvhost_device_register(&nvavp_device);
	if (err) {
		pr_err("nvavp device registration failed\n");
		return err;
	}
#endif
	return err;
}
#else
int __init pluto_panel_init(void)
{
	return pluto_host1x_init();
}
#endif
