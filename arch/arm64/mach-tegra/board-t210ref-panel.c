/*
 * arch/arm64/mach-tegra/board-t210ref-panel.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dma-contiguous.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/tegra-soc.h>

#include <mach/irqs.h>
#include <mach/dc.h>
#include <mach/io_dpd.h>

#include "board.h"
#include "tegra-board-id.h"
#include "devices.h"
#include "gpio-names.h"
#include "board-t210ref.h"
#include "board-panel.h"
#include "common.h"
#include "iomap.h"
#include "tegra12_host1x_devices.h"
#include "dvfs.h"

struct platform_device * __init t210ref_host1x_init(void)
{
	struct platform_device *pdev = NULL;

#ifdef CONFIG_TEGRA_GRHOST
	if (!of_have_populated_dt())
		pdev = tegra12_register_host1x_devices();
	else
		pdev = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "host1x"));

	if (!pdev) {
		pr_err("host1x devices registration failed\n");
		return NULL;
	}
#endif
	return pdev;
}

/* hdmi related regulators */
static struct regulator *t210ref_hdmi_reg;
static struct regulator *t210ref_hdmi_pll;
static struct regulator *t210ref_hdmi_vddio;

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct resource t210ref_disp1_resources[] = {
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
		.start	= 0, /* Filled in by t210ref_panel_init() */
		.end	= 0, /* Filled in by t210ref_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsia_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsib_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "mipi_cal",
		.start	= TEGRA_MIPI_CAL_BASE,
		.end	= TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

static struct resource t210ref_disp2_resources[] = {
	{
		.name	= "irq",
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
#else
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
#endif
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
#else
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
#endif
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by t210ref_panel_init() */
		.end	= 0, /* Filled in by t210ref_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sor0",
		.start  = TEGRA_SOR_BASE,
		.end    = TEGRA_SOR_BASE + TEGRA_SOR_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "sor1",
		.start  = TEGRA_SOR1_BASE,
		.end    = TEGRA_SOR1_BASE + TEGRA_SOR1_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "dpaux",
		.start  = TEGRA_DPAUX_BASE,
		.end    = TEGRA_DPAUX_BASE + TEGRA_DPAUX_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name	= "irq_dp",
		.start	= INT_DPAUX,
		.end	= INT_DPAUX,
		.flags	= IORESOURCE_IRQ,
	},
};


#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct tegra_dc_sd_settings sd_settings;

static struct tegra_dc_out t210ref_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
	.sd_settings	= &sd_settings,
};
#endif

static int t210ref_hdmi_enable(struct device *dev)
{
	int ret;
	if (!t210ref_hdmi_reg) {
		t210ref_hdmi_reg = regulator_get(dev, "avdd_hdmi");
		if (IS_ERR(t210ref_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			t210ref_hdmi_reg = NULL;
			return PTR_ERR(t210ref_hdmi_reg);
		}
	}
	ret = regulator_enable(t210ref_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!t210ref_hdmi_pll) {
		t210ref_hdmi_pll = regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR(t210ref_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			t210ref_hdmi_pll = NULL;
			regulator_put(t210ref_hdmi_reg);
			t210ref_hdmi_reg = NULL;
			return PTR_ERR(t210ref_hdmi_pll);
		}
	}
	ret = regulator_enable(t210ref_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int t210ref_hdmi_disable(void)
{
	if (t210ref_hdmi_reg) {
		regulator_disable(t210ref_hdmi_reg);
		regulator_put(t210ref_hdmi_reg);
		t210ref_hdmi_reg = NULL;
	}

	if (t210ref_hdmi_pll) {
		regulator_disable(t210ref_hdmi_pll);
		regulator_put(t210ref_hdmi_pll);
		t210ref_hdmi_pll = NULL;
	}
	return 0;
}

static int t210ref_hdmi_postsuspend(void)
{
	if (t210ref_hdmi_vddio) {
		regulator_disable(t210ref_hdmi_vddio);
		regulator_put(t210ref_hdmi_vddio);
		t210ref_hdmi_vddio = NULL;
	}
	return 0;
}

static int t210ref_hdmi_hotplug_init(struct device *dev)
{
	if (!t210ref_hdmi_vddio) {
		t210ref_hdmi_vddio = regulator_get(dev, "vdd_hdmi_5v0");
		if (WARN_ON(IS_ERR(t210ref_hdmi_vddio))) {
			pr_err("%s: couldn't get regulator vdd_hdmi_5v0: %ld\n",
				__func__, PTR_ERR(t210ref_hdmi_vddio));
				t210ref_hdmi_vddio = NULL;
		} else {
			return regulator_enable(t210ref_hdmi_vddio);
		}
	}

	return 0;
}

struct tmds_config t210ref_tmds_config_fpga[] = {
	{ /* 480p/576p / 25.2MHz/27MHz modes */
	.pclk = 27000000,
	.pll0 = 0x01003110,
	.pll1 = 0x00300F00,
	.pe_current = 0x00000000,
	.drive_current = 0xffffffff,
	.peak_current = 0x00000000,
	},
	{ /* 720p / 74.25MHz modes */
	.pclk = 74250000,
	.pll0 =  0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x20202020,
	.peak_current = 0x00000000,
	},
	{ /* 1080p / 148.5MHz modes */
	.pclk = 148500000,
	.pll0 = 0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x20202020,
	.peak_current = 0x00000000,
	},
	{
	.pclk = INT_MAX,
	.pll0 = 0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x3A353536, /* lane3 needs a slightly lower current */
	.peak_current = 0x00000000,
	},
};

struct tmds_config t210ref_tmds_config0[] = {
	{ /* 480p/576p / 25.2MHz/27MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 27000000,
	.pll0 = 0x01003010,
	.pll1 = 0x00301B00,
	.pe_current = 0x00000000,
	.drive_current = 0x1F1F1F1F,
	.peak_current = 0x03030303,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{ /* 720p / 74.25MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 74250000,
	.pll0 = 0x01003110,
	.pll1 = 0x00301500,
	.pe_current = 0x00000000,
	.drive_current = 0x2C2C2C2C,
	.peak_current = 0x07070707,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{ /* 1080p / 148.5MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 148500000,
	.pll0 = 0x01003310,
	.pll1 = 0x00301500,
	.pe_current = 0x00000000,
	.drive_current = 0x33333333,
	.peak_current = 0x0C0C0C0C,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{
	.version = MKDEV(1, 0),
	.pclk = INT_MAX,
	.pll0 = 0x01003F10,
	.pll1 = 0x00300F00,
	.pe_current = 0x00000000,
	.drive_current = 0x37373737, /* lane3 needs a slightly lower current */
	.peak_current = 0x17171717,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000600, /* BG_VREF_LEVEL */
	},
};

struct tmds_config t210ref_tmds_config1[] = {
	{ /* 480p/576p / 25.2MHz/27MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 27000000,
	.pll0 = 0x01003010,
	.pll1 = 0x00301b00,
	.pe_current    = 0x00000000,
	.drive_current = 0x1C1C1C1C,
	.peak_current  = 0x00000000,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{ /* 720p / 74.25MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 74250000,
	.pll0 = 0x01003110,
	.pll1 = 0x00301500,
	.pe_current    = 0x00000000,
	.drive_current = 0x23232323,
	.peak_current  = 0x00000000,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{ /* 1080p / 148.5MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 148500000,
	.pll0 = 0x01003310,
	.pll1 = 0x10300F00,
	.pe_current    = 0x00000000,
	.drive_current = 0x2B2C2D2B,
	.peak_current  = 0x00000000,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{
	.version = MKDEV(1, 0),
	.pclk = INT_MAX,
	.pll0 = 0x01003F10,
	.pll1 = 0x10300700,
	.pe_current    = 0x00000000,
	.drive_current = 0x32323131,
	.peak_current  = 0x10101010,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000600, /* BG_VREF_LEVEL */
	},
};

/* Equivalent to T132 Bowmore */
struct tmds_config t210ref_tmds_config2[] = {
	{ /* 480p/576p / 25.2MHz/27MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 27000000,
	.pll0 = 0x01003010,
	.pll1 = 0x00301B00,
	.pe_current = 0x00000000,
	.drive_current = 0x1C1C1C1C,
	.peak_current = 0x00000000,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{ /* 720p / 74.25MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 74250000,
	.pll0 = 0x01003110,
	.pll1 = 0x00301500,
	.pe_current = 0x00000000,
	.drive_current = 0x22232323,
	.peak_current = 0x00000000,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{ /* 1080p / 148.5MHz modes */
	.version = MKDEV(1, 0),
	.pclk = 148500000,
	.pll0 = 0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x00000000,
	.drive_current = 0x2A2C2C2A,
	.peak_current = 0x00000000,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000400, /* BG_VREF_LEVEL */
	},
	{
	.version = MKDEV(1, 0),
	.pclk = INT_MAX,
	.pll0 = 0x01003F10,
	.pll1 = 0x10300700,
	.pe_current = 0x00000000,
	.drive_current = 0x30323333,
	.peak_current = 0x10101010,
	.pad_ctls0_mask    = 0xfffff0ff,
	.pad_ctls0_setting = 0x00000600, /* BG_VREF_LEVEL */
	},
};

struct tegra_hdmi_out t210ref_hdmi_out = {
	.tmds_config = t210ref_tmds_config2,
	.n_tmds_config = ARRAY_SIZE(t210ref_tmds_config2),
};

static struct tegra_dc_mode t210ref_hdmi_modes[] = {
	{
		.pclk =			27000000,
		.h_ref_to_sync =	1,
		.v_ref_to_sync =	1,
		.h_sync_width =		62,	/* hsync_len */
		.v_sync_width =		6,	/* vsync_len */
		.h_back_porch =		60,	/* left_margin */
		.v_back_porch =		30,	/* upper_margin */
		.h_active =		720,	/* xres */
		.v_active =		480,	/* yres */
		.h_front_porch =	16,	/* right_margin */
		.v_front_porch =	9,	/* lower_margin */
	},
};
#ifdef CONFIG_TEGRA_HDMI2_0
static struct tegra_dc_out_pin t210ref_hdmi_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name   = TEGRA_DC_OUT_PIN_DATA_ENABLE,
		.pol    = TEGRA_DC_OUT_PIN_POL_HIGH,
	},
};
#endif
static struct tegra_dc_out t210ref_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_LOW,
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	.parent_clk	= "pll_d2",
#else
	.parent_clk	= "pll_d",
#endif

	.ddc_bus	= 3,
	.hotplug_gpio	= t210ref_hdmi_hpd,
	.hdmi_out	= &t210ref_hdmi_out,

	/* TODO: update max pclk to POR */
	.max_pixclock	= KHZ2PICOS(297000),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
#ifdef CONFIG_TEGRA_HDMI2_0
	.out_pins		= t210ref_hdmi_out_pins,
	.n_out_pins	= ARRAY_SIZE(t210ref_hdmi_out_pins),
#endif
	.enable		= t210ref_hdmi_enable,
	.disable	= t210ref_hdmi_disable,
	.postsuspend	= t210ref_hdmi_postsuspend,
	.hotplug_init	= t210ref_hdmi_hotplug_init,
};

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct tegra_fb_data t210ref_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data t210ref_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &t210ref_disp1_out,
	.fb		= &t210ref_disp1_fb_data,
	.emc_clk_rate	= 204000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 1,
#endif
	.low_v_win	= 0x02,
};
#endif

static struct tegra_fb_data t210ref_disp2_fb_data = {
	.win		= 0,
	.xres		= 1920,
	.yres		= 1080,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data t210ref_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &t210ref_disp2_out,
	.fb		= &t210ref_disp2_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct platform_device t210ref_disp2_device = {
	.name		= "tegradc",
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	.id		= 1,
#else
	.id		= 0,
#endif
	.resource	= t210ref_disp2_resources,
	.num_resources	= ARRAY_SIZE(t210ref_disp2_resources),
	.dev = {
		.platform_data = &t210ref_disp2_pdata,
	},
};

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct platform_device t210ref_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= t210ref_disp1_resources,
	.num_resources	= ARRAY_SIZE(t210ref_disp1_resources),
	.dev = {
		.platform_data = &t210ref_disp1_pdata,
	},
};
#endif

static struct tegra_io_dpd dsic_io = {
	.name			= "DSIC",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 8,
};
static struct tegra_io_dpd dsid_io = {
	.name			= "DSID",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 9,
};

static struct tegra_dc_dp_lt_settings t210ref_edp_lt_data[] = {
	{
		.drive_current = {
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
		},
		.lane_preemphasis = {
			PRE_EMPHASIS_L0,
			PRE_EMPHASIS_L0,
			PRE_EMPHASIS_L0,
			PRE_EMPHASIS_L0,
		},
		.post_cursor = {
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
		},
		.tx_pu = 0,
		.load_adj = 0x3,
	},
	{
		.drive_current = {
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
		},
		.lane_preemphasis = {
			PRE_EMPHASIS_L0,
			PRE_EMPHASIS_L0,
			PRE_EMPHASIS_L0,
			PRE_EMPHASIS_L0,
		},
		.post_cursor = {
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
		},
		.tx_pu = 0,
		.load_adj = 0x4,
	},
	{
		.drive_current = {
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
			DRIVE_CURRENT_L0,
		},
		.lane_preemphasis = {
			PRE_EMPHASIS_L1,
			PRE_EMPHASIS_L1,
			PRE_EMPHASIS_L1,
			PRE_EMPHASIS_L1,
		},
		.post_cursor = {
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
			POST_CURSOR2_L0,
		},
		.tx_pu = 0,
		.load_adj = 0x6,
	},
};

static struct tegra_dp_out dp_settings = {
	/* Panel can override this with its own LT data */
	.lt_settings = t210ref_edp_lt_data,
	.n_lt_settings = ARRAY_SIZE(t210ref_edp_lt_data),
	.tx_pu_disable = true,
};
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
/* can be called multiple times */
static __init struct tegra_panel *t210ref_panel_configure(
				struct board_info *board_out,
				u8 *dsi_instance_out)
{
	struct tegra_panel *panel = NULL;
	u8 dsi_instance = DSI_INSTANCE_0;
	struct board_info boardtmp;

	if (!board_out)
		board_out = &boardtmp;
	tegra_get_display_board_info(board_out);

	switch (board_out->board_id) {
	case BOARD_E1824:
		panel = &edp_a_1080p_14_0;
		t210ref_disp1_out.type = TEGRA_DC_OUT_DP;
		t210ref_disp1_out.dp_out = &dp_settings;
		break;
	case BOARD_E2129:
		switch (board_out->sku) {
		case 1000:
			panel = &dsi_j_1440_810_5_8;
			dsi_instance = DSI_INSTANCE_0;
			break;
		default:
			panel = &dsi_a_1200_1920_8_0;
			dsi_instance = DSI_INSTANCE_0;
			break;
		}
		tegra_io_dpd_enable(&dsic_io);
		tegra_io_dpd_enable(&dsid_io);
		break;
	case BOARD_E1937:
		panel = &dsi_a_1200_1920_8_0;
		dsi_instance = DSI_INSTANCE_0;
		tegra_io_dpd_enable(&dsic_io);
		tegra_io_dpd_enable(&dsid_io);
		break;
	case BOARD_E1639: /* fall through */
	case BOARD_E1813: /* fall through */
	default:
		if (tegra_platform_is_fpga()) {
#ifdef CONFIG_TEGRA_DP
			/* Fpga eDP is on disp2. Clear all default hdmi data.*/
			memset(&t210ref_disp2_out, 0,
				sizeof(t210ref_disp2_out));
			t210ref_disp2_out.dp_out = &dp_settings;
			edp_c_1366_768_14_0.init_dc_out(&t210ref_disp2_out);
#elif defined(CONFIG_TEGRA_HDMI2_0)
			t210ref_disp2_fb_data.xres = 720;
			t210ref_disp2_fb_data.yres = 480;
			t210ref_disp2_fb_data.bits_per_pixel = -1;
#endif
			panel = &dsi_s_wqxga_10_1;
		} else {
			panel = &dsi_j_1440_810_5_8;
			dsi_instance = DSI_INSTANCE_0;
			tegra_io_dpd_enable(&dsic_io);
			tegra_io_dpd_enable(&dsid_io);
			break;
		}
	}

	if (dsi_instance_out)
		*dsi_instance_out = dsi_instance;
	return panel;
}

static __init void t210ref_panel_select(void)
{
	struct tegra_panel *panel = NULL;
	struct board_info board;
	u8 dsi_instance;

	panel = t210ref_panel_configure(&board, &dsi_instance);

	if (panel) {
		if (panel->init_sd_settings)
			panel->init_sd_settings(&sd_settings);

		if (panel->init_dc_out) {
			panel->init_dc_out(&t210ref_disp1_out);
			if (t210ref_disp1_out.type == TEGRA_DC_OUT_DSI) {
				t210ref_disp1_out.dsi->dsi_instance =
					dsi_instance;
				t210ref_disp1_out.dsi->dsi_panel_rst_gpio =
					DSI_PANEL_RST_GPIO;
				t210ref_disp1_out.dsi->dsi_panel_bl_pwm_gpio =
					DSI_PANEL_BL_PWM_GPIO;
				t210ref_disp1_out.dsi->te_gpio = TEGRA_GPIO_PR6;
			}
		}

		if (panel->init_fb_data)
			panel->init_fb_data(&t210ref_disp1_fb_data);

		if (panel->init_cmu_data)
			panel->init_cmu_data(&t210ref_disp1_pdata);

		if (panel->set_disp_device)
			panel->set_disp_device(&t210ref_disp1_device);

		if (t210ref_disp1_out.type == TEGRA_DC_OUT_DSI) {
			tegra_dsi_resources_init(dsi_instance,
				t210ref_disp1_resources,
				ARRAY_SIZE(t210ref_disp1_resources));
		}

		if (panel->register_bl_dev)
			panel->register_bl_dev();

		if (panel->register_i2c_bridge)
			panel->register_i2c_bridge();
	}
}
#endif

static __init int t210ref_fb_init(void)
{
	struct resource *res;

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	res = platform_get_resource_byname(&t210ref_disp1_device,
					 IORESOURCE_MEM, "fbmem");
#else
	res = platform_get_resource_byname(&t210ref_disp2_device,
					 IORESOURCE_MEM, "fbmem");
#endif
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	res = platform_get_resource_byname(&t210ref_disp2_device,
					IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
#endif
	return 0;
}

static __init int t210ref_fb_copy(void)
{
	/* Copy the bootloader fb to the fb. */
	if (tegra_bootloader_fb_size)
		__tegra_move_framebuffer(NULL,
					tegra_fb_start,
					tegra_bootloader_fb_start,
					min(tegra_fb_size,
					tegra_bootloader_fb_size));
	else
		__tegra_clear_framebuffer(NULL,
					  tegra_fb_start,
					  tegra_fb_size);

	/* Copy the bootloader fb2 to the fb2. */
	if (tegra_bootloader_fb2_size)
		__tegra_move_framebuffer(NULL,
					tegra_fb2_start,
					tegra_bootloader_fb2_start,
					min(tegra_fb2_size,
					tegra_bootloader_fb2_size));
	else
		__tegra_clear_framebuffer(NULL,
					tegra_fb2_start,
					tegra_fb2_size);
	return 0;
}

int __init t210ref_panel_init(void)
{
	int err = 0;
	struct platform_device *phost1x = NULL;
	struct device_node *dc1_node = NULL;
	struct device_node *dc2_node = NULL;
	struct board_info board_info;

	find_dc_node(&dc1_node, &dc2_node);

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	t210ref_panel_select();
#endif

	phost1x = t210ref_host1x_init();
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}

	t210ref_fb_init();
	t210ref_fb_copy();

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	if (!of_have_populated_dt() || !dc1_node ||
		!of_device_is_available(dc1_node)) {
		t210ref_disp1_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&t210ref_disp1_device);
		if (err) {
			pr_err("disp1 device registration failed\n");
			return err;
		}
	}
#endif
	tegra_get_board_info(&board_info);

	switch (board_info.board_id) {
	case BOARD_E2141:
		t210ref_disp2_out.hotplug_gpio = TEGRA_GPIO_PN7;
		break;
	default:
		break;
	}

	if (tegra_platform_is_fpga()) {
		struct tegra_dc_platform_data *pdata =
			t210ref_disp2_device.dev.platform_data;
		pdata->default_out->depth = 24;
		pdata->default_out->hotplug_state = 1;
		pdata->default_out->hdmi_out->tmds_config =
				t210ref_tmds_config_fpga;
		pdata->default_out->hdmi_out->n_tmds_config =
				ARRAY_SIZE(t210ref_tmds_config_fpga);
		pdata->default_out->modes = t210ref_hdmi_modes;
		pdata->default_out->n_modes = ARRAY_SIZE(t210ref_hdmi_modes);
	}

	if (!of_have_populated_dt() || !dc2_node ||
		!of_device_is_available(dc2_node)) {
		t210ref_disp2_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&t210ref_disp2_device);
		if (err) {
			pr_err("disp2 device registration failed\n");
			return err;
		}
	}

	return err;
}

int __init t210ref_display_init(void)
{
	struct clk *disp1_clk = clk_get_sys("tegradc.0", NULL);
	struct clk *disp2_clk = clk_get_sys("tegradc.1", NULL);
	struct tegra_panel *panel;
	struct board_info board;
	long disp1_rate = 0;
	long disp2_rate = 0;

	/*
	 * TODO
	 * Need to skip t210ref_display_init
	 * when disp is registered by device_tree
	 */

	if (WARN_ON(IS_ERR(disp1_clk))) {
		if (disp2_clk && !IS_ERR(disp2_clk))
			clk_put(disp2_clk);
		return PTR_ERR(disp1_clk);
	}

	if (WARN_ON(IS_ERR(disp2_clk))) {
		clk_put(disp1_clk);
		return PTR_ERR(disp1_clk);
	}

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	panel = t210ref_panel_configure(&board, NULL);

	if (panel && panel->init_dc_out) {
		panel->init_dc_out(&t210ref_disp1_out);
		if (t210ref_disp1_out.n_modes && t210ref_disp1_out.modes)
			disp1_rate = t210ref_disp1_out.modes[0].pclk;
	} else {
		disp1_rate = 0;
		if (!panel || !panel->init_dc_out)
			printk(KERN_ERR "disp1 panel output not specified!\n");
	}

	printk(KERN_DEBUG "disp1 pclk=%ld\n", disp1_rate);
	if (disp1_rate)
		tegra_dvfs_resolve_override(disp1_clk, disp1_rate);
#endif

	/* set up disp2 */
	if (t210ref_disp2_out.max_pixclock)
		disp2_rate = PICOS2KHZ(t210ref_disp2_out.max_pixclock) * 1000;
	else
		disp2_rate = 297000000; /* HDMI 4K */
	printk(KERN_DEBUG "disp2 pclk=%ld\n", disp2_rate);
	if (disp2_rate)
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		tegra_dvfs_resolve_override(disp2_clk, disp2_rate);
#else
		tegra_dvfs_resolve_override(disp1_clk, disp2_rate);
#endif

	clk_put(disp1_clk);
	clk_put(disp2_clk);
	return 0;
}
