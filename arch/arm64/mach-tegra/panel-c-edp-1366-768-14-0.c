/*
 * arch/arm64/mach-tegra/panel-c-edp-1366-768-14-0.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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
#include "board-panel.h"

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

static struct tegra_dc_mode panel_edp_modes[] = {
	{
		.pclk = 27000000,
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 34,
		.v_sync_width = 6,
		.h_back_porch = 64,
		.v_back_porch = 4,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 16,
		.v_front_porch = 2,
	},
};

static struct tegra_dc_out_pin panel_edp_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_HIGH,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},
	{
		.name   = TEGRA_DC_OUT_PIN_DATA_ENABLE,
		.pol    = TEGRA_DC_OUT_PIN_POL_HIGH,
	},
};

static void panel_edp_dc_out_init(struct tegra_dc_out *dc_out)
{
	dc_out->type = TEGRA_DC_OUT_DP;
	dc_out->align = TEGRA_DC_ALIGN_MSB;
	dc_out->order = TEGRA_DC_ORDER_RED_BLUE;
	dc_out->flags = DC_CTRL_MODE;
	dc_out->out_pins = panel_edp_out_pins;
	dc_out->n_out_pins = ARRAY_SIZE(panel_edp_out_pins);
	dc_out->depth = 18;
	dc_out->modes = panel_edp_modes;
	dc_out->n_modes = ARRAY_SIZE(panel_edp_modes);

	dc_out->dp_out->link_bw = SOR_LINK_SPEED_G1_62;
}

struct tegra_panel __initdata edp_c_1366_768_14_0 = {
	.init_dc_out = panel_edp_dc_out_init,
};
EXPORT_SYMBOL(edp_c_1366_768_14_0);

