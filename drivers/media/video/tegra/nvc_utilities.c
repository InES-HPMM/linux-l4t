/*
 * nvc_utilities.c - nvc utility functions
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation. All Rights Reserved.
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

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/nvc.h>
#include <media/nvc_image.h>


int parse_nvc_imager_caps(struct device_node *np,
		struct nvc_imager_cap *imager_cap)
{
	const char *cap_identifier = NULL;
	u32 min_blank_width = 0;
	u32 min_blank_height = 0;
	u32 temp_prop_read = 0;

	of_property_read_string(np, "nvidia,identifier", &cap_identifier);
	strcpy(imager_cap->identifier, cap_identifier);
	of_property_read_u32(np, "nvidia,sensor_nvc_interface",
				&imager_cap->sensor_nvc_interface);
	of_property_read_u32(np, "nvidia,pixel_types",
				&imager_cap->pixel_types[0]);
	of_property_read_u32(np, "nvidia,orientation",
				&imager_cap->orientation);
	of_property_read_u32(np, "nvidia,direction",
				&imager_cap->direction);
	of_property_read_u32(np, "nvidia,initial_clock_rate_khz",
				&imager_cap->initial_clock_rate_khz);
	of_property_read_u32(np, "nvidia,h_sync_edge",
				&imager_cap->h_sync_edge);
	of_property_read_u32(np, "nvidia,v_sync_edge",
				&imager_cap->v_sync_edge);
	of_property_read_u32(np, "nvidia,mclk_on_vgp0",
				&imager_cap->mclk_on_vgp0);
	of_property_read_u32(np, "nvidia,csi_port",
				&temp_prop_read);
	imager_cap->csi_port = (u8)temp_prop_read;
	of_property_read_u32(np, "nvidia,data_lanes",
				&temp_prop_read);
	imager_cap->data_lanes = (u8)temp_prop_read;
	of_property_read_u32(np, "nvidia,virtual_channel_id",
				&temp_prop_read);
	imager_cap->virtual_channel_id = (u8)temp_prop_read;
	of_property_read_u32(np, "nvidia,discontinuous_clk_mode",
				&temp_prop_read);
	imager_cap->discontinuous_clk_mode = (u8)temp_prop_read;
	of_property_read_u32(np, "nvidia,cil_threshold_settle",
				&temp_prop_read);
	imager_cap->cil_threshold_settle = (u8)temp_prop_read;
	of_property_read_u32(np, "nvidia,min_blank_time_width",
				&min_blank_width);
	imager_cap->min_blank_time_width = (s32)min_blank_width;
	of_property_read_u32(np, "nvidia,min_blank_time_height",
				&min_blank_height);
	imager_cap->min_blank_time_width = (s32)min_blank_height;
	of_property_read_u32(np, "nvidia,preferred_mode_index",
				&imager_cap->preferred_mode_index);
	of_property_read_u32(np, "nvidia,external_clock_khz_0",
		&imager_cap->clock_profiles[0].external_clock_khz);
	of_property_read_u32(np, "nvidia,clock_multiplier_0",
		&imager_cap->clock_profiles[0].clock_multiplier);
	of_property_read_u32(np, "nvidia,external_clock_khz_1",
		&imager_cap->clock_profiles[1].external_clock_khz);
	of_property_read_u32(np, "nvidia,clock_multiplier_1",
		&imager_cap->clock_profiles[1].clock_multiplier);

	return 0;
}