/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_XUSB_H_
#define _TEGRA_XUSB_H_
/**
 * board specific pad parameters
 */
struct tegra_xusb_pad_data {
	u32 pad_mux;
	u32 port_cap;
	u32 snps_oc_map;
	u32 usb2_oc_map;
	u32 ss_port_map;
	u32 oc_det;
	u32 rx_wander;
	u32 rx_eq;
	u32 cdr_cntl;
	u32 dfe_cntl;
	u32 otg_pad0_ctl0;
	u32 hs_slew;
	u32 hs_curr_level;
	u32 hs_iref_cap;
	u32 hs_term_range_adj;
	u32 hs_squelch_level;
	u32 otg_pad0_ctl1;
	u32 otg_pad1_ctl0;
	u32 otg_pad1_ctl1;
	u32 bias_pad_ctl0;
	u32 hsic_pad0_ctl0;
	u32 hsic_pad0_ctl1;
	u32 pmc_value;
};
#endif /* _TEGRA_XUSB_H_ */
