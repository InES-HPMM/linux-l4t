/*
 * arch/arm/mach-tegra/mcerr-t11.c
 *
 * Tegra 11x SoC-specific mcerr code.
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation. All rights reserved.
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

#include "mcerr.h"

#define dummy_client	client("dummy/unknown (BUG?)")

static char unknown_buf[30];

struct mc_client mc_clients[] = {
	client("ptc"),
	client("display0_wina"),	client("display1_wina"),
	client("display0_winb"),	client("display1_winb"),
	client("display0_winc"),	client("display1_winc"),
	dummy_client,			dummy_client,
	client("epp"),
	client("gr2d_pat"),		client("gr2d_src"),
	dummy_client,			dummy_client,
	dummy_client,			client("avp"),
	client("display0_cursor"),	client("display1_cursor"),
	client("gr3d_fdc0"),		client("gr3d_fdc1"),
	client("gr2d_dst"),
	client("hda"),
	client("host1x_dma"),		client("host1x_generic"),
	client("gr3d0_idx"),		dummy_client,
	dummy_client,			dummy_client,
	client("msenc"),
	client("ahb_dma"),		client("ahb_slave"),
	dummy_client,			client("gr3d0_tex"),
	dummy_client,
	client("vde_bsev"),		client("vde_mbe"),
	client("vde_mce"),		client("vde_tpe"),
	client("cpu_lp"),		client("cpu"),
	client("epp_u"),
	client("epp_v"),
	client("epp_y"),
	client("msenc"),
	client("vi_sb"),		client("vi_u"),
	client("vi_v"),			client("vi_y"),
	client("gr2d_dst"),
	dummy_client,
	client("avp"),
	client("gr3d_fdc0"),		client("gr3d_fdc1"),
	client("hda"),
	client("host1x"),
	client("isp"),
	client("cpu_lp"),		client("cpu"),
	dummy_client,
	client("ahb_dma"),		client("ahb_slave"),
	dummy_client,
	client("vde_bsev"),		client("vde_dbg"),
	client("vde_mbe"),		client("vde_tpm"),
	dummy_client,			dummy_client,
	dummy_client,			dummy_client,
	dummy_client,			dummy_client,
	dummy_client,			dummy_client,
	client("xusb_host"),		client("xusb_host"),
	client("xusb_dev"),		client("xusb_dev"),
	client("gr3d_fdc2"),		client("gr3d_fdc2"),
	client("gr3d_fdc3"),		client("gr3d_fdc3"),
	client("emucif"),		client("emucif"),
	client("tsec"),			client("tsec"),
};

static const char *mcerr_t11x_info(u32 stat)
{
	if (stat & MC_INT_DECERR_EMEM)
		return "MC_DECERR";
	else if (stat & MC_INT_SECURITY_VIOLATION)
		return "MC_SECURITY_ERR";
	else if (stat & MC_INT_INVALID_SMMU_PAGE)
		return "MC_SMMU_ERR";
	else if (stat & MC_INT_DECERR_VPR)
		return "MC_DECERR_VPR";
	else if (stat & MC_INT_SECERR_SEC)
		return "MC_DECERR_SEC";

	/* Otherwise we have an unknown type... */
	snprintf(unknown_buf, 30, "unknown - 0x%02x", stat);
	return unknown_buf;
}

/*
 * T11x reports addresses in a 32 byte range thus we can only give an
 * approximate location for the invalid memory request, not the exact address.
 */
static void mcerr_t11x_print(const char *mc_type, u32 err, u32 addr,
			     const struct mc_client *client, int is_secure,
			     int is_write, const char *mc_err_info)
{
	pr_err("%s (0x%08X): [0x%p -> 0x%p] %s (%s %s %s)\n", mc_type,
	       err, (void *)(addr & ~0x1f), (void *)(addr | 0x1f),
	       (client) ? client->name : "unknown",
	       (is_secure) ? "secure" : "non-secure",
	       (is_write) ? "write" : "read",
	       mc_err_info);
}

/*
 * Specify special functions for dealing with the T11x mcerr handling.
 */
void mcerr_chip_specific_setup(struct mcerr_chip_specific *spec)
{
	spec->mcerr_info = mcerr_t11x_info;
	spec->mcerr_print = mcerr_t11x_print;
	return;
}
