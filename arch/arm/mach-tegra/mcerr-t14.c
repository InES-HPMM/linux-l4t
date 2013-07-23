/*
 * arch/arm/mach-tegra/mcerr-t14.c
 *
 * Tegra 14x SoC-specific mcerr code.
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation. All rights reserved.
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

#define dummy_client	client("dummy")

struct mc_client mc_clients[] = {
	client("ptc"),
	client("display0_wina"),	client("display1_wina"),
	client("display0_winb"),	client("display1_winb"),
	client("display0_winc"),	client("display1_winc"),
	dummy_client,			dummy_client,
	client("epp"),
	client("gr2d_pat"), /* 10 */	client("gr2d_src"),
	dummy_client,			dummy_client,
	dummy_client,			client("avp"),
	client("display0_cursor"),	client("display1_cursor"),
	client("gr3d_fdc0"),		client("gr3d_fdc1"),
	client("gr2d_dst"), /* 20 */
	client("hda"),
	client("host1x_dma"),		client("host1x_generic"),
	client("gr3d0_idx"),		dummy_client,
	dummy_client,			dummy_client,
	client("msenc"),
	client("ahb_dma"),		client("ahb_slave_r"), /* 30 */
	dummy_client,			client("gr3d0_tex"),
	dummy_client,
	client("vde_bsev"),		client("vde_mbe"),
	client("vde_mce"),		client("vde_tpe"),
	client("cpu_lp"),		client("cpu"),
	client("epp_u"), /* 40 */
	client("epp_v"),
	client("epp_y"),
	client("msenc"),
	client("vi_sb"),		client("vi_u"),
	client("vi_v"),			client("vi_y"),
	client("gr2d_dst"),
	dummy_client,
	client("avp"), /* 50 */
	client("gr3d_fdc0"),		client("gr3d_fdc1"),
	client("hda"),
	client("host1x"),
	client("isp"),
	client("cpu_lp"),		client("cpu"),
	dummy_client,
	client("ahb_dma"),		client("ahb_slave_w"), /* 60 */
	dummy_client,
	client("vde_bsev"),		client("vde_dbg"),
	client("vde_mbe"),		client("vde_tpm"),
	dummy_client,			dummy_client,
	client("ve_ispra"),		dummy_client,
	client("ve_ispwa"), /* 70 */	client("ve_ispwb"),
	dummy_client,			dummy_client,
	dummy_client,			dummy_client,
	dummy_client,			dummy_client,
	dummy_client,			dummy_client,
	dummy_client, /* 80 */		dummy_client,
	client("emucif"),		client("emucif"),
	client("tsec"),			client("tsec"),
	client("viw"),			client("bbcr"),
	client("bbcw"),			client("bbcllr"),
	client("disp_t"), /* 90 */	dummy_client,
	client("disp_d"),
};

int mc_client_last = ARRAY_SIZE(mc_clients) - 1;

static void mcerr_t14x_info_update(struct mc_client *c, u32 stat)
{
	if (stat & MC_INT_DECERR_EMEM)
		c->intr_counts[0]++;
	if (stat & MC_INT_SECURITY_VIOLATION)
		c->intr_counts[1]++;
	if (stat & MC_INT_INVALID_SMMU_PAGE)
		c->intr_counts[2]++;
	if (stat & MC_INT_DECERR_VPR)
		c->intr_counts[3]++;
	if (stat & MC_INT_SECERR_SEC)
		c->intr_counts[4]++;
	if (stat & MC_INT_BBC_PRIVATE_MEM_VIOLATION)
		c->intr_counts[5]++;
	if (stat & MC_INT_DECERR_BBC)
		c->intr_counts[6]++;

	if (stat & ~MC_INT_EN_MASK)
		c->intr_counts[7]++;
}

static int mcerr_t14x_debugfs_show(struct seq_file *s, void *v)
{
	int i, j;
	int do_print;
	const char *fmt = "%-18s %-9u %-9u %-9u %-10u %-10u %-9u %-9u %-9u\n";

	seq_printf(s, "%-18s %-9s %-9s %-9s %-10s %-10s %-9s %-9s %-9s\n",
		   "client", "decerr", "secerr", "smmuerr",
		   "decerr-VPR", "secerr-SEC", "priv-bbc", "decerr-bbc",
		   "unknown");
	for (i = 0; i < ARRAY_SIZE(mc_clients); i++) {
		do_print = 0;
		if (strcmp(mc_clients[i].name, "dummy") == 0)
			continue;
		/* Only print clients who actually have errors. */
		for (j = 0; j < INTR_COUNT; j++) {
			if (mc_clients[i].intr_counts[j]) {
				do_print = 1;
				break;
			}
		}
		if (do_print)
			seq_printf(s, fmt,
				   mc_clients[i].name,
				   mc_clients[i].intr_counts[0],
				   mc_clients[i].intr_counts[1],
				   mc_clients[i].intr_counts[2],
				   mc_clients[i].intr_counts[3],
				   mc_clients[i].intr_counts[4],
				   mc_clients[i].intr_counts[5],
				   mc_clients[i].intr_counts[6],
				   mc_clients[i].intr_counts[7]);
	}
	return 0;
}

/*
 * Set up chip specific functions and data for handling this particular chip's
 * error decoding and logging.
 */
void mcerr_chip_specific_setup(struct mcerr_chip_specific *spec)
{
	spec->mcerr_info_update = mcerr_t14x_info_update;
	spec->mcerr_debugfs_show = mcerr_t14x_debugfs_show;
	spec->nr_clients = ARRAY_SIZE(mc_clients);
	return;
}
