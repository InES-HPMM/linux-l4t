/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All Rights Reserved.
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

#ifndef __TEGRA_PPM_H
#define __TEGRA_PPM_H

#include <linux/clk.h>
#include <linux/fs.h>

struct fv_relation;
struct fv_relation *fv_relation_create(
	struct clk *, int, ssize_t, int (*)(struct clk *, long unsigned int));

#define TEGRA_PPM_MAX_CORES 4
struct tegra_ppm_params {
	const int n_cores;

	unsigned int temp_scaled;

	unsigned int dyn_scaled;
	int dyn_consts_n[TEGRA_PPM_MAX_CORES];

	unsigned int consts_scaled;
	int leakage_consts_n[TEGRA_PPM_MAX_CORES];

	unsigned int ijk_scaled;
	int leakage_consts_ijk[4][4][4];
	unsigned int leakage_min;	 /* minimum leakage current */
};

struct tegra_ppm;

struct tegra_ppm *tegra_ppm_create(char *name,
				   struct fv_relation *fv,
				   struct tegra_ppm_params *params,
				   int iddq_ma,
				   struct dentry *debugfs_dir);

#define TEGRA_PPM_UNITS_MILLIAMPS 0
#define TEGRA_PPM_UNITS_MILLIWATTS 1


unsigned tegra_ppm_get_maxf(struct tegra_ppm *ctx,
			    unsigned int limit, int units,
			    int temp_c, int cores);

void tegra_ppm_drop_cache(struct tegra_ppm *ctx);

#endif
