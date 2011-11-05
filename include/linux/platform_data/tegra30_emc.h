/*
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *	Olof Johansson <olof@lixom.net>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_EMC_H_
#define __TEGRA_EMC_H_

#define TEGRA_EMC_NUM_REGS	110

struct tegra30_emc_table {
	u8 rev;
	unsigned long rate;

	/* unconditionally updated in one burst shot */
	u32 burst_regs[TEGRA_EMC_NUM_REGS];

	/* updated separately under some conditions */
	u32 emc_zcal_cnt_long;
	u32 emc_acal_interval;
	u32 emc_periodic_qrst;
	u32 emc_mode_reset;
	u32 emc_mode_1;
	u32 emc_mode_2;
	u32 emc_dsr;
	int emc_min_mv;
};

struct tegra30_emc_pdata {
	int num_tables;
	const struct tegra30_emc_table *tables;
};

#endif
