/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_data/tegra_emc.h>

#include "board.h"
#include "board-ardbeg.h"
#include "tegra-board-id.h"
#include "tegra12_emc.h"
#include "devices.h"

static struct tegra12_emc_table norrin_emc_table[] = {

};

#ifdef CONFIG_TEGRA_USE_NCT
static struct tegra12_emc_pdata board_emc_pdata;
#endif

static struct tegra12_emc_pdata norrin_emc_pdata = {
	.description = "norrin_emc_tables",
	.tables = norrin_emc_table,
	.num_tables = ARRAY_SIZE(norrin_emc_table),
};

int __init norrin_emc_init(void)
{

	struct board_info bi;
	int use_dt_emc_table = 0;

	/*
	 * If the EMC table is successfully read from the NCT partition,
	 * we do not need to check for board ids and blindly load the one
	 * flashed on the NCT partition.
	 */
	#ifdef CONFIG_TEGRA_USE_NCT
	if (!tegra12_nct_emc_table_init(&board_emc_pdata)) {
		tegra_emc_device.dev.platform_data = &board_emc_pdata;
		pr_info("Loading EMC table read from NCT partition.\n");
	} else {
	#endif
		tegra_get_board_info(&bi);

		switch (bi.board_id) {
		case BOARD_PM374:
			/* Norrin only has 2G board */
			pr_info("Loading Norrin EMC tables.\n");
			tegra_emc_device.dev.platform_data =
				&norrin_emc_pdata;
			break;
		/* case BOARD_BOWMORE: */
		/* 	if (tegra_get_memory_type()) { */
		/* 		pr_info("Loading Norrin 4GB EMC tables.\n"); */
		/* 		tegra_emc_device.dev.platform_data = */
		/* 			&norrin_4GB_emc_pdata; */
		/* 	} else { */
		/* 		pr_info("Loading Norrin EMC tables.\n"); */
		/* 		tegra_emc_device.dev.platform_data = */
		/* 			&norrin_emc_pdata; */
		/* 	} */
			/* break; */
		default:
			WARN(1, "Invalid board ID: %u\n", bi.board_id);
			return -EINVAL;
		}
	#ifdef CONFIG_TEGRA_USE_NCT
	}
	#endif

	if (!use_dt_emc_table)
		platform_device_register(&tegra_emc_device);

	tegra12_emc_init();
	return 0;
}
