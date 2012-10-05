/*
 * Copyright (C) 2012 NVIDIA, Inc.
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
#include "board-dalmore.h"

#include "tegra-board-id.h"
#include "tegra11_emc.h"
#include "fuse.h"
#include "devices.h"

static struct tegra11_emc_table e1611_h5tc4g63mfr_pba_table[] = {
};

static struct tegra11_emc_pdata e1613_h9ccnnn8jtmlar_ntm_pdata = {
	.description = "e1613_h9ccnnn8jtmlar_ntm",
};

static struct tegra11_emc_pdata e1611_h5tc4g63mfr_pba_pdata = {
	.description = "e1611_h5tc4g63mfr_pba",
	.tables = e1611_h5tc4g63mfr_pba_table,
	.num_tables = ARRAY_SIZE(e1611_h5tc4g63mfr_pba_table),
};

static struct tegra11_emc_pdata *dalmore_get_emc_data(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_E1611)
		return &e1611_h5tc4g63mfr_pba_pdata;

	return &e1613_h9ccnnn8jtmlar_ntm_pdata;
}

int __init dalmore_emc_init(void)
{
	tegra_emc_device.dev.platform_data = dalmore_get_emc_data();
	platform_device_register(&tegra_emc_device);
	tegra11_emc_init();

	return 0;
}
