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
 *
 * arch/arm/mach-tegra/therm-monitor.c
 *
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/platform_data/tmon_tmp411.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <asm/io.h>
#include "therm-monitor.h"

static struct therm_monitor_ldep_data *lc_temp_reg_data;

static struct tmon_plat_data tmon_pdata;

/* For now assume only one entry */
struct i2c_board_info __initdata tgr_i2c_board_info[1];

static inline void pg_writel(unsigned long value, unsigned long offset)
{
	writel(value, IO_TO_VIRT(TEGRA_APB_MISC_BASE) + offset);
}

/* Call back function, invoked by driver */
static void ltemp_dependent_reg_update(int curr_ltemp)
{
	int i, j;
	for (i = 0; lc_temp_reg_data[i].reg_addr != INVALID_ADDR; i++) {
		for (j = 0; ((j < MAX_NUM_TEMPERAT) &&
			(lc_temp_reg_data[i].temperat[j] != INT_MAX)); j++) {
			if (curr_ltemp <= lc_temp_reg_data[i].temperat[j]) {
				if (lc_temp_reg_data[i].previous_val !=
						lc_temp_reg_data[i].value[j]) {
					pg_writel(lc_temp_reg_data[i].value[j],
						lc_temp_reg_data[i].reg_addr);
					lc_temp_reg_data[i].previous_val =
						lc_temp_reg_data[i].value[j];
				}
				break;
			}
		}
	}
}

void register_therm_monitor(struct therm_monitor_data *brd_therm_monitor_data)
{
	/* Array which has list of register values with temperature ranges */
	lc_temp_reg_data = brd_therm_monitor_data->brd_ltemp_reg_data;

	/* Thermal monitor operational parameters */
	tmon_pdata.delta_temp = brd_therm_monitor_data->delta_temp;
	tmon_pdata.delta_time = brd_therm_monitor_data->delta_time;
	tmon_pdata.remote_offset = brd_therm_monitor_data->remote_offset;

	/* Local temperature monitoring: Used for pad controls */
	if (brd_therm_monitor_data->local_temp_update)
		tmon_pdata.ltemp_dependent_reg_update =
			ltemp_dependent_reg_update;

	/* Remote temperature monitoring: Used for USB registers */
	if (brd_therm_monitor_data->remote_temp_update) {
		tmon_pdata.rtemp_low_boundary =
			brd_therm_monitor_data->rtemp_low_boundary;
		tmon_pdata.rtemp_high_boundary =
			brd_therm_monitor_data->rtemp_high_boundary;
		/*tmon_pdata.rtemp_dependent_reg_update =
			rtemp_dependent_reg_update;*/
	}

	/* Fill the i2c board info */
	strcpy(tgr_i2c_board_info[0].type,
		brd_therm_monitor_data->i2c_dev_name);
	tgr_i2c_board_info[0].addr = brd_therm_monitor_data->i2c_dev_addrs;
	tgr_i2c_board_info[0].platform_data = &tmon_pdata;

	i2c_register_board_info(brd_therm_monitor_data->i2c_bus_num,
				tgr_i2c_board_info, 1);
}
