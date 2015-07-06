/*
 *
 * Copyright (c) 2012-2015, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * include/linux/platform_data/tmon_tmp411.h
 *
 */


#ifndef __TMON_TMP411_INCL
#define __TMON_TMP411_INCL

#include <linux/thermal.h>
#include <linux/platform_data/thermal_sensors.h>
struct tmon_plat_data {
	unsigned int remote_offset;
	signed int alert_gpio;

	/* contains information for all trips */
	struct thermal_trip_info *trips;

	/* how many trips this thermal zone have */
	unsigned int num_trips;

	/* parameter that contains information like governor name
	 * governor params etc
	 */
	struct thermal_zone_params *tzp;

	/* delay for polling in case of active trip */
	int polling_delay;

	/* delay for polling if trip type is passive */
	int passive_delay;
};


#endif
