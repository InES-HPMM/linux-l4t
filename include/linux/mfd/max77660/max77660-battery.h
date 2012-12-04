/*
 * Fuel gauge driver for MAX77660 PMIC
 *
 * Copyright 2012 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __MAX77660_BATTERY_H__
#define __MAX77660_BATTERY_H__

struct max77660_fg_platform_data {
	u8 valrt_min;
	u8 valrt_max;
	bool alsc;
	bool alrt;
	bool athd;
 };

#endif /* __MAX77660_BATTERY_H__ */
