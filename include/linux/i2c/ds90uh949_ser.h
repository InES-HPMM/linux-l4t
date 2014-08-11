/*
 * FPDLink Serializer driver
 *
 * Copyright (C) 2014 NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_DS90UH949_SER_H
#define __LINUX_DS90UH949_SER_H

#include <linux/types.h>

#define DS90UH949_SER_REG_RESET			0x01
#define DS90UH949_SER_REG_RESET_DIGRESET0		BIT(0)
#define DS90UH949_SER_REG_RESET_DIGRESET1		BIT(1)

/* The platform data for the FPDLink Serializer driver */
struct ds90uh949_platform_data {
	int en_gpio; /* GPIO */
	int en_gpio_flags;
	int power_on_delay;
/* TODO: have configuration parameters like HDCP support etc.. */
};

#endif /* __LINUX_DS90UH949_SER_H */
