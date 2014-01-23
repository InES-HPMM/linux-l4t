/*
 * arch/arm/mach-tegra/board-norrin-kbc.c
 * Keys configuration for NVIDIA T124 Norrin platform.
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/input/tegra_kbc.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include "tegra-board-id.h"
#include "board.h"
#include "board-ardbeg.h"
#include "devices.h"
#include "iomap.h"
#include "wakeups-t12x.h"

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = _gpio,			\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

#define PMC_WAKE_STATUS         0x14
#define TEGRA_WAKE_PWR_INT      (1UL << 18)
#define PMC_WAKE2_STATUS        0x168

static struct gpio_keys_button norrin_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, TEGRA_GPIO_PQ0, 1),
	[1] = GPIO_KEY(KEY_HOME, PMU_TCA6416_GPIO(1), 0),
	[2] = GPIO_KEY(KEY_VOLUMEUP, PMU_TCA6416_GPIO(4), 0),
	[3] = GPIO_KEY(KEY_VOLUMEDOWN, PMU_TCA6416_GPIO(5), 0),
};

static int norrin_wakeup_key(void)
{
	int wakeup_key;
	u32 status;
	status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS)
		| (u64)readl(IO_ADDRESS(TEGRA_PMC_BASE)
		+ PMC_WAKE2_STATUS) << 32;

	if (status & ((u64)1 << TEGRA_WAKE_GPIO_PQ0))
		wakeup_key = KEY_POWER;
	else
		wakeup_key = -1;

	return wakeup_key;
}

static struct gpio_keys_platform_data norrin_keys_pdata = {
	.buttons	= norrin_keys,
	.nbuttons	= ARRAY_SIZE(norrin_keys),
	.wakeup_key	= norrin_wakeup_key,
};

static struct platform_device norrin_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data  = &norrin_keys_pdata,
	},
};

int __init norrin_kbc_init(void)
{
	return platform_device_register(&norrin_keys_device);
}
