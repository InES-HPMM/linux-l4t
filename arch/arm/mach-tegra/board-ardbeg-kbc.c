/*
 * arch/arm/mach-tegra/board-ardbeg-kbc.c
 * Keys configuration for Nvidia tegra4 ardbeg platform.
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/mfd/palmas.h>

#include "tegra-board-id.h"
#include "board.h"
#include "board-ardbeg.h"
#include "devices.h"
#include "iomap.h"
#include "wakeups-t12x.h"


#define GPIO_KEY(_id, _gpio, _iswake)           \
	{                                       \
		.code = _id,                    \
		.gpio = TEGRA_GPIO_##_gpio,     \
		.active_low = 1,                \
		.desc = #_id,                   \
		.type = EV_KEY,                 \
		.wakeup = _iswake,              \
		.debounce_interval = 10,        \
	}

#define GPIO_IKEY(_id, _irq, _iswake, _deb)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
	}

#define PMC_WAKE2_STATUS         0x168
#define TEGRA_WAKE_PWR_INT      (1UL << 19)

static int ardbeg_wakeup_key(void);

static struct gpio_keys_button ardbeg_int_keys[] = {
	[0] = GPIO_IKEY(KEY_POWER, 0, 1, 10),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PQ6, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PQ7, 0),
	[3] = GPIO_KEY(KEY_HOME, PI5, 0),
	[4] = GPIO_KEY(KEY_CAMERA_FOCUS, PQ2, 0),
#if 0
	[5] = GPIO_KEY(KEY_CAMERA, PV3, 0),
#endif
};

static struct gpio_keys_platform_data ardbeg_int_keys_pdata = {
	.buttons	= ardbeg_int_keys,
	.nbuttons	= ARRAY_SIZE(ardbeg_int_keys),
	.wakeup_key	= ardbeg_wakeup_key,
};

static struct platform_device ardbeg_int_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data  = &ardbeg_int_keys_pdata,
	},
};

static int ardbeg_wakeup_key(void)
{
	u32 status;
	status = __raw_readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE2_STATUS);

	pr_info("%s: Power key pressed\n", __func__);

	return (status & TEGRA_WAKE_PWR_INT) ? KEY_POWER : KEY_RESERVED;
}

int __init ardbeg_kbc_init(void)
{
	struct board_info board_info;
	int ret;

	tegra_get_board_info(&board_info);
	pr_info("Boardid:SKU = 0x%04x:0x%04x\n",
			board_info.board_id, board_info.sku);

	ardbeg_int_keys[0].gpio = TEGRA_GPIO_PQ0;
	ardbeg_int_keys[0].active_low = 1;

	ret = platform_device_register(&ardbeg_int_keys_device);
	return ret;
}
