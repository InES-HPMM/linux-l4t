/*
 * arch/arm/mach-tegra/board-roth-kbc.c
 * Keys configuration for Nvidia tegra3 roth platform.
 *
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
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/kbc.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/palmas.h>

#include "tegra-board-id.h"
#include "board.h"
#include "board-roth.h"
#include "devices.h"

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

/* Make KEY_POWER to index 0 only */
static struct gpio_keys_button roth_p2454_keys[] = {
	[0] = GPIO_KEY(KEY_POWER, PR0, 0),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PR2, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PR1, 0),
	[3] = {
		.code = SW_LID,
		.gpio = TEGRA_GPIO_HALL,
		.irq = -1,
		.type = EV_SW,
		.desc = "Hall Effect Sensor",
		.active_low = 1,
		.wakeup = 1,
		.debounce_interval = 100,
	},
	[4] = GPIO_IKEY(KEY_POWER, PALMAS_TEGRA_IRQ_BASE +
					PALMAS_PWRON_IRQ, 1, 100),
};


static struct gpio_keys_platform_data roth_p2454_keys_pdata = {
	.buttons	= roth_p2454_keys,
	.nbuttons	= ARRAY_SIZE(roth_p2454_keys),
};

static struct platform_device roth_p2454_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data  = &roth_p2454_keys_pdata,
	},
};

int __init roth_kbc_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* Rev A01 and onward have the POWER key in the KBC-COL0 */
	if (board_info.major_revision > BOARD_FAB_A00)
		roth_p2454_keys[0].gpio = TEGRA_GPIO_PQ0;

	if (board_info.major_revision >= BOARD_FAB_A02) {
		roth_p2454_keys[1].code = KEY_BACK;
		roth_p2454_keys[2].code = KEY_HOME;
	}

	platform_device_register(&roth_p2454_keys_device);
	return 0;
}

