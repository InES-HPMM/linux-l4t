/*
 * arch/arm/mach-tegra/board-dalmore-kbc.c
 * Keys configuration for Nvidia tegra3 dalmore platform.
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
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/palmas.h>

#include "tegra-board-id.h"
#include "board.h"
#include "board-dalmore.h"
#include "devices.h"

#define DALMORE_ROW_COUNT	3
#define DALMORE_COL_COUNT	3

#define DALMORE_ROW_COUNT_1001	1
#define DALMORE_COL_COUNT_1001	1

static const u32 kbd_keymap[] = {
	KEY(0, 0, KEY_POWER),
	KEY(0, 1, KEY_HOME),

	KEY(1, 0, KEY_RESERVED),
	KEY(1, 1, KEY_VOLUMEDOWN),

	KEY(2, 0, KEY_CAMERA),
	KEY(2, 1, KEY_VOLUMEUP),
	KEY(2, 2, KEY_2),
};

static const struct matrix_keymap_data keymap_data = {
	.keymap		= kbd_keymap,
	.keymap_size	= ARRAY_SIZE(kbd_keymap),
};

static const u32 kbd_keymap_1001[] = {
	KEY(0, 0, KEY_POWER),
};

static const struct matrix_keymap_data keymap_data_1001 = {
	.keymap		= kbd_keymap_1001,
	.keymap_size	= ARRAY_SIZE(kbd_keymap_1001),
};

static struct tegra_kbc_wake_key dalmore_wake_cfg[] = {
	[0] = {
		.row = 0,
		.col = 0,
	},
};

static struct tegra_kbc_platform_data dalmore_kbc_platform_data = {
	.debounce_cnt = 20 * 32, /* 20 ms debaunce time */
	.repeat_cnt = 1,
	.scan_count = 30,
	.wakeup = true,
	.keymap_data = &keymap_data,
	.wake_cnt = 1,
	.wake_cfg = &dalmore_wake_cfg[0],
	.wakeup_key = KEY_POWER,
#ifdef CONFIG_ANDROID
	.disable_ev_rep = true,
#endif
};

static struct tegra_kbc_platform_data dalmore_kbc_platform_data_1001 = {
	.debounce_cnt = 20 * 32, /* 20 ms debaunce time */
	.repeat_cnt = 1,
	.scan_count = 30,
	.wakeup = true,
	.keymap_data = &keymap_data_1001,
	.wake_cnt = 1,
	.wake_cfg = &dalmore_wake_cfg[0],
	.wakeup_key = KEY_POWER,
#ifdef CONFIG_ANDROID
	.disable_ev_rep = true,
#endif
};

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

static struct gpio_keys_button dalmore_int_keys[] = {
	[0] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE +
				MAX77663_IRQ_ONOFF_EN0_FALLING, 0, 100),
	[1] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE +
				MAX77663_IRQ_ONOFF_EN0_1SEC, 0, 3000),
};

static struct gpio_keys_button dalmore_e1611_1001_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PR2, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PR1, 0),
};

static struct gpio_keys_platform_data dalmore_int_keys_pdata = {
	.buttons	= dalmore_int_keys,
	.nbuttons	= ARRAY_SIZE(dalmore_int_keys),
};

static struct gpio_keys_platform_data dalmore_e1611_1001_keys_pdata = {
	.buttons	= dalmore_e1611_1001_keys,
	.nbuttons	= ARRAY_SIZE(dalmore_e1611_1001_keys),
};

static struct platform_device dalmore_int_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data  = &dalmore_int_keys_pdata,
	},
};

static struct platform_device dalmore_e1611_1001_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data  = &dalmore_e1611_1001_keys_pdata,
	},
};

static void __init dalmore_register_kbc(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E1611 && board_info.sku != 1001) {
		struct tegra_kbc_platform_data *data = &dalmore_kbc_platform_data;
		int i;

		tegra_kbc_device.dev.platform_data = &dalmore_kbc_platform_data;
		pr_info("Registering tegra-kbc\n");

		BUG_ON((KBC_MAX_ROW + KBC_MAX_COL) > KBC_MAX_GPIO);
		for (i = 0; i < DALMORE_ROW_COUNT; i++) {
			data->pin_cfg[i].num = i;
			data->pin_cfg[i].type = PIN_CFG_ROW;
		}
		for (i = 0; i < DALMORE_COL_COUNT; i++) {
			data->pin_cfg[i + KBC_PIN_GPIO_11].num = i;
			data->pin_cfg[i + KBC_PIN_GPIO_11].type = PIN_CFG_COL;
		}

		platform_device_register(&tegra_kbc_device);
		pr_info("Registering successful tegra-kbc\n");
	} else {
		struct tegra_kbc_platform_data *data = &dalmore_kbc_platform_data_1001;
		int i;

		tegra_kbc_device.dev.platform_data = &dalmore_kbc_platform_data_1001;
		pr_info("Registering tegra-kbc\n");

		BUG_ON((KBC_MAX_ROW + KBC_MAX_COL) > KBC_MAX_GPIO);
		for (i = 0; i < DALMORE_ROW_COUNT_1001; i++) {
			data->pin_cfg[i].num = i;
			data->pin_cfg[i].type = PIN_CFG_ROW;
		}
		for (i = 0; i < DALMORE_COL_COUNT_1001; i++) {
			data->pin_cfg[i + KBC_PIN_GPIO_11].num = i;
			data->pin_cfg[i + KBC_PIN_GPIO_11].type = PIN_CFG_COL;
		}

		platform_device_register(&tegra_kbc_device);
		pr_info("Registering successful tegra-kbc\n");
	}
}

int __init dalmore_kbc_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	pr_info("Boardid:SKU = 0x%04x:0x%04x\n", board_info.board_id, board_info.sku);

	if (board_info.board_id != BOARD_E1611 &&
		board_info.board_id != BOARD_P2454) {
		dalmore_register_kbc();
		platform_device_register(&dalmore_int_keys_device);
	} else if (board_info.board_id == BOARD_E1611) {
		if (board_info.sku != 1001) {
			dalmore_register_kbc();
		} else {
			dalmore_register_kbc();
			platform_device_register(
				&dalmore_e1611_1001_keys_device);
		}
	}

	return 0;
}

