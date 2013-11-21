/*
 * arch/arm/mach-tegra/board-tn8-p1761-pinmux.c
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/gpio-tegra.h>

#include "board.h"
#include "board-tn8-p1761.h"
#include "tegra-board-id.h"
#include "devices.h"
#include "gpio-names.h"

#include <mach/pinmux-t12.h>

static __initdata struct tegra_drive_pingroup_config
			 tn8_p1761_drive_pinmux[] = {

	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 32, 42, FASTEST, FASTEST),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 20, 36, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 1, 2, FASTEST,
								FASTEST, 1),
};

#include "board-tn8-p1761-pinmux-t12x.h"

static void __init tn8_p1761_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_tn8_p1761_common);
	pins_info = init_gpio_mode_tn8_p1761_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}

}

int __init tn8_p1761_pinmux_init(void)
{
	tn8_p1761_gpio_init_configure();

	tegra_pinmux_config_table(tn8_p1761_pinmux_common,
		ARRAY_SIZE(tn8_p1761_pinmux_common));
	tegra_drive_pinmux_config_table(tn8_p1761_drive_pinmux,
		ARRAY_SIZE(tn8_p1761_drive_pinmux));
	tegra_pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	return 0;
}
