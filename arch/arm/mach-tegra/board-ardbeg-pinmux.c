/*
 * arch/arm/mach-tegra/board-ardbeg-pinmux.c
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
#include "board-ardbeg.h"
#include "devices.h"
#include "gpio-names.h"

#include <mach/pinmux-t12.h>
#include "board-ardbeg-pinmux-t12x.h"

static __initdata struct tegra_drive_pingroup_config ardbeg_drive_pinmux[] = {

	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 32, 42, FASTEST, FASTEST),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 20, 36, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 1, 2, FASTEST,
								FASTEST, 1),
};

static void __init ardbeg_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_ardbeg_common);
	pins_info = init_gpio_mode_ardbeg_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init ardbeg_pinmux_init(void)
{
	ardbeg_gpio_init_configure();

	tegra_pinmux_config_table(ardbeg_pinmux_common, ARRAY_SIZE(ardbeg_pinmux_common));
	tegra_drive_pinmux_config_table(ardbeg_drive_pinmux,
					ARRAY_SIZE(ardbeg_drive_pinmux));
	tegra_pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	return 0;
}
