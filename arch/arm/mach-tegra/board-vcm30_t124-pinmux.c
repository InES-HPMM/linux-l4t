/*
 * arch/arm/mach-tegra/board-vcm30_t124-pinmux.c
 *
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t12.h>

#include "board.h"
#include "board-vcm30_t124.h"
#include "devices.h"
#include "gpio-names.h"

#include "board-vcm30_t124-pinmux-t12x.h"

/* FIXME: Check these drive strengths for VCM30_T124. */
static __initdata
struct tegra_drive_pingroup_config vcm30_t124_drive_pinmux[] = {

	/*Set DAP2 drive (required for Codec Master Mode)*/
	SET_DRIVE(DAP2, DISABLE, ENABLE, DIV_1, 51, 51, FASTEST, FASTEST),

	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 54, 70, FASTEST, FASTEST),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 20, 42, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 1, 2, FASTEST,
			FASTEST, 1),
};


static void __init vcm30_t124_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_vcm30_t124_common);
	pins_info = init_gpio_mode_vcm30_t124_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init vcm30_t124_pinmux_init(void)
{
	vcm30_t124_gpio_init_configure();

	tegra_pinmux_config_table(vcm30_t124_pinmux_common,
					ARRAY_SIZE(vcm30_t124_pinmux_common));

	tegra_drive_pinmux_config_table(vcm30_t124_drive_pinmux,
					ARRAY_SIZE(vcm30_t124_drive_pinmux));
	tegra_pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	return 0;
}
