/*
 * arch/arm/mach-tegra/board-loki-pinmux.c
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
#include "board-loki.h"
#include "tegra-board-id.h"
#include "devices.h"
#include "gpio-names.h"

#include <mach/pinmux-t12.h>

/* Pinmux changes to support UART over uSD adapter E2542 */
static __initdata struct tegra_pingroup_config loki_sdmmc3_uart_pinmux[] = {

	DEFAULT_PINMUX(SDMMC3_CMD,    UARTA,      NORMAL,   NORMAL,   INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1,   UARTA,      NORMAL,   NORMAL,   OUTPUT),
};

int __init loki_pinmux_init(void)
{
	if (is_uart_over_sd_enabled()) {
		tegra_pinmux_config_table(loki_sdmmc3_uart_pinmux,
				 ARRAY_SIZE(loki_sdmmc3_uart_pinmux));
		/* On loki, UART-A is the physical device for
		 * UART over uSD card
		 */
		set_sd_uart_port_id(0);
	}
	return 0;
}
