/*
 * arch/arm/mach-tegra/board-touch-raydium_spi.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/rm31080a_ts.h>

#include <mach/gpio-tegra.h>

int __init touch_init_raydium(int irq_gpio, int reset_gpio, struct rm_spi_ts_platform_data *rm31080ts_data, struct spi_board_info *rm31080a_spi_board, int asize)
{
	int err = 0;
	err = gpio_request(irq_gpio, "raydium-irq");
	if (err < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, err);
		return err;
	}
	err = gpio_direction_input(irq_gpio);
	if (err < 0) {
		pr_err("%s: gpio_direction_input failed %d\n",
			__func__, err);
		gpio_free(irq_gpio);
		return err;
	}
	err = gpio_request(reset_gpio, "raydium-reset");
	if (err < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, err);
		return err;
	}
	err = gpio_direction_output(reset_gpio, 0);
	if (err < 0) {
		pr_err("%s: gpio_direction_output failed %d\n",
			__func__, err);
		gpio_free(reset_gpio);
		return err;
	}

	rm31080ts_data->gpio_reset = reset_gpio;

	msleep(5);
	gpio_set_value(reset_gpio, 1);
	msleep(5);

	spi_register_board_info(rm31080a_spi_board, asize);
	pr_info("Raydium touch platform_id:  %d\n",
				rm31080ts_data->platform_id);

	return err;
}
