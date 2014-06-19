/*
 * arch/arm/mach-tegra/board-p2360.h
 * Based on arch/arm/mach-tegra/board-vcm30_t124.h
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _MACH_TEGRA_BOARD_P2360_H
#define _MACH_TEGRA_BOARD_P2360_H

#include "gpio-names.h"

int p2360_panel_init(void);
int p2360_sdhci_init(void);
int p2360_suspend_init(void);
int p2360_regulator_init(void);

#define UTMI1_PORT_OWNER_XUSB	0x1
#define UTMI2_PORT_OWNER_XUSB	0x2

/* Tegra GPIOs */
#define TEGRA_GPIO_GPUPWR	TEGRA_GPIO_PR2

#define TEGRA_GPIO_TV1ENA	TEGRA_GPIO_PI5
#define TEGRA_GPIO_TV2ENA	TEGRA_GPIO_PK2
#define TEGRA_GPIO_TV3ENA	TEGRA_GPIO_PK0
#define TEGRA_GPIO_TV4ENA	TEGRA_GPIO_PK1

/* Thermal monitor data */
#define DELTA_TEMP		4000
#define DELTA_TIME		2000
#define REMT_OFFSET		8000
#define I2C_ADDR_TMP411		0x4c
#define I2C_BUS_TMP411		1

/* External peripheral act as gpio */
#define MAX77663_IRQ_BASE	TEGRA_NR_IRQS
#define MAX77663_IRQ_END	(MAX77663_IRQ_BASE + MAX77663_IRQ_NR)
#define MAX77663_GPIO_BASE	TEGRA_NR_GPIOS
#define MAX77663_GPIO_END	(MAX77663_GPIO_BASE + MAX77663_GPIO_NR)

#endif
