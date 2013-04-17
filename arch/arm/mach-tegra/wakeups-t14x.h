/*
 * arch/arm/mach-tegra/wakeups-t14x.h
 *
 * Declarations of Tegra 14x LP0 wakeup sources
 *
 * Copyright (c) 2013, NVIDIA Corporation.
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

#ifndef __MACH_TEGRA_WAKEUPS_T14X_H
#define __MACH_TEGRA_WAKEUPS_T14X_H

#ifndef CONFIG_ARCH_TEGRA_14x_SOC
#error "Tegra 14x wakeup sources valid only for CONFIG_ARCH_TEGRA_14x_SOC"
#endif


/* Must align with tegra_gpio_wakes table in wakeups-t14x.c */
#define TEGRA_WAKE_GPIO_PL0		0			/* wake0 */
#define TEGRA_WAKE_GPIO_PL2		1			/* wake1 */
#define TEGRA_WAKE_GPIO_PM2		2			/* wake2 */
#define TEGRA_WAKE_GPIO_PM4		4			/* wake4 */
#define TEGRA_WAKE_GPIO_PM7		5			/* wake5 */
#define TEGRA_WAKE_GPIO_PN1		6			/* wake6 */
#define TEGRA_WAKE_GPIO_PO0		7			/* wake7 */
#define TEGRA_WAKE_GPIO_PO1		8			/* wake8 */
#define TEGRA_WAKE_GPIO_PO2		9			/* wake9 */
#define TEGRA_WAKE_GPIO_PO3		11			/* wake11 */
#define TEGRA_WAKE_GPIO_PO4		12			/* wake12 */
#define TEGRA_WAKE_GPIO_PO5		14			/* wake14 */
#define TEGRA_WAKE_GPIO_PO6		15			/* wake15 */
#define TEGRA_WAKE_GPIO_PJ5		23			/* wake23 */
#define TEGRA_WAKE_GPIO_PJ6		24			/* wake24 */
#define TEGRA_WAKE_GPIO_PJ1		25			/* wake25 */
#define TEGRA_WAKE_GPIO_PJ2		26			/* wake26 */
#define TEGRA_WAKE_GPIO_PJ3		27			/* wake27 */
#define TEGRA_WAKE_GPIO_PJ4		28			/* wake28 */
#define TEGRA_WAKE_GPIO_PJ0		33			/* wake33 */
#define TEGRA_WAKE_GPIO_PK2		34			/* wake34 */
#define TEGRA_WAKE_GPIO_PI6		35			/* wake35 */
#define TEGRA_WAKE_GPIO_PBB6	45			/* wake45 */
#define TEGRA_WAKE_GPIO_PR7		49			/* wake49 */
#define TEGRA_WAKE_GPIO_PR4		50			/* wake50 */
#define TEGRA_WAKE_GPIO_PQ5		54			/* wake54 */

#endif
