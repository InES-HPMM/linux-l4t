/*
 * arch/arm/mach-tegra/board-laguna.h
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#ifndef _MACH_TEGRA_BOARD_LAGUNA_H
#define _MACH_TEGRA_BOARD_LAGUNA_H

#include <linux/mfd/as3722-reg.h>
#include <mach/gpio-tegra.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include "gpio-names.h"

/* External peripheral act as gpio */
/* AS3720 GPIO */
#define AS3722_GPIO_BASE        TEGRA_NR_GPIOS
#define AS3722_GPIO_END		(AS3722_GPIO_BASE + AS3722_NUM_GPIO)
/* PMU_TCA6416 GPIOs */
#define PMU_TCA6416_GPIO_BASE   (AS3722_GPIO_END)
/* External peripheral act as interrupt controller */
/* AS3720 IRQs */
#define AS3722_IRQ_BASE		TEGRA_NR_IRQS

int laguna_pinmux_init(void);
int laguna_panel_init(void);
int laguna_kbc_init(void);
int laguna_sdhci_init(void);
int laguna_sensors_init(void);
int laguna_regulator_init(void);
int laguna_suspend_init(void);

/* Baseband IDs */
enum tegra_bb_type {
	TEGRA_BB_NEMO = 1,
	TEGRA_BB_HSIC_HUB = 6,
};

#define UTMI1_PORT_OWNER_XUSB   0x1
#define UTMI2_PORT_OWNER_XUSB   0x2
#define HSIC1_PORT_OWNER_XUSB   0x4

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ TEGRA_GPIO_PW3
#define TEGRA_GPIO_LDO1_EN TEGRA_GPIO_PV3
#define TEGRA_GPIO_HP_DET  TEGRA_GPIO_PR7
#define TEGRA_GPIO_CODEC1_EN  -1 /*TEGRA_GPIO_PP3*/
#define TEGRA_GPIO_CODEC2_EN  -1 /*TEGRA_GPIO_PP1*/
#define TEGRA_GPIO_CODEC3_EN  -1 /*TEGRA_GPIO_PV0*/
#define TEGRA_GPIO_INT_MIC_EN -1 /*TEGRA_GPIO_PK3*/
#define TEGRA_GPIO_SPKR_EN    -1
#define TEGRA_GPIO_EXT_MIC_EN -1

/* Display related GPIO */
#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PH3 /* GMI_AD11 */
#define LCD_RST_L		TEGRA_GPIO_PH5 /* GMI_AD13 */
#define LCD_LR			TEGRA_GPIO_PH6 /* GMI_AD14 */
#define LCD_TE			TEGRA_GPIO_PI4 /* GMI_RST_N */
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1 /*GMI_AD9 */
#define en_vdd_bl       TEGRA_GPIO_PP2 /* DAP3_DOUT */
#define lvds_en		TEGRA_GPIO_PI0 /* GMI_WR_N */
#define refclk_en	TEGRA_GPIO_PG4 /* GMI_AD4 */

#else
/*for t124 ardbeg */
#define TEGRA_GPIO_CDC_IRQ TEGRA_GPIO_PH4
#define TEGRA_GPIO_LDO1_EN TEGRA_GPIO_PR2
#define TEGRA_GPIO_HP_DET  TEGRA_GPIO_PR7

/* Display related GPIO */
#define DSI_PANEL_RST_GPIO	TEGRA_GPIO_PH3 /* GMI_AD11 */
#define LCD_RST_L		TEGRA_GPIO_PH5 /* GMI_AD13 */
#define LCD_LR			TEGRA_GPIO_PH6 /* GMI_AD14 */
#define LCD_TE			TEGRA_GPIO_PI4 /* GMI_RST_N */
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1 /*GMI_AD9 */
#define en_vdd_bl       TEGRA_GPIO_PP2 /* DAP3_DOUT */
#define lvds_en		TEGRA_GPIO_PI0 /* GMI_WR_N */
#define refclk_en	TEGRA_GPIO_PG4 /* GMI_AD4 */
#endif

#endif
