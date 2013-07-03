/*
 * linux/arch/arm/mach-tegra/pinmux-t14-tables.c
 *
 * Common pinmux configurations for Tegra14x SoCs
 *
 * Copyright (C) 2012 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/syscore_ops.h>
#include <linux/bug.h>
#include <linux/bitops.h>

#include <mach/pinmux.h>
#include <mach/pinmux-t14.h>
#include "gpio-names.h"
#include "iomap.h"

static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

#define TRISTATE		(1<<4)
#define PMC_DPD_SAMPLE		0x20
#define PMC_IO_DPD_REQ_0	0x1B8
#define PMC_IO_DPD2_REQ_0	0x1C0

#define PINGROUP_REG_A	0x868
#define MUXCTL_REG_A	0x3000

#define SET_DRIVE_PINGROUP(pg_name, r, drv_down_offset, drv_down_mask, drv_up_offset, drv_up_mask,	\
	slew_rise_offset, slew_rise_mask, slew_fall_offset, slew_fall_mask)	\
	[TEGRA_DRIVE_PINGROUP_ ## pg_name] = {			\
		.name = #pg_name,				\
		.reg_bank = 0,					\
		.reg = ((r) - PINGROUP_REG_A),			\
		.drvup_offset = drv_up_offset,			\
		.drvup_mask = drv_up_mask,			\
		.drvdown_offset = drv_down_offset,		\
		.drvdown_mask = drv_down_mask,			\
		.slewrise_offset = slew_rise_offset,		\
		.slewrise_mask = slew_rise_mask,		\
		.slewfall_offset = slew_fall_offset,		\
		.slewfall_mask = slew_fall_mask,		\
	}

#define DEFAULT_DRIVE_PINGROUP(pg_name, r)		\
	[TEGRA_DRIVE_PINGROUP_ ## pg_name] = {		\
		.name = #pg_name,			\
		.reg_bank = 0,				\
		.reg = ((r) - PINGROUP_REG_A),		\
		.drvup_offset = 20,			\
		.drvup_mask = 0x1f,			\
		.drvdown_offset = 12,			\
		.drvdown_mask = 0x1f,			\
		.slewrise_offset = 28,			\
		.slewrise_mask = 0x3,			\
		.slewfall_offset = 30,			\
		.slewfall_mask = 0x3,			\
	}

const struct tegra_drive_pingroup_desc tegra_soc_drive_pingroups[TEGRA_MAX_DRIVE_PINGROUP] = {
	DEFAULT_DRIVE_PINGROUP(AO1,		0x868),
	DEFAULT_DRIVE_PINGROUP(AO2,		0x86c),
	DEFAULT_DRIVE_PINGROUP(CDEV1,		0x884),
	DEFAULT_DRIVE_PINGROUP(CDEV2,		0x888),
	DEFAULT_DRIVE_PINGROUP(CSUS,		0x88c),
	DEFAULT_DRIVE_PINGROUP(DAP1,		0x890),
	DEFAULT_DRIVE_PINGROUP(DAP2,		0x894),
	DEFAULT_DRIVE_PINGROUP(DBG,		0x8a0),
	DEFAULT_DRIVE_PINGROUP(SDIO3,		0x8b0),
	DEFAULT_DRIVE_PINGROUP(UART2,		0x8c0),
	DEFAULT_DRIVE_PINGROUP(UART3,		0x8c4),
	DEFAULT_DRIVE_PINGROUP(PAD,		0x8e8),
	DEFAULT_DRIVE_PINGROUP(SDIO1,		0x8ec),
	DEFAULT_DRIVE_PINGROUP(DDC,		0x8fc),
	DEFAULT_DRIVE_PINGROUP(GMA,		0x900),
	SET_DRIVE_PINGROUP(GME,		0x910,	14,	0x1f,	19,	0x1f,
	28,	0x3,	30,	0x3),
	DEFAULT_DRIVE_PINGROUP(OWR,		0x920),
	DEFAULT_DRIVE_PINGROUP(CEC,		0x938),
	DEFAULT_DRIVE_PINGROUP(DAP5,		0x998),
	DEFAULT_DRIVE_PINGROUP(DMIC0,		0x9a0),
	DEFAULT_DRIVE_PINGROUP(DMIC1,		0x9a4),
	DEFAULT_DRIVE_PINGROUP(AO3,		0x9a8),
	DEFAULT_DRIVE_PINGROUP(SPI2,		0x9ac),
	DEFAULT_DRIVE_PINGROUP(AO0,		0x9b0),
	DEFAULT_DRIVE_PINGROUP(DCA,		0x9b8),
	DEFAULT_DRIVE_PINGROUP(SPI3,		0x9bc),

};

#define PINGROUP(pg_name, gpio_nr, vdd, f0, f1, f2, f3, fs, iod, reg)	\
	[TEGRA_PINGROUP_ ## pg_name] = {			\
		.name = #pg_name,				\
		.vddio = TEGRA_VDDIO_ ## vdd,			\
		.funcs = {					\
			TEGRA_MUX_ ## f0,			\
			TEGRA_MUX_ ## f1,			\
			TEGRA_MUX_ ## f2,			\
			TEGRA_MUX_ ## f3,			\
		},						\
		.gpionr = TEGRA_GPIO_ ## gpio_nr,		\
		.func_safe = TEGRA_MUX_ ## fs,			\
		.tri_bank = 1,					\
		.tri_reg = ((reg) - MUXCTL_REG_A),		\
		.tri_bit = 4,					\
		.mux_bank = 1,					\
		.mux_reg = ((reg) - MUXCTL_REG_A),		\
		.mux_bit = 0,					\
		.pupd_bank = 1,					\
		.pupd_reg = ((reg) - MUXCTL_REG_A),		\
		.pupd_bit = 2,					\
		.io_default = TEGRA_PIN_ ## iod,		\
		.od_bit = 6,					\
		.lock_bit = 7,					\
		.ioreset_bit = 8,				\
	}

/* !!!FIXME!!! FILL IN fSafe COLUMN IN TABLE ....... */
#define PINGROUPS	\
	/*       NAME		  GPIO		VDD	    f0		f1          f2          f3           fSafe       io	reg */\
	PINGROUP(SDMMC1_CLK,	  PA0,		SDMMC1,     SDMMC1,	RSVD1,	    RSVD2,	RSVD3,       RSVD,	INPUT,	0x3048),\
	PINGROUP(SDMMC1_CMD,	  PA1,		SDMMC1,     SDMMC1,	RSVD1,	    RSVD2,	RSVD3,       RSVD,	INPUT,	0x304c),\
	PINGROUP(SDMMC1_DAT3,	  PA2,		SDMMC1,     SDMMC1,	RSVD1,	    RSVD2,	RSVD3,       RSVD,	INPUT,	0x3050),\
	PINGROUP(SDMMC1_DAT2,	  PA3,		SDMMC1,     SDMMC1,	RSVD1,	    RSVD2,	RSVD3,       RSVD,	INPUT,	0x3054),\
	PINGROUP(SDMMC1_DAT1,	  PA4,		SDMMC1,     SDMMC1,	RSVD1,	    RSVD2,	RSVD3,       RSVD,	INPUT,	0x3058),\
	PINGROUP(SDMMC1_DAT0,	  PA5,		SDMMC1,     SDMMC1,	RSVD1,	    RSVD2,	RSVD3,       RSVD,	INPUT,	0x305c),\
	PINGROUP(DDC_SCL,	  PN6,		UART,	    I2C4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3114),\
	PINGROUP(DDC_SDA,	  PN7,		UART,	    I2C4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3118),\
	PINGROUP(UART2_RXD,	  PF6,		AUDIO,	    UARTB,	I2S3,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3164),\
	PINGROUP(UART2_TXD,	  PF5,		AUDIO,	    UARTB,	I2S3,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3168),\
	PINGROUP(UART2_RTS_N,	  PF7,		AUDIO,	    UARTB,	I2S3,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x316c),\
	PINGROUP(UART2_CTS_N,	  PG0,		AUDIO,	    UARTB,	I2S3,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3170),\
	PINGROUP(UART3_TXD,	  PK0,		UART,	    UARTC,	SPI2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3174),\
	PINGROUP(UART3_RXD,	  PK1,		UART,	    UARTC,	SPI2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3178),\
	PINGROUP(UART3_CTS_N,	  PK3,		UART,	    UARTC,	SPI2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x317c),\
	PINGROUP(UART3_RTS_N,	  PK2,		UART,	    UARTC,	SPI2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3180),\
	PINGROUP(GEN2_I2C_SCL,	  PL5,		UART,	    I2C2,	UARTD,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3250),\
	PINGROUP(GEN2_I2C_SDA,	  PL6,		UART,	    I2C2,	UARTD,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3254),\
	PINGROUP(SDMMC4_CLK,	  PC0,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3258),\
	PINGROUP(SDMMC4_CMD,	  PC1,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x325c),\
	PINGROUP(SDMMC4_DAT0,	  PC2,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3260),\
	PINGROUP(SDMMC4_DAT1,	  PC3,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3264),\
	PINGROUP(SDMMC4_DAT2,	  PC4,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3268),\
	PINGROUP(SDMMC4_DAT3,	  PC5,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x326c),\
	PINGROUP(SDMMC4_DAT4,	  PC6,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3270),\
	PINGROUP(SDMMC4_DAT5,	  PC7,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3274),\
	PINGROUP(SDMMC4_DAT6,	  PD0,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3278),\
	PINGROUP(SDMMC4_DAT7,	  PD1,		SDMMC4,     SDMMC4,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x327c),\
	PINGROUP(CAM_I2C_SCL,	  PR2,		CAM,	    I2C3,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3290),\
	PINGROUP(CAM_I2C_SDA,	  PR3,		CAM,	    I2C3,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3294),\
	PINGROUP(JTAG_RTCK,	  INVALID,	SYS,	    RTCK,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x32b0),\
	PINGROUP(PWR_I2C_SCL,	  PJ7,		SYS,	    I2CPWR,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x32b4),\
	PINGROUP(PWR_I2C_SDA,	  PP0,		SYS,	    I2CPWR,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x32b8),\
	PINGROUP(KB_ROW0,	  PJ1,		SYS,	    KBC,	RSVD1,	    RSVD2,	RSVD3,	     KBC,	INPUT,	0x32bc),\
	PINGROUP(KB_ROW1,	  PJ2,		SYS,	    KBC,	RSVD1,	    RSVD2,	RSVD3,	     KBC,	INPUT,	0x32c0),\
	PINGROUP(KB_ROW2,	  PJ3,		SYS,	    KBC,	RSVD1,	    RSVD2,	RSVD3,	     KBC,	INPUT,	0x32c4),\
	PINGROUP(KB_COL0,	  PJ4,	 	SYS,	    KBC,	RSVD1,	    RSVD2,	RSVD3,       KBC,	INPUT,	0x32fc),\
	PINGROUP(KB_COL1,	  PJ5,		SYS,	    KBC,	RSVD1,	    RSVD2,	RSVD3,       KBC,	INPUT,	0x3300),\
	PINGROUP(KB_COL2,	  PJ6,		SYS,	    KBC,	RSVD1,	    RSVD2,	RSVD3,	     KBC,	INPUT,	0x3304),\
	PINGROUP(RF_CLK_REQ,	  INVALID,	SYS,	    SYSCLK,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3320),\
	PINGROUP(SOC_PWR_REQ,	  INVALID,	SYS,	    SOC,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3324),\
	PINGROUP(CPU_PWR_REQ,	  INVALID,	SYS,	    CPU,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3328),\
	PINGROUP(PWR_INT_N,	  INVALID,	SYS,	    PMI,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x332c),\
	PINGROUP(CLK_32K_IN,	  INVALID,	SYS,	    CLK,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3330),\
	PINGROUP(DAP1_FS,	  PE0,		AUDIO,      I2S0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3338),\
	PINGROUP(DAP1_DIN,	  PE1,		AUDIO,      I2S0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x333c),\
	PINGROUP(DAP1_DOUT,	  PE2,		AUDIO,      I2S0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3340),\
	PINGROUP(DAP1_SCLK,	  PE3,		AUDIO,      I2S0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3344),\
	PINGROUP(CLK1_OUT,	  PF0,		AUDIO,      EXTPERIPH1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x334c),\
	PINGROUP(DAP2_FS,	  PK4,		UART,      I2S1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3358),\
	PINGROUP(DAP2_DIN,	  PK6,		UART,      I2S1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x335c),\
	PINGROUP(DAP2_DOUT,	  PK7,		UART,      I2S1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3360),\
	PINGROUP(DAP2_SCLK,	  PK5,		UART,      I2S1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3364),\
	PINGROUP(SPI2_MOSI,	  PH2,		AUDIO,      SPI2,	DTV,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3368),\
	PINGROUP(SPI2_MISO,	  PH3,		AUDIO,      SPI2,	DTV,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x336c),\
	PINGROUP(SPI2_CS0_N,	  PH5,		AUDIO,      SPI2,	DTV,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3370),\
	PINGROUP(SPI2_SCK,	  PH4,		AUDIO,      SPI2,	DTV,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3374),\
	PINGROUP(SPI2_CS1_N,	  PH6,		AUDIO,      SPI2,	SPI1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3388),\
	PINGROUP(SPI2_CS2_N,	  PH1,		AUDIO,      SPI2,	SPI1,	    RSVD2,	RSVD3,	     SPI2,	INPUT,	0x338c),\
	PINGROUP(SDMMC3_CLK,	  PB0,		SDMMC3,     SDMMC3,	TRACE,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3390),\
	PINGROUP(SDMMC3_CMD,	  PB1,		SDMMC3,     SDMMC3,	TRACE,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3394),\
	PINGROUP(SDMMC3_DAT0,	  PB5,		SDMMC3,     SDMMC3,	TRACE,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3398),\
	PINGROUP(SDMMC3_DAT1,	  PB4,		SDMMC3,     SDMMC3,	TRACE,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x339c),\
	PINGROUP(SDMMC3_DAT2,	  PB3,		SDMMC3,     SDMMC3,	TRACE,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x33a0),\
	PINGROUP(SDMMC3_DAT3,	  PB2,		SDMMC3,     SDMMC3,	TRACE,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x33a4),\
	PINGROUP(RESET_OUT_N,	  INVALID,	SYS,        RESET_OUT_N,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	OUTPUT,	0x3408),\
	PINGROUP(ALS_PROX_INT_L,  PN0,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x340c),\
	PINGROUP(AP_GPS_EN,	  PH7,		AUDIO,      RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3410),\
	PINGROUP(AP_GPS_RST,	  PI0,		AUDIO,      RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3414),\
	PINGROUP(AP_WAKE_BT,	  PM1,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3418),\
	PINGROUP(AP_WIFI_EN,	  PL7,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x341c),\
	PINGROUP(AP_WIFI_RST,	  PM0,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3420),\
	PINGROUP(BT_RESET_L,	  PM3,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3428),\
	PINGROUP(BT_WAKE_AP,	  PM2,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x342c),\
	PINGROUP(DAP5_DIN,	  PG6,		AUDIO,      I2S4,	SPI1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3430),\
	PINGROUP(DAP5_DOUT,	  PG7,		AUDIO,      I2S4,	SPI1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3434),\
	PINGROUP(DAP5_FS,	  PG5,		AUDIO,      I2S4,	SPI1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3438),\
	PINGROUP(DAP5_SCLK,	  PH0,		AUDIO,      I2S4,	SPI1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x343c),\
	PINGROUP(DCA_LPM,	  PG2,		AUDIO,      DISPLAYA,	DISPLAYB,   PWM0,	RSVD3,	     RSVD,	INPUT,	0x3440),\
	PINGROUP(DCA_LSC,	  PG3,		AUDIO,      DISPLAYA,	DISPLAYB,   RSVD2,	RSVD3,	     DISPLAYA,	INPUT,	0x3444),\
	PINGROUP(DCA_LSPII,	  PG1,		AUDIO,      DISPLAYA,	DISPLAYB,   RSVD2,	RSVD3,	     DISPLAYA,	INPUT,	0x3448),\
	PINGROUP(DCA_LVS,	  PG4,		AUDIO,      DISPLAYA,	DISPLAYB,   RSVD2,	RSVD3,	     DISPLAYA, 	INPUT,	0x344c),\
	PINGROUP(DMIC0_CLK,	  PE4,		AUDIO,      DMIC0,	I2S2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3450),\
	PINGROUP(DMIC0_DATA,	  PE5,		AUDIO,      DMIC0,	I2S2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3454),\
	PINGROUP(DMIC1_CLK,	  PE6,		AUDIO,      DMIC1,	I2S2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3458),\
	PINGROUP(DMIC1_DATA,	  PE7,		AUDIO,      DMIC1,	I2S2,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x345c),\
	PINGROUP(GPS_AP_INT,	  PI1,		AUDIO,      RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3460),\
	PINGROUP(AP_READY,	  PN4,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3464),\
	PINGROUP(HDMI_CEC,	  PN5,		UART,       CEC,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3464),\
	PINGROUP(I2C1_SCL,	  PF1,		AUDIO,      I2C1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3468),\
	PINGROUP(I2C1_SDA,	  PF2,		AUDIO,      I2C1,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x346c),\
	PINGROUP(I2C6_SCL,	  PF3,		AUDIO,      I2C6,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3470),\
	PINGROUP(I2C6_SDA,	  PF4,		AUDIO,      I2C6,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3474),\
	PINGROUP(MODEM_WAKE_AP,	  PN3,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x347c),\
	PINGROUP(MOTION_INT_L,	  PM7,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3480),\
	PINGROUP(NFC_DATA_EN,	  PM6,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3484),\
	PINGROUP(NFC_EN,	  PM5,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3488),\
	PINGROUP(NFC_INT_L,	  PM4,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x348c),\
	PINGROUP(SPI3_CS0_N,	  PL3,		UART,       SPI3,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3494),\
	PINGROUP(SPI3_CS1_N,	  PL4,		UART,       SPI3,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3498),\
	PINGROUP(SPI3_MISO,	  PL1,		UART,       SPI3,	UARTD,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x349c),\
	PINGROUP(SPI3_MOSI,	  PL0,		UART,       SPI3,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x34a0),\
	PINGROUP(SPI3_SCK,	  PL2,		UART,       SPI3,	UARTD,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x34a4),\
	PINGROUP(TOUCH_INT_L,	  PN1,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x34a8),\
	PINGROUP(TOUCH_RESET_L,	  PN2,		UART,       RSVD0,	RSVD1,	    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x34ac),\
	PINGROUP(GPIO_PO0,	  PO0,		UART,       UARTD,	RSVD1,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34b0),\
	PINGROUP(GPIO_PO1,	  PO1,		UART,       UARTD,	RSVD1,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34b4),\
	PINGROUP(GPIO_PO2,	  PO2,		UART,       UARTD,	RSVD1,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34b8),\
	PINGROUP(GPIO_PO3,	  PO3,		UART,       UARTD,	RSVD1,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34bc),\
	PINGROUP(CAM1_MCLK,	  PR0,		CAM,        VI,		VI_ALT1,    VI_ALT3,	RSVD3,	     RSVD,	INPUT,	0x34c0),\
	PINGROUP(CAM2_MCLK,	  PR1,		CAM,        VIMCLK2,	VIMCLK2_ALT, VIMCLK2_ALT_ALT, RSVD3, RSVD,	INPUT,	0x34c4),\
	PINGROUP(GPIO_PS0,	  PS0,		CAM,        VGP1,	RSVD1,	    RSVD2,	RSVD3,	     VGP1,	INPUT,	0x34c8),\
	PINGROUP(GPIO_PS1,	  PS1,		CAM,        VGP2,	RSVD1,	    RSVD2,	RSVD3,	     VGP2,	INPUT,	0x34cc),\
	PINGROUP(GPIO_PS2,	  PS2,		CAM,        VGP3,	RSVD1,	    RSVD2,	RSVD3,	     VGP3,	INPUT,	0x34d0),\
	PINGROUP(GPIO_PS3,	  PS3,		CAM,        VGP4,	RSVD1,	    RSVD2,	RSVD3,	     VGP4,	INPUT,	0x34d4),\
	PINGROUP(GPIO_PS4,	  PS4,		CAM,        VGP5,	RSVD1,	    RSVD2,	RSVD3,	     VGP5,	INPUT,	0x34d8),\
	PINGROUP(GPIO_PS5,	  PS5,		CAM,        VGP6,	RSVD1,	    RSVD2,	RSVD3,	     VGP6,	INPUT,	0x34dc),\
	PINGROUP(DDR0_A0,	  INVALID,	DDR,        DDR0,	UARTA,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34e4),\
	PINGROUP(DDR0_A1,	  INVALID,	DDR,        DDR0,	UARTA,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34e8),\
	PINGROUP(DDR0_A2,	  INVALID,	DDR,        DDR0,	UARTA,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34ec),\
	PINGROUP(DDR0_A3,	  INVALID,	DDR,        DDR0,	UARTA,	    RSVD2,	RSVD3,	     UARTD,	INPUT,	0x34f0),\
	PINGROUP(UART1_RXD,	  PQ1,		UART,       UARTA,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x34f4),\
	PINGROUP(UART1_TXD,	  PQ0,		UART,       UARTA,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x34f8),\
	PINGROUP(UART1_RTS_N,	  PQ2,		UART,       UARTA,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x34fc),\
	PINGROUP(UART1_CTS_N,	  PQ3,		UART,       UARTA,	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3500),\
	PINGROUP(GPIO_PO4,	  PO4,		UART,       SOC,	SOC_ALT,    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3504),\
	PINGROUP(GPIO_PO5,	  PO5,		UART,       SOC,	SOC_ALT,    RSVD2,	RSVD3,	     RSVD0,	INPUT,	0x3508),\
	PINGROUP(GPIO_PO6,	  PO6,		UART,       OWR,	SOC,	    SOC_ALT,	RSVD3,	     RSVD0,	INPUT,	0x350c),\
	PINGROUP(PG_OC,		  INVALID,	SYS,	    PG, 	RSVD1,	    RSVD2,	RSVD3,	     RSVD,	INPUT,	0x3510),\
	PINGROUP(BCL,		  PJ0,		SYS,	    BCL,	OWR,	    RSVD2,	RSVD3,	     BCL,	INPUT,	0x3514),

const struct tegra_pingroup_desc tegra_soc_pingroups[TEGRA_MAX_PINGROUP] = {
	PINGROUPS
};

#undef PINGROUP
#undef TEGRA_GPIO_INVALID
#define TEGRA_GPIO_INVALID	TEGRA_MAX_GPIO

#define PINGROUP(pg_name, gpio_nr, vdd, f0, f1, f2, f3, fs, iod, reg)	\
	[TEGRA_GPIO_##gpio_nr] =  TEGRA_PINGROUP_ ##pg_name\

const int gpio_to_pingroup[TEGRA_MAX_GPIO + 1] = {
	PINGROUPS

};

#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_##_hsm,			\
		.schmitt = TEGRA_SCHMITT_##_schmitt,		\
		.drive = TEGRA_DRIVE_##_drive,			\
		.pull_down = TEGRA_PULL_##_pulldn_drive,	\
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,	\
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

static __initdata struct tegra_drive_pingroup_config t14x_def_drive_pinmux[] = {
};

#ifdef CONFIG_PM_SLEEP

static u32 pinmux_reg[TEGRA_MAX_PINGROUP +
		      ARRAY_SIZE(tegra_soc_drive_pingroups)];

static int tegra14x_pinmux_suspend(void)
{
	unsigned int i;
	u32 *ctx = pinmux_reg;

	for (i = 0; i < TEGRA_MAX_PINGROUP; i++)
		*ctx++ = pg_readl(tegra_soc_pingroups[i].mux_bank,
				tegra_soc_pingroups[i].mux_reg);

	for (i = 0; i < ARRAY_SIZE(tegra_soc_drive_pingroups); i++)
		*ctx++ = pg_readl(tegra_soc_drive_pingroups[i].reg_bank,
				tegra_soc_drive_pingroups[i].reg);

	return 0;
}

static void tegra14x_pinmux_resume(void)
{
	unsigned int i;
	u32 *ctx = pinmux_reg;
	u32 *tmp = pinmux_reg;
	u32 reg_value;

	for (i = 0; i < TEGRA_MAX_PINGROUP; i++) {
		reg_value = *tmp++;
		reg_value |= TRISTATE;
		pg_writel(reg_value, tegra_soc_pingroups[i].mux_bank,
			tegra_soc_pingroups[i].mux_reg);
	}

	writel(0x400fffff, pmc + PMC_IO_DPD_REQ_0);
	writel(0x40001fff, pmc + PMC_IO_DPD2_REQ_0);

	for (i = 0; i < TEGRA_MAX_PINGROUP; i++)
		pg_writel(*ctx++, tegra_soc_pingroups[i].mux_bank,
			tegra_soc_pingroups[i].mux_reg);

	for (i = 0; i < ARRAY_SIZE(tegra_soc_drive_pingroups); i++)
		pg_writel(*ctx++, tegra_soc_drive_pingroups[i].reg_bank,
			tegra_soc_drive_pingroups[i].reg);

	/* Clear DPD sample */
	writel(0x0, pmc + PMC_DPD_SAMPLE);

}

static struct syscore_ops tegra14x_pinmux_syscore_ops = {
	.suspend = tegra14x_pinmux_suspend,
	.resume = tegra14x_pinmux_resume,
};
#endif

void tegra14x_pinmux_init(const struct tegra_pingroup_desc **pg,
		int *pg_max, const struct tegra_drive_pingroup_desc **pgdrive,
		int *pgdrive_max, const int **gpiomap, int *gpiomap_max)
{
	*pg = tegra_soc_pingroups;
	*pg_max = TEGRA_MAX_PINGROUP;
	*pgdrive = tegra_soc_drive_pingroups;
	*pgdrive_max = TEGRA_MAX_DRIVE_PINGROUP;
	*gpiomap = gpio_to_pingroup;
	*gpiomap_max = TEGRA_MAX_GPIO;

#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&tegra14x_pinmux_syscore_ops);
#endif
}

void tegra14x_default_pinmux(void)
{
	tegra_drive_pinmux_config_table(t14x_def_drive_pinmux,
					ARRAY_SIZE(t14x_def_drive_pinmux));
}
