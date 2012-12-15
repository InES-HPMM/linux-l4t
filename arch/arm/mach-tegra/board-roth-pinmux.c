/*
 * arch/arm/mach-tegra/board-roth-pinmux.c
 *
 * Copyright (C) 2012 NVIDIA Corporation
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
#include "board-roth.h"
#include "devices.h"
#include "gpio-names.h"

#include <mach/pinmux-t11.h>

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * drive_type - Drive type to be used depending on the resistors.
 */

#define SET_DRIVE_WITH_TYPE(_name, _hsm, _schmitt, _drive, _pulldn_drive,\
		_pullup_drive, _pulldn_slew, _pullup_slew, _drive_type)	\
	{								\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,		\
		.hsm = TEGRA_HSM_##_hsm,				\
		.schmitt = TEGRA_SCHMITT_##_schmitt,			\
		.drive = TEGRA_DRIVE_##_drive,				\
                .pull_down = TEGRA_PULL_##_pulldn_drive,		\
		.pull_up = TEGRA_PULL_##_pullup_drive,			\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,		\
		.slew_falling = TEGRA_SLEW_##_pullup_slew,		\
		.drive_type = TEGRA_DRIVE_TYPE_##_drive_type,		\
	}

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define DDC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _rcv_sel) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.rcv_sel	= TEGRA_PIN_RCV_SEL_##_rcv_sel,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

#define CEC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od)   \
	{                                                               \
		.pingroup   = TEGRA_PINGROUP_##_pingroup,                   \
		.func       = TEGRA_MUX_##_mux,                             \
		.pupd       = TEGRA_PUPD_##_pupd,                           \
		.tristate   = TEGRA_TRI_##_tri,                             \
		.io         = TEGRA_PIN_##_io,                              \
		.lock       = TEGRA_PIN_LOCK_##_lock,                       \
		.od         = TEGRA_PIN_OD_##_od,                           \
		.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,                   \
	}

#define USB_PINMUX CEC_PINMUX

#define GPIO_INIT_PIN_MODE(_gpio, _is_input, _value)	\
	{					\
		.gpio_nr	= _gpio,	\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

static __initdata struct tegra_drive_pingroup_config roth_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SDMMC1 */
	SET_DRIVE(SDIO1, DISABLE, DISABLE, DIV_1, 36, 20, SLOW, SLOW),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, DISABLE, DISABLE, DIV_1, 22, 36, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, DISABLE, DISABLE, DIV_1, 2, 1, FASTEST, FASTEST, 1),
};

/* Initially setting all used GPIO's to non-TRISTATE */
static __initdata struct tegra_pingroup_config roth_pinmux_set_nontristate[] = {
	DEFAULT_PINMUX(GPIO_X4_AUD,     RSVD,   PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_X5_AUD,     RSVD,   PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_X6_AUD,     RSVD3,  PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_X7_AUD,     RSVD,   PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_W2_AUD,     RSVD1,  PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_W3_AUD,     SPI6,   PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_X1_AUD,     RSVD3,  PULL_DOWN,    NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_X3_AUD,     RSVD3,  PULL_UP,      NORMAL,    INPUT),

	DEFAULT_PINMUX(DAP3_FS,         I2S2,   PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(DAP3_DIN,        I2S2,   PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(DAP3_DOUT,       I2S2,   PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(DAP3_SCLK,       I2S2,   PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PV0,        RSVD3,  NORMAL,       NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_PV1,        RSVD,   NORMAL,       NORMAL,    INPUT),

	DEFAULT_PINMUX(GPIO_PBB3,       RSVD3,  PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB5,       RSVD3,  PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB6,       RSVD3,  PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PBB7,       RSVD3,  PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PCC1,       RSVD3,  PULL_DOWN,    NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_PCC2,       RSVD3,  PULL_DOWN,    NORMAL,    INPUT),

	DEFAULT_PINMUX(GMI_AD0,         GMI,    NORMAL,       NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_AD1,         GMI,    NORMAL,       NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_AD10,        GMI,    PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_AD11,        GMI,    PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_AD12,        GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_AD13,        GMI,    PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_AD2,         GMI,    NORMAL,       NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_AD3,         GMI,    NORMAL,       NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_AD8,         GMI,    PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_ADV_N,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_CLK,         GMI,    PULL_DOWN,    NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_CS0_N,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_CS2_N,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_CS3_N,       GMI,    PULL_UP,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GMI_CS4_N,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_CS7_N,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_DQS_P,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_IORDY,       GMI,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GMI_WP_N,        GMI,    PULL_UP,      NORMAL,    INPUT),

	DEFAULT_PINMUX(SDMMC1_WP_N,     SPI4,   PULL_UP,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(CLK2_REQ,        RSVD3,  NORMAL,       NORMAL,    OUTPUT),

	DEFAULT_PINMUX(KB_COL3,         PWM2,   NORMAL,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(KB_COL5,         KBC,    PULL_UP,      NORMAL,    INPUT),
	DEFAULT_PINMUX(KB_COL6,         KBC,    PULL_UP,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(KB_COL7,         KBC,    PULL_UP,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(KB_ROW3,         KBC,    PULL_DOWN,    NORMAL,    INPUT),
	DEFAULT_PINMUX(KB_ROW4,         KBC,    PULL_DOWN,    NORMAL,    INPUT),
	DEFAULT_PINMUX(KB_ROW6,         KBC,    PULL_DOWN,    NORMAL,    INPUT),
	DEFAULT_PINMUX(KB_ROW8,         KBC,    PULL_UP,    NORMAL,    INPUT),

	DEFAULT_PINMUX(CLK3_REQ,        RSVD3,  NORMAL,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PU4,        RSVD3,  NORMAL,      NORMAL,    OUTPUT),
	DEFAULT_PINMUX(GPIO_PU5,        RSVD3,  NORMAL,      NORMAL,    INPUT),
	DEFAULT_PINMUX(GPIO_PU6,        RSVD3,  NORMAL,      NORMAL,    INPUT),

	DEFAULT_PINMUX(GPIO_PU3,        PWM0,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(HDMI_INT,        RSVD,   PULL_DOWN,    NORMAL,    INPUT),

	DEFAULT_PINMUX(GMI_AD9,         PWM1,   NORMAL,    NORMAL,     OUTPUT),
};

#include "board-roth-pinmux-t11x.h"

static void __init roth_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_roth_common);
	pins_info = init_gpio_mode_roth_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init roth_pinmux_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);

	tegra_pinmux_config_table(roth_pinmux_set_nontristate,
					ARRAY_SIZE(roth_pinmux_set_nontristate));
	roth_gpio_init_configure();

	tegra_pinmux_config_table(roth_pinmux_common, ARRAY_SIZE(roth_pinmux_common));
	tegra_drive_pinmux_config_table(roth_drive_pinmux,
					ARRAY_SIZE(roth_drive_pinmux));
	tegra_pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	return 0;
}
