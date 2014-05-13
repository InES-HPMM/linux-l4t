/*
 * tegra210_mvc.h - Tegra210 MVC driver header.
 *
 * Author: Ashok Mudithanapalli  <ashokm@nvidia.com>
 *
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TEGRA210_MVC_H
#define __TEGRA210_MVC_H

#include "tegra210_ahub_utils.h"

#define TEGRA210_MVC_COUNT		2
#define TEGRA210_MVC_MAX_CHANNEL_COUNT		8

/* RX Registers */
#define TEGRA210_MVC_RX_STATUS			0x0c
#define TEGRA210_MVC_RX_INT_STATUS		0x10
#define TEGRA210_MVC_RX_INT_MASK		0x14
#define TEGRA210_MVC_RX_INT_SET			0x18
#define TEGRA210_MVC_RX_INT_CLEAR		0x1c
#define TEGRA210_MVC_RX_CIF_CTRL		0x20


/* TX Registers */
#define TEGRA210_MVC_TX_STATUS			0x4c
#define TEGRA210_MVC_TX_INT_STATUS		0x50
#define TEGRA210_MVC_TX_INT_MASK		0x54
#define TEGRA210_MVC_TX_INT_SET			0x58
#define TEGRA210_MVC_TX_INT_CLEAR		0x5c
#define TEGRA210_MVC_TX_CIF_CTRL		0x60


/* MVC Common Registers */
#define TEGRA210_MVC_ENABLE				0x80
#define TEGRA210_MVC_SOFT_RESET			0x84
#define TEGRA210_MVC_CG					0x88
#define TEGRA210_MVC_STATUS				0x90
#define TEGRA210_MVC_INT_STATUS			0x94
#define TEGRA210_MVC_CTRL				0xa8
#define TEGRA210_MVC_SWITCH				0xac
#define TEGRA210_MVC_INIT_VOL_CH0		0xb0
#define TEGRA210_MVC_INIT_VOL_CH1		0xb4
#define TEGRA210_MVC_INIT_VOL_CH2		0xb8
#define TEGRA210_MVC_INIT_VOL_CH3		0xbc
#define TEGRA210_MVC_INIT_VOL_CH4		0xc0
#define TEGRA210_MVC_INIT_VOL_CH5		0xc4
#define TEGRA210_MVC_INIT_VOL_CH6		0xc8
#define TEGRA210_MVC_INIT_VOL_CH7		0xcc
#define TEGRA210_MVC_TARGET_VOL_CH0		0xd0
#define TEGRA210_MVC_TARGET_VOL_CH1		0xd4
#define TEGRA210_MVC_TARGET_VOL_CH2		0xd8
#define TEGRA210_MVC_TARGET_VOL_CH3		0xdc
#define TEGRA210_MVC_TARGET_VOL_CH4		0xe0
#define TEGRA210_MVC_TARGET_VOL_CH5		0xe4
#define TEGRA210_MVC_TARGET_VOL_CH6		0xe8
#define TEGRA210_MVC_TARGET_VOL_CH7		0xec
#define TEGRA210_MVC_DURATION			0xf0
#define TEGRA210_MVC_DURATION_INV		0xf4
#define TEGRA210_MVC_POLY_N1			0xf8
#define TEGRA210_MVC_POLY_N2			0xfc
#define TEGRA210_MVC_PEAK_CTRL			0x100
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL		0x104
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA		0x108
#define TEGRA210_MVC_PEAK_VALUE			0x10c
#define TEGRA210_MVC_CONFIG_ERR_TYPE	0x12c


/* Fields in TEGRA210_MVC_RX_STATUS */
#define TEGRA210_MVC_RX_STATUS_ACIF_FIFO_FULL_SHIFT	1
#define TEGRA210_MVC_RX_STATUS_ACIF_FIFO_FULL_MASK	\
	(0x1 << TEGRA210_MVC_RX_STATUS_ACIF_FIFO_FULL_SHIFT)

#define TEGRA210_MVC_RX_STATUS_ACIF_FIFO_EMPTY_SHIFT	0
#define TEGRA210_MVC_RX_STATUS_ACIF_FIFO_EMPTY_MASK		\
	BIT(0x1 << TEGRA210_MVC_RX_STATUS_ACIF_FIFO_EMPTY_SHIFT)

/* Fields in TEGRA210_MVC_RX_INT_STATUS */
#define TEGRA210_MVC_RX_INT_STATUS_SET	BIT(0)

/* Fields in  TEGRA210_MVC_RX_INT_MASK */
#define TEGRA210_MVC_RX_INT_MASK_SET	BIT(0)

/* Fields in  TEGRA210_MVC_RX_INT_SET */
#define TEGRA210_MVC_RX_INT_SET_EN	BIT(0)

/* Fields in TEGRA210_MVC_RX_INT_CLEAR */
#define TEGRA210_MVC_RX_INT_CLEAR_EN	BIT(0)

/* Fields in TEGRA210_MVC_TX_STATUS */
#define TEGRA210_MVC_TX_STATUS_ACIF_FIFO_FULL_SHIFT		1
#define TEGRA210_MVC_TX_STATUS_ACIF_FIFO_FULL_MASK  \
	(0x1 << TEGRA210_MVC_TX_STATUS_ACIF_FIFO_FULL_SHIFT)

#define TEGRA210_MVC_TX_STATUS_ACIF_FIFO_EMPTY_SHIFT	0
#define TEGRA210_MVC_TX_STATUS_ACIF_FIFO_EMPTY_MASK \
	BIT(0x1 << TEGRA210_MVC_TX_STATUS_ACIF_FIFO_EMPTY_SHIFT)

/* Fields in TEGRA210_MVC_TX_INT_STATUS */
#define TEGRA210_MVC_TX_INT_STATUS_SET	BIT(0)

/* Fields in  TEGRA210_MVC_TX_INT_MASK */
#define TEGRA210_MVC_TX_INT_MASK_SET	BIT(0)

/* Fields in  TEGRA210_MVC_TX_INT_SET */
#define TEGRA210_MVC_TX_INT_SET_EN	BIT(0)

/* Fields in TEGRA210_MVC_TX_INT_CLEAR */
#define TEGRA210_MVC_TX_INT_CLEAR_EN	BIT(0)


/*Fields in common registers*/

/*Fields in TEGRA210_MVC_ENABLE */
#define TEGRA210_MVC_ENABLE_EN		BIT(0)

/*Fields in TEGRA210_MVC_SOFT_RESET */
#define TEGRA210_MVC_SOFT_RESET_EN		BIT(0)

/*Fields in TEGRA210_MVC_CG*/
#define TEGRA210_MVC_CG_SLCG_EN		BIT(0)

/*Fields in TEGRA210_MVC_STATUS */
#define TEGRA210_MVC_STATUS_ENABLE_STATUS		BIT(0)
#define TEGRA210_MVC_STATUS_SLCG_CLKEN			BIT(8)
#define TEGRA210_MVC_STATUS_CONFIG_ERROR		BIT(31)

/*Fields in TEGRA210_MVC_INT_STATUS */
#define TEGRA210_MVC_INT_STATUS_TX_DONE_SHIFT		0
#define TEGRA210_MVC_INT_STATUS_TX_DONE_MASK	\
	(0x1 << TEGRA210_MVC_INT_STATUS_TX_DONE_SHIFT)

#define TEGRA210_MVC_INT_STATUS_RX_DONE_SHIFT		1
#define TEGRA210_MVC_INT_STATUS_RX_DONE_MASK	\
	(0x1 << TEGRA210_MVC_INT_STATUS_RX_DONE_SHIFT)

/*Fields in TEGRA210_MVC_CTRL */
#define TEGRA210_MVC_CTRL_ROUNDING_TYPE_UNBIASED			BIT(0)

#define TEGRA210_MVC_CTRL_CURVE_TYPE_LINEAR		BIT(1)

#define TEGRA210_MVC_CTRL_MUTE_UNMUTE_SHIFT		8
#define TEGRA210_MVC_CTRL_MUTE_UNMUTE_MASK	\
	(0xff << TEGRA210_MVC_CTRL_MUTE_UNMUTE_SHIFT)

#define TEGRA210_MVC_CTRL_PER_CH_CTRL_ENABLE	BIT(30)

#define TEGRA210_MVC_CTRL_BYPASS_ENABLE		BIT(31)

/*Fields in TEGRA210_MVC_SWITCH */
#define TEGRA210_MVC_SWITCH_DURATION_SET	BIT(0)
#define TEGRA210_MVC_SWITCH_COEF_SET		BIT(1)
#define TEGRA210_MVC_SWITCH_VOLUME_SET		BIT(2)

/*Fields in TEGRA210_MVC_INIT_VOL */
#define TEGRA210_MVC_INIT_VOL_SHIFT		0
#define TEGRA210_MVC_INIT_VOL_MASK	\
	(0xffffffff << TEGRA210_MVC_INIT_VOL_SHIFT)

/*Fields in TEGRA210_MVC_TARGET_VOL */
#define TEGRA210_MVC_TARGET_VOL_SHIFT		0
#define TEGRA210_MVC_TARGET_VOL_MASK	\
	(0xffffffff << TEGRA210_MVC_TARGET_VOL_SHIFT)

/*Fields in TEGRA210_MVC_DURATION */
#define TEGRA210_MVC_DURATION_SHIFT		0
#define TEGRA210_MVC_DURATION_MASK	\
	(0xffffff << TEGRA210_MVC_DURATION_SHIFT)

/*Fields in TEGRA210_MVC_DURATION_INV */
#define TEGRA210_MVC_DURATION_INV_SHIFT		0
#define TEGRA210_MVC_DURATION_INV_MASK		\
	(0xffffffff << TEGRA210_MVC_DURATION_INV_SHIFT)

/*Fields in TEGRA210_MVC_POLY_N1 */
#define TEGRA210_MVC_POLY_N1_SHIFT		0
#define TEGRA210_MVC_POLY_N1_MASK	(0xffffff << TEGRA210_MVC_POLY_N1_SHIFT)

/*Fields in TEGRA210_MVC_POLY_N2 */
#define TEGRA210_MVC_POLY_N2_SHIFT		0
#define TEGRA210_MVC_POLY_N2_MASK	(0xffffff << TEGRA210_MVC_POLY_N1_SHIFT)

/*Fields in TEGRA210_MVC_PEAK_CTRL */
#define TEGRA210_MVC_PEAK_CTRL_PEAK_WINDOW_SIZE_SHIFT	0
#define TEGRA210_MVC_PEAK_CTRL_PEAK_WINDOW_SIZE_MASK	\
	(0xffffff << TEGRA210_MVC_PEAK_CTRL_PEAK_WINDOW_SIZE_SHIFT)

#define TEGRA210_MVC_PEAK_CTRL_PEAK_TYPE_SHIFT		31
#define TEGRA210_MVC_PEAK_CTRL_PEAK_TYPE_MASK	\
	(0x1 << TEGRA210_MVC_PEAK_CTRL_PEAK_TYPE_SHIFT)


/*Fields in TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL */
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_RAM_ADDRS_SHIFT		0
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_RAM_ADDRS_MASK		\
	(0x1ff << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_RAM_ADDRS_SHIFT)

#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_SEQ_ACESS_EN_SHIFT	12
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_SEQ_ACESS_EN_MASK	\
	(0x1 << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_SEQ_ACESS_EN_SHIFT)

#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_ADDRS_INIT_EN_SHIFT	13
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_ADDRS_INIT_EN_MASK	\
	(0x1 << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_ADDRS_INIT_EN_SHIFT)

#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_RW_SHIFT		14
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_RW_MASK		\
	(0x1 << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_RW_SHIFT)

#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_SEQ_READ_COUNT_SHIFT	\
	16
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_SEQ_READ_COUNT_MASK \
	(0xff << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_SEQ_READ_COUNT_SHIFT)

#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_READ_BUSY_SHIFT		31
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_READ_BUS_MASK		\
	(0x1 << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_CTRL_READ_BUSY_SHIFT)

/*Fields in TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA */
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA_SHIFT		0
#define TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA_MASK		\
	(0xffffffff << TEGRA210_MVC_AHUBRAMCTL_CONFIG_RAM_DATA_SHIFT)

/*Fields in TEGRA210_MVC_PEAK_VALUE */
#define TEGRA210_MVC_PEAK_VALUE_SHIFT		0
#define TEGRA210_MVC_PEAK_VALUE_MASK		\
	((0xffffffff << TEGRA210_MVC_PEAK_VALUE_SHIFT))

/*Fields in TEGRA210_MVC_CONFIG_ERR_TYPE */
#define TEGRA210_MVC_CONFIG_ERR_TYPE_DURATION_CONFIG_ERROR  BIT(0)
#define TEGRA210_MVC_CONFIG_ERR_TYPE_COEF_CONFIG_ERROR		BIT(1)
#define TEGRA210_MVC_CONFIG_ERR_TYPE_VOLUME_CONFIG_ERROR	BIT(2)

enum T210_MVC_PEAK_TYPE {
	MVC_PEAK_WINDOW_BASED = 0,
	MVC_PEAK_RESET_ON_READ = 1,
};

#define TEGRA210_MVC_MAX_PEAK_WIN_SIZE	((2<<24)-1)
#define TEGRA210_MVC_NUMBER_OF_COEFS		9


enum mvc_switch_type {
	DURATION_SWITCH = 1,
	COEFF_SWITCH = 2,
	VOLUME_SWITCH = 4,

	SWITCH_ALL = 7,
};

enum MVC_CONFIG_ERROR {
	VOLUME_CONFIG_ERROR,
	COEFF_CONFIG_ERROR,
	DURATION_CONFIG_ERROR,
};

enum T210_MVC_RND_TYPE {
	MVC_RND_BIASED = 0,
	MVC_RND_UN_BIASED = 1,
};

/* MVC APIs */
int tegra210_mvc_get(enum tegra210_ahub_cifs *cif);
int tegra210_mvc_put(enum tegra210_ahub_cifs cif);
int tegra210_mvc_enable(enum tegra210_ahub_cifs cif, bool en);
int tegra210_mvc_reset(enum tegra210_ahub_cifs cif);
int tegra210_mvc_set_slcg(enum tegra210_ahub_cifs cif, bool en);
int tegra210_mvc_is_enable(enum tegra210_ahub_cifs cif);
int tegra210_mvc_is_slcg_enable(enum tegra210_ahub_cifs cif);
int tegra210_mvc_is_config_err(enum tegra210_ahub_cifs cif);
int tegra210_mvc_config_err_type(enum tegra210_ahub_cifs cif);
int tegra210_mvc_set_acif_param(enum tegra210_ahub_cifs cif,
	struct tegra210_axbar_cif_param *acif);
int tegra210_mvc_get_acif_param(enum tegra210_ahub_cifs cif,
	struct tegra210_axbar_cif_param *acif);

/* Peak meter related api's */
int tegra210_mvc_rx_set_peak_ctrl(enum tegra210_ahub_cifs cif, int type,
	unsigned int win_size);
int tegra210_mvc_rx_get_peak_values(enum tegra210_ahub_cifs cif, u32 *peak);

/* api's for register TEGRA210_MVC_CTRL */
int tegra210_mvc_enable_bypass(enum tegra210_ahub_cifs cif, bool en);
int tegra210_mvc_enable_per_ch_ctrl(enum tegra210_ahub_cifs cif, bool en);
int tegra210_mvc_mute_unmute_ctrl(enum tegra210_ahub_cifs cif, u8 ctrlbits);
int tegra210_mvc_curve_type(enum tegra210_ahub_cifs cif, bool en);
int tegra210_mvc_rounding_type(enum tegra210_ahub_cifs cif, bool en);

/* apis for register TEGRA210_MVC_SWITCH */
int tegra210_mvc_set_switch(enum tegra210_ahub_cifs cif,
	enum mvc_switch_type switch_type, u8 switch_data);
bool tegra210_mvc_is_switch_done(enum tegra210_ahub_cifs cif,
	enum mvc_switch_type switch_type);

/* apis for register TEGRA210_MVC_INIT_VOL & TEGRA210_MVC_TARGET_VOL*/
int tegra210_mvc_set_init_vol(enum tegra210_ahub_cifs cif, int *init_vol);
int tegra210_mvc_set_target_vol(enum tegra210_ahub_cifs cif, int *target_vol);

/* Duration API's */
int tegra210_mvc_set_duration(enum tegra210_ahub_cifs cif,
	u32 number_of_samples);
int tegra210_mvc_set_inv_duration(enum tegra210_ahub_cifs cif,
	u32 number_of_samples);
int tegra210_mvc_set_poly_curve_split_points(enum tegra210_ahub_cifs cif,
	u32 split_points[2]);

/* api for setting coefs */
int tegra210_mvc_set_poly_curve_coefs(enum tegra210_ahub_cifs cif, u32 *coef);
#endif
