/*
 * tegra210_dmic.h - Tegra210 DMIC driver header.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
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

#ifndef __TEGRA210_DMIC_H
#define __TEGRA210_DMIC_H

#include "tegra210_ahub_utils.h"

#define TEGRA210_DMIC_COUNT	3

/* Register offsets from DMIC BASE */
#define TEGRA210_DMIC_TX_STATUS				0x0c
#define TEGRA210_DMIC_TX_INT_STATUS			0x10
#define TEGRA210_DMIC_TX_INT_MASK			0x14
#define TEGRA210_DMIC_TX_INT_SET			0x18
#define TEGRA210_DMIC_TX_INT_CLEAR			0x1c
#define TEGRA210_DMIC_TX_CIF_CTRL			0x20

#define TEGRA210_DMIC_ENABLE				0x40
#define TEGRA210_DMIC_SOFT_RESET			0x44
#define TEGRA210_DMIC_CG				0x48
#define TEGRA210_DMIC_STATUS				0x4c
#define TEGRA210_DMIC_INT_STATUS			0x50
#define TEGRA210_DMIC_CTRL				0x64

#define TEGRA210_DMIC_DBG_CTRL				0x70
#define TEGRA210_DMIC_DCR_FILTER_GAIN			0x74
#define TEGRA210_DMIC_DCR_BIQUAD_0_COEF_0		0x78
#define TEGRA210_DMIC_DCR_BIQUAD_0_COEF_1		0x7c
#define TEGRA210_DMIC_DCR_BIQUAD_0_COEF_2		0x80
#define TEGRA210_DMIC_DCR_BIQUAD_0_COEF_3		0x84
#define TEGRA210_DMIC_DCR_BIQUAD_0_COEF_4		0x88
#define TEGRA210_DMIC_LP_FILTER_GAIN			0x8c
#define TEGRA210_DMIC_LP_BIQUAD_0_COEF_0		0x90
#define TEGRA210_DMIC_LP_BIQUAD_0_COEF_1		0x94
#define TEGRA210_DMIC_LP_BIQUAD_0_COEF_2		0x98
#define TEGRA210_DMIC_LP_BIQUAD_0_COEF_3		0x9c
#define TEGRA210_DMIC_LP_BIQUAD_0_COEF_4		0xa0
#define TEGRA210_DMIC_LP_BIQUAD_1_COEF_0		0xa4
#define TEGRA210_DMIC_LP_BIQUAD_1_COEF_1		0xa8
#define TEGRA210_DMIC_LP_BIQUAD_1_COEF_2		0xac
#define TEGRA210_DMIC_LP_BIQUAD_1_COEF_3		0xb0
#define TEGRA210_DMIC_LP_BIQUAD_1_COEF_4		0xb4
#define TEGRA210_DMIC_CORRECTION_FILTER_GAIN		0xb8
#define TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_0	0xbc
#define TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_1	0xc0
#define TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_2	0xc4
#define TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_3	0xc8
#define TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_4	0xcc
#define TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_0	0xd0
#define TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_1	0xd4
#define TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_2	0xd8
#define TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_3	0xdc
#define TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4	0xe0

/* Constants for DMIC */
#define TEGRA210_DMIC_OSR_64				0
#define TEGRA210_DMIC_OSR_128				1
#define TEGRA210_DMIC_OSR_256				2

/* Fields in TEGRA210_DMIC_ENABLE */
#define TEGRA210_DMIC_ENABLE_EN				BIT(0)

/* Fields in TEGRA210_DMIC_SOFT_RESET */
#define TEGRA210_DMIC_SOFT_RESET_EN			BIT(0)

/* Fields in TEGRA210_DMIC_CG */
#define TEGRA210_DMIC_CG_SLCG_EN			BIT(0)

/* Fields in TEGRA210_DMIC_STATUS */
#define TEGRA210_DMIC_STATUS_CONFIG_ERROR_SHIFT		31
#define TEGRA210_DMIC_STATUS_CONFIG_ERROR_MASK		(0x1 << TEGRA210_DMIC_STATUS_CONFIG_ERROR_SHIFT)

#define TEGRA210_DMIC_STATUS_SLCG_CLKEN_SHIFT		8
#define TEGRA210_DMIC_STATUS_SLCG_CLKEN_MASK		(0x1 << TEGRA210_DMIC_STATUS_SLCG_CLKEN_SHIFT)

#define TEGRA210_DMIC_STATUS_ENABLE_STATUS_SHIFT	0
#define TEGRA210_DMIC_STATUS_ENABLE_STATUS_MASK		(0x1 << TEGRA210_DMIC_STATUS_ENABLE_STATUS_SHIFT)

/* Fields in TEGRA210_DMIC_CTRL */
#define TEGRA210_DMIC_CTRL_TRIMMER_SEL_SHIFT		12
#define TEGRA210_DMIC_CTRL_TRIMMER_SEL_MASK		(0x1f << TEGRA210_DMIC_CTRL_TRIMMER_SEL_SHIFT)

#define TEGRA210_DMIC_CTRL_CHANNEL_SELECT_SHIFT		8
#define TEGRA210_DMIC_CTRL_CHANNEL_SELECT_MASK		(0x3 << TEGRA210_DMIC_CTRL_CHANNEL_SELECT_SHIFT)

#define TEGRA210_DMIC_CTRL_LRSEL_POLARITY_SHIFT		4
#define TEGRA210_DMIC_CTRL_LRSEL_POLARITY_MASK		(0x1 << TEGRA210_DMIC_CTRL_LRSEL_POLARITY_SHIFT)

#define TEGRA210_DMIC_CTRL_OSR_SHIFT			0
#define TEGRA210_DMIC_CTRL_OSR_MASK			(0x3 << TEGRA210_DMIC_CTRL_OSR_SHIFT)

/* Fields in TEGRA210_DMIC_DBG_CTRL */
#define TEGRA210_DMIC_DBG_CTRL_DCR_ENABLE		BIT(3)
#define TEGRA210_DMIC_DBG_CTRL_LP_ENABLE		BIT(2)
#define TEGRA210_DMIC_DBG_CTRL_SC_ENABLE		BIT(1)
#define TEGRA210_DMIC_DBG_CTRL_BYPASS			BIT(0)

/* DMIC APIs */
int tegra210_dmic_get(enum tegra210_ahub_cifs *cif);
int tegra210_dmic_put(enum tegra210_ahub_cifs cif);
int tegra210_dmic_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_dmic_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_dmic_enable(enum tegra210_ahub_cifs cif, bool en);
int tegra210_dmic_reset(enum tegra210_ahub_cifs cif);
int tegra210_dmic_set_slcg(enum tegra210_ahub_cifs cif, bool en);
int tegra210_dmic_set_osr(enum tegra210_ahub_cifs cif, int osr);
int tegra210_dmic_set_lr_polarity(enum tegra210_ahub_cifs cif, int pol);
int tegra210_dmic_set_cap_channels(enum tegra210_ahub_cifs cif, int ch);
int tegra210_dmic_set_trimmer_sel(enum tegra210_ahub_cifs cif, int trim);
int tegra210_dmic_enable_sc(enum tegra210_ahub_cifs cif, bool en);
int tegra210_dmic_enable_dcr(enum tegra210_ahub_cifs cif, bool en);
int tegra210_dmic_enable_lp(enum tegra210_ahub_cifs cif, bool en);
int tegra210_dmic_enable_bypass(enum tegra210_ahub_cifs cif, bool en);

#endif
