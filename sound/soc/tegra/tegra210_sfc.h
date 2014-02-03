/*
 * tegra210_sfc.h - Tegra210 SFC driver header.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
 *
 * Copyright (C) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __TEGRA210_SFC_H
#define __TEGRA210_SFC_H

#include "tegra210_ahub_utils.h"

#define TEGRA210_SFC_COUNT	4

/* Register offsets from SFC BASE */
#define TEGRA210_SFC_RX_STATUS			0x0c
#define TEGRA210_SFC_RX_INT_STATUS		0x10
#define TEGRA210_SFC_RX_INT_MASK		0x14
#define TEGRA210_SFC_RX_INT_SET			0x18
#define TEGRA210_SFC_RX_INT_CLEAR		0x1c
#define TEGRA210_SFC_RX_CIF_CTRL		0x20
#define TEGRA210_SFC_RX_FREQ			0x24

#define TEGRA210_SFC_TX_STATUS			0x4c
#define TEGRA210_SFC_TX_INT_STATUS		0x50
#define TEGRA210_SFC_TX_INT_MASK		0x54
#define TEGRA210_SFC_TX_INT_SET			0x58
#define TEGRA210_SFC_TX_INT_CLEAR		0x5c
#define TEGRA210_SFC_TX_CIF_CTRL		0x60
#define TEGRA210_SFC_TX_FREQ			0x64

#define TEGRA210_SFC_ENABLE			0x80
#define TEGRA210_SFC_SOFT_RESET			0x84
#define TEGRA210_SFC_CG				0x88
#define TEGRA210_SFC_STATUS			0x8c
#define TEGRA210_SFC_INT_STATUS			0x90
#define TEGRA210_SFC_COEF_RAM			0xbc
#define TEGRA210_SFC_ARAMCTL_COEF_CTRL		0xc0
#define TEGRA210_SFC_ARAMCTL_COEF_DATA		0xc4

/* Constants for SFC*/
#define TEGRA210_SFC_FS_8000HZ			0
#define TEGRA210_SFC_FS_11025HZ			1
#define TEGRA210_SFC_FS_16000HZ			2
#define TEGRA210_SFC_FS_22050HZ			3
#define TEGRA210_SFC_FS_24000HZ			4
#define TEGRA210_SFC_FS_32000HZ			5
#define TEGRA210_SFC_FS_44100HZ			6
#define TEGRA210_SFC_FS_48000HZ			7
#define TEGRA210_SFC_FS_64000HZ			8
#define TEGRA210_SFC_FS_88200HZ			9
#define TEGRA210_SFC_FS_96000HZ			10
#define TEGRA210_SFC_FS_176400HZ		11
#define TEGRA210_SFC_FS_192000HZ		12

/* Fields in TEGRA210_SFC_RX_FREQ */
#define TEGRA210_SFC_RX_FREQ_FS_IN_SHIFT	0
#define TEGRA210_SFC_RX_FREQ_FS_IN_MASK		(0xf << TEGRA210_SFC_RX_FREQ_FS_IN_SHIFT)

/* Fields in TEGRA210_SFC_TX_FREQ */
#define TEGRA210_SFC_TX_FREQ_FS_OUT_SHIFT	0
#define TEGRA210_SFC_TX_FREQ_FS_OUT_MASK	(0xf << TEGRA210_SFC_TX_FREQ_FS_OUT_SHIFT)

/* Fields in TEGRA210_SFC_ENABLE */
#define TEGRA210_SFC_ENABLE_EN			BIT(0)

/* Fields in TEGRA210_SFC_SOFT_RESET */
#define TEGRA210_SFC_SOFT_RESET_EN		BIT(0)

/* Fields in TEGRA210_SFC_CG */
#define TEGRA210_SFC_CG_SLCG_EN			BIT(0)

/* Fields in TEGRA210_SFC_STATUS */
#define TEGRA210_SFC_STATUS_CONFIG_ERROR_SHIFT	31
#define TEGRA210_SFC_STATUS_CONFIG_ERROR_MASK	(0x1 << TEGRA210_SFC_STATUS_CONFIG_ERROR_SHIFT)

#define TEGRA210_SFC_STATUS_SLCG_CLKEN_SHIFT	8
#define TEGRA210_SFC_STATUS_SLCG_CLKEN_MASK	(0x1 << TEGRA210_SFC_STATUS_SLCG_CLKEN_SHIFT)

#define TEGRA210_SFC_STATUS_ENABLE_STATUS_SHIFT	0
#define TEGRA210_SFC_STATUS_ENABLE_STATUS_MASK	(0x1 << TEGRA210_SFC_STATUS_ENABLE_STATUS_SHIFT)

/* Fields in TEGRA210_SFC_COEF_RAM */
#define TEGRA210_SFC_COEF_RAM_COEF_RAM_EN	BIT(0)

/* SRC coefficients */
#define TEGRA210_SFC_COEF_RAM_DEPTH		64

/* SFC APIs */
int tegra210_sfc_get(enum tegra210_ahub_cifs *cif);
int tegra210_sfc_put(enum tegra210_ahub_cifs cif);
int tegra210_sfc_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_sfc_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_sfc_set_rx_rate(enum tegra210_ahub_cifs cif, int fs_in);
int tegra210_sfc_set_tx_rate(enum tegra210_ahub_cifs cif, int fs_out);
int tegra210_sfc_enable(enum tegra210_ahub_cifs cif, bool en);
int tegra210_sfc_reset(enum tegra210_ahub_cifs cif);
int tegra210_sfc_set_slcg(enum tegra210_ahub_cifs cif, bool en);
int tegra210_sfc_enable_coef_ram(enum tegra210_ahub_cifs cif,
				u32 *coef, bool en);

#endif
