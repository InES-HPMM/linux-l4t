/*
 * tegra210_ope.h - Tegra OPE driver header.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
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

#ifndef __TEGRA210_OPE_H__
#define __TEGRA210_OPE_H__

#include "tegra210_ahub_utils.h"

#define TEGRA210_OPE_COUNT		2

/* Register offsets from TEGRA210_OPE_BASE */
#define TEGRA210_OPE_XBAR_RX_STATUS		0xc
#define TEGRA210_OPE_XBAR_RX_INT_STATUS		0x10
#define TEGRA210_OPE_XBAR_RX_INT_MASK		0x14
#define TEGRA210_OPE_XBAR_RX_INT_SET		0x18
#define TEGRA210_OPE_XBAR_RX_INT_CLEAR		0x1c
#define TEGRA210_OPE_XBAR_RX_CIF_CTRL		0x20
#define TEGRA210_OPE_XBAR_TX_STATUS		0x4c
#define TEGRA210_OPE_XBAR_TX_INT_STATUS		0x50
#define TEGRA210_OPE_XBAR_TX_INT_MASK		0x54
#define TEGRA210_OPE_XBAR_TX_INT_SET		0x58
#define TEGRA210_OPE_XBAR_TX_INT_CLEAR		0x5c
#define TEGRA210_OPE_XBAR_TX_CIF_CTRL		0x60
#define TEGRA210_OPE_ENABLE			0x80
#define TEGRA210_OPE_SOFT_RST			0x84
#define TEGRA210_OPE_CG				0x88
#define TEGRA210_OPE_STATUS			0x8c
#define TEGRA210_OPE_INT_STATUS			0x90
#define TEGRA210_OPE_DIRECTION			0x94

#define TEGEA210_OPE_MAX_REGISTER		TEGRA210_OPE_DIRECTION

/* Fields for TEGRA210_OPE_XBAR_RX_STATUS */
#define TEGRA210_OPE_XBAR_RX_STATUS_ACIF_FIFO_FULL	BIT(1)
#define TEGRA210_OPE_XBAR_RX_STATUS_ACIF_FIFO_EMPTY	BIT(0)

/* Fields for TEGRA210_OPE_XBAR_RX_INT_STATUS */
#define TEGRA210_OPE_XBAR_RX_INT_STATUS_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_RX_INT_MASK */
#define TEGRA210_OPE_XBAR_RX_INT_MASK_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_RX_INT_SET */
#define TEGRA210_OPE_XBAR_RX_INT_SET_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_RX_INT_CLEAR */
#define TEGRA210_OPE_XBAR_RX_INT_CLEAR_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_RX_CIF_CTRL defined in tegra210_ahub_utils.h */

/* Fields for TEGRA210_OPE_XBAR_TX_STATUS */
#define TEGRA210_OPE_XBAR_TX_STATUS_ACIF_FIFO_FULL	BIT(1)
#define TEGRA210_OPE_XBAR_TX_STATUS_ACIF_FIFO_EMPTY	BIT(0)

/* Fields for TEGRA210_OPE_XBAR_TX_INT_STATUS */
#define TEGRA210_OPE_XBAR_TX_INT_STATUS_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_TX_INT_MASK */
#define TEGRA210_OPE_XBAR_TX_INT_MASK_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_TX_INT_SET */
#define TEGRA210_OPE_XBAR_TX_INT_SET_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_TX_INT_CLEAR */
#define TEGRA210_OPE_XBAR_TX_INT_CLEAR_RX_DONE		BIT(0)

/* Fields for TEGRA210_OPE_XBAR_TX_CIF_CTRL defined in tegra210_ahub_utils.h */

/* Fields for TEGRA210_OPE_ENABLE */
#define TEGRA210_OPE_ENABLE_ENABLE			BIT(0)

/* Fields for TEGRA210_OPE_SOFT_RST */
#define TEGRA210_OPE_SOFT_RST_RESET			BIT(0)

/* Fields for TEGRA210_OPE_CG */
#define TEGRA210_OPE_CG_SLCG_EN				BIT(0)

/* Fields for TEGRA210_OPE_STATUS */
#define TEGRA210_OPE_STATUS_CONFIG_ERR			BIT(31)
#define TEGRA210_OPE_STATUS_SLCG_CLKEN			BIT(8)
#define TEGRA210_OPE_STATUS_ENABLE_STATUS		BIT(0)

/* Fields for TEGRA210_OPE_INT_STATUS */
#define TEGRA210_OPE_INT_STATUS_TX_DONE			BIT(1)
#define TEGRA210_OPE_INT_STATUS_RX_DONE			BIT(0)

/* Fields for TEGRA210_OPE_DIRECTION */
#define TEGRA210_OPE_DIRECTION_DIR_SHIFT		0
#define TEGRA210_OPE_DIRECTION_DIR_MASK			1

#define TEGRA210_OPE_DIRECTION_DIR_MBDRC_TO_PEQ		(0 << TEGRA210_OPE_DIRECTION_DIR_SHIFT)
#define TEGRA210_OPE_DIRECTION_DIR_PEQ_TO_MBDRC		(1 << TEGRA210_OPE_DIRECTION_DIR_SHIFT)

int tegra210_ope_get(enum tegra210_ahub_cifs *cif);
int tegra210_ope_put(enum tegra210_ahub_cifs cif);
int tegra210_ope_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_ope_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_ope_is_cif_fifo_full(enum tegra210_ahub_cifs cif);
int tegra210_ope_is_cif_fifo_empty(enum tegra210_ahub_cifs cif);
int tegra210_ope_is_enable(enum tegra210_ahub_cifs cif);
int tegra210_ope_is_slcg_enable(enum tegra210_ahub_cifs cif);
int tegra210_ope_is_config_err(enum tegra210_ahub_cifs cif);
int tegra210_ope_enable(enum tegra210_ahub_cifs cif, bool en);
int tegra210_ope_reset(enum tegra210_ahub_cifs cif);
int tegra210_ope_set_slcg(enum tegra210_ahub_cifs cif, int en);
int tegra210_ope_dir_peq_to_mbdrc(enum tegra210_ahub_cifs cif, int val);

#endif
