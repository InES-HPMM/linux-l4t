/*
 * tegra210_admaif.c - Tegra ADMAIF driver.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Author: Dara Ramesh <dramesh@nvidia.com>
 * Based on code by Stephen Warren <swarren@nvidia.com>
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

#ifndef __MACH_TEGRA_ADMAIF_H
#define __MACH_TEGRA_ADMAIF_H

#include "tegra210_ahub_utils.h"

#define TEGRA210_ADMAIF_CHAN_COUNT						10
#define TEGRA210_ADMAIF_CHANNEL_REG_STRIDE				0x40

#define TEGRA210_ADMAIF_XBAR_TX_ENABLE					0x300
#define TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET				0x304
#define TEGRA210_ADMAIF_XBAR_TX_STATUS					0x30c
#define TEGRA210_ADMAIF_XBAR_TX_INT_STATUS				0x310
#define TEGRA210_ADMAIF_XBAR_TX_INT_MASK				0x314
#define TEGRA210_ADMAIF_XBAR_TX_INT_SET					0x318
#define TEGRA210_ADMAIF_XBAR_TX_INT_CLEAR				0x31c
#define TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL				0x320
#define TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL				0x328
#define TEGRA210_ADMAIF_XBAR_TX_FIFO_WRITE				0x32c

#define TEGRA210_ADMAIF_XBAR_RX_ENABLE					0x0
#define TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET				0x4
#define TEGRA210_ADMAIF_XBAR_RX_STATUS					0xc
#define TEGRA210_ADMAIF_XBAR_RX_INT_STATUS				0x10
#define TEGRA210_ADMAIF_XBAR_RX_INT_MASK				0x14
#define TEGRA210_ADMAIF_XBAR_RX_INT_SET					0x18
#define TEGRA210_ADMAIF_XBAR_RX_INT_CLEAR				0x1c
#define TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL				0x20
#define TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL				0x28
#define TEGRA210_ADMAIF_XBAR_RX_FIFO_READ				0x2c

#define TEGRA210_ADMAIF_GLOBAL_ENABLE					0x700
#define TEGRA210_ADMAIF_GLOBAL_CG_0						0x708

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT		0
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_MASK	(0x1f << TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT			8
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_MASK			(0x1f << TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT		20
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_MASK	(0x2ff << TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT)

#define TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_SHIFT		0
#define TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_MASK		(0x1 << TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_SHIFT)

#define TEGRA210_ADMAIF_CHAN_CTRL_TX_ENABLE				BIT(31)
#define TEGRA210_ADMAIF_CHAN_CTRL_RX_ENABLE				BIT(30)

#define TEGRA210_ADMAIF_LAST_REG						0x75f

/* TODO: PACK and UNPACK ENABLE */
int tegra210_admaif_enable(enum tegra210_ahub_cifs cif, int en);
int tegra210_admaif_get(enum tegra210_ahub_cifs *cif);
int tegra210_admaif_put(enum tegra210_ahub_cifs cif);
int tegra210_admaif_set_acif_param(enum tegra210_ahub_cifs cif,
				   struct tegra210_axbar_cif_param *acif);
int tegra210_admaif_get_acif_param(enum tegra210_ahub_cifs cif,
				   struct tegra210_axbar_cif_param *acif);
int tegra210_admaif_set_rx_dma_fifo(enum tegra210_ahub_cifs cif,
				 int start_addr, int size);
int tegra210_admaif_set_tx_dma_fifo(enum tegra210_ahub_cifs cif,
				 int start_addr, int size, int threshold);

int tegra210_admaif_soft_reset(enum tegra210_ahub_cifs cif);
int tegra210_admaif_xbar_transfer_status(enum tegra210_ahub_cifs cif);

int tegra210_admaif_global_enable(int en);

void tegra210_admaif_enable_clk(void);
void tegra210_admaif_disable_clk(void);
#endif
