/*
 * tegra210_amixer.h - Tegra210 AMIXER driver header.
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

#ifndef __TEGRA210_AMIXER_H
#define __TEGRA210_AMIXER_H

#include "tegra210_ahub_utils.h"

#define TEGRA210_AMIXER_MAX_CHANNEL_COUNT		8
#define TEGRA210_AMIXER_RX_MAX				10
#define TEGRA210_AMIXER_RX_STRIDE			0x40
#define TEGRA210_AMIXER_TX_MAX				5
#define TEGRA210_AMIXER_TX_STRIDE			0x40

#define TEGRA210_AMIXER_RX_SOFT_RESET			0x04
#define TEGRA210_AMIXER_RX_STATUS			0x10
#define TEGRA210_AMIXER_RX_CIF_CTRL			0x24
#define TEGRA210_AMIXER_RX_CTRL				0x28
#define TEGRA210_AMIXER_RX_PEAK_CTRL			0x2c
#define TEGRA210_AMIXER_RX_SAMPLE_COUNT			0x30

#define TEGRA210_AMIXER_TX_ENABLE			0x280
#define TEGRA210_AMIXER_TX_SOFT_RESET			0x284
#define TEGRA210_AMIXER_TX_STATUS			0x290
#define TEGRA210_AMIXER_TX_INT_STATUS			0x294
#define TEGRA210_AMIXER_TX_INT_MASK			0x298
#define TEGRA210_AMIXER_TX_INT_SET			0x29c
#define TEGRA210_AMIXER_TX_INT_CLEAR			0x2a0
#define TEGRA210_AMIXER_TX_CIF_CTRL			0x2a4
#define TEGRA210_AMIXER_TX_ADDER_CONFIG			0x2a8

/* Mixer global registers */
#define TEGRA210_AMIXER_ENABLE				0x400
#define TEGRA210_AMIXER_SOFT_RESET			0x404
#define TEGRA210_AMIXER_CG				0x408
#define TEGRA210_AMIXER_STATUS				0x410
#define TEGRA210_AMIXER_INT_STATUS			0x414
#define TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_CTRL	0x42c
#define TEGRA210_AMIXER_AHUBRAMCTL_GAIN_CONFIG_DATA	0x430
#define TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_CTRL		0x434
#define TEGRA210_AMIXER_AHUBRAMCTL_PEAKM_DATA		0x438
#define TEGRA210_AMIXER_CTRL				0x43c

#define TEGRA210_AMIXER_RX_LIMIT	(TEGRA210_AMIXER_RX_MAX * TEGRA210_AMIXER_RX_STRIDE)
#define TEGRA210_AMIXER_TX_LIMIT	(TEGRA210_AMIXER_RX_LIMIT + (TEGRA210_AMIXER_TX_MAX * TEGRA210_AMIXER_TX_STRIDE))

/* Fields in TEGRA210_AMIXER_RX_SOFT_RESET */
#define TEGRA210_AMIXER_RX_SOFT_RESET_EN	BIT(0)

/* Fields in TEGRA210_AMIXER_RX_STATUS */
#define TEGRA210_AMIXER_RX_STATUS_CONFIG_ERROR_SHIFT	31
#define TEGRA210_AMIXER_RX_STATUS_CONFIG_ERROR_MASK	(0x1 << TEGRA210_AMIXER_RX_STATUS_CONFIG_ERROR_SHIFT)

#define TEGRA210_AMIXER_RX_STATUS_CONFIG_BUSY_SHIFT	8
#define TEGRA210_AMIXER_RX_STATUS_CONFIG_BUSY_MASK	(0x1 << TEGRA210_AMIXER_RX_STATUS_CONFIG_BUSY_SHIFT)

#define TEGRA210_AMIXER_RX_STATUS_ACIF_FIFO_FULL_SHIFT	1
#define TEGRA210_AMIXER_RX_STATUS_ACIF_FIFO_FULL_MASK	(0x1 << TEGRA210_AMIXER_RX_STATUS_ACIF_FIFO_FULL_SHIFT)

#define TEGRA210_AMIXER_RX_STATUS_ACIF_FIFO_EMPTY_SHIFT	0
#define TEGRA210_AMIXER_RX_STATUS_ACIF_FIFO_EMPTY_MASK	(0x1 << TEGRA210_AMIXER_RX_STATUS_ACIF_FIFO_EMPTY_SHIFT)

/* Fields in TEGRA210_AMIXER_RX_CTRL */
#define TEGRA210_AMIXER_RX_CTRL_GAIN_BYPASS_EN		BIT(31)

#define TEGRA210_AMIXER_RX_CTRL_SAMPLE_COUNTER_RESET_EN	BIT(24)

#define TEGRA210_AMIXER_RX_CTRL_ACT_THRESHOLD_SHIFT	16
#define TEGRA210_AMIXER_RX_CTRL_ACT_THRESHOLD_MASK	(0x1f << TEGRA210_AMIXER_RX_CTRL_ACT_THRESHOLD_SHIFT)

#define TEGRA210_AMIXER_RX_CTRL_DEACT_THRESHOLD_SHIFT	0
#define TEGRA210_AMIXER_RX_CTRL_DEACT_THRESHOLD_MASK	(0xffff << TEGRA210_AMIXER_RX_CTRL_DEACT_THRESHOLD_SHIFT)

/* Fields in TEGRA210_AMIXER_RX_PEAK_CTRL */
#define TEGRA210_AMIXER_RX_PEAK_CTRL_PEAK_TYPE_SHIFT	31
#define TEGRA210_AMIXER_RX_PEAK_CTRL_PEAK_TYPE_MASK	(0x1 << TEGRA210_AMIXER_RX_PEAK_CTRL_PEAK_TYPE_SHIFT)

#define TEGRA210_AMIXER_RX_PEAK_CTRL_WIN_SIZE_SHIFT	0
#define TEGRA210_AMIXER_RX_PEAK_CTRL_WIN_SIZE_MASK	(0x7fffffff << TEGRA210_AMIXER_RX_PEAK_CTRL_WIN_SIZE_SHIFT)

/* Fields in TEGRA210_AMIXER_RX_SAMPLE_COUNT */
#define TEGRA210_AMIXER_RX_SAMPLE_COUNT_SAMPLE_CNT_SHIFT	0
#define TEGRA210_AMIXER_RX_SAMPLE_COUNT_SAMPLE_CNT_MASK		(0xffffffff << TEGRA210_AMIXER_RX_SAMPLE_COUNT_SAMPLE_CNT_SHIFT)

/* Fields in TEGRA210_AMIXER_TX_ENABLE */
#define TEGRA210_AMIXER_TX_ENABLE_EN			BIT(0)

/* Fields in TEGRA210_AMIXER_TX_SOFT_RESET */
#define TEGRA210_AMIXER_TX_SOFT_RESET_EN		BIT(0)

/* Fields in TEGRA210_AMIXER_TX_STATUS */
#define TEGRA210_AMIXER_TX_STATUS_ACIF_FIFO_FULL_SHIFT	2
#define TEGRA210_AMIXER_TX_STATUS_ACIF_FIFO_FULL_MASK	(0x1 << TEGRA210_AMIXER_TX_STATUS_ACIF_FIFO_FULL_SHIFT)

#define TEGRA210_AMIXER_TX_STATUS_ACIF_FIFO_EMPTY_SHIFT	1
#define TEGRA210_AMIXER_TX_STATUS_ACIF_FIFO_EMPTY_MASK	(0x1 << TEGRA210_AMIXER_TX_STATUS_ACIF_FIFO_EMPTY_SHIFT)

#define TEGRA210_AMIXER_TX_STATUS_ENABLE_STATUS_SHIFT	0
#define TEGRA210_AMIXER_TX_STATUS_ENABLE_STATUS_MASK	(0x1 << TEGRA210_AMIXER_TX_STATUS_ENABLE_STATUS_SHIFT)

/* Fields in TEGRA210_AMIXER_TX_ADDER_CONFIG */
#define TEGRA210_AMIXER_TX_ADDER_CONFIG_INPUT_ENABLE(i)	BIT(i)

/* Fields in TEGRA210_AMIXER_ENABLE */
#define TEGRA210_AMIXER_ENABLE_EN			BIT(0)

/* Fields in TEGRA210_AMIXER_SOFT_RESET */
#define TEGRA210_AMIXER_SOFT_RESET_EN			BIT(0)

/* Fields in TEGRA210_AMIXER_CG */
#define TEGRA210_AMIXER_CG_SLCG_EN			BIT(0)

/* Fields in TEGRA210_AMIXER_STATUS */
#define TEGRA210_AMIXER_STATUS_SLCG_CLKEN_SHIFT		1
#define TEGRA210_AMIXER_STATUS_SLCG_CLKEN_MASK		(0x1 << TEGRA210_AMIXER_STATUS_SLCG_CLKEN_SHIFT)

#define TEGRA210_AMIXER_STATUS_ENABLE_STATUS_SHIFT	0
#define TEGRA210_AMIXER_STATUS_ENABLE_STATUS_MASK	(0x1 << TEGRA210_AMIXER_STATUS_ENABLE_STATUS_SHIFT)

/* Fields in TEGRA210_AMIXER_CTRL */
#define TEGRA210_AMIXER_CTRL_BYPASS_MODE_EN		BIT(0)

enum T210_AMIXER_PEAK_TYPE {
	PEAK_WINDOW_BASED = 0,
	PEAK_RESET_ON_READ = 1,
};

#define TEGRA210_AMIXER_MAX_PEAK_WIN_SIZE	0x7fffffff
#define TEGRA210_AMIXER_GAIN_COEF_DEPTH		16

/* AMIXER APIs */
int tegra210_amixer_get(void);
int tegra210_amixer_put(void);
int tegra210_amixer_enable(bool en);
int tegra210_amixer_reset(void);
int tegra210_amixer_set_slcg(bool en);
int tegra210_amixer_enable_bypass_mode(bool en);
int tegra210_amixer_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_amixer_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_amixer_rx_reset(enum tegra210_ahub_cifs cif);
int tegra210_amixer_rx_set_act_threshold(enum tegra210_ahub_cifs cif,
				int act_th);
int tegra210_amixer_rx_set_deact_threshold(enum tegra210_ahub_cifs cif,
				int deact_th);
int tegra210_amixer_rx_reset_sample_counter(enum tegra210_ahub_cifs cif);
int tegra210_amixer_rx_bypass_gain(enum tegra210_ahub_cifs cif, bool en);
int tegra210_amixer_rx_set_gain_coef(enum tegra210_ahub_cifs cif, u32 *coef);
int tegra210_amixer_rx_set_peak_ctrl(enum tegra210_ahub_cifs cif, int type,
				unsigned int win_size);
int tegra210_amixer_rx_get_peak_values(enum tegra210_ahub_cifs cif, u32 *peak);
u32 tegra210_amixer_rx_get_sample_count(enum tegra210_ahub_cifs cif);
int tegra210_amixer_tx_enable(enum tegra210_ahub_cifs cif, bool en);
int tegra210_amixer_tx_reset(enum tegra210_ahub_cifs cif);
int tegra210_amixer_tx_enable_input(enum tegra210_ahub_cifs txcif,
				enum tegra210_ahub_cifs rxcif, bool en);

#endif
