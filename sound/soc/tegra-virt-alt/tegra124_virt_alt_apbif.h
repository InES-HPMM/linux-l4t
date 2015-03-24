/*
 * tegra124_virt_alt_apbif.h
 *
 * Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA124_VIRT_ALT_APBIF_H__
#define __TEGRA124_VIRT_ALT_APBIF_H__

#include "tegra124_virt_alt_ivc_common.h"

#define TEGRA_APBIF_BASE		0x70300000
#define TEGRA_APBIF2_BASE2		0x70300200

#define TEGRA_AHUB_AUDIO_RX_STRIDE 0x4

/* TEGRA_AHUB_CHANNEL_TXFIFO */

#define TEGRA_APBIF_CHANNEL_TXFIFO			0xc
#define TEGRA_APBIF_CHANNEL_TXFIFO_STRIDE		0x20
#define TEGRA_APBIF_CHANNEL_TXFIFO_COUNT		4

/* TEGRA_AHUB_CHANNEL_RXFIFO */

#define TEGRA_APBIF_CHANNEL_RXFIFO			0x10
#define TEGRA_APBIF_CHANNEL_RXFIFO_STRIDE		0x20
#define TEGRA_APBIF_CHANNEL_RXFIFO_COUNT		4

#define TEGRA_AHUB_CHANNEL_CTRL			0x0
#define TEGRA_AHUB_CHANNEL_CTRL_TX_EN			(1 << 31)
#define TEGRA_AHUB_CHANNEL_CTRL_RX_EN			(1 << 30)

/* Audio bit width */
enum {
	AUDIO_BITS_4 = 0,
	AUDIO_BITS_8,
	AUDIO_BITS_12,
	AUDIO_BITS_16,
	AUDIO_BITS_20,
	AUDIO_BITS_24,
	AUDIO_BITS_28,
	AUDIO_BITS_32,
};

/* APBIF ids */
enum {
	APBIF_ID_0 = 0,
	APBIF_ID_1,
	APBIF_ID_2,
	APBIF_ID_3,
	APBIF_ID_4,
	APBIF_ID_5,
	APBIF_ID_6,
	APBIF_ID_7,
	APBIF_ID_8,
	APBIF_ID_9,
	MAX_APBIF_IDS
};

/* Audio cif definition */
struct tegra124_virt_audio_cif {
	unsigned int threshold;
	unsigned int audio_channels;
	unsigned int client_channels;
	unsigned int audio_bits;
	unsigned int client_bits;
	unsigned int expand;
	unsigned int stereo_conv;
	unsigned int replicate;
	unsigned int direction;
	unsigned int truncate;
	unsigned int mono_conv;
};

/*  apbif data */
struct tegra124_virt_apbif_client_data {
	unsigned int apbif_id;
	struct tegra124_virt_audio_cif cif;
	struct nvaudio_ivc_ctxt *hivc_client;
};

/*  */
struct tegra124_virt_apbif_client {
	struct tegra_alt_pcm_dma_params *capture_dma_data;
	struct tegra_alt_pcm_dma_params *playback_dma_data;
	struct tegra124_virt_apbif_client_data client_data;
};

#define TEGRA_AHUB_AUDIO_RX1					0x200
#define TEGRA_AHUB_AUDIO_UPDATE_MAX_REG		2
struct tegra124_virt_alt_xbar_soc_data {
	unsigned int num_dais;
	unsigned int num_mux_widgets;
	unsigned int num_mux0_input;
	unsigned int num_mux1_input;
	unsigned int mask[2];
	unsigned int reg_count;
	unsigned int reg_offset;
};

int tegra124_virt_apbif_register_component(struct platform_device *pdev);
#endif
