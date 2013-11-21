/*
 * tegra210_spkprot.h - Tegra210 SPKPROT driver header.
 *
 * Author: Vinod Subbarayalu <vsubbarayalu@nvidia.com>
 *
 * Copyright (C) 2013, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __TEGRA210_SPKPROT_H
#define __TEGRA210_SPKPROT_H

#include "tegra210_ahub_utils.h"

#define TEGRA210_SPKPROT_COUNT	1
#define TEGRA210_SPKPROT_STRIDE 0x4
#define TEGRA210_SPKPROT_BIQUAD_COEFS 5

/* Register offsets from SPKPROT BASE */
#define TEGRA210_SPKPROT_XBAR_RX_STATUS					0x00C
#define TEGRA210_SPKPROT_XBAR_RX_INT_STATUS				0x010
#define TEGRA210_SPKPROT_XBAR_RX_INT_MASK				0x014
#define TEGRA210_SPKPROT_XBAR_RX_INT_SET				0x018
#define TEGRA210_SPKPROT_XBAR_RX_INT_CLEAR				0x01C
#define TEGRA210_SPKPROT_XBAR_RX_CIF_CTRL				0x020
#define TEGRA210_SPKPROT_XBAR_TX_STATUS					0x04C
#define TEGRA210_SPKPROT_XBAR_TX_INT_STATUS				0x050
#define TEGRA210_SPKPROT_XBAR_TX_INT_MASK				0x054
#define TEGRA210_SPKPROT_XBAR_TX_INT_SET				0x058
#define TEGRA210_SPKPROT_XBAR_TX_INT_CLEAR				0x05C
#define TEGRA210_SPKPROT_XBAR_TX_CIF_CTRL				0x060
#define TEGRA210_SPKPROT_ENABLE						0x080
#define TEGRA210_SPKPROT_SOFT_RESET					0x084
#define TEGRA210_SPKPROT_CG						0x088
#define TEGRA210_SPKPROT_STATUS						0x08C
#define TEGRA210_SPKPROT_INT_STATUS					0x090
#define TEGRA210_SPKPROT_CONFIG						0x0A4
#define TEGRA210_SPKPROT_LGAINATTACK					0x0A8
#define TEGRA210_SPKPROT_HGAINATTACK					0x0C8
#define TEGRA210_SPKPROT_LGAINRELEASE					0x0E8
#define TEGRA210_SPKPROT_HGAINRELEASE					0x108
#define TEGRA210_SPKPROT_LGAIN_FASTRELEASE				0x128
#define TEGRA210_SPKPROT_HGAIN_FASTRELEASE				0x148
#define TEGRA210_SPKPROT_FASTFACTOR					0x168
#define TEGRA210_SPKPROT_THRESH_DB					0x188
#define TEGRA210_SPKPROT_RATIO						0x1A8
#define TEGRA210_SPKPROT_ARSWITCH					0x1C8
#define TEGRA210_SPKPROT_GAINSWITCH					0x1CC
#define TEGRA210_SPKPROT_THRESHSWITCH					0x1D0
#define TEGRA210_SPKPROT_CONFIG_ERR_TYPE				0x1D4
#define TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_CTRL			0x1D8
#define TEGRA210_SPKPROT_AHUBRAMCTL_ARCOEFF_DATA			0x1DC
#define TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_CTRL			0x1E0
#define TEGRA210_SPKPROT_AHUBRAMCTL_BANDCOEFF_DATA			0x1E4
#define TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_CTRL			0x1E8
#define TEGRA210_SPKPROT_AHUBRAMCTL_INITCURR_GAIN_DATA			0x1EC
#define TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_CTRL			0x1F0
#define TEGRA210_SPKPROT_AHUBRAMCTL_NEW_GAIN_DATA			0x1F4
#define TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_CTRL			0x1F8
#define TEGRA210_SPKPROT_AHUBRAMCTL_SPCOEFF_DATA			0x1FC

/* Fields in TEGRA210_SPKPROT_ENABLE */
#define TEGRA210_SPKPROT_ENABLE_EN				BIT(0)

/* Fields in TEGRA210_SPKPROT_SOFT_RESET */
#define TEGRA210_SPKPROT_SOFT_RESET_EN			BIT(0)

/* Fields in TEGRA210_SPKPROT_CG */
#define TEGRA210_SPKPROT_CG_SLCG_EN				BIT(0)

#define TEGRA210_SPKPROT_ENABLE_STATUS				BIT(0)
#define TEGRA210_SPKPROT_STATUS_CONFIG_ERR			BIT(31)

/*Fields in SWITCH for ARfilter,Gain and threshold*/

#define TEGRA210_SPKPROT_SWITCH_ENABLE				BIT(0)
#define TEGRA210_SPKPROT_SWITCH_ACK				BIT(0)

/* TEGRA210_SPKPROT_CONFIG*/

enum tegra210_spkprot_config_mode {
	SPKPROT_MODE_BYPASS  = 0,
	SPKPROT_MODE_FULLBAND = 1,
	SPKPROT_MODE_DUALBAND = 2,
};

enum tegra210_spkprot_bandfilter_type {
	SPKPROT_BANDFILTER_CUSTOM = 0,
	SPKPROT_BANDFILTER_FLEX = 1,
};

struct tegra210_spkprot_config_param {
	u32 mode;
	u32 bias;
	u32 channels;
	u32 bandfilter_type;
	u32 bandfilter_biquad_stages;
	u32 arfilter_biquad_stages;
	u32 spfilter_biquad_stages;
};

struct tegra210_spkprot_algo_params {
	u32 mode;
	u32 channel_id;
	s32 gainattack_l;
	s32 gainattack_h;
	s32 gainrelease_l;
	s32 gainrelease_h;
	s32 fastrelease_l;
	s32 fastrelease_h;
	s32 fastfactor;
	s32 ratio;
};

#define TEGRA210_SPKPROT_CONFIG_MODE_SHIFT 0
#define TEGRA210_SPKPROT_CONFIG_MODE_MASK \
	(0x3 << TEGRA210_SPKPROT_CONFIG_MODE_SHIFT)

#define TEGRA210_SPKPROT_CONFIG_ROUND_SHIFT 2
#define TEGRA210_SPKPROT_CONFIG_ROUND_MASK \
	(0x1 << TEGRA210_SPKPROT_CONFIG_ROUND_SHIFT)

#define TEGRA210_SPKPROT_CONFIG_BAND_FILTER_SHIFT 3
#define TEGRA210_SPKPROT_CONFIG_BAND_FILTER_MASK \
	(0x1 << TEGRA210_SPKPROT_CONFIG_BAND_FILTER_SHIFT)

#define TEGRA210_SPKPROT_CONFIG_BAND_BIQUAD_STAGES_SHIFT 8
#define TEGRA210_SPKPROT_CONFIG_BAND_BIQUAD_STAGES_MASK \
	(0x3 << TEGRA210_SPKPROT_CONFIG_BAND_BIQUAD_STAGES_SHIFT)

#define TEGRA210_SPKPROT_CONFIG_AR_BIQUAD_STAGES_SHIFT	12
#define TEGRA210_SPKPROT_CONFIG_AR_BIQUAD_STAGES_MASK \
	(0x03 << TEGRA210_SPKPROT_CONFIG_AR_BIQUAD_STAGES_SHIFT)

#define TEGRA210_SPKPROT_CONFIG_SP_BIQUAD_STAGES_SHIFT	16
#define TEGRA210_SPKPROT_CONFIG_SP_BIQUAD_STAGES_MASK \
	(0x07 << TEGRA210_SPKPROT_CONFIG_SP_BIQUAD_STAGES_SHIFT)

/*Antiresonance Filter */
#define TEGRA210_SPKPROT_ARFILTER_MAX_BIQ_STAGES 3

struct spkprot_biq_params {
	s32 biq_b0;
	s32 biq_b1;
	s32 biq_b2;
	s32 biq_a1;
	s32 biq_a2;
};

struct spkprot_arcoeff_params {
	s32 pre_gain;
	struct spkprot_biq_params
		biq[TEGRA210_SPKPROT_ARFILTER_MAX_BIQ_STAGES];
	s32 post_gain;
};

/*Speakerprotection filter*/
#define TEGRA210_SPKPROT_SPFILTER_MAX_BIQ_STAGES 7

struct spkprot_spcoeff_params {
	s32 pre_gain;
	struct spkprot_biq_params
		spcoeff[TEGRA210_SPKPROT_SPFILTER_MAX_BIQ_STAGES];
	s32 post_gain;
};

/* Bandfilter */
#define TEGRA210_SPKPROT_BANDFILTER_MAX_BIQ_STAGES 3

struct spkprot_bandcoeff_params {
	struct spkprot_biq_params
		bandcoeff[SPKPROT_MODE_DUALBAND *
			TEGRA210_SPKPROT_BANDFILTER_MAX_BIQ_STAGES];
};

#define ARFILTER_RAM_DEPTH ((TEGRA210_SPKPROT_BIQUAD_COEFS \
				* TEGRA210_SPKPROT_ARFILTER_MAX_BIQ_STAGES) + 2)
#define SPFILTER_RAM_DEPTH ((TEGRA210_SPKPROT_BIQUAD_COEFS \
				* TEGRA210_SPKPROT_SPFILTER_MAX_BIQ_STAGES) + 2)
#define BANDFILTER_RAM_DEPTH (TEGRA210_SPKPROT_BIQUAD_COEFS \
	* TEGRA210_SPKPROT_BANDFILTER_MAX_BIQ_STAGES * SPKPROT_MODE_DUALBAND)

#define GAIN_RAM_DEPTH 1

/* SPKPROT APIs */
int tegra210_spkprot_get(enum tegra210_ahub_cifs *cif);

int tegra210_spkprot_put(enum tegra210_ahub_cifs cif);

int tegra210_spkprot_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);

int tegra210_spkprot_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);

int tegra210_spkprot_enable(enum tegra210_ahub_cifs cif, bool en);

int tegra210_spkprot_reset(enum tegra210_ahub_cifs cif);

int tegra210_spkprot_set_slcg(enum tegra210_ahub_cifs cif, bool en);

int tegra210_spkprot_set_config(enum tegra210_ahub_cifs cif,
				struct tegra210_spkprot_config_param *param);

int tegra210_spkprot_set_arcoef(enum tegra210_ahub_cifs cif,
				struct spkprot_arcoeff_params *params,
				int channel_id);

int tegra210_spkprot_set_spcoef(enum tegra210_ahub_cifs cif,
				struct spkprot_spcoeff_params *params,
				int channel_id);

int tegra210_spkprot_set_bandcoef(enum tegra210_ahub_cifs cif,
				struct spkprot_bandcoeff_params *params);

int tegra210_spkprot_set_threshold(enum tegra210_ahub_cifs cif,
					s32 threshold, int channel_id);

int tegra210_spkprot_set_algo_params(enum tegra210_ahub_cifs cif,
			struct tegra210_spkprot_algo_params params);

int tegra210_spkprot_set_newgain(enum tegra210_ahub_cifs cif,
					u32 *gain_l,
					u32 *gain_h,
					int channel_id,
					int max_supported_channels);

int tegra210_spkprot_set_initgain(enum tegra210_ahub_cifs cif,
					u32 *gain_l,
					u32 *gain_h,
					int channel_id,
					int max_supported_channels);

int tegra210_spkprot_set_arswitch(enum tegra210_ahub_cifs cif);

int tegra210_spkprot_set_gainswitch(enum tegra210_ahub_cifs cif);

int tegra210_spkprot_set_threshswitch(enum tegra210_ahub_cifs cif);
#endif
