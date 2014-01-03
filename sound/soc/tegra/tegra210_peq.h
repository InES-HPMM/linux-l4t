/*
 * tegra210_PEQ.h - Tegra PEQ driver header.
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

#ifndef __TEGRA210_PEQ_H__
#define __TEGRA210_PEQ_H__

#include "tegra210_ahub_utils.h"

#define TEGRA210_PEQ_COUNT			2
#define TEGRA210_PEQ_MAX_BIQ_STAGES		12
#define TEGRA210_PEQ_MAX_CHAN			8
#define TEGRA210_PEQ_SHIFT_WIDTH		5

/* Register offsets from TEGRA210_PEQ_BASE */
#define TEGRA210_PEQ_SOFT_RST			0X0
#define TEGRA210_PEQ_CG				0X4
#define TEGRA210_PEQ_STATUS			0X8
#define TEGRA210_PEQ_CONFIG			0Xc
#define TEGRA210_PEQ_AHUBRAMCTL_PEQ_CTRL	0X10
#define TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA	0X14
#define TEGRA210_PEQ_AHUBRAMCTL_SHIFT_CTRL	0X18
#define TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA	0X1c

#define TEGRA210_PEQ_MAX_REGISTER		TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA

/* Fields for TEGRA210_PEQ_SOFT_RST */
#define TEGRA210_PEQ_SOFT_RST_RESET			BIT(0)

/* Fields for TEGRA210_PEQ_CG */
#define TEGRA210_PEQ_CG_SLCG_EN				BIT(0)

/* Fields for TEGRA210_PEQ_STATUS */
#define TEGRA210_PEQ_STATUS_CONFIG_ERR			BIT(31)
#define TEGRA210_PEQ_STATUS_SLCG_CLKEN			BIT(0)

/* Fields for TEGRA210_PEQ_CONFIG */
#define TEGRA210_PEQ_CONFIG_BIQUAD_STAGES_SHIFT		2
#define TEGRA210_PEQ_CONfIG_BIQUAD_STAGES_MASK		(0xf << TEGRA210_PEQ_CONFIG_BIQUAD_STAGES_SHIFT)

#define TEGRA210_PEQ_CONFIG_BIAS_UNBIAS			BIT(1)

#define PEQ_MODE_BYPASS					0
#define PEQ_MODE_ACTIVE					1

#define TEGRA210_PEQ_CONFIG_MODE_SHIFT			0
#define TEGRA210_PEQ_CONFIG_MODE_MASK			(0x1 << TEGRA210_PEQ_CONFIG_MODE_SHIFT)
#define TEGRA210_PEQ_CONFIG_MODE_BYPASS			(PEQ_MODE_BYPASS << TEGRA210_PEQ_CONFIG_MODE_SHIFT)
#define TEGRA210_PEQ_CONFIG_MODE_ACTIVE			(PEQ_MODE_ACTIVE << TEGRA210_PEQ_CONFIG_MODE_SHIFT)


/* Fields for TEGRA210_PEQ_AHUBRAMCTL_PEQ_CTRL defined in tegra210_ahub_utils.h */
/* Fields for TEGRA210_PEQ_AHUBRAMCTL_PEQ_DATA defined in tegra210_ahub_utils.h */
/* Fields for TEGRA210_PEQ_AHUBRAMCTL_SHIFT_CTRL defined in tegra210_ahub_utils.h */
/* Fields for TEGRA210_PEQ_AHUBRAMCTL_SHIFT_DATA defined in tegra210_ahub_utils.h */

#define TEGRA210_PEQ_COEFF_DATA_SIZE_PER_CH		((TEGRA210_PEQ_MAX_BIQ_STAGES * 5) + 2)
#define TEGRA210_PEQ_SHIFT_DATA_SIZE_PER_CH		(TEGRA210_PEQ_MAX_BIQ_STAGES + 2)

/* order and size of each structure element for following structures should not
   be altered size order of elements and their size are based on PEQ
   co-eff ram and shift ram layout.
*/
struct peq_biq_params {
	s32 biq_b0;
	s32 biq_b1;
	s32 biq_b2;
	s32 biq_a1;
	s32 biq_a2;
};

struct peq_coeff_params {
	s32 pre_gain;
	struct peq_biq_params biq[TEGRA210_PEQ_MAX_BIQ_STAGES];
	s32 post_gain;
};

struct peq_shift_params {
	s32 pre_shift;
	s32 biq_shift[TEGRA210_PEQ_MAX_BIQ_STAGES];
	s32 post_shift;
};

void tegra210_peq_enable_clk(enum tegra210_ahub_cifs cif);
void tegra210_peq_disable_clk(enum tegra210_ahub_cifs cif);
int tegra210_peq_is_slcg_enable(enum tegra210_ahub_cifs cif);
int tegra210_peq_is_config_err(enum tegra210_ahub_cifs cif);
int tegra210_peq_reset(enum tegra210_ahub_cifs cif);
int tegra210_peq_set_slcg(enum tegra210_ahub_cifs cif, int en);
int tegra210_peq_set_active(enum tegra210_ahub_cifs cif, int active);
int tegra210_peq_set_biquad_stages(enum tegra210_ahub_cifs cif, int biq_stages);
int tegra210_peq_write_coeff_params(enum tegra210_ahub_cifs cif,
				    struct peq_coeff_params *params,
				    int channel_id);
int tegra210_peq_write_shift_params(enum tegra210_ahub_cifs cif,
				    struct peq_shift_params *params,
				    int channel_id);
#endif
