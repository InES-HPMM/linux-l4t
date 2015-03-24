/*
 * tegra124_virt_alt_dam.h - Definitions for Tegra124 DAM driver
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHIN
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA124_DAM_VIRT_ALT_H__
#define __TEGRA124_DAM_VIRT_ALT_H__

#include "tegra124_virt_alt_ivc.h"


#define TEGRA_DAM_CH0_CONV	0x14
#define TEGRA_DAM_CH1_CONV	0x24

struct tegra124_dam {
	int dev_id;
	struct nvaudio_ivc_ctxt *hivc_client;
};

#endif
