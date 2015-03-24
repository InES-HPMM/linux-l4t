/*
 * tegra124_virt_alt_xbar.h
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

#ifndef __TEGRA124_VIRT_ALT_XBAR_H__
#define __TEGRA124_VIRT_ALT_XBAR_H__

#include "tegra124_virt_alt_ivc.h"

#define TEGRA_AHUB_AUDIO_RX_STRIDE 0x4

struct tegra124_virt_xbar {
	struct nvaudio_ivc_ctxt *hivc_client;
};

#endif
