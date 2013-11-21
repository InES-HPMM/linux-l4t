/*
 * tegra210_sfc.h - Tegra210 SFC driver header.
 *
 * Author: Rahul Mittal <rmittal@nvidia.com>
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
#define TEGRA210_SFC_FS_8000HZ		0
#define TEGRA210_SFC_FS_11025HZ		1
#define TEGRA210_SFC_FS_16000HZ		2
#define TEGRA210_SFC_FS_22050HZ		3
#define TEGRA210_SFC_FS_24000HZ		4
#define TEGRA210_SFC_FS_32000HZ		5
#define TEGRA210_SFC_FS_44100HZ		6
#define TEGRA210_SFC_FS_48000HZ		7
#define TEGRA210_SFC_FS_64000HZ		8
#define TEGRA210_SFC_FS_88200HZ		9
#define TEGRA210_SFC_FS_96000HZ 	10
#define TEGRA210_SFC_FS_176400HZ	11
#define TEGRA210_SFC_FS_192000HZ	12

/* Fields in TEGRA210_SFC_RX_FREQ */
#define TEGRA210_SFC_RX_FREQ_FS_IN_SHIFT	0
#define TEGRA210_SFC_RX_FREQ_FS_IN_MASK		(0xf << TEGRA210_SFC_RX_FREQ_FS_IN_SHIFT)

/* Fields in TEGRA210_SFC_TX_FREQ */
#define TEGRA210_SFC_TX_FREQ_FS_OUT_SHIFT	0
#define TEGRA210_SFC_TX_FREQ_FS_OUT_MASK	(0xf << TEGRA210_SFC_TX_FREQ_FS_OUT_SHIFT)

/* Fields in TEGRA210_SFC_ENABLE */
#define TEGRA210_SFC_ENABLE_EN				BIT(0)

/* Fields in TEGRA210_SFC_SOFT_RESET */
#define TEGRA210_SFC_SOFT_RESET_EN			BIT(0)

/* Fields in TEGRA210_SFC_CG */
#define TEGRA210_SFC_CG_SLCG_EN				BIT(0)

/* Fields in TEGRA210_SFC_STATUS */
#define TEGRA210_SFC_STATUS_CONFIG_ERROR_SHIFT		31
#define TEGRA210_SFC_STATUS_CONFIG_ERROR_MASK		(0x1 << TEGRA210_SFC_STATUS_CONFIG_ERROR_SHIFT)

#define TEGRA210_SFC_STATUS_SLCG_CLKEN_SHIFT		8
#define TEGRA210_SFC_STATUS_SLCG_CLKEN_MASK			(0x1 << TEGRA210_SFC_STATUS_SLCG_CLKEN_SHIFT)

#define TEGRA210_SFC_STATUS_ENABLE_STATUS_SHIFT		0
#define TEGRA210_SFC_STATUS_ENABLE_STATUS_MASK		(0x1 << TEGRA210_SFC_STATUS_ENABLE_STATUS_SHIFT)

/* Fields in TEGRA210_SFC_COEF_RAM */
#define TEGRA210_SFC_COEF_RAM_COEF_RAM_EN	BIT(0)

/* SRC coefficients */
#define TEGRA210_SFC_COEF_RAM_DEPTH		64

u32 coef_8to16[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00006105, // interpolation + IIR Filter
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000002, // ouptut gain
};

u32 coef_8to44[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x0156105, // interpolation + IIR filter
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000002, // ouptut gain
	0x0021a102, // interpolation + IIR filter
	0x00000e00, // input gain
	0x00e2e000,0xff6e1a00,0x002aaa00,
	0x00610a00,0xff5dda00,0x003ccc00,
	0x00163a00,0xff3c0400,0x00633200,
	0x00000003,// Output gain
	0x00000204, // Farrow filter
	0x000aaaab,
	0xffaaaaab,
	0xfffaaaab,
	0x00555555,
	0xff600000,
	0xfff55555,
	0x00155555,
	0x00055555,
	0xffeaaaab,
	0x00200000,
};

u32 coef_8to48[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00156105, // interpolation + IIR Filter
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000002, // ouptut gain
	0x0000a102, // interpolation + IIR filter
	0x00000e00, // input gain
	0x00e2e000,0xff6e1a00,0x002aaa00,
	0x00610a00,0xff5dda00,0x003ccc00,
	0x00163a00,0xff3c0400,0x00633200,
	0x00000003, //output gain
};

u32 coef_16to44[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00156105, // interpolation + IIR filter
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000002, //output gain
	0x0021a102, // interpolation + IIR filter
	0x00000e00, // input gain
	0x00e2e000,0xff6e1a00,0x002aaa00,
	0x00610a00,0xff5dda00,0x003ccc00,
	0x00163a00,0xff3c0400,0x00633200,
	0x00000003, // output gain
	0x002c0204, // Farrow Filter
	0x000aaaab,
	0xffaaaaab,
	0xfffaaaab,
	0x00555555,
	0xff600000,
	0xfff55555,
	0x00155555,
	0x00055555,
	0xffeaaaab,
	0x00200000,
	0x00005101, // IIR Filter + Decimator
	0x0000203c, // input gain
	0x00f52d35,0xff2e2162,0x005a21e0,
	0x00c6f0f0,0xff2ecd69,0x006fa78d,
	0x00000001, // output gain
};

u32 coef_16to48[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x0000a105, // interpolation + IIR Filter
	0x00000784, // input gain
	0x00cc516e,0xff2c9639,0x005ad5b3,
	0x0013ad0d,0xff3d4799,0x0063ce75,
	0xffb6f398,0xff5138d1,0x006e9e1f,
	0xff9186e5,0xff5f96a4,0x0076a86e,
	0xff82089c,0xff676b81,0x007b9f8a,
	0xff7c48a5,0xff6a31e7,0x007ebb7b,
	0x00000003, // output gain
};


u32 coef_16to8[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00005105,   //IIR Filter + Decimator
	0x0000d649, //input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000001, // ouptut gain
};

u32 coef_44to8[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00120104, // IIR Filter
	0x00000af2, // input gain
	0x0057eebe,0xff1e9863,0x00652604,
	0xff7206ea,0xff22ad7e,0x006d47e1,
	0xff42a4d7,0xff26e722,0x0075fd83,
	0xff352f66,0xff29312b,0x007b986b,
	0xff310a07,0xff296f51,0x007eca7c,
	0x00000001, // output gain
	0x001d9204, // Farrow Filter + decimation
	0x000aaaab,
	0xffaaaaab,
	0xfffaaaab,
	0x00555555,
	0xff600000,
	0xfff55555,
	0x00155555,
	0x00055555,
	0xffeaaaab,
	0x00200000,
	0x00005105, // IIR Filter + decimation
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000001, // output gain
};

u32 coef_44to16[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00126104, // IIR Filter + interpolation
	0x00000af2, // input gain
	0x0057eebe,0xff1e9863,0x00652604,
	0xff7206ea,0xff22ad7e,0x006d47e1,
	0xff42a4d7,0xff26e722,0x0075fd83,
	0xff352f66,0xff29312b,0x007b986b,
	0xff310a07,0xff296f51,0x007eca7c,
	0x00000002, // output gain
	0x001d9204, // Farrow Filter + Decimation
	0x000aaaab,
	0xffaaaaab,
	0xfffaaaab,
	0x00555555,
	0xff600000,
	0xfff55555,
	0x00155555,
	0x00055555,
	0xffeaaaab,
	0x00200000,
	0x00005105, // IIR Filter + decimation
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000001, // output gain
};

u32 coef_48to8[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x000c9102, // IIR Filter + decimation
	0x00000e00, // input gain
	0x00e2e000,0xff6e1a00,0x002aaa00,
	0x00610a00,0xff5dda00,0x003ccc00,
	0x00163a00,0xff3c0400,0x00633200,
	0x00000001, // output gain
	0x00005105, // IIR Filter + Decimator
	0x0000d649, // input gain
	0x00e87afb, 0xff5f69d0, 0x003df3cf,
	0x007ce488, 0xff99a5c8, 0x0056a6a0,
	0x00344928, 0xffcba3e5, 0x006be470,
	0x00137aa7, 0xffe60276, 0x00773410,
	0x0005fa2a, 0xfff1ac11, 0x007c795b,
	0x00012d36, 0xfff5eca2, 0x007f10ef,
	0x00000001, // ouptut gain
};

u32 coef_48to16[TEGRA210_SFC_COEF_RAM_DEPTH] = {
	0x00009105, // IIR FIlter + Decimation
	0x00000784, // input gain
	0x00cc516e,0xff2c9639,0x005ad5b3,
	0x0013ad0d,0xff3d4799,0x0063ce75,
	0xffb6f398,0xff5138d1,0x006e9e1f,
	0xff9186e5,0xff5f96a4,0x0076a86e,
	0xff82089c,0xff676b81,0x007b9f8a,
	0xff7c48a5,0xff6a31e7,0x007ebb7b,
	0x00000001, // output gain
};

/* SFC APIs */
int tegra210_sfc_allocate(int *id);
int tegra210_sfc_free(int id);
int tegra210_sfc_set_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_sfc_get_acif_param(enum tegra210_ahub_cifs cif,
				struct tegra210_axbar_cif_param *acif);
int tegra210_sfc_set_rx_rate(int id, int fs_in);
int tegra210_sfc_set_tx_rate(int id, int fs_out);
int tegra210_sfc_enable(int id, bool en);
int tegra210_sfc_reset(int id);
int tegra210_sfc_set_slcg(int id, bool en);
int tegra210_sfc_enable_coef_ram(int id, bool en, int fs_in, int fs_out);

#endif
