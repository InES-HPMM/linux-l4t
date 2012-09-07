/*
 * arch/arm/mach-tegra/tegra11x_la_priv.h
 *
 * Copyright (C) 2012, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_TEGRA_TEGRA11x_LA_PRIV_H_
#define _MACH_TEGRA_TEGRA11x_LA_PRIV_H_

#if defined(CONFIG_ARCH_TEGRA_11x_SOC)

#define MC_LA_AVPC_ARM7_0	0x2e4
#define MC_LA_DC_0		0x2e8
#define MC_LA_DC_1		0x2ec
#define MC_LA_DC_2		0x2f0
#define MC_LA_DCB_0		0x2f4
#define MC_LA_DCB_1		0x2f8
#define MC_LA_DCB_2		0x2fc
#define MC_LA_EPP_0		0x300
#define MC_LA_EPP_1		0x304
#define MC_LA_G2_0		0x308
#define MC_LA_G2_1		0x30c
#define MC_LA_HC_0		0x310
#define MC_LA_HC_1		0x314
#define MC_LA_HDA_0		0x318
#define MC_LA_ISP_0		0x31C
#define MC_LA_MPCORE_0		0x320
#define MC_LA_MPCORELP_0	0x324
#define MC_LA_MSENC_0		0x328
#define MC_LA_NV_0		0x334
#define MC_LA_NV_1		0x338
#define MC_LA_NV2_0		0x33c
#define MC_LA_NV2_1		0x340
#define MC_LA_PPCS_0		0x344
#define MC_LA_PPCS_1		0x348
#define MC_LA_PTC_0		0x34c

#define MC_LA_VDE_0		0x354
#define MC_LA_VDE_1		0x358
#define MC_LA_VDE_2		0x35c
#define MC_LA_VDE_3		0x360
#define MC_LA_VI_0		0x364
#define MC_LA_VI_1		0x368
#define MC_LA_VI_2		0x36c

#define MC_LA_XUSB_0		0x37c /* T11x specific*/
#define MC_LA_XUSB_1		0x380 /* T11x specific*/
#define MC_LA_NV_2		0x384 /* T11x specific*/
#define MC_LA_NV_3		0x388 /* T11x specific*/

#define MC_LA_EMUCIF_0		0x38c
#define MC_LA_TSEC_0		0x390

/*
 * The rule for getting the fifo_size_in_atoms is:
 * 1.If REORDER_DEPTH exists, use it(default is overridden).
 * 2.Else if (write_client) use RFIFO_DEPTH.
 * 3.Else (read client) use RDFIFO_DEPTH.
 * Multiply the value by 2 for dual channel.
 * Multiply the value by 2 for wide clients.
 * A client is wide, if CMW is larger than MW.
 * Refer to project.h file.
 */
struct la_client_info la_info_array[] = {
	LA_INFO(3,	150,	AVPC_ARM7_0, 7 : 0,	AVPC_ARM7R,	false),
	LA_INFO(3,	150,	AVPC_ARM7_0, 23 : 16,	AVPC_ARM7W,	false),
	LA_INFO(256,	1050,	DC_0,	7 : 0,		DISPLAY_0A,	true),
	LA_INFO(256,	1050,	DC_0,	23 : 16,	DISPLAY_0B,	true),
	LA_INFO(256,	1050,	DC_1,	7 : 0,		DISPLAY_0C,	true),
	LA_INFO(96,	1050,	DC_2,	7 : 0,		DISPLAY_HC,	false),
	LA_INFO(256,	1050,	DCB_0,	7 : 0,		DISPLAY_0AB,	true),
	LA_INFO(256,	1050,	DCB_0,	23 : 16,	DISPLAY_0BB,	true),
	LA_INFO(256,	1050,	DCB_1,	7 : 0,		DISPLAY_0CB,	true),
	LA_INFO(96,	1050,	DCB_2,	7 : 0,		DISPLAY_HCB,	false),
	LA_INFO(16,	150,	EPP_0,	7 : 0,		EPPUP,		false),
	LA_INFO(64,	150,	EPP_0,	23 : 16,	EPPU,		false),
	LA_INFO(64,	150,	EPP_1,	7 : 0,		EPPV,		false),
	LA_INFO(64,	150,	EPP_1,	23 : 16,	EPPY,		false),
	LA_INFO(128,	150,	G2_0,	7 : 0,		G2PR,		false),
	LA_INFO(128,	150,	G2_0,	23 : 16,	G2SR,		false),
	LA_INFO(96,	150,	G2_1,	7 : 0,		G2DR,		false),
	LA_INFO(256,	150,	G2_1,	23 : 16,	G2DW,		false),
	LA_INFO(32,	150,	HC_0,	7 : 0,		HOST1X_DMAR,	false),
	LA_INFO(16,	150,	HC_0,	23 : 16,	HOST1XR,	false),
	LA_INFO(64,	150,	HC_1,	7 : 0,		HOST1XW,	false),
	LA_INFO(32,	150,	HDA_0,	7 : 0,		HDAR,		false),
	LA_INFO(32,	150,	HDA_0,	23 : 16,	HDAW,		false),
	LA_INFO(128,	150,	ISP_0,	7 : 0,		ISPW,		false),
	LA_INFO(96,	150,	MPCORE_0,   7 : 0,	MPCORER,	false),
	LA_INFO(128,	150,	MPCORE_0,   23 : 16,	MPCOREW,	false),
	LA_INFO(96,	150,	MPCORELP_0, 7 : 0,	MPCORE_LPR,	false),
	LA_INFO(128,	150,	MPCORELP_0, 23 : 16,	MPCORE_LPW,	false),
	LA_INFO(128,	150,	NV_0,	7 : 0,		FDCDRD,		false),
	LA_INFO(256,	150,	NV_0,	23 : 16,	IDXSRD,		false),
	LA_INFO(432,	150,	NV_1,	7 : 0,		TEXL2SRD,	false),
	LA_INFO(128,	150,	NV_1,	23 : 16,	FDCDWR,		false),
	LA_INFO(128,	150,	NV2_0,	7 : 0,		FDCDRD2,	false),
	LA_INFO(128,	150,	NV2_1,	23 : 16,	FDCDWR2,	false),
	LA_INFO(8,	150,	PPCS_0,	7 : 0,		PPCS_AHBDMAR,	false),
	LA_INFO(80,	150,	PPCS_0,	23 : 16,	PPCS_AHBSLVR,	false),
	LA_INFO(16,	150,	PPCS_1,	7 : 0,		PPCS_AHBDMAW,	false),
	LA_INFO(80,	150,	PPCS_1,	23 : 16,	PPCS_AHBSLVW,	false),
	LA_INFO(40,	150,	PTC_0,	7 : 0,		PTCR,		false),
	LA_INFO(16,	150,	VDE_0,	7 : 0,		VDE_BSEVR,	false),
	LA_INFO(8,	150,	VDE_0,	23 : 16,	VDE_MBER,	false),
	LA_INFO(64,	150,	VDE_1,	7 : 0,		VDE_MCER,	false),
	LA_INFO(32,	150,	VDE_1,	23 : 16,	VDE_TPER,	false),
	LA_INFO(8,	150,	VDE_2,	7 : 0,		VDE_BSEVW,	false),
	LA_INFO(32,	150,	VDE_2,	23 : 16,	VDE_DBGW,	false),
	LA_INFO(16,	150,	VDE_3,	7 : 0,		VDE_MBEW,	false),
	LA_INFO(32,	150,	VDE_3,	23 : 16,	VDE_TPMW,	false),
	LA_INFO(128,	1050,	VI_0,	23 : 16,	VI_WSB,		true),
	LA_INFO(128,	1050,	VI_1,	7 : 0,		VI_WU,		true),
	LA_INFO(128,	1050,	VI_1,	23 : 16,	VI_WV,		true),
	LA_INFO(128,	1050,	VI_2,	7 : 0,		VI_WY,		true),

	LA_INFO(128,	150,	MSENC_0,    7 : 0,	MSENCSRD,	false),
	LA_INFO(32,	150,	MSENC_0,    23 : 16,	MSENCSWR,	false),
	LA_INFO(160,	150,	XUSB_0,	    7 : 0,	XUSB_HOSTR,	false),
	LA_INFO(160,	150,	XUSB_0,	    23 : 16,	XUSB_HOSTW,	false),
	LA_INFO(160,	150,	XUSB_1,	    7 : 0,	XUSB_DEVR,	false),
	LA_INFO(160,	150,	XUSB_1,	    23 : 16,	XUSB_DEVW,	false),
	LA_INFO(128,	150,	NV_2,	    7 : 0,	FDCDRD3,	false),
	LA_INFO(128,	150,	NV_2,	    23 : 16,	FDCDRD4,	false),
	LA_INFO(128,	150,	NV_3,	    7 : 0,	FDCDWR3,	false),
	LA_INFO(128,	150,	NV_3,	    23 : 16,	FDCDWR4,	false),
	LA_INFO(28,	150,	EMUCIF_0,   7 : 0,	EMUCIFR,	false),
	LA_INFO(48,	150,	EMUCIF_0,   23 : 16,	EMUCIFW,	false),
	LA_INFO(32,	150,	TSEC_0,	    7 : 0,	TSECSRD,	false),
	LA_INFO(32,	150,	TSEC_0,	    23 : 16,	TSECSWR,	false),

/* end of list. */
	LA_INFO(0,	0,	TSEC_0,	    0 : 0,	MAX_ID,		false)
};

static int ns_per_tick = 30;
/* MC atom size in bytes */
static const int normal_atom_size = 16;
#endif

#endif /* _MACH_TEGRA_TEGRA11x_LA_PRIV_H_ */
