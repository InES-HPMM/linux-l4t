/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _DT_BINDINGS_CLK_TEGRA210_CLK_ID_H
#define _DT_BINDINGS_CLK_TEGRA210_CLK_ID_H

/*
 * The first 224 clocks IDs are used for clocks that has individual enable
 * controls in T210 7 CAR's CLK_OUT_ENB registers. Although IDs do match the
 * enable bits numbers, clock manipulation does not rely on such match in any
 * way.
 */
#define TEGRA210_CLK_ID_CAR_BASE	0   /* Not assigned to any clock */
/* FIXME: IDs 1 ... 3 to be assigned */
#define TEGRA210_CLK_ID_RTC		4
#define TEGRA210_CLK_ID_TIMER		5
#define TEGRA210_CLK_ID_UARTA		6
#define TEGRA210_CLK_ID_UARTB		7
/* FIXME: ID 8 to be assigned */
#define TEGRA210_CLK_ID_SDMMC2		9
/* FIXME: ID 10 to be assigned */
#define TEGRA210_CLK_ID_I2S1		11
#define TEGRA210_CLK_ID_I2C1		12
/* ID 13 is reserved */
#define TEGRA210_CLK_ID_SDMMC1		14
#define TEGRA210_CLK_ID_SDMMC4		15
/* ID 16 is reserved */
#define TEGRA210_CLK_ID_PWM		17
#define TEGRA210_CLK_ID_I2S2		18
/* ID 19 is reserved */
/* FIXME: ID 20 to be assigned */
/* ID 21 is reserved */
#define TEGRA210_CLK_ID_USBD		22
/* FIXME: ID 23 to be assigned */
#define TEGRA210_CLK_ID_SATA_AUX	24
/* ID 25 is reserved */
#define TEGRA210_CLK_ID_DISP2		26
#define TEGRA210_CLK_ID_DISP1		27
#define TEGRA210_CLK_ID_HOST1X		28
#define TEGRA210_CLK_ID_VCP		29
#define TEGRA210_CLK_ID_I2S0		30
/* FIXME: IDs 31 ... 32 to be assigned */
#define TEGRA210_CLK_ID_AHBDMA		33
#define TEGRA210_CLK_ID_APBDMA		34
/* IDs 35 ... 36 are reserved */
/* FIXME: IDs 37 ... 39 to be assigned */
#define TEGRA210_CLK_ID_KFUSE		40
#define TEGRA210_CLK_ID_SBC1		41
/* IDs 42 ... 43 are reserved */
#define TEGRA210_CLK_ID_SBC2		44
/* FIXME: ID 45 to be assigned */
#define TEGRA210_CLK_ID_SBC3		46
#define TEGRA210_CLK_ID_I2C5		47
#define TEGRA210_CLK_ID_DSIA		48
/* IDs 49 ... 51 are reserved */
#define TEGRA210_CLK_ID_CSI		52
/* ID 53 is reserved */
#define TEGRA210_CLK_ID_I2C2		54
#define TEGRA210_CLK_ID_UARTC		55
/* FIXME: IDs 56 ... 57 to be assigned */
#define TEGRA210_CLK_ID_USB2		58
/* IDs 59 ... 62 are reserved */
#define TEGRA210_CLK_ID_BSEV		63
/* ID 64 is reserved */
#define TEGRA210_CLK_ID_UARTD		65
/* ID 66 is reserved */
#define TEGRA210_CLK_ID_I2C3		67
#define TEGRA210_CLK_ID_SBC4		68
#define TEGRA210_CLK_ID_SDMMC3		69
#define TEGRA210_CLK_ID_PCIE		70
#define TEGRA210_CLK_ID_OWR		71
#define TEGRA210_CLK_ID_AFI		72
#define TEGRA210_CLK_ID_CSITE		73
/* FIXME: ID 74 to be assigned */
/* ID 75 is reserved */
#define TEGRA210_CLK_ID_LA		76
/* ID 77 is reserved */
#define TEGRA210_CLK_ID_SOC_THERM	78
#define TEGRA210_CLK_ID_DTV		79
/* ID 80 is reserved */
#define TEGRA210_CLK_ID_I2CSLOW		81
#define TEGRA210_CLK_ID_DSIB		82
/* FIXME: IDs 83 ... 88 to be assigned */
#define TEGRA210_CLK_ID_XUSB_HOST	89
/* IDs 90 ... 91 are reserved */
/* FIXME: IDs 92 ... 94 to be assigned */
#define TEGRA210_CLK_ID_XUSB_DEV	95
/* FIXME: IDs 96 ... 97 to be assigned */
/* ID 98 is reserved */
/* FIXME: ID 99 to be assigned */
#define TEGRA210_CLK_ID_TSENSOR		100
#define TEGRA210_CLK_ID_I2S3		101
#define TEGRA210_CLK_ID_I2S4		102
#define TEGRA210_CLK_ID_I2C4		103
/* IDs 104 ... 105 are reserved */
#define TEGRA210_CLK_ID_AHUB		106
#define TEGRA210_CLK_ID_APB2APE		107
/* IDs 108 ... 110 are reserved */
#define TEGRA210_CLK_ID_HDA2CODEC_2X	111
/* IDs 112 ... 118 are reserved */
#define TEGRA210_CLK_ID_ACTMON		119
#define TEGRA210_CLK_ID_EXTERN1		120
#define TEGRA210_CLK_ID_EXTERN2		121
#define TEGRA210_CLK_ID_EXTERN3		122
#define TEGRA210_CLK_ID_SATA_OOB	123
#define TEGRA210_CLK_ID_SATA		124
#define TEGRA210_CLK_ID_HDA		125
/* FIXME: IDs 126 ... 127 to be assigned */
#define TEGRA210_CLK_ID_HDA2HDMI	128
#define TEGRA210_CLK_ID_SATA_COLD	129
/* FIXME: IDs 130 ... 135 to be assigned */
#define TEGRA210_CLK_ID_CEC		136
/* FIXME: IDs 137 ... 138 to be assigned */
/* ID 139 is reserved */
/* FIXME: IDs 140 ... 141 to be assigned */
#define TEGRA210_CLK_ID_XUSB_PADCTRL	142
#define TEGRA210_CLK_ID_XUSB_GATE	143
#define TEGRA210_CLK_ID_CILAB		144
#define TEGRA210_CLK_ID_CILCD		145
#define TEGRA210_CLK_ID_CILE		146
#define TEGRA210_CLK_ID_DSIALP		147
#define TEGRA210_CLK_ID_DSIBLP		148
#define TEGRA210_CLK_ID_ENTROPY		149
/* IDs 150 ... 151 are reserved */
#define TEGRA210_CLK_ID_DP2		152
/* IDs 153 ... 154 are reserved */
#define TEGRA210_CLK_ID_DFLL_SOC	155
#define TEGRA210_CLK_ID_XUSB_SS		156
/* IDs 157 ... 160 are reserved */
#define TEGRA210_CLK_ID_DMIC1		161
#define TEGRA210_CLK_ID_DMIC2		162
/* ID 163 is reserved */
#define TEGRA210_CLK_ID_VI_SENSOR	164
#define TEGRA210_CLK_ID_VI_SENSOR2	165
#define TEGRA210_CLK_ID_I2C6		166
#define TEGRA210_CLK_ID_MC_CAPA		167
#define TEGRA210_CLK_ID_MC_CBPA		168
#define TEGRA210_CLK_ID_MC_CPU		169
#define TEGRA210_CLK_ID_MC_BBC		170
/* FIXME: ID 171 to be assigned */
/* ID 172 is reserved */
#define TEGRA210_CLK_ID_MIPIIF		173
/* IDs 174 ... 176 are reserved */
#define TEGRA210_CLK_ID_UART_MIPI_CAL	177
/* FIXME: ID 178 to be assigned */
/* IDs 179 ... 180 are reserved */
#define TEGRA210_CLK_ID_DPAUX		181
#define TEGRA210_CLK_ID_SOR0		182
#define TEGRA210_CLK_ID_SOR1		183
#define TEGRA210_CLK_ID_GPU_GATE	184
#define TEGRA210_CLK_ID_DBGAPB		185
/* ID 186 is reserved */
#define TEGRA210_CLK_ID_PLL_P_OUT_ADSP	187
/* ID 188 is reserved */
#define TEGRA210_CLK_ID_GPU_REF		189
/* IDs 190 ... 191 are reserved */
#define TEGRA210_CLK_ID_SPARE1		192
#define TEGRA210_CLK_ID_SDMMC_LEGACY	193
/* FIXME: IDs 194 ... 195 to be assigned */
#define TEGRA210_CLK_ID_AXIAP		196
#define TEGRA210_CLK_ID_DMIC3		197
#define TEGRA210_CLK_ID_APE		198
/* ID 199 is reserved */
#define TEGRA210_CLK_ID_MC_CDPA		200
#define TEGRA210_CLK_ID_MC_CCPA		201
#define TEGRA210_CLK_ID_MAUD		202
/* ID 203 is reserved */
#define TEGRA210_CLK_ID_SATA_UPHY	204
#define TEGRA210_CLK_ID_PEX_UPHY	205
/* FIXME: ID 206 to be assigned */
#define TEGRA210_CLK_ID_DPAUX1		207
#define TEGRA210_CLK_ID_VII2C		208
#define TEGRA210_CLK_ID_HSIC_TRK	209
#define TEGRA210_CLK_ID_USB2_TRK	210
#define TEGRA210_CLK_ID_QSPI		211
#define TEGRA210_CLK_ID_UARTAPE		212
/* IDs 213 ... 218 are reserved */
/* FIXME: ID 219 to be assigned */
#define TEGRA210_CLK_ID_IQC2		220
#define TEGRA210_CLK_ID_IQC1		221
#define TEGRA210_CLK_ID_PLL_P_OUT_SOR	222
#define TEGRA210_CLK_ID_PLL_P_OUT_CPU	223

/*
 * Clock IDs in the 224 ... 351 range are used for PLL outputs; clocks that have
 * enable controls embedded into source selection registers; clocks that share
 * controls within CLK_OUT_ENB, but have different sources; clocks that do not
 * have enable controls at all.
 */
#define TEGRA210_CLK_ID_PLL_BASE	224 /* Not assigned to any clock */
#define TEGRA210_CLK_ID_CLK_32K		225
#define TEGRA210_CLK_ID_CLK_OSC		226
#define TEGRA210_CLK_ID_CLK_M		227
#define TEGRA210_CLK_ID_CLK_M_DIV2	228
#define TEGRA210_CLK_ID_CLK_M_DIV4	229
#define TEGRA210_CLK_ID_PLL_REF		230
#define TEGRA210_CLK_ID_PLL_A		231
#define TEGRA210_CLK_ID_PLL_A_OUT0	232
#define TEGRA210_CLK_ID_PLL_A1		233
#define TEGRA210_CLK_ID_PLL_C		234
#define TEGRA210_CLK_ID_PLL_C_OUT1	235
#define TEGRA210_CLK_ID_PLL_C2		236
#define TEGRA210_CLK_ID_PLL_C3		237
#define TEGRA210_CLK_ID_PLL_C4		238
#define TEGRA210_CLK_ID_PLL_C4_OUT0	239
#define TEGRA210_CLK_ID_PLL_C4_OUT1	240
#define TEGRA210_CLK_ID_PLL_C4_OUT2	241
#define TEGRA210_CLK_ID_PLL_C4_OUT3	242
#define TEGRA210_CLK_ID_PLL_D		243
#define TEGRA210_CLK_ID_PLL_D_OUT0	244
#define TEGRA210_CLK_ID_PLL_D2		245
#define TEGRA210_CLK_ID_PLL_DP		246
#define TEGRA210_CLK_ID_PLL_E		247
#define TEGRA210_CLK_ID_PLL_M		248
#define TEGRA210_CLK_ID_PLL_MB		249
#define TEGRA210_CLK_ID_PLL_RE		250
#define TEGRA210_CLK_ID_PLL_RE_OUT	251
#define TEGRA210_CLK_ID_PLL_RE_OUT1	252
#define TEGRA210_CLK_ID_PLL_P		253
#define TEGRA210_CLK_ID_PLL_P_OUT2	254
#define TEGRA210_CLK_ID_PLL_P_OUT3	255
#define TEGRA210_CLK_ID_PLL_P_OUT4	256
#define TEGRA210_CLK_ID_PLL_P_OUT5	257
#define TEGRA210_CLK_ID_PLL_P_OUT_HSIO	258
#define TEGRA210_CLK_ID_PLL_P_OUT_XUSB	259
/* IDs 260 ... 261 are not assigned */
#define TEGRA210_CLK_ID_PLL_X		262
#define TEGRA210_CLK_ID_PLL_U		263
#define TEGRA210_CLK_ID_PLL_U_OUT	264
#define TEGRA210_CLK_ID_PLL_U_OUT1	265
#define TEGRA210_CLK_ID_PLL_U_OUT2	266
#define TEGRA210_CLK_ID_PLL_U_480M	267
#define TEGRA210_CLK_ID_PLL_U_60M	268
#define TEGRA210_CLK_ID_PLL_U_48M	269
/* IDs 270 ... 279 are not assigned */
#define TEGRA210_CLK_ID_DFLL_CPU	280
#define TEGRA210_CLK_ID_DFLL_REF	281 /* Share enb bit w DFLL_SOC */
/* FIXME: IDs 282 ... 351 to be assigned */

/*
 * Clock IDs in the 352 ... 479 range are used identify tegra virtual clock
 * objects, shared buses, and shared bus user objects.
 */
#define TEGRA210_CLK_ID_VIRT_BASE	352 /* Not assigned to any clock */
#define TEGRA210_CLK_ID_CPU_G		353
#define TEGRA210_CLK_ID_CPU_LP		354
#define TEGRA210_CLK_ID_ADSP_CPU	355
/* FIXME: IDs 356 ... 479 to be assigned */

#endif /* _DT_BINDINGS_CLK_TEGRA210_CLK_ID_H */
