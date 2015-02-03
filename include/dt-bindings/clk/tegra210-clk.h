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
/* FIXME: IDs 1 ... 127 to be assigned */
#define TEGRA210_CLK_ID_I2C5		128
/* FIXME: IDs 129 ... 154 to be assigned */
#define TEGRA210_CLK_ID_DFLL_SOC	155
/* FIXME: IDs 156 ... 186 to be assigned */
#define TEGRA210_CLK_ID_PLL_P_OUT_ADSP	187
/* FIXME: IDs 156 ... 221 to be assigned */
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
