/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/linkage.h>
#include <asm/assembler.h>

/* x0: retention control value, 0 => disable retention */
ENTRY(cpu_retention_enable)
	/* CPUECTLR_EL1[2:0] */
	mrs	x1, s3_1_c15_c2_1
	bic	x1, x1, #0x7
	and	x0, x0, #0x7
	orr	x1, x1, x0
	msr	s3_1_c15_c2_1, x1

	/* L2ECTLR_EL1[2:0] */
	mrs	x1, s3_1_c11_c0_3
	bic	x1, x1, #0x7
	and	x0, x0, #0x7
	orr	x1, x1, x0
	msr	s3_1_c11_c0_3, x1

	isb
	dsb	sy
	ret
ENDPROC(cpu_retention_enable)
