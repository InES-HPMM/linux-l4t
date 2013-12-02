/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/proc-fns.h>

#define OSDLR_DBL_LOCK_BIT	1
#define CPUECTLR_SMP_BIT	(1 << 6)

ENTRY(tegra210_hotplug_shutdown)
	bl	cpu_cache_off

	mrs	x0, s3_1_c15_c2_1
	mov	x1, #0x1
	orr	x0, x0, x1, lsl #38
	mov	x1, #0x3
	bic	x0, x0, x1, lsl #35
	bic	x0, x0, x1, lsl #32
	msr	s3_1_c15_c2_1, x0

	isb
	dsb	sy

	bl	__flush_dcache_all

	mrs	x0, s3_1_c15_c2_1
	bic	x0, x0, #CPUECTLR_SMP_BIT
	msr	s3_1_c15_c2_1, x0

	mrs	x0, osdlr_el1
	orr	x0, x0, #OSDLR_DBL_LOCK_BIT
	msr	osdlr_el1, x0

	isb
	dsb	sy

	wfi
ENDPROC(tegra210_hotplug_shutdown)
