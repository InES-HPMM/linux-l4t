/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <asm/asm-offsets.h>
#include <asm/ptrace.h>
#include <linux/linkage.h>
#include <linux/init.h>

#ifdef CONFIG_ARM64

/* uint32_t tlk_generic_smc(uint32_t arg0, uint32_t arg1, uint32_t arg2) */
ENTRY(_tlk_generic_smc)
	smc	#0
	ret
ENDPROC(_tlk_generic_smc)

	/* allows MAX_EXT_SMC_ARGS (r0-r11) to be passed in registers */

/* uint32_t tlk_extended_smc(uint32_t *regs) */
ENTRY(_tlk_extended_smc)
	/*
	 * Allows MAX_EXT_SMC_ARGS (r0-r11) to be passed in registers
	 * (for aarch64, these are scratch, so no need to save them)
	 */
	mov	x12, x0
	ldp	x0, x1, [x12], #16
	ldp	x2, x3, [x12], #16
	ldp	x4, x5, [x12], #16
	ldp	x6, x7, [x12], #16
	ldp	x8, x9, [x12], #16
	ldp	x10, x11, [x12], #16
	smc	#0
	ret
ENDPROC(_tlk_extended_smc)

ENTRY(tlk_fiq_glue_aarch64)
	sub	sp, sp, #S_FRAME_SIZE - S_LR
	stp	x0, x1, [sp, #-16]!
	stp	x2, x3, [sp, #-16]!
	stp	x4, x5, [sp, #-16]!
	stp	x6, x7, [sp, #-16]!
	stp	x8, x9, [sp, #-16]!
	stp	x10, x11, [sp, #-16]!
	stp	x12, x13, [sp, #-16]!
	stp	x14, x15, [sp, #-16]!
	stp	x16, x17, [sp, #-16]!
	stp	x18, x19, [sp, #-16]!
	stp	x20, x21, [sp, #-16]!
	stp	x22, x23, [sp, #-16]!
	stp	x24, x25, [sp, #-16]!
	stp	x26, x27, [sp, #-16]!
	stp	x28, x29, [sp, #-16]!
	ldr	x0, =0x82000006		/* get FIQ regs */
	smc	#0
	stp	x0, x1, [sp, #S_PC]	/* original pc, cpsr */
	tst	x1, PSR_MODE_MASK
	csel	x2, x2, x3, eq		/* sp el0, sp el1 */
	stp	x30, x2, [sp, #S_LR]	/* lr, original sp */
	mov	x0, sp
	mov	x1, x3
	bl	tlk_fiq_handler		/* handle FIQ exception */
	b	.
ENDPROC(tlk_fiq_glue_aarch64)

#else

ENTRY(_tlk_generic_smc)
	smc	#0
	mov	pc, lr
ENDPROC(_tlk_generic_smc)

ENTRY(_tlk_extended_smc)
	stmfd	sp!, {r4-r12}	@ save reg state
	mov	r12, r0		@ reg ptr to r12
	ldmia	r12, {r0-r11}	@ load arg regs
	smc	#0
	ldmfd	sp!, {r4-r12}	@ restore saved regs
ENDPROC(_tlk_extended_smc)

#endif
