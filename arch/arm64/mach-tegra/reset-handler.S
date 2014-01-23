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
#include <linux/init.h>

#include <asm/cache.h>
#include <asm/asm-offsets.h>
#include <asm/assembler.h>

#include "flowctrl.h"
#include "iomap.h"
#include "reset.h"

#define PMC_SCRATCH41	0x140

/* TEGRA_FLOW_CTRL_BASE = 0x60007000 */
#define TEGRA_FLOW_CTRL_BASE_LO16	0x7000
#define TEGRA_FLOW_CTRL_BASE_HI16	0x6000

/* returns the offset of the flow controller csr register for a cpu */
.macro cpu_to_csr_reg rd, rcpu
	cmp	\rcpu, #0
	beq 1f
	sub	\rd, \rcpu, #1
	lsl	\rd, \rd, #3
	add	\rd, \rd, #0x18
1:
	mov	\rd, #8
.endm

#ifdef CONFIG_PM_SLEEP
/*
 *	tegra_resume
 *
 *	  CPU boot vector when restarting the a CPU following
 *	  an LP2 transition. Also branched to by LP0 and LP1 resume after
 *	  re-enabling sdram.
 *
 *	x6: SoC ID << 8
 *	x8: CPU part number
 */
ENTRY(tegra_resume)
#ifdef CONFIG_SMP
	mrs	x0, mpidr_el1
	and x0, x0, #15
#else
	mov	x0, #0
#endif

	cmp	x0, #0				/* CPU0? */
	bne	done				/* no */

	/* Clear the flow controller flags for this CPU. */
	cpu_to_csr_reg	x1, x0
	mov	x2, #TEGRA_FLOW_CTRL_BASE_LO16
	movk x2, #TEGRA_FLOW_CTRL_BASE_HI16, lsl #16
	ldr	x1, [x2, x1]

	/* Clear event & intr flag */
	orr	x1, x1, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG

	/* enable, cluster_switch, immed, & bitmaps */
	movk	x0, #0x3ffd, lsl #16

	bic	x1, x1, x0
	str	x1, [x2]

done:
	b	cpu_resume
ENDPROC(tegra_resume)
#endif

	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_start)

/*
 * __tegra_cpu_reset_handler:
 *
 * Common handler for all CPU reset events.
 *
 * Register usage within the reset handler:
 *
 *      Others: scratch
 *      x6  = SoC ID << 8
 *      x7  = CPU present (to the OS) mask
 *      x8  = CPU in LP1 state mask
 *      x9  = CPU in LP2 state mask
 *      x10 = CPU number
 *      x11 = CPU mask
 *      x12 = pointer to reset handler data
 *
 * NOTE: This code is copied to IRAM. All code and data accesses
 *       must be position-independent.
 */

	.align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler)
	disable_irq

#ifdef CONFIG_SMP
	mrs	x10, mpidr_el1
#else
	mov x10, #0
#endif

	and	x10, x10, #0xf
	and	x10, x10, #0x3		/* x10 = cpu id */

	mov x11, #1
	lsl x11, x11, x10		/* x11 = CPU mask */

	adr x12, __tegra_cpu_reset_handler_data

#ifdef CONFIG_SMP
	/* Does the OS know about this CPU? */
	ldr	x7, [x12, #RESET_DATA(MASK_PRESENT)]
	tst	x7, x11 		/* if !present */
	beq		__die		/* CPU not present (to OS) */
#endif

	/* Waking up from LP1? */
	ldr	x8, [x12, #RESET_DATA(MASK_LP1)]
	tst	x8, x11				/* if in_lp1 */
	beq	__is_not_lp1
	cmp	x10, #0
	bne	__die				/* only CPU0 can be here */
	ldr	lr, [x12, #RESET_DATA(STARTUP_LP1)]
	cmp	lr, #0
	beq	__die				/* no LP1 startup handler */
	ret
__is_not_lp1:

	/* Waking up from LP2? */
	ldr	x9, [x12, #RESET_DATA(MASK_LP2)]
	tst	x9, x11				/* if in_lp2 */
	beq	__is_not_lp2
	ldr	lr, [x12, #RESET_DATA(STARTUP_LP2)]
	cmp	lr, #0
	beq	__die				/* no LP2 startup handler */
	ret

__is_not_lp2:

#ifdef CONFIG_SMP
	ldr	lr, [x12, #RESET_DATA(STARTUP_SECONDARY)]
	cmp	lr, #0
	beq	__die			/* no secondary startup handler */
	ret
#endif

/*
 * We don't know why the CPU reset. Just kill it.
 */

__die:
	/* just spin here for now */
	b	.
ENDPROC(__tegra_cpu_reset_handler)

	.align L1_CACHE_SHIFT
	.type	__tegra_cpu_reset_handler_data, %object
	.globl	__tegra_cpu_reset_handler_data
__tegra_cpu_reset_handler_data:
	.rept	TEGRA_RESET_DATA_SIZE
	.quad	0
	.endr
	.align L1_CACHE_SHIFT

ENTRY(__tegra_cpu_reset_handler_end)
