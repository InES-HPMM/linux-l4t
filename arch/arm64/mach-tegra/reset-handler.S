/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <asm/memory.h>

#include <linux/platform/tegra/flowctrl.h>
#include "iomap.h"
#include <linux/platform/tegra/reset.h>

#define PMC_SCRATCH41	0x140
#define INVALID_HWID	~0

#define CNTFRQ	12000000
#define GIC_DIST_BASE	0x50041000
#define GIC_CPU_BASE	0x50042000

/* returns the offset of the flow controller csr register for a cpu */
.macro cpu_to_csr_reg rd, rcpu
	cmp	\rcpu, #0
	beq 1f
	sub	\rd, \rcpu, #1
	lsl	\rd, \rd, #3
	add	\rd, \rd, #0x18
	b	2f
1:
	mov	\rd, #8
2:
.endm

#ifdef CONFIG_PM_SLEEP
/*
 *	CPU boot vector after LP0 exit.
 */
ENTRY(tegra_resume)
	/*
	 * EL3 initialisation
	 */
	mrs	x0, CurrentEL
	cmp	x0, #0xc			// EL3?
	b.ne	start_ns			// skip EL3 initialisation

	mov	x0, #0x30				// RES1
	orr	x0, x0, #(1 << 10)		// 64-bit EL1 (skip EL2)
	msr	scr_el3, x0

	msr	cptr_el3, xzr			// Disable copro. traps to EL3
	msr	sctlr_el2, xzr

	ldr	x0, =CNTFRQ
	msr	cntfrq_el0, x0

	/*
	 * Check for the primary CPU to avoid a race on the distributor
	 * registers.
	 */
	mrs	x0, mpidr_el1
	tst	x0, #15
	b.ne	1f				// secondary CPU

	ldr	x1, =GIC_DIST_BASE		// GICD_CTLR
	mov	w0, #3				// EnableGrp0 | EnableGrp1
	str	w0, [x1]

1:	ldr	x1, =GIC_DIST_BASE + 0x80	// GICD_IGROUPR
	mov	w0, #0				// Grp0 (secure) interrupts
	str	w0, [x1], #4			// SGI and PPI (#0~31)
	b.ne	2f				// Only local interrupts for secondary CPUs
	str	w0, [x1], #4			// SPI (#32~191)
	str	w0, [x1], #4
	str	w0, [x1], #4
	str	w0, [x1], #4
	str	w0, [x1], #4

2:	ldr	x1, =GIC_CPU_BASE		// GICC_CTLR
	ldr	w0, [x1]
	mov	w0, #3				// EnableGrp0 | EnableGrp1
	str	w0, [x1]

	mov	w0, #1 << 7			// allow NS access to GICC_PMR
	str	w0, [x1, #4]			// GICC_PMR

	msr	sctlr_el2, xzr

	/*
	 * Prepare the switch to the secure EL1_SP1 mode from EL3
	 */
	adr	x0, start_ns			// Return after mode switch
	mov	x1, #0x3c5              // EL1_SP1 | D | A | I | F
	msr	elr_el3, x0
	msr	spsr_el3, x1
	eret

start_ns:
	mrs	x0, mpidr_el1
	and x0, x0, #15

	/* Clear the flow controller flags for this CPU. */
	cpu_to_csr_reg	x1, x0
	ldr	x2, =TEGRA_FLOW_CTRL_BASE
	ldr	x3, [x2, x1]

	/* Clear event & intr flag */
	orr	x3, x3, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG

	/* enable, cluster_switch, immed, & bitmaps */
	movz	x0, #0x3ffd

	bic	x3, x3, x0
	str	x3, [x2, x1]

done:
	b	cpu_resume
ENDPROC(tegra_resume)
#endif

#ifdef CONFIG_PM_SLEEP
/* start of timestamps */
.globl tegra_resume_timestamps_start
tegra_resume_timestamps_start:

.globl tegra_resume_smc_entry_time
tegra_resume_smc_entry_time:
	.long	0
.globl tegra_resume_smc_exit_time
tegra_resume_smc_exit_time:
	.long	0
.globl tegra_resume_entry_time
tegra_resume_entry_time:
	.long	0

/* end of timestamps */
.global tegra_resume_timestamps_end
tegra_resume_timestamps_end:
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
