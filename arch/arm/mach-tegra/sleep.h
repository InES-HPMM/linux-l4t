/*
 * Copyright (c) 2010-2012, NVIDIA Corporation. All rights reserved.
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

#ifndef __MACH_TEGRA_SLEEP_H
#define __MACH_TEGRA_SLEEP_H

#include "iomap.h"

#ifndef CONFIG_TRUSTED_FOUNDATIONS
/* FIXME: The code associated with this should be removed if our change to
   save the diagnostic regsiter in the CPU context is accepted. */
#define USE_TEGRA_DIAG_REG_SAVE	1
#else
#define USE_TEGRA_DIAG_REG_SAVE	0
#endif

#define TEGRA_POWER_LP1_AUDIO		(1 << 25) /* do not turn off pll-p in LP1 */
#define TEGRA_POWER_SDRAM_SELFREFRESH	(1 << 26) /* SDRAM is in self-refresh */
#define TEGRA_POWER_HOTPLUG_SHUTDOWN	(1 << 27) /* Hotplug shutdown */
#define TEGRA_POWER_CLUSTER_G		(1 << 28) /* G CPU */
#define TEGRA_POWER_CLUSTER_LP		(1 << 29) /* LP CPU */
#define TEGRA_POWER_CLUSTER_MASK	(TEGRA_POWER_CLUSTER_G | \
						TEGRA_POWER_CLUSTER_LP)
#define TEGRA_POWER_CLUSTER_IMMEDIATE	(1 << 30) /* Immediate wake */
#define TEGRA_POWER_CLUSTER_FORCE	(1 << 31) /* Force switch */

#define TEGRA_IRAM_CODE_AREA		(TEGRA_IRAM_BASE + SZ_4K)

/* PMC_SCRATCH37-39 and 41 are used for tegra_pen_lock in Tegra2 idle */
#define PMC_SCRATCH37                   0x130
#define PMC_SCRATCH38                   0x134
/* PMC_SCRATCH39 stores the reset vector of the AVP (always 0) after LP0 */
#define PMC_SCRATCH39                   0x138
/* PMC_SCRATCH41 stores the reset vector of the CPU after LP0 and LP1 */
#define PMC_SCRATCH41                   0x140

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#define CPU_RESETTABLE			2
#define CPU_RESETTABLE_SOON		1
#define CPU_NOT_RESETTABLE		0
#endif

#define TEGRA_PL310_VIRT (TEGRA_ARM_PL310_BASE - IO_CPU_PHYS + IO_CPU_VIRT)
#define TEGRA_ARM_PERIF_VIRT (TEGRA_ARM_PERIF_BASE - IO_CPU_PHYS \
					+ IO_CPU_VIRT)
#define TEGRA_FLOW_CTRL_VIRT (TEGRA_FLOW_CTRL_BASE - IO_PPSB_PHYS \
					+ IO_PPSB_VIRT)
#define TEGRA_CLK_RESET_VIRT (TEGRA_CLK_RESET_BASE - IO_PPSB_PHYS \
					+ IO_PPSB_VIRT)

#ifdef __ASSEMBLY__
/* waits until the microsecond counter (base) ticks, for exact timing loops */
.macro  wait_for_us, rd, base, tmp
	ldr    \rd, [\base]
1001:   ldr    \tmp, [\base]
	cmp    \rd, \tmp
	beq    1001b
	mov    \tmp, \rd
.endm

/* waits until the microsecond counter (base) is > rn */
.macro	wait_until, rn, base, tmp
	add	\rn, \rn, #1
1002:	ldr	\tmp, [\base]
	sub	\tmp, \tmp, \rn
	ands	\tmp, \tmp, #0x80000000
	dmb
	bne	1002b
.endm

/* returns the offset of the flow controller halt register for a cpu */
.macro cpu_to_halt_reg rd, rcpu
	cmp	\rcpu, #0
	subne	\rd, \rcpu, #1
	movne	\rd, \rd, lsl #3
	addne	\rd, \rd, #0x14
	moveq	\rd, #0
.endm

/* returns the offset of the flow controller csr register for a cpu */
.macro cpu_to_csr_reg rd, rcpu
	cmp	\rcpu, #0
	subne	\rd, \rcpu, #1
	movne	\rd, \rd, lsl #3
	addne	\rd, \rd, #0x18
	moveq	\rd, #8
.endm

/* returns the ID of the current processor */
.macro cpu_id, rd
	mrc	p15, 0, \rd, c0, c0, 5
	and	\rd, \rd, #0xF
.endm

/* loads a 32-bit value into a register without a data access */
.macro mov32, reg, val
	movw	\reg, #:lower16:\val
	movt	\reg, #:upper16:\val
.endm

/* Macro to exit SMP coherency. */
.macro exit_smp, tmp1, tmp2
	mrc	p15, 0, \tmp1, c1, c0, 1	@ ACTLR
	bic	\tmp1, \tmp1, #(1<<6) | (1<<0)	@ clear ACTLR.SMP | ACTLR.FW
	mcr	p15, 0, \tmp1, c1, c0, 1	@ ACTLR
	isb
	cpu_id	\tmp1
	mov	\tmp1, \tmp1, lsl #2
	mov	\tmp2, #0xf
	mov	\tmp2, \tmp2, lsl \tmp1
	mov32	\tmp1, TEGRA_ARM_PERIF_VIRT + 0xC
	str	\tmp2, [\tmp1]			@ invalidate SCU tags for CPU
	dsb
.endm

#define DEBUG_CONTEXT_STACK	0

/* pops a debug check token from the stack */
.macro	pop_stack_token tmp1, tmp2
#if DEBUG_CONTEXT_STACK
	mov32	\tmp1, 0xBAB1F00D
	ldmfd	sp!, {\tmp2}
	cmp	\tmp1, \tmp2
	movne	pc, #0
#endif
.endm

/* pushes a debug check token onto the stack */
.macro	push_stack_token tmp1
#if DEBUG_CONTEXT_STACK
	mov32	\tmp1, 0xBAB1F00D
	stmfd	sp!, {\tmp1}
#endif
.endm

#else	/* !defined(__ASSEMBLY__) */

#ifdef CONFIG_HOTPLUG_CPU
void tegra20_hotplug_init(void);
void tegra30_hotplug_init(void);
#else
static inline void tegra20_hotplug_init(void) {}
static inline void tegra30_hotplug_init(void) {}
#endif

void tegra_pen_lock(void);
void tegra_pen_unlock(void);
void tegra_cpu_wfi(void);
int tegra_sleep_cpu_finish(unsigned long v2p);
void tegra_resume(void);
void tegra_cpu_resume(void);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
extern void tegra2_iram_start;
extern void tegra2_iram_end;
int  tegra2_cpu_is_resettable_soon(void);
void tegra2_cpu_reset(int cpu);
void tegra2_cpu_set_resettable_soon(void);
void tegra2_cpu_clear_resettable(void);
int tegra2_sleep_core_finish(unsigned long int);
void tegra2_sleep_wfi(unsigned long v2p);
#else
extern void tegra3_iram_start;
extern void tegra3_iram_end;
int tegra3_sleep_core_finish(unsigned long int);
int tegra3_sleep_cpu_secondary_finish(unsigned long int);
#endif

static inline void *tegra_iram_start(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return &tegra2_iram_start;
#else
	return &tegra3_iram_start;
#endif
}

static inline void *tegra_iram_end(void)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	return &tegra2_iram_end;
#else
	return &tegra3_iram_end;
#endif
}
#endif
#endif
