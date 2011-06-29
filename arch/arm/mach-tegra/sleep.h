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

#define TEGRA_IRAM_CODE_AREA		(TEGRA_IRAM_BASE + SZ_4K)

/* PMC_SCRATCH37-39 and 41 are used for tegra_pen_lock in idle */
#define PMC_SCRATCH37                   0x130
#define PMC_SCRATCH38                   0x134
/* PMC_SCRATCH39 stores the reset vector of the AVP (always 0) after LP0 */
#define PMC_SCRATCH39                   0x138
/* PMC_SCRATCH41 stores the reset vector of the CPU after LP0 and LP1 */
#define PMC_SCRATCH41                   0x140

#define CPU_RESETTABLE			2
#define CPU_RESETTABLE_SOON		1
#define CPU_NOT_RESETTABLE		0

#define FLOW_CTRL_WAITEVENT		(2 << 29)
#define FLOW_CTRL_STOP_UNTIL_IRQ	(4 << 29)
#define FLOW_CTRL_JTAG_RESUME		(1 << 28)
#define FLOW_CTRL_IRQ_RESUME		(1 << 10)
#define FLOW_CTRL_FIQ_RESUME		(1 << 8)

#define TEGRA_ARM_PERIF_VIRT (TEGRA_ARM_PERIF_BASE - IO_CPU_PHYS \
					+ IO_CPU_VIRT)
#define TEGRA_FLOW_CTRL_VIRT (TEGRA_FLOW_CTRL_BASE - IO_PPSB_PHYS \
					+ IO_PPSB_VIRT)
#define TEGRA_CLK_RESET_VIRT (TEGRA_CLK_RESET_BASE - IO_PPSB_PHYS \
					+ IO_PPSB_VIRT)

#ifdef __ASSEMBLY__
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
#else

#ifdef CONFIG_HOTPLUG_CPU
void tegra20_hotplug_init(void);
void tegra30_hotplug_init(void);
#else
static inline void tegra20_hotplug_init(void) {}
static inline void tegra30_hotplug_init(void) {}
#endif

/* assembly routines implemented in sleep.S */
void tegra_pen_lock(void);
void tegra_pen_unlock(void);
void tegra_cpu_wfi(void);
void tegra_cpu_set_resettable_soon(void);
int tegra_cpu_is_resettable_soon(void);

extern void tegra_lp1_reset;
extern void tegra_iram_start;
extern void tegra_iram_end;

void tegra_lp2_in_idle(bool enable);

void tegra_sleep_wfi(unsigned long v2p);
void tegra_sleep_cpu(unsigned long v2p);
void tegra_sleep_core(unsigned long v2p);
void tegra_resume(void);
void tegra_secondary_resume(void);

#endif

#endif
