/*
 * arch/arm64/mach-tegra/sleep-tegra132.S
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/linkage.h>

#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/cache.h>

#include "sleep.h"
#include "pm-tegra132.h"

/* TEGRA_PMC_BASE 0x7000E400 */
#define TEGRA_PMC_BASE_LO16	0xE400
#define TEGRA_PMC_BASE_HI16	0x7000

#ifdef CONFIG_PM_SLEEP

/* START OF ROUTINES COPIED TO IRAM */
	.align L1_CACHE_SHIFT
	.global tegra132_iram_start
tegra132_iram_start:

/*
 * tegra132_lp1_reset
 *
 * reset vector for LP1 restore; copied into IRAM during suspend.
 * At this point, DPMU has already brought the system back up to
 * a safe staring point:
 *  - SDRAM out of self-refresh
    - PLLC, PLLM and PLLP reenabled
    - CPU running on PLLX,
    - system clock running on the same PLL that it suspended at

 * Here we simply jump to tegra_resume to restore virtual addressing.
 * The physical address of tegra_resume expected to be stored in
 * PMC_SCRATCH41.
 *
 * NOTE: THIS *MUST* BE RELOCATED TO TEGRA_IRAM_CODE_AREA AND MUST BE FIRST.
 */
ENTRY(tegra132_lp1_reset)
	mov	x0, #TEGRA_PMC_BASE_LO16
	movk	x0, #TEGRA_PMC_BASE_HI16, lsl #16
	ldr	x0, [x0, #PMC_SCRATCH41]
	br	x0			/* jump to tegra_resume */
ENDPROC(tegra132_lp1_reset)


	.ltorg
/* dummy symbol for end of IRAM */
	.align L1_CACHE_SHIFT
	.global tegra132_iram_end
tegra132_iram_end:
	b	.

#endif
