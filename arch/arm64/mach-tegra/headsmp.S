/* FIXME: temporary */
/*
 * arch/arm64/mach-tegra/headsmp.S
 *
 * CPU initialization routines for Tegra SoCs
 *
 * Copyright (c) 2009-2012, NVIDIA Corporation.
 * Copyright (c) 2011 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
 *         Gary King <gking@nvidia.com>
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

#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/cache.h>

#include "reset.h"

#ifdef CONFIG_SMP
/*
 *  tegra_secondary_startup
 *
 *   Initial secondary processor boot vector; jumps to kernel's
 *   secondary_startup routine. Used for initial boot and hotplug
 *   of secondary CPUs.
 */
    __CPUINIT
ENTRY(tegra_secondary_startup)
    bl  __invalidate_cpu_state
    b   secondary_startup
ENDPROC(tegra_secondary_startup)
#endif

        .section ".text.head", "ax"
#ifdef CONFIG_PM_SLEEP
/*
 *  tegra_resume
 *
 *    CPU boot vector when restarting the a CPU following
 *    an LP2 transition. Also branched to by LP0 and LP1 resume after
 *    re-enabling sdram.
 */
ENTRY(tegra_resume)
    bl  __invalidate_cpu_state

#ifdef CONFIG_TRUSTED_FOUNDATIONS
    /* wake up (should have specified args?) */
    bl  tegra_generic_smc
#endif

    b   cpu_resume
ENDPROC(tegra_resume)
#endif

/*
 *  __invalidate_cpu_state
 *
 *    Invalidates volatile CPU state (SCU tags, caches, branch address
 *    arrays, exclusive monitor, etc.) so that they can be safely enabled
 *    instruction caching and branch predicition enabled
 */
__invalidate_cpu_state:
    nop

enable_icache_bp:
    nop
    /* fall through */

/*
 *  tegra_invalidate_cache
 *
 *    Invalidates the L1 or L2 data cache (no clean) during initial boot of
 *    a cpu. For architecture with external L2, invalidate L1 only. For
 *    architecture with integrated L2 and SCU, invalidate L2 if current CPU
 *    boots up with a power gated NC partition initially or power rail was
 *    initially off, invalidates L1 in other cases
 *
 *    Corrupted registers: r0-r6
 */
tegra_invalidate_cache:
    ret

/*
 * __tegra_cpu_reset_handler_halt_failed:
 *
 * Alternate entry point for reset handler for cases where the
 * WFI halt failed to take effect.
 *
 */
    .align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_start)

/*
 * __tegra_cpu_reset_handler:
 *
 * Common handler for all CPU reset events.
 *
 * Register usage within the reset handler:
 *
 *      R7  = CPU present (to the OS) mask
 *      R8  = CPU in LP1 state mask
 *      R9  = CPU in LP2 state mask
 *      R10 = CPU number
 *      R11 = CPU mask
 *      R12 = pointer to reset handler data
 *
 * NOTE: This code is copied to IRAM. All code and data accesses
 *       must be position-independent.
 */

    .align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler)
    ret
ENDPROC(__tegra_cpu_reset_handler)

    .align L1_CACHE_SHIFT
    .type   __tegra_cpu_reset_handler_data, %object
    .globl  __tegra_cpu_reset_handler_data
__tegra_cpu_reset_handler_data:
    .rept   TEGRA_RESET_DATA_SIZE
    .long   0
    .endr
    .size   __tegra_cpu_reset_handler_data, . - __tegra_cpu_reset_handler_data
    .align L1_CACHE_SHIFT
ENTRY(__tegra_cpu_reset_handler_end)
