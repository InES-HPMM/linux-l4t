#include <linux/linkage.h>
#include <asm/assembler.h>

/*
 * tegra_pen_lock
 *
 * spinlock implementation with no atomic test-and-set and no coherence
 * using Peterson's algorithm on strongly-ordered registers
 * used to synchronize a cpu waking up from wfi with entering lp2 on idle
 *
 * SCRATCH37 = x1 = !turn (inverted from Peterson's algorithm)
 * on cpu 0:
 * SCRATCH38 = x2 = flag[0]
 * SCRATCH39 = x3 = flag[1]
 * on cpu1:
 * SCRATCH39 = x2 = flag[1]
 * SCRATCH38 = x3 = flag[0]
 *
 * must be called with MMU on
 * corrupts x0-x3, x12
 */
ENTRY(tegra_pen_lock)
    ret          // locked
ENDPROC(tegra_pen_lock)

ENTRY(tegra_pen_unlock)
    ret          // locked
ENDPROC(tegra_pen_unlock)

/*
 * tegra_cpu_exit_coherency
 *
 * Exits SMP coherency.
 * corrupts x4-x5
 */
ENTRY(tegra_cpu_exit_coherency)
    ret
ENDPROC(tegra_cpu_exit_coherency)

/*
 * tegra_flush_cache
 *
 * clean & invalidate inner cache
 *
 * Disable is needed before flush to prevent allocations during flush
 * When cache is disabled, we cannot push to stack.
 */
ENTRY(tegra_flush_cache)
    ret
ENDPROC(tegra_flush_cache)

/*
 * tegra_flush_l1_cache
 *
 * clean & invalidate the L1 cache
 *
 * The flush_cache_all flushes all caches within level of coherence, this
 * may not be desired if all we need is to flush L1 only. Therefore this
 * function is implemented to flush the L1 cache only.
 *
 * Disable is needed before flush to prevent allocations during flush
 * When cache is disabled, we cannot push to stack.
 *
 * Corrupted registers: x0-x7, x9-x11
 */
ENTRY(tegra_flush_l1_cache)
    ret
ENDPROC(tegra_flush_l1_cache)

#ifdef CONFIG_PM_SLEEP
/*
 * tegra_sleep_cpu_finish(unsigned long int)
 *
 * enters suspend in LP2 by turning off the mmu and jumping to
 * tegra?_tear_down_cpu
 */
ENTRY(tegra_sleep_cpu_finish)
    b   tegra_turn_off_mmu
ENDPROC(tegra_sleep_cpu_finish)

/*
 * tegra_turn_off_mmu
 *
 * x0 = v2p
 * x1 = physical address to jump to with mmu off
 */
ENTRY(tegra_turn_off_mmu)
    mov   x3, tegra_shut_off_mmu
    add x3, x3, x0
    mov x0, x1
    br x3
ENDPROC(tegra_turn_off_mmu)

tegra_pgd_phys_address:

/*
 * tegra_shut_off_mmu
 *
 * x0 = physical address to jump to with mmu off
 *
 * called with VA=PA mapping
 * turns off MMU, icache, dcache and branch prediction
 */
tegra_shut_off_mmu:
    br x0

/*
 * tegra_cpu_clk32k
 *
 * In LP2 the normal cpu clock pllx will be turned off. Switch the CPU to pllp
 */
ENTRY(tegra_cpu_pllp)
    ret
ENDPROC(tegra_cpu_pllp)
#endif

#ifdef CONFIG_TRUSTED_FOUNDATIONS
/*
 * tegra_generic_smc
 *
 * x0 = smc type
 * x1 = smc subtype
 * x2 = argument passed to smc
 *
 * issues SMC (secure monitor call) instruction with
 * the specified parameters.
 */
ENTRY(tegra_generic_smc)
    nop
ENDPROC(tegra_generic_smc)
    .type   __tegra_smc_stack, %object
__tegra_smc_stack:
    .long   0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    .size   __tegra_smc_stack, . - __tegra_smc_stack
#endif
