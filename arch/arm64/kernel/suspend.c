#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <asm/memory.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>
#include <asm/tlbflush.h>

extern int __cpu_suspend(unsigned long, int (*)(unsigned long));
/*
 * This is called by __cpu_suspend() to save the state, and do whatever
 * flushing is required to ensure that when the CPU goes to sleep we have
 * the necessary data available when the caches are not searched.
 *
 * @ptr: CPU context virtual address
 * @save_ptr: address of the location where the context physical address
 *            must be saved
 */
void __cpu_suspend_save(struct cpu_suspend_ctx *ptr, u64 *save_ptr)
{
	*save_ptr = virt_to_phys(ptr);

	ptr->ttbr0_el1 = virt_to_phys(idmap_pg_dir);

	cpu_do_suspend(ptr);
	/*
	 * Only flush the context that must be retrieved with the MMU
	 * off. VA primitives ensure the flush is applied to all
	 * cache levels so context is pushed to DRAM.
	 */
	__flush_dcache_area(ptr, sizeof(*ptr));
	__flush_dcache_area(save_ptr, sizeof(*save_ptr));
}

/*
 * This hook is provided so that cpu_suspend code can restore HW
 * breakpoints as early as possible in the resume path, before reenabling
 * debug exceptions. Code cannot be run from a CPU PM notifier since by the
 * time the notifier runs debug exceptions might have been enabled already,
 * with HW breakpoints registers content still in an unknown state.
 */
void (*hw_breakpoint_restore)(void *);
void __init cpu_suspend_set_dbg_restorer(void (*hw_bp_restore)(void *))
{
	/* Prevent multiple restore hook initializations */
	if (WARN_ON(hw_breakpoint_restore))
		return;
	hw_breakpoint_restore = hw_bp_restore;
}

/**
 * cpu_suspend
 *
 * @arg: argument to pass to the finisher function
 * @fn: suspend finisher function, the function that executes last
 *	operations required to suspend a processor
 */
int cpu_suspend(unsigned long arg, int (*fn)(unsigned long))
{
	struct mm_struct *mm = current->active_mm;
	int ret;
	/*
	 * Save the mm context on the stack, it will be restored when
	 * the cpu comes out of reset through the identity mapped
	 * page tables, so that the thread address space is properly
	 * set-up on function return.
	 */
	ret = __cpu_suspend(arg, fn);
	if (ret == 0) {
		cpu_switch_mm(mm->pgd, mm);
		flush_tlb_all();
		/*
		 * Restore HW breakpoint registers to sane values
		 * before debug exceptions are possibly reenabled
		 * through local_dbg_restore.
		 */
		if (hw_breakpoint_restore)
			hw_breakpoint_restore(NULL);
	}

	return ret;
}

extern struct sleep_save_sp sleep_save_sp;

static int cpu_suspend_alloc_sp(void)
{
	void *ctx_ptr;
	/* ctx_ptr is an array of physical addresses */
	ctx_ptr = kcalloc(mpidr_hash_size(), sizeof(phys_addr_t), GFP_KERNEL);

	if (WARN_ON(!ctx_ptr))
		return -ENOMEM;
	sleep_save_sp.save_ptr_stash = ctx_ptr;
	sleep_save_sp.save_ptr_stash_phys = virt_to_phys(ctx_ptr);
	__flush_dcache_area(&sleep_save_sp, sizeof(struct sleep_save_sp));
	return 0;
}
early_initcall(cpu_suspend_alloc_sp);
