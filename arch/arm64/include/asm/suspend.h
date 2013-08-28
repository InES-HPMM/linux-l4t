#ifndef __ASM_ARM_SUSPEND_H
#define __ASM_ARM_SUSPEND_H

struct sleep_save_sp {
	u64 *save_ptr_stash;
	phys_addr_t save_ptr_stash_phys;
};

extern void cpu_resume(void);
extern int cpu_suspend(unsigned long, int (*)(unsigned long));

#endif
