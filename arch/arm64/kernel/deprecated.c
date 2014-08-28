#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/perf_event.h>

#include <asm/opcodes.h>
#include <asm/traps.h>
#include <asm/uaccess.h>
#include <asm/system_misc.h>
#include <asm/deprecated.h>
#include <linux/debugfs.h>

static int deprecated_handler(struct pt_regs *, unsigned int);

static struct undef_hook setend_hook = {
	.instr_mask	= 0xfffffdff,
	.instr_val	= 0xf1010000,
	.pstate_mask	= COMPAT_PSR_MODE_MASK | COMPAT_PSR_T_BIT,
	.pstate_val	= COMPAT_PSR_MODE_USR,
	.fn		= deprecated_handler,
};

static struct undef_hook setend_thumb_hook = {
	.instr_mask	= 0xfff7,
	.instr_val	= 0xb650,
	.pstate_mask	= COMPAT_PSR_MODE_MASK | COMPAT_PSR_T_BIT,
	.pstate_val	= COMPAT_PSR_MODE_USR | COMPAT_PSR_T_BIT,
	.fn		= deprecated_handler,
};

static struct undef_hook cp15_barrier_hook = {
	.instr_mask	= 0x0fff0fd5,
	.instr_val	= 0x0e070f90,
	.pstate_mask	= COMPAT_PSR_MODE_MASK,
	.pstate_val	= COMPAT_PSR_MODE_USR,
	.fn		= deprecated_handler,
};

static int deprecated_handler(struct pt_regs *regs, unsigned int instr)
{
	int sctlr_el1;

	if ((instr & setend_hook.instr_mask) == setend_hook.instr_val ||
		(instr & setend_thumb_hook.instr_mask) ==
		setend_thumb_hook.instr_val) {
		pr_notice("Warning: %s: PID %d: Using deprecated SETEND instruction\n",
			current->comm, current->pid);
		task_thread_info(current)->deprecated_flags &= ~SETEND_DISABLE;
	} else if ((instr & cp15_barrier_hook.instr_mask) ==
		cp15_barrier_hook.instr_val &&
		(((instr & 0xff) == 0x95) ||
		((instr & 0xff) == 0x9a) ||
		((instr & 0xff) == 0xba))) {
		pr_notice("Warning: %s: PID %d: Using deprecated CP15 barrier instruction\n",
			current->comm, current->pid);
		task_thread_info(current)->deprecated_flags |= CP15_BARRIER_ENABLE;
	} else
		return 1;

	asm volatile("mrs %0, sctlr_el1" : "=r" (sctlr_el1));
	sctlr_el1 = (sctlr_el1 & ~DEPRECATED_TRAP_MASK) |
		task_thread_info(current)->deprecated_flags;
	asm volatile("msr sctlr_el1, %0" : : "r" (sctlr_el1));

	return 0;
}


static int __init deprecated_init(void)
{
	register_undef_hook(&setend_hook);
	register_undef_hook(&cp15_barrier_hook);
	return 0;
}

late_initcall(deprecated_init);
