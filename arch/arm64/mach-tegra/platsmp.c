/* FIXME: temporary */
#include <linux/compiler.h>
#include "../../arm/mach-tegra/include/mach/io.h"

/* FIXME: smp_operations is n/a in arm64 */
struct smp_operations {
#ifdef CONFIG_SMP
	/*
	 * Setup the set of possible CPUs (via set_cpu_possible)
	 */
	void (*smp_init_cpus)(void);
	/*
	 * Initialize cpu_possible map, and enable coherency
	 */
	void (*smp_prepare_cpus)(unsigned int max_cpus);

	/*
	 * Perform platform specific initialisation of the specified CPU.
	 */
	void (*smp_secondary_init)(unsigned int cpu);
	/*
	 * Boot a secondary CPU, and assign it the specified idle task.
	 * This also gives us the initial stack to use for this CPU.
	 */
	int  (*smp_boot_secondary)(unsigned int cpu, struct task_struct *idle);
#ifdef CONFIG_HOTPLUG_CPU
	int  (*cpu_kill)(unsigned int cpu);
	void (*cpu_die)(unsigned int cpu);
	int  (*cpu_disable)(unsigned int cpu);
#endif
#endif
};

#include "../../arm/mach-tegra/platsmp.c"
