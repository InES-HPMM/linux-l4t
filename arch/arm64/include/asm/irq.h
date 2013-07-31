#ifndef __ASM_IRQ_H
#define __ASM_IRQ_H

/* FIXME: mach/irqs.h defines Tegra specific NR_IRQS
 * which should be picked up in favor of the one in
   asm-generic/irq.h, so we include mach/irqs.h here.
 */
#include <mach/irqs.h>
#include <asm-generic/irq.h>

#ifndef irq_canonicalize
#define irq_canonicalize(i)	(i)
#endif

extern void migrate_irqs(void);

extern void (*handle_arch_irq)(struct pt_regs *);
extern void set_handle_irq(void (*handle_irq)(struct pt_regs *));

#endif
