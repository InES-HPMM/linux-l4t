#ifndef __ASM_DEPRECATED_H
#define __ASM_DEPRECATED_H

/* sctlr_el1 bits to enable deprecated armv7 instructions */
#define SETEND_DISABLE	0x00000100
#define CP15_BARRIER_ENABLE	0x00000020

#define DEPRECATED_TRAP_MASK	    (SETEND_DISABLE | CP15_BARRIER_ENABLE)

/* init with all deprecated instructions disabled */
#define INIT_DEPRECATED_FLAGS	SETEND_DISABLE

#endif
