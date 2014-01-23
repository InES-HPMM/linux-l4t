/*
 *  Based on arch/arm/kernel/smp.c
 *
 *  Original Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *  Copyright (C) 2013 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/completion.h>

extern volatile ulong secondary_holding_pen_release;

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void tegra_cpu_die(unsigned int cpu)
{
	static const unsigned long pmstate = 2;

	do {
		/* Enter C6  and wait for secondary_holding_pen_release */
		asm volatile(
		"	msr actlr_el1, %0\n"
		"	wfi\n"
		:
		: "r" (pmstate));
	} while (secondary_holding_pen_release != cpu);
}
