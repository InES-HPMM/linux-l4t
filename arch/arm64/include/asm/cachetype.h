/*
 * Copyright (C) 2012 ARM Ltd.
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
#ifndef __ASM_CACHETYPE_H
#define __ASM_CACHETYPE_H

#include <asm/cputype.h>

#define CTR_L1IP_SHIFT		14
#define CTR_L1IP_MASK		3

#define CLIDR_LOUIS_SHIFT	21
#define CLIDR_LOUIS_MASK	7

#define ICACHE_POLICY_RESERVED	0
#define ICACHE_POLICY_AIVIVT	1
#define ICACHE_POLICY_VIPT	2
#define ICACHE_POLICY_PIPT	3

static inline u32 icache_policy(void)
{
	return (read_cpuid_cachetype() >> CTR_L1IP_SHIFT) & CTR_L1IP_MASK;
}

/*
 * Whilst the D-side always behaves as PIPT on AArch64, aliasing is
 * permitted in the I-cache.
 */
static inline int icache_is_aliasing(void)
{
	return icache_policy() != ICACHE_POLICY_PIPT;
}

static inline int icache_is_aivivt(void)
{
	return icache_policy() == ICACHE_POLICY_AIVIVT;
}

/*
* If the LoUIS field value is 0x0, this means that no levels of
* cache need to cleaned or invalidated when cleaning or
* invalidating to the point of unification for the Inner
* Shareable shareability domain.
*/
static inline int need_user_flush_range(void)
{
	static int read, louis;

	if (!read) {
		read = 1;
		asm volatile ("mrs %0, CLIDR_EL1" : "=r" (louis));
		louis = (louis >> CLIDR_LOUIS_SHIFT) & CLIDR_LOUIS_MASK;
	}

	return louis;
}

#endif	/* __ASM_CACHETYPE_H */
