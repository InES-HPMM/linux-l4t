/*
 * Copyright (C) ST-Ericsson SA 2011
 * Author: Maxime Coquelin <maxime.coquelin@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */
#ifndef _LINUX_PASR_H
#define _LINUX_PASR_H

#include <linux/mm.h>
#include <linux/spinlock.h>
#include <mach/memory.h>

#ifdef CONFIG_PASR

/**
 * struct pasr_section - Represent either a DDR Bank or Segment depending on
 * the DDR configuration (Bank-Row-Column or Row-Bank-Coloumn)
 *
 * @start: Start address of the segment.
 * @pair: Pointer on another segment in case of dependency (e.g. interleaving).
 *	Masking of the dependant segments have to be done accordingly.
 * @free_size: Represents the free memory size in the segment.
 * @lock: Protect the free_size counter
 * @die: Pointer to the Die the segment is part of.
 */
struct pasr_section {
	phys_addr_t start;
	struct pasr_section *pair;
	unsigned long free_size;
	spinlock_t *lock;
	struct pasr_die *die;
};

/**
 * struct pasr_die - Represent a DDR die
 *
 * @start: Start address of the die.
 * @idx: Index of the die.
 * @nr_sections: Number of Bank or Segment in the die.
 * @section: Table of the die's segments.
 * @mem_reg: Represents the PASR mask of the die. It is either MR16 or MR17,
 *	depending on the addressing configuration (RBC or BRC).
 * @apply_mask: Callback registred by the platform's PASR driver to apply the
 *	calculated PASR mask.
 * @cookie: Private data for the platform's PASR driver.
 */
struct pasr_die {
	phys_addr_t start;
	int idx;
	int nr_sections;
	struct pasr_section section[PASR_MAX_SECTION_NR_PER_DIE];
};

/**
 * struct pasr_map - Represent the DDR physical map
 *
 * @nr_dies: Number of DDR dies.
 * @die: Table of the dies.
 */
struct pasr_map {
	int nr_dies;
	struct pasr_die die[PASR_MAX_DIE_NR];
};

#define for_each_pasr_section(i, j, map, s)			\
	for (i = 0; i < map.nr_dies; i++)			\
		for (s = &map.die[i].section[0], j = 0;		\
			j < map.die[i].nr_sections;		\
			j++, s = &map.die[i].section[j])

#endif /* CONFIG_PASR */

#endif /* _LINUX_PASR_H */
