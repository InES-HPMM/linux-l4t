/* FIXME:
 * This is the !CONFIG_CPA part of the CPA implementation in
 * arch/arm/mm/pageattr.c, which exists only in NVIDIA trees.
 * CPA is used by nvmap and we need to know if CPA is still
 * needed or can be dropped.
 */

#include <linux/highmem.h>
#include <linux/bootmem.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/pfn.h>
#include <linux/percpu.h>
#include <linux/gfp.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>

#include <asm/processor.h>
#include <asm/tlbflush.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/uaccess.h>
#include <asm/pgalloc.h>

#include "mm.h"

void update_page_count(int level, unsigned long pages)
{
}

static void flush_cache(struct page **pages, int numpages)
{
	unsigned int i;
	bool flush_inner = true;

#if defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS)
	if (numpages >= FLUSH_CLEAN_BY_SET_WAY_PAGE_THRESHOLD) {
		inner_flush_cache_all();
		flush_inner = false;
	}
#endif

	for (i = 0; i < numpages; i++) {
		if (flush_inner)
			__flush_dcache_page(pages[i]);
	}
}

int set_pages_array_uc(struct page **pages, int addrinarray)
{
	flush_cache(pages, addrinarray);
	return 0;
}
EXPORT_SYMBOL(set_pages_array_uc);

int set_pages_array_wc(struct page **pages, int addrinarray)
{
	flush_cache(pages, addrinarray);
	return 0;
}
EXPORT_SYMBOL(set_pages_array_wc);

int set_pages_array_wb(struct page **pages, int addrinarray)
{
	return 0;
}
EXPORT_SYMBOL(set_pages_array_wb);

int set_pages_array_iwb(struct page **pages, int addrinarray)
{
	flush_cache(pages, addrinarray);
	return 0;
}
EXPORT_SYMBOL(set_pages_array_iwb);
