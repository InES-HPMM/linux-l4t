#include <asm/page.h>
#include <linux/types.h>


void clear_page(void *to) {
	u64 *buffer = (u64 *) to;
	u64 zero = 0ULL;
	u32 i;

	for (i = 0; i < PAGE_SIZE / sizeof(u64); i += 2)
		asm volatile("stnp  %0, %0, %1" : : "r" (zero), "Q" (buffer[i]) : );
}
