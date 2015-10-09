/*
 * Copyright (c) 2014, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM lowmemorykiller

#if !defined(_TRACE_LOWMEMORYKILLER) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LOWMEMORYKILLER

#include <linux/tracepoint.h>

TRACE_EVENT(lowmem_shrink,

	TP_PROTO(unsigned long lowmem_print_ts,
		long int other_free,
		long int other_file,
		unsigned long sigkill_diff,
		unsigned int task_count),

	TP_ARGS(lowmem_print_ts,
		other_free,
		other_file,
		sigkill_diff,
		task_count),

	TP_STRUCT__entry(
		__field(unsigned long, lowmem_print_ts)
		__field(long int, other_free)
		__field(long int, other_file)
		__field(unsigned long, sigkill_diff)
		__field(unsigned int, task_count)
	),

	TP_fast_assign(
		__entry->lowmem_print_ts = lowmem_print_ts;
		__entry->other_free = other_free;
		__entry->other_file = other_file;
		__entry->sigkill_diff = sigkill_diff;
		__entry->task_count = task_count;
	),

	TP_printk("LOWMEM_PRINT_ELAPSED_TS %u ms| " \
		"other_free %ldkB | other_file %ldkB | " \
		"SIGKILL %u ms | Task Count %u",
		jiffies_to_msecs(__entry->lowmem_print_ts),
		__entry->other_free, __entry->other_file,
		jiffies_to_msecs(__entry->sigkill_diff),
		__entry->task_count)

);
#endif /* _TRACE_TEGRA_LOWMEMORYKILLER */

#include <trace/define_trace.h>

