#undef TRACE_SYSTEM
#define TRACE_SYSTEM lowmemorykiller

#if !defined(_TRACE_LOWMEMKILLER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LOWMEMKILLER_H

#include <linux/tracepoint.h>

TRACE_EVENT(lowmem_utilization,

	TP_PROTO(char *s),

	TP_ARGS(s),

	TP_STRUCT__entry(
		__field(char *, s)
	),

	TP_fast_assign(
		__entry->s = s;
	),

	TP_printk("%s", __entry->s)
);

DECLARE_EVENT_CLASS(lowmem_task_info,

	TP_PROTO(char *name, int pid, short oom_adj, long size),

	TP_ARGS(name, pid, oom_adj, size),

	TP_STRUCT__entry(
		__field(char *, name)
		__field(int, pid)
		__field(short, oom_adj)
		__field(long, size)
	),

	TP_fast_assign(
		__entry->name = name;
		__entry->pid = pid;
		__entry->oom_adj = oom_adj;
		__entry->size = size;
	),

	TP_printk("Task: %s, pid: %d, adj: %hd, size: %ld kB",
		  __entry->name, __entry->pid, __entry->oom_adj,
		  __entry->size * (long)(PAGE_SIZE / 1024))
);

DEFINE_EVENT(lowmem_task_info, lowmem_task_list,

	TP_PROTO(char *name, int pid, short oom_adj, long size),

	TP_ARGS(name, pid, oom_adj, size)
);

DEFINE_EVENT(lowmem_task_info, lowmem_task_selected,

	TP_PROTO(char *name, int pid, short oom_adj, long size),

	TP_ARGS(name, pid, oom_adj, size)
);

#endif /* _TRACE_LOWMEMKILLER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
