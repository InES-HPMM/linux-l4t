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

TRACE_EVENT(lowmem_oom_threshold,

	TP_PROTO(short oom_adj),

	TP_ARGS(oom_adj),

	TP_STRUCT__entry(
		__field(short, oom_adj)
	),

	TP_fast_assign(
		__entry->oom_adj = oom_adj;
	),

	TP_printk("oom_score_adj threshold: %hd",
		 __entry->oom_adj)
);

DECLARE_EVENT_CLASS(lowmem_task_info,

	TP_PROTO(char *name, int pid, short oom_adj, long size),

	TP_ARGS(name, pid, oom_adj, size),

	TP_STRUCT__entry(
		__string(msg, name)
		__field(int, pid)
		__field(short, oom_adj)
		__field(long, size)
	),

	TP_fast_assign(
		__assign_str(msg, name);
		__entry->pid = pid;
		__entry->oom_adj = oom_adj;
		__entry->size = size;
	),

	TP_printk("Task: %s, pid: %d, adj: %hd, size: %ld kB",
		  __get_str(msg), __entry->pid, __entry->oom_adj,
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

DEFINE_EVENT(lowmem_task_info, lowmem_task_wait_timeout,

	TP_PROTO(char *name, int pid, short oom_adj, long size),

	TP_ARGS(name, pid, oom_adj, size)
);

#endif /* _TRACE_LOWMEMKILLER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
