#undef TRACE_SYSTEM
#define TRACE_SYSTEM sys_calls

#if !defined(_TRACE_SYS_CALL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYS_CALL_H

#include <linux/tracepoint.h>

TRACE_EVENT(syscall_enter,

	TP_PROTO(char *name),

	TP_ARGS(name),

	TP_STRUCT__entry(
		__field( char *, name)
	),

	TP_fast_assign(
		__entry->name = name;
	),

	TP_printk("%s", __entry->name)
);

TRACE_EVENT(syscall_exit,

	TP_PROTO(char *name),

	TP_ARGS(name),

	TP_STRUCT__entry(
		__field( char *, name)
	),

	TP_fast_assign(
		__entry->name = name;
	),

	TP_printk("%s", __entry->name)
);


#endif /*  _TRACE_SYSCALL_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
