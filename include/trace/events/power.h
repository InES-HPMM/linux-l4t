#undef TRACE_SYSTEM
#define TRACE_SYSTEM power

#if !defined(_TRACE_POWER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_POWER_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>
#include <linux/ftrace_event.h>

#define TPS(x)  tracepoint_string(x)

DECLARE_EVENT_CLASS(cpu,

	TP_PROTO(unsigned int state, unsigned int cpu_id),

	TP_ARGS(state, cpu_id),

	TP_STRUCT__entry(
		__field(	u32,		state		)
		__field(	u32,		cpu_id		)
	),

	TP_fast_assign(
		__entry->state = state;
		__entry->cpu_id = cpu_id;
	),

	TP_printk("state=%lu cpu_id=%lu", (unsigned long)__entry->state,
		  (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(cpu, cpu_idle,

	TP_PROTO(unsigned int state, unsigned int cpu_id),

	TP_ARGS(state, cpu_id)
);

/* This file can get included multiple times, TRACE_HEADER_MULTI_READ at top */
#ifndef _PWR_EVENT_AVOID_DOUBLE_DEFINING
#define _PWR_EVENT_AVOID_DOUBLE_DEFINING

#define PWR_EVENT_EXIT -1

enum {
	CPU_SUSPEND_START,
	CPU_SUSPEND_DONE
};

enum {
	POWER_CPU_UP_START,
	POWER_CPU_UP_DONE,
	POWER_CPU_DOWN_START,
	POWER_CPU_DOWN_DONE,
};

enum {
	POWER_CPU_SCALE_START,
	POWER_CPU_SCALE_DONE,
};

#endif

TRACE_EVENT(cpu_suspend,

	TP_PROTO(unsigned int state, unsigned int ts),

	TP_ARGS(state, ts),

	TP_STRUCT__entry(
		__field(u32, state)
		__field(u32, ts)
	),

	TP_fast_assign(
		__entry->state = state;
		__entry->ts = ts;
	),

	TP_printk("state %u, time %u", __entry->state, __entry->ts)
);

TRACE_EVENT(cpu_hotplug,

	TP_PROTO(unsigned int cpu_id, int state),

	TP_ARGS(cpu_id, state),

	TP_STRUCT__entry(
		__field(u32, cpu_id)
		__field(u32, state)
	),

	TP_fast_assign(
		__entry->cpu_id = cpu_id;
		__entry->state = state;
	),

	TP_printk("cpu_id=%lu, state=%lu",
		  (unsigned long)__entry->cpu_id,
		  (unsigned long)__entry->state)
);

TRACE_EVENT(cpu_scale,

	TP_PROTO(unsigned int cpu_id, unsigned int freq, int state),

	TP_ARGS(cpu_id, freq, state),

	TP_STRUCT__entry(
		__field(u64, cpu_id)
		__field(u32, freq)
		__field(u32, state)
	),

	TP_fast_assign(
		__entry->cpu_id = cpu_id;
		__entry->freq = freq;
		__entry->state = state;
	),

	TP_printk("cpu_id=%lu, freq=%lu, state=%lu",
		  (unsigned long)__entry->cpu_id,
		  (unsigned long)__entry->freq,
		  (unsigned long)__entry->state)
);

TRACE_EVENT(pm_qos_request,

	TP_PROTO(u32 class, s32 value, u32 priority, u64 request),

	TP_ARGS(class, value, priority, request),

	TP_STRUCT__entry(
		__field(u32, class)
		__field(s32, value)
		__field(u32, priority)
		__field(u64, request)
	),

	TP_fast_assign(
		__entry->class = class;
		__entry->value = value;
		__entry->priority = priority;
		__entry->request = request;
	),

	TP_printk("class=%lu, value=%d, prio=%lu, request=0x%lx",
		  (unsigned long)__entry->class,
		  (int)__entry->value,
		  (unsigned long)__entry->priority,
		  (unsigned long)__entry->request)
);

#define pm_verb_symbolic(event) \
	__print_symbolic(event, \
		{ PM_EVENT_SUSPEND, "suspend" }, \
		{ PM_EVENT_RESUME, "resume" }, \
		{ PM_EVENT_FREEZE, "freeze" }, \
		{ PM_EVENT_QUIESCE, "quiesce" }, \
		{ PM_EVENT_HIBERNATE, "hibernate" }, \
		{ PM_EVENT_THAW, "thaw" }, \
		{ PM_EVENT_RESTORE, "restore" }, \
		{ PM_EVENT_RECOVER, "recover" })

DEFINE_EVENT(cpu, cpu_frequency,

	TP_PROTO(unsigned int frequency, unsigned int cpu_id),

	TP_ARGS(frequency, cpu_id)
);

TRACE_EVENT(device_pm_callback_start,

	TP_PROTO(struct device *dev, const char *pm_ops, int event_in),

	TP_ARGS(dev, pm_ops, event_in),

	TP_STRUCT__entry(
		__string(device, dev_name(dev))
		__string(driver, dev_driver_string(dev))
		__string(parent, dev->parent ? dev_name(dev->parent) : "none")
		__string(pm_ops, pm_ops ? pm_ops : "none ")
		__field(int, event)
	),

	TP_fast_assign(
		__assign_str(device, dev_name(dev));
		__assign_str(driver, dev_driver_string(dev));
		__assign_str(parent, dev->parent ? dev_name(dev->parent) : "none");
		__assign_str(pm_ops, pm_ops ? pm_ops : "none ");
		__entry->event = event_in;
	),

	TP_printk("%s %s, parent: %s, %s[%s]", __get_str(driver),
		__get_str(device), __get_str(parent), __get_str(pm_ops),
		pm_verb_symbolic(__entry->event))
);

TRACE_EVENT(device_pm_callback_end,

	TP_PROTO(struct device *dev, int error_in),

	TP_ARGS(dev, error_in),

	TP_STRUCT__entry(
		__string(device, dev_name(dev))
		__string(driver, dev_driver_string(dev))
		__field(int, error)
	),

	TP_fast_assign(
		__assign_str(device, dev_name(dev));
		__assign_str(driver, dev_driver_string(dev));
		__entry->error = error_in;
	),

	TP_printk("%s %s, err=%d",
		__get_str(driver), __get_str(device), __entry->error)
);

TRACE_EVENT(suspend_resume,

	TP_PROTO(const char *action, int val, bool start),

	TP_ARGS(action, val, start),

	TP_STRUCT__entry(
		__field(const char *, action)
		__field(int, val)
		__field(bool, start)
	),

	TP_fast_assign(
		__entry->action = action;
		__entry->val = val;
		__entry->start = start;
	),

	TP_printk("%s[%u] %s", __entry->action, (unsigned int)__entry->val,
		(__entry->start)?"begin":"end")
);

DECLARE_EVENT_CLASS(wakeup_source,

	TP_PROTO(const char *name, unsigned int state),

	TP_ARGS(name, state),

	TP_STRUCT__entry(
		__string(       name,           name            )
		__field(        u64,            state           )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->state = state;
	),

	TP_printk("%s state=0x%lx", __get_str(name),
		(unsigned long)__entry->state)
);

DEFINE_EVENT(wakeup_source, wakeup_source_activate,

	TP_PROTO(const char *name, unsigned int state),

	TP_ARGS(name, state)
);

DEFINE_EVENT(wakeup_source, wakeup_source_deactivate,

	TP_PROTO(const char *name, unsigned int state),

	TP_ARGS(name, state)
);

/*
 * The clock events are used for clock enable/disable and for
 *  clock rate change
 */
DECLARE_EVENT_CLASS(clock,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id),

	TP_STRUCT__entry(
		__string(       name,           name            )
		__field(        u64,            state           )
		__field(        u64,            cpu_id          )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->state = state;
		__entry->cpu_id = cpu_id;
	),

	TP_printk("%s state=%lu cpu_id=%lu", __get_str(name),
		(unsigned long)__entry->state, (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(clock, clock_enable,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

DEFINE_EVENT(clock, clock_disable,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

DEFINE_EVENT(clock, clock_set_rate,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

TRACE_EVENT(clock_set_parent,

	TP_PROTO(const char *name, const char *parent_name),

	TP_ARGS(name, parent_name),

	TP_STRUCT__entry(
		__string(       name,           name            )
		__string(       parent_name,    parent_name     )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__assign_str(parent_name, parent_name);
	),

	TP_printk("%s parent=%s", __get_str(name), __get_str(parent_name))
);

DEFINE_EVENT(clock, clock_set_start,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

DEFINE_EVENT(clock, clock_set_done,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

/*
 * The power domain events are used for power domains transitions
 */
DECLARE_EVENT_CLASS(power_domain,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id),

	TP_STRUCT__entry(
		__string(       name,           name            )
		__field(        u64,            state           )
		__field(        u64,            cpu_id          )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->state = state;
		__entry->cpu_id = cpu_id;
),

	TP_printk("%s state=%lu cpu_id=%lu", __get_str(name),
		(unsigned long)__entry->state, (unsigned long)__entry->cpu_id)
);

DEFINE_EVENT(power_domain, power_domain_target,

	TP_PROTO(const char *name, unsigned int state, unsigned int cpu_id),

	TP_ARGS(name, state, cpu_id)
);

TRACE_EVENT(powergate,

	TP_PROTO(const char *func, const char *name, int id, bool start, int ret),

	TP_ARGS(func, name, id, start, ret),

	TP_STRUCT__entry(
		__field(const char *, func);
		__field(const char *, name)
		__field(int, id)
		__field(bool, start)
		__field(int, ret)
	),

	TP_fast_assign(
		__entry->func = func;
		__entry->name = name;
		__entry->id = id;
		__entry->start = start;
		__entry->ret = ret;
	),

	TP_printk("%s called for %s with id = %u at %s ret = %u\n",
		 __entry->func, __entry->name, (unsigned int)__entry->id,
		(__entry->start)?"ENTRY":"EXIT", (unsigned int)__entry->ret)
);

#endif /* _TRACE_POWER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
