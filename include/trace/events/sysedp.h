/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM sysedp

#if !defined(_TRACE_SYSEDP_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYSEDP_H

#include <linux/string.h>
#include <linux/sysedp.h>
#include <linux/tracepoint.h>

TRACE_EVENT(sysedp_change_state,

	    TP_PROTO(const char *name, unsigned int old, unsigned int new),

	    TP_ARGS(name, old, new),

	    TP_STRUCT__entry(
		    __array(char,     name,       SYSEDP_NAME_LEN)
		    __field(unsigned int,       old)
		    __field(unsigned int,       new)
		    ),

	    TP_fast_assign(
		    memcpy(__entry->name, name, SYSEDP_NAME_LEN);
		    __entry->old = old;
		    __entry->new = new;
		    ),

	    TP_printk("%s: %u -> %u", __entry->name,
		      __entry->old, __entry->new)
	);

TRACE_EVENT(sysedp_set_avail_budget,

	    TP_PROTO(unsigned int old, unsigned int new),

	    TP_ARGS(old, new),

	    TP_STRUCT__entry(
		    __field(unsigned int,       old)
		    __field(unsigned int,       new)
		    ),

	    TP_fast_assign(
		    __entry->old = old;
		    __entry->new = new;
		    ),

	    TP_printk("%umW -> %umW", __entry->old, __entry->new)
	);

#endif /* _TRACE_SYSEDP_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
