/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _PSTORE_LOGGER_H
#define _PSTORE_LOGGER_H

#include "logger.h"

#ifdef PSTORE_LOGGER_DEBUG
#define pstore_dbg(fmt, ...) printk(KERN_INFO"pstore_logger:"fmt, ##__VA_ARGS__)
#else
#define pstore_dbg(fmt, ...)
#endif

#define MSGBUF_SIZE 1024
#define PSTORE_LOG_BUFFER_SIZE	(3 * MSGBUF_SIZE)
#define PSTORE_LOG_TIMESTAMP 1
enum {
	ANDROID_LOG_UNKNOWN = 0,
	ANDROID_LOG_DEFAULT,    /* only for SetMinPriority() */
	ANDROID_LOG_VERBOSE,
	ANDROID_LOG_DEBUG,
	ANDROID_LOG_INFO,
	ANDROID_LOG_WARN,
	ANDROID_LOG_ERROR,
	ANDROID_LOG_FATAL,
	ANDROID_LOG_SILENT,     /* only for SetMinPriority(); must be last */
};

enum {
	PSTORE_LOG_PRIO = 0,
	PSTORE_LOG_TAG,
	PSTORE_LOG_MSG,
	PSTORE_LOG_PAYLOAD,
};

struct io_vec {
	unsigned char *base;
	size_t len;
};


struct pstore_logger {
	struct io_vec iovec[PSTORE_LOG_PAYLOAD];
	int	nr_msg;
	unsigned char *buffer;
	unsigned char *msg_buf;
	size_t offset;
};


extern int __init create_pstore_log(struct logger_log *log);
extern int do_write_pstore_log(struct logger_log *log, const void *buf,
				size_t count);
extern void do_write_pstore_log_ram(struct logger_log *log,
			struct logger_entry *header);
extern void clean_pstore_log(struct logger_log *log);
#endif /* _PSTORE_LOGGER_H */
