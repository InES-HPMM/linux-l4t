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

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/aio.h>
#include "logger.h"
#include <linux/pstore_ram.h>
#include "pstore_logger.h"
#include <asm/ioctls.h>

static char pri_to_char(int pri)
{
	switch (pri) {
	case ANDROID_LOG_VERBOSE:       return 'V';
	case ANDROID_LOG_DEBUG:         return 'D';
	case ANDROID_LOG_INFO:          return 'I';
	case ANDROID_LOG_WARN:          return 'W';
	case ANDROID_LOG_ERROR:         return 'E';
	case ANDROID_LOG_FATAL:         return 'F';
	case ANDROID_LOG_SILENT:        return 'S';
	case ANDROID_LOG_DEFAULT:
	case ANDROID_LOG_UNKNOWN:
	default:                        return '?';
	}
}

static inline size_t pstore_log_offset(size_t n)
{
	return n & (PSTORE_LOG_BUFFER_SIZE - 1);
}

int __init create_pstore_log(struct logger_log *log)
{
	int ret = 0;

	log->pstore_log = kzalloc(sizeof(struct pstore_logger), GFP_KERNEL);
	if (log->pstore_log == NULL)
		return -ENOMEM;

	log->pstore_log->buffer = vmalloc(PSTORE_LOG_BUFFER_SIZE);
	if (log->pstore_log->buffer == NULL) {
		ret = -ENOMEM;
		goto out_free_pstore_log;
	}

	log->pstore_log->msg_buf = vmalloc(MSGBUF_SIZE);
	if (log->pstore_log->msg_buf == NULL) {
		ret = -ENOMEM;
		goto out_free_log_buf;
	}

	return 0;

out_free_log_buf:
	vfree(log->pstore_log->buffer);
out_free_pstore_log:
	kfree(log->pstore_log);

	return ret;
}

void clean_pstore_log(struct logger_log *log)
{
	struct pstore_logger *pstore_log = log->pstore_log;

	memset(pstore_log->buffer, 0, PSTORE_LOG_BUFFER_SIZE);
	memset(pstore_log->msg_buf, 0, MSGBUF_SIZE);
	pstore_log->nr_msg = 0;
	pstore_log->offset = 0;
}

int do_write_pstore_log(struct logger_log *log, const void __user *buf,
		size_t count)
{
	int ret = 0;
	size_t len;
	struct pstore_logger *pstore_log;
	int nr_msg;

	pstore_log = log->pstore_log;
	if (!pstore_log)
		goto exit;

	nr_msg = pstore_log->nr_msg;
	if (nr_msg >= PSTORE_LOG_PAYLOAD)
		goto exit;

	len = min(count, PSTORE_LOG_BUFFER_SIZE - pstore_log->offset);

	if (len && copy_from_user(pstore_log->buffer +
			pstore_log->offset, buf, len)) {
		clean_pstore_log(log);
		return -EFAULT;
	}

	pstore_log->iovec[nr_msg].base = pstore_log->buffer +
		pstore_log->offset;
	pstore_log->iovec[nr_msg].len = len;
	pstore_log->nr_msg++;
	pstore_log->offset = pstore_log_offset(pstore_log->offset + len);

exit:
	return ret;
}

void do_write_pstore_log_ram(struct logger_log *log,
		struct logger_entry *header)
{
	struct pstore_logger *pstore_log;
	size_t ram_len, line_len;
	unsigned char *p, *pm;
	struct io_vec msg;
	unsigned char *msg_buf;
	int pri;

	pstore_log = log->pstore_log;
	if (!pstore_log)
		return;

	msg = pstore_log->iovec[PSTORE_LOG_MSG];
	msg_buf = pstore_log->msg_buf;
	pri = *(pstore_log->iovec[PSTORE_LOG_PRIO].base);

	if (pri <= ANDROID_LOG_UNKNOWN || pri > ANDROID_LOG_SILENT)
		goto exit;

	p = pm = msg.base;

	while (pm < (msg.base + msg.len)) {
		ram_len = line_len = 0;

#if defined(PSTORE_LOG_TIMESTAMP)
		u64 ts = local_clock();
		unsigned long rem_nsec;

		rem_nsec = do_div(ts, 1000000000);
		ram_len = snprintf(msg_buf, MSGBUF_SIZE,
					"[%5lu.%06lu] ", (unsigned long)ts,
					rem_nsec / 1000);
#endif
		ram_len += snprintf(msg_buf + ram_len,
					MSGBUF_SIZE - ram_len,
					"%c/%s(%5d): ", pri_to_char(pri),
					pstore_log->iovec[PSTORE_LOG_TAG].base,
					header->pid);

		while (*pm != '\n' && pm < (msg.base + msg.len))
			pm++;

		line_len = (pm - p >= MSGBUF_SIZE - ram_len - 1) ?
					(MSGBUF_SIZE - ram_len - 1) : (pm - p);

		/*
		 * Remove the null character
		 */
		if (pm == msg.base + msg.len)
			line_len--;

		if (line_len <= 0)
			goto exit;

		strncpy(msg_buf + ram_len, p, line_len);
		ram_len += line_len;
		p += line_len;

		strcpy(msg_buf + ram_len, "\n");
		ram_len++;

		if (*p == '\n')
			p++;

		pstore_dbg("%s", msg_buf);
		ramoops_pmsg_write_buf(msg_buf, ram_len);

		pm = p;
		memset(msg_buf, 0, MSGBUF_SIZE);
	}

exit:
	clean_pstore_log(log);
}

