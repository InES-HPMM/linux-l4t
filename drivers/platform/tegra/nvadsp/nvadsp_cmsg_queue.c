/*
 * nvadsp_cmsg_queue.c
 *
 * ADSP Circular Message Queue
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/tegra_nvadsp.h>

status_t nvadsp_cmsg_queue_init(int mq_key, nvadsp_cmsg_queue_t *q)
{
	return -ENOENT;
}

nvadsp_cmsg_queue_t *nvadsp_cmsg_queue_connect(int mq_key)
{
	return NULL;
}

status_t nvadsp_cmsg_queue_enqueue(nvadsp_cmsg_queue_t *q,
				   nvadsp_cmsg_queue_msg_t *m)
{
	return -ENOENT;
}
nvadsp_cmsg_queue_msg_t *nvadsp_cmsg_queue_dequeue(nvadsp_cmsg_queue_t *q)
{
	return NULL;
}

bool nvadsp_cmsg_queue_empty(nvadsp_cmsg_queue_t *q)
{
	return false;
}

bool nvadsp_cmsg_queue_full(nvadsp_cmsg_queue_t *q)
{
	return false;
}

status_t nvadsp_cmsg_queue_deinit(nvadsp_cmsg_queue_t *q)
{
	return -ENOENT;
}
