/*
 * nvadsp_mailbox.c
 *
 * ADSP mailbox
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

nvadsp_mbmsg_t *nvadsp_mbox_get_msg(void)
{
	return NULL;
}

void nvadsp_mbox_put_msg(nvadsp_mbmsg_t *msg)
{
	return;
}

nvadsp_mbox_t *nvadsp_mbox_init(uint16_t serviceid, uint16_t mbid)
{
	return NULL;
}

status_t nvadsp_mbox_connect(nvadsp_mbox_t *mb, uint16_t dst_serviceid,
			     uint16_t dst_mbid)
{
	return -ENOENT;
}
status_t nvadsp_mbox_send(nvadsp_mbox_t *mb, nvadsp_mbmsg_t *msg)
{
	return -ENOENT;
}
nvadsp_mbmsg_t *nvadsp_mbox_recv(nvadsp_mbox_t *mb)
{
	return NULL;
}

status_t nvadsp_mbox_destroy(nvadsp_mbox_t *mb)
{
	return -ENOENT;
}
