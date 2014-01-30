/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _BPMP_PRIVATE_H
#define _BPMP_PRIVATE_H

#include <linux/kernel.h>
#include <linux/platform_device.h>

int bpmp_ipc_init(struct platform_device *pdev);

/* should be called from non-preemptible context */
int bpmp_post(int mrq, void *data, int sz);

/* should be called from non-preemptible context */
int bpmp_rpc(int mrq, void *ob_data, int ob_sz, void *ib_data, int ib_sz);

/* should be called from sleepable context */
int bpmp_threaded_rpc(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz);

int bpmp_ping(void);

#endif
