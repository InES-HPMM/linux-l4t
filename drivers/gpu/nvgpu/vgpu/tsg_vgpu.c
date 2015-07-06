/*
 * Virtualized GPU TSG (Time Slice Group)
 *
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
 */

#include "vgpu/vgpu.h"

static int vgpu_tsg_open(struct gk20a *g, struct file *filp)
{
	gk20a_warn(dev_from_gk20a(g), "TSG is not supported");

	return -EIO;
}

static void vgpu_tsg_release(struct kref *ref)
{
	gk20a_dbg(gpu_dbg_fn, "TSG is not supported");
}

void vgpu_init_tsg_ops(struct gpu_ops *gops)
{
	gops->tsg.open = vgpu_tsg_open;
	gops->tsg.release = vgpu_tsg_release;
}
