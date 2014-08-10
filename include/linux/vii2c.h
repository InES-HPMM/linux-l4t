/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_VII2C_H
#define __LINUX_VII2C_H

struct nvhost_vii2c_struct {
	struct platform_device *pdev;
	struct platform_device *nvhost_dev;
	void *ctx;
	void *channel;
	void *job;
	void *completed_waiter;
	void __iomem *base;
	int syncval;
	/* functions */
	int (*open)(struct nvhost_vii2c_struct *p_vii2c);
	int (*close)(struct nvhost_vii2c_struct *p_vii2c);
	int (*module_start)(struct nvhost_vii2c_struct *p_vii2c);
	int (*syncpt_inc)(struct nvhost_vii2c_struct *p_vii2c, int num);
	int (*syncpt_wait)(struct nvhost_vii2c_struct *p_vii2c);
	int (*module_idle)(struct nvhost_vii2c_struct *p_vii2c);
	void (*write_reg)(u32 reg, u32 val);
	u32 (*read_reg)(u32 reg);
};

struct nvhost_vii2c_struct *nvhost_vii2c_open(struct platform_device *pdev);
void nvhost_vii2c_close(struct nvhost_vii2c_struct *p_vii2c);

#endif
