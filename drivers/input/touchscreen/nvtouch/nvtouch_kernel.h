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
 */
/*
 * nvtouch_kernel.h
 *
 *  Created on: Apr 7, 2014
 *      Author: kartamonov
 */

#ifndef NVTOUCH_KERNEL_H_
#define NVTOUCH_KERNEL_H_

#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/debugfs.h>

#include <linux/regulator/consumer.h> /* regulator & voltage */
#include <linux/spi/spi.h>
#include <linux/wakelock.h> /* wakelock */

#include "nvtouch.h"

void	nvtouch_kernel_init(
		int sensor_w, int sensor_h,
		int screen_w, int screen_h,
		int invert_x, int invert_y,
		struct input_dev *input_device);

void	nvtouch_kernel_process(void *samples, int samples_size, u8 report_mode);
long nvtouch_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

u64	nvtouch_kernel_get_time_us(void);

char	*nvtouch_kernel_get_version_string(void);

struct nvtouch_events *nvtouch_kernel_get_vendor_events_ptr(void);

void	nvtouch_kernel_timestamp_touch_irq(void);

void	nvtouch_handle_work(struct work_struct *work);
void	nvtouch_set_userspace_pid(unsigned long pid);
int	nvtouch_is_enabled(void);

#endif /* NVTOUCH_KERNEL_H_ */
