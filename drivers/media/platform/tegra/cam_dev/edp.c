/*
 * edp.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.

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

#define CAMERA_DEVICE_INTERNAL

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/sysedp.h>

#include <media/nvc.h>
#include <media/camera.h>

void camera_edp_register(struct camera_device *cdev)
{
}

int camera_edp_req(struct camera_device *cdev, unsigned new_state)
{
	dev_dbg(cdev->dev, "%s %d\n", __func__, new_state);

	return 0;
}

void camera_edp_lowest(
	struct camera_device *cdev
)
{
}
