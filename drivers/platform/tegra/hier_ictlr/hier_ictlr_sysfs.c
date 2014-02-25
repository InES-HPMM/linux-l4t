/*
 * Copyright (c) 2014-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include "hier_ictlr.h"

static ssize_t mselect_timeout_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct tegra_hier_ictlr *ictlr = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", ictlr->mselect_timeout_cycles);
}

static ssize_t mselect_timeout_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tegra_hier_ictlr *ictlr = dev_get_drvdata(device);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	tegra_hier_ictlr_set_mselect_timeout(ictlr, val);

	return count;
}

static DEVICE_ATTR(mselect_timeout, S_IWUSR | S_IRUGO, mselect_timeout_show,
			mselect_timeout_store);

void tegra_hier_ictlr_create_sysfs(struct platform_device *pdev)
{
	int error;

	error = device_create_file(&pdev->dev, &dev_attr_mselect_timeout);

	if (error)
		dev_err(&pdev->dev, "Failed to create sysfs attributes!\n");
}

void tegra_hier_ictlr_remove_sysfs(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_mselect_timeout);
}

