/*
 * drivers/net/wireless/bcmdhd/dhd_custom_sysfs_tegra_scan.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
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

#include "dhd_custom_sysfs_tegra.h"

void
tegra_sysfs_histogram_scan_work_start(void)
{
//	pr_info("%s\n", __func__);

	/* placeholder for scan work */

}

void
tegra_sysfs_histogram_scan_work_stop(void)
{
//	pr_info("%s\n", __func__);

	/* placeholder for scan work */

}

ssize_t
tegra_sysfs_histogram_scan_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	static int i;

//	pr_info("%s\n", __func__);

	if (!i) {
		i++;
		strcpy(buf, "dummy scan!");
		return strlen(buf);
	} else {
		i = 0;
		return 0;
	}
}

ssize_t
tegra_sysfs_histogram_scan_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
//	pr_info("%s\n", __func__);

	return count;
}
