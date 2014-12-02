/*
 * drivers/net/wireless/bcmdhd/dhd_custom_sysfs_tegra.c
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

static DEVICE_ATTR(ping, S_IRUGO | S_IWUGO,
	tegra_sysfs_histogram_ping_show,
	tegra_sysfs_histogram_ping_store);

static DEVICE_ATTR(rssi, S_IRUGO | S_IWUGO,
	tegra_sysfs_histogram_rssi_show,
	tegra_sysfs_histogram_rssi_store);

static DEVICE_ATTR(scan, S_IRUGO | S_IWUGO,
	tegra_sysfs_histogram_scan_show,
	tegra_sysfs_histogram_scan_store);

static DEVICE_ATTR(stat, S_IRUGO | S_IWUGO,
	tegra_sysfs_histogram_stat_show,
	tegra_sysfs_histogram_stat_store);

static DEVICE_ATTR(tcpdump, S_IRUGO | S_IWUGO,
	tegra_sysfs_histogram_tcpdump_show,
	tegra_sysfs_histogram_tcpdump_store);

static struct attribute *tegra_sysfs_entries_histogram[] = {
	&dev_attr_ping.attr,
	&dev_attr_rssi.attr,
	&dev_attr_scan.attr,
	&dev_attr_stat.attr,
	&dev_attr_tcpdump.attr,
	NULL,
};

static struct attribute_group tegra_sysfs_group_histogram = {
	.name = "histogram",
	.attrs = tegra_sysfs_entries_histogram,
};

static struct dentry *tegra_debugfs_root;

static struct file_operations tegra_debugfs_histogram_tcpdump_fops = {
	.read = tegra_debugfs_histogram_tcpdump_read,
	.write = tegra_debugfs_histogram_tcpdump_write,
};

int
tegra_sysfs_register(struct device *dev)
{
	int err;

	pr_info("%s\n", __func__);

	/* create sysfs */
	err = sysfs_create_group(&dev->kobj, &tegra_sysfs_group_histogram);
	if (err) {
		pr_err("%s: failed to create tegra sysfs group %s\n",
			__func__, tegra_sysfs_group_histogram.name);
		return err;
	}

	/* create debugfs */
	tegra_debugfs_root = debugfs_create_dir("bcmdhd_histogram", NULL);
	if (tegra_debugfs_root) {
		debugfs_create_file("tcpdump", S_IRUGO | S_IWUGO,
			tegra_debugfs_root, (void *) 0,
			&tegra_debugfs_histogram_tcpdump_fops);
	}

	/* start sysfs work */
#if 0
	tegra_sysfs_histogram_ping_work_start();
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_scan_work_start();
	tegra_sysfs_histogram_stat_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();
#endif

	return 0;
}

void
tegra_sysfs_unregister(struct device *dev)
{
	pr_info("%s\n", __func__);

	/* stop sysfs work */
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_stat_work_stop();
	tegra_sysfs_histogram_scan_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();
	tegra_sysfs_histogram_ping_work_stop();

	/* remove debugfs */
	if (tegra_debugfs_root) {
		debugfs_remove_recursive(tegra_debugfs_root);
		tegra_debugfs_root = NULL;
	}

	/* remove sysfs */
	sysfs_remove_group(&dev->kobj, &tegra_sysfs_group_histogram);

}

void
tegra_sysfs_suspend(void)
{
	pr_info("%s\n", __func__);

	/* suspend (stop) sysfs work */
	tegra_sysfs_histogram_tcpdump_work_stop();
	tegra_sysfs_histogram_stat_work_stop();
	tegra_sysfs_histogram_scan_work_stop();
	tegra_sysfs_histogram_rssi_work_stop();
	tegra_sysfs_histogram_ping_work_stop();

}

void
tegra_sysfs_resume(void)
{
	pr_info("%s\n", __func__);

	/* resume (start) sysfs work */
	tegra_sysfs_histogram_ping_work_start();
	tegra_sysfs_histogram_rssi_work_start();
	tegra_sysfs_histogram_scan_work_start();
	tegra_sysfs_histogram_stat_work_start();
	tegra_sysfs_histogram_tcpdump_work_start();

}
