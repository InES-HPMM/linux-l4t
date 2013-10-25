/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysedp.h>
#include <linux/debugfs.h>
#include "sysedp_internal.h"

struct dentry *edp_debugfs_dir;
struct dentry *sysedp_debugfs_dir;

static int sysedp_status_show(struct seq_file *file, void *data)
{
	int consumer_sum = 0;
	struct sysedp_consumer *c;

	mutex_lock(&sysedp_lock);
	list_for_each_entry(c, &registered_consumers, link) {
		consumer_sum += _cur_level(c);
	}

	seq_printf(file, "  avail_budget : %u\n", avail_budget);
	seq_printf(file, "- consumer_sum : %u\n", consumer_sum);
	seq_printf(file, "- margin       : %d\n", margin);
	seq_printf(file, "= remaining    : %d\n", ((int)avail_budget -
						   consumer_sum - margin));

	seq_puts(file, "------------------------------------------\n");
	seq_printf(file, "%-16s %7s\n",	"consumer", "current");
	seq_puts(file, "------------------------------------------\n");

	list_for_each_entry(c, &registered_consumers, link)
		seq_printf(file, "%-16s %7u\n", c->name, _cur_level(c));

	mutex_unlock(&sysedp_lock);
	return 0;
}

static int sysedp_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, sysedp_status_show, inode->i_private);
}

static const struct file_operations sysedp_status_fops = {
	.open = sysedp_status_open,
	.read = seq_read,
};

void sysedp_init_debugfs(void)
{
	struct dentry *d;

	d = debugfs_create_dir("edp", NULL);
	if (IS_ERR_OR_NULL(d)) {
		WARN_ON(1);
		return;
	}
	edp_debugfs_dir = d;

	d = debugfs_create_dir("sysedp", edp_debugfs_dir);
	if (IS_ERR_OR_NULL(d)) {
		WARN_ON(1);
		return;
	}
	sysedp_debugfs_dir = d;

	d = debugfs_create_file("status", S_IRUGO, sysedp_debugfs_dir, NULL,
				&sysedp_status_fops);
	WARN_ON(IS_ERR_OR_NULL(d));
}
