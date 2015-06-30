/*
 * linux/drivers/platform/tegra/tegra_bootloader_debug.c
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/debugfs.h>

static const char *module_name = "tegra_bootloader_debug";
static const char *dir_name = "tegra_bootloader";
static const char *gr_file = "tegra_bootloader_verify_regs";
static const char *prof_file = "tegra_bootloader_prof";

struct gr_address_value {
	unsigned int gr_address;
	unsigned int gr_value;
};

extern phys_addr_t tegra_bl_debug_data_start;
extern phys_addr_t tegra_bl_debug_data_size;
extern phys_addr_t tegra_bl_prof_start;

static int dbg_golden_register_show(struct seq_file *s, void *unused);
static int dbg_golden_register_open(struct inode *inode, struct file *file);
static int dbg_prof_show(struct seq_file *s, void *unused);
static int dbg_prof_open(struct inode *inode, struct file *file);
static struct dentry *bl_debug_node;
static struct dentry *bl_debug_verify_reg_node;
static struct dentry *bl_debug_verify_prof_node;

static const struct file_operations debug_gr_fops = {
	.open           = dbg_golden_register_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static const struct file_operations debug_prof_fops = {
	.open           = dbg_prof_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int dbg_golden_register_show(struct seq_file *s, void *unused)
{
	struct gr_address_value *gr_memory_dump;
	unsigned int gr_entries = 0;
	int i;

	gr_entries = tegra_bl_debug_data_size / sizeof(struct gr_address_value);

	gr_memory_dump = (struct gr_address_value *)
				phys_to_virt(tegra_bl_debug_data_start);

	for (i = 0; i < gr_entries; i++) {
		seq_printf(s, "{Address 0x%08x}, {Value 0x%08x}\n",
			gr_memory_dump->gr_address, gr_memory_dump->gr_value);

		gr_memory_dump++;
	}

	return 0;
}

static int dbg_prof_show(struct seq_file *s, void *unused)
{
	char *addr;

	addr = (char *)
		phys_to_virt(tegra_bl_prof_start);

	if (!tegra_bl_prof_start || !addr) {
		printk(KERN_INFO "No Bootloader profiling data in kernel commandline\n");
		return -EFAULT;
	}

	seq_printf(s, "%s\n", addr);
	return 0;
}

static int dbg_golden_register_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_golden_register_show, &inode->i_private);
}

static int dbg_prof_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_prof_show, &inode->i_private);
}

static int __init tegra_bootloader_golden_register_debuginit(void)
{

	bl_debug_verify_reg_node = debugfs_create_file(gr_file, S_IRUGO,
				bl_debug_node, NULL, &debug_gr_fops);

	if (!bl_debug_verify_reg_node) {
		printk(KERN_ERR "%s: failed to create debugfs entries: %ld\n",
			module_name, PTR_ERR(bl_debug_verify_reg_node));
		goto out_err;
	}

	printk(KERN_INFO "%s: Created sysfs interface %s in %s directory\n",
		module_name, gr_file, dir_name);

	return 0;

out_err:

	return -ENODEV;
}

static int __init tegra_bootloader_prof_debuginit(void)
{

	bl_debug_verify_prof_node = debugfs_create_file(prof_file, S_IRUGO,
				bl_debug_node, NULL, &debug_prof_fops);

	if (!bl_debug_verify_prof_node) {
		printk(KERN_ERR "%s: failed to create debugfs entries: %ld\n",
			module_name, PTR_ERR(bl_debug_verify_prof_node));
		goto out_err;
	}

	printk(KERN_INFO "%s: Created sysfs interface %s in %s directory\n",
		module_name, prof_file, dir_name);

	return 0;

out_err:

	return -ENODEV;
}

static int __init tegra_bl_debuginit_module_init(void)
{
	int err1 = 0, err2 = 0;
	bl_debug_node = debugfs_create_dir(dir_name, NULL);

	if (!bl_debug_node) {
		printk(KERN_ERR "%s: failed to create debugfs entries: %ld\n",
			module_name, PTR_ERR(bl_debug_node));
		goto out_err;
	}

	printk(KERN_INFO "%s: Created %s directory\n", module_name, dir_name);

	err1 = tegra_bootloader_golden_register_debuginit();
	err2 = tegra_bootloader_prof_debuginit();

	if ((err1 < 0) && (err2 < 0))
		goto out_err;

	return 0;

out_err:
	if (bl_debug_node)
		debugfs_remove_recursive(bl_debug_node);

	return -ENODEV;
}

static void __exit tegra_bl_debuginit_module_exit(void)
{
	printk(KERN_INFO "%s: Exiting\n", module_name);
	if (bl_debug_node)
		debugfs_remove_recursive(bl_debug_node);

	bl_debug_node = NULL;
	bl_debug_verify_reg_node = NULL;
}

module_init(tegra_bl_debuginit_module_init);
module_exit(tegra_bl_debuginit_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver to enumerate bootloader's debug data");
MODULE_AUTHOR("Mohit Dhingra <mdhingra@nvidia.com>");
