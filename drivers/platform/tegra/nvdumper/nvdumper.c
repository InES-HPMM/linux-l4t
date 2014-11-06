/*
 * arch/arm/mach-tegra/nvdumper.c
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../../../arch/arm/mach-tegra/board.h"
#include "nvdumper.h"
#include "nvdumper-footprint.h"

#ifdef CONFIG_TEGRA_USE_NCT
#include "../../../arch/arm/mach-tegra/include/mach/nct.h"
#endif

static void __init nvdumper_debugfs_init(void);
static void __exit nvdumper_debugfs_exit(void);

#define NVDUMPER_CLEAN      0xf000caf3U
#define NVDUMPER_DIRTY      0x2badfaceU
#define NVDUMPER_DIRTY_DUMP 0xdeadbeefU

#define RW_MODE (S_IWUSR | S_IRUGO)

static void __iomem *nvdumper_ptr;
static uint32_t nvdumper_last_reboot;

static uint32_t get_dirty_state(void)
{
	return ioread32(nvdumper_ptr);
}

static void set_dirty_state(uint32_t state)
{
	pr_info("nvdumper: set_dirty_state 0x%x\n", state);
	iowrite32(state, nvdumper_ptr);
}

static int nvdumper_reboot_cb(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	pr_info("nvdumper: rebooting cleanly.\n");
	set_dirty_state(NVDUMPER_CLEAN);
	return NOTIFY_DONE;
}

static struct notifier_block nvdumper_reboot_notifier = {
	.notifier_call = nvdumper_reboot_cb,
};

static int __init nvdumper_init(void)
{
	int ret;

#ifdef CONFIG_TEGRA_USE_NCT
	union nct_item_type *item;
#endif

	if (!nvdumper_reserved) {
		pr_info("nvdumper: not configured\n");
		return -ENOTSUPP;
	}
	nvdumper_ptr = ioremap_nocache(nvdumper_reserved,
			NVDUMPER_RESERVED_SIZE);
	if (!nvdumper_ptr) {
		pr_info("nvdumper: failed to ioremap memory at 0x%08lx\n",
				nvdumper_reserved);
		return -EIO;
	}
	ret = register_reboot_notifier(&nvdumper_reboot_notifier);
	if (ret)
		goto err_out1;

	ret = nvdumper_regdump_init();
	if (ret)
		goto err_out2;

	nvdumper_dbg_footprint_init();

	nvdumper_last_reboot = get_dirty_state();
	switch (nvdumper_last_reboot) {
	case NVDUMPER_CLEAN:
		pr_info("nvdumper: last reboot was clean\n");
		break;
	case NVDUMPER_DIRTY:
	case NVDUMPER_DIRTY_DUMP:
		pr_info("nvdumper: last reboot was dirty\n");
		break;
	default:
		pr_info("nvdumper: last reboot was unknown\n");
		break;
	}

	nvdumper_debugfs_init();

#ifdef CONFIG_TEGRA_USE_NCT
	item = kzalloc(sizeof(*item), GFP_KERNEL);
	if (!item) {
		pr_err("failed to allocate memory\n");
		goto err_out3;
	}

	ret = tegra_nct_read_item(NCT_ID_RAMDUMP, item);
	if (ret < 0) {
		pr_err("%s: NCT read failure\n", __func__);
		kfree(item);
		set_dirty_state(NVDUMPER_CLEAN);
		goto err_out0;
	}

	pr_info("%s: RAMDUMP flag(%d) from NCT\n",
			__func__, item->ramdump.flag);
	if (item->ramdump.flag == 1)
		set_dirty_state(NVDUMPER_DIRTY_DUMP);
	else if (item->ramdump.flag == 2)
		set_dirty_state(NVDUMPER_DIRTY);
	else
		set_dirty_state(NVDUMPER_CLEAN);

	kfree(item);

	return 0;

err_out3:

#else
	set_dirty_state(NVDUMPER_DIRTY);
	return 0;
#endif

err_out2:
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
err_out1:
	iounmap(nvdumper_ptr);

#ifdef CONFIG_TEGRA_USE_NCT /* avoid build error if NCT is not enabled*/
err_out0:
#endif

	return ret;

}

static void __exit nvdumper_exit(void)
{
	nvdumper_debugfs_exit();
	nvdumper_regdump_exit();
	nvdumper_dbg_footprint_exit();
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
	set_dirty_state(NVDUMPER_CLEAN);
	iounmap(nvdumper_ptr);
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *nvdumper_dbg_dentry;
static struct dentry *nvdumper_set_dbg_dentry;
static char *nvdumper_set_str = "dirty_dump";

static int nvdumper_reboot_state_show(struct seq_file *s, void *data)
{
	seq_puts(s, nvdumper_set_str);
	seq_puts(s, "\n");
	return 0;
}

static int nvdumper_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvdumper_reboot_state_show, inode->i_private);
}

static const struct file_operations nvdumper_dbg_fops = {
	.open		= nvdumper_dbg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int nvdumper_set_state_show(struct seq_file *s, void *data)
{
	seq_puts(s, nvdumper_set_str);
	seq_puts(s, "\n");
	return 0;
}

static int nvdumper_set_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvdumper_set_state_show, inode->i_private);
}

static ssize_t nvdumper_write_set(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	size_t count_copy;

	if (count < 1)
		return 0;

	count_copy = copy_from_user(nvdumper_set_str, buf, count);
	if (count_copy != 0)
		return 0;

	nvdumper_set_str[count-1] = '\0';

	if (!strcmp(nvdumper_set_str, "clean"))
		set_dirty_state(NVDUMPER_CLEAN);
	else if (!strcmp(nvdumper_set_str, "dirty"))
		set_dirty_state(NVDUMPER_DIRTY);
	else if (!strcmp(nvdumper_set_str, "dirty_dump"))
		set_dirty_state(NVDUMPER_DIRTY_DUMP);
	else
		strcpy(nvdumper_set_str, "unknown");

	pr_err("nvdumper_set was updated to %s\n", nvdumper_set_str);

	return count;
}

static const struct file_operations nvdumper_set_dbg_fops = {
	.open		= nvdumper_set_dbg_open,
	.read		= seq_read,
	.write		= nvdumper_write_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void __init nvdumper_debugfs_init(void)
{
	nvdumper_dbg_dentry = debugfs_create_file("nvdumper_prev", S_IRUGO,
			NULL, NULL, &nvdumper_dbg_fops);

	if (!nvdumper_dbg_dentry)
		return;

	switch (nvdumper_last_reboot) {
	case NVDUMPER_CLEAN:
		nvdumper_set_str = "clean\n";
		break;
	case NVDUMPER_DIRTY:
		nvdumper_set_str = "dirty\n";
		break;
	case NVDUMPER_DIRTY_DUMP:
		nvdumper_set_str = "dirty_dump\n";
		break;
	default:
		nvdumper_set_str = "dirty\n";
		break;
	}

	nvdumper_set_dbg_dentry = debugfs_create_file("nvdumper_set",
			S_IRUGO | S_IWUGO, NULL, NULL, &nvdumper_set_dbg_fops);
	if (!nvdumper_set_dbg_dentry)
		return;
}

static void __exit nvdumper_debugfs_exit(void)
{
	debugfs_remove(nvdumper_dbg_dentry);
	debugfs_remove(nvdumper_set_dbg_dentry);
}

#else

static void __init nvdumper_debugfs_init(void)
{
}

static void __exit nvdumper_debugfs_exit(void)
{
}

#endif

arch_initcall(nvdumper_init);
module_exit(nvdumper_exit);

MODULE_LICENSE("GPL");
