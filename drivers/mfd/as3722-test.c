/*
 * as3722-test.c - testing the AS3722 driver
 *
 * Copyright (C) 2013 ams AG
 *
 * Author: Florian Lobmaier <florian.lobmaier@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/err.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/as3722-reg.h>
#include <linux/mfd/as3722-plat.h>

struct dyn_file_data {
	struct regulator *reg;
	int is_open;
	int is_status_read;
};

struct regulator_list {
	struct list_head list;
	struct dentry *fs_entry;
	struct dyn_file_data *fdata;
};

static struct dentry *debug_dir;
static struct dentry *create_reg;

static struct regulator_list regulators;

static ssize_t dyn_regulator_read(struct file *file,
		char __user *user_buf,
		size_t size, loff_t *ppos)
{
	int i;
	char buf[100];
	ssize_t buf_size;
	struct dyn_file_data *fdata = file->private_data;
	struct regulator *reg = fdata->reg;

	int enabled = regulator_is_enabled(reg);
	int voltage = regulator_get_voltage(reg);
	int current_limit = regulator_get_current_limit(reg);
	int mode = regulator_get_mode(reg);

	if (fdata->is_status_read)
		return 0;

	buf_size = min(size, (sizeof(buf) - 1));

	i = sprintf(buf,
			"enabled: %d voltage: %duV current_limit: %d%s mode=%s\n",
			enabled, voltage, current_limit,
			current_limit < 1 ? "" : "uA",
			mode < 0 ? "<unknown>" : mode ==
			REGULATOR_MODE_FAST ? "fast" : "normal");

	if (copy_to_user(user_buf, buf, i))
		return -EFAULT;

	fdata->is_status_read = 1;

	return i;
}

static ssize_t dyn_regulator_write(struct file *file,
		const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret;
	char buf[80];
	char *pos;
	ssize_t buf_size;
	struct dyn_file_data *fdata = file->private_data;
	struct regulator *reg = fdata->reg;

	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	if (strcmp(buf, "enable") == 0) {

		/* calls to enable and disable needs to be balanced!
		   check if the regulator is already enabled */

		if (regulator_is_enabled(reg) > 0) {
			printk(KERN_ERR "%s: regulator enabled already\n",
					__func__);
			return buf_size;
		}

		ret = regulator_enable(reg);
		if (ret < 0) {
			printk(KERN_ERR "%s: enable %d\n", __func__, ret);
			return -EINVAL;
		}
		return buf_size;

	} else if (strcmp(buf, "disable") == 0) {

		if (regulator_is_enabled(reg) == 0) {
			printk(KERN_ERR "%s: regulator disabled already\n",
					__func__);
			return buf_size;
		}

		ret = regulator_disable(reg);
		if (ret < 0) {
			printk(KERN_ERR "%s: disable %d\n", __func__, ret);
			return -EINVAL;
		}
		return buf_size;

	} else if (strstr(buf, "set_voltage") != NULL) {

		/* we expect "set_voltage uV_min uV_max" */

		int uV_min = -1;
		int uV_max = -1;

		pos = strchr(buf, ' ');
		if (!pos) {
			printk(KERN_ERR "%s: set_voltage error parse uV_min\n",
					__func__);
			return -EINVAL;
		}
		pos++;
		uV_min = simple_strtol(pos, 0, 10);
		pos = strchr(pos, ' ');
		if (!pos) {
			printk(KERN_ERR "%s: set_voltage error parse uV_max\n",
					__func__);
			return -EINVAL;
		}
		pos++;
		uV_max = simple_strtol(pos, 0, 10);
		if (uV_min < 0 || uV_max < 0) {
			printk(KERN_ERR
					"%s: error voltage: uV_min=%d uV_max=%d\n",
					__func__, uV_min, uV_max);
			return -EINVAL;
		}
		ret = regulator_set_voltage(reg, uV_min, uV_max);
		if (ret < 0) {
			printk(KERN_ERR
					"%s: set_voltage: uV_min=%d uV_max=%d ret=%d\n",
					__func__, uV_min, uV_max, ret);
			return -EINVAL;
		}

		return buf_size;

	} else if (strstr(buf, "set_current") != NULL) {

		/* we expect "set_current uA_min uA_max" */

		int uA_min = -1;
		int uA_max = -1;

		pos = strchr(buf, ' ');
		if (!pos) {
			printk(KERN_ERR "%s: set_current error parse uA_min\n",
					__func__);
			return -EINVAL;
		}
		pos++;
		uA_min = simple_strtol(pos, 0, 10);
		pos = strchr(pos, ' ');
		if (!pos) {
			printk(KERN_ERR "%s: set_current error parse uA_max\n",
					__func__);
			return -EINVAL;
		}
		pos++;
		uA_max = simple_strtol(pos, 0, 10);
		if (uA_min < 0 || uA_max < 0) {
			printk(KERN_ERR
					"%s: error current: uA_min=%d uA_max=%d\n",
					__func__, uA_min, uA_max);
			return -EINVAL;
		}

		ret = regulator_set_current_limit(reg, uA_min, uA_max);
		if (ret < 0) {
			printk(KERN_ERR
					"%s: set_current: uA_min=%d uA_max=%d ret=%d\n",
					__func__, uA_min, uA_max, ret);
			return -EINVAL;
		}

		return buf_size;
	} else if (strstr(buf, "set_mode") != NULL) {

		int mode;

		pos = strchr(buf, ' ');
		if (!pos) {
			printk(KERN_ERR "%s: set_mode command parse\n",
					__func__);
			return -EINVAL;
		}
		pos++;
		if (strcmp(pos, "normal") == 0) {
			mode = REGULATOR_MODE_NORMAL;
		} else if (strcmp(pos, "fast") == 0) {
			mode = REGULATOR_MODE_FAST;
		} else {
			printk(KERN_ERR "%s: mode %s not supported!\n",
					__func__, pos);
			return -EINVAL;
		}

		ret = regulator_set_mode(reg, mode);
		if (ret < 0) {
			printk(KERN_ERR "%s: set_mode: %s ret=%d\n",
					__func__, pos, ret);
			return -EINVAL;
		}

		return buf_size;
	}

	printk(KERN_ERR "%s: command invalid: %s\n", __func__, buf);

	return buf_size;
}

static int dyn_regulator_open(struct inode *inode, struct file *file)
{
	struct dyn_file_data *fdata = inode->i_private;

	if (fdata->is_open)
		return -EBUSY;

	fdata->is_open = 1;
	fdata->is_status_read = 0;

	file->private_data = fdata;

	return 0;
}

static int dyn_regulator_release(struct inode *inode, struct file *file)
{
	struct dyn_file_data *fdata = file->private_data;

	fdata->is_open = 0;

	return 0;
}

static const struct file_operations dyn_regulator_fsops = {
	.open = dyn_regulator_open,
	.read = dyn_regulator_read,
	.write = dyn_regulator_write,
	.release = dyn_regulator_release,
};

static int create_regulator_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t create_regulator_write(struct file *file,
		const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret;
	char buf[32];
	ssize_t buf_size;
	struct regulator *reg;
	struct dyn_file_data *fdata;
	struct regulator_list *reg_list_entry;
	struct dentry *fs_entry;

	/* get userspace string and assure termination */
	buf_size = min(count, (sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	buf[buf_size] = 0;

	reg = regulator_get(NULL, buf);
	if (IS_ERR(reg)) {
		printk(KERN_ERR "%s: there is no supply voltage: %s\n",
				__func__, buf);
		return -EINVAL;
	}

	fdata = kmalloc(sizeof(struct dyn_file_data), GFP_KERNEL);
	if (!fdata) {
		printk(KERN_ERR "%s: cannot kmalloc fdata\n", __func__);
		return -ENOMEM;
	}

	fdata->reg = reg;
	fdata->is_open = 0;
	fdata->is_status_read = 0;

	fs_entry = debugfs_create_file(buf, (S_IWUGO | S_IRUGO), debug_dir,
			fdata, &dyn_regulator_fsops);
	if (!fs_entry) {
		printk(KERN_ERR "%s: cannot create fs_entry for %s\n",
				__func__, buf);
		ret = -EINVAL;
		goto exit_no_fs_entry;
	}

	reg_list_entry = kmalloc(sizeof(struct regulator_list), GFP_KERNEL);
	if (!reg_list_entry) {
		printk(KERN_ERR "%s: cannot kmalloc reg_list_entry for %s\n",
				__func__, buf);
		ret = -ENOMEM;
		goto exit_no_reg_list_entry;
	}

	reg_list_entry->fs_entry = fs_entry;

	/* we save the fdata for free(), how to get it from dentry? */
	reg_list_entry->fdata = fdata;

	list_add(&(reg_list_entry->list), &(regulators.list));

	return buf_size;

exit_no_reg_list_entry:
	debugfs_remove(fs_entry);
exit_no_fs_entry:
	kfree(fdata);
	return ret;
}

static const struct file_operations create_regulator_fsops = {
	.open = create_regulator_open,
	.write = create_regulator_write,
};

static int as3722_test_probe(void)
{
	INIT_LIST_HEAD(&regulators.list);

	debug_dir = debugfs_create_dir("as3722", NULL);
	if (!debug_dir) {
		printk(KERN_ERR "%s: cannot create debugfs\n", __func__);
		goto exit_no_debugfs;
	}

	create_reg = debugfs_create_file("create_regulator", S_IWUGO, debug_dir,
			NULL, &create_regulator_fsops);
	if (!create_reg) {
		printk(KERN_ERR "%s: cannot create create_reg\n", __func__);
		goto exit_no_create_reg;
	}

	return 0;

exit_no_create_reg:
	debugfs_remove(debug_dir);
	debug_dir = NULL;
exit_no_debugfs:
	return -EINVAL;
}

static int as3722_test_remove(void)
{
	struct list_head *pos, *q;
	struct regulator_list *reg_list_entry;
	struct dyn_file_data *fdata;

	list_for_each_safe(pos, q, &regulators.list) {
		reg_list_entry = list_entry(pos, struct regulator_list, list);
		list_del(pos);
		fdata = reg_list_entry->fdata;
		regulator_put(fdata->reg);
		debugfs_remove(reg_list_entry->fs_entry);
		kfree(fdata);
		kfree(reg_list_entry);
	}

	if (create_reg)
		debugfs_remove(create_reg);

	if (debug_dir)
		debugfs_remove(debug_dir);

	return 0;
}

static int __init as3722_test_init(void)
{
	as3722_test_probe();
	return 0;
}

static void __exit as3722_test_exit(void)
{
	as3722_test_remove();
}

module_init(as3722_test_init);
module_exit(as3722_test_exit);

MODULE_DESCRIPTION("AS3722 PMIC testing");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Lobmaier <florian.lobmaier@ams.com>");

