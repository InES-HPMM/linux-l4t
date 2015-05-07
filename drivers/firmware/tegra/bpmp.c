/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <soc/tegra/tegra_bpmp.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp.h"

#define BPMP_MODULE_MAGIC		0x646f6d
#define SHARED_SIZE			512

struct device *device;
static void *shared_virt;
static uint32_t shared_phys;
static DEFINE_SPINLOCK(shared_lock);

#ifdef CONFIG_DEBUG_FS
static struct dentry *bpmp_root;
static struct dentry *module_root;
static char firmware_tag[32];
static LIST_HEAD(modules);
DEFINE_MUTEX(bpmp_lock);

struct bpmp_module {
	struct list_head entry;
	char name[MODULE_NAME_LEN];
	struct dentry *root;
	u32 handle;
	u32 size;
};

struct module_hdr {
	u32 magic;
	u32 size;
	u32 reloc_size;
	u32 bss_size;
	u32 init_offset;
	u32 cleanup_offset;
	u8 reserved[72];
	u8 parent_tag[32];
};

int bpmp_create_attrs(const struct fops_entry *fent,
		struct dentry *parent, void *data)
{
	struct dentry *d;

	while (fent->name) {
		d = debugfs_create_file(fent->name, fent->mode, parent, data,
				fent->fops);
		if (IS_ERR_OR_NULL(d))
			return -EFAULT;
		fent++;
	}

	return 0;
}

static struct bpmp_module *bpmp_find_module(const char *name)
{
	struct bpmp_module *m;

	list_for_each_entry(m, &modules, entry) {
		if (!strncmp(m->name, name, MODULE_NAME_LEN))
			return m;
	}

	return NULL;
}

static int bpmp_module_load(struct device *dev, const void *base, u32 size,
		u32 *handle)
{
	void *virt;
	dma_addr_t phys;
	struct { u32 phys; u32 size; } __packed msg;
	int r;

	virt = dma_alloc_coherent(dev, size, &phys, GFP_KERNEL);
	if (virt == NULL)
		return -ENOMEM;

	memcpy(virt, base, size);

	msg.phys = phys;
	msg.size = size;

	r = tegra_bpmp_send_receive(MRQ_MODULE_LOAD, &msg, sizeof(msg),
			handle, sizeof(*handle));

	dma_free_coherent(dev, size, virt, phys);
	return r;
}

static int bpmp_module_unload(struct device *dev, u32 handle)
{
	return tegra_bpmp_send_receive(MRQ_MODULE_UNLOAD,
			&handle, sizeof(handle), NULL, 0);
}

static ssize_t bpmp_module_unload_store(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct bpmp_module *m;
	char buf[MODULE_NAME_LEN];
	char *name;
	int r;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (strncpy_from_user(buf, user_buf, count) <= 0)
		return -EFAULT;

	buf[count] = 0;
	name = strim(buf);

	mutex_lock(&bpmp_lock);

	m = bpmp_find_module(name);
	if (!m) {
		r = -ENODEV;
		goto clean;
	}

	r = bpmp_module_unload(device, m->handle);
	if (r) {
		dev_err(device, "%s: failed to unload module (%d)\n", name, r);
		goto clean;
	}

	debugfs_remove_recursive(m->root);
	list_del(&m->entry);
	kfree(m);

clean:
	mutex_unlock(&bpmp_lock);
	return r ?: count;
}

static const struct file_operations bpmp_module_unload_fops = {
	.write = bpmp_module_unload_store
};

static int bpmp_module_ready(const char *name, const struct firmware *fw,
		struct bpmp_module *m)
{
	struct module_hdr *hdr;
	const int sz = sizeof(firmware_tag);
	char fmt[sz + 1];
	int err;

	hdr = (struct module_hdr *)fw->data;

	if (fw->size < sizeof(struct module_hdr) ||
			hdr->magic != BPMP_MODULE_MAGIC ||
			hdr->size + hdr->reloc_size != fw->size) {
		dev_err(device, "%s: invalid module format\n", name);
		return -EINVAL;
	}

	if (memcmp(hdr->parent_tag, firmware_tag, sz) &&
			!IS_ENABLED(CONFIG_ARCH_TEGRA_18x_SOC)) {
		dev_err(device, "%s: bad module - tag mismatch\n", name);
		memcpy(fmt, firmware_tag, sz);
		fmt[sz] = 0;
		dev_err(device, "firmware: %s\n", fmt);
		memcpy(fmt, hdr->parent_tag, sz);
		fmt[sz] = 0;
		dev_err(device, "%s : %s\n", name, fmt);
		return -EINVAL;
	}

	m->size = hdr->size + hdr->bss_size;

	err = bpmp_module_load(device, fw->data, fw->size, &m->handle);
	if (err) {
		dev_err(device, "failed to load module, code=%d\n", err);
		return err;
	}

	if (!debugfs_create_x32("handle", S_IRUGO, m->root, &m->handle))
		return -ENOMEM;

	if (!debugfs_create_x32("size", S_IRUGO, m->root, &m->size))
		return -ENOMEM;

	list_add_tail(&m->entry, &modules);

	return 0;
}

static ssize_t bpmp_module_load_store(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	const struct firmware *fw;
	struct bpmp_module *m;
	char buf[MODULE_NAME_LEN];
	int r;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (strncpy_from_user(buf, user_buf, count) <= 0)
		return -EFAULT;

	buf[count] = 0;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (m == NULL)
		return -ENOMEM;

	mutex_lock(&bpmp_lock);

	strcpy(m->name, strim(buf));
	if (bpmp_find_module(m->name)) {
		dev_err(device, "module %s exist\n", m->name);
		r = -EEXIST;
		goto clean;
	}

	m->root = debugfs_create_dir(m->name, module_root);
	if (!m->root) {
		r = -ENOMEM;
		goto clean;
	}

	r = request_firmware(&fw, m->name, device);
	if (r) {
		dev_err(device, "request_firmware() failed: %d\n", r);
		goto clean;
	}

	if (!fw) {
		r = -EFAULT;
		WARN_ON(0);
		goto clean;
	}

	dev_info(device, "%s: module ready %zu@%p\n",
			m->name, fw->size, fw->data);
	r = bpmp_module_ready(m->name, fw, m);
	release_firmware(fw);
	uncache_firmware(m->name);

clean:
	mutex_unlock(&bpmp_lock);

	if (r) {
		debugfs_remove_recursive(m->root);
		kfree(m);
		return r;
	}

	return count;
}

static const struct file_operations bpmp_module_load_fops = {
	.write = bpmp_module_load_store
};

void bpmp_cleanup_modules(void)
{
	debugfs_remove_recursive(module_root);

	while (!list_empty(&modules)) {
		struct bpmp_module *m = list_first_entry(&modules,
				struct bpmp_module,
				entry);
		list_del(&m->entry);
		kfree(m);
	}
}

int bpmp_init_modules(struct platform_device *pdev)
{
	struct dentry *d;

	module_root = debugfs_create_dir("module", bpmp_root);
	if (IS_ERR_OR_NULL(module_root))
		goto clean;

	d = debugfs_create_file("load", S_IWUSR, module_root, pdev,
			&bpmp_module_load_fops);
	if (IS_ERR_OR_NULL(d))
		goto clean;

	d = debugfs_create_file("unload", S_IWUSR, module_root, pdev,
			&bpmp_module_unload_fops);
	if (IS_ERR_OR_NULL(d))
		goto clean;

	return 0;

clean:
	WARN_ON(1);
	debugfs_remove_recursive(module_root);
	module_root = NULL;
	return -EFAULT;
}

static int bpmp_cpuidle_name_show(struct seq_file *file, void *data)
{
	struct bpmp_cpuidle_state *state = file->private;
	seq_printf(file, "%s\n", state->name);
	return 0;
}

static int bpmp_cpuidle_name_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_cpuidle_name_show, inode->i_private);
}

static const struct file_operations cpuidle_name_fops = {
	.open = bpmp_cpuidle_name_open,
	.read = seq_read,
	.release = single_release
};

static int __bpmp_cpuidle_show(struct seq_file *file, bool show_time)
{
	struct bpmp_cpuidle_state *state = file->private;
	struct { uint64_t usage; uint64_t time; } __packed mb[3];
	uint32_t id = state->id;
	int ret;

	ret = tegra_bpmp_send_receive(MRQ_CPUIDLE_USAGE, &id, sizeof(id),
			&mb, sizeof(mb));
	if (ret) {
		seq_printf(file, "%d\n", ret);
		return ret;
	}

	if (show_time) {
		seq_printf(file, "%llu (%llu, %llu)\n",
				mb[0].time, mb[1].time, mb[2].time);
	} else {
		seq_printf(file, "%llu (%llu, %llu)\n",
				mb[0].usage, mb[1].usage, mb[2].usage);
	}

	return 0;
}

static int bpmp_cpuidle_usage_show(struct seq_file *file, void *data)
{
	return __bpmp_cpuidle_show(file, false);
}

static int bpmp_cpuidle_usage_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_cpuidle_usage_show, inode->i_private);
}

static const struct file_operations cpuidle_usage_fops = {
	.open = bpmp_cpuidle_usage_open,
	.read = seq_read,
	.release = single_release
};

static int bpmp_cpuidle_time_show(struct seq_file *file, void *data)
{
	return __bpmp_cpuidle_show(file, true);
}

static int bpmp_cpuidle_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_cpuidle_time_show, inode->i_private);
}

static const struct file_operations cpuidle_time_fops = {
	.open = bpmp_cpuidle_time_open,
	.read = seq_read,
	.release = single_release
};

static const struct fops_entry cpuidle_attrs[] = {
	{ "name", &cpuidle_name_fops, S_IRUGO },
	{ "usage", &cpuidle_usage_fops, S_IRUGO },
	{ "time", &cpuidle_time_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static int bpmp_create_cpuidle_debug(int index, struct dentry *parent,
		struct bpmp_cpuidle_state *state)
{
	struct dentry *top;
	char name[16];

	sprintf(name, "state%d", index);
	top = debugfs_create_dir(name, parent);
	if (IS_ERR_OR_NULL(top))
		return -EFAULT;

	return bpmp_create_attrs(cpuidle_attrs, top, state);
}

static int bpmp_init_cpuidle_debug(struct dentry *root)
{
	struct bpmp_cpuidle_state *state;
	struct dentry *d;
	unsigned int i;

	d = debugfs_create_dir("cpuidle", root);
	if (IS_ERR_OR_NULL(d))
		return -EFAULT;

	for (i = 0, state = plat_cpuidle_state; state->name; i++, state++) {
		if (bpmp_create_cpuidle_debug(i, d, state))
			return -EFAULT;
	}

	return 0;
}

int bpmp_get_fwtag(void)
{
	unsigned long flags;
	int r;

	if (IS_ENABLED(CONFIG_ARCH_TEGRA_18x_SOC))
		return 0;

	spin_lock_irqsave(&shared_lock, flags);
	r = tegra_bpmp_send_receive_atomic(MRQ_QUERY_TAG,
			&shared_phys, sizeof(shared_phys), NULL, 0);
	if (!r)
		memcpy(firmware_tag, shared_virt, sizeof(firmware_tag));
	spin_unlock_irqrestore(&shared_lock, flags);

	return r;
}

static int bpmp_ping_show(void *data, u64 *val)
{
	unsigned long flags;
	ktime_t tm;
	int ret;
	int challenge = 1;
	int reply;

	local_irq_save(flags);
	tm = ktime_get();
	ret = tegra_bpmp_send_receive_atomic(MRQ_PING,
			&challenge, sizeof(challenge), &reply, sizeof(reply));
	tm = ktime_sub(ktime_get(), tm);
	local_irq_restore(flags);

	*val = ret ?: ktime_to_us(tm);
	return 0;
}

static int bpmp_modify_trace_mask(uint32_t clr, uint32_t set)
{
	uint32_t mb[] = { clr, set };
	uint32_t new;
	return tegra_bpmp_send_receive(MRQ_TRACE_MODIFY, mb, sizeof(mb),
			&new, sizeof(new)) ?: new;
}

static int bpmp_trace_enable_show(void *data, u64 *val)
{
	*val = bpmp_modify_trace_mask(0, 0);
	return 0;
}

static int bpmp_trace_enable_store(void *data, u64 val)
{
	return bpmp_modify_trace_mask(0, val);
}

static int bpmp_trace_disable_store(void *data, u64 val)
{
	return bpmp_modify_trace_mask(val, 0);
}

static int bpmp_mount_show(void *data, u64 *val)
{
	*val = bpmp_fwdebug_init(bpmp_root);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bpmp_ping_fops, bpmp_ping_show, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(trace_enable_fops, bpmp_trace_enable_show,
		bpmp_trace_enable_store, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(trace_disable_fops, NULL,
		bpmp_trace_disable_store, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(bpmp_mount_fops, bpmp_mount_show, NULL, "%lld\n");

static int bpmp_copy_trace(struct seq_file *file, dma_addr_t phys, void *virt,
		int size)
{
	uint32_t mb[] = { phys, size };
	int ret;
	int eof = 0;

	while (!eof) {
		ret = tegra_bpmp_send_receive(MRQ_WRITE_TRACE, mb, sizeof(mb),
				&eof, sizeof(eof));
		if (ret < 0)
			return ret;
		seq_write(file, virt, ret);
	}

	return 0;
}

static int bpmp_trace_show(struct seq_file *file, void *data)
{
	dma_addr_t phys;
	void *virt;
	int size = SZ_16K;
	int ret;

	virt = dma_alloc_coherent(device, size, &phys, GFP_KERNEL);
	if (!virt)
		return -ENOMEM;

	ret = bpmp_copy_trace(file, phys, virt, size);
	dma_free_coherent(device, size, virt, phys);
	return ret;
}

static int bpmp_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_trace_show, inode->i_private);
}

static const struct file_operations trace_fops = {
	.open = bpmp_trace_open,
	.read = seq_read,
	.release = single_release
};

static int bpmp_tag_show(struct seq_file *file, void *data)
{
	seq_write(file, firmware_tag, sizeof(firmware_tag));
	seq_putc(file, '\n');
	return 0;
}

static int bpmp_tag_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_tag_show, inode->i_private);
}

static const struct file_operations bpmp_tag_fops = {
	.open = bpmp_tag_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

#define MSG_NR_FIELDS	((MSG_DATA_SZ + 3) / 4)
#define MSG_DATA_COUNT	(MSG_NR_FIELDS + 1)

static uint32_t inbox_data[MSG_DATA_COUNT];

static ssize_t bpmp_mrq_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	/* size in dec, space, new line, terminator */
	char buf[MSG_DATA_COUNT * 11 + 1 + 1];
	uint32_t outbox_data[MSG_DATA_COUNT];
	char *line;
	char *p;
	int i;
	int ret;

	memset(outbox_data, 0, sizeof(outbox_data));
	memset(inbox_data, 0, sizeof(inbox_data));

	count = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, count)) {
		ret = -EFAULT;
		goto complete;
	}

	buf[count] = 0;
	line = strim(buf);


	for (i = 0; i < MSG_DATA_COUNT && line; i++) {
		p = strsep(&line, " ");
		ret = kstrtouint(p, 0, outbox_data + i);
		if (ret)
			break;
	}

	if (!i) {
		ret = -EINVAL;
		goto complete;
	}

	ret = tegra_bpmp_send_receive(outbox_data[0], outbox_data + 1,
			MSG_DATA_SZ, inbox_data + 1, MSG_DATA_SZ);

complete:
	inbox_data[0] = ret;
	return ret ?: count;
}

static int bpmp_mrq_show(struct seq_file *file, void *data)
{
	int i;
	for (i = 0; i < MSG_DATA_COUNT; i++) {
		seq_printf(file, "0x%x%s", inbox_data[i],
				i == MSG_DATA_COUNT - 1 ? "\n" : " ");
	}

	return 0;
}

static int bpmp_mrq_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_mrq_show, inode->i_private);
}

static const struct file_operations bpmp_mrq_fops = {
	.open = bpmp_mrq_open,
	.llseek = seq_lseek,
	.read = seq_read,
	.write = bpmp_mrq_write,
	.release = single_release
};

static const struct fops_entry root_attrs[] = {
	{ "ping", &bpmp_ping_fops, S_IRUGO },
	{ "trace_enable", &trace_enable_fops, S_IRUGO | S_IWUSR },
	{ "trace_disable", &trace_disable_fops, S_IWUSR },
	{ "trace", &trace_fops, S_IRUGO },
	{ "tag", &bpmp_tag_fops, S_IRUGO },
	{ "mrq", &bpmp_mrq_fops, S_IRUGO | S_IWUSR },
	{ "mount", &bpmp_mount_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static int bpmp_init_debug(struct platform_device *pdev)
{
	struct dentry *root;

	root = debugfs_create_dir("bpmp", NULL);
	if (IS_ERR_OR_NULL(root))
		goto clean;

	if (bpmp_create_attrs(root_attrs, root, pdev))
		goto clean;

	if (bpmp_init_cpuidle_debug(root))
		goto clean;

	if (bpmp_platdbg_init(root, pdev))
		goto clean;

	bpmp_root = root;
	return 0;

clean:
	WARN_ON(1);
	debugfs_remove_recursive(root);
	return -EFAULT;
}
#else
static inline int bpmp_init_debug(struct platform_device *pdev) { return 0; }
int bpmp_init_modules(struct platform_device *pdev) { return 0; }
int bpmp_get_fwtag(void) { return 0; }
#endif

void tegra_bpmp_trace_printk(void)
{
	uint32_t mb[] = { shared_phys, SHARED_SIZE };
	unsigned long flags;
	int eof = 0;

	spin_lock_irqsave(&shared_lock, flags);

	while (!eof) {
		if (tegra_bpmp_send_receive_atomic(MRQ_WRITE_TRACE,
				mb, sizeof(mb), &eof, sizeof(eof)))
			goto done;
		pr_info("%s", (char *)shared_virt);
	}

done:
	spin_unlock_irqrestore(&shared_lock, flags);
}

static int bpmp_mem_init(void)
{
	dma_addr_t phys;

	shared_virt = dma_alloc_coherent(device, SHARED_SIZE, &phys,
			GFP_KERNEL);
	if (!shared_virt)
		return -ENOMEM;

	shared_phys = phys;
	return 0;
}

static int bpmp_probe(struct platform_device *pdev)
{
	int r = 0;

	device = &pdev->dev;

	r = bpmp_linear_map_init();
	r = r ?: bpmp_mem_init();
	r = r ?: bpmp_clk_init(pdev);
	r = r ?: bpmp_init_debug(pdev);
	r = r ?: bpmp_init_modules(pdev);
	r = r ?: bpmp_mail_init();
	r = r ?: bpmp_get_fwtag();
	r = r ?: of_platform_populate(device->of_node, NULL, NULL, device);

	return r;
}

static const struct of_device_id bpmp_of_matches[] = {
	{ .compatible = "nvidia,tegra186-bpmp" },
	{ .compatible = "nvidia,tegra210-bpmp" },
	{}
};

static struct platform_driver bpmp_driver = {
	.probe = bpmp_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bpmp",
		.of_match_table = of_match_ptr(bpmp_of_matches)
	}
};

static __init int bpmp_init(void)
{
	return platform_driver_register(&bpmp_driver);
}
core_initcall(bpmp_init);
