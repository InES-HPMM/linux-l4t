/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_data/tegra_bpmp.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/clk.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp_private.h"

#define BPMP_FIRMWARE_NAME		"bpmp.bin"
#define BPMP_MODULE_MAGIC		0x646f6d

#define SHARED_SIZE			512

static struct clk *cop_clk;
static struct clk *sclk;
static struct clk *emc_clk;
static void *bpmp_virt;
static struct tegra_bpmp_platform_data *platform_data;
static struct device *device;
static struct mutex bpmp_lock;
static void *shared_virt;
static uint32_t shared_phys;
static DEFINE_SPINLOCK(shared_lock);

#ifdef CONFIG_DEBUG_FS
static struct dentry *bpmp_root;
static struct dentry *module_root;
static LIST_HEAD(modules);

struct bpmp_module {
	struct list_head entry;
	struct dentry *root;
	char name[20];
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
};

struct fops_entry {
	char *name;
	const struct file_operations *fops;
	mode_t mode;
};

static int bpmp_create_attrs(const struct fops_entry *fent,
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

static int bpmp_module_unload_show(void *data, u64 *val)
{
	struct bpmp_module *m = data;
	int err;

	mutex_lock(&bpmp_lock);
	err = bpmp_module_unload(device, m->handle);
	if (err)
		dev_err(device, "failed to unload module, code=%d\n", err);
	else
		debugfs_remove_recursive(m->root);
	mutex_unlock(&bpmp_lock);

	*val = err;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bpmp_module_unload_fops, bpmp_module_unload_show,
		NULL, "%lld\n");

static void bpmp_module_ready(const struct firmware *fw, void *context)
{
	struct bpmp_module *m = context;
	struct module_hdr *hdr;
	struct dentry *d;
	int err;

	if (!fw) {
		dev_info(device, "module not ready\n");
		kfree(m);
		return;
	}

	dev_info(device, "module %s ready: %zu@%p\n",
			m->name, fw->size, fw->data);

	hdr = (struct module_hdr *)fw->data;

	if (fw->size < sizeof(struct module_hdr) ||
			hdr->magic != BPMP_MODULE_MAGIC ||
			hdr->size + hdr->reloc_size != fw->size) {

		dev_err(device, "unknown or invalid module format\n");
		goto err_nolock;
	}

	m->size = hdr->size + hdr->bss_size;

	mutex_lock(&bpmp_lock);

	err = bpmp_module_load(device, fw->data, fw->size, &m->handle);
	if (err) {
		dev_err(device, "failed to load module, code=%d\n", err);
		goto err;
	}

	m->root = debugfs_create_dir(m->name, module_root);
	if (IS_ERR_OR_NULL(m->root))
		goto err;

	d = debugfs_create_x32("handle", S_IRUGO, m->root, &m->handle);
	if (IS_ERR_OR_NULL(d))
		goto err;

	d = debugfs_create_x32("size", S_IRUGO, m->root, &m->size);
	if (IS_ERR_OR_NULL(d))
		goto err;

	d = debugfs_create_file("unload", S_IRUSR, m->root, m,
			&bpmp_module_unload_fops);
	if (IS_ERR_OR_NULL(d))
		goto err;

	list_add_tail(&m->entry, &modules);

	mutex_unlock(&bpmp_lock);
	release_firmware(fw);
	uncache_firmware(m->name);
	return;

err:
	mutex_unlock(&bpmp_lock);
err_nolock:
	release_firmware(fw);
	uncache_firmware(m->name);
	debugfs_remove_recursive(m->root);
	kfree(m);
}

static int bpmp_module_load_show(void *data, u64 *val)
{
	static u32 serial;
	struct platform_device *pdev = data;
	struct bpmp_module *m;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (m == NULL)
		return -ENOMEM;

	*val = ++serial;
	snprintf(m->name, sizeof(m->name), "bpmp.mod.%u", serial);

	return request_firmware_nowait(THIS_MODULE, false, m->name,
			&pdev->dev, GFP_KERNEL, m, bpmp_module_ready);
}

DEFINE_SIMPLE_ATTRIBUTE(bpmp_module_load_fops, bpmp_module_load_show,
		NULL, "%lld\n");

static int bpmp_init_modules(struct platform_device *pdev)
{
	struct dentry *d;

	module_root = debugfs_create_dir("module", bpmp_root);
	if (IS_ERR_OR_NULL(module_root))
		goto clean;

	d = debugfs_create_file("load", S_IRUSR, module_root, pdev,
			&bpmp_module_load_fops);
	if (IS_ERR_OR_NULL(d))
		goto clean;

	return 0;

clean:
	WARN_ON(1);
	debugfs_remove_recursive(module_root);
	module_root = NULL;
	return -EFAULT;
}

struct bpmp_cpuidle_state {
	int id;
	const char *name;
};

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

static int bpmp_cpuidle_usage_show(void *data, u64 *val)
{
	struct bpmp_cpuidle_state *state = data;
	*val = bpmp_cpuidle_usage(state->id);
	return 0;
}

static int bpmp_cpuidle_time_show(void *data, u64 *val)
{
	struct bpmp_cpuidle_state *state = data;
	*val = bpmp_cpuidle_time(state->id);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cpuidle_usage_fops, bpmp_cpuidle_usage_show,
		NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(cpuidle_time_fops, bpmp_cpuidle_time_show,
		NULL, "%lld\n");

static const struct fops_entry cpuidle_attrs[] = {
	{ "name", &cpuidle_name_fops, S_IRUGO },
	{ "usage", &cpuidle_usage_fops, S_IRUGO },
	{ "time", &cpuidle_time_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static struct bpmp_cpuidle_state cpuidle_state[] = {
	{ TEGRA_PM_CC4, "CC4" },
	{ TEGRA_PM_CC6, "CC6" },
	{ TEGRA_PM_CC7, "CC7" },
	{ TEGRA_PM_SC2, "SC2" },
	{ TEGRA_PM_SC3, "SC3" },
	{ TEGRA_PM_SC4, "SC4" },
	{ TEGRA_PM_SC7, "SC7" }
};

static int bpmp_create_cpuidle_debug(int index, struct dentry *parent)
{
	struct bpmp_cpuidle_state *state;
	struct dentry *top;
	char name[16];

	sprintf(name, "state%d", index);
	top = debugfs_create_dir(name, parent);
	if (IS_ERR_OR_NULL(top))
		return -EFAULT;

	state = &cpuidle_state[index];
	return bpmp_create_attrs(cpuidle_attrs, top, state);
}

static int bpmp_init_cpuidle_debug(struct dentry *root)
{
	struct dentry *d;
	unsigned int i;

	d = debugfs_create_dir("cpuidle", root);
	if (IS_ERR_OR_NULL(d))
		return -EFAULT;

	for (i = 0; i < ARRAY_SIZE(cpuidle_state); i++) {
		if (bpmp_create_cpuidle_debug(i, d))
			return -EFAULT;
	}

	return 0;
}

static int bpmp_ping_show(void *data, u64 *val)
{
	*val = bpmp_ping();
	return 0;
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

DEFINE_SIMPLE_ATTRIBUTE(bpmp_ping_fops, bpmp_ping_show, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(trace_enable_fops, bpmp_trace_enable_show,
		bpmp_trace_enable_store, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(trace_disable_fops, NULL,
		bpmp_trace_disable_store, "%lld\n");

static int bpmp_copy_trace(struct seq_file *file, dma_addr_t phys, void *virt,
		int size)
{
	int ret;
	int eof = 0;

	while (!eof) {
		ret = bpmp_write_trace(phys, size, &eof);
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

static const struct fops_entry root_attrs[] = {
	{ "ping", &bpmp_ping_fops, S_IRUGO },
	{ "trace_enable", &trace_enable_fops, S_IRUGO | S_IWUSR },
	{ "trace_disable", &trace_disable_fops, S_IWUSR },
	{ "trace", &trace_fops, S_IRUGO },
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

	bpmp_root = root;
	return 0;

clean:
	WARN_ON(1);
	debugfs_remove_recursive(root);
	return -EFAULT;
}
#else
static inline int bpmp_init_debug(struct platform_device *pdev) { return 0; }
#endif

void tegra_bpmp_trace_printk(void)
{
	unsigned long flags;
	int eof = 0;

	spin_lock_irqsave(&shared_lock, flags);

	while (!eof) {
		if (bpmp_write_trace(shared_phys, SHARED_SIZE, &eof) <= 0)
			goto done;
		pr_info("%s", (char *)shared_virt);
	}

done:
	spin_unlock_irqrestore(&shared_lock, flags);
}

static int init_clks(void)
{
	cop_clk = clk_get_sys(NULL, "cop");
	if (IS_ERR(cop_clk)) {
		dev_err(device, "cannot get cop clock\n");
		return -ENODEV;
	}

	sclk = clk_get_sys(NULL, "sclk");
	if (IS_ERR(sclk)) {
		dev_err(device, "cannot get avp sclk\n");
		return -ENODEV;
	}

	emc_clk = clk_get_sys(NULL, "emc");
	if (IS_ERR(emc_clk)) {
		dev_err(device, "cannot get avp emc clk\n");
		return -ENODEV;
	}

	clk_prepare_enable(cop_clk);
	clk_prepare_enable(sclk);
	clk_prepare_enable(emc_clk);

	return 0;
}

static int bpmp_probe(struct platform_device *pdev)
{
	dma_addr_t phys;
	void *virt;
	int r;

	device = &pdev->dev;
	platform_data = device->platform_data;

	bpmp_virt = ioremap(platform_data->phys_start, platform_data->size);
	dev_info(device, "%x@%x mapped to %p\n", (u32)platform_data->size,
			(u32)platform_data->phys_start, bpmp_virt);

	virt = dma_alloc_coherent(device, SHARED_SIZE, &phys, GFP_KERNEL);
	if (!virt)
		return -ENOMEM;

	r = init_clks();
	if (r)
		goto abort;

	r = bpmp_ipc_init(pdev);
	if (r)
		goto abort;

	mutex_init(&bpmp_lock);

	r = bpmp_init_debug(pdev);
	if (r)
		goto abort;

	r = bpmp_init_modules(pdev);
	if (r)
		goto abort;

	shared_virt = virt;
	shared_phys = (uint32_t)phys;
	return 0;

abort:
	dma_free_coherent(device, SHARED_SIZE, virt, phys);
	return r;
}

static struct platform_driver bpmp_driver = {
	.probe = bpmp_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bpmp"
	}
};

static __init int bpmp_init(void)
{
	return platform_driver_register(&bpmp_driver);
}
module_init(bpmp_init);
