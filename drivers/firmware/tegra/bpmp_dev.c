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

static struct clk *cop_clk;
static struct clk *sclk;
static struct clk *emc_clk;
static void *bpmp_virt;
static struct tegra_bpmp_platform_data *platform_data;
static struct device *device;
static struct mutex bpmp_lock;

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

static int bpmp_ping_show(void *data, u64 *val)
{
	*val = bpmp_ping();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bpmp_ping_fops, bpmp_ping_show, NULL, "%lld\n");

static int bpmp_init_debug(struct platform_device *pdev)
{
	struct dentry *root;
	struct dentry *d;

	root = debugfs_create_dir("bpmp", NULL);
	if (IS_ERR_OR_NULL(root)) {
		WARN_ON(1);
		return -EFAULT;
	}

	d = debugfs_create_file("ping", S_IRUSR, root, pdev, &bpmp_ping_fops);
	if (IS_ERR_OR_NULL(d))
		goto clean;

	return 0;

clean:
	WARN_ON(1);
	debugfs_remove_recursive(root);
	return -EFAULT;
}
#else
static inline int bpmp_init_debug(struct platform_device *pdev) { return 0; }
#endif

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
	int r;

	device = &pdev->dev;
	platform_data = device->platform_data;

	bpmp_virt = ioremap(platform_data->phys_start, platform_data->size);
	dev_info(device, "%x@%x mapped to %p\n", (u32)platform_data->size,
			(u32)platform_data->phys_start, bpmp_virt);

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

	return 0;

abort:
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
