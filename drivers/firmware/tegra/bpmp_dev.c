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
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_data/tegra_bpmp.h>
#include <linux/platform_device.h>
#include <mach/clk.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp_private.h"

static struct clk *cop_clk;
static struct clk *sclk;
static struct clk *emc_clk;
static void *bpmp_virt;
static struct tegra_bpmp_platform_data *platform_data;
static struct device *device;

#ifdef CONFIG_DEBUG_FS
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

	r = bpmp_init_debug(pdev);
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
