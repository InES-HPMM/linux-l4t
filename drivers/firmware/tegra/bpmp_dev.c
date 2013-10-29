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
#include <mach/clk.h>
#include "../../../arch/arm/mach-tegra/iomap.h"
#include "bpmp_private.h"

#define FLOW_CTRL_HALT_COP_EVENTS	IO_ADDRESS(TEGRA_FLOW_CTRL_BASE + 0x4)
#define FLOW_MODE_STOP			(0x2 << 29)
#define FLOW_MODE_NONE			0x0
#define TEGRA_NVAVP_RESET_VECTOR_ADDR	IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE + 0x200)
#define CLK_ENB_V_SET			IO_ADDRESS(TEGRA_CLK_RESET_BASE + 0x440)
#define SET_CLK_ENB_ATOMICS		(1 << 16)

static struct clk *cop_clk;
static struct clk *sclk;
static struct clk *emc_clk;
static void *bpmp_virt;
static struct tegra_bpmp_platform_data *platform_data;
static struct device *device;

#ifdef CONFIG_DEBUG_FS
struct header {
	u32 magic;
	u32 version;
	u32 chipid;
	u32 memsize;
	u32 reset_off;
};

struct platform_config {
	u32 magic;
	u32 version;
	u32 chip_id;
	u32 chip_version;
	u32 sku;
	u32 debug_port;
};

struct platform_config bpmp_config = {
	.magic = 0x757063,
	.version = 0,
	.chip_id = 0x40,
	.chip_version = 0,
	.sku = 0,
	.debug_port = 4
};

static void bpmp_reset(u32 addr)
{
	writel(FLOW_MODE_STOP, FLOW_CTRL_HALT_COP_EVENTS);
	writel(addr, TEGRA_NVAVP_RESET_VECTOR_ADDR);

	tegra_periph_reset_assert(cop_clk);
	udelay(2);
	tegra_periph_reset_deassert(cop_clk);

	writel(FLOW_MODE_NONE, FLOW_CTRL_HALT_COP_EVENTS);
}

static void bpmp_fwready(const struct firmware *fw, void *context)
{
	struct header *h;
	u32 addr;
	unsigned int cfgsz = sizeof(bpmp_config);

	if (!fw) {
		dev_err(device, "firmware not ready\n");
		return;
	}

	dev_info(device, "firmware_ready: %d bytes\n", fw->size);

	h = (struct header *)fw->data;
	addr = platform_data->phys_start + h->reset_off;

	dev_info(device, "magic     : %x\n", h->magic);
	dev_info(device, "version   : %x\n", h->version);
	dev_info(device, "chip      : %x\n", h->chipid);
	dev_info(device, "memsize   : %u byes\n", h->memsize);
	dev_info(device, "reset off : %x\n", h->reset_off);
	dev_info(device, "reset addr: %x\n", addr);

	if (fw->size > h->memsize ||
			h->memsize + cfgsz > platform_data->size) {
		dev_info(device, "firmware too big\n");
		return;
	}

	memcpy(bpmp_virt, fw->data, fw->size);
	memset(bpmp_virt + fw->size, 0, platform_data->size - fw->size);
	memcpy(bpmp_virt + h->memsize, &bpmp_config, cfgsz);

	bpmp_reset(addr);
	release_firmware(fw);
}

static int bpmp_reset_store(void *data, u64 val)
{
	struct platform_device *pdev = data;

	return request_firmware_nowait(THIS_MODULE, false, "bpmpfw.bin",
			&pdev->dev, GFP_KERNEL, NULL, bpmp_fwready);
}

DEFINE_SIMPLE_ATTRIBUTE(bpmp_reset_fops, NULL, bpmp_reset_store, "%lld\n");

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

	d = debugfs_create_file("reset", S_IWUSR, root, pdev, &bpmp_reset_fops);
	if (IS_ERR_OR_NULL(d))
		goto clean;

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

static int bpmp_probe(struct platform_device *pdev)
{
	int r;

	device = &pdev->dev;
	platform_data = device->platform_data;

	bpmp_virt = ioremap(platform_data->phys_start, platform_data->size);
	dev_info(device, "%x@%x mapped to %p\n", (u32)platform_data->size,
			(u32)platform_data->phys_start, bpmp_virt);

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
	clk_set_rate(sclk, ULONG_MAX);
	clk_set_rate(emc_clk, ULONG_MAX);

	writel(SET_CLK_ENB_ATOMICS, CLK_ENB_V_SET);

	r = request_irq(INT_SHR_SEM_INBOX_IBF, bpmp_inbox_irq, 0,
			dev_name(&pdev->dev), NULL);
	if (r)
		return r;

	return bpmp_init_debug(pdev);
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
