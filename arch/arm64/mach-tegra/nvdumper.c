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
#include "board.h"
#include <mach/nvdumper.h>
#include <mach/nvdumper-footprint.h>

#ifdef CONFIG_TEGRA_USE_NCT
#include <mach/nct.h>
#endif

#define NVDUMPER_CLEAN 0xf000caf3U
#define NVDUMPER_DIRTY 0xdeadbeefU

#define RW_MODE (S_IWUSR | S_IRUGO)

static uint32_t *nvdumper_ptr;

static int get_dirty_state(void)
{
	uint32_t val;

	val = ioread32(nvdumper_ptr);
	if (val == NVDUMPER_DIRTY)
		return 1;
	else if (val == NVDUMPER_CLEAN)
		return 0;
	else
		return -1;
}

static void set_dirty_state(int dirty)
{
	if (dirty)
		iowrite32(NVDUMPER_DIRTY, nvdumper_ptr);
	else
		iowrite32(NVDUMPER_CLEAN, nvdumper_ptr);
}

static int nvdumper_reboot_cb(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	pr_info("nvdumper: rebooting cleanly.\n");
	set_dirty_state(0);
	return NOTIFY_DONE;
}

struct notifier_block nvdumper_reboot_notifier = {
	.notifier_call = nvdumper_reboot_cb,
};

static int __init nvdumper_init(void)
{
	int ret, dirty;

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

	dirty = get_dirty_state();
	switch (dirty) {
	case 0:
		pr_info("nvdumper: last reboot was clean\n");
		break;
	case 1:
		pr_info("nvdumper: last reboot was dirty\n");
		break;
	default:
		pr_info("nvdumper: last reboot was unknown\n");
		break;
	}
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
		goto err_out3;
	}

	pr_info("%s: RAMDUMP flag(%d) from NCT\n",
			__func__, item->ramdump.flag);
	if (item->ramdump.flag == 1)
		set_dirty_state(1);
	else
		set_dirty_state(0);

	kfree(item);

	return 0;

err_out3:

#else
	set_dirty_state(1);
	return 0;
#endif

err_out2:
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
err_out1:
	iounmap(nvdumper_ptr);

	return ret;

}

static void __exit nvdumper_exit(void)
{
	nvdumper_regdump_exit();
	nvdumper_dbg_footprint_exit();
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
	set_dirty_state(0);
	iounmap(nvdumper_ptr);
}

arch_initcall(nvdumper_init);
module_exit(nvdumper_exit);

MODULE_LICENSE("GPL");
