/*
 * arch/arm/mach-tegra/tegra_bb.c
 *
 * Copyright (C) 2012-2013 NVIDIA Corporation. All rights reserved.
 *
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
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <mach/clk.h>
#include <mach/tegra_bb.h>
#include <linux/platform_data/nvshm.h>

#include "clock.h"
#include "iomap.h"
#include "sleep.h"
#include "tegra_emc.h"

/* BB mailbox offset */
#define TEGRA_BB_REG_MAILBOX (0x0)
/* BB mailbox return code (MOSD) offset */
#define TEGRA_BB_REG_RETCODE (0x4)
/* BB status mask */
#define TEGRA_BB_STATUS_MASK  (0xffff)
#define TEGRA_BB_IPC_COLDBOOT  (0x0001)
#define TEGRA_BB_IPC_READY  (0x0005)

#define BBC_MC_MIN_FREQ		600000000

#define APBDEV_PMC_EVENT_COUNTER_0	(0x44c)
#define APBDEV_PMC_EVENT_COUNTER_0_EN_MASK	(1<<20)
#define APBDEV_PMC_EVENT_COUNTER_0_LP0BB	(0<<16)
#define APBDEV_PMC_EVENT_COUNTER_0_LP0ACTIVE	(1<<16)

#define APBDEV_PMC_IPC_PMC_IPC_STS_0 (0x500)
#define APBDEV_PMC_IPC_PMC_IPC_STS_0_AP2BB_RESET_SHIFT (1)
#define APBDEV_PMC_IPC_PMC_IPC_STS_0_AP2BB_RESET_DEFAULT_MASK (1)
#define APBDEV_PMC_IPC_PMC_IPC_STS_0_BB2AP_MEM_REQ_SHIFT (3)
#define APBDEV_PMC_IPC_PMC_IPC_STS_0_BB2AP_MEM_REQ_SOON_SHIFT (4)

#define APBDEV_PMC_IPC_PMC_IPC_SET_0 (0x504)
#define APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_RESET_SHIFT (1)
#define APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_MEM_STS_SHIFT (5)

#define APBDEV_PMC_IPC_PMC_IPC_CLEAR_0 (0x508)
#define APBDEV_PMC_IPC_PMC_IPC_CLEAR_0_AP2BB_RESET_SHIFT (1)

#define FLOW_CTLR_IPC_FLOW_IPC_STS_0 (0x500)
#define FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_MSC_STS_SHIFT  (4)
#define FLOW_CTLR_IPC_FLOW_IPC_STS_0_BB2AP_INT1_STS_SHIFT (3)
#define FLOW_CTLR_IPC_FLOW_IPC_STS_0_BB2AP_INT0_STS_SHIFT (2)
#define FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_INT0_STS_SHIFT (0)

#define FLOW_CTLR_IPC_FLOW_IPC_SET_0 (0x504)
#define FLOW_CTLR_IPC_FLOW_IPC_SET_0_AP2BB_INT0_STS_SHIFT (0)

#define FLOW_CTLR_IPC_FLOW_IPC_CLR_0 (0x508)
#define FLOW_CTLR_IPC_FLOW_IPC_CLR_0_BB2AP_INT1_STS_SHIFT (3)
#define FLOW_CTLR_IPC_FLOW_IPC_CLR_0_BB2AP_INT0_STS_SHIFT (2)
#define FLOW_CTLR_IPC_FLOW_IPC_CLR_0_AP2BB_INT0_STS_SHIFT (0)

#define MC_BBC_MEM_REGIONS_0_OFFSET (0xF0)

#define MC_BBC_MEM_REGIONS_0_PRIV_SIZE_MASK  (0x3)
#define MC_BBC_MEM_REGIONS_0_PRIV_SIZE_SHIFT (0)

#define MC_BBC_MEM_REGIONS_0_PRIV_BASE_MASK  (0x1FF)
#define MC_BBC_MEM_REGIONS_0_PRIV_BASE_SHIFT (3)

#define MC_BBC_MEM_REGIONS_0_IPC_SIZE_MASK   (0x3)
#define MC_BBC_MEM_REGIONS_0_IPC_SIZE_SHIFT  (16)

#define MC_BBC_MEM_REGIONS_0_IPC_BASE_MASK   (0x1FF)
#define MC_BBC_MEM_REGIONS_0_IPC_BASE_SHIFT  (19)

enum bbc_pm_state {
	BBC_REMOVE_FLOOR = 1,
	BBC_SET_FLOOR,
};

struct tegra_bb {
	spinlock_t lock;
	int status;
	int instance;
	char name[16];
	char priv_name[16];
	char ipc_name[16];
	unsigned long priv_phy;
	unsigned long ipc_phy;
	void *ipc_virt;
	void *mb_virt;
	unsigned long priv_size;
	unsigned long ipc_size;
	unsigned long ipc_irq;
	char ipc_serial[NVSHM_SERIAL_BYTE_SIZE];
	unsigned int irq;
	unsigned int mem_req_soon;
	enum bbc_pm_state state, prev_state;
	struct regulator *vdd_bb_core;
	struct regulator *vdd_bb_pll;
	void (*ipc_cb)(void *data);
	void  *ipc_cb_data;
	struct miscdevice dev_priv;
	struct miscdevice dev_ipc;
	struct device *dev;
	struct sysfs_dirent *sd;
	struct nvshm_platform_data nvshm_pdata;
	struct platform_device nvshm_device;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct clk *emc_clk;
};

static int tegra_bb_open(struct inode *inode, struct file *filp);
static int tegra_bb_map(struct file *filp, struct vm_area_struct *vma);
static int tegra_bb_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

static const struct file_operations tegra_bb_priv_fops = {
	.owner		= THIS_MODULE,
	.open           = tegra_bb_open,
	.mmap		= tegra_bb_map,
};

static const struct file_operations tegra_bb_ipc_fops = {
	.owner		= THIS_MODULE,
	.open           = tegra_bb_open,
	.mmap		= tegra_bb_map,
};

static const struct vm_operations_struct tegra_bb_vm_ops = {
	.fault = tegra_bb_vm_fault,
};

void tegra_bb_register_ipc(struct platform_device *pdev,
			   void (*cb)(void *data), void *cb_data)
{
	struct tegra_bb *bb;
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		pr_err("%s platform device is NULL!\n", __func__);
		return;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform dev not found!\n", __func__);
		return;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return;
	}
	bb->ipc_cb = cb;
	bb->ipc_cb_data = cb_data;
}
EXPORT_SYMBOL_GPL(tegra_bb_register_ipc);

void tegra_bb_generate_ipc(struct platform_device *pdev)
{
	struct tegra_bb *bb;
#ifndef CONFIG_TEGRA_BASEBAND_SIMU
	void __iomem *flow = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
#endif
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		pr_err("%s platform device is NULL!\n", __func__);
		return;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform dev not found!\n", __func__);
		return;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return;
	}

#ifdef CONFIG_TEGRA_BASEBAND_SIMU
	{
		int sts;
		if (bb->ipc_cb)
			bb->ipc_cb(bb->ipc_cb_data);
		/* Notify sysfs */
		sts = *(unsigned int *)bb->mb_virt & TEGRA_BB_STATUS_MASK;

		if ((bb->status != TEGRA_BB_IPC_READY) ||
		    (bb->status != sts)) {
			sysfs_notify(&bb->dev->kobj, NULL, "status");
		}
		bb->status = sts;
	}
#else
	{
		u32 sts = readl(flow + FLOW_CTLR_IPC_FLOW_IPC_STS_0);
		sts |= 1 << FLOW_CTLR_IPC_FLOW_IPC_SET_0_AP2BB_INT0_STS_SHIFT |
		(0x8 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_MSC_STS_SHIFT);

		/* unmask mem_req_soon interrupt */
		writel(sts, flow + FLOW_CTLR_IPC_FLOW_IPC_SET_0);
	}
#endif
}
EXPORT_SYMBOL_GPL(tegra_bb_generate_ipc);

void tegra_bb_clear_ipc(struct platform_device *dev)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* clear BB2AP INT status */
	writel(1 << FLOW_CTLR_IPC_FLOW_IPC_CLR_0_BB2AP_INT0_STS_SHIFT,
	       fctrl + FLOW_CTLR_IPC_FLOW_IPC_CLR_0);
}
EXPORT_SYMBOL_GPL(tegra_bb_clear_ipc);

void tegra_bb_abort_ipc(struct platform_device *dev)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* clear AP2BB INT status */
	writel(1 << FLOW_CTLR_IPC_FLOW_IPC_CLR_0_AP2BB_INT0_STS_SHIFT,
	       fctrl + FLOW_CTLR_IPC_FLOW_IPC_CLR_0);
}
EXPORT_SYMBOL_GPL(tegra_bb_abort_ipc);

int tegra_bb_check_ipc(struct platform_device *dev)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	int sts;

	sts = readl(fctrl + FLOW_CTLR_IPC_FLOW_IPC_STS_0);
	if (sts & (1 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_INT0_STS_SHIFT))
		return 0;
	return 1;
}
EXPORT_SYMBOL_GPL(tegra_bb_check_ipc);

static int tegra_bb_open(struct inode *inode, struct file *filp)
{
	int ret;

	ret = nonseekable_open(inode, filp);
	return ret;
}

static int tegra_bb_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	pr_warn("%s  vma->vm_start=0x%x vma->vm_end=0x%x vma->vm_pgoff=0x%x\n"
		, __func__,
		(unsigned int)vma->vm_start,
		(unsigned int)vma->vm_end,
		(unsigned int)vma->vm_pgoff);
	vmf = vmf;
	return VM_FAULT_NOPAGE;
}

static int tegra_bb_map(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_bb *bb;
	unsigned long phy;
	unsigned long off;
	unsigned long vsize;
	unsigned long psize;
	int ret;

	off = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	bb = dev_get_drvdata(miscdev->parent);
	if (miscdev->fops == &tegra_bb_priv_fops) {
		phy = bb->priv_phy + off;
		pr_debug("%s map priv section @0x%x\n",
			__func__,
			(unsigned int)phy);
		psize = bb->priv_size - off;
	} else {
		phy = bb->ipc_phy + off;
		pr_debug("%s map ipc section @0x%x\n",
			__func__,
			(unsigned int)phy);
		psize =  bb->ipc_size - off;
	}
	if (vsize > psize) {
		pr_err("%s request exceed mapping!\n",
		       __func__);
		return -EINVAL;
	}

	vma->vm_ops = &tegra_bb_vm_ops;
	ret = remap_pfn_range(vma, vma->vm_start, __phys_to_pfn(phy),
			      vsize, pgprot_noncached(vma->vm_page_prot));
	if (ret) {
		pr_err("%s remap_pfn_range ret %d\n", __func__, (int)ret);
		return -EAGAIN;
	}
	return ret;
}

static ssize_t show_tegra_bb_retcode(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tegra_bb *bb;
	int retcode;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL!\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found!\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	retcode = *(int *)((unsigned long)bb->mb_virt + TEGRA_BB_REG_RETCODE);
	dev_dbg(&pdev->dev, "%s retcode=%d\n", __func__, (int)retcode);
	return sprintf(buf, "%d\n", retcode);
}
static DEVICE_ATTR(retcode, S_IRUSR | S_IRGRP, show_tegra_bb_retcode, NULL);

static ssize_t show_tegra_bb_status(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tegra_bb *bb;
	unsigned int *ptr;
	int status;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL!\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found!\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	ptr = bb->mb_virt + TEGRA_BB_REG_MAILBOX;
	status = *ptr & 0xFFFF;
	dev_dbg(&pdev->dev, "%s status=%x\n", __func__,
		 status);
	return sprintf(buf, "%d\n", status);
}

static ssize_t store_tegra_bb_status(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct tegra_bb *bb;
	int ret, value;
	unsigned int *ptr;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL!\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found!\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	ret = sscanf(buf, "%d", &value);
	if (ret != 1)
		return -1;
	value &= 0xFFFF;
	ptr = bb->mb_virt + TEGRA_BB_REG_MAILBOX;
	*ptr = (value & 0xFFFF) | ((~value << 16) & 0xFFFF0000);
	tegra_bb_generate_ipc(pdev);
	dev_dbg(&pdev->dev, "%s status=0x%x\n",
		 __func__,
		 (unsigned int)value);
	return count;
}

static DEVICE_ATTR(status, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
		   show_tegra_bb_status, store_tegra_bb_status);

static ssize_t show_tegra_bb_priv_size(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct tegra_bb *bb;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL!\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found!\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	dev_dbg(&pdev->dev, "%s priv_size=%d\n", __func__,
		 (unsigned int)bb->priv_size);
	return sprintf(buf, "%d\n", (int)bb->priv_size);
}
static DEVICE_ATTR(priv_size, S_IRUSR | S_IRGRP, show_tegra_bb_priv_size, NULL);

static ssize_t show_tegra_bb_ipc_size(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct tegra_bb *bb;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL!\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found!\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	dev_dbg(&pdev->dev, "%s ipc_size=%d\n",
		 __func__,
		 (unsigned int)bb->ipc_size);
	return sprintf(buf, "%d\n", (int)bb->ipc_size);
}
static DEVICE_ATTR(ipc_size, S_IRUSR | S_IRGRP, show_tegra_bb_ipc_size, NULL);

static ssize_t store_tegra_bb_reset(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret, value;
	static bool regulator_status;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;
	struct tegra_bb *bb;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;
	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	ret = sscanf(buf, "%d", &value);

	if (ret != 1)
		return -1;

	if ((value < 0) || (value > 1))
		return -EINVAL;

	dev_dbg(&pdev->dev, "%s reset=%d\n",
		 __func__,
		 (unsigned int)value);

	/* reset is active low - sysfs interface assume reset is active high */
	if (value) {
		writel(1 << APBDEV_PMC_IPC_PMC_IPC_CLEAR_0_AP2BB_RESET_SHIFT |
		       1 << APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_MEM_STS_SHIFT,
			pmc + APBDEV_PMC_IPC_PMC_IPC_CLEAR_0);

		if (regulator_status == true) {
			pr_debug("%s: disabling bbc regulators\n", __func__);
			regulator_disable(bb->vdd_bb_core);
			regulator_disable(bb->vdd_bb_pll);

			regulator_status = false;
		}
	} else {
		/* power on bbc rails */
		if (bb->vdd_bb_core && bb->vdd_bb_pll &&
				regulator_status == false) {
			pr_debug("%s: enabling bbc regulators\n", __func__);
			regulator_set_voltage(bb->vdd_bb_core, 1100000,
							1100000);
			regulator_enable(bb->vdd_bb_core);

			regulator_set_voltage(bb->vdd_bb_pll, 900000, 900000);
			regulator_enable(bb->vdd_bb_pll);

			regulator_status = true;
		}

		writel(1 << APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_RESET_SHIFT |
		       1 << APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_MEM_STS_SHIFT,
			pmc + APBDEV_PMC_IPC_PMC_IPC_SET_0);
	}

	return ret;
}

static ssize_t show_tegra_bb_reset(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int sts;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	/* reset is active low - sysfs interface assume reset is active high */

	sts = readl(pmc + APBDEV_PMC_IPC_PMC_IPC_STS_0);
	dev_dbg(&pdev->dev, "%s IPC_STS=0x%x\n",
		 __func__,
		 (unsigned int)sts);
	sts = ~sts >> APBDEV_PMC_IPC_PMC_IPC_STS_0_AP2BB_RESET_SHIFT;
	sts &= APBDEV_PMC_IPC_PMC_IPC_STS_0_AP2BB_RESET_DEFAULT_MASK;

	dev_dbg(&pdev->dev, "%s reset=%d\n",
		 __func__,
		 (unsigned int)sts);
	return sprintf(buf, "%d\n", (int)sts);
}

static DEVICE_ATTR(reset, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
		   show_tegra_bb_reset, store_tegra_bb_reset);

static ssize_t show_tegra_bb_state(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	unsigned int sts, mem_req, mem_req_soon;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	sts = readl(pmc + APBDEV_PMC_IPC_PMC_IPC_STS_0);
	dev_dbg(&pdev->dev, "%s IPC_STS=0x%x\n", __func__,
			 (unsigned int)sts);

	mem_req = (sts >> APBDEV_PMC_IPC_PMC_IPC_STS_0_BB2AP_MEM_REQ_SHIFT) & 1;
	mem_req_soon =
		(sts >> APBDEV_PMC_IPC_PMC_IPC_STS_0_BB2AP_MEM_REQ_SOON_SHIFT)
		& 1;

	dev_dbg(&pdev->dev, "%s mem_req=%d mem_req_soon=%d\n", __func__,
					mem_req, mem_req_soon);
	return sprintf(buf, "%d\n", (unsigned int)mem_req);
}

static DEVICE_ATTR(state, S_IRUSR | S_IRGRP, show_tegra_bb_state, NULL);

static ssize_t show_tegra_bb_serial(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct tegra_bb *bb;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct tegra_bb_platform_data *pdata;
	int idx, ret;

	if (!pdev) {
		dev_err(dev, "%s platform device is NULL!\n", __func__);
		return 0;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform data not found!\n", __func__);
		return 0;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	for (idx = 0; NVSHM_SERIAL_BYTE_SIZE > idx; ++idx) {
		ret = sprintf(buf+2*idx, "%02X", bb->ipc_serial[idx]);
		if (ret < 0) {
			dev_err(&pdev->dev, "%s sprintf shm serial failure!\n",
					__func__);
			return 0;
		}
	}

	buf[2*NVSHM_SERIAL_BYTE_SIZE] = '\n';

	return (2*NVSHM_SERIAL_BYTE_SIZE+1);
}

static DEVICE_ATTR(serial, S_IRUSR | S_IRGRP, show_tegra_bb_serial, NULL);

void tegra_bb_set_ipc_serial(struct platform_device *pdev, char *serial)
{
	struct tegra_bb *bb;
	struct tegra_bb_platform_data *pdata;

	if (!pdev) {
		pr_err("%s platform device is NULL!\n", __func__);
		return;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s platform dev not found!\n", __func__);
		return;
	}

	bb = (struct tegra_bb *)pdata->bb_handle;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return;
	}

	if (serial == NULL) {
		/* Remove sysfs entry */
		device_remove_file(&pdev->dev, &dev_attr_serial);
		return;
	}

	/* Create sysfs entry */
	device_create_file(&pdev->dev, &dev_attr_serial);

	/* Locally store serail number for future sysfs access */
	memcpy(bb->ipc_serial, serial, sizeof(bb->ipc_serial));
}
EXPORT_SYMBOL_GPL(tegra_bb_set_ipc_serial);

#ifndef CONFIG_TEGRA_BASEBAND_SIMU
static irqreturn_t tegra_bb_isr_handler(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	unsigned long irq_flags;
	int sts;

	disable_irq_nosync(irq);

	spin_lock_irqsave(&bb->lock, irq_flags);
	/* read/clear INT status */
	sts = readl(fctrl + FLOW_CTLR_IPC_FLOW_IPC_STS_0);
	if (sts & (1 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_BB2AP_INT0_STS_SHIFT)) {
		writel(1 << FLOW_CTLR_IPC_FLOW_IPC_CLR_0_BB2AP_INT0_STS_SHIFT,
		       fctrl + FLOW_CTLR_IPC_FLOW_IPC_CLR_0);
	}
	spin_unlock_irqrestore(&bb->lock, irq_flags);
	/* wake IPC mechanism */
	if (bb->ipc_cb)
		bb->ipc_cb(bb->ipc_cb_data);

	/* Notify sysfs */
	sts = *(unsigned int *)bb->mb_virt & TEGRA_BB_STATUS_MASK;

	if ((bb->status != TEGRA_BB_IPC_READY) ||
	    (bb->status != sts)) {
		pr_debug("%s: notify sysfs status %d\n", __func__, sts);
		sysfs_notify_dirent(bb->sd);
	}
	bb->status = sts;
	return IRQ_HANDLED;
}

static irqreturn_t tegra_bb_mem_req_soon(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	u32 fc_sts;

	spin_lock(&bb->lock);
	/* clear the interrupt */
	fc_sts = readl(fctrl + FLOW_CTLR_IPC_FLOW_IPC_STS_0);
	if (fc_sts &
		(0x8 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_MSC_STS_SHIFT)) {

		writel((0x8 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_MSC_STS_SHIFT)
				, fctrl + FLOW_CTLR_IPC_FLOW_IPC_CLR_0);

		bb->state = BBC_SET_FLOOR;
		queue_work(bb->workqueue, &bb->work);
	}
	spin_unlock(&bb->lock);

	return IRQ_HANDLED;
}

static inline void tegra_bb_enable_mem_req_soon(void)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	int val = readl(fctrl + FLOW_CTLR_IPC_FLOW_IPC_STS_0);

	/* AP2BB_MSC_STS[3] is to mask or unmask
	 * mem_req_soon interrupt to interrupt controller */
	val = val | (0x8 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_AP2BB_MSC_STS_SHIFT);
	writel(val, fctrl + FLOW_CTLR_IPC_FLOW_IPC_SET_0);

	pr_debug("%s: fctrl ipc_sts = %x\n", __func__, val);
}

static inline void pmc_32kwritel(u32 val, unsigned long offs)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	writel(val, pmc + offs);
	udelay(130);
}

static void tegra_bb_enable_pmc_wake(void)
{
	/* Set PMC wake interrupt on active low
	 * please see Bug 1181348 for details of SW WAR
	 */
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg = readl(pmc + PMC_CTRL2);
	reg &= ~(PMC_CTRL2_WAKE_DET_EN);
	pmc_32kwritel(reg, PMC_CTRL2);

	reg = readl(pmc + PMC_WAKE2_LEVEL);
	reg &= ~(PMC_WAKE2_BB_MEM_REQ);
	pmc_32kwritel(reg, PMC_WAKE2_LEVEL);

	pmc_32kwritel(1, PMC_AUTO_WAKE_LVL);

	reg = readl(pmc + PMC_WAKE2_MASK);
	reg |= PMC_WAKE2_BB_MEM_REQ;
	pmc_32kwritel(reg, PMC_WAKE2_MASK);

	reg = readl(pmc + PMC_CTRL2);
	reg |= PMC_CTRL2_WAKE_DET_EN;
	pmc_32kwritel(reg, PMC_CTRL2);

	pr_debug("%s\n", __func__);
}

static irqreturn_t tegra_pmc_wake_intr(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	static void __iomem *tert_ictlr = IO_ADDRESS(TEGRA_TERTIARY_ICTLR_BASE);
	u32 lic, sts;
	int mem_req;

	spin_lock(&bb->lock);
	sts = readl(pmc + APBDEV_PMC_IPC_PMC_IPC_STS_0);
	mem_req = sts & (1 << APBDEV_PMC_IPC_PMC_IPC_STS_0_BB2AP_MEM_REQ_SHIFT);

	/* clear interrupt */
	lic = readl(tert_ictlr + TRI_ICTLR_VIRQ_CPU);
	if (lic & TRI_ICTLR_PMC_WAKE_INT) {
		pr_debug("clear pmc_wake sts %x mem_req %d\n", sts, mem_req);
		writel(TRI_ICTLR_PMC_WAKE_INT,
				tert_ictlr + TRI_ICTLR_CPU_IER_CLR);
	}

	if (!mem_req) {
		/* reenable mem_req_soon irq */
		tegra_bb_enable_mem_req_soon();

		bb->state = BBC_REMOVE_FLOOR;
		queue_work(bb->workqueue, &bb->work);
	}
	spin_unlock(&bb->lock);

	return IRQ_HANDLED;
}

static void tegra_bb_emc_dvfs(struct work_struct *work)
{
	struct tegra_bb *bb = container_of(work, struct tegra_bb, work);
	unsigned long flags;

	if (!bb)
		return;

	spin_lock_irqsave(&bb->lock, flags);
	if (bb->prev_state == bb->state) {
		spin_unlock_irqrestore(&bb->lock, flags);
		return;
	}

	switch (bb->state) {
	case BBC_SET_FLOOR:
		tegra_bb_enable_pmc_wake();
		bb->prev_state = bb->state;
		spin_unlock_irqrestore(&bb->lock, flags);

		/* going from 0 to high */
		clk_prepare_enable(bb->emc_clk);
		tegra_emc_dsr_override(TEGRA_EMC_DSR_OVERRIDE);
		clk_set_rate(bb->emc_clk, BBC_MC_MIN_FREQ);
		pr_debug("bbc setting floor to %d\n", BBC_MC_MIN_FREQ/1000000);

		return;
	case BBC_REMOVE_FLOOR:
		/* discard erroneous request */
		if (bb->prev_state != BBC_SET_FLOOR) {
			spin_unlock_irqrestore(&bb->lock, flags);
			return;
		}
		bb->prev_state = bb->state;
		spin_unlock_irqrestore(&bb->lock, flags);

		/* going from high to 0 */
		tegra_emc_dsr_override(TEGRA_EMC_DSR_NORMAL);
		clk_set_rate(bb->emc_clk, 0);
		clk_disable_unprepare(bb->emc_clk);
		pr_debug("bbc removing emc floor\n");

		return;
	default:
		spin_unlock_irqrestore(&bb->lock, flags);
		break;
	}

	return;
}
#endif

static int tegra_bb_probe(struct platform_device *pdev)
{
	struct tegra_bb *bb;
	int ret;
	struct tegra_bb_platform_data *pdata;
	void __iomem *tegra_mc = IO_ADDRESS(TEGRA_MC_BASE);
	unsigned int size, bbc_mem_regions_0;
	struct clk *c;


	if (!pdev) {
		pr_err("%s platform device is NULL!\n", __func__);
		return -EINVAL;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "%s\n", __func__);
	bb =  kzalloc(sizeof(struct tegra_bb), GFP_KERNEL);

	if (bb == NULL) {
		kfree(bb);
		return -ENOMEM;
	}

	spin_lock_init(&bb->lock);

	/* Private region */
	bbc_mem_regions_0 = readl(tegra_mc + MC_BBC_MEM_REGIONS_0_OFFSET);

	pr_info("%s MC_BBC_MEM_REGIONS_0=0x%x\n", __func__, bbc_mem_regions_0);

	size = (bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_PRIV_SIZE_SHIFT) &
		MC_BBC_MEM_REGIONS_0_PRIV_SIZE_MASK;

	/* Private */
	switch (size) {
	case 0:
		bb->priv_size = SZ_8M;
		break;
	case 1:
		bb->priv_size = SZ_16M;
		break;
	case 2:
		bb->priv_size = SZ_32M;
		break;
	case 3:
		bb->priv_size = SZ_64M;
		break;
	case 7:
		pr_err("%s no private memory mapped\n", __func__);
		break;
	default:
		pr_err("%s invalid private memory size 0x%x\n", __func__, size);
		break;
	}

	bb->priv_phy =
		((bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_PRIV_BASE_SHIFT)
		 & MC_BBC_MEM_REGIONS_0_PRIV_BASE_MASK) << 23;

	/* IPC */
	size = (bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_IPC_SIZE_SHIFT) &
		MC_BBC_MEM_REGIONS_0_IPC_SIZE_MASK;

	switch (size) {
	case 0:
		bb->ipc_size = SZ_8M;
		break;
	case 1:
		bb->ipc_size = SZ_16M;
		break;
	case 2:
		bb->ipc_size = SZ_32M;
		break;
	case 3:
		bb->ipc_size = SZ_64M;
		break;
	case 7:
		pr_err("%s no IPC memory mapped\n", __func__);
		break;
	default:
		pr_err("%s invalid IPC  memory size 0x%x\n", __func__, size);
		break;
	}

	bb->ipc_phy =
		((bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_IPC_BASE_SHIFT)
		 & MC_BBC_MEM_REGIONS_0_IPC_BASE_MASK) << 23;

	pr_info("%s  priv@0x%lx/0x%lx\n", __func__,
		 (unsigned long)bb->priv_phy,
		bb->priv_size);

	pr_info("%s  ipc@0x%lx/0x%lx\n", __func__,
		 (unsigned long)bb->ipc_phy,
		bb->ipc_size);

	if (!(bb->ipc_size && bb->priv_size)) {
		pr_err("%s: invalid tegra_bb regions\n", __func__);
		BUG();
	}

	bb->irq = pdata->bb_irq;
	bb->mem_req_soon = pdata->mem_req_soon;

	pdata->bb_handle = bb;

	/* Map mb_virt uncached (first 4K of IPC) */
	bb->mb_virt = ioremap_nocache(bb->ipc_phy,
					      SZ_1K*4);
	pr_debug("%s: uncached IPC Virtual=0x%p\n", __func__, bb->mb_virt);

	/* IPC memory is cached */
	bb->ipc_virt =  ioremap_cached(bb->ipc_phy,
					bb->ipc_size);
	pr_debug("%s: IPC Virtual=0x%p\n", __func__, bb->ipc_virt);

	/* clear the first 4K of IPC memory */
	memset(bb->mb_virt, 0, SZ_1K*4);

	/* init value of cold boot */
	*(unsigned int *)bb->mb_virt = TEGRA_BB_IPC_COLDBOOT |
		((~TEGRA_BB_IPC_COLDBOOT) << 16);

	/* Register devs */
	bb->dev_priv.minor = MISC_DYNAMIC_MINOR;
	snprintf(bb->priv_name, sizeof(bb->priv_name),
		 "tegra_bb_priv%d", pdev->id);

	bb->dev_priv.name = bb->priv_name;
	bb->dev_priv.fops = &tegra_bb_priv_fops;
	bb->dev_priv.parent = &pdev->dev;

	bb->dev_ipc.minor = MISC_DYNAMIC_MINOR;
	snprintf(bb->ipc_name, sizeof(bb->ipc_name),
		 "tegra_bb_ipc%d",
		 pdev->id);

	bb->dev_ipc.name = bb->ipc_name;
	bb->dev_ipc.fops = &tegra_bb_ipc_fops;
	bb->dev_ipc.parent = &pdev->dev;

	ret = misc_register(&bb->dev_priv);
	if (ret) {
		dev_err(&pdev->dev, "unable to register miscdevice %s\n",
			bb->dev_priv.name);
		kfree(bb);
		return -EAGAIN;
	}

	ret = misc_register(&bb->dev_ipc);
	if (ret) {
		dev_err(&pdev->dev, "unable to register miscdevice %s\n",
			bb->dev_ipc.name);
		kfree(bb);
		return -EAGAIN;
	}

	bb->dev = &pdev->dev;
	bb->instance = pdev->id;
	bb->status = 0;

	ret = device_create_file(&pdev->dev, &dev_attr_status);
	ret = device_create_file(&pdev->dev, &dev_attr_retcode);
	ret = device_create_file(&pdev->dev, &dev_attr_priv_size);
	ret = device_create_file(&pdev->dev, &dev_attr_ipc_size);
	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	ret = device_create_file(&pdev->dev, &dev_attr_state);

	bb->sd = sysfs_get_dirent(pdev->dev.kobj.sd, NULL, "status");

	bb->vdd_bb_core = regulator_get(NULL, "vdd_bb");
	if (IS_ERR_OR_NULL(bb->vdd_bb_core)) {
		pr_err("vdd_bb regulator get failed\n");
		bb->vdd_bb_core = NULL;
	}

	bb->vdd_bb_pll = regulator_get(NULL, "avdd_bb_pll");
	if (IS_ERR_OR_NULL(bb->vdd_bb_pll)) {
		pr_err("avdd_bb_pll regulator get failed\n");
		bb->vdd_bb_pll = NULL;
	}

	/* clk enable for mc_bbc / pll_p_bbc */
	c = tegra_get_clock_by_name("mc_bbc");
	if (IS_ERR_OR_NULL(c))
		pr_err("mc_bbc get failed\n");
	else
		clk_enable(c);
	c = tegra_get_clock_by_name("pll_p_bbc");
	if (IS_ERR_OR_NULL(c))
		pr_err("pll_p_bbc get failed\n");
	else
		clk_enable(c);

	bb->nvshm_pdata.ipc_base_virt = bb->ipc_virt;
	bb->nvshm_pdata.ipc_size = bb->ipc_size;
	bb->nvshm_pdata.mb_base_virt = bb->mb_virt;
	bb->nvshm_pdata.mb_size = 4*SZ_1K;
	bb->nvshm_pdata.bb_irq = bb->irq;
	bb->nvshm_pdata.tegra_bb = pdev;
	bb->nvshm_device.name = "nvshm";
	bb->nvshm_device.id = bb->instance;
	bb->nvshm_device.dev.platform_data = &bb->nvshm_pdata;
	platform_device_register(&bb->nvshm_device);
	platform_set_drvdata(pdev, bb);

#ifndef CONFIG_TEGRA_BASEBAND_SIMU
	snprintf(bb->name, sizeof(bb->name), "tegra_bb%d", pdev->id);

	ret = request_irq(bb->irq, tegra_bb_isr_handler, IRQF_TRIGGER_HIGH,
				bb->name, bb);
	if (ret) {
		dev_err(&pdev->dev, "Could not register irq handler\n");
		kfree(bb);
		return -EAGAIN;
	}

	ret = enable_irq_wake(bb->irq);
	if (ret) {
		dev_err(&pdev->dev, "set enable_irq_wake failed\n");
		kfree(bb);
		return -EAGAIN;
	}

	/* setup emc dvfs request framework */
	bb->emc_clk = clk_get(&pdev->dev, "emc");
	if (IS_ERR(bb->emc_clk)) {
		dev_err(&pdev->dev, "can't get emc clock\n");
		kfree(bb);
		return -ENOENT;
	}

	bb->workqueue = alloc_workqueue("bbc-pm", WQ_HIGHPRI, 1);
	if (!bb->workqueue) {
		dev_err(&pdev->dev, "failed to create workqueue\n");
		kfree(bb);
		return -ENOMEM;
	}
	INIT_WORK(&bb->work, tegra_bb_emc_dvfs);

	ret = request_irq(bb->mem_req_soon, tegra_bb_mem_req_soon,
			IRQF_TRIGGER_RISING, "bb_mem_req_soon", bb);
	if (ret) {
		dev_err(&pdev->dev, "Could not register mem_req_soon irq\n");
		kfree(bb);
		return -EAGAIN;
	}
	tegra_bb_enable_mem_req_soon();

	ret = request_irq(INT_PMC_WAKE_INT, tegra_pmc_wake_intr,
			IRQF_TRIGGER_RISING, "tegra_pmc_wake_intr", bb);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not register pmc_wake_int irq handler\n");
		kfree(bb);
		return -EAGAIN;
	}
	tegra_bb_enable_pmc_wake();

#endif
	return 0;
}

#ifdef CONFIG_PM
static int tegra_bb_suspend(struct platform_device *pdev, pm_message_t state)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	int sts;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	/* Be sure to ack any pending irq from BB */

	sts = readl(fctrl + FLOW_CTLR_IPC_FLOW_IPC_STS_0);
	if (sts & (1 << FLOW_CTLR_IPC_FLOW_IPC_STS_0_BB2AP_INT0_STS_SHIFT)) {
		writel(1 << FLOW_CTLR_IPC_FLOW_IPC_CLR_0_BB2AP_INT0_STS_SHIFT,
		       fctrl + FLOW_CTLR_IPC_FLOW_IPC_CLR_0);
	}
	return 0;
}

static int tegra_bb_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);
#ifndef CONFIG_TEGRA_BASEBAND_SIMU
	tegra_bb_enable_mem_req_soon();
#endif
	return 0;
}
#endif

static struct platform_driver tegra_bb_driver = {
	.driver = {
		.name = "tegra_bb",
		.owner = THIS_MODULE,
	},
	.probe = tegra_bb_probe,
#ifdef CONFIG_PM
	.suspend = tegra_bb_suspend,
	.resume = tegra_bb_resume,
#endif
};

static int __init tegra_bb_init(void)
{
	int ret;
	ret = platform_driver_register(&tegra_bb_driver);
	pr_debug("%s ret %d\n", __func__, ret);
	return ret;
}

static void __exit tegra_bb_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&tegra_bb_driver);
}

fs_initcall(tegra_bb_init);
module_exit(tegra_bb_exit);

#ifdef CONFIG_DEBUG_FS
static struct dentry *bb_debugfs_root;
static enum pmc_event_sel {
	PMC_EVENT_NONE,
	PMC_EVENT_LP0BB,
	PMC_EVENT_LP0ACTIVE,
} pmc_event_sel;

static int lp0bb_transitions_set(void *data, u64 val)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg;

	if (!val) {
		/* disable event counting and clear counter */
		reg = 0;
		writel(reg, pmc + APBDEV_PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_NONE;
	} else if (val == 1) {
		reg = APBDEV_PMC_EVENT_COUNTER_0_EN_MASK |
				APBDEV_PMC_EVENT_COUNTER_0_LP0BB;
		 /* lp0->lp0bb transitions */
		writel(reg, pmc + APBDEV_PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_LP0BB;
	}
	return 0;
}

static inline unsigned long read_pmc_event_counter(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg = readl(pmc + APBDEV_PMC_EVENT_COUNTER_0);
	/* hw event counter is 16 bit */
	reg = reg & 0xffff;
	return reg;
}

static int lp0bb_transitions_get(void *data, u64 *val)
{
	if (pmc_event_sel == PMC_EVENT_LP0BB)
		*val = (u32) read_pmc_event_counter();
	else
		*val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lp0bb_fops, lp0bb_transitions_get,
		lp0bb_transitions_set, "%lld\n");

static int lp0active_transitions_set(void *data, u64 val)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg;

	if (!val) {
		/* disable event counting and clear counter */
		reg = 0;
		writel(reg, pmc + APBDEV_PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_NONE;
	} else if (val == 1) {
		reg = APBDEV_PMC_EVENT_COUNTER_0_EN_MASK |
				APBDEV_PMC_EVENT_COUNTER_0_LP0ACTIVE;
		 /* lp0->active transitions */
		writel(reg, pmc + APBDEV_PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_LP0ACTIVE;
	}
	return 0;
}
static int lp0active_transitions_get(void *data, u64 *val)
{
	if (pmc_event_sel == PMC_EVENT_LP0ACTIVE)
		*val = (u32) read_pmc_event_counter();
	else
		*val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lp0active_fops, lp0active_transitions_get,
		lp0active_transitions_set, "%lld\n");

static int __init tegra_bb_debug_init(void)
{
	struct dentry *d;

	bb_debugfs_root = debugfs_create_dir("tegra_bb", NULL);
	if (!bb_debugfs_root)
		return -ENOMEM;

	d = debugfs_create_file("lp0bb_transitions", S_IWUSR | S_IRUGO,
			bb_debugfs_root, NULL, &lp0bb_fops);
	if (!d)
		goto err;

	d = debugfs_create_file("lp0active_transitions", S_IWUSR | S_IRUGO,
			bb_debugfs_root, NULL, &lp0active_fops);
	if (!d)
		goto err;

	return 0;
err:
	debugfs_remove_recursive(bb_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_bb_debug_init);
#endif

MODULE_DESCRIPTION("Tegra T148 BB Module");
MODULE_LICENSE("GPL");
