/*
 * arch/arm/mach-tegra/tegra_bb.c
 *
 * Copyright (C) 2012-2013 NVIDIA Corporation.
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

#include <mach/tegra_bb.h>
#include <linux/platform_data/nvshm.h>

#include "iomap.h"

/* BB mailbox offset */
#define TEGRA_BB_REG_MAILBOX (0x0)
/* BB mailbox return code (MOSD) offset */
#define TEGRA_BB_REG_RETCODE (0x4)
/* BB status mask */
#define TEGRA_BB_STATUS_MASK  (0xffff)
#define TEGRA_BB_IPC_COLDBOOT  (0x0001)
#define TEGRA_BB_IPC_READY  (0x0005)

#define APBDEV_PMC_IPC_PMC_IPC_STS_0 (0x500)
#define APBDEV_PMC_IPC_PMC_IPC_STS_0_AP2BB_RESET_SHIFT (1)
#define APBDEV_PMC_IPC_PMC_IPC_STS_0_AP2BB_RESET_DEFAULT_MASK (1)

#define APBDEV_PMC_IPC_PMC_IPC_SET_0 (0x504)
#define APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_RESET_SHIFT (1)
#define APBDEV_PMC_IPC_PMC_IPC_SET_0_AP2BB_MEM_STS_SHIFT (5)

#define APBDEV_PMC_IPC_PMC_IPC_CLEAR_0 (0x508)
#define APBDEV_PMC_IPC_PMC_IPC_CLEAR_0_AP2BB_RESET_SHIFT (1)

#define FLOW_CTLR_IPC_FLOW_IPC_STS_0 (0x500)
#define FLOW_CTLR_IPC_FLOW_IPC_STS_0_BB2AP_INT0_STS_SHIFT (2)

#define FLOW_CTLR_IPC_FLOW_IPC_SET_0 (0x504)
#define FLOW_CTLR_IPC_FLOW_IPC_SET_0_AP2BB_INT0_STS_SHIFT (0)

#define FLOW_CTLR_IPC_FLOW_IPC_CLR_0 (0x508)
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

struct tegra_bb {
	spinlock_t lock;
	int status;
	int instance;
	char name[16];
	char priv_name[16];
	char ipc_name[16];
	unsigned long priv_phy;
	unsigned long ipc_phy;
	void *priv_virt;
	void *ipc_virt;
	void *mb_virt;
	unsigned long priv_size;
	unsigned long ipc_size;
	unsigned long ipc_irq;
	unsigned int irq;
	void (*ipc_cb)(void *data);
	void  *ipc_cb_data;
	struct miscdevice dev_priv;
	struct miscdevice dev_ipc;
	struct device *dev;
	struct sysfs_dirent *sd;
	struct nvshm_platform_data nvshm_pdata;
	struct platform_device nvshm_device;
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
	void __iomem *flow = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
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
	writel(1 << FLOW_CTLR_IPC_FLOW_IPC_SET_0_AP2BB_INT0_STS_SHIFT,
	       flow + FLOW_CTLR_IPC_FLOW_IPC_SET_0);
#endif
}
EXPORT_SYMBOL_GPL(tegra_bb_generate_ipc);

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
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);

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
	} else {
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

static irqreturn_t tegra_bb_isr_handler(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	unsigned long irq_flags;
	int sts;

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

static int tegra_bb_probe(struct platform_device *pdev)
{
	struct tegra_bb *bb;
	int ret;
	struct tegra_bb_platform_data *pdata =
		pdev->dev.platform_data;
	void __iomem *tegra_mc = IO_ADDRESS(TEGRA_MC_BASE);
	unsigned int size, bbc_mem_regions_0;


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

	pr_debug("%s MC_BBC_MEM_REGIONS_0=0x%x\n", __func__, bbc_mem_regions_0);

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

	pr_debug("%s  priv@0x%lx/0x%lx\n", __func__,
		 (unsigned long)bb->priv_phy,
		bb->priv_size);

	pr_debug("%s  ipc@0x%lx/0x%lx\n", __func__,
		 (unsigned long)bb->ipc_phy,
		bb->ipc_size);

	if (!(bb->ipc_size && bb->priv_size)) {
		pr_err("%s: invalid tegra_bb regions\n", __func__);
		BUG();
	}

	bb->irq = pdata->bb_irq;

	pdata->bb_handle = bb;

	/* Map mb_virt uncached (first 4K of IPC) */
	bb->mb_virt = ioremap_nocache(bb->ipc_phy,
					      SZ_1K*4);

	/* Private is uncached */
	bb->priv_virt =  ioremap_nocache(bb->priv_phy,
					bb->priv_size);
	pr_debug("%s: Priv Virtual=0x%x\n", __func__, bb->priv_virt);

	/* IPC memory is cached */
	bb->ipc_virt =  ioremap_cached(bb->ipc_phy,
					bb->ipc_size);
	pr_debug("%s: IPC Virtual=0x%x\n", __func__, bb->ipc_virt);

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

	bb->sd = sysfs_get_dirent(pdev->dev.kobj.sd, NULL, "status");

	bb->nvshm_pdata.ipc_base_virt = bb->ipc_virt;
	bb->nvshm_pdata.ipc_size = bb->ipc_size;
	bb->nvshm_pdata.mb_base_virt = bb->mb_virt;
	bb->nvshm_pdata.mb_size = 4*SZ_1K;

	bb->nvshm_pdata.tegra_bb = pdev;
	bb->nvshm_device.name = "nvshm";
	bb->nvshm_device.id = bb->instance;
	bb->nvshm_device.dev.platform_data = &bb->nvshm_pdata;
	platform_device_register(&bb->nvshm_device);
	platform_set_drvdata(pdev, bb);

#ifndef CONFIG_TEGRA_BASEBAND_SIMU
	snprintf(bb->name, sizeof(bb->name), "tegra_bb%d", pdev->id);

	ret = request_irq(bb->irq, tegra_bb_isr_handler, 0, bb->name, bb);
	if (ret) {
		dev_err(&pdev->dev, "Could not register irq handler\n");
		kfree(bb);
		return -EAGAIN;
	}
#endif
	return 0;
}

#ifdef CONFIG_PM
static int tegra_bb_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);
	return 0;
}

static int tegra_bb_resume(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);
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

MODULE_DESCRIPTION("Tegra T148 BB Module");
MODULE_LICENSE("GPL");
