/*
 * IOMMU driver for SMMU on Tegra 3 series SoCs and later.
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/tegra-ahb.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/tegra-ahb.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <linux/tegra-soc.h>
#include <linux/tegra_smmu.h>
#include <linux/pci.h>

#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <asm/dma-iommu.h>

#include <dt-bindings/memory/tegra-swgroup.h>

/* HACK! This needs to come from device tree */
#include "../../arch/arm/mach-tegra/iomap.h"

#define CREATE_TRACE_POINTS
#include <trace/events/tegra_smmu.h>

#include "of_tegra-smmu.h"

/* bitmap of the page sizes currently supported */
#define SMMU_IOMMU_PGSIZES	(SZ_4K | SZ_4M)

#define SMMU_CONFIG				0x10
#define SMMU_CONFIG_DISABLE			0
#define SMMU_CONFIG_ENABLE			1

enum {
	_TLB = 0,
	_PTC,
};

#define SMMU_CACHE_CONFIG_BASE			0x14
#define __SMMU_CACHE_CONFIG(mc, cache)		(SMMU_CACHE_CONFIG_BASE + 4 * cache)
#define SMMU_CACHE_CONFIG(cache)		__SMMU_CACHE_CONFIG(_MC, cache)

#define SMMU_CACHE_CONFIG_STATS_SHIFT		31
#define SMMU_CACHE_CONFIG_STATS_ENABLE		(1 << SMMU_CACHE_CONFIG_STATS_SHIFT)
#define SMMU_CACHE_CONFIG_STATS_TEST_SHIFT	30
#define SMMU_CACHE_CONFIG_STATS_TEST		(1 << SMMU_CACHE_CONFIG_STATS_TEST_SHIFT)

#define SMMU_TLB_CONFIG_HIT_UNDER_MISS__ENABLE	(1 << 29)
#define SMMU_TLB_CONFIG_ACTIVE_LINES__VALUE	0x10
#define SMMU_TLB_CONFIG_RESET_VAL		0x30000000

#define SMMU_PTC_CONFIG_CACHE__ENABLE		(1 << 29)
#define SMMU_PTC_CONFIG_INDEX_MAP__PATTERN	0x3f
#define SMMU_PTC_CONFIG_RESET_VAL		0x2000003f
#define SMMU_PTC_REQ_LIMIT			(8 << 24)

#define SMMU_PTB_ASID				0x1c
#define SMMU_PTB_ASID_CURRENT_SHIFT		0

#define SMMU_PTB_DATA				0x20
#define SMMU_PTB_DATA_RESET_VAL			0
#define SMMU_PTB_DATA_ASID_NONSECURE_SHIFT	29
#define SMMU_PTB_DATA_ASID_WRITABLE_SHIFT	30
#define SMMU_PTB_DATA_ASID_READABLE_SHIFT	31

#define SMMU_TLB_FLUSH				0x30
#define SMMU_TLB_FLUSH_VA_MATCH_ALL		0
#define SMMU_TLB_FLUSH_VA_MATCH_SECTION		2
#define SMMU_TLB_FLUSH_VA_MATCH_GROUP		3
#define SMMU_TLB_FLUSH_ASID_SHIFT_BASE		31
#define SMMU_TLB_FLUSH_ASID_MATCH_DISABLE	0
#define SMMU_TLB_FLUSH_ASID_MATCH_ENABLE	1
#define SMMU_TLB_FLUSH_ASID_MATCH_SHIFT		31
#define SMMU_TLB_FLUSH_ASID_ENABLE					\
	(SMMU_TLB_FLUSH_ASID_MATCH_ENABLE << SMMU_TLB_FLUSH_ASID_MATCH_SHIFT)

#define SMMU_TLB_FLUSH_ASID_SHIFT(as)		\
	(SMMU_TLB_FLUSH_ASID_SHIFT_BASE -	\
				__ffs((as)->smmu->chip_data->num_asids))
#define SMMU_ASID_MASK	((1 << __ffs((as)->smmu->chip_data->num_asids)) - 1)

#define SMMU_PTC_FLUSH				0x34
#define SMMU_PTC_FLUSH_TYPE_ALL			0
#define SMMU_PTC_FLUSH_TYPE_ADR			1
#define SMMU_PTC_FLUSH_ADR_MASK			0xfffffff0

#define SMMU_PTC_FLUSH_1			0x9b8

#define SMMU_ASID_SECURITY			0x38
#define SMMU_ASID_SECURITY_1			0x3c
#define SMMU_ASID_SECURITY_2			0x9e0
#define SMMU_ASID_SECURITY_3			0x9e4
#define SMMU_ASID_SECURITY_4			0x9e8
#define SMMU_ASID_SECURITY_5			0x9ec
#define SMMU_ASID_SECURITY_6			0x9f0
#define SMMU_ASID_SECURITY_7			0x9f4

#define SMMU_STATS_CACHE_COUNT_BASE		0x1f0

#define SMMU_STATS_CACHE_COUNT(mc, cache, hitmiss)		\
	(SMMU_STATS_CACHE_COUNT_BASE + 8 * cache + 4 * hitmiss)

#define SMMU_TRANSLATION_ENABLE_0		0x228
#define SMMU_TRANSLATION_ENABLE_4		0xb98

#define HWGRP_COUNT	64

#define SMMU_PDE_NEXT_SHIFT		28

/* AHB Arbiter Registers */
#define AHB_XBAR_CTRL				0xe0
#define AHB_XBAR_CTRL_SMMU_INIT_DONE_DONE	1
#define AHB_XBAR_CTRL_SMMU_INIT_DONE_SHIFT	17
#define AHB_MASTER_SWID_0			0x18
#define AHB_MASTER_SELECT_SDMMC		(BIT(9) | BIT(12) \
						| BIT(19) | BIT(20))

#define SMMU_NUM_ASIDS				4
#define SMMU_NUM_ASIDS_TEGRA12			128
#define SMMU_TLB_FLUSH_VA_SECTION__MASK		0xffc00000
#define SMMU_TLB_FLUSH_VA_SECTION__SHIFT	12 /* right shift */
#define SMMU_TLB_FLUSH_VA_GROUP__MASK		0xffffc000
#define SMMU_TLB_FLUSH_VA_GROUP__SHIFT		12 /* right shift */
#define SMMU_TLB_FLUSH_VA(iova, which)	\
	((((iova) & SMMU_TLB_FLUSH_VA_##which##__MASK) >> \
		SMMU_TLB_FLUSH_VA_##which##__SHIFT) |	\
	SMMU_TLB_FLUSH_VA_MATCH_##which)
#define SMMU_PTB_ASID_CUR(n)	\
		((n) << SMMU_PTB_ASID_CURRENT_SHIFT)

#define SMMU_TLB_FLUSH_ALL 0

#define SMMU_TLB_FLUSH_ASID_MATCH_disable		\
		(SMMU_TLB_FLUSH_ASID_MATCH_DISABLE <<	\
			SMMU_TLB_FLUSH_ASID_MATCH_SHIFT)
#define SMMU_TLB_FLUSH_ASID_MATCH__ENABLE		\
		(SMMU_TLB_FLUSH_ASID_MATCH_ENABLE <<	\
			SMMU_TLB_FLUSH_ASID_MATCH_SHIFT)

#define SMMU_PAGE_SHIFT 12
#define SMMU_PAGE_SIZE	(1 << SMMU_PAGE_SHIFT)

#define SMMU_PDIR_COUNT	1024
#define SMMU_PDIR_SIZE	(sizeof(u32) * SMMU_PDIR_COUNT)
#define SMMU_PTBL_COUNT	1024
#define SMMU_PTBL_SIZE	(sizeof(u32) * SMMU_PTBL_COUNT)
#define SMMU_PDIR_SHIFT	12
#define SMMU_PDE_SHIFT	12
#define SMMU_PTE_SHIFT	12
#define SMMU_PFN_MASK	0x0fffffff

#define SMMU_ADDR_TO_PTN(addr)	((((u32)addr) >> 12) & (BIT(10) - 1))
#define SMMU_ADDR_TO_PDN(addr)	(((u32)addr) >> 22)
#define SMMU_PDN_TO_ADDR(pdn)	((pdn) << 22)

#define _READABLE	(1 << SMMU_PTB_DATA_ASID_READABLE_SHIFT)
#define _WRITABLE	(1 << SMMU_PTB_DATA_ASID_WRITABLE_SHIFT)
#define _NONSECURE	(1 << SMMU_PTB_DATA_ASID_NONSECURE_SHIFT)
#define _PDE_NEXT	(1 << SMMU_PDE_NEXT_SHIFT)
#define _MASK_ATTR	(_READABLE | _WRITABLE | _NONSECURE)

#define _PDIR_ATTR	(_READABLE | _WRITABLE | _NONSECURE)

#define _PDE_ATTR	(_READABLE | _WRITABLE | _NONSECURE)
#define _PDE_ATTR_N	(_PDE_ATTR | _PDE_NEXT)
#define _PDE_VACANT(pdn)	(0)

#define _PTE_ATTR	(_READABLE | _WRITABLE | _NONSECURE)
#define _PTE_VACANT(addr)	(0)

#ifdef	CONFIG_TEGRA_IOMMU_SMMU_LINEAR
#undef	_PDE_VACANT
#undef	_PTE_VACANT
#define	_PDE_VACANT(pdn)	(((pdn) << 10) | _PDE_ATTR)
#define	_PTE_VACANT(addr)	((((u32)addr) >> SMMU_PAGE_SHIFT) | _PTE_ATTR)
#endif

#define SMMU_MK_PDIR(page, attr)	\
		((page_to_phys(page) >> SMMU_PDIR_SHIFT) | (attr))
#define SMMU_MK_PDE(page, attr)		\
		(u32)((page_to_phys(page) >> SMMU_PDE_SHIFT) | (attr))
#define SMMU_EX_PTBL_PAGE(pde)		\
		pfn_to_page((u32)(pde) & SMMU_PFN_MASK)
#define SMMU_PFN_TO_PTE(pfn, attr)	(u32)((pfn) | (attr))

#define SMMU_ASID_ENABLE(asid, idx)   (((asid) << (idx * 8)) | (1 << 31))
#define SMMU_ASID_DISABLE	0
#define MAX_AS_PER_DEV 4
#define SMMU_ASID_GET_IDX(iova) (int)(((u64)iova >> 32) & (MAX_AS_PER_DEV - 1))

#define SMMU_CLIENT_CONF0	0x40

#define smmu_client_enable_hwgrp(c, m)	smmu_client_set_hwgrp(c, m, 1)
#define smmu_client_disable_hwgrp(c)	smmu_client_set_hwgrp(c, 0, 0)
#define __smmu_client_enable_hwgrp(c, m) __smmu_client_set_hwgrp(c, m, 1)
#define __smmu_client_disable_hwgrp(c)	__smmu_client_set_hwgrp(c, 0, 0)

static struct device *save_smmu_device;

static size_t smmu_flush_all_th_pages = SZ_512; /* number of threshold pages */

enum {
	IOMMMS_PROPS_TEGRA_SWGROUP_BIT_HI = 0,
	IOMMMS_PROPS_TEGRA_SWGROUP_BIT_LO = 1,
	IOMMUS_PROPS_AS = 2,
};

static const u32 smmu_asid_security_ofs[] = {
	SMMU_ASID_SECURITY,
	SMMU_ASID_SECURITY_1,
	SMMU_ASID_SECURITY_2,
	SMMU_ASID_SECURITY_3,
	SMMU_ASID_SECURITY_4,
	SMMU_ASID_SECURITY_5,
	SMMU_ASID_SECURITY_6,
	SMMU_ASID_SECURITY_7,
};

struct tegra_smmu_chip_data {
	int num_asids;
};

struct smmu_domain {
	struct smmu_as *as[MAX_AS_PER_DEV];
	unsigned long bitmap[1];
};

/*
 * Per client for address space
 */
struct smmu_client {
	struct rb_node		node;
	struct device		*dev;
	struct list_head	list;
	struct smmu_domain	*domain;
	u64			swgids;

	struct smmu_map_prop	*prop;

	struct dentry		*debugfs_root;
	struct dentry		*as_link[MAX_AS_PER_DEV];
};

/*
 * Per address space
 */
struct smmu_as {
	struct smmu_device	*smmu;	/* back pointer to container */
	unsigned int		asid;
	spinlock_t		lock;	/* for pagetable */
	struct page		*pdir_page;
	u32			pdir_attr;
	u32			pde_attr;
	u32			pte_attr;
	unsigned int		*pte_count;

	struct list_head	client;
	spinlock_t		client_lock; /* for client list */

	struct dentry		*debugfs_root;
};

struct smmu_debugfs_info {
	struct smmu_device *smmu;
	int mc;
	int cache;
};

/*
 * Per SMMU device - IOMMU device
 */
struct smmu_device {
	void __iomem	*regs, *regs_ahbarb;
	unsigned long	iovmm_base;	/* remappable base address */
	unsigned long	page_count;	/* total remappable size */
	spinlock_t	lock;
	char		*name;
	struct device	*dev;
	u64		swgids;		/* memory client ID bitmap */
	u32		ptc_cache_size;	/* u32 is ok as ptc_cache_size < 4 GB */

	struct rb_root	clients;

	struct page *avp_vector_page;	/* dummy page shared by all AS's */

	/*
	 * Register image savers for suspend/resume
	 */
	u32 num_translation_enable;
	u32 translation_enable[5];
	u32 num_asid_security;
	u32 asid_security[8];

	struct dentry *debugfs_root;
	struct smmu_debugfs_info *debugfs_info;
	struct dentry *masters_root;

	const struct tegra_smmu_chip_data *chip_data;
	struct list_head asprops;

	u32		num_as;
	struct smmu_as	as[0];		/* Run-time allocated array */
};

static struct smmu_as *domain_to_as(struct iommu_domain *_domain,
						unsigned long iova)
{
	struct smmu_domain *domain = _domain->priv;
	int idx;

	BUG_ON(iova != -1 && !domain->bitmap[0]);
	if (iova == -1 && !domain->bitmap[0])
		return NULL;

	if (iova == -1)
		idx = __ffs(domain->bitmap[0]);
	else
		idx = SMMU_ASID_GET_IDX(iova);

	BUG_ON(idx >= MAX_AS_PER_DEV);
	return domain->as[idx];
}

static void dma_map_to_as_bitmap(struct dma_iommu_mapping *map,
				unsigned long *bitmap)
{
	int start_idx, end_idx;

	start_idx = (int) ((u64)map->base >> 32);
	end_idx = (int) ((u64)map->end >> 32);
	BUG_ON(end_idx >= MAX_AS_PER_DEV);

	*bitmap = 0;
	while (start_idx <= end_idx) {
		set_bit(start_idx, bitmap);
		start_idx++;
	}
}

#define smmu_as_bitmap(domain) \
	(domain->as[__ffs(domain->bitmap[0])])

static struct smmu_device *smmu_handle; /* unique for a system */

/*
 *	SMMU/AHB register accessors
 */
static inline u32 smmu_read(struct smmu_device *smmu, size_t offs)
{
	return readl(smmu->regs + offs);
}
static inline void smmu_write(struct smmu_device *smmu, u32 val, size_t offs)
{
	writel(val, smmu->regs + offs);
}

static inline u32 ahb_read(struct smmu_device *smmu, size_t offs)
{
	return readl(smmu->regs_ahbarb + offs);
}
static inline void ahb_write(struct smmu_device *smmu, u32 val, size_t offs)
{
	writel(val, smmu->regs_ahbarb + offs);
}

static void __smmu_client_ordered(struct smmu_device *smmu, int id)
{
	size_t offs;
	u32 val;

	offs = SMMU_CLIENT_CONF0;
	offs += (id / BITS_PER_LONG) * sizeof(u32);

	val = smmu_read(smmu, offs);
	val |= BIT(id % BITS_PER_LONG);
	smmu_write(smmu, val, offs);
}

static void smmu_client_ordered(struct smmu_device *smmu)
{
	int i, id[] = {
		/* Add client ID here to be ordered */
	};

	for (i = 0; i < ARRAY_SIZE(id); i++)
		__smmu_client_ordered(smmu, id[i]);
}

#define VA_PAGE_TO_PA(va, page)	\
	(page_to_phys(page) + ((unsigned long)(va) & ~PAGE_MASK))

#define VA_PAGE_TO_PA_HI(va, page)	\
	(u32)((u64)(page_to_phys(page)) >> 32)

#define FLUSH_CPU_DCACHE(va, page, size)	\
	do {	\
		unsigned long _pa_ = VA_PAGE_TO_PA(va, page);		\
		FLUSH_DCACHE_AREA((void *)(va), (size_t)(size));	\
		outer_flush_range(_pa_, _pa_+(size_t)(size));		\
	} while (0)

/*
 * Any interaction between any block on PPSB and a block on APB or AHB
 * must have these read-back barriers to ensure the APB/AHB bus
 * transaction is complete before initiating activity on the PPSB
 * block.
 */
#define FLUSH_SMMU_REGS(smmu)	smmu_read(smmu, SMMU_CONFIG)

static struct of_device_id tegra_smmu_of_match[];

static struct smmu_client *tegra_smmu_find_client(struct smmu_device *smmu,
						  struct device *dev)
{
	struct rb_node *node = smmu->clients.rb_node;

	while (node) {
		struct smmu_client *client;

		client = container_of(node, struct smmu_client, node);
		if (dev < client->dev)
			node = node->rb_left;
		else if (dev > client->dev)
			node = node->rb_right;
		else
			return client;
	}

	return NULL;
}

static int tegra_smmu_insert_client(struct smmu_device *smmu,
				    struct smmu_client *client)
{
	struct rb_node **new, *parent;

	new = &smmu->clients.rb_node;
	parent = NULL;
	while (*new) {
		struct smmu_client *this;

		this = container_of(*new, struct smmu_client, node);
		parent = *new;
		if (client->dev < this->dev)
			new = &((*new)->rb_left);
		else if (client->dev > this->dev)
			new = &((*new)->rb_right);
		else
			return -EEXIST;
	}

	rb_link_node(&client->node, parent, new);
	rb_insert_color(&client->node, &smmu->clients);
	return 0;
}

static struct smmu_client *tegra_smmu_register_client(struct smmu_device *smmu,
					      struct device *dev, u64 swgids,
					      struct smmu_map_prop *prop)
{
	int err;
	struct smmu_client *client;

	client = tegra_smmu_find_client(smmu, dev);
	if (client) {
		dev_err(dev,
			"rejecting multiple registrations for client device %s\n",
			dev->of_node ? dev->of_node->full_name : "<no device node>");
		return NULL;
	}

	client = devm_kzalloc(smmu->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	client->dev = dev;
	client->swgids = swgids;
	client->prop = prop;

	err = tegra_smmu_insert_client(smmu, client);
	if (err) {
		devm_kfree(smmu->dev, client);
		return NULL;
	}

	return client;
}

static u64 tegra_smmu_get_swgids(struct device *dev)
{
	u64 swgids = SWGIDS_ERROR_CODE;
	struct smmu_client *client;
	struct iommu_linear_map *area = NULL;
	struct smmu_map_prop *prop;

	if (!smmu_handle) {
		dev_info(dev, "SMMU isn't ready yet\n");
		return SWGIDS_ERROR_CODE;
	}

	client = tegra_smmu_find_client(smmu_handle, dev);
	if (client)
		return client->swgids;

	swgids = tegra_smmu_of_get_swgids(dev, tegra_smmu_of_match, &area);
	if (swgids_is_error(swgids))
		return SWGIDS_ERROR_CODE;

	list_for_each_entry(prop, &smmu_handle->asprops, list)
		if (swgids & prop->swgid_mask)
			goto found;

	dev_err(dev, "Unable to retrieve as prop for swgids:%lld\n", swgids);
	return SWGIDS_ERROR_CODE;

found:
	prop->area = area;

	client = tegra_smmu_register_client(smmu_handle, dev, swgids, prop);
	if (!client)
		swgids = SWGIDS_ERROR_CODE;

	return swgids;
}

static int __smmu_client_set_hwgrp(struct smmu_client *c, u64 map, int on)
{
	int i;
	struct smmu_domain *dom = c->domain;
	struct smmu_as *as = smmu_as_bitmap(dom);
	u32 val, offs, mask = 0;
	struct smmu_device *smmu = as->smmu;

	WARN_ON(!on && map);
	if (on && !map)
		return -EINVAL;
	if (!on)
		map = c->swgids;

	for_each_set_bit(i, (unsigned long *)&(dom->bitmap),
				MAX_AS_PER_DEV)
		mask |= SMMU_ASID_ENABLE(dom->as[i]->asid, i);

	for_each_set_bit(i, (unsigned long *)&map, HWGRP_COUNT) {
		offs = tegra_smmu_of_offset(i);
		val = smmu_read(smmu, offs);
		val &= ~SMMU_ASID_MASK; /* always overwrite ASID */

		if (on)
			val |= mask;
		else if (list_empty(&c->list))
			val = 0; /* turn off if this is the last */
		else
			return 0; /* leave if off but not the last */

		smmu_write(smmu, val, offs);

		dev_dbg(c->dev, "swgid:%d asid:%d %s @%s\n",
			i, val & SMMU_ASID_MASK,
			 (val & BIT(31)) ? "Enabled" : "Disabled", __func__);
	}
	FLUSH_SMMU_REGS(smmu);
	c->swgids = map;
	return 0;

}

static int smmu_client_set_hwgrp(struct smmu_client *c, u64 map, int on)
{
	u32 val;
	unsigned long flags;
	struct smmu_domain *dom = c->domain;
	struct smmu_as *as =  smmu_as_bitmap(dom);
	struct smmu_device *smmu = as->smmu;

	spin_lock_irqsave(&smmu->lock, flags);
	val = __smmu_client_set_hwgrp(c, map, on);
	spin_unlock_irqrestore(&smmu->lock, flags);
	return val;
}

/*
 * Flush all TLB entries and all PTC entries
 * Caller must lock smmu
 */
static void smmu_flush_regs(struct smmu_device *smmu, int enable)
{
	u32 val;

	smmu_write(smmu, SMMU_PTC_FLUSH_TYPE_ALL, SMMU_PTC_FLUSH);
	FLUSH_SMMU_REGS(smmu);
	val = SMMU_TLB_FLUSH_VA_MATCH_ALL |
		SMMU_TLB_FLUSH_ASID_MATCH_disable;
	smmu_write(smmu, val, SMMU_TLB_FLUSH);

	if (enable)
		smmu_write(smmu, SMMU_CONFIG_ENABLE, SMMU_CONFIG);
	FLUSH_SMMU_REGS(smmu);
}

static int translation_enable_offset(int i)
{
	if (i < 4)
		return SMMU_TRANSLATION_ENABLE_0 + i * sizeof(u32);
	else if (i == 4)
		return SMMU_TRANSLATION_ENABLE_4;
	else
		BUG();
}

static void smmu_setup_regs(struct smmu_device *smmu)
{
	int i;
	u32 val;

	for (i = 0; i < smmu->num_as; i++) {
		struct smmu_as *as = &smmu->as[i];
		struct smmu_client *c;

		smmu_write(smmu, SMMU_PTB_ASID_CUR(as->asid), SMMU_PTB_ASID);
		val = as->pdir_page ?
			SMMU_MK_PDIR(as->pdir_page, as->pdir_attr) :
			SMMU_PTB_DATA_RESET_VAL;
		smmu_write(smmu, val, SMMU_PTB_DATA);

		list_for_each_entry(c, &as->client, list)
			__smmu_client_set_hwgrp(c, c->swgids, 1);
	}

	for (i = 0; i < smmu->num_translation_enable; i++)
		smmu_write(smmu, smmu->translation_enable[i],
			   translation_enable_offset(i));

	for (i = 0; i < smmu->num_asid_security; i++)
		smmu_write(smmu,
			   smmu->asid_security[i], smmu_asid_security_ofs[i]);

	val = SMMU_PTC_CONFIG_RESET_VAL;
	val |= SMMU_PTC_REQ_LIMIT;

	smmu_write(smmu, val, SMMU_CACHE_CONFIG(_PTC));

	val = SMMU_TLB_CONFIG_RESET_VAL;
	if ((IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC) &&
	     (tegra_get_chipid() == TEGRA_CHIPID_TEGRA12)) ||
	    (IS_ENABLED(CONFIG_ARCH_TEGRA_13x_SOC) &&
	     (tegra_get_chipid() == TEGRA_CHIPID_TEGRA13)))
		val |= SMMU_TLB_CONFIG_ACTIVE_LINES__VALUE << 1;
	else  /* T210. */
		val |= (SMMU_TLB_CONFIG_ACTIVE_LINES__VALUE * 3);

	smmu_write(smmu, val, SMMU_CACHE_CONFIG(_TLB));

	smmu_client_ordered(smmu);
	smmu_flush_regs(smmu, 1);

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3
			|| tegra_get_chipid() == TEGRA_CHIPID_TEGRA11
			|| tegra_get_chipid() == TEGRA_CHIPID_TEGRA14) {
		val = ahb_read(smmu, AHB_XBAR_CTRL);
		val |= AHB_XBAR_CTRL_SMMU_INIT_DONE_DONE <<
			AHB_XBAR_CTRL_SMMU_INIT_DONE_SHIFT;
		ahb_write(smmu, val, AHB_XBAR_CTRL);
	}
	/* On T114, Set PPCS1 ASID for SDMMC */
	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA11)
		ahb_write(smmu, AHB_MASTER_SELECT_SDMMC, AHB_MASTER_SWID_0);
}


static void __smmu_flush_ptc(struct smmu_device *smmu, u32 *pte,
			     struct page *page)
{
	u32 val;

	if (!pte) {
		smmu_write(smmu, SMMU_PTC_FLUSH_TYPE_ALL, SMMU_PTC_FLUSH);
		return;
	}

	val = VA_PAGE_TO_PA_HI(pte, page);
	smmu_write(smmu, val, SMMU_PTC_FLUSH_1);
	val = SMMU_PTC_FLUSH_TYPE_ADR |
		(VA_PAGE_TO_PA(pte, page) & SMMU_PTC_FLUSH_ADR_MASK);

	smmu_write(smmu, val, SMMU_PTC_FLUSH);
}

static void smmu_flush_ptc(struct smmu_device *smmu, u32 *pte,
			   struct page *page)
{
	__smmu_flush_ptc(smmu, pte, page);
	FLUSH_SMMU_REGS(smmu);
}

static inline void __smmu_flush_ptc_all(struct smmu_device *smmu)
{
	__smmu_flush_ptc(smmu, 0, NULL);
}

static void __smmu_flush_tlb(struct smmu_device *smmu, struct smmu_as *as,
			   dma_addr_t iova, int is_pde)
{
	u32 val;

	if (is_pde)
		val = SMMU_TLB_FLUSH_VA(iova, SECTION);
	else
		val = SMMU_TLB_FLUSH_VA(iova, GROUP);

	smmu_write(smmu, val, SMMU_TLB_FLUSH);
}

static inline void __smmu_flush_tlb_section(struct smmu_as *as, dma_addr_t iova)
{
	__smmu_flush_tlb(as->smmu, as, iova, 1);
}

static void flush_ptc_and_tlb(struct smmu_device *smmu,
			      struct smmu_as *as, dma_addr_t iova,
			      u32 *pte, struct page *page, int is_pde)
{
	__smmu_flush_ptc(smmu, pte, page);
	__smmu_flush_tlb(smmu, as, iova, is_pde);
	FLUSH_SMMU_REGS(smmu);
}

#ifdef CONFIG_TEGRA_ERRATA_1053704
/* Flush PTEs within the same L2 pagetable */
static void ____smmu_flush_tlb_range(struct smmu_device *smmu, dma_addr_t iova,
				   dma_addr_t end)
{
	size_t unit = SZ_16K;

	iova = round_down(iova, unit);
	while (iova < end) {
		u32 val;

		val = SMMU_TLB_FLUSH_VA(iova, GROUP);
		smmu_write(smmu, val, SMMU_TLB_FLUSH);
		iova += unit;
	}
}
#endif

static void flush_ptc_and_tlb_range(struct smmu_device *smmu,
				    struct smmu_as *as, dma_addr_t iova,
				    u32 *pte, struct page *page,
				    size_t count)
{
	size_t unit = SZ_16K;
	dma_addr_t end = iova + count * PAGE_SIZE;

	iova = round_down(iova, unit);
	while (iova < end) {
		int i;

		__smmu_flush_ptc(smmu, pte, page);
		pte += smmu->ptc_cache_size / PAGE_SIZE;

		for (i = 0; i < smmu->ptc_cache_size / unit; i++) {
			u32 val;

			val = SMMU_TLB_FLUSH_VA(iova, GROUP);
			smmu_write(smmu, val, SMMU_TLB_FLUSH);
			iova += unit;
		}
	}

	FLUSH_SMMU_REGS(smmu);
}

static inline void flush_ptc_and_tlb_all(struct smmu_device *smmu,
					 struct smmu_as *as)
{
	flush_ptc_and_tlb(smmu, as, 0, 0, NULL, 1);
}

static void free_ptbl(struct smmu_as *as, dma_addr_t iova, bool flush)
{
	int pdn = SMMU_ADDR_TO_PDN(iova);
	u32 *pdir = (u32 *)page_address(as->pdir_page);

	if (pdir[pdn] != _PDE_VACANT(pdn)) {
		dev_dbg(as->smmu->dev, "pdn: %x\n", pdn);

		__free_page(SMMU_EX_PTBL_PAGE(pdir[pdn]));
		pdir[pdn] = _PDE_VACANT(pdn);
		FLUSH_CPU_DCACHE(&pdir[pdn], as->pdir_page, sizeof pdir[pdn]);
		if (!flush)
			return;

		flush_ptc_and_tlb(as->smmu, as, iova, &pdir[pdn],
				  as->pdir_page, 1);
	}
}

#ifdef CONFIG_TEGRA_ERRATA_1053704
static void __smmu_flush_tlb_range(struct smmu_as *as, dma_addr_t iova,
				 dma_addr_t end)
{
	u32 *pdir;
	struct smmu_device *smmu = as->smmu;

	if (!pfn_valid(page_to_pfn(as->pdir_page)))
		return;

	pdir = page_address(as->pdir_page);
	while (iova < end) {
		int pdn = SMMU_ADDR_TO_PDN(iova);

		if (pdir[pdn] & _PDE_NEXT) {
			struct page *page = SMMU_EX_PTBL_PAGE(pdir[pdn]);
			dma_addr_t _end = min_t(dma_addr_t, end,
						SMMU_PDN_TO_ADDR(pdn + 1));

			if (pfn_valid(page_to_pfn(page)))
				____smmu_flush_tlb_range(smmu, iova, _end);

			iova = _end;
		} else {
			if (pdir[pdn])
				__smmu_flush_tlb_section(as, iova);

			iova = SMMU_PDN_TO_ADDR(pdn + 1);
		}

		if (pdn == SMMU_PTBL_COUNT - 1)
			break;
	}
}

static void __smmu_flush_tlb_as(struct smmu_as *as, dma_addr_t iova,
			      dma_addr_t end)
{
	__smmu_flush_tlb_range(as, iova, end);
}
#else
static void __smmu_flush_tlb_as(struct smmu_as *as, dma_addr_t iova,
			      dma_addr_t end)
{
	u32 val;
	struct smmu_device *smmu = as->smmu;

	val = SMMU_TLB_FLUSH_ASID_ENABLE |
		(as->asid << SMMU_TLB_FLUSH_ASID_SHIFT(as));
	smmu_write(smmu, val, SMMU_TLB_FLUSH);
}
#endif

static void flush_ptc_and_tlb_as(struct smmu_as *as, dma_addr_t start,
				 dma_addr_t end)
{
	struct smmu_device *smmu = as->smmu;

	__smmu_flush_ptc_all(smmu);
	__smmu_flush_tlb_as(as, start, end);
	FLUSH_SMMU_REGS(smmu);
}

static void free_pdir(struct smmu_as *as)
{
	unsigned long addr;
	int count;
	struct device *dev = as->smmu->dev;

	if (!as->pdir_page)
		return;

	addr = as->smmu->iovmm_base;
	count = as->smmu->page_count;
	while (count-- > 0) {
		free_ptbl(as, addr, 1);
		addr += SMMU_PAGE_SIZE * SMMU_PTBL_COUNT;
	}
	__free_page(as->pdir_page);
	as->pdir_page = NULL;
	devm_kfree(dev, as->pte_count);
	as->pte_count = NULL;
}

static struct page *alloc_ptbl(struct smmu_as *as, dma_addr_t iova, bool flush)
{
	int i;
	u32 *pdir = page_address(as->pdir_page);
	int pdn = SMMU_ADDR_TO_PDN(iova);
	unsigned long addr = SMMU_PDN_TO_ADDR(pdn);
	struct page *page;
	u32 *ptbl;
	gfp_t gfp = GFP_ATOMIC;

	if (IS_ENABLED(CONFIG_PREEMPT) && !in_atomic())
		gfp = GFP_KERNEL;

	if (!IS_ENABLED(CONFIG_TEGRA_IOMMU_SMMU_LINEAR))
		gfp |= __GFP_ZERO;

	/* Vacant - allocate a new page table */
	dev_dbg(as->smmu->dev, "New PTBL pdn: %x\n", pdn);

	page = alloc_page(gfp);
	if (!page)
		return NULL;

	ptbl = (u32 *)page_address(page);
	if (IS_ENABLED(CONFIG_TEGRA_IOMMU_SMMU_LINEAR)) {
		for (i = 0; i < SMMU_PTBL_COUNT; i++) {
			ptbl[i] = _PTE_VACANT(addr);
			addr += SMMU_PAGE_SIZE;
		}
	}

	FLUSH_CPU_DCACHE(ptbl, page, SMMU_PTBL_SIZE);
	pdir[pdn] = SMMU_MK_PDE(page, as->pde_attr | _PDE_NEXT);
	FLUSH_CPU_DCACHE(&pdir[pdn], as->pdir_page, sizeof pdir[pdn]);
	if (flush)
		flush_ptc_and_tlb(as->smmu, as, iova, &pdir[pdn],
				  as->pdir_page, 1);
	return page;
}

/*
 * Maps PTBL for given iova and returns the PTE address
 * Caller must unmap the mapped PTBL returned in *ptbl_page_p
 */
static u32 *locate_pte(struct smmu_as *as,
				 dma_addr_t iova, bool allocate,
				 struct page **ptbl_page_p,
				 unsigned int **count)
{
	int ptn = SMMU_ADDR_TO_PTN(iova);
	int pdn = SMMU_ADDR_TO_PDN(iova);
	u32 *pdir = page_address(as->pdir_page);
	u32 *ptbl;

	if (pdir[pdn] != _PDE_VACANT(pdn)) {
		/* Mapped entry table already exists */
		*ptbl_page_p = SMMU_EX_PTBL_PAGE(pdir[pdn]);
	} else if (!allocate) {
		return NULL;
	} else {
		*ptbl_page_p = alloc_ptbl(as, iova, 1);
		if (!*ptbl_page_p)
			return NULL;
	}

	ptbl = page_address(*ptbl_page_p);
	*count = &as->pte_count[pdn];
	return &ptbl[ptn];
}

#ifdef CONFIG_SMMU_SIG_DEBUG
static void put_signature(struct smmu_as *as,
			  dma_addr_t iova, unsigned long pfn)
{
	struct page *page;
	u32 *vaddr;

	page = pfn_to_page(pfn);
	vaddr = page_address(page);
	if (!vaddr)
		return;

	vaddr[0] = iova;
	vaddr[1] = pfn << PAGE_SHIFT;
	FLUSH_CPU_DCACHE(vaddr, page, sizeof(vaddr[0]) * 2);
}
#else
static inline void put_signature(struct smmu_as *as,
				 unsigned long addr, unsigned long pfn)
{
}
#endif

/*
 * Caller must not hold as->lock
 */
static int alloc_pdir(struct smmu_as *as)
{
	u32 *pdir;
	unsigned long flags;
	int pdn, err = 0;
	u32 val;
	struct smmu_device *smmu = as->smmu;
	struct page *page;
	unsigned int *cnt;

	/*
	 * do the allocation, then grab as->lock
	 */
	cnt = devm_kzalloc(smmu->dev,
			   sizeof(cnt[0]) * SMMU_PDIR_COUNT,
			   GFP_KERNEL);
	page = alloc_page(GFP_KERNEL | __GFP_DMA);

	spin_lock_irqsave(&as->lock, flags);

	if (as->pdir_page) {
		/* We raced, free the redundant */
		err = -EAGAIN;
		goto err_out;
	}

	if (!page || !cnt) {
		dev_err(smmu->dev, "failed to allocate at %s\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	as->pdir_page = page;
	as->pte_count = cnt;

	pdir = page_address(as->pdir_page);

	for (pdn = 0; pdn < SMMU_PDIR_COUNT; pdn++)
		pdir[pdn] = _PDE_VACANT(pdn);
	FLUSH_CPU_DCACHE(pdir, as->pdir_page, SMMU_PDIR_SIZE);
	smmu_flush_ptc(smmu, pdir, as->pdir_page);
	val = SMMU_TLB_FLUSH_VA_MATCH_ALL |
		SMMU_TLB_FLUSH_ASID_MATCH__ENABLE |
		(as->asid << SMMU_TLB_FLUSH_ASID_SHIFT(as));
	smmu_write(smmu, val, SMMU_TLB_FLUSH);
	FLUSH_SMMU_REGS(as->smmu);

	spin_unlock_irqrestore(&as->lock, flags);

	return 0;

err_out:
	spin_unlock_irqrestore(&as->lock, flags);

	if (page)
		__free_page(page);
	if (cnt)
		devm_kfree(smmu->dev, cnt);
	return err;
}

static size_t __smmu_iommu_unmap_pages(struct smmu_as *as, dma_addr_t iova,
				       size_t bytes)
{
	int total = bytes >> PAGE_SHIFT;
	u32 *pdir = page_address(as->pdir_page);
	struct smmu_device *smmu = as->smmu;
	unsigned long iova_base = iova;
	bool flush_all = (total > smmu_flush_all_th_pages) ? true : false;

	while (total > 0) {
		int ptn = SMMU_ADDR_TO_PTN(iova);
		int pdn = SMMU_ADDR_TO_PDN(iova);
		struct page *page = SMMU_EX_PTBL_PAGE(pdir[pdn]);
		u32 *ptbl;
		u32 *pte;
		int count;

		if (!pfn_valid(page_to_pfn(page))) {
			total -= SMMU_PDN_TO_ADDR(pdn + 1) - iova;
			iova = SMMU_PDN_TO_ADDR(pdn + 1);
			continue;
		}

		ptbl = page_address(page);
		pte = &ptbl[ptn];
		count = min_t(int, SMMU_PTBL_COUNT - ptn, total);

		dev_dbg(as->smmu->dev, "unmapping %d pages at once\n", count);

		if (pte) {
			int i;
			unsigned int *rest = &as->pte_count[pdn];
			size_t bytes = sizeof(*pte) * count;

			memset(pte, 0, bytes);
			FLUSH_CPU_DCACHE(pte, page, bytes);

			for (i = 0; i < count; i++)
				trace_smmu_set_pte(as->asid,
						   iova + i * PAGE_SIZE, 0,
						   PAGE_SIZE, 0);
			*rest -= count;
			if (!*rest)
				free_ptbl(as, iova, !flush_all);

			if (!flush_all)
				flush_ptc_and_tlb_range(smmu, as, iova, pte,
							page, count);
		}

		iova += PAGE_SIZE * count;
		total -= count;
	}

	if (flush_all)
		flush_ptc_and_tlb_as(as, iova_base,
				     iova_base + bytes);

	return bytes;
}

static size_t __smmu_iommu_unmap_largepage(struct smmu_as *as, dma_addr_t iova)
{
	int pdn = SMMU_ADDR_TO_PDN(iova);
	u32 *pdir = (u32 *)page_address(as->pdir_page);

	pdir[pdn] = _PDE_VACANT(pdn);
	trace_smmu_set_pte(as->asid, iova, 0, SZ_4M, 0);

	FLUSH_CPU_DCACHE(&pdir[pdn], as->pdir_page, sizeof pdir[pdn]);
	flush_ptc_and_tlb(as->smmu, as, iova, &pdir[pdn], as->pdir_page, 1);
	return SZ_4M;
}

static int __smmu_iommu_map_pfn(struct smmu_as *as, dma_addr_t iova,
				unsigned long pfn, unsigned long prot)
{
	struct smmu_device *smmu = as->smmu;
	u32 *pte;
	unsigned int *count;
	struct page *page;
	int attrs = as->pte_attr;

	pte = locate_pte(as, iova, true, &page, &count);
	if (WARN_ON(!pte))
		return -ENOMEM;

	if (*pte == _PTE_VACANT(iova))
		(*count)++;

	if (dma_get_attr(DMA_ATTR_READ_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_WRITABLE;
	else if (dma_get_attr(DMA_ATTR_WRITE_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_READABLE;

	*pte = SMMU_PFN_TO_PTE(pfn, attrs);
	trace_smmu_set_pte(as->asid, iova, PFN_PHYS(pfn), PAGE_SIZE, attrs);

	FLUSH_CPU_DCACHE(pte, page, sizeof(*pte));
	flush_ptc_and_tlb(smmu, as, iova, pte, page, 0);
	put_signature(as, iova, pfn);
	return 0;
}

static int __smmu_iommu_map_page(struct smmu_as *as, dma_addr_t iova,
				 phys_addr_t pa, unsigned long prot)
{
	unsigned long pfn = __phys_to_pfn(pa);

	return __smmu_iommu_map_pfn(as, iova, pfn, prot);
}

static int __smmu_iommu_map_largepage(struct smmu_as *as, dma_addr_t iova,
				 phys_addr_t pa, unsigned long prot)
{
	int pdn = SMMU_ADDR_TO_PDN(iova);
	u32 *pdir = (u32 *)page_address(as->pdir_page);
	int attrs = _PDE_ATTR;

	if (pdir[pdn] != _PDE_VACANT(pdn))
		return -EINVAL;

	if (dma_get_attr(DMA_ATTR_READ_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_WRITABLE;
	else if (dma_get_attr(DMA_ATTR_WRITE_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_READABLE;

	pdir[pdn] = SMMU_ADDR_TO_PDN(pa) << 10 | attrs;
	trace_smmu_set_pte(as->asid, iova, pa, SZ_4M, attrs);

	FLUSH_CPU_DCACHE(&pdir[pdn], as->pdir_page, sizeof pdir[pdn]);
	flush_ptc_and_tlb(as->smmu, as, iova, &pdir[pdn], as->pdir_page, 1);

	return 0;
}

static int smmu_iommu_map(struct iommu_domain *domain, unsigned long iova,
			  phys_addr_t pa, size_t bytes, unsigned long prot)
{
	struct smmu_as *as = domain_to_as(domain, iova);
	unsigned long flags;
	int err;
	int (*fn)(struct smmu_as *as, dma_addr_t iova, phys_addr_t pa,
		  unsigned long prot);

	dev_dbg(as->smmu->dev, "[%d] %pad:%pap\n", as->asid, &iova, &pa);

	switch (bytes) {
	case SZ_4K:
		fn = __smmu_iommu_map_page;
		break;
	case SZ_4M:
		fn = __smmu_iommu_map_largepage;
		break;
	default:
		WARN(1,  "%lld not supported\n", (u64)bytes);
		return -EINVAL;
	}

	spin_lock_irqsave(&as->lock, flags);
	err = fn(as, iova, pa, prot);
	spin_unlock_irqrestore(&as->lock, flags);
	return err;
}

static int smmu_iommu_map_pages(struct iommu_domain *domain, unsigned long iova,
				struct page **pages, size_t total, unsigned long prot)
{
	struct smmu_as *as = domain_to_as(domain, iova);
	struct smmu_device *smmu = as->smmu;
	u32 *pdir = page_address(as->pdir_page);
	int err = 0;
	unsigned long iova_base = iova;
	bool flush_all = (total > smmu_flush_all_th_pages) ? true : false;
	int attrs = as->pte_attr;

	if (dma_get_attr(DMA_ATTR_READ_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_WRITABLE;
	else if (dma_get_attr(DMA_ATTR_WRITE_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_READABLE;

	while (total > 0) {
		int pdn = SMMU_ADDR_TO_PDN(iova);
		int ptn = SMMU_ADDR_TO_PTN(iova);
		unsigned int *rest = &as->pte_count[pdn];
		int count = min_t(size_t, SMMU_PTBL_COUNT - ptn, total);
		struct page *tbl_page;
		u32 *ptbl;
		u32 *pte;
		int i;
		unsigned long flags;

		spin_lock_irqsave(&as->lock, flags);

		if (pdir[pdn] == _PDE_VACANT(pdn)) {
			tbl_page = alloc_ptbl(as, iova, !flush_all);
			if (!tbl_page) {
				err = -ENOMEM;
				spin_unlock_irqrestore(&as->lock, flags);
				goto out;
			}

		} else {
			tbl_page = SMMU_EX_PTBL_PAGE(pdir[pdn]);
		}

		ptbl = page_address(tbl_page);
		for (i = 0; i < count; i++) {
			pte = &ptbl[ptn + i];

			if (*pte == _PTE_VACANT(iova + i * PAGE_SIZE))
				(*rest)++;

			*pte = SMMU_PFN_TO_PTE(page_to_pfn(pages[i]), attrs);
			trace_smmu_set_pte(as->asid, iova, page_to_phys(pages[i]),
					   PAGE_SIZE, attrs);
		}

		pte = &ptbl[ptn];
		FLUSH_CPU_DCACHE(pte, tbl_page, count * sizeof(*pte));
		if (!flush_all)
			flush_ptc_and_tlb_range(smmu, as, iova, pte, tbl_page,
						count);

		iova += PAGE_SIZE * count;
		total -= count;
		pages += count;

		spin_unlock_irqrestore(&as->lock, flags);
	}

out:
	if (flush_all)
		flush_ptc_and_tlb_as(as, iova_base,
				     iova_base + total * PAGE_SIZE);
	return err;
}

#define sg_num_pages(sg)					\
	(PAGE_ALIGN((sg)->offset + (sg)->length) >> PAGE_SHIFT)

static int smmu_iommu_map_sg(struct iommu_domain *domain, unsigned long iova,
			     struct scatterlist *sgl, int npages, unsigned long prot)
{
	int err = 0;
	unsigned long iova_base = iova;
	bool flush_all = (npages > smmu_flush_all_th_pages) ? true : false;
	struct smmu_as *as = domain_to_as(domain, iova);
	u32 *pdir = page_address(as->pdir_page);
	struct smmu_device *smmu = as->smmu;
	int attrs = as->pte_attr;
	size_t total = npages;
	size_t sg_remaining = sg_num_pages(sgl);
	unsigned long sg_pfn = page_to_pfn(sg_page(sgl));

	if (dma_get_attr(DMA_ATTR_READ_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_WRITABLE;
	else if (dma_get_attr(DMA_ATTR_WRITE_ONLY, (struct dma_attrs *)prot))
		attrs &= ~_READABLE;

	while (total > 0) {
		int pdn = SMMU_ADDR_TO_PDN(iova);
		int ptn = SMMU_ADDR_TO_PTN(iova);
		unsigned int *rest = &as->pte_count[pdn];
		int count = min_t(size_t, SMMU_PTBL_COUNT - ptn, total);
		struct page *tbl_page;
		u32 *ptbl;
		u32 *pte;
		int i;
		unsigned long flags;

		spin_lock_irqsave(&as->lock, flags);

		if (pdir[pdn] == _PDE_VACANT(pdn)) {
			tbl_page = alloc_ptbl(as, iova, !flush_all);
			if (!tbl_page) {
				err = -ENOMEM;
				spin_unlock_irqrestore(&as->lock, flags);
				break;
			}

		} else {
			tbl_page = SMMU_EX_PTBL_PAGE(pdir[pdn]);
		}

		ptbl = page_address(tbl_page);
		for (i = 0; i < count; i++) {

			pte = &ptbl[ptn + i];
			if (*pte == _PTE_VACANT(iova + i * PAGE_SIZE))
				(*rest)++;

			*pte = SMMU_PFN_TO_PTE(sg_pfn, attrs);
			trace_smmu_set_pte(as->asid, iova, PFN_PHYS(sg_pfn),
					   PAGE_SIZE, attrs);
			sg_pfn++;
			if (--sg_remaining)
				continue;

			sgl = sg_next(sgl);
			if (sgl) {
				sg_pfn = page_to_pfn(sg_page(sgl));
				sg_remaining = sg_num_pages(sgl);
			}
		}

		pte = &ptbl[ptn];
		FLUSH_CPU_DCACHE(pte, tbl_page, count * sizeof(*pte));
		if (!flush_all)
			flush_ptc_and_tlb_range(smmu, as, iova, pte, tbl_page,
						count);

		iova += PAGE_SIZE * count;
		total -= count;

		spin_unlock_irqrestore(&as->lock, flags);
	}

	if (flush_all)
		flush_ptc_and_tlb_as(as, iova_base,
				     iova_base + npages * PAGE_SIZE);

	return err;
}

static int __smmu_iommu_unmap(struct smmu_as *as, dma_addr_t iova,
	size_t bytes)
{
	int pdn = SMMU_ADDR_TO_PDN(iova);
	u32 *pdir = page_address(as->pdir_page);

	if (!(pdir[pdn] & _PDE_NEXT))
		return __smmu_iommu_unmap_largepage(as, iova);

	return __smmu_iommu_unmap_pages(as, iova, bytes);
}

static size_t smmu_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
			       size_t bytes)
{
	struct smmu_as *as = domain_to_as(domain, iova);
	unsigned long flags;
	size_t unmapped;

	dev_dbg(as->smmu->dev, "[%d] %pad\n", as->asid, &iova);

	spin_lock_irqsave(&as->lock, flags);
	unmapped = __smmu_iommu_unmap(as, iova, bytes);
	spin_unlock_irqrestore(&as->lock, flags);
	return unmapped;
}

static phys_addr_t smmu_iommu_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	struct smmu_as *as = domain_to_as(domain, iova);
	unsigned long flags;
	int pdn = SMMU_ADDR_TO_PDN(iova);
	u32 *pdir = page_address(as->pdir_page);
	phys_addr_t pa = 0;

	spin_lock_irqsave(&as->lock, flags);

	if (pdir[pdn] & _PDE_NEXT) {
		u32 *pte;
		unsigned int *count;
		struct page *page;

		pte = locate_pte(as, iova, false, &page, &count);
		if (pte) {
			unsigned long pfn = *pte & SMMU_PFN_MASK;
			pa = PFN_PHYS(pfn);
		}
	} else {
		pa = pdir[pdn] << SMMU_PDE_SHIFT;
	}

	dev_dbg(as->smmu->dev, "iova:%pad pfn:%pap asid:%d\n",
		&iova, &pa, as->asid);

	spin_unlock_irqrestore(&as->lock, flags);
	return pa;
}

static int smmu_iommu_domain_has_cap(struct iommu_domain *domain,
				     unsigned long cap)
{
	return 0;
}

#if defined(CONFIG_DMA_API_DEBUG) || defined(CONFIG_FTRACE)
char *debug_dma_platformdata(struct device *dev)
{
	static char buf[21];
	struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(dev);
	int asid = -1;

	if (mapping) {
		struct smmu_domain *dom = mapping->domain->priv;
		int i, len = 0;
		for_each_set_bit(i, (unsigned long *)&(dom->bitmap),
				MAX_AS_PER_DEV) {
			asid = dom->as[i]->asid;
			len += sprintf(buf + len, "%d ", asid);
		}
	} else
		sprintf(buf, "%d", asid);

	return buf;
}
#endif

static const struct file_operations smmu_ptdump_fops;

static void debugfs_create_as(struct smmu_as *as)
{
	struct dentry *dent;
	char name[] = "as000";

	sprintf(name, "as%03d", as->asid);
	dent = debugfs_create_dir(name, as->smmu->debugfs_root);
	if (!dent)
		return;
	as->debugfs_root = dent;
	debugfs_create_file("iovainfo", S_IRUSR, as->debugfs_root,
			    as + as->asid, &smmu_ptdump_fops);
}

static struct smmu_as *smmu_as_alloc(void)
{
	int i, err = -EAGAIN;
	unsigned long flags;
	struct smmu_as *as;
	struct smmu_device *smmu = smmu_handle;

	/* Look for a free AS with lock held */
	for  (i = 0; i < smmu->num_as; i++) {
		as = &smmu->as[i];

		if (as->pdir_page)
			continue;

		err = alloc_pdir(as);
		if (!err)
			goto found;

		if (err != -EAGAIN)
			break;
	}
	if (i == smmu->num_as)
		dev_err(smmu->dev,  "no free AS\n");
	return ERR_PTR(err);

found:
	spin_lock_irqsave(&smmu->lock, flags);

	/* Update PDIR register */
	smmu_write(smmu, SMMU_PTB_ASID_CUR(as->asid), SMMU_PTB_ASID);
	smmu_write(smmu,
		   SMMU_MK_PDIR(as->pdir_page, as->pdir_attr), SMMU_PTB_DATA);
	FLUSH_SMMU_REGS(smmu);

	spin_unlock_irqrestore(&smmu->lock, flags);

	debugfs_create_as(as);
	dev_dbg(smmu->dev, "smmu_as@%p\n", as);

	return as;
}

static void debugfs_create_master(struct smmu_client *c)
{
	int i;

	for (i = 0; i < MAX_AS_PER_DEV; i++) {
		char name[] = "as000";
		char target[256];
		struct smmu_as *as = c->domain->as[i];
		struct dentry *dent;

		if (!as)
			continue;

		if (!c->debugfs_root)
			c->debugfs_root =
				debugfs_create_dir(dev_name(c->dev),
						as->smmu->masters_root);
		sprintf(name, "as%03d", as->asid);
		sprintf(target, "../../as%03d", as->asid);
		debugfs_create_symlink(name, c->debugfs_root, target);

		sprintf(target, "../masters/%s", dev_name(c->dev));
		dent = debugfs_create_symlink(dev_name(c->dev), as->debugfs_root,
					      target);
		c->as_link[i] = dent;
	}
}

static int smmu_iommu_attach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct smmu_domain *dom = domain->priv;
	struct smmu_as *as;
	struct smmu_device *smmu;
	struct smmu_client *client, *c;
	struct iommu_linear_map *area;
	struct dma_iommu_mapping *dma_map;
	u64 swgids;
	int err = -ENOMEM;
	int idx;
	unsigned long as_bitmap[1];
	unsigned long as_alloc_bitmap = 0;

	client = tegra_smmu_find_client(smmu_handle, dev);
	if (!client)
		return -ENOMEM;

	dma_map = to_dma_iommu_mapping(dev);
	dma_map_to_as_bitmap(dma_map, as_bitmap);

	for_each_set_bit(idx, as_bitmap, MAX_AS_PER_DEV) {
		if (test_and_set_bit(idx, dom->bitmap))
			continue;
		as = smmu_as_alloc();
		if (IS_ERR(as)) {
			err = PTR_ERR(as);
			goto release_as;
		}
		dom->as[idx] = as;
		set_bit(idx, &as_alloc_bitmap);
		dev_info(dev, "domain=%p allocates as[%d]=%p\n", dom, idx, as);
	}

	/* get the first valid asid */
	idx = __ffs(dom->bitmap[0]);
	as = dom->as[idx];
	smmu = as->smmu;

	area = client->prop->area;
	while (area && area->size) {
		DEFINE_DMA_ATTRS(attrs);
		size_t size = PAGE_ALIGN(area->size);

		dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, &attrs);
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
		err = dma_map_linear_attrs(dev, area->start, size, 0, &attrs);
		if (err == DMA_ERROR_CODE)
			dev_err(dev, "Failed IOVA linear map %pad(%zx)\n",
				&area->start, size);
		else
			dev_info(dev, "IOVA linear map %pad(%zx)\n",
				 &area->start, size);
		area++;
	}

	client->domain = dom;

	swgids = client->swgids;
	swgids &= smmu_handle->swgids;
	err = smmu_client_enable_hwgrp(client, swgids);
	if (err)
		goto err_hwgrp;

	spin_lock(&as->client_lock);
	list_for_each_entry(c, &as->client, list) {
		if (c->dev == dev) {
			dev_err(smmu->dev,
				"%s is already attached\n", dev_name(c->dev));
			err = -EINVAL;
			goto err_client;
		}
	}
	list_add(&client->list, &as->client);
	spin_unlock(&as->client_lock);

	/*
	 * Reserve "page zero" for AVP vectors using a common dummy
	 * page.
	 */
	if (swgids & TEGRA_SWGROUP_BIT(AVPC)) {
		struct page *page;

		page = as->smmu->avp_vector_page;
		__smmu_iommu_map_pfn(as, 0, page_to_pfn(page), 0);

		pr_debug("Reserve \"page zero\" \
			for AVP vectors using a common dummy\n");
	}

	dev_dbg(smmu->dev, "%s is attached\n", dev_name(dev));
	debugfs_create_master(client);
	return 0;

err_client:
	smmu_client_disable_hwgrp(client);
	spin_unlock(&as->client_lock);
err_hwgrp:
	devm_kfree(smmu->dev, client);
release_as:
	for_each_set_bit(idx, &as_alloc_bitmap, MAX_AS_PER_DEV) {
		free_pdir(dom->as[idx]);
		dom->as[idx] = NULL;
	}
	return err;
}

static void smmu_iommu_detach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct smmu_as *as = domain_to_as(domain, -1);
	struct smmu_device *smmu;
	struct smmu_client *c;
	struct dentry *temp, **as_link;
	int i;

	if (!as)
		return;
	smmu = as->smmu;

	spin_lock(&as->client_lock);

	list_for_each_entry(c, &as->client, list) {
		if (c->dev == dev) {
			temp = c->debugfs_root;
			as_link = c->as_link;
			c->debugfs_root = NULL;
			list_del(&c->list);
			smmu_client_disable_hwgrp(c);
			dev_dbg(smmu->dev,
				"%s is detached\n", dev_name(c->dev));
			goto out;
		}
	}
	dev_err(smmu->dev, "Couldn't find %s\n", dev_name(dev));
	spin_unlock(&as->client_lock);
	return;
out:
	spin_unlock(&as->client_lock);
	for (i = 0; i < MAX_AS_PER_DEV; i++)
		debugfs_remove(as_link[i]);
	debugfs_remove_recursive(temp);
}

static int smmu_iommu_domain_init(struct iommu_domain *domain)
{
	struct smmu_device *smmu = smmu_handle;
	struct smmu_domain *smmu_domain;

	BUG_ON(domain->priv);
	smmu_domain = devm_kzalloc(smmu->dev, sizeof(*smmu_domain), GFP_KERNEL);
	if (!smmu_domain)
		return -ENOMEM;

	domain->priv = smmu_domain;

	domain->geometry.aperture_start = smmu->iovmm_base;
	domain->geometry.aperture_end   = smmu->iovmm_base +
		smmu->page_count * SMMU_PAGE_SIZE - 1;
	domain->geometry.force_aperture = true;
	return 0;
}

static void smmu_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct smmu_as *as = domain_to_as(domain, -1);
	struct smmu_device *smmu;
	unsigned long flags;
	struct smmu_map_prop *prop;


	/* find the smmu_map_prop containing this domain */
	list_for_each_entry(prop, &smmu_handle->asprops, list) {
		if (prop->map && (prop->map->domain == domain)) {
			prop->map = NULL;
			break;
		}
	}

	if (!as)
		return;
	smmu = as->smmu;

	spin_lock_irqsave(&as->lock, flags);

	debugfs_remove_recursive(as->debugfs_root);

	if (as->pdir_page) {
		spin_lock(&smmu->lock);
		smmu_write(smmu, SMMU_PTB_ASID_CUR(as->asid), SMMU_PTB_ASID);
		smmu_write(smmu, SMMU_PTB_DATA_RESET_VAL, SMMU_PTB_DATA);
		FLUSH_SMMU_REGS(smmu);
		spin_unlock(&smmu->lock);

		free_pdir(as);
	}

	if (!list_empty(&as->client)) {
		struct smmu_client *c, *tmp_c;
		list_for_each_entry_safe(c, tmp_c, &as->client, list) {
			dev_err(smmu->dev,
					"detaching %s because iommu domain is destroyed!\n",
					dev_name(c->dev));
			smmu_iommu_detach_dev(domain, c->dev);
		}
	}

	spin_unlock_irqrestore(&as->lock, flags);

	devm_kfree(smmu->dev, domain->priv);
	domain->priv = NULL;
	dev_dbg(smmu->dev, "smmu_as@%p\n", as);
}

static struct iommu_ops smmu_iommu_ops = {
	.domain_init	= smmu_iommu_domain_init,
	.domain_destroy	= smmu_iommu_domain_destroy,
	.attach_dev	= smmu_iommu_attach_dev,
	.detach_dev	= smmu_iommu_detach_dev,
	.map		= smmu_iommu_map,
	.map_pages	= smmu_iommu_map_pages,
	.map_sg		= smmu_iommu_map_sg,
	.unmap		= smmu_iommu_unmap,
	.iova_to_phys	= smmu_iommu_iova_to_phys,
	.domain_has_cap	= smmu_iommu_domain_has_cap,
	.pgsize_bitmap	= SMMU_IOMMU_PGSIZES,
};

/* Should be in the order of enum */
static const char * const smmu_debugfs_mc[] = { "mc", };
static const char * const smmu_debugfs_cache[] = {  "tlb", "ptc", };

static ssize_t smmu_debugfs_stats_write(struct file *file,
					const char __user *buffer,
					size_t count, loff_t *pos)
{
	struct smmu_debugfs_info *info;
	struct smmu_device *smmu;
	int i;
	enum {
		_OFF = 0,
		_ON,
		_RESET,
	};
	const char * const command[] = {
		[_OFF]		= "off",
		[_ON]		= "on",
		[_RESET]	= "reset",
	};
	char str[] = "reset";
	u32 val;
	size_t offs;

	count = min_t(size_t, count, sizeof(str));
	if (copy_from_user(str, buffer, count))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(command); i++)
		if (strncmp(str, command[i],
			    strlen(command[i])) == 0)
			break;

	if (i == ARRAY_SIZE(command))
		return -EINVAL;

	info = file_inode(file)->i_private;
	smmu = info->smmu;

	offs = SMMU_CACHE_CONFIG(info->cache);
	val = smmu_read(smmu, offs);
	switch (i) {
	case _OFF:
		val &= ~SMMU_CACHE_CONFIG_STATS_ENABLE;
		val &= ~SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		break;
	case _ON:
		val |= SMMU_CACHE_CONFIG_STATS_ENABLE;
		val &= ~SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		break;
	case _RESET:
		val |= SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		val &= ~SMMU_CACHE_CONFIG_STATS_TEST;
		smmu_write(smmu, val, offs);
		break;
	default:
		BUG();
		break;
	}

	dev_dbg(smmu->dev, "%s() %08x, %08x @%08llx\n", __func__,
		val, smmu_read(smmu, offs), (u64)offs);

	return count;
}

static int smmu_debugfs_stats_show(struct seq_file *s, void *v)
{
	struct smmu_debugfs_info *info = s->private;
	struct smmu_device *smmu = info->smmu;
	int i;
	const char * const stats[] = { "hit", "miss", };


	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		u32 val;
		size_t offs;

		offs = SMMU_STATS_CACHE_COUNT(info->mc, info->cache, i);
		val = smmu_read(smmu, offs);
		seq_printf(s, "%s:%08x ", stats[i], val);

		dev_dbg(smmu->dev, "%s() %s %08x @%08llx\n", __func__,
			stats[i], val, (u64)offs);
	}
	seq_printf(s, "\n");
	return 0;
}

static int smmu_debugfs_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, smmu_debugfs_stats_show, inode->i_private);
}

static const struct file_operations smmu_debugfs_stats_fops = {
	.open		= smmu_debugfs_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= smmu_debugfs_stats_write,
};

static void smmu_debugfs_delete(struct smmu_device *smmu)
{
	debugfs_remove_recursive(smmu->debugfs_root);
	kfree(smmu->debugfs_info);
}

struct smmu_addr_marker {
	u32 start_address;
	const char *name;
};

#define SZ_3G	(SZ_1G + SZ_2G)
static struct smmu_addr_marker address_markers[] = {
	{ 0,		"0x0000:0000", },
	{ SZ_1G,	"0x4000:0000", },
	{ SZ_2G,	"0x8000:0000", },
	{ SZ_3G,	"0xc000:0000", },
	{ -1,			NULL, },
};

struct smmu_pg_state {
	struct seq_file *seq;
	const struct smmu_addr_marker *marker;
	u32 start_address;
	unsigned level;
	u32 current_prot;
};

static void smmu_dump_attr(struct smmu_pg_state *st)
{
	int i;
	const char prot_set[] = "RW-";
	const char prot_clr[] = "--S";

	for (i = 0; i < ARRAY_SIZE(prot_set); i++) {
		if (st->current_prot & BIT(31 - i))
			seq_printf(st->seq, "%c", prot_set[i]);
		else
			seq_printf(st->seq, "%c", prot_clr[i]);
	}
}

static void smmu_note_page(struct smmu_pg_state *st, u32 addr, int level,
			   u32 val)
{
	static const char units[] = "KMGTPE";
	u32 prot = val & _MASK_ATTR;

	if (!st->level) {
		st->level = level;
		st->current_prot = prot;
		seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
	} else if (prot != st->current_prot || level != st->level ||
		   addr >= st->marker[1].start_address) {
		const char *unit = units;
		unsigned long delta;

		if (st->current_prot) {
			seq_printf(st->seq, "0x%08x-0x%08x   ",
				   st->start_address, addr);

			delta = (addr - st->start_address) >> 10;
			while (!(delta & 1023) && unit[1]) {
				delta >>= 10;
				unit++;
			}
			seq_printf(st->seq, "%9lu%c ", delta, *unit);
			smmu_dump_attr(st);
			seq_puts(st->seq, "\n");
		}

		if (addr >= st->marker[1].start_address) {
			st->marker++;
			seq_printf(st->seq, "---[ %s ]---\n", st->marker->name);
		}

		st->start_address = addr;
		st->current_prot = prot;
		st->level = level;
	}
}

static void smmu_walk_pte(struct smmu_pg_state *st, u32 *pgd, u32 start)
{
	int i;
	u32 *pte = page_address(SMMU_EX_PTBL_PAGE(*pgd));

	for (i = 0; i < PTRS_PER_PTE; i++, pte++)
		smmu_note_page(st, start + i * PAGE_SIZE, 2, *pte);
}

static void smmu_walk_pgd(struct seq_file *m, struct smmu_as *as)
{
	int i;
	u32 *pgd;
	struct smmu_pg_state st = {
		.seq	= m,
		.marker	= address_markers,
	};

	if (!pfn_valid(page_to_pfn(as->pdir_page)))
		return;

	pgd = page_address(as->pdir_page);
	for (i = 0; i < SMMU_PDIR_COUNT; i++, pgd++) {
		u32 addr = i * SMMU_PAGE_SIZE * SMMU_PTBL_COUNT;

		if (*pgd & _PDE_NEXT)
			smmu_walk_pte(&st, pgd, addr);
		else
			smmu_note_page(&st, addr, 1, *pgd);
	}

	smmu_note_page(&st, 0, 0, 0);
}

static int smmu_ptdump_show(struct seq_file *m, void *v)
{
	smmu_walk_pgd(m, m->private);
	return 0;
}

static int smmu_ptdump_open(struct inode *inode, struct file *file)
{
	return single_open(file, smmu_ptdump_show, inode->i_private);
}

static const struct file_operations smmu_ptdump_fops = {
	.open		= smmu_ptdump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static void smmu_debugfs_create(struct smmu_device *smmu)
{
	int i;
	size_t bytes;
	struct dentry *root;

	bytes = ARRAY_SIZE(smmu_debugfs_mc) * ARRAY_SIZE(smmu_debugfs_cache) *
		sizeof(*smmu->debugfs_info);
	smmu->debugfs_info = kmalloc(bytes, GFP_KERNEL);
	if (!smmu->debugfs_info)
		return;

	root = debugfs_create_dir(dev_name(smmu->dev), NULL);
	if (!root)
		goto err_out;
	smmu->debugfs_root = root;

	root = debugfs_create_dir("masters", smmu->debugfs_root);
	if (!root)
		goto err_out;
	smmu->masters_root = root;

	for (i = 0; i < ARRAY_SIZE(smmu_debugfs_mc); i++) {
		int j;
		struct dentry *mc;

		mc = debugfs_create_dir(smmu_debugfs_mc[i], root);
		if (!mc)
			goto err_out;

		for (j = 0; j < ARRAY_SIZE(smmu_debugfs_cache); j++) {
			struct dentry *cache;
			struct smmu_debugfs_info *info;

			info = smmu->debugfs_info;
			info += i * ARRAY_SIZE(smmu_debugfs_mc) + j;
			info->smmu = smmu;
			info->mc = i;
			info->cache = j;

			cache = debugfs_create_file(smmu_debugfs_cache[j],
						    S_IWUSR | S_IRUSR, mc,
						    (void *)info,
						    &smmu_debugfs_stats_fops);
			if (!cache)
				goto err_out;
		}
	}

	debugfs_create_size_t("flush_all_threshold_pages", S_IWUSR | S_IRUSR,
			      root, &smmu_flush_all_th_pages);
	return;

err_out:
	smmu_debugfs_delete(smmu);
}

int tegra_smmu_suspend(struct device *dev)
{
	int i;
	struct smmu_device *smmu = dev_get_drvdata(dev);

	for (i = 0; i < smmu->num_translation_enable; i++)
		smmu->translation_enable[i] = smmu_read(smmu,
				translation_enable_offset(i));

	for (i = 0; i < smmu->num_asid_security; i++)
		smmu->asid_security[i] =
			smmu_read(smmu, smmu_asid_security_ofs[i]);

	return 0;
}
EXPORT_SYMBOL(tegra_smmu_suspend);

int tegra_smmu_save(void)
{
	return tegra_smmu_suspend(save_smmu_device);
}

struct device *get_smmu_device(void)
{
	return save_smmu_device;
}
EXPORT_SYMBOL(get_smmu_device);

int tegra_smmu_resume(struct device *dev)
{
	struct smmu_device *smmu = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&smmu->lock, flags);
	smmu_setup_regs(smmu);
	spin_unlock_irqrestore(&smmu->lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_smmu_resume);

int tegra_smmu_restore(void)
{
	return tegra_smmu_resume(save_smmu_device);
}

static int tegra_smmu_probe(struct platform_device *pdev)
{
	int err = -EINVAL;
	struct smmu_device *smmu;
	struct resource *regs, *regs2;
	struct device *dev = &pdev->dev;
	int i, count;
	u32 num_as;
	dma_addr_t base;
	size_t size, bytes;
	const struct of_device_id *match;
	struct tegra_smmu_chip_data *chip_data = NULL;

	if (smmu_handle) {
		dev_info(dev, "skip %s", __func__);
		return -ENODEV;
	}

	BUILD_BUG_ON(PAGE_SHIFT != SMMU_PAGE_SHIFT);

	save_smmu_device = dev;

	match = of_match_node(tegra_smmu_of_match, dev->of_node);
	if (!match)
		goto exit_probe;

	if (!match->data) {
		chip_data = devm_kzalloc(dev,
					sizeof(*chip_data), GFP_KERNEL);
		if (!chip_data)
			goto exit_probe;

		chip_data->num_asids = 128;
	} else
		chip_data = (struct tegra_smmu_chip_data *) match->data;

	if (of_get_dma_window(dev->of_node, NULL, 0, NULL, &base,
				&size))
		goto exit_probe;
	size >>= SMMU_PAGE_SHIFT;

	if (of_property_read_u32(dev->of_node, "#asids",
				   &num_as))
		goto exit_probe;

	if (num_as > chip_data->num_asids) {
		dev_err(dev, "invalid number of asid\n");
		goto exit_probe;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!regs || !regs2 || !size) {
		dev_err(dev, "No SMMU resources\n");
		goto exit_probe;
	}

	err = -ENOMEM;
	bytes = sizeof(*smmu) + num_as * sizeof(*smmu->as);
	smmu = devm_kzalloc(dev, bytes, GFP_KERNEL);
	if (!smmu)
		goto exit_probe;

	smmu->dev = dev;
	INIT_LIST_HEAD(&smmu->asprops);
	count = tegra_smmu_of_register_asprops(smmu->dev, &smmu->asprops);
	if (!count) {
		dev_err(dev, "invalid domains property\n");
		err = -EINVAL;
		goto exit_probe;
	}

	smmu->chip_data = chip_data;
	smmu->num_as = num_as;
	smmu->clients = RB_ROOT;

	smmu->iovmm_base = base;
	smmu->page_count = size;
	smmu->regs = devm_ioremap(dev, regs->start, resource_size(regs));
	smmu->regs_ahbarb = devm_ioremap(dev, regs2->start,
					 resource_size(regs2));
	if (!smmu->regs || !smmu->regs_ahbarb) {
		err = -ENXIO;
		goto fail_cleanup;
	}

	if (of_property_read_u64(dev->of_node, "swgid-mask",
				   &smmu->swgids))
		goto fail_cleanup;

	if (of_property_read_u32(dev->of_node, "#num-translation-enable",
				   &smmu->num_translation_enable))
		goto fail_cleanup;

	if (of_property_read_u32(dev->of_node, "#num-asid-security",
				   &smmu->num_asid_security))
		goto fail_cleanup;

	if (of_property_read_u32(dev->of_node, "ptc-cache-size",
				   &smmu->ptc_cache_size))
		goto fail_cleanup;

	for (i = 0; i < smmu->num_translation_enable; i++)
		smmu->translation_enable[i] = ~0;

	for (i = 0; i < smmu->num_as; i++) {
		struct smmu_as *as = &smmu->as[i];

		as->smmu = smmu;
		as->asid = i;
		as->pdir_attr = _PDIR_ATTR;
		as->pde_attr = _PDE_ATTR;
		as->pte_attr = _PTE_ATTR;

		spin_lock_init(&as->lock);
		spin_lock_init(&as->client_lock);
		INIT_LIST_HEAD(&as->client);
	}
	spin_lock_init(&smmu->lock);
	smmu_setup_regs(smmu);
	platform_set_drvdata(pdev, smmu);

	smmu->avp_vector_page = alloc_page(GFP_KERNEL);
	if (!smmu->avp_vector_page)
		goto fail_cleanup;

	smmu_debugfs_create(smmu);
	BUG_ON(cmpxchg(&smmu_handle, NULL, smmu));
	bus_set_iommu(&platform_bus_type, &smmu_iommu_ops);

	dev_info(dev, "Loaded Tegra IOMMU driver\n");
	return 0;
fail_cleanup:
	devm_kfree(dev, smmu);
exit_probe:
	dev_err(dev, "tegra smmu probe failed, e=%d", err);
	return err;
}

static int tegra_smmu_remove(struct platform_device *pdev)
{
	struct smmu_device *smmu = platform_get_drvdata(pdev);
	int i;

	smmu_debugfs_delete(smmu);

	smmu_write(smmu, SMMU_CONFIG_DISABLE, SMMU_CONFIG);
	for (i = 0; i < smmu->num_as; i++)
		free_pdir(&smmu->as[i]);
	__free_page(smmu->avp_vector_page);
	smmu_handle = NULL;
	return 0;
}

const struct dev_pm_ops tegra_smmu_pm_ops = {
	.suspend	= tegra_smmu_suspend,
	.resume		= tegra_smmu_resume,
};

static struct of_device_id tegra_smmu_of_match[] = {
	{ .compatible = "nvidia,tegra210-smmu", },
	{ .compatible = "nvidia,tegra132-smmu", },
	{ .compatible = "nvidia,tegra124-smmu", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_smmu_of_match);

static struct platform_driver tegra_smmu_driver = {
	.probe		= tegra_smmu_probe,
	.remove		= tegra_smmu_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_smmu",
		.pm	= &tegra_smmu_pm_ops,
		.of_match_table = tegra_smmu_of_match,
	},
};

static int tegra_smmu_device_notifier(struct notifier_block *nb,
				      unsigned long event, void *_dev)
{
	struct dma_iommu_mapping *map;
	struct device *dev = _dev;
	u64 swgids;
	const char * const event_to_string[] = {
		"-----",
		"ADD_DEVICE",
		"DEL_DEVICE",
		"BIND_DRIVER",
		"BOUND_DRIVER",
		"UNBIND_DRIVER",
		"UNBOUND_DRIVER",
	};

	swgids = tegra_smmu_get_swgids(dev);
	if (swgids_is_error(swgids))
		goto end;
	/* dev is a smmu client */

	pr_debug("%s() dev=%s swgids=%llx event=%s ops=%p\n",
		__func__, dev_name(dev), swgids,
		event_to_string[event], get_dma_ops(dev));

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		if (get_dma_ops(dev) != &arm_dma_ops)
			break;

		if (!smmu_handle) {
			dev_err(dev, "No map available yet!!!\n");
			break;
		}

		map = tegra_smmu_of_get_mapping(dev, swgids,
						&smmu_handle->asprops);
		if (!map) {
			dev_err(dev, "map creation failed!!!\n");
			break;
		}

		if (arm_iommu_attach_device(dev, map)) {
			dev_err(dev, "Failed to attach %s\n", dev_name(dev));
			arm_iommu_release_mapping(map);
			break;
		}
		dev_dbg(dev, "Attached %s to map %p\n", dev_name(dev), map);
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:
		dev_dbg(dev, "Detaching %s from map %p\n", dev_name(dev),
			to_dma_iommu_mapping(dev));
		arm_iommu_detach_device(dev);
		break;
	default:
		break;
	}

end:
	return NOTIFY_DONE;
}

struct notifier_block tegra_smmu_device_nb = {
	.notifier_call = tegra_smmu_device_notifier,
};

struct notifier_block tegra_smmu_device_pci_nb = {
	.notifier_call = tegra_smmu_device_notifier,
};

void tegra_smmu_map_misc_device(struct device *dev)
{

	tegra_smmu_device_notifier(&tegra_smmu_device_nb,
				   BUS_NOTIFY_BIND_DRIVER, dev);
}
EXPORT_SYMBOL(tegra_smmu_map_misc_device);

void tegra_smmu_unmap_misc_device(struct device *dev)
{
	tegra_smmu_device_notifier(&tegra_smmu_device_nb,
				   BUS_NOTIFY_UNBOUND_DRIVER, dev);
}
EXPORT_SYMBOL(tegra_smmu_unmap_misc_device);

static int tegra_smmu_init(void)
{
	int err;

	err = platform_driver_register(&tegra_smmu_driver);
	if (err)
		return err;
	if (IS_ENABLED(CONFIG_ARM_DMA_USE_IOMMU))
		bus_register_notifier(&platform_bus_type,
				      &tegra_smmu_device_nb);
	return 0;
}

static int tegra_smmu_remove_map(struct device *dev, void *data)
{
	struct dma_iommu_mapping *map = to_dma_iommu_mapping(dev);
	if (map)
		arm_iommu_release_mapping(map);
	return 0;
}

static void __exit tegra_smmu_exit(void)
{
	if (IS_ENABLED(CONFIG_ARM_DMA_USE_IOMMU)) {
		bus_for_each_dev(&platform_bus_type, NULL, NULL,
				 tegra_smmu_remove_map);
		bus_unregister_notifier(&platform_bus_type,
					&tegra_smmu_device_nb);
	}
	platform_driver_unregister(&tegra_smmu_driver);
}

core_initcall(tegra_smmu_init);
module_exit(tegra_smmu_exit);

MODULE_DESCRIPTION("IOMMU API for SMMU in Tegra SoC");
MODULE_AUTHOR("Hiroshi DOYU <hdoyu@nvidia.com>");
MODULE_LICENSE("GPL v2");
