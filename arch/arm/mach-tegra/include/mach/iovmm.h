/*
 * arch/arm/mach-tegra/include/mach/iovmm.h
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed i the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "../../iomap.h"

#ifndef _MACH_TEGRA_IOVMM_H_
#define _MACH_TEGRA_IOVMM_H_

typedef u32 tegra_iovmm_addr_t;

struct tegra_iovmm_device_ops;

/*
 * each I/O virtual memory manager unit should register a device with
 * the iovmm system
 */
struct tegra_iovmm_device {
	struct tegra_iovmm_device_ops	*ops;
	const char			*name;
	struct list_head		list;
	int				pgsize_bits;
};

/*
 * tegra_iovmm_domain serves a purpose analagous to mm_struct as defined in
 * <linux/mm_types.h> - it defines a virtual address space within which
 * tegra_iovmm_areas can be created.
 */
struct tegra_iovmm_domain {
	atomic_t		clients;
	atomic_t		locks;
	spinlock_t		block_lock;  /* RB-tree for iovmm_area blocks */
	unsigned long		flags;
	wait_queue_head_t	delay_lock;  /* when lock_client fails */
	struct rw_semaphore	map_lock;
	struct rb_root		all_blocks;  /* ordered by address */
	struct rb_root		free_blocks; /* ordered by size */
	struct tegra_iovmm_device *dev;
};

/*
 * tegra_iovmm_client is analagous to an individual task in the task group
 * which owns an mm_struct.
 */

struct iovmm_share_group;

struct tegra_iovmm_client {
	struct device *dev;
};

struct tegra_iovmm_area {
	dma_addr_t		iovm_start;
	size_t			iovm_length;
	pgprot_t		pgprot;
	struct device		*dev;
};

struct tegra_iovmm_device_ops {
	/* maps a VMA using the page residency functions provided by the VMA */
	int (*map)(struct tegra_iovmm_domain *domain,
		struct tegra_iovmm_area *io_vma);
	/* marks all PTEs in a VMA as invalid; decommits the virtual addres
	 * space (potentially freeing PDEs when decommit is true.) */
	void (*unmap)(struct tegra_iovmm_domain *domain,
		struct tegra_iovmm_area *io_vma, bool decommit);
	void (*map_pfn)(struct tegra_iovmm_domain *domain,
		struct tegra_iovmm_area *io_vma,
		unsigned long offs, unsigned long pfn);
	/*
	 * ensures that a domain is resident in the hardware's mapping region
	 * so that it may be used by a client
	 */
	int (*lock_domain)(struct tegra_iovmm_domain *domain,
		struct tegra_iovmm_client *client);
	void (*unlock_domain)(struct tegra_iovmm_domain *domain,
		struct tegra_iovmm_client *client);
	/*
	 * allocates a vmm_domain for the specified client; may return the same
	 * domain for multiple clients
	 */
	struct tegra_iovmm_domain* (*alloc_domain)(
		struct tegra_iovmm_device *dev,
		struct tegra_iovmm_client *client);
	void (*free_domain)(struct tegra_iovmm_domain *domain,
		struct tegra_iovmm_client *client);
	int (*suspend)(struct tegra_iovmm_device *dev);
	void (*resume)(struct tegra_iovmm_device *dev);
};

struct tegra_iovmm_area_ops {
	/*
	 * ensures that the page of data starting at the specified offset
	 * from the start of the iovma is resident and pinned for use by
	 * DMA, returns the system pfn, or an invalid pfn if the
	 * operation fails.
	 */
	unsigned long (*lock_makeresident)(struct tegra_iovmm_area *area,
		tegra_iovmm_addr_t offs);
	/* called when the page is unmapped from the I/O VMA */
	void (*release)(struct tegra_iovmm_area *area, tegra_iovmm_addr_t offs);
};

/*
 * Replace tegra_iovmm_*() with tegra_iommu_*() helpers
 */
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>

#include <asm/dma-iommu.h>

#define tegra_iovmm_alloc_client(d, s, m)	tegra_iommu_alloc_client(d)
#define tegra_iovmm_free_client(c)		tegra_iommu_free_client(c)

#define tegra_iovmm_create_vm(c, o, s, a, p, i)		\
	tegra_iommu_create_vm((c)->dev, i, s, p)
#define tegra_iovmm_free_vm(v)	tegra_iommu_free_vm(v)

#define tegra_iovmm_zap_vm(v)	tegra_iommu_zap_vm(v)

#define tegra_iovmm_get_vm_size(c)	TEGRA_IOMMU_SIZE
#define tegra_iovmm_get_max_free(c)	dma_iova_get_free_max((c)->dev)

#define tegra_iovmm_vm_insert_pfn(area, handle, pfn)			\
	({								\
		dma_addr_t da;						\
		struct device *dev = area->dev;				\
		struct dma_map_ops *ops = get_dma_ops(dev);		\
		DEFINE_DMA_ATTRS(attrs);				\
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);		\
		da = ops->map_page_at(dev, pfn_to_page(pfn), handle,	\
				 PAGE_SIZE, 0, 0, &attrs);		\
		dma_mapping_error(dev, da) ? -ENOMEM : 0;		\
	})

static inline int tegra_iovmm_vm_insert_pages(struct tegra_iovmm_area *area,
					      dma_addr_t va,
					      struct page **pages, size_t count)
{
	dma_addr_t da;
	struct device *dev = area->dev;
	struct dma_map_ops *ops = get_dma_ops(dev);
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	da = ops->map_pages(dev, pages, va, count, 0, &attrs);
	return dma_mapping_error(dev, da) ? -ENOMEM : 0;
}

#ifdef CONFIG_IOMMU_API
struct tegra_iovmm_area *tegra_iommu_create_vm(struct device *dev,
		       dma_addr_t req, size_t size, pgprot_t prot);

void tegra_iommu_free_vm(struct tegra_iovmm_area *area);

void tegra_iommu_zap_vm(struct tegra_iovmm_area *area);

struct tegra_iovmm_client *tegra_iommu_alloc_client(struct device *dev);

void tegra_iommu_free_client(struct tegra_iovmm_client *client);
#else
static struct tegra_iovmm_area *tegra_iommu_create_vm(struct device *dev,
			dma_addr_t req, size_t size, pgprot_t prot) {
	return NULL;
}

static void tegra_iommu_free_vm(struct tegra_iovmm_area *area) {}

static void tegra_iommu_zap_vm(struct tegra_iovmm_area *area) {}

static struct tegra_iovmm_client *tegra_iommu_alloc_client(struct device *dev) {
	return NULL;
}

static void tegra_iommu_free_client(struct tegra_iovmm_client *client) {}
#endif

#endif /* _MACH_TEGRA_IOVMM_H_*/
