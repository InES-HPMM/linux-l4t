/*
 * mods_mem.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2008-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA MODS kernel driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * NVIDIA MODS kernel driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVIDIA MODS kernel driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "mods_internal.h"

#include <linux/pagemap.h>

#ifdef CONFIG_BIGPHYS_AREA
#include <linux/bigphysarea.h>
#endif

static int mods_post_alloc(struct MODS_PHYS_CHUNK *pt,
			   u64			   phys_addr,
			   struct MODS_MEM_INFO   *p_mem_info);
static void mods_pre_free(struct MODS_PHYS_CHUNK *pt,
			  struct MODS_MEM_INFO	 *p_mem_info);

#if !defined(CONFIG_ARCH_TEGRA) || defined(CONFIG_CPA) ||\
	defined(CONFIG_ARCH_TEGRA_3x_SOC)
static int mods_set_mem_type(u64 virt_addr, u64 pages, u32 type)
{
	if (type == MODS_MEMORY_UNCACHED)
		return MODS_SET_MEMORY_UC(virt_addr, pages);
	else if (type == MODS_MEMORY_WRITECOMBINE)
		return MODS_SET_MEMORY_WC(virt_addr, pages);
	return 0;
}
#endif

static int mods_restore_mem_type(u64 virt_addr,
				 u64 pages,
				 u32 type_override)
{
	if ((type_override == MODS_MEMORY_UNCACHED) ||
			(type_override == MODS_MEMORY_WRITECOMBINE)) {
		return MODS_SET_MEMORY_WB(virt_addr, pages);
	}
	return 0;
}

static void mods_restore_cache(struct MODS_MEM_INFO *p_mem_info)
{
	unsigned int i;
	for (i = 0; i < p_mem_info->max_chunks; i++) {
		struct MODS_PHYS_CHUNK *pt = &p_mem_info->pages[i];
		if (!pt->allocated)
			break;

		mods_pre_free(pt, p_mem_info);
	}
}

static void mods_free_pages(struct MODS_MEM_INFO *p_mem_info)
{
	unsigned int i;
	/* release in reverse order */
	for (i = p_mem_info->max_chunks; i > 0; ) {
		struct MODS_PHYS_CHUNK *pt;
		--i;
		pt = &p_mem_info->pages[i];
		if (!pt->allocated)
			continue;

#if defined(CONFIG_PPC64)
		pci_unmap_page(p_mem_info->dev, pt->map_addr,
			       (1U<<pt->order)*PAGE_SIZE, DMA_BIDIRECTIONAL);
#endif

#ifdef CONFIG_BIGPHYS_AREA
		if (p_mem_info->alloc_type == MODS_ALLOC_TYPE_BIGPHYS_AREA) {
			bigphysarea_free_pages((void *)
					       p_mem_info->logical_addr);
		} else
#endif
			__free_pages(pt->p_page, pt->order);
	}
}

static gfp_t mods_alloc_flags(struct MODS_MEM_INFO *p_mem_info)
{
	gfp_t flags = GFP_KERNEL;
	if (p_mem_info->alloc_type != MODS_ALLOC_TYPE_NON_CONTIG)
		flags |= __GFP_COMP;
	if ((p_mem_info->addr_bits & 0xFF) == 32)
		flags |= __GFP_DMA32;
	else
		flags |= __GFP_HIGHMEM;
	return flags;
}

static int mods_alloc_contig_sys_pages(struct MODS_MEM_INFO *p_mem_info)
{
	u64 phys_addr;
	u64 end_addr  = 0;
	u32 order     = 0;

	LOG_ENT();

	while ((1U << order) < p_mem_info->num_pages)
		order++;
	p_mem_info->pages[0].order = order;

	p_mem_info->pages[0].p_page = alloc_pages_node(
			p_mem_info->numa_node,
			mods_alloc_flags(p_mem_info),
			order);

#ifdef CONFIG_BIGPHYS_AREA
	if (!p_mem_info->pages[0].p_page) {
		mods_debug_printk(DEBUG_MEM, "falling back to bigphysarea\n");
		p_mem_info->logical_addr = (u64)
			bigphysarea_alloc_pages(1U << order, 0, GFP_KERNEL);
		p_mem_info->alloc_type = MODS_ALLOC_TYPE_BIGPHYS_AREA;
	}
#endif

	if (!p_mem_info->pages[0].p_page &&
	    p_mem_info->logical_addr == 0) {
		LOG_EXT();
		return -ENOMEM;
	}

	p_mem_info->pages[0].allocated = 1;

#ifdef CONFIG_BIGPHYS_AREA
	if (p_mem_info->alloc_type == MODS_ALLOC_TYPE_BIGPHYS_AREA) {
		phys_addr = __pa(p_mem_info->logical_addr);
	} else
#endif
		phys_addr = page_to_phys(p_mem_info->pages[0].p_page);
	if (phys_addr == 0) {
		mods_error_printk("failed to determine physical address\n");
		mods_free_pages(p_mem_info);
		p_mem_info->logical_addr = 0;
		LOG_EXT();
		return -ENOMEM;
	}
	p_mem_info->pages[0].dma_addr = MODS_PHYS_TO_DMA(phys_addr);
#if defined(CONFIG_PPC64)
	if (p_mem_info->dev != NULL) {
		p_mem_info->pages[0].map_addr = pci_map_page(p_mem_info->dev,
						p_mem_info->pages[0].p_page,
						0,
						(1U << order) * PAGE_SIZE,
						DMA_BIDIRECTIONAL);
	} else {
		p_mem_info->pages[0].map_addr = p_mem_info->pages[0].dma_addr;
	}
#endif

	mods_debug_printk(DEBUG_MEM,
	    "alloc contig 0x%lx bytes%s, 2^%u pages, %s, phys 0x%llx\n",
	    (unsigned long)p_mem_info->length,
	    p_mem_info->alloc_type == MODS_ALLOC_TYPE_BIGPHYS_AREA ?
		" bigphys" : "",
	    p_mem_info->pages[0].order,
	    mods_get_prot_str(p_mem_info->cache_type),
	    (unsigned long long)p_mem_info->pages[0].dma_addr);

	end_addr = p_mem_info->pages[0].dma_addr + p_mem_info->length;
	if (((p_mem_info->addr_bits & 0xFF) == 32) &&
	    (end_addr > 0x100000000ULL)) {
		mods_error_printk("allocation exceeds 32-bit addressing\n");
		mods_free_pages(p_mem_info);
		p_mem_info->logical_addr = 0;
		LOG_EXT();
		return -ENOMEM;
	}

	if (mods_post_alloc(p_mem_info->pages, phys_addr, p_mem_info)) {
		mods_free_pages(p_mem_info);
		p_mem_info->logical_addr = 0;
		LOG_EXT();
		return -EINVAL;
	}
	LOG_EXT();
	return 0;
}

static int mods_get_max_order_needed(u32 num_pages)
{
	int order = 0;
	while (order < 10 && (1U<<(order+1)) <= num_pages)
		++order;
	return order;
}

static int mods_alloc_noncontig_sys_pages(struct MODS_MEM_INFO *p_mem_info)
{
	u32 pages_left = p_mem_info->num_pages;
	u32 num_chunks = 0;

	LOG_ENT();

	memset(p_mem_info->pages, 0,
	       p_mem_info->max_chunks * sizeof(p_mem_info->pages[0]));

	/* alloc pages */
	while (pages_left > 0) {
		u64 phys_addr = 0;
		int order     = mods_get_max_order_needed(pages_left);
		struct MODS_PHYS_CHUNK *pt = &p_mem_info->pages[num_chunks];

		while (order >= 0) {
			pt->p_page = alloc_pages_node(
					p_mem_info->numa_node,
					mods_alloc_flags(p_mem_info),
					(unsigned int)order);
			if (pt->p_page)
				break;
			else
				--order;
		}
		if (!pt->p_page) {
			mods_error_printk("out of memory\n");
			goto failed;
		}
		pt->allocated = 1;

		pages_left -= 1U << order;
		pt->order = (u32)order;

		phys_addr = page_to_phys(pt->p_page);
		if (phys_addr == 0) {
			mods_error_printk("phys addr lookup failed\n");
			goto failed;
		}
		pt->dma_addr = MODS_PHYS_TO_DMA(phys_addr);
#if defined(CONFIG_PPC64)
		if (p_mem_info->dev != NULL) {
			pt->map_addr = pci_map_page(p_mem_info->dev,
						pt->p_page,
						0,
						(1U << pt->order) * PAGE_SIZE,
						DMA_BIDIRECTIONAL);
		} else {
			pt->map_addr = pt->dma_addr;
		}
#endif
		mods_debug_printk(DEBUG_MEM,
		    "alloc 0x%lx bytes [%u], 2^%u pages, %s, phys 0x%llx\n",
		    (unsigned long)p_mem_info->length,
		    (unsigned int)num_chunks,
		    pt->order,
		    mods_get_prot_str(p_mem_info->cache_type),
		    (unsigned long long)pt->dma_addr);

		++num_chunks;

		if (mods_post_alloc(pt, phys_addr, p_mem_info))
			goto failed;
	}

	return 0;

failed:
	mods_restore_cache(p_mem_info);
	mods_free_pages(p_mem_info);
	return -ENOMEM;
}

static int mods_register_alloc(struct file          *fp,
			       struct MODS_MEM_INFO *p_mem_info)
{
	MODS_PRIVATE_DATA(private_data, fp);
	if (unlikely(mutex_lock_interruptible(&private_data->mtx)))
		return -EINTR;
	list_add(&p_mem_info->list, private_data->mods_alloc_list);
	mutex_unlock(&private_data->mtx);
	return OK;
}

static int mods_unregister_and_free(struct file          *fp,
				    struct MODS_MEM_INFO *p_del_mem)
{
	struct MODS_MEM_INFO *p_mem_info;

	MODS_PRIVATE_DATA(private_data, fp);
	struct list_head  *head;
	struct list_head  *iter;

	mods_debug_printk(DEBUG_MEM_DETAILED, "free %p\n", p_del_mem);

	if (unlikely(mutex_lock_interruptible(&private_data->mtx)))
		return -EINTR;

	head = private_data->mods_alloc_list;

	list_for_each(iter, head) {
		p_mem_info = list_entry(iter, struct MODS_MEM_INFO, list);

		if (p_del_mem == p_mem_info) {
			list_del(iter);

			mutex_unlock(&private_data->mtx);

			mods_restore_cache(p_mem_info);
			mods_free_pages(p_mem_info);

			kfree(p_mem_info);

			return OK;
		}
	}

	mutex_unlock(&private_data->mtx);

	mods_error_printk("failed to unregister allocation %p\n",
			  p_del_mem);
	return -EINVAL;
}

int mods_unregister_all_alloc(struct file *fp)
{
	MODS_PRIVATE_DATA(private_data, fp);
	struct list_head *head = private_data->mods_alloc_list;
	struct list_head *iter;
	struct list_head *tmp;

	list_for_each_safe(iter, tmp, head) {
		struct MODS_MEM_INFO *p_mem_info;
		int ret;

		p_mem_info = list_entry(iter, struct MODS_MEM_INFO, list);
		ret = mods_unregister_and_free(fp, p_mem_info);
		if (ret)
			return ret;
	}

	return OK;
}

/* Returns an offset within an allocation deduced from physical address.
 * If dma address doesn't belong to the allocation, returns non-zero. */
int mods_get_alloc_offset(struct MODS_MEM_INFO *p_mem_info,
			  u64			dma_addr,
			  u32		       *ret_offs)
{
	u32 i;
	u64 offset = 0;

	for (i = 0; i < p_mem_info->max_chunks; i++) {
		struct MODS_PHYS_CHUNK *pt = &p_mem_info->pages[i];
		u64 addr = pt->dma_addr;
		u32 size = PAGE_SIZE << pt->order;

		if (!pt->allocated)
			break;

		if (dma_addr >= addr &&
		    dma_addr <  addr + size) {
			*ret_offs = dma_addr - addr + offset;
			return 0;
		}

		offset += size;
	}

	/* The physical address doesn't belong to the allocation */
	return -EINVAL;
}

struct MODS_MEM_INFO *mods_find_alloc(struct file *fp, u64 phys_addr)
{
	MODS_PRIVATE_DATA(private_data, fp);
	struct list_head     *plist_head = private_data->mods_alloc_list;
	struct list_head     *plist_iter;
	struct MODS_MEM_INFO *p_mem_info;
	u32		      offset;

	list_for_each(plist_iter, plist_head) {
		p_mem_info = list_entry(plist_iter,
					struct MODS_MEM_INFO,
					list);
		if (!mods_get_alloc_offset(p_mem_info, phys_addr, &offset))
			return p_mem_info;
	}

	/* The physical address doesn't belong to any allocation */
	return NULL;
}

static u32 mods_estimate_max_chunks(u32 num_pages)
{
	u32 max_chunks = 0;
	u32 bit_scan;

	/* Count each contiguous block <=256KB */
	for (bit_scan = num_pages; bit_scan && max_chunks < 6; bit_scan >>= 1)
		++max_chunks;

	/* Count remaining contiguous blocks >256KB */
	max_chunks += bit_scan;

	/* 4x slack for medium memory fragmentation */
	max_chunks <<= 2;

	/* No sense to allocate more chunks than pages */
	if (max_chunks > num_pages)
		max_chunks = num_pages;

	/* Now, if memory is heavily fragmented, we are screwed */

	return max_chunks;
}

static struct MODS_PHYS_CHUNK *mods_find_phys_chunk(
					struct MODS_MEM_INFO *p_mem_info,
					u32 offset,
					u32 *chunk_offset)
{
	struct MODS_PHYS_CHUNK	*pt = NULL;
	u32			pages_left;
	u32			page_offs;
	u32			i;

	if (!p_mem_info)
		return NULL;

	pages_left = offset >> PAGE_SHIFT;
	page_offs  = offset & (~PAGE_MASK);

	for (i = 0; i < p_mem_info->max_chunks; i++) {
		u32 num_pages;

		pt = &p_mem_info->pages[i];
		if (!pt->allocated)
			break;

		num_pages = 1U << pt->order;
		if (pages_left < num_pages)
			break;
		pages_left -= num_pages;
		pt = NULL;
	}

	*chunk_offset = (pages_left << PAGE_SHIFT) + page_offs;
	return pt;
}

/************************
 * ESCAPE CALL FUNCTONS *
 ************************/

int esc_mods_device_alloc_pages_new(struct file	*fp,
				    struct MODS_DEVICE_ALLOC_PAGES_NEW *p)
{
	struct MODS_MEM_INFO *p_mem_info;
	u32    num_pages;
	u32    alloc_size;
	u32    max_chunks;
	int    ret = OK;

	LOG_ENT();

	mods_debug_printk(
		DEBUG_MEM_DETAILED,
		"alloc 0x%x bytes %s %s\n",
		p->num_bytes,
		p->contiguous ? "contiguous" : "noncontiguous",
		mods_get_prot_str(p->attrib));

	switch (p->attrib) {
	case MODS_MEMORY_CACHED:
	case MODS_MEMORY_UNCACHED:
	case MODS_MEMORY_WRITECOMBINE:
		break;

	default:
		mods_error_printk("invalid memory type: %u\n", p->attrib);
		LOG_EXT();
		return -EINVAL;
	}

	num_pages = (p->num_bytes >> PAGE_SHIFT) +
		    ((p->num_bytes & ~PAGE_MASK) ? 1 : 0);
	if (p->contiguous)
		max_chunks = 1;
	else
		max_chunks = mods_estimate_max_chunks(num_pages);
	alloc_size = sizeof(*p_mem_info) +
		     (max_chunks - 1) * sizeof(struct MODS_PHYS_CHUNK);

	p_mem_info = kmalloc(alloc_size, GFP_KERNEL);
	if (unlikely(!p_mem_info)) {
		mods_error_printk("failed to allocate auxiliary 0x%x bytes\n",
				  alloc_size);
		LOG_EXT();
		return -ENOMEM;
	}

	p_mem_info->max_chunks   = max_chunks;
	p_mem_info->alloc_type	 = p->contiguous
		? MODS_ALLOC_TYPE_CONTIG : MODS_ALLOC_TYPE_NON_CONTIG;
	p_mem_info->cache_type	 = p->attrib;
	p_mem_info->length	 = p->num_bytes;
	p_mem_info->logical_addr = 0;
	p_mem_info->addr_bits	 = p->address_bits;
	p_mem_info->num_pages	 = num_pages;
	p_mem_info->numa_node    = numa_node_id();
#if defined(CONFIG_PPC64)
	p_mem_info->dev		 = NULL;
#endif

	if (p->pci_device.bus || p->pci_device.device) {
		unsigned int devfn = PCI_DEVFN(p->pci_device.device,
					       p->pci_device.function);
		struct pci_dev *dev = MODS_PCI_GET_SLOT(p->pci_device.domain,
							p->pci_device.bus,
							devfn);
		if (!dev) {
			LOG_EXT();
			return -EINVAL;
		}
#if defined(CONFIG_PPC64)
		p_mem_info->dev = dev;
#endif
#if defined(MODS_HAS_DEV_TO_NUMA_NODE)
		p_mem_info->numa_node = dev_to_node(&dev->dev);
#endif
		mods_debug_printk(DEBUG_MEM_DETAILED,
			"affinity %x:%x.%x node %d\n",
			p->pci_device.bus,
			p->pci_device.device,
			p->pci_device.function,
			p_mem_info->numa_node);
	}

	p->memory_handle = 0;

	if (p->contiguous) {
		if (mods_alloc_contig_sys_pages(p_mem_info)) {
			mods_error_printk(
				"failed to alloc 0x%x contiguous bytes\n",
				p_mem_info->length);
			kfree(p_mem_info);
			LOG_EXT();
			return -ENOMEM;
		}
	} else {
		if (mods_alloc_noncontig_sys_pages(p_mem_info)) {
			mods_error_printk(
			    "failed to alloc 0x%x noncontiguous bytes\n",
			    p_mem_info->length);
			kfree(p_mem_info);
			LOG_EXT();
			return -ENOMEM;
		}
	}

	p->memory_handle = (u64)(size_t)p_mem_info;

	mods_debug_printk(DEBUG_MEM_DETAILED, "alloc %p\n", p_mem_info);

	ret = mods_register_alloc(fp, p_mem_info);
	LOG_EXT();
	return ret;
}

int esc_mods_device_alloc_pages(struct file                    *fp,
				struct MODS_DEVICE_ALLOC_PAGES *p)
{
	int retval;
	struct MODS_DEVICE_ALLOC_PAGES_NEW dev_alloc_pages = {0};
	LOG_ENT();

	dev_alloc_pages.num_bytes		= p->num_bytes;
	dev_alloc_pages.contiguous		= p->contiguous;
	dev_alloc_pages.address_bits		= p->address_bits;
	dev_alloc_pages.attrib			= p->attrib;
	dev_alloc_pages.pci_device.domain	= 0;
	dev_alloc_pages.pci_device.bus		= p->pci_device.bus;
	dev_alloc_pages.pci_device.device	= p->pci_device.device;
	dev_alloc_pages.pci_device.function	= p->pci_device.function;

	retval = esc_mods_device_alloc_pages_new(fp, &dev_alloc_pages);
	if (!retval)
		p->memory_handle = dev_alloc_pages.memory_handle;

	LOG_EXT();
	return retval;
}

int esc_mods_alloc_pages(struct file *fp, struct MODS_ALLOC_PAGES *p)
{
	int retval;
	struct MODS_DEVICE_ALLOC_PAGES_NEW dev_alloc_pages;
	LOG_ENT();

	dev_alloc_pages.num_bytes	    = p->num_bytes;
	dev_alloc_pages.contiguous	    = p->contiguous;
	dev_alloc_pages.address_bits	    = p->address_bits;
	dev_alloc_pages.attrib		    = p->attrib;
	dev_alloc_pages.pci_device.domain   = 0;
	dev_alloc_pages.pci_device.bus	    = 0;
	dev_alloc_pages.pci_device.device   = 0;
	dev_alloc_pages.pci_device.function = 0;

	retval = esc_mods_device_alloc_pages_new(fp, &dev_alloc_pages);
	if (!retval)
		p->memory_handle = dev_alloc_pages.memory_handle;

	LOG_EXT();
	return retval;
}

int esc_mods_free_pages(struct file *fp, struct MODS_FREE_PAGES *p)
{
	int ret;

	LOG_ENT();

	ret = mods_unregister_and_free(fp,
	    (struct MODS_MEM_INFO *)(size_t)p->memory_handle);

	LOG_EXT();

	return ret;
}

int esc_mods_set_mem_type(struct file *fp, struct MODS_MEMORY_TYPE *p)
{
	struct MODS_MEM_INFO *p_mem_info;
	MODS_PRIVATE_DATA(private_data, fp);

	LOG_ENT();

	switch (p->type) {
	case MODS_MEMORY_CACHED:
	case MODS_MEMORY_UNCACHED:
	case MODS_MEMORY_WRITECOMBINE:
		break;

	default:
		mods_error_printk("unsupported memory type: %u\n", p->type);
		LOG_EXT();
		return -EINVAL;
	}

	if (unlikely(mutex_lock_interruptible(&private_data->mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	p_mem_info = mods_find_alloc(fp, p->physical_address);
	if (p_mem_info) {
		mutex_unlock(&private_data->mtx);
		mods_error_printk("cannot set mem type on phys addr 0x%llx\n",
				  p->physical_address);
		LOG_EXT();
		return -EINVAL;
	}

	private_data->mem_type.dma_addr = p->physical_address;
	private_data->mem_type.size     = p->size;
	private_data->mem_type.type     = p->type;

	mutex_unlock(&private_data->mtx);

	LOG_EXT();
	return OK;
}

int esc_mods_get_phys_addr(struct file *fp, struct MODS_GET_PHYSICAL_ADDRESS *p)
{
	struct MODS_MEM_INFO	*p_mem_info;
	struct MODS_PHYS_CHUNK	*pt = NULL;
	u32			chunk_offset;

	LOG_ENT();

	p_mem_info = (struct MODS_MEM_INFO *)(size_t)p->memory_handle;
	pt = mods_find_phys_chunk(p_mem_info, p->offset, &chunk_offset);

	if (!pt || !pt->allocated) {
		mods_error_printk("invalid offset requested\n");
		LOG_EXT();
		return -EINVAL;
	}

	p->physical_address = pt->dma_addr + chunk_offset;
	mods_debug_printk(DEBUG_MEM_DETAILED,
		"get phys: %p+0x%x -> 0x%llx\n",
		p_mem_info, p->offset, p->physical_address);
	LOG_EXT();
	return 0;
}

int esc_mods_get_mapped_phys_addr(struct file *fp,
				  struct MODS_GET_PHYSICAL_ADDRESS *p)
{
#if !defined(CONFIG_PPC64)
	return esc_mods_get_phys_addr(fp, p);
#else
	struct MODS_MEM_INFO	*p_mem_info;
	struct MODS_PHYS_CHUNK	*pt = NULL;
	u32			chunk_offset;

	LOG_ENT();

	p_mem_info = (struct MODS_MEM_INFO *)(size_t)p->memory_handle;
	pt = mods_find_phys_chunk(p_mem_info, p->offset, &chunk_offset);

	if (!pt || !pt->allocated) {
		mods_error_printk("invalid offset requested\n");
		LOG_EXT();
		return -EINVAL;
	}

	p->physical_address = pt->map_addr + chunk_offset;
	mods_debug_printk(DEBUG_MEM_DETAILED,
		"get mapped phys: %p+0x%x -> 0x%llx\n",
		p_mem_info, p->offset, p->physical_address);
	LOG_EXT();
	return 0;
#endif
}

int esc_mods_virtual_to_phys(struct file *fp,
			     struct MODS_VIRTUAL_TO_PHYSICAL *p)
{
	struct MODS_GET_PHYSICAL_ADDRESS get_phys_addr;
	MODS_PRIVATE_DATA(private_data, fp);
	struct list_head *head;
	struct list_head *iter;

	LOG_ENT();

	if (unlikely(mutex_lock_interruptible(&private_data->mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	head = private_data->mods_mapping_list;

	list_for_each(iter, head) {
		struct SYS_MAP_MEMORY *p_map_mem;
		u64		       begin, end;
		u32                    phys_offs;

		p_map_mem = list_entry(iter, struct SYS_MAP_MEMORY, list);

		begin = p_map_mem->virtual_addr;
		end   = p_map_mem->virtual_addr + p_map_mem->mapping_length;

		if (p->virtual_address >= begin && p->virtual_address < end) {

			u32 virt_offs = p->virtual_address - begin;
			int ret;

			/* device memory mapping */
			if (!p_map_mem->p_mem_info) {
				p->physical_address = p_map_mem->dma_addr
						      + virt_offs;
				mutex_unlock(&private_data->mtx);

				mods_debug_printk(DEBUG_MEM_DETAILED,
				    "get phys: map %p virt 0x%llx -> 0x%llx\n",
				    p_map_mem, p->virtual_address,
				    p->physical_address);

				LOG_EXT();
				return OK;
			}

			if (mods_get_alloc_offset(p_map_mem->p_mem_info,
						  p_map_mem->dma_addr,
						  &phys_offs) != OK)
				break;

			get_phys_addr.memory_handle =
				(u64)(size_t)p_map_mem->p_mem_info;
			get_phys_addr.offset = virt_offs + phys_offs;

			mutex_unlock(&private_data->mtx);

			ret = esc_mods_get_phys_addr(fp, &get_phys_addr);
			if (ret != OK)
				return ret;

			p->physical_address = get_phys_addr.physical_address;

			mods_debug_printk(DEBUG_MEM_DETAILED,
			    "get phys: map %p virt 0x%llx -> 0x%llx\n",
			    p_map_mem, p->virtual_address, p->physical_address);

			LOG_EXT();
			return OK;
		}
	}

	mutex_unlock(&private_data->mtx);

	mods_error_printk("invalid virtual address\n");
	return -EINVAL;
}

int esc_mods_phys_to_virtual(struct file *fp,
			     struct MODS_PHYSICAL_TO_VIRTUAL *p)
{
	struct SYS_MAP_MEMORY *p_map_mem;
	MODS_PRIVATE_DATA(private_data, fp);
	struct list_head *head;
	struct list_head *iter;
	u32	offset;
	u32	map_offset;

	LOG_ENT();

	if (unlikely(mutex_lock_interruptible(&private_data->mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	head = private_data->mods_mapping_list;

	list_for_each(iter, head) {
		p_map_mem = list_entry(iter, struct SYS_MAP_MEMORY, list);

		/* device memory mapping */
		if (!p_map_mem->p_mem_info) {
			u64 end = p_map_mem->dma_addr
				+ p_map_mem->mapping_length;
			if (p->physical_address <  p_map_mem->dma_addr ||
			    p->physical_address >= end)
				continue;

			offset = p->physical_address
				 - p_map_mem->dma_addr;
			p->virtual_address = p_map_mem->virtual_addr
					     + offset;
			mutex_unlock(&private_data->mtx);

			mods_debug_printk(DEBUG_MEM_DETAILED,
			    "get virt: map %p phys 0x%llx -> 0x%llx\n",
			    p_map_mem, p->physical_address, p->virtual_address);

			LOG_EXT();
			return OK;
		}

		/* offset from the begining of the allocation */
		if (mods_get_alloc_offset(p_map_mem->p_mem_info,
					  p->physical_address,
					  &offset))
			continue;

		/* offset from the beginning of the mapping */
		if (mods_get_alloc_offset(p_map_mem->p_mem_info,
					  p_map_mem->dma_addr,
					  &map_offset))
			continue;

		if ((offset >= map_offset) &&
		    (offset <  map_offset + p_map_mem->mapping_length)) {
			p->virtual_address = p_map_mem->virtual_addr
					   + offset - map_offset;

			mutex_unlock(&private_data->mtx);
			mods_debug_printk(DEBUG_MEM_DETAILED,
			    "get virt: map %p phys 0x%llx -> 0x%llx\n",
			    p_map_mem, p->physical_address, p->virtual_address);

			LOG_EXT();
			return OK;
		}
	}
	mutex_unlock(&private_data->mtx);
	mods_error_printk("phys addr 0x%llx is not mapped\n",
			  p->physical_address);
	return -EINVAL;
}

int esc_mods_memory_barrier(struct file *fp)
{
	wmb();
	return OK;
}

#ifdef CONFIG_ARCH_TEGRA

static void clear_contiguous_cache
(
	u64 virt_start,
	u64 virt_end,
	u64 phys_start,
	u64 phys_end
)
{
	/* We are expecting virt_end and phys_end to point to the first address
	 * of the next range */
	u32 size = virt_end - virt_start;
	BUG_ON(size != phys_end - phys_start);
	size += (~virt_end + 1) & (~PAGE_MASK);  /* Align up to page boundary */

	mods_debug_printk(DEBUG_MEM_DETAILED,
	    "clear cache virt 0x%llx phys 0x%llx size 0x%x\n",
	    virt_start, phys_start, size);

#ifdef CONFIG_ARM64
	/* Flush L1 cache */
	__flush_dcache_area((void *)(size_t)(virt_start), size);
#else
	/* Flush L1 cache */
	__cpuc_flush_dcache_area((void *)(size_t)(virt_start), size);

	/* Now flush L2 cache. */
	outer_flush_range(phys_start, phys_end);
#endif
}

static void clear_entry_cache_mappings
(
	struct SYS_MAP_MEMORY *p_map_mem,
	u64		       virt_offs,
	u64		       virt_offs_end
)
{
	struct MODS_MEM_INFO *p_mem_info = p_map_mem->p_mem_info;
	u64	     cur_vo = p_map_mem->virtual_addr;
	unsigned int i;

	if (!p_mem_info)
		return;

	if (p_mem_info->cache_type != MODS_MEMORY_CACHED)
		return;

	for (i = 0; i < p_mem_info->max_chunks; i++) {
		struct MODS_PHYS_CHUNK *pt = &p_mem_info->pages[i];
		u64 cur_vo_end = cur_vo + (PAGE_SIZE << pt->order);

		if (!pt->allocated)
			break;

		if (virt_offs_end <= cur_vo)
			break;

		if (virt_offs < cur_vo_end) {
			u64 clear_vo	 = virt_offs;
			u64 clear_vo_end = virt_offs_end;
			u64 clear_pa;
			u64 clear_pa_end;

			if (clear_vo < cur_vo)
				clear_vo = cur_vo;

			if (clear_vo_end > cur_vo_end)
				clear_vo_end = cur_vo_end;

			clear_pa = MODS_DMA_TO_PHYS(pt->dma_addr);
			if (clear_vo > cur_vo)
				clear_pa += clear_vo - cur_vo;

			clear_pa_end = clear_pa + (clear_vo_end - clear_vo);

			mods_debug_printk(DEBUG_MEM_DETAILED,
			    "clear cache %p [%u]\n", p_mem_info, i);

			clear_contiguous_cache(clear_vo,
					       clear_vo_end,
					       clear_pa,
					       clear_pa_end);
		}

		cur_vo = cur_vo_end;
	}
}

int esc_mods_flush_cpu_cache_range(struct file *fp,
				   struct MODS_FLUSH_CPU_CACHE_RANGE *p)
{
	MODS_PRIVATE_DATA(private_data, fp);
	struct list_head *head;
	struct list_head *iter;

	if (irqs_disabled() || in_interrupt() ||
	    p->virt_addr_start > p->virt_addr_end ||
	    MODS_INVALIDATE_CPU_CACHE == p->flags) {

		mods_debug_printk(DEBUG_MEM_DETAILED, "cannot clear cache\n");
		return -EINVAL;
	}

	if (unlikely(mutex_lock_interruptible(&private_data->mtx))) {
		LOG_EXT();
		return -EINTR;
	}

	head = private_data->mods_mapping_list;

	list_for_each(iter, head) {
		struct SYS_MAP_MEMORY *p_map_mem
			= list_entry(iter, struct SYS_MAP_MEMORY, list);

		u64 mapped_va = p_map_mem->virtual_addr;

		/* Note: mapping end points to the first address of next range*/
		u64 mapping_end = mapped_va + p_map_mem->mapping_length;

		int start_on_page = p->virt_addr_start >= mapped_va
				    && p->virt_addr_start < mapping_end;
		int start_before_page = p->virt_addr_start < mapped_va;
		int end_on_page = p->virt_addr_end >= mapped_va
				  && p->virt_addr_end < mapping_end;
		int end_after_page = p->virt_addr_end >= mapping_end;
		u64 virt_start = p->virt_addr_start;

		/* Kernel expects end to point to the first address of next
		 * range */
		u64 virt_end = p->virt_addr_end + 1;

		if ((start_on_page || start_before_page)
			&& (end_on_page || end_after_page)) {

			if (!start_on_page)
				virt_start = p_map_mem->virtual_addr;
			if (!end_on_page)
				virt_end = mapping_end;
			clear_entry_cache_mappings(p_map_mem,
						   virt_start,
						   virt_end);
		}
	}
	mutex_unlock(&private_data->mtx);
	return OK;
}

#endif

static int mods_post_alloc(struct MODS_PHYS_CHUNK *pt,
			   u64			   phys_addr,
			   struct MODS_MEM_INFO   *p_mem_info)
{
	u32 num_pages = 1U << pt->order;
	u32 i;

	for (i = 0; i < num_pages; i++) {
		u64 ptr = 0;
		int ret = 0;
#ifdef CONFIG_BIGPHYS_AREA
		if (p_mem_info->alloc_type == MODS_ALLOC_TYPE_BIGPHYS_AREA) {
			ptr = p_mem_info->logical_addr + (i << PAGE_SHIFT);
		} else
#endif
			ptr = (u64)(size_t)kmap(pt->p_page + i);
		if (!ptr) {
			mods_error_printk("kmap failed\n");
			return -EINVAL;
		}
#if defined(CONFIG_ARCH_TEGRA) && !defined(CONFIG_CPA) &&\
	!defined(CONFIG_ARCH_TEGRA_3x_SOC)
		clear_contiguous_cache(ptr,
				ptr + PAGE_SIZE,
				phys_addr + (i << PAGE_SHIFT),
				phys_addr + ((i+1) << PAGE_SHIFT));
#else
		ret = mods_set_mem_type(ptr, 1, p_mem_info->cache_type);
#endif
#ifdef CONFIG_BIGPHYS_AREA
		if (p_mem_info->alloc_type != MODS_ALLOC_TYPE_BIGPHYS_AREA)
#endif
			kunmap(pt->p_page + i);
		if (ret) {
			mods_error_printk("set cache type failed\n");
			return -EINVAL;
		}
	}
	return 0;
}

static void mods_pre_free(struct MODS_PHYS_CHUNK *pt,
			  struct MODS_MEM_INFO	 *p_mem_info)
{
	u32 num_pages = 1U << pt->order;
	u32 i;

	for (i = 0; i < num_pages; i++) {
		u64 ptr = 0;
#ifdef CONFIG_BIGPHYS_AREA
		if (p_mem_info->alloc_type == MODS_ALLOC_TYPE_BIGPHYS_AREA)
			ptr = p_mem_info->logical_addr + (i << PAGE_SHIFT);
		else
#endif
			ptr = (u64)(size_t)kmap(pt->p_page + i);
		if (ptr)
			mods_restore_mem_type(ptr, 1, p_mem_info->cache_type);
#ifdef CONFIG_BIGPHYS_AREA
		if (p_mem_info->alloc_type != MODS_ALLOC_TYPE_BIGPHYS_AREA)
#endif
			kunmap(pt->p_page + i);
	}
}
