/*
 * mods_pci.c - This file is part of NVIDIA MODS kernel driver.
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

#include <linux/io.h>

/************************
 * PCI ESCAPE FUNCTIONS *
 ************************/

int esc_mods_find_pci_dev_new(struct file *pfile,
			      struct MODS_FIND_PCI_DEVICE_NEW *p)
{
	struct pci_dev *dev;
	int index = 0;

	mods_debug_printk(DEBUG_PCICFG,
			  "find pci dev %04x:%04x, index %d\n",
			  (int) p->vendor_id,
			  (int) p->device_id,
			  (int) p->index);

	dev = pci_get_device(p->vendor_id, p->device_id, NULL);

	while (dev) {
		if (index == p->index) {
			p->pci_device.domain	= pci_domain_nr(dev->bus);
			p->pci_device.bus	= dev->bus->number;
			p->pci_device.device	= PCI_SLOT(dev->devfn);
			p->pci_device.function	= PCI_FUNC(dev->devfn);
			return OK;
		}
		dev = pci_get_device(p->vendor_id, p->device_id, dev);
		index++;
	}

	return -EINVAL;
}

int esc_mods_find_pci_dev(struct file *pfile,
			  struct MODS_FIND_PCI_DEVICE *p)
{
	struct pci_dev *dev;
	int index = 0;

	mods_debug_printk(DEBUG_PCICFG,
			  "find pci dev %04x:%04x, index %d\n",
			  (int) p->vendor_id,
			  (int) p->device_id,
			  (int) p->index);

	dev = pci_get_device(p->vendor_id, p->device_id, NULL);

	while (dev) {
		if (index == p->index && pci_domain_nr(dev->bus) == 0) {
			p->bus_number		= dev->bus->number;
			p->device_number	= PCI_SLOT(dev->devfn);
			p->function_number	= PCI_FUNC(dev->devfn);
			return OK;
		}
		/* Only return devices in the first domain, but don't assume
		   that they're the first devices in the list */
		if (pci_domain_nr(dev->bus) == 0)
			index++;
		dev = pci_get_device(p->vendor_id, p->device_id, dev);
	}

	return -EINVAL;
}

int esc_mods_find_pci_class_code_new(struct file *pfile,
				     struct MODS_FIND_PCI_CLASS_CODE_NEW *p)
{
	struct pci_dev *dev;
	int index = 0;

	mods_debug_printk(DEBUG_PCICFG, "find pci class code %04x, index %d\n",
			  (int) p->class_code, (int) p->index);

	dev = pci_get_class(p->class_code, NULL);

	while (dev) {
		if (index == p->index) {
			p->pci_device.domain	= pci_domain_nr(dev->bus);
			p->pci_device.bus	= dev->bus->number;
			p->pci_device.device	= PCI_SLOT(dev->devfn);
			p->pci_device.function	= PCI_FUNC(dev->devfn);
			return OK;
		}
		dev = pci_get_class(p->class_code, dev);
		index++;
	}

	return -EINVAL;
}

int esc_mods_find_pci_class_code(struct file *pfile,
				 struct MODS_FIND_PCI_CLASS_CODE *p)
{
	struct pci_dev *dev;
	int index = 0;

	mods_debug_printk(DEBUG_PCICFG, "find pci class code %04x, index %d\n",
			  (int) p->class_code, (int) p->index);

	dev = pci_get_class(p->class_code, NULL);

	while (dev) {
		if (index == p->index && pci_domain_nr(dev->bus) == 0) {
			p->bus_number		= dev->bus->number;
			p->device_number	= PCI_SLOT(dev->devfn);
			p->function_number	= PCI_FUNC(dev->devfn);
			return OK;
		}
		/* Only return devices in the first domain, but don't assume
		   that they're the first devices in the list */
		if (pci_domain_nr(dev->bus) == 0)
			index++;
		dev = pci_get_class(p->class_code, dev);
	}

	return -EINVAL;
}

int esc_mods_pci_get_bar_info_new(struct file *pfile,
				  struct MODS_PCI_GET_BAR_INFO_NEW *p)
{
	struct pci_dev *dev;
	unsigned int devfn, bar_resource_offset, i;
#if !defined(MODS_HAS_IORESOURCE_MEM_64)
	__u32 temp;
#endif

	devfn = PCI_DEVFN(p->pci_device.device, p->pci_device.function);
	dev = MODS_PCI_GET_SLOT(p->pci_device.domain, p->pci_device.bus, devfn);

	if (dev == NULL)
		return -EINVAL;

	mods_debug_printk(DEBUG_PCICFG,
			  "pci get bar info %04x:%x:%02x:%x, bar index %d\n",
			  (int) p->pci_device.domain,
			  (int) p->pci_device.bus, (int) p->pci_device.device,
			  (int) p->pci_device.function, (int) p->bar_index);

	bar_resource_offset = 0;
	for (i = 0; i < p->bar_index; i++) {
#if defined(MODS_HAS_IORESOURCE_MEM_64)
		if (pci_resource_flags(dev, bar_resource_offset)
		    & IORESOURCE_MEM_64) {
#else
		pci_read_config_dword(dev,
				      (PCI_BASE_ADDRESS_0
				       + (bar_resource_offset * 4)),
				      &temp);
		if (temp & PCI_BASE_ADDRESS_MEM_TYPE_64) {
#endif
			bar_resource_offset += 2;
		} else {
			bar_resource_offset += 1;
		}
	}
	p->base_address = pci_resource_start(dev, bar_resource_offset);
	p->bar_size	= pci_resource_len(dev, bar_resource_offset);

	return OK;
}

int esc_mods_pci_get_bar_info(struct file *pfile,
			      struct MODS_PCI_GET_BAR_INFO *p)
{
	int retval;
	struct MODS_PCI_GET_BAR_INFO_NEW get_bar_info = { {0} };
	get_bar_info.pci_device.domain		= 0;
	get_bar_info.pci_device.bus		= p->pci_device.bus;
	get_bar_info.pci_device.device		= p->pci_device.device;
	get_bar_info.pci_device.function	= p->pci_device.function;
	get_bar_info.bar_index			= p->bar_index;

	retval = esc_mods_pci_get_bar_info_new(pfile, &get_bar_info);
	if (retval)
		return retval;

	p->base_address	= get_bar_info.base_address;
	p->bar_size	= get_bar_info.bar_size;
	return OK;
}

int esc_mods_pci_get_irq_new(struct file *pfile,
			     struct MODS_PCI_GET_IRQ_NEW *p)
{
	struct pci_dev *dev;
	unsigned int devfn;

	devfn = PCI_DEVFN(p->pci_device.device, p->pci_device.function);
	dev = MODS_PCI_GET_SLOT(p->pci_device.domain, p->pci_device.bus, devfn);

	if (dev == NULL)
		return -EINVAL;

	mods_debug_printk(DEBUG_PCICFG,
			  "pci get irq %04x:%x:%02x:%x\n",
			  (int) p->pci_device.domain,
			  (int) p->pci_device.bus, (int) p->pci_device.device,
			  (int) p->pci_device.function);

	p->irq = dev->irq;

	return OK;
}

int esc_mods_pci_get_irq(struct file *pfile,
			 struct MODS_PCI_GET_IRQ *p)
{
	int retval;
	struct MODS_PCI_GET_IRQ_NEW get_irq = { {0} };
	get_irq.pci_device.domain	= 0;
	get_irq.pci_device.bus		= p->pci_device.bus;
	get_irq.pci_device.device	= p->pci_device.device;
	get_irq.pci_device.function	= p->pci_device.function;

	retval = esc_mods_pci_get_irq_new(pfile, &get_irq);
	if (retval)
		return retval;

	p->irq = get_irq.irq;
	return OK;
}

int esc_mods_pci_read_new(struct file *pfile, struct MODS_PCI_READ_NEW *p)
{
	struct pci_bus *bus;
	unsigned int devfn;

	devfn = PCI_DEVFN(p->pci_device.device, p->pci_device.function);
	bus = pci_find_bus(p->pci_device.domain, p->pci_device.bus);

	if (bus == NULL)
		return -EINVAL;

	mods_debug_printk(DEBUG_PCICFG,
			  "pci read %04x:%x:%02x.%x, addr 0x%04x, size %d\n",
			  (int) p->pci_device.domain,
			  (int) p->pci_device.bus, (int) p->pci_device.device,
			  (int) p->pci_device.function, (int) p->address,
			  (int) p->data_size);

	p->data = 0;
	switch (p->data_size) {
	case 1:
		pci_bus_read_config_byte(bus, devfn, p->address,
					 (u8 *) &p->data);
		break;
	case 2:
		pci_bus_read_config_word(bus, devfn, p->address,
					 (u16 *) &p->data);
		break;
	case 4:
		pci_bus_read_config_dword(bus, devfn, p->address,
					  (u32 *) &p->data);
		break;
	default:
		return -EINVAL;
	}
	return OK;
}

int esc_mods_pci_read(struct file *pfile, struct MODS_PCI_READ *p)
{
	int retval;
	struct MODS_PCI_READ_NEW pci_read = { {0} };
	pci_read.pci_device.domain	= 0;
	pci_read.pci_device.bus		= p->bus_number;
	pci_read.pci_device.device	= p->device_number;
	pci_read.pci_device.function	= p->function_number;
	pci_read.address		= p->address;
	pci_read.data_size		= p->data_size;

	retval = esc_mods_pci_read_new(pfile, &pci_read);
	if (retval)
		return retval;

	p->data = pci_read.data;
	return OK;
}

int esc_mods_pci_write_new(struct file *pfile, struct MODS_PCI_WRITE_NEW *p)
{
	struct pci_bus *bus;
	unsigned int devfn;

	mods_debug_printk(DEBUG_PCICFG,
			  "pci write %04x:%x:%02x.%x, addr 0x%04x, size %d, "
			  "data 0x%x\n",
			  (int) p->pci_device.domain,
			  (int) p->pci_device.bus, (int) p->pci_device.device,
			  (int) p->pci_device.function,
			  (int) p->address, (int) p->data_size, (int) p->data);

	devfn = PCI_DEVFN(p->pci_device.device, p->pci_device.function);
	bus = pci_find_bus(p->pci_device.domain, p->pci_device.bus);

	if (bus == NULL) {
		mods_error_printk(
		  "pci write to %04x:%x:%02x.%x, addr 0x%04x, size %d failed\n",
		    (unsigned)p->pci_device.domain,
		    (unsigned)p->pci_device.bus,
		    (unsigned)p->pci_device.device,
		    (unsigned)p->pci_device.function,
		    (unsigned)p->address,
		    (int)p->data_size);
		return -EINVAL;
	}

	switch (p->data_size) {
	case 1:
		pci_bus_write_config_byte(bus, devfn, p->address, p->data);
		break;
	case 2:
		pci_bus_write_config_word(bus, devfn, p->address, p->data);
		break;
	case 4:
		pci_bus_write_config_dword(bus, devfn, p->address, p->data);
		break;
	default:
		return -EINVAL;
	}
	return OK;
}

int esc_mods_pci_write(struct file *pfile,
		       struct MODS_PCI_WRITE *p)
{
	struct MODS_PCI_WRITE_NEW pci_write = { {0} };
	pci_write.pci_device.domain	= 0;
	pci_write.pci_device.bus	= p->bus_number;
	pci_write.pci_device.device	= p->device_number;
	pci_write.pci_device.function	= p->function_number;
	pci_write.address		= p->address;
	pci_write.data			= p->data;
	pci_write.data_size		= p->data_size;

	return esc_mods_pci_write_new(pfile, &pci_write);
}

int esc_mods_pci_bus_add_dev(struct file *pfile,
			     struct MODS_PCI_BUS_ADD_DEVICES *scan)
{
#if defined(CONFIG_PCI)
	mods_info_printk("scanning pci bus %x\n", scan->bus);

	/* initiate a PCI bus scan to find hotplugged PCI devices in domain 0 */
	pci_scan_child_bus(pci_find_bus(0, scan->bus));

	/* add newly found devices */
	pci_bus_add_devices(pci_find_bus(0, scan->bus));

	return OK;
#else
	return -EINVAL;
#endif
}

/************************
 * PIO ESCAPE FUNCTIONS *
 ************************/

int esc_mods_pio_read(struct file *pfile, struct MODS_PIO_READ *p)
{
	LOG_ENT();
	switch (p->data_size) {
	case 1:
		p->data = inb(p->port);
		break;
	case 2:
		p->data = inw(p->port);
		break;
	case 4:
		p->data = inl(p->port);
		break;
	default:
		return -EINVAL;
	}
	LOG_EXT();
	return OK;
}

int esc_mods_pio_write(struct file *pfile, struct MODS_PIO_WRITE  *p)
{
	LOG_ENT();
	switch (p->data_size) {
	case 1:
		outb(p->data, p->port);
		break;
	case 2:
		outw(p->data, p->port);
		break;
	case 4:
		outl(p->data, p->port);
		break;
	default:
		return -EINVAL;
	}
	LOG_EXT();
	return OK;
}

int esc_mods_device_numa_info_new(struct file *fp,
				  struct MODS_DEVICE_NUMA_INFO_NEW *p)
{
#ifdef MODS_HAS_DEV_TO_NUMA_NODE
	unsigned int devfn = PCI_DEVFN(p->pci_device.device,
				       p->pci_device.function);
	struct pci_dev *dev = MODS_PCI_GET_SLOT(p->pci_device.domain,
						p->pci_device.bus, devfn);

	LOG_ENT();

	if (dev == NULL) {
		mods_error_printk("PCI device %u:%u:%u.%u not found\n",
				  p->pci_device.domain,
				  p->pci_device.bus, p->pci_device.device,
				  p->pci_device.function);
		LOG_EXT();
		return -EINVAL;
	}

	p->node = dev_to_node(&dev->dev);
	if (-1 != p->node) {
		const unsigned long *maskp
			= cpumask_bits(cpumask_of_node(p->node));
		unsigned int i, word, bit, maskidx;

		if (((nr_cpumask_bits + 31) / 32) > MAX_CPU_MASKS) {
			mods_error_printk("too many CPUs (%d) for mask bits\n",
					  nr_cpumask_bits);
			LOG_EXT();
			return -EINVAL;
		}

		for (i = 0, maskidx = 0;
		     i < nr_cpumask_bits;
		     i += 32, maskidx++) {
			word = i / BITS_PER_LONG;
			bit = i % BITS_PER_LONG;
			p->node_cpu_mask[maskidx]
				= (maskp[word] >> bit) & 0xFFFFFFFFUL;
		}
	}
	p->node_count = num_possible_nodes();
	p->cpu_count = num_possible_cpus();

	LOG_EXT();
	return OK;
#else
	return -EINVAL;
#endif
}

int esc_mods_device_numa_info(struct file *fp,
			      struct MODS_DEVICE_NUMA_INFO *p)
{
	int retval, i;
	struct MODS_DEVICE_NUMA_INFO_NEW numa_info = { {0} };
	numa_info.pci_device.domain	= 0;
	numa_info.pci_device.bus	= p->pci_device.bus;
	numa_info.pci_device.device	= p->pci_device.device;
	numa_info.pci_device.function	= p->pci_device.function;

	retval = esc_mods_device_numa_info_new(fp, &numa_info);
	if (retval)
		return retval;

	p->node				= numa_info.node;
	p->node_count			= numa_info.node_count;
	for (i = 0; i < MAX_CPU_MASKS; i++)
		p->node_cpu_mask[i]	= numa_info.node_cpu_mask[i];
	p->cpu_count			= numa_info.cpu_count;
	return OK;
}
