/*
 * Tegra Graphics Host Virtual Memory
 *
 * Copyright (c) 2014-2015, NVIDIA Corporation. All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>

#include "chip_support.h"
#include "nvhost_vm.h"
#include "dev.h"

int nvhost_vm_init_device(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!vm_op().init_device || !pdata->isolate_contexts)
		return 0;

	return vm_op().init_device(pdev);
}

int nvhost_vm_get_id(struct nvhost_vm *vm)
{
	if (!vm_op().get_id)
		return -ENOSYS;

	return vm_op().get_id(vm);
}

int nvhost_vm_map_static(struct platform_device *pdev,
			 void *vaddr, dma_addr_t paddr,
			 size_t size)
{
	/* if static mappings are not supported, exit */
	if (!vm_op().pin_static_buffer)
		return 0;

	return vm_op().pin_static_buffer(pdev, vaddr, paddr, size);
}

static void nvhost_vm_deinit(struct kref *kref)
{
	struct nvhost_vm *vm = container_of(kref, struct nvhost_vm, kref);

	if (vm_op().deinit && vm->enable_hw)
		vm_op().deinit(vm);

	kfree(vm);
	vm = NULL;
}

void nvhost_vm_put(struct nvhost_vm *vm)
{
	kref_put(&vm->kref, nvhost_vm_deinit);
}

void nvhost_vm_get(struct nvhost_vm *vm)
{
	kref_get(&vm->kref);
}

struct nvhost_vm *nvhost_vm_allocate(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_vm *vm;
	int err;

	/* get room to keep vm */
	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return NULL;

	kref_init(&vm->kref);
	vm->pdev = pdev;
	vm->enable_hw = pdata->isolate_contexts;

	if (vm_op().init && vm->enable_hw) {
		err = vm_op().init(vm);
		if (err)
			goto err_init;
	}

	return vm;

err_init:
	kfree(vm);

	return NULL;
}
