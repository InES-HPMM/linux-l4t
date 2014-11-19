/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gfp.h>
#include <linux/pci.h>
#include <linux/tegra_smmu.h>

#include <asm/dma-iommu.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#include "of_tegra-smmu.h"

#define SMMU_AFI_ASID		0x238   /* PCIE */
#define SMMU_SWGRP_ASID_BASE	SMMU_AFI_ASID

size_t tegra_smmu_of_offset(int id)
{
	switch (id) {
	case TEGRA_SWGROUP_DC14:
		return 0x490;
	case TEGRA_SWGROUP_DC12:
		return 0xa88;
	case TEGRA_SWGROUP_AFI...TEGRA_SWGROUP_ISP:
	case TEGRA_SWGROUP_MPE...TEGRA_SWGROUP_PPCS1:
		return (id - TEGRA_SWGROUP_AFI) * sizeof(u32) + SMMU_AFI_ASID;
	case TEGRA_SWGROUP_SDMMC1A...63:
		return (id - TEGRA_SWGROUP_SDMMC1A) * sizeof(u32) + 0xa94;
	};

	BUG();
}

struct dma_iommu_mapping *tegra_smmu_of_get_mapping(struct device *dev,
						    u64 swgids,
						    struct list_head *asprops)
{
	struct smmu_map_prop *tmp;

	list_for_each_entry(tmp, asprops, list) {
		struct dma_iommu_mapping *map;

		if (!(swgids & tmp->swgid_mask))
			continue;

		if ((swgids & tmp->swgid_mask) != swgids)
			dev_info(dev, "mask=%llx doesn't include swgids=%llx\n",
				 tmp->swgid_mask, swgids);

		if (tmp->map)
			return tmp->map;

		map = arm_iommu_create_mapping(&platform_bus_type,
					       (dma_addr_t)tmp->iova_start,
					       (size_t)tmp->iova_size, 0);
		if (IS_ERR(map)) {
			dev_err(dev, "fail to create iommu map prop=%p\n", tmp);
			goto err_out;
		}

		/* FIXME: residual data */
		map->alignment = tmp->alignment;
		map->gap_page = !!tmp->gap_page;
		map->num_pf_page = tmp->num_pf_page;

		tmp->map = map;
		return map;
	}

err_out:
	return NULL;
}

/* FIXME: Add linear map logic as well */
int tegra_smmu_of_register_asprops(struct device *dev,
				   struct list_head *asprops)
{
	int err, count = 0, sum_hweight = 0;
	struct of_phandle_iter iter;
	u64 swgid_mask = 0;
	struct smmu_map_prop *prop, *temp;

	of_property_for_each_phandle_with_args(iter, dev->of_node,
					       "domains", NULL, 2) {
		struct of_phandle_args *ret = &iter.out_args;
		struct device_node *np = ret->np;

		if (ret->args_count < 2) {
			dev_err(dev,
				"domains expects 2 params but %d\n",
				ret->args_count);
			goto free_mem;
		}

		prop = devm_kzalloc(dev, sizeof(*prop), GFP_KERNEL);
		if (!prop) {
			dev_err(dev, "no memory available\n");
			goto free_mem;
		}

		memcpy(&prop->swgid_mask, ret->args, sizeof(u64));

		err = of_property_read_u64(np, "iova-start", &prop->iova_start);
		err |= of_property_read_u64(np, "iova-size", &prop->iova_size);
		err |= of_property_read_u32(np, "num-pf-page",
					    &prop->num_pf_page);
		err |= of_property_read_u32(np, "gap-page", &prop->gap_page);
		if (err) {
			dev_err(dev,
				"invalid address-space-prop %s\n",
				np->name);
			goto free_mem;
		}
		err = of_property_read_u32(np, "alignment", &prop->alignment);
		if (err)
			prop->alignment = 0;

		count++;
		list_add_tail(&prop->list, asprops);

		/*
		 * The final entry in domains property is
		 * domains = <... &as_prop 0xFFFFFFFF 0xFFFFFFFF>;
		 * This entry is similar to SYSTEM_DEFAULT
		 * Skip the bit overlap check for this final entry
		 */
		if (prop->swgid_mask != ~0ULL) {
			swgid_mask |= prop->swgid_mask;
			sum_hweight += hweight64(prop->swgid_mask);
		}
	}

	if (sum_hweight == hweight64(swgid_mask))
		return count;

	/* check bit mask overlap in domains= property */
	dev_warn(dev, "overlapping bitmaps in domains!!!");
free_mem:
	list_for_each_entry_safe(prop, temp, asprops, list)
		devm_kfree(dev, prop);
	return 0;
}

u64 tegra_smmu_of_get_swgids(struct device *dev,
			       const struct of_device_id *matches,
			       struct iommu_linear_map **area)
{
	struct of_phandle_iter iter;
	u64 fixup, swgids = 0;

	if (dev_is_pci(dev)) {
		return SWGIDS_ERROR_CODE;
		swgids = TEGRA_SWGROUP_BIT(AFI);
		goto try_fixup;
	}

	of_property_for_each_phandle_with_args(iter, dev->of_node, "iommus",
					       "#iommu-cells", 0) {
		struct of_phandle_args *ret = &iter.out_args;

		if (!of_match_node(matches, ret->np))
			continue;

		if (ret->args_count != 1) {
			dev_err(dev, "iommus contains %d cells, expected 1\n",
				ret->args_count);
			break;
		}

		swgids |= (1ULL << ret->args[0]);
	}

	swgids = swgids ? swgids : SWGIDS_ERROR_CODE;

try_fixup:
	fixup = tegra_smmu_fixup_swgids(dev, area);

	if (swgids_is_error(fixup))
		return swgids;

	if (swgids_is_error(swgids)) {
		dev_dbg(dev,
			"No iommus property found in DT node, got swgids from fixup(%llx)\n",
			fixup);
		return fixup;
	}

	if (swgids != fixup) {
		dev_notice(dev,	"fixup(%llx) is different from DT(%llx)\n",
			   fixup, swgids);
		return fixup;
	}

	return swgids;
}
