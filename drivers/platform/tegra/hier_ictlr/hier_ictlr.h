/*
 * Copyright (c) 2013-2014, NVIDIA Corporation.  All rights reserved.
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

#ifndef __HIER_ICTLR_H
#define __HIER_ICTLR_H

#include <linux/types.h>

struct tegra_hier_ictlr {
	u32 irq;
	u32 mselect_timeout_cycles;
	void __iomem *hier_ictlr_base;
	void __iomem *mselect_base;
};

void tegra_hier_ictlr_create_sysfs(struct platform_device *pdev);
void tegra_hier_ictlr_remove_sysfs(struct platform_device *pdev);

void tegra_hier_ictlr_set_mselect_timeout(struct tegra_hier_ictlr *ictlr,
					  u32 timeout);

#endif
