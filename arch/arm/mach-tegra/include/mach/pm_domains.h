/*
 * Copyright (c) 2012-2013 NVIDIA Corporation.
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

#ifndef _MACH_TEGRA_PM_DOMAINS_H_
#define _MACH_TEGRA_PM_DOMAINS_H_

#include <linux/pm_domain.h>

struct tegra_pm_domain {
	struct generic_pm_domain gpd;
};

#define to_tegra_pd(_pd) container_of(_pd, struct tegra_pm_domain, gpd);

#ifdef CONFIG_PM_GENERIC_DOMAINS
extern struct tegra_pm_domain tegra_mc_clk;

void tegra_mc_clk_add_device(struct device *dev);
void tegra_mc_clk_add_sd(struct generic_pm_domain *sd);
#else
#define tegra_mc_clk_add_device(dev) do { } while (0)
#define tegra_mc_clk_add_sd(sd) do { } while (0)
#endif /* CONFIG_PM_GENERIC_DOMAINS */

#endif /* _MACH_TEGRA_PM_DOMAINS_H_ */
