/*
 * include/linux/tegra_pm_domains.h
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _INCLUDE_TEGRA_PM_DOMAINS_H_
#define _INCLUDE_TEGRA_PM_DOMAINS_H_

#include <linux/clk.h>
#include <linux/pm_domain.h>

struct tegra_pm_domain {
	struct generic_pm_domain gpd;
	struct clk *clk;
};

#define to_tegra_pd(_pd) container_of(_pd, struct tegra_pm_domain, gpd);

#if defined(CONFIG_TEGRA20_APB_DMA)
int tegra_dma_restore(void);
int tegra_dma_save(void);
#else
static inline int tegra_dma_restore(void)
{ return -ENODEV; }
static inline int tegra_dma_save(void)
{ return -ENODEV; }
#endif

#if defined(CONFIG_TEGRA_ACTMON)
int tegra_actmon_save(void);
int tegra_actmon_restore(void);
#else
static inline int tegra_actmon_save(void)
{ return -ENODEV; }
static inline int tegra_actmon_restore(void)
{ return -ENODEV; }
#endif

#ifdef CONFIG_TEGRA_MC_DOMAINS
void tegra_pd_add_device(struct device *dev);
void tegra_pd_remove_device(struct device *dev);
void tegra_pd_add_sd(struct generic_pm_domain *sd);
void tegra_ape_pd_add_device(struct device *dev);
void tegra_ape_pd_remove_device(struct device *dev);
#else
static inline void tegra_pd_add_device(struct device *dev) { }
static inline void tegra_pd_remove_device(struct device *dev) { }
static inline void tegra_pd_add_sd(struct generic_pm_domain *sd) { }
static inline void tegra_ape_pd_add_device(struct device *dev) { }
static inline void tegra_ape_pd_remove_device(struct device *dev) { }
#endif /* CONFIG_TEGRA_MC_DOMAINS */

#endif /* _INCLUDE_TEGRA_PM_DOMAINS_H_ */
