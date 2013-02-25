/*
 * arch/arm/mach-tegra/pm_domains.c
 *
 * Copyright (c) 2012-2013 NVIDIA Corporation.
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

#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>

#include <mach/pm_domains.h>

#define TEGRA_PD_DEV_CALLBACK(callback, dev)			\
({								\
	int (*__routine)(struct device *__d);			\
	int __ret = 0;						\
								\
	if (dev->type && dev->type->pm)				\
		__routine = dev->type->pm->callback;		\
	else if (dev->class && dev->class->pm)			\
		__routine = dev->class->pm->callback;		\
	else if (dev->bus && dev->bus->pm)			\
		__routine = dev->bus->pm->callback;		\
	else							\
		__routine = NULL;				\
								\
	if (!__routine && dev->driver && dev->driver->pm)	\
		__routine = dev->driver->pm->callback;		\
								\
	if (__routine)						\
		__ret = __routine(dev);				\
	__ret;							\
})

#ifdef CONFIG_PM_SLEEP

static int tegra_pd_suspend_dev(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(suspend, dev);
}

static int tegra_pd_suspend_late(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(suspend_late, dev);
}

static int tegra_pd_resume_early(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(resume_early, dev);
}

static int tegra_pd_resume_dev(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(resume, dev);
}

static int tegra_pd_freeze_dev(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(freeze, dev);
}

static int tegra_pd_freeze_late(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(freeze_late, dev);
}

static int tegra_pd_thaw_early(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(thaw_early, dev);
}

static int tegra_pd_thaw_dev(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(thaw, dev);
}
#else /* !CONFIG_PM_SLEEP */

#define tegra_pd_suspend_dev	NULL
#define tegra_pd_suspend_late	NULL
#define tegra_pd_resume_early	NULL
#define tegra_pd_resume_dev	NULL
#define tegra_pd_freeze_dev	NULL
#define tegra_pd_freeze_late	NULL
#define tegra_pd_thaw_early	NULL
#define tegra_pd_thaw_dev	NULL

#endif /* !CONFIG_PM_SLEEP */

static bool tegra_pd_active_wakeup(struct device *dev)
{
	return false;
}

static int tegra_pd_save_dev(struct device *dev)
{
	return 0;
}

static int tegra_pd_restore_dev(struct device *dev)
{
	return 0;
}

static int tegra_pd_stop_dev(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(runtime_suspend, dev);
}

static int tegra_pd_start_dev(struct device *dev)
{
	return TEGRA_PD_DEV_CALLBACK(runtime_resume, dev);
}

struct gpd_dev_ops tegra_pd_ops = {
	.active_wakeup = tegra_pd_active_wakeup,
	.save_state = tegra_pd_save_dev,
	.restore_state = tegra_pd_restore_dev,
	.stop = tegra_pd_stop_dev,
	.start = tegra_pd_start_dev,
	.suspend = tegra_pd_suspend_dev,
	.suspend_late = tegra_pd_suspend_late,
	.resume_early = tegra_pd_resume_early,
	.resume = tegra_pd_resume_dev,
	.freeze = tegra_pd_freeze_dev,
	.freeze_late = tegra_pd_freeze_late,
	.thaw_early = tegra_pd_thaw_early,
	.thaw = tegra_pd_thaw_dev,
};

static int tegra_mc_clk_power_off(struct generic_pm_domain *genpd)
{
	struct tegra_pm_domain *pd = to_tegra_pd(genpd);

	if (!pd)
		return -EINVAL;

	if (IS_ERR_OR_NULL(pd->clk))
		return 0;

	clk_disable_unprepare(pd->clk);

	return 0;
}

static int tegra_mc_clk_power_on(struct generic_pm_domain *genpd)
{
	struct tegra_pm_domain *pd = to_tegra_pd(genpd);

	if (!pd)
		return -EINVAL;

	if (IS_ERR_OR_NULL(pd->clk))
		return 0;

	clk_prepare_enable(pd->clk);

	return 0;
}

struct tegra_pm_domain tegra_mc_clk = {
	.gpd.name = "tegra_mc_clk",
	.gpd.power_off = tegra_mc_clk_power_off,
	.gpd.power_on = tegra_mc_clk_power_on,
};

struct tegra_pm_domain tegra_mc_chain_a = {
	.gpd.name = "tegra_mc_chain_a",
	.gpd.power_off = tegra_mc_clk_power_off,
	.gpd.power_on = tegra_mc_clk_power_on,
};

struct tegra_pm_domain tegra_mc_chain_b = {
	.gpd.name = "tegra_chain_b",
	.gpd.power_off = tegra_mc_clk_power_off,
	.gpd.power_on = tegra_mc_clk_power_on,
};

static int __init tegra_init_pm_domain(void)
{
	pm_genpd_init(&tegra_mc_clk.gpd, &simple_qos_governor, false);

	tegra_mc_chain_a.clk = clk_get_sys("mc_capa", "mc_capa");
	pm_genpd_init(&tegra_mc_chain_a.gpd, &simple_qos_governor, false);
	tegra_pd_add_sd(&tegra_mc_clk, &tegra_mc_chain_a.gpd);

	tegra_mc_chain_b.clk = clk_get_sys("mc_cbpa", "mc_cbpa");
	pm_genpd_init(&tegra_mc_chain_b.gpd, &simple_qos_governor, false);
	tegra_pd_add_sd(&tegra_mc_clk, &tegra_mc_chain_b.gpd);

	return 0;
}
core_initcall(tegra_init_pm_domain);

void tegra_pd_add_device(struct tegra_pm_domain *pd, struct device *dev)
{
	device_set_wakeup_capable(dev, 1);
	pm_genpd_add_device(&pd->gpd, dev);
	pm_genpd_add_callbacks(dev, &tegra_pd_ops, NULL);
}

void tegra_pd_add_sd(struct tegra_pm_domain *pd, struct generic_pm_domain *sd)
{
	pm_genpd_add_subdomain(&pd->gpd, sd);
}
