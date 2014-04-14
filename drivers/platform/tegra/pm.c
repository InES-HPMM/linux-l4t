/*
 * drivers/platform/tegra/pm.c
 *
 * CPU complex suspend & resume functions for Tegra SoCs
 *
 * Copyright (c) 2009-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <linux/notifier.h>
#include <linux/tegra_pm_domains.h>

static RAW_NOTIFIER_HEAD(tegra_pm_chain_head);

int tegra_register_pm_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&tegra_pm_chain_head, nb);
}
EXPORT_SYMBOL(tegra_register_pm_notifier);

int tegra_unregister_pm_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&tegra_pm_chain_head, nb);
}
EXPORT_SYMBOL(tegra_unregister_pm_notifier);

#ifdef CONFIG_TEGRA_LP0_IN_IDLE
static int tegra_pm_notifier_call_chain(unsigned int val)
{
	int ret = raw_notifier_call_chain(&tegra_pm_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

int tegra_enter_lp0(unsigned long sleep_time)
{
	int err = 0;

	/*
	 * This state is managed by power domains,
	 * hence no voice call expected if
	 * we are entering this state
	 */

	tegra_pm_notifier_call_chain(TEGRA_PM_SUSPEND);

	tegra_rtc_set_trigger(sleep_time);

	tegra_actmon_save();

	tegra_dma_save();

	tegra_smmu_save();

	err = syscore_save();
	if (err) {
		tegra_smmu_restore();
		tegra_dma_restore();
		tegra_rtc_set_trigger(0);
		return err;
	}

	tegra_suspend_dram(TEGRA_SUSPEND_LP0, 0);

	syscore_restore();

	tegra_smmu_restore();

	tegra_dma_restore();

	tegra_actmon_restore();

	tegra_rtc_set_trigger(0);

	tegra_pm_notifier_call_chain(TEGRA_PM_RESUME);

	return 0;
}
#endif
